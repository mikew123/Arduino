// ObjectDetect.cpp
// Function library to detetc objects using a HC-SR04 
// ultrasonic sensor.
// It is optimized for location soda cans in the 6can competition
//
// Mike Williamson 2/28/2023
// MRW 3/5/2023 Fixed -Y wall dont detect 
// MRW 3/11/2023 Used seperate UsPerDeg for left and right sensor servos
// MRW 3/13/2023 detect works OK (1 can) with new state machine and averaged degrees
// MRW 3/13/2023 Horrible detection with 4 cans
// MRW 3/16/2023 Use LIDAR for final can approach before grabbing


#include "ObjectDetect.h"


void ObjectDetect::initObjectDetect(bool serial_OK, PCA9685 *pca9685_ptr, HC_SR04_BASE *sonicSensors_ptr
                                    , NanoBleSensors *bleSensors_ptr, int wallAbsX, int wallAbsY)
{
  serialOK = serial_OK;
  pca9685_p = pca9685_ptr;
  sonicSensors_p = sonicSensors_ptr;
  bleSensors_p = bleSensors_ptr;

  // absolute wall +- offset from center X=0
  wallLocX = wallAbsX;
  wallLocY = wallAbsY;

  sonicSensors_p->beginAsync(); 

  // init sensor servos to point straight and level
  pca9685_p->setChannelServoPulseDuration(8, 1500); // right
  pca9685_p->setChannelServoPulseDuration(12, 1500);// left
}

// returns servo usec from degrees
// the servo input is used for future optimization for each servo
// 0=right, 1=left sensor servo
int ObjectDetect::servoDeg2Us(int servo, int degree)
{
  int usec;
  if(servo == rightSensorServo) usec = rightServoZeroNomUs + (rightServoUsPerDeg * degree);
  else                          usec = leftServoZeroNomUs  + (leftServoUsPerDeg  * degree);
  return(usec);
}


// Limit distance to range of sensor
void ObjectDetect::limitDistance(int &distance)
{
  if(distance > 395) distance = 395;
}


// position ultrasonic range sensors
// sensors mounted on the side right(8) 0 to +180, left(12) 0 to -180
// servos are -90 to +90 but can be pushed a bit past that
// When both servos need to be different use scanDegL for left sensor
void ObjectDetect::positionRangeSensors(int scanDeg, int scanDegL)
{

  if((scanDegL > 10) || (scanDegL < -200)) // not invalid Left scan value
  { // Left and Right sensors smoothly scan to same +- degree
    // adjust degrees to conform to servo degrees
    scanDeg -= 90;

    // move the range sensors 1 degree at a time to the desired position
    for(int deg = scanDegLast;  
      ((scanDeg<scanDegLast)?(scanDeg<=deg):(scanDeg>=deg));
      (((scanDeg<scanDegLast)?(deg--):(deg++))))
    {
      pca9685_p->setChannelServoPulseDuration(rightSensorServo, servoDeg2Us(rightSensorServo, -deg)); // right
      delay(1); // time for last cmd to finish?
      pca9685_p->setChannelServoPulseDuration(leftSensorServo,  servoDeg2Us(leftSensorServo,  +deg)); // left
      delay(5); // slow down movement, less jerky ???
    }
    scanDegLast = scanDeg;
  }
  else
  { // move each sensor to their different degrees, not smooth
    scanDeg  -= 90;
    scanDegL += 90;
    pca9685_p->setChannelServoPulseDuration(rightSensorServo, servoDeg2Us(rightSensorServo, -scanDeg)); // right
    delay(1); // time for last cmd to finish?
    pca9685_p->setChannelServoPulseDuration(leftSensorServo,  servoDeg2Us(leftSensorServo,  -scanDegL)); // left
    delay(1); // time for last cmd to finish?
  }
}


void ObjectDetect::rangePolar2Xy(int deg, int dCm, int &xCm, int &yCm)
{
  yCm = dCm * cos(deg*PI/180);
  xCm = dCm * sin(deg*PI/180);
}

// translate scanned degrees,distance to robot degrees,distance
// The robot reference point is the front center of the robot chassis
// The scanned reference point is at the sensor
// Physical fixed offsets:
void ObjectDetect::scan2Rob(bool sensorL, int scanDeg, int scanDist, 
              int &robDeg, int &robDist, int &robX, int &robY)
{
  double X;
  double Y;
  double D;
  if(sensorL) // Scan using left sensor  (+5 to -185)
    X = ((scanDist + SENSOR2SERVO_OFFSET) * sin(scanDeg*PI/180)) - SERVO2ROBOTX_OFFSET -1.5;
  else // scan using right sensor (-5 to +185)
    X = ((scanDist + SENSOR2SERVO_OFFSET) * sin(scanDeg*PI/180)) + SERVO2ROBOTX_OFFSET;

  Y = ((scanDist + SENSOR2SERVO_OFFSET) * cos(scanDeg*PI/180)) - SERVO2ROBOTY_OFFSET;
  
  D = sqrt(X*X + Y*Y);

  if((scanDeg>45) && (scanDeg<120))
    robDeg = acos(Y/D)*180/PI;
  else if((scanDeg<-45) && (scanDeg>-120))
    robDeg = -1*(acos(Y/D)*180/PI);
  else
    // TODO: calc >120 properly (sin valid for -90 to +90)
    robDeg = asin(X/D)*180/PI;

  robDist = D;  
  robX = X;
  robY = Y;
}


// Send coordinates in sequence, 
// reliable scan dist = 1M and walls at a out 1M 
// the object will be sensed for about 30 degrees of scan
// but not between 45 and 120 degrees, the wall echo overpowers
// determine center of scan degrees
// the object scan starts at first distance < max (about 100 cm)
// and ends at next distance < max (about 100 cm)
// the object scan must have at least 5 points
// After the first object is detected no other objects are considered
// If an object is sensed on both sensors at start or end of scan
// the scans intersect so extend the scan detection 
// set global detected object degree and distance (min dist)
// 2nd arg selects 0=right, 1=left
// 1st arg used to determine last scan point - see if 2 sensor scans intersect at end
void ObjectDetect::detectObjects(bool scanEnd, int rl, int scanDeg, int scanDist, int x, int y)
{

//  bool objScanPt0 = false;
  bool possibleObj;
  objDetVars[rl].objScanEnd = scanEnd;

  // TODO: L R wall limits based on relative location of robot reference point
  int wallLimX;
  int wallLimY;
  if(x>=0)  wallLimX = abs(wallLocX - robotLocX) - sensorError; // looking right
  else      wallLimX = abs(-wallLocX - robotLocX) - sensorError;// looking left
  // TODO: Do I need other start non-goal Y wall?
  wallLimY = wallLocY - robotLocY - sensorError;

  switch(objDetVars[rl].detObjState)
  {
  ////////////////////////////////////////////////////////////////////////
  case detObj_Init0:
    // both sensors at 1st scan point
    objDetVars[rl].objScanStart = true;
    // drop through to init1


  ////////////////////////////////////////////////////////////////////////
  case detObj_Init1:
    objDetVars[rl].objDetected = false;
    objDetVars[rl].objDistMin = 0;    
    objDetVars[rl].nonObjSeqCnt = 0;
    objDetVars[rl].objSeqCnt = 0;
    objDetVars[rl].detObjState = detObj_ObjScan;
    objDetVars[rl].lastScanDist = scanDist;
    // drop through to Scan

  /////////////////////////////////////////////////////////////////////////////
  case detObj_ObjScan: // process object data points while scanning   
    possibleObj = (scanDist<100) && (abs(x)<wallLimX) && (abs(y)<wallLimY);
    // add code to detect new object and only detect first
//    possibleObj &= abs(objDetVars[rl].objDistMin - scanDist) < 20;
//    possibleObj &= abs(objDetVars[rl].lastScanDist - scanDist) < 20;
//    objDetVars[rl].lastScanDist = scanDist;

    if(possibleObj == true)
    {
      // save possible object deg and dist and inc count
      if(objDetVars[rl].objSeqCnt == 0) {
        objDetVars[rl].objDegStart = scanDeg;
        objDetVars[rl].objDegAcc = 0;
        objDetVars[rl].objDistMin = scanDist;
      }

      objDetVars[rl].objDegAcc += scanDeg; // for average
      objDetVars[rl].objDegLast = scanDeg;
      // save minimum distance  
      objDetVars[rl].objDistMin = min(objDetVars[rl].objDistMin, scanDist);

      objDetVars[rl].objSeqCnt++;
      objDetVars[rl].nonObjSeqCnt = 0;
    }
    else
    { // probably not an object
      objDetVars[rl].nonObjSeqCnt++;
    }

    // *********  object scan at END, only detects if both sensors detect object *************
    // NOTE: this will not happen after modifying scan to always go 0 to 90 and not 90 to 0
    if(    objDetVars[rl].objScanEnd     && (objDetVars[rl].objSeqCnt > 0)
        && objDetVars[rl?0:1].objScanEnd && (objDetVars[rl?0:1].objSeqCnt > 0)
        && (abs(objDetVars[rl].objDegStart - objDetVars[rl?0:1].objDegStart) < 75))
    { // last scan point, combine RL data
      // TODO: test LR dist min for larger offset maybe indicating 2 cans
      // test LR max degree range to avoid thinking 2 cans are 1 can
      // Combine R and L object counts
      int objCnt = objDetVars[rl].objSeqCnt + objDetVars[rl?0:1].objSeqCnt;
      if(objCnt >= 5)
      { // calc object location using both RL
        // Calc average degrees while object was detected
        objDetVars[rl].objDeg = objDetVars[rl].objDegAcc/objCnt;
        // use minimum distance as the detected object distance
        objDetVars[rl].objDist = min(objDetVars[rl].objDistMin, objDetVars[rl?0:1].objDistMin);
        
        objDetVars[rl].objDetected     = true;
        objDetVars[rl?0:1].objDetected = false; // remove other sensor object det
        objDetVars[rl].detObjState     = detObj_ObjEnd;

        break; // State detObj_ObjScan    
      }
      else
      { // not enough counts to detect object at the end of the scan
        objDetVars[rl].objDetected = false;
        objDetVars[rl].detObjState = detObj_ObjEnd;

        break; // State detObj_ObjScan
      }
    } // end if end and both counts >0

    else if((objDetVars[rl].nonObjSeqCnt > 5) || objDetVars[rl].objScanEnd)
    { // detected end of object while scanning

      if(objDetVars[rl].objScanStart && (objDetVars[rl].objSeqCnt == 0))
      { // terminate start status
        objDetVars[rl].objScanStart     = false;
        objDetVars[rl?0:1].objScanStart = false;
      }
      
      // ************** object scan from START *****************
      else if(objDetVars[rl].objScanStart && objDetVars[rl?0:1].objScanStart 
          && (objDetVars[rl?0:1].objSeqCnt > 0) && (objDetVars[rl?0:1].nonObjSeqCnt > 5)
          && (abs(objDetVars[rl].objDegLast - objDetVars[rl?0:1].objDegLast) < 75))
      { // attempt to combine object data from both sensors from start of scan
        objDetVars[rl].objScanStart     = false;
        objDetVars[rl?0:1].objScanStart = false;
        // TODO: Check DistMin RL offset is not too far for single can
        int objCnt = objDetVars[rl].objSeqCnt + objDetVars[rl?0:1].objSeqCnt;
        if(objCnt >= 5)
        { // calc object location using both RL

          if(0) {
            // determine mid scan degrees, consider fwd and rev scan direction
            if(objDetVars[rl].objDegLast > objDetVars[rl?0:1].objDegLast) 
            {
              objDetVars[rl].objDeg = objDetVars[rl].objDegLast -
                ((objDetVars[rl].objDegLast - objDetVars[rl?0:1].objDegLast)/2);
            }
            else 
            {
              objDetVars[rl].objDeg = objDetVars[rl?0:1].objDegLast -
                ((objDetVars[rl?0:1].objDegLast - objDetVars[rl].objDegLast)/2);
            }
          } else {
            // Calc average degrees while object was detected
            objDetVars[rl].objDeg = objDetVars[rl].objDegAcc/objCnt;
          }

          // use minimum distance of either RL as the detected object distance
          objDetVars[rl].objDist = min(objDetVars[rl].objDistMin, objDetVars[rl?0:1].objDistMin);
            
          objDetVars[rl].objDetected = true;
          objDetVars[rl?0:1].objDetected = false; // remove other sensor object det ???
          objDetVars[rl].detObjState = detObj_ObjEnd; // stop looking when object is found

          break; // State detObj_ObjScan
        }
      } // end if objScanStart == true

      // ************ object scan while scanning not at start or single sensor at end ******************
      else if(objDetVars[rl].objSeqCnt >= 5)
      { // valid obj, use only R or L scan data
        if(0) {
        // determine mid scan degrees, consider fwd and rev scan direction
        if(objDetVars[rl].objDegLast > objDetVars[rl].objDegStart) 
          objDetVars[rl].objDeg = objDetVars[rl].objDegStart
            + ((objDetVars[rl].objDegLast - objDetVars[rl].objDegStart)/2);
        else 
          objDetVars[rl].objDeg = objDetVars[rl].objDegStart
            - ((objDetVars[rl].objDegStart - objDetVars[rl].objDegLast)/2);
        } else {
          // Calc average degrees while object was detected
          objDetVars[rl].objDeg = objDetVars[rl].objDegAcc/objDetVars[rl].objSeqCnt;
        }

        // use minimum distance as the detected object distance
        objDetVars[rl].objDist = objDetVars[rl].objDistMin;
        objDetVars[rl].objDetected = true;
        // stop looking when object is found
        objDetVars[rl].detObjState = detObj_ObjEnd;

        break; // State detObj_ObjScanbreak; // State detObj_ObjScan        
      }
      else if((objDetVars[rl?0:1].objScanStart == false))
      { // not valid object, continue scanning for new object
        // reset object count
        objDetVars[rl].detObjState = detObj_Init1;

        break; // state detObj_ObjScan
      }
      else
      { // hold obj counts for other RL start combine

        break; // state detObj_ObjScan
      }
    } // end else if nonObjSeqCnt > 5 OR end of scan range
    else
    { // possible still object in scan, continue scanning

      break; // state detObj_ObjScan
    }

    break; //  state detObj_ObjScan

  /////////////////////////////////////////////////////////////////////////
  case detObj_ObjEnd: // stay here after an object detected
      //TODO:  multiple objects detected per sensor?
      break;
  }

}


// returns objectDetected data and location in polar an XY
bool ObjectDetect::getObject(int &moveDeg, int &moveDist, int &xCm, int &yCm)
{
  bool objDetected;
  // if both sensors detected use minimum degrees // distance detected
  if(((objDetVars[rightSensor].objDetected == true) && (objDetVars[leftSensor].objDetected != true)) 
      || (((objDetVars[rightSensor].objDetected == true) && (objDetVars[leftSensor].objDetected == true))
//            && ((objDetVars[rightSensor].objDist <= objDetVars[leftSensor].objDist))))
            && (abs(objDetVars[rightSensor].objDist) <= abs(objDetVars[leftSensor].objDist))))
  {

if(serialOK) {    
Serial.println("objDetVars[rightSensor].objDetected == true");
}

    moveDeg  = objDetVars[rightSensor].objDeg;
    moveDist = objDetVars[rightSensor].objDist;
    objDetected = true;
  }
  else if(((objDetVars[leftSensor].objDetected == true) && (objDetVars[rightSensor].objDetected != true)) 
            || (((objDetVars[leftSensor].objDetected == true) && (objDetVars[rightSensor].objDetected == true))
//                  && ((objDetVars[leftSensor].objDist <= objDetVars[rightSensor].objDist))))
                  && (abs(objDetVars[leftSensor].objDist) <= abs(objDetVars[rightSensor].objDist))))
  {

if(serialOK) {
Serial.println("objDetVars[leftSensor].objDetected == true");
}

    moveDeg  = objDetVars[leftSensor].objDeg;
    moveDist = objDetVars[leftSensor].objDist;
    objDetected = true;    
  }
  else
  {
    moveDeg  = 0;
    moveDist = 0;
    objDetected = false;
  }  
  
  // convert deg dist to XY
  rangePolar2Xy(moveDeg, moveDist, xCm, yCm);

  return(objDetected);  
}

// TODO: modify scanRange to return object detected
// TODO: create overload with no print info

// scan using ultrasonic sensors
// exit when scan is complete
void ObjectDetect::scanRange(int robotX, int robotY)
{
  int robXR, robYR;
  int robXL, robYL;
  int scanDistR;
  int scanDistL;
  int robDegR, robDistR;
  int robDegL, robDistL;

  initCharMatrix();

  // save robot location for detection algorithm during scan
  robotLocX = robotX;
  robotLocY = robotY;

  // init object detect state for 1st state
  objDetVars[rightSensor].detObjState = detObj_Init0;
  objDetVars[leftSensor].detObjState = detObj_Init0;

  int scanCntDwn = abs(scanDegRev - scanDegFwd)/scanDegInc;

  // always start from fwd position, this may help keep focus on can it moved close to
  scanMax2Min = false;
  
  for(scanDeg = (scanMax2Min ? scanDegRev : scanDegFwd); 
                (scanMax2Min ? (scanDeg>=scanDegFwd) : (scanDeg<=scanDegRev)); 
      scanDeg += (scanMax2Min ? -scanDegInc : scanDegInc))
  {
    positionRangeSensors(scanDeg);
    sonicSensors_p->startAsync(0);
//delay(25); // makes measurement more stable???
    while (!sonicSensors_p->isFinished()) {delay(25); if(serialOK) Serial.print(".");}


    scanDistR = sonicSensors_p->getDist_cm(rightSensor); // right sensor
    scanDistL = sonicSensors_p->getDist_cm(leftSensor); // left sensor

    limitDistance(scanDistR);
    limitDistance(scanDistL);

    // convert object detect offset from scanner reference to robot reference
    scan2Rob(rightSensor, +scanDeg, scanDistR, robDegR, robDistR, robXR, robYR);
    scan2Rob(leftSensor,  -scanDeg, scanDistL, robDegL, robDistL, robXL, robYL);

    // object detection algoritm, process left and right sensors
    detectObjects(scanCntDwn==0, rightSensor, robDegR, robDistR, robXR, robYR);
    detectObjects(scanCntDwn==0, leftSensor,  robDegL, robDistL, robXL, robYL);
    scanCntDwn--;

    printWhileScanning(robDegR, robDistR, robXR, robYR, robDegL, robDistL, robXL, robYL);

  }
  scanMax2Min = ! scanMax2Min;

  // add closest object to char print matrix
  int objDeg, objDist, objX, objY;
  if(getObject(objDeg, objDist, objX, objY)) xyCm2CharMatrix(objX, objY, '@');
  printCharMatrix();
}


/*********************************************************************************************************
Print debug functions
***********************************************************************************************************/


// initialize to space chars (blank)
void ObjectDetect::initCharMatrix()
{
  for(int x=0; x<41; x++)
  {
    for(int y=0; y<20; y++)
    {
      charMatrix[x][y] = '-';
    }
  }
}

void ObjectDetect::xyCm2CharMatrix(int xCm, int yCm, char marker)
{
  //x -400 to +400 cm to 0 to 40 matrix table  
  //each x table entry is 20 cm, x -10 to 10 is table entry x=20 (0)
  int x = ((xCm + 400)/20.0 + 0.5);
  int y = (yCm/20.0);
  charMatrix[x][y] = marker;
}

void ObjectDetect::printCharMatrix()
{

  if(serialOK) {
  for(int y=19; y>=0; y--)
  {
    for(int x=0; x<41; x++)
    {
      Serial.print(charMatrix[x][y]);
      Serial.print(' ');
    }
    Serial.print('\n');
  }
  // sensor/robot position
  Serial.println("                                        ^"); // sensor position
  }
  
}


// print functions for object detect debug
void ObjectDetect::printRangeSensors(int degR, int distR, int xR, int yR, int degL, int distL, int xL, int yL)
{

  if(serialOK) {
  Serial.print(" Left: ");
  Serial.print(degL); Serial.print(", ");
  Serial.print(distL); Serial.print(", ");
  Serial.print(xL); Serial.print(", ");
  Serial.print(yL); Serial.print("");
  Serial.print(" Right: ");
  Serial.print(degR); Serial.print(", ");
  Serial.print(distR); Serial.print(", ");
  Serial.print(xR); Serial.print(", ");
  Serial.print(yR); Serial.print("\n");
  }

}


void ObjectDetect::printWhileScanning(int degR, int distR, int xR, int yR, int degL, int distL, int xL, int yL)
{
    xyCm2CharMatrix(xR, yR, 'R');       
    xyCm2CharMatrix(xL, yL, 'L');
//int proximity;
//bleSensors_p->readAPDSSensor(proximity);

if(serialOK) {
//Serial.print("LIDAR "); Serial.print(proximity); Serial.print(" - ");

Serial.print("L "); Serial.print(objDetVars[leftSensor].detObjState); 
Serial.print(',');  Serial.print(objDetVars[leftSensor].objScanStart);
Serial.print(',');  Serial.print(objDetVars[leftSensor].objSeqCnt);
Serial.print(',');  Serial.print(objDetVars[leftSensor].objDistMin);
Serial.print(',');  Serial.print(objDetVars[leftSensor].objDetected); 
Serial.print("; ");   

Serial.print("R "); Serial.print(objDetVars[rightSensor].detObjState); 
Serial.print(',');  Serial.print(objDetVars[rightSensor].objScanStart);
Serial.print(',');  Serial.print(objDetVars[rightSensor].objSeqCnt);
Serial.print(',');  Serial.print(objDetVars[rightSensor].objDistMin);
Serial.print(',');  Serial.print(objDetVars[rightSensor].objDetected); 
Serial.print(": ");   
}

  printRangeSensors(degR, distR, xR, yR, degL, distL, xL, yL);

}

