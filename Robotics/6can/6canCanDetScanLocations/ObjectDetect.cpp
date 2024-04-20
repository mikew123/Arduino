// ObjectDetect.cpp
// Function library to detetc objects using a HC-SR04 
// ultrasonic sensor.
// It is optimized for location soda cans in the 6can competition
//
// Mike Williamson 2/28/2023
// MRW 3/5/2023 Fixed -Y wall dont detect 
// MRW 3/11/2023 Used seperate UsPerDeg for left and right sensor servos

#include "ObjectDetect.h"


void ObjectDetect::initSonicSensors(PCA9685 *pca9685_ptr, HC_SR04_BASE *sonicSensors_ptr, int wallAbsX, int wallAbsY)
{
  pca9685_p = pca9685_ptr;
  sonicSensors_p = sonicSensors_ptr;

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
  if(servo == rightSensorServo) usec = servoZeroNomUs + (rightServoUsPerDeg * degree);
  else           usec = servoZeroNomUs + (leftServoUsPerDeg  * degree);
  return(usec);
}


// Limit distance to range of sensor
void ObjectDetect::limitDistance(int &distance)
{
  if(distance > 395) distance = 395;
}


// position ultrasonic range sensors
// sensors mounted on the side right(8) 0 to +180, left(12) 0 to -180
// servos are -90 to +90
//int leftDegLast = 0;
void ObjectDetect::positionRangeSensors(int scanDeg)
{

  // adjust degrees to conform to servo degrees
  scanDeg -= 90;

  // move the range sensors 1 degree at a time to the desired position
  for(int deg = scanDegLast;  
     ((scanDeg<scanDegLast)?(scanDeg<=deg):(scanDeg>=deg));
     (((scanDeg<scanDegLast)?(deg--):(deg++))))
  {
    pca9685_p->setChannelServoPulseDuration(rightSensorServo, servoDeg2Us(rightSensorServo, -deg)); // right
    pca9685_p->setChannelServoPulseDuration(leftSensorServo,  servoDeg2Us(leftSensorServo,  +deg)); // left
    delay(5); // slow down movement, less jerky ???
  }
  scanDegLast = scanDeg;

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
    X = ((scanDist + SENSOR2SERVO_OFFSET) * sin(scanDeg*PI/180)) - SERVO2ROBOTX_OFFSET;
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

  bool objScanPt0 = false;
  
  // TODO: L R wall limits based on relative location of robot reference point
  int wallLimX;
  int wallLimY;
  if(x>=0)  wallLimX = abs(wallLocX - robotLocX) - 25; // 25 is for measure error
  else      wallLimX = abs(-wallLocX - robotLocX) - 25;// looking left
  wallLimY = wallLocY - robotLocY - 25; // 25 is for measure error

  switch(objDetVars[rl].detObjState)
  {
  case detObj_Init0:
    objDetVars[rl].objDetected = false;
    // both sensors at 1st scan point
    objScanPt0 = true;
    objIntersectsAtScanPt0 = false;
    // drop through to init1
  case detObj_Init1:
    objDetVars[rl].nonObjSeqCnt = 0;
    objDetVars[rl].detObjState = detObj_ObjStart;
    // drop through to start
  case detObj_ObjStart: // look for start of object in scan
//    if(scanDist<95 && (scanDeg>=0?x<wallLimX:x>-wallLimX) && (scanDeg<=90?y<wallLimY:-y<wallLimY))
    if((scanDist<100) && (abs(x)<wallLimX) && (abs(y)<wallLimY))
    { // consider data point for object
      objDetVars[rl].objSeqCnt0 = 1;
      objDetVars[rl].objDegStart0 = scanDeg;
      objDetVars[rl].objDegLast0 = scanDeg;
      objDetVars[rl].objDistMin0 = scanDist;
      objDetVars[rl].detObjState = detObj_ObjScan;
      // determine if both sensors det same obj at 1st scan point
      if(objScanPt0 && (objDetVars[rl?0:1].detObjState == detObj_ObjScan))
      { // first scan data point of other sensor has possible object
        // test to see if 1st scan dist of the objects are at the same distance
//        if(abs(objDetVars[rl?0:1].objDistMin0 - objDetVars[rl].objDistMin0) < 10)
//        {
          objIntersectsAtScanPt0 = true;
//        }
      }
    }
    objScanPt0 = false;
    break;

// TODO: improve combining RL combined scans when first or last scan point appears to be nonobject
  case detObj_ObjScan: // process object data points while scanning   
    objDetVars[rl].objDetected = false; // assume no object is detected yet
//    if(scanDist<95 && (scanDeg>=0?x<wallLimX:x>-wallLimX) && (scanDeg>=90?y<wallLimY:-y<wallLimY))
    if((scanDist<100) && (abs(x)<wallLimX) && (abs(y)<wallLimY))
    { // consider for object
      objDetVars[rl].nonObjSeqCnt = 0;
      if(abs(scanDist - objDetVars[rl].objDistMin0) < 10)  
      { // distance to min is small, should be the same object 
        if(scanDist < objDetVars[rl].objDistMin0) 
        { // save minimum object distance point during scan
          objDetVars[rl].objDistMin0 = scanDist;
        }

        // keep degrees of scan, in case it is the last point        
        objDetVars[rl].objDegLast0 = scanDeg;
        objDetVars[rl].objSeqCnt0++;

        // determine if object is detected at the end of scan
        if((scanEnd == true) && (objDetVars[rl].objSeqCnt0 >= 5))
        {      
          objDetVars[rl].objDetected = true;
        }
      }
      // else different 2nd object??
    }
    else 
    { // not an object - assume end of object scan after more than 5 consecutive nonobject
      objDetVars[rl].nonObjSeqCnt++;
      if(objDetVars[rl].nonObjSeqCnt > 5) 
      {
        // test for possible object intersect at Pt0 start of scan
        if((objIntersectsAtScanPt0 == true) && (objDetVars[rl?0:1].objDetected == true)
          && (objDetVars[rl].objSeqCnt0 + objDetVars[rl?0:1].objSeqCnt0 >= 10))
        { // will combine both sensor points
          objDetVars[rl].objDetected = true;  
        }
        else if(objDetVars[rl].objSeqCnt0 >= 5)
        { // Enough consecutive object scan counts for valid detect ???
          objDetVars[rl].objDetected = true;
        }
        else
        { // Attempt to detect an object again
          objDetVars[rl].detObjState = detObj_Init1;
//          objIntersectsAtScanPt0 = false;
        }
      }
      else if((scanEnd == true) && (objDetVars[rl].objSeqCnt0 >= 5))
      { // end of scan and seems like an object has been detected
        objDetVars[rl].objDetected = true;
      }
    }


    if(objDetVars[rl].objDetected == true)
    {
      if((objIntersectsAtScanPt0 == true) && (objDetVars[rl?0:1].objDetected == true))
      { // both sensors have detected objects, now combine
        // determine mid scan degrees, consider fwd and rev scan direction
        if(objDetVars[rl].objDegLast0 > objDetVars[rl?0:1].objDegLast0) 
        {
          objDetVars[rl].objDeg = objDetVars[rl].objDegLast0 -
            ((objDetVars[rl].objDegLast0 - objDetVars[rl?0:1].objDegLast0)/2);
        }
        else 
        {
          objDetVars[rl].objDeg = objDetVars[rl?0:1].objDegLast0 -
            ((objDetVars[rl?0:1].objDegLast0 - objDetVars[rl].objDegLast0)/2);
        }
        // use minimum distance as the detected object distance
        if(objDetVars[rl].objDistMin0 <= objDetVars[rl?0:1].objDistMin0)
        {
          objDetVars[rl].objDist = objDetVars[rl].objDistMin0;
          objDetVars[rl?0:1].objDetected = false; // remove other sensor object det
        }
        else
        {
          objDetVars[rl?0:1].objDist = objDetVars[rl?0:1].objDistMin0;
          objDetVars[rl].objDetected = false; // remove other sensor object det
        }
        // stop looking when object is found
        objDetVars[rl].detObjState = detObj_ObjEnd;
      }
      else if((scanEnd == true) && (objDetVars[rl?0:1].objDetected == true))
      { // combine sensors at end of scan
        // determine mid scan degrees, consider fwd and rev scan direction
        if(objDetVars[rl].objDegStart0 > objDetVars[rl?0:1].objDegStart0) 
        {
          objDetVars[rl].objDeg = objDetVars[rl].objDegStart0 -
            ((objDetVars[rl].objDegStart0 - objDetVars[rl?0:1].objDegStart0)/2);
        }
        else 
        {
          objDetVars[rl].objDeg = objDetVars[rl?0:1].objDegStart0 -
            ((objDetVars[rl?0:1].objDegStart0 - objDetVars[rl].objDegStart0)/2);
        }
        
        // use minimum distance as the detected object distance
        if(objDetVars[rl].objDistMin0 <= objDetVars[rl?0:1].objDistMin0)
        {
          objDetVars[rl].objDist = objDetVars[rl].objDistMin0;
//          objDetVars[rl?0:1].objDetected = false; // remove other sensor object det
        }
        else
        {
//          objDetVars[rl?0:1].objDist = objDetVars[rl?0:1].objDistMin0;
//          objDetVars[rl].objDetected = false; // remove other sensor object det
          objDetVars[rl].objDist = objDetVars[rl?0:1].objDistMin0;
        }
        
        objDetVars[rl?0:1].objDetected = false; // remove other sensor object det

        // stop looking when object is found
        objDetVars[rl].detObjState = detObj_ObjEnd;
        
      }      
      else
      { // not combining both scans after nonobject after object
//          objIntersectsAtScanPt0 = false;
        // determine mid scan degrees, consider fwd and rev scan direction
        if(objDetVars[rl].objDegLast0 > objDetVars[rl].objDegStart0) 
        {
          objDetVars[rl].objDeg = objDetVars[rl].objDegStart0 + 
                  ((objDetVars[rl].objDegLast0 - objDetVars[rl].objDegStart0)/2);
        }
        else 
        {
          objDetVars[rl].objDeg = objDetVars[rl].objDegStart0 - 
                  ((objDetVars[rl].objDegStart0 - objDetVars[rl].objDegLast0)/2);
        }
        // use minimum distance as the detected object distance
        objDetVars[rl].objDist = objDetVars[rl].objDistMin0;
        // stop looking when object is found
        objDetVars[rl].detObjState = detObj_ObjEnd;
      }
    }
    break;

  case detObj_ObjEnd: // stay here after an object detected
      //TODO:  multiple objects detected per sensor?
      break;
  }

}


// returns objectDetected status and location in polar an XY
bool ObjectDetect::getObject(int &moveDeg, int &moveDist, int &xCm, int &yCm)
{
  bool objDetected;
  // if both sensors detected use minimum distance detected
  if(((objDetVars[rightSensor].objDetected == true) && (objDetVars[leftSensor].objDetected != true)) 
      || (((objDetVars[rightSensor].objDetected == true) && (objDetVars[leftSensor].objDetected == true))
            && ((objDetVars[rightSensor].objDist <= objDetVars[leftSensor].objDist))))
  {
Serial.println("objDetVars[rightSensor].objDetected == true");
    moveDeg  = objDetVars[rightSensor].objDeg;
    moveDist = objDetVars[rightSensor].objDist;
    objDetected = true;
  }
  else if(((objDetVars[leftSensor].objDetected == true) && (objDetVars[rightSensor].objDetected != true)) 
            || (((objDetVars[leftSensor].objDetected == true) && (objDetVars[rightSensor].objDetected == true))
                  && ((objDetVars[leftSensor].objDist <= objDetVars[rightSensor].objDist))))
  {
Serial.println("objDetVars[leftSensor].objDetected == true");
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

  for(scanDeg = (scanMax2Min ? scanDegRev : scanDegFwd); 
                (scanMax2Min ? (scanDeg>=scanDegFwd) : (scanDeg<=scanDegRev)); 
      scanDeg += (scanMax2Min ? -scanDegInc : scanDegInc))
  {
    positionRangeSensors(scanDeg);
    sonicSensors_p->startAsync(0);
delay(25); // makes measurement more stable???
    while (!sonicSensors_p->isFinished()) {delay(25); Serial.print(".");}


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


// print functions for object detect debug
void ObjectDetect::printRangeSensors(int degR, int distR, int xR, int yR, int degL, int distL, int xL, int yL)
{
  Serial.print(" Right: ");
  Serial.print(degR); Serial.print(", ");
  Serial.print(distR); Serial.print(", ");
  Serial.print(xR); Serial.print(", ");
  Serial.print(yR); Serial.print("");
  Serial.print(" Left: ");
  Serial.print(degL); Serial.print(", ");
  Serial.print(distL); Serial.print(", ");
  Serial.print(xL); Serial.print(", ");
  Serial.print(yL); Serial.print("\n");
}


void ObjectDetect::printWhileScanning(int degR, int distR, int xR, int yR, int degL, int distL, int xL, int yL)
{
    xyCm2CharMatrix(xR, yR, 'R');       
    xyCm2CharMatrix(xL, yL, 'L');    

Serial.print(objIntersectsAtScanPt0);
Serial.print("R "); Serial.print(objDetVars[rightSensor].detObjState); 
Serial.print(','); Serial.print(objDetVars[rightSensor].objDetected); Serial.print("; ");   
Serial.print("L "); Serial.print(objDetVars[leftSensor].detObjState); 
Serial.print(','); Serial.print(objDetVars[leftSensor].objDetected); Serial.print(": ");   
    printRangeSensors(degR, distR, xR, yR, degL, distL, xL, yL);
}

