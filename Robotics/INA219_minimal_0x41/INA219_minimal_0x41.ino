//
//    FILE: INA219_minimal.ino
//  AUTHOR: Rob Tillaart
// PURPOSE: demo
//    DATE: 2022-09-06
//     URL: https://github.com/RobTillaart/INA219


#include "INA219.h"
#include "Wire.h"


TwoWire *wire_ptr = &Wire;

INA219 INA(0x41, wire_ptr);

void setup()
{
  Serial.begin(115200);
  
  delay(1000);

  Serial.println(__FILE__);
  Serial.print("INA219_LIB_VERSION: ");
  Serial.println(INA219_LIB_VERSION);

  Wire.begin(); 
  
  if (!INA.begin())Serial.println("could not connect. Fix and Reboot");
  else Serial.println("INA connected OK");

  INA.setMaxCurrentShunt(2.5, 0.002);

  delay(5000);
}


void loop()
{
  //  these two can be read without further configuration.
  Serial.print(INA.getBusVoltage(), 3);
  Serial.print("\t");
  Serial.print(INA.getShuntVoltage_mV(), 3);
  Serial.print("\t");
  Serial.print(INA.getCurrent_mA(), 3);
  Serial.print("\n");
  delay(200);
}


// -- END OF FILE --
