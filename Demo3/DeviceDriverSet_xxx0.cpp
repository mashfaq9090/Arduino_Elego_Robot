/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2020-06-12 16:36:20
 * @LastEditors: Changhua
 * @Description: conqueror robot tank
 * @FilePath: 
 */
#include "DeviceDriverSet_xxx0.h"
#include <IRremote.hpp>  // Note: .hpp not .h in v4.x

void DeviceDriverSet_IR::DeviceDriverSet_IR_Init(void)
{
  IrReceiver.begin(_PIN, ENABLE_LED_FEEDBACK);
}

bool DeviceDriverSet_IR::DeviceDriverSet_IR_Get(uint8_t *IR_Get)
{
  if (IrReceiver.decode())
  {
    IR_PreMillis = millis();
    uint32_t value = IrReceiver.decodedIRData.decodedRawData;
    // Serial.print("RAW: 0x");
    // Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX);
    
    switch (value)
    {
      case a_upper:      *IR_Get = 1; break;
      case a_lower:      *IR_Get = 2; break;
      case a_Left:       *IR_Get = 3; break;
      case a_right:      *IR_Get = 4; break;
      case a_ok:         *IR_Get = 5; break;
      case a_1:          *IR_Get = 6; break;
      case a_2:          *IR_Get = 7; break;
      case a_3:          *IR_Get = 8; break;
      case a_4:          *IR_Get = 9; break;
      case a_5:          *IR_Get = 10; break;
      case a_6:          *IR_Get = 11; break;
      case a_7:          *IR_Get = 12; break;
      case a_8:          *IR_Get = 13; break;
      case a_9:          *IR_Get = 14; break;
      case a_0:          *IR_Get = 15; break;
      case a_star:       *IR_Get = 16; break;
      case a_pound:      *IR_Get = 17; break;
      default:
        IrReceiver.resume();
        return false;
    }
    IrReceiver.resume();
    return true;
  }
  return false;
}