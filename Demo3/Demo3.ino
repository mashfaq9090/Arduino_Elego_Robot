/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2020-06-28 14:55:26
 * @LastEditors: Changhua
 * @Description: conqueror robot tank
 * @FilePath: 
 */
#include <avr/wdt.h>
#include "DeviceDriverSet_xxx0.h"
#include "ApplicationFunctionSet_xxx0.h"

void setup()
{
  // put your setup code here, to run once:
  Application_FunctionSet.ApplicationFunctionSet_Init();
}

void loop(){
  // refer to devicedriverset_xxx to see maping to numbers
  uint8_t output = &Application_FunctionSet.ApplicationFunctionSet_IR();
  if (output==1){
    // do action
  }
  if (output ==2){
    //do action
  }
  // ...
}
