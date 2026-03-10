/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2020-06-28 14:10:45
 * @LastEditors: Changhua
 * @Description: conqueror robot tank
 * @FilePath: 
 */
#include <hardwareSerial.h>
#include <stdio.h>
#include <string.h>
#include "ApplicationFunctionSet_xxx0.h"
#include "DeviceDriverSet_xxx0.h"

#include "ArduinoJson-v6.11.1.h" //ArduinoJson

#define _is_print 1
#define _Test_print 0

DeviceDriverSet_IR AppIR;
/*f(x) int */
static boolean
function_xxx(long x, long s, long e) //f(x)
{
  if (s <= x && x <= e)
    return true;
  else
    return false;
}


/*模式控制序列*/
enum ConquerorCarFunctionalModel
{
  Standby_mode,           /*空闲模式*/
  TraceBased_mode,        /*循迹模式*/
  ObstacleAvoidance_mode, /*避障模式*/
  Follow_mode,            /*跟随模式*/
  Rocker_mode,            /*摇杆模式*/
};

struct Application_xxx
{
  ConquerorCarFunctionalModel Functional_Mode;
};
ApplicationFunctionSet Application_FunctionSet; // add this as a global
Application_xxx Application_ConquerorCarxxx0;

void ApplicationFunctionSet::ApplicationFunctionSet_Init(void)
{
  bool res_error = true;
  Serial.begin(9600);
  AppIR.DeviceDriverSet_IR_Init();
}

void ApplicationFunctionSet::ApplicationFunctionSet_IR(void)
{
  uint8_t IR_button;
  if (AppIR.DeviceDriverSet_IR_Get(&IR_button))
  {
    switch (IR_button)
    {
      case 1: Serial.println("Forward"); break;
      case 2: Serial.println("Backward"); break;
      case 3: Serial.println("Left"); break;
      case 4: Serial.println("Right"); break;
      case 5: Serial.println("OK"); break;
      case 6: Serial.println("1"); break;
      case 7: Serial.println("2"); break;
      case 8: Serial.println("3"); break;
      case 9: Serial.println("4"); break;
      case 10: Serial.println("5"); break;
      case 11: Serial.println("6"); break;
      case 12: Serial.println("7"); break;
      case 13: Serial.println("8"); break;
      case 14: Serial.println("9"); break;
      case 15: Serial.println("0"); break;
      case 16: Serial.println("*"); break;
      case 17: Serial.println("#"); break;
      default: Serial.println("Unknown"); break;
    }
  }
}
