/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2020-06-12 14:45:27
 * @LastEditors: Changhua
 * @Description: conqueror robot tank
 * @FilePath: 
 */
#pragma once
#ifndef _DeviceDriverSet_xxx0_H_
#define _DeviceDriverSet_xxx0_H_

#define _Test_DeviceDriverSet 0
#include <arduino.h>
/*IR*/

class DeviceDriverSet_IR
{
public:
  void DeviceDriverSet_IR_Init(void);
  bool DeviceDriverSet_IR_Get(uint8_t *IR_Get /*out*/);
  void DeviceDriverSet_IR_Test(void);

public:
  unsigned long IR_PreMillis;

private:
#define _PIN 9

/*A:4294967295*/
#define a_upper 0xB946FF00
#define a_lower 0xEA15FF00
#define a_Left 0xBB44FF00
#define a_right 0xBC43FF00
#define a_ok 0xBF40FF00
#define a_1 0xE916FF00
#define a_2 0xE619FF00
#define a_3 0xF20DFF00
#define a_4 0xF30CFF00
#define a_5 0xE718FF00
#define a_6 0xA15EFF00
#define a_7 0xF708FF00
#define a_8 0xE31CFF00
#define a_9 0xA55AFF00
#define a_star 0xBD42FF00
#define a_0 0xAD52FF00
#define a_pound 0xB54AFF00
/*B:*/
#define b_upper 0xB946FF00
#define b_lower 0xEA15FF00
#define b_Left 0xBB44FF00
#define b_right 0xBC43FF00
#define b_ok 0xBF40FF00
#define b_1 0xE916FF00
#define b_2 0xE619FF00
#define b_3 0xF20DFF00
#define b_4 0xF30CFF00
#define b_5 0xE718FF00
#define b_6 0xA15EFF00
#define b_7 0xF708FF00
#define b_8 0xE31CFF00
#define b_9 0xA55AFF00
#define b_star 0xBD42FF00
#define b_0 0xAD52FF00
#define b_pound 0xB54AFF00
};

#endif
