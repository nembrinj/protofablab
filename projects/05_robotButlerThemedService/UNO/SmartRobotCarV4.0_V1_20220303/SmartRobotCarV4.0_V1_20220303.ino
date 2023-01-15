/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2023-01-10
 * @LastEditors: COSTA Kimmy, MEMBREZ Cédric
 * @Description: Smart Robot Car V4.0
 * @FilePath: 
 */
#include <avr/wdt.h>
#include "ApplicationFunctionSet_xxx0.h"

void setup()
{
  // ELEGOO CODE
  Application_FunctionSet.ApplicationFunctionSet_Init();
  wdt_enable(WDTO_2S);
}

void loop()
{
  wdt_reset();

  Application_FunctionSet.ApplicationFunctionSet_SensorDataUpdate();
  Application_FunctionSet.ApplicationFunctionSet_KeyCommand();
  Application_FunctionSet.ApplicationFunctionSet_RGB();
  //Application_FunctionSet.ApplicationFunctionSet_Follow();
  //Application_FunctionSet.ApplicationFunctionSet_Obstacle();
  //Application_FunctionSet.ApplicationFunctionSet_Tracking(1);   // 
  Application_FunctionSet.ApplicationFunctionSet_Standby();
  Application_FunctionSet.ApplicationFunctionSet_IRrecv();
  Application_FunctionSet.TeddyCtrlUARTMessage("Zoro");    // handle UART message and forward to TeddyCTrlManager

  Application_FunctionSet.CMD_ClearAllFunctions_xxx0();
}
