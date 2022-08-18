/*********************************************************************************************
* 文件：zxbee-sys-command.c
* 作者：xuzhy 2018.1.12
* 说明：lte 模块公共zxbee指令处理模块
*       
*       
*       
* 修改：
* 注释：
*********************************************************************************************/
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "zxbee.h"



/*********************************************************************************************
* 名称：ZXBeeSysCommandProc
* 功能：lte模块公共zxbee指令处理
* 参数：
* 返回：
* 修改：
* 注释：
*********************************************************************************************/
int ZXBeeSysCommandProc(char* ptag, char* pval)
{
  int ret = -1;
  if (memcmp(ptag, "ECHO", 4) == 0) {
    ZXBeeAdd(ptag, pval);
    return 1;
  }
  if (memcmp(ptag, "TYPE", 4) == 0) {
    if (pval[0] == '?') {
      char buf[16];
      ret = sprintf(buf, "32%03d", SENSOR_TYPE);
      ZXBeeAdd("TYPE", buf);
      return 1;
    }
  }
 
  return ret;
}


