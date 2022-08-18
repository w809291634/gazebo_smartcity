/*********************************************************************************************
* 文件：sensor.h
* 作者：Xuzhy 2018.5.16
* 说明：sensor头文件
* 修改：
* 注释：
*********************************************************************************************/
#ifndef __SENSOR_H__
#define __SENSOR_H__
/*********************************************************************************************
* 宏定义
*********************************************************************************************/
#include <stdint.h>

typedef struct {
  char      A0[128];     //[光强]&[温度]&[湿度]&[大气压力]&[TVOC]&[燃气]，示例：612&30.2&45.6&101.5&600&356
                    //光强值，整型：0~18000，单位：Lux
                    //温度值，浮点型：0.1精度，-40.0~70.0，单位：°C 
                    //湿度值，浮点型：0.1精度，0~100.0，单位：%RH
                    //大气压力值，浮点型：0.1精度，0~120.0，单位：kPa
                    //TVOC含量，整型：0~1000，单位：ug/m3
                    //燃气浓度，整型：0~1000，单位：ug/m3
  int       A1;     //电池电量，0~100%
  char      A2[20];     //车牌信息，字符串
  char      A3[20];     //字符串，格式为a&b&c&d，abcd分别表示前后左右的距离，单位：mm
  char      A4[20];     //字符串，格式为a&b，a表示线速度m/s，b表示角速度rad/s
  int       A5;
  int       A6;
  char      A7[20]; //实时位置，字符串，格式为a&b，a表示x轴，b表示y轴，原点为地图的中心示例：-1.34&5.20
  uint8_t   D0;     //D0的Bit0~7分别代表A0~A7的上报状态，1表示主动上报，0为不上报
  uint8_t   D1;     //
  uint16_t  V0;     //A0~A7主动上报时间间隔，默认30，单位：S
  char      V1[20]; //字符串，格式为：地图点&[导航|取货|送货]&参数，返回值是1/0
                    //命令：地图点&任务&参数（地图点：车位1~16对应1~16，任务：1导航|2取货|3送货，参数可以没有），示例：15&1
                    //返回：0/1（命令执行成功后返回1）
  char      V2[20];
  char      V3[28]; //uwb位置，字符串，格式为a&b，a表示x轴，b表示y轴，原点为地图的中心 示例：-1.34&5.20
} t_sensor_value;

typedef struct {
  int A0_1;
  float A0_2;
  float A0_3;
  float A0_4;
  int A0_5;
  int A0_6;
} t_A0_value;

typedef struct {
  int A3_forward;   //超声波前
  int A3_back;      //超声波后
  int A3_left;      //超声波左
  int A3_right;     //超声波右
} t_A3_value;


typedef struct {
  char A4_linear[10];    //线速度
  char A4_angular[10];   //角速度
} t_A4_value;


typedef struct {
  char A7_x[10];         //实时位置x轴
  char A7_y[10];         //实时位置y轴
} t_A7_value;


typedef struct {
  int V1_pos;         //地图点
  int V1_task;        //1.导航|2.取货|3.送货
} t_V1_value;

typedef struct {
  float V3_x;         //UWB位置x轴
  float V3_y;         //UWB位置y轴
} t_V3_value;

extern t_sensor_value sensor_value;
extern t_A0_value A0_value;
extern t_A3_value A3_value;
extern t_A4_value A4_value;
extern t_A7_value A7_value;
extern t_V1_value V1_value;
extern t_V3_value V3_value;

#endif
