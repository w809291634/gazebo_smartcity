/*********************************************************************************************
* 文件：sensor.c
* 作者：zonesion
* 说明：通用节点传感器程序
* 修改：
* 注释：
*********************************************************************************************/

/*********************************************************************************************
* 头文件
*********************************************************************************************/
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include <unistd.h>
#include "../../common/app/zxbee.h"
#include "../../common/app/zxbee-inf.h"
#include "../../common/app/util.h"
#include "sensor.h"
#include "ros-sensor.h"

static char LicensePlate[16];
/*********************************************************************************************
* 全局变量
*********************************************************************************************/
#define ITEM_SIZE 64

static uint8_t D0 = 0x9F;                                       // 默认打开主动上报功能
static uint8_t D1 = 0;                                          // 继电器初始状态为全关
static char A0[ITEM_SIZE];                                            // A0[光强]&[温度]&[湿度]&[大气压力]&[TVOC]&[燃气]，示例：612&30.2&45.6&101.5&600&356
static char A1[ITEM_SIZE];                                              // A1存储电池电量，0~100%
static char A2[ITEM_SIZE];                                             // A2车牌信息，字符串
static char A3[ITEM_SIZE];                                             // 字符串，格式为a&b&c&d，abcd分别表示前后左右的距离，单位：mm
static char A4[ITEM_SIZE];                                             // 字符串，格式为a&b，a表示线速度m/s，b表示角速度rad/s
static char A5[ITEM_SIZE];                                         // 
static char A6[ITEM_SIZE];                                         // 
static char A7[ITEM_SIZE];                                             // A7存储原点为地图的中心
static uint16_t V0 = 30;                                        // V0设置为上报时间间隔，默认为30s
static char V1[ITEM_SIZE];                                             // V1，地图点&[导航|取货|送货]&参数，返回值是1/0
static uint8_t V2 = 0;                                             // 
static char V3[ITEM_SIZE]={"0&0"};                                             // V3，形式为a&b，a表示经度，b表示纬度

struct {
    /* 小车位置信息 */
    float pos_x;
    float pos_y;
    float pos_yaw;
    /* 小车速度信息 */
    float speed_x;
    float speed_yaw;
    
    /* 目标任务*/
    char  target[ITEM_SIZE];
    
    /* 车体传感器信息 */
    float vbat; //电池电压
    float temp; //环境温度
    unsigned int humi; //环境湿度
    unsigned int pressure; //环境气压
    unsigned int light;    //环境光强
    unsigned int mp503;    //tvoc
    unsigned int mp2;      //空气质量
    
    float sonar1;           //前声纳
    float sonar2;           //左声纳
    float sonar3;           //后声纳
    float sonar4;           //右声纳
    
    float uwb_x;        //uwb定位x
    float uwb_y;        //uwb定位y
} xcar_info;

#define property_set(i,  exp) \
    do{\
        char *p = getItem(msg,i); \
        if (p != NULL) {    \
            exp; \
        }   \
    }while(0)
            
char *getItem(char *str, int i)
{
    int j;
    char *p = str;
    for (j=0; j<i; j++) {
        if (p == NULL)break;
        p = strchr(p, ',');
        if (p != NULL) p++;
    }
    return p;
}

/*********************************************************************************************
* 名称：xcar_send_target
* 功能：设置小车的任务，即V1参数
* 参数： 
* 返回：
* 修改：
* 注释：
*********************************************************************************************/
void xcar_send_target(char *v1)
{
    char cmd[96];
    strcpy(xcar_info.target, v1);
    sprintf(cmd, "rostopic pub /demo/acar/to_client/target std_msgs/String \"data: '%s'\"  -1 &", v1);
    system(cmd);
}

/*********************************************************************************************
* 名称：xcar_send_mode
* 功能：设置小车的模式，即V2参数
* 参数： 
* 返回：
* 修改：
* 注释：
*********************************************************************************************/
void xcar_send_mode(int v2)
{
    char cmd[96];
    sprintf(cmd, "rostopic pub /demo/acar/mode std_msgs/String \"data: '%d'\"  -1 &", v2);
    system(cmd);
}
/*********************************************************************************************
* 名称：xcar_send_stop
* 功能：设置小车停止标志
* 参数： 
* 返回：
* 修改：
* 注释：
*********************************************************************************************/
void xcar_send_stop(int v5)
{
    char cmd[96];
    sprintf(cmd, "rostopic pub /demo/acar/stop std_msgs/String \"data: '%d'\"  -1 &", v5);
    system(cmd);
}
/*********************************************************************************************
* 名称：xcar_send_voice
* 功能：小车语音播报
* 参数： 
* 返回：
* 修改：
* 注释：
*********************************************************************************************/
void xcar_send_voice(char *v4)
{
    char cmd[512];
    sprintf(cmd, "rostopic pub /demo/acar/voice std_msgs/String \"data: '%s'\"  -1 &", v4);
    system(cmd);
}

/*********************************************************************************************
* 名称：on_sensor_msg_cb
* 功能：ros传感器数据解析
* 参数： 
* 返回：
* 修改：
* 注释：
*********************************************************************************************/
static void  on_sensor_msg_cb (unsigned long long tm, char *msg)
{
    //printf("on_sensor_msg_cb： %ld, %s", tm, msg);
    //"1634004184166723966, 0,124,287,23,2932,101561,7,330,579,20,-1,-1"
    if (tm != 0) {
        property_set(1, xcar_info.vbat = atoi(p)/10.0f);
        property_set(2, xcar_info.temp = atoi(p)/10.0f);
        property_set(3, xcar_info.humi = atoi(p));
        property_set(5, xcar_info.pressure = atoi(p));
        property_set(6, xcar_info.light = atoi(p));
        property_set(7, xcar_info.mp503 = atoi(p));
        property_set(8, xcar_info.mp2 = atoi(p));
        
        property_set(9, xcar_info.sonar1 = atoi(p)/100.0f);
        property_set(10, xcar_info.sonar2 = atoi(p)/100.0f);
        property_set(11, xcar_info.sonar3 = atoi(p)/100.0f);
        property_set(12, xcar_info.sonar4 = atoi(p)/100.0f);
    }
}

/*********************************************************************************************
* 名称：on_pos_msg_cb
* 功能： 小车位置解析
* 参数： 
* 返回：
* 修改：
* 注释：
*********************************************************************************************/
static void  on_pos_msg_cb (unsigned long long tm, char *msg)
{
    //printf("on_pos_msg_cb %ld, %s", tm, msg);
    if (tm != 0) {
        property_set(1, xcar_info.pos_x = atof(p));
        property_set(2, xcar_info.pos_y = atof(p));
        property_set(3, xcar_info.pos_yaw = atof(p));
        
        property_set(4, xcar_info.speed_x = atof(p));
        property_set(5, xcar_info.speed_yaw = atof(p));
    }
}

/*********************************************************************************************
* 名称：on_target_msg_cb
* 功能：任务话题监听
* 参数： 
* 返回：
* 修改：
* 注释：
*********************************************************************************************/
static void  on_target_msg_cb (unsigned long long tm, char *msg)
{
    //printf("%ld, %s", tm, msg);
    if (tm != 0) {
        msg[strlen(msg)-1] = '\0'; //去掉最后的\n
        strcpy(xcar_info.target, msg);
    }
}
/*********************************************************************************************
* 名称：on_uwb_msg_cb
* 功能： uwb数据处理
* 参数： 
* 返回：
* 修改：
* 注释：
*********************************************************************************************/
static void on_uwb_msg_cb(unsigned long long tm, char *msg)
{
    //printf("on_uwb_msg_cb %llu, %s", tm, msg);
    //on_uwb_msg_cb 1634094447272943973,
    // msg format 
    // 264,1634094447272943973,/gps,-1,1,nan,nan,nan,nan,0.0,0.0,0.0,nan,0.0,0.0,0.0,nan,1
    //                                    y,  x,  z
    if (tm != 0) {
        property_set(6, xcar_info.uwb_x = atof(p));
        property_set(5, xcar_info.uwb_y = atof(p));
    }
}
/*********************************************************************************************
* 名称：on_plate_msg_cb
* 功能： 应用设置车牌消息处理
* 参数： 
* 返回：
* 修改：
* 注释：
*********************************************************************************************/
static void on_plate_msg_cb(unsigned long long tm, char *msg)
{
    if (tm != 0) {
        msg[strlen(msg)-1] = '\0'; //去掉最后的\n
        strncpy(LicensePlate, msg, sizeof LicensePlate);
    }
}
/*********************************************************************************************
* 名称：updateV0()
* 功能：更新V0的值
* 参数：*val -- 待更新的变量
* 返回：
* 修改：
* 注释：
*********************************************************************************************/
void updateV0(char *val)
{
  //将字符串变量val解析转换为整型变量赋值
  V0 = atoi(val);                                               // 获取数据上报时间更改值
}
/*********************************************************************************************
* 名称：updateV3()
* 功能：更新V3的值
* 参数： 
* 返回：
* 修改：
* 注释：
*********************************************************************************************/
void updateV3()
{
    sprintf(V3, "%.3f&%.3f", xcar_info.uwb_x, xcar_info.uwb_y);
}
/*********************************************************************************************
* 名称：updateA0()
* 功能：更新A0的值
* 参数：
* 返回：
* 修改：
* 注释：
*********************************************************************************************/
void updateA0(void)
{
  // 温度值，浮点型：0.1精度，-40.0~70.0，单位：°C 
    sprintf(A0, "%d&%.1f&%d&%.1f&%d&%d",
        xcar_info.light,xcar_info.temp, xcar_info.humi,
        xcar_info.pressure/1000.0f, xcar_info.mp503,
        xcar_info.mp2);
}
/*********************************************************************************************
* 名称：updateA1()
* 功能：更新A1的值
* 参数：
* 返回：
* 修改：
* 注释：
*********************************************************************************************/
void updateA1(void)
{
    //电池电量，0~100%
    if (xcar_info.vbat >= 12.5f){
        sprintf(A1, "100");
    } else if (xcar_info.vbat<10.5f) {
        sprintf(A1, "0");
    } else {
        int v = (xcar_info.vbat - 10.5f)/2.0f*100;
        sprintf(A1, "%d", v);
    }
}
/*********************************************************************************************
* 名称：updateA2()
* 功能：更新A2的值
* 参数：
* 返回：
* 修改：
* 注释：
*********************************************************************************************/
void updateA2(void)
{
     //车牌信息，字符串
  strcpy(A2, LicensePlate);                                 
}
/*********************************************************************************************
* 名称：updateA3()
* 功能：更新A3的值
* 参数：
* 返回：
* 修改：
* 注释：
*********************************************************************************************/
void updateA3(void)
{
    //前后左右的距离，格式为a&b&c&d 单位m
    sprintf(A3, "%.2f&%.2f&%.2f&%.2f", (xcar_info.sonar1),
        (xcar_info.sonar3), (xcar_info.sonar2),
        (xcar_info.sonar4));
}
/*********************************************************************************************
* 名称：updateA4()
* 功能：更新A4的值
* 参数：
* 返回：
* 修改：
* 注释：
*********************************************************************************************/
void updateA4(void)
{
    // a表示线速度m/s，b表示角速度rad/s，格式为a&b   
    sprintf(A4, "%.3f&%.3f", xcar_info.speed_x, xcar_info.speed_yaw);
}
/*********************************************************************************************
* 名称：updateA5()
* 功能：更新A5的值
* 参数：
* 返回：
* 修改：
* 注释：
*********************************************************************************************/
void updateA5(void)
{

}
/*********************************************************************************************
* 名称：updateA6()
* 功能：更新A6的值
* 参数：
* 返回：
* 修改：
* 注释：
*********************************************************************************************/
void updateA6(void)
{

}
/*********************************************************************************************
* 名称：updateA7()
* 功能：更新A7的值
* 参数：
* 返回：
* 修改：
* 注释：
*********************************************************************************************/
void updateA7(void)
{
   sprintf(A7, "%.3f&%.3f&%.3f", xcar_info.pos_x, xcar_info.pos_y, xcar_info.pos_yaw);
}
/*********************************************************************************************
* 名称：sensorInit()
* 功能：传感器硬件初始化
* 参数：无
* 返回：无
* 修改：
* 注释：
*********************************************************************************************/
void sensorInit(void)
{
    strncpy(LicensePlate, "鄂A111111", sizeof LicensePlate);
    // 初始化传感器代码
    ros_topic_register("/demo/acar/to_server/target", on_target_msg_cb, 128);
    //ros_topic_register("/demo/acar/to_client/target", on_target_msg_cb, 128);
    ros_topic_register("/demo/acar/pos", on_pos_msg_cb, 128);
    ros_topic_register("/xcar/sensors", on_sensor_msg_cb, 128);
    ros_topic_register("/gps/fix", on_uwb_msg_cb, 128);
    ros_topic_register("/demo/acar/plate", on_plate_msg_cb, 128);
}
/*********************************************************************************************
* 名称：sensorUpdate()
* 功能：处理主动上报的数据
* 参数：无
* 返回：无
* 修改：
* 注释：
*********************************************************************************************/
void sensorUpdate(void)
{ 
 
  
  ZXBeeBegin();                                                 // 智云数据帧格式包头
  
  // 根据D0的位状态判定需要主动上报的数值
  if ((D0 & 0x01) == 0x01){                                    
    updateA0();
    ZXBeeAdd("A0", A0);
  }
  if ((D0 & 0x02) == 0x02){                                     
    updateA1();
    ZXBeeAdd("A1", A1);
  }
  if ((D0 & 0x04) == 0x04){                                   
    updateA2();
    ZXBeeAdd("A2", A2);
  }
  if ((D0 & 0x08) == 0x08){                                  
    updateA3();
    ZXBeeAdd("A3", A3);
  }
  if ((D0 & 0x10) == 0x10){                                 
    updateA4();
    ZXBeeAdd("A4", A4);
  }
  if ((D0 & 0x20) == 0x20){                                    
    ZXBeeAdd("A5", A5);
  }
  if ((D0 & 0x40) == 0x40){                                    
    updateA6();
    ZXBeeAdd("A6", A6);
  }
  if ((D0 & 0x80) == 0x80){                                  
    updateA7();
    ZXBeeAdd("A7", A7);
  }
 
  
  char *p = ZXBeeEnd();                                               // 智云数据帧格式包尾
  if (p != NULL) {												
    ZXBeeInfSend(p, strlen(p));	                                // 将需要上传的数据进行打包操作，并通过zb_SendDataRequest()发送到协调器
  }
}
/*********************************************************************************************
* 名称：sensorCheck()
* 功能：传感器监测
* 参数：无
* 返回：无
* 修改：
* 注释：
*********************************************************************************************/
void sensorCheck(void)
{
    ZXBeeBegin();
    // 根据D0的位来决定是否上报
    if (strcmp(V1, xcar_info.target)) {
        strcpy(V1, xcar_info.target);
        ZXBeeAdd("V1", V1);
    }
    if ((D0 & (1<<3)) != 0) {   
        char a3[ITEM_SIZE];
        strcpy(a3, A3);
        updateA3();
        if (strcmp(a3, A3)){
            ZXBeeAdd("A3", A3);
        }
    }
    if ((D0 & (1<<4)) != 0) {   
        char a4[ITEM_SIZE];
        strcpy(a4, A4);
        updateA4();
        if (strcmp(a4, A4)){
            ZXBeeAdd("A4", A4);
        }
    }
    if ((D0 & (1<<7)) != 0) {   
        char a7[ITEM_SIZE];
        strcpy(a7, A7);
        updateA7();
        if (strcmp(a7, A7)){
            ZXBeeAdd("A7", A7);
        }
    }
    char *p = ZXBeeEnd();
    if ((p != NULL)) {
        int len = strlen(p); 
        ZXBeeInfSend(p, len);
    }
}
/*********************************************************************************************
* 名称：sensorControl()
* 功能：传感器控制
* 参数：cmd - 控制命令
* 返回：无
* 修改：
* 注释：
*********************************************************************************************/
void sensorControl(uint8_t cmd)
{
  // 根据cmd参数处理对应的控制程序
}
/*********************************************************************************************
* 名称：ZXBeeUserProcess()
* 功能：解析收到的控制命令
* 参数：*ptag -- 控制命令名称
*       *pval -- 控制命令参数
* 返回：ret -- pout字符串长度
* 修改：
* 注释：
*********************************************************************************************/
int ZXBeeUserProcess(char *ptag, char *pval)
{ 
  int val;
  int ret = 0;	
  char p[64];
 
  
  // 将字符串变量pval解析转换为整型变量赋值
  val = atoi(pval);	
  // 控制命令解析
  if (0 == strcmp("CD0", ptag)){                                // 对D0的位进行操作，CD0表示位清零操作
    D0 &= ~val;
  }
  if (0 == strcmp("OD0", ptag)){                                // 对D0的位进行操作，OD0表示位置一操作
    D0 |= val;
  }
  if (0 == strcmp("D0", ptag)){                                 // 查询上报使能编码
    if (0 == strcmp("?", pval)){
      sprintf(p, "%u", D0);
      ZXBeeAdd("D0", p);
    } 
  }
  if (0 == strcmp("CD1", ptag)){                                // 对D1的位进行操作，CD1表示位清零操作
    D1 &= ~val;
    sensorControl(D1);                                          // 处理执行命令
  }
  if (0 == strcmp("OD1", ptag)){                                // 对D1的位进行操作，OD1表示位置一操作
    D1 |= val;
    sensorControl(D1);                                          // 处理执行命令
  }
  if (0 == strcmp("D1", ptag)){                                 // 查询执行器命令编码
    if (0 == strcmp("?", pval)){
      sprintf(p, "%u", D1);
      ZXBeeAdd("D1", p);
    } 
  }
  if (0 == strcmp("A0", ptag)){ 
    if (0 == strcmp("?", pval)){
      updateA0();
      ZXBeeAdd("A0", A0);
    } 
  }
  if (0 == strcmp("A1", ptag)){ 
    if (0 == strcmp("?", pval)){
      updateA1();
      ZXBeeAdd("A1", A1);
    } 
  }
  if (0 == strcmp("A2", ptag)){ 
    if (0 == strcmp("?", pval)){
      updateA2(); 
      ZXBeeAdd("A2", A2);
    } 
  }
  if (0 == strcmp("A3", ptag)){ 
    if (0 == strcmp("?", pval)){
      updateA3();
      ZXBeeAdd("A3", A3);
    } 
  }
  if (0 == strcmp("A4", ptag)){ 
    if (0 == strcmp("?", pval)){
      updateA4();
      ZXBeeAdd("A4", A4);
    } 
  }
  if (0 == strcmp("A5", ptag)){ 
    if (0 == strcmp("?", pval)){
      updateA5();
      ZXBeeAdd("A5", A5);
    } 
  }
  if (0 == strcmp("A6", ptag)){ 
    if (0 == strcmp("?", pval)){
      updateA6();
      ZXBeeAdd("A6", A6);
    } 
  }
  if (0 == strcmp("A7", ptag)){ 
    if (0 == strcmp("?", pval)){
      updateA7();
      ZXBeeAdd("A7", A7);
    } 
  }
  if (0 == strcmp("V0", ptag)){
    if (0 == strcmp("?", pval)){
      ret = sprintf(p, "%u", V0);                         	// 上报时间间隔
      ZXBeeAdd("V0", p);
    }else{
      updateV0(pval);
    }
  }
  if (0 == strcmp("V1", ptag)){
    if (0 == strcmp("?", pval)){
      ZXBeeAdd("V1", V1);
    }else{
      strcpy(V1, pval);
      xcar_send_target(pval);
    }
  }
  if (0 == strcmp("V2", ptag)){
    if (0 == strcmp("?", pval)){
      sprintf(p, "%u", V2);
      ZXBeeAdd("V2", p);
    }else{
       V2 = atoi(pval);
       xcar_send_mode(V2);
    }
  }
  if (0 == strcmp("V3", ptag)){
    if (0 == strcmp("?", pval)){
      updateV3();
      ZXBeeAdd("V3", V3);
    } 
  }
  if (0 == strcmp("V4", ptag)){
      xcar_send_voice(pval);
  }
  if (0 == strcmp("V5", ptag)){
      xcar_send_stop(atoi(pval));
  }
  return ret;
}

/*********************************************************************************************
* 名称：sensor()
* 功能：传感器采集线程
* 参数：
* 返回：
* 修改：
* 注释：
*********************************************************************************************/
void sensorPoll(void)
{
    static uint32_t lastUpdateTm, sensorCheckTm;

    if (timeoutMs(sensorCheckTm)) {
        sensorCheck();
        sensorCheckTm = millis()+1000;
    }
    if (timeoutMs(lastUpdateTm)){
        sensorUpdate();
        lastUpdateTm = millis()+1000*V0;
    }
    
}
