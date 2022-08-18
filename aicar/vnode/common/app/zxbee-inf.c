/*********************************************************************************************
* 文件：zxbee-inf.c
* 作者：xuzhy 
* 说明：
*       
*       
*      
* 修改：
* 注释：
*********************************************************************************************/
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <sys/select.h>
#include <sys/epoll.h>
#include <arpa/inet.h>
#include <errno.h>

#include "zxbee.h"
#include "zxbee-inf.h"
#define DEBUG 1
#define USE_CONNECT     0

static char sMac[32];
static int sMacLen;
struct sockaddr_in server_addr;

static int gsk;

/*********************************************************************************************
* 名称：ZXBeeInfInit
* 功能：ZXBee接口底层初始化
* 参数：
* 返回：
* 修改：
* 注释：
*********************************************************************************************/
void ZXBeeInfInit(char *mac, char* gwip, int gwport)
{
    char umac[32];
    int offset = 0;
    sMacLen = strlen(mac);
    for (int i=0; i<sMacLen; i++){
        umac[i] = toupper(mac[i]);
    }
    umac[sMacLen] = 0;
    if (memcmp(umac, "WIFI:",5) == 0) {
        offset = 5;
    }
    sMacLen -= offset;
    memcpy(sMac, &umac[offset], sMacLen);
    
    printf("   mac: %s\n", umac);
    printf("  gwIp: %s\n", gwip);
    printf("gwPort: %d\n", gwport);
    
   
    gsk = socket(AF_INET, SOCK_DGRAM, 0);
    if (gsk < 0) {
        printf("Error: socket %s\n", strerror(errno));
    }
    
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(gwport);
    server_addr.sin_addr.s_addr = inet_addr(gwip);
#if USE_CONNECT
    if(connect(gsk, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0){
        printf("Error: connect %s\n", strerror(errno));
    }
#endif
}
/*********************************************************************************************
* 名称：ZXBeeInfSend
* 功能：ZXBee底层发送接口
* 参数：p: ZXBee格式数据
       len：数据长度
* 返回：
* 修改：
* 注释：
*********************************************************************************************/
void  ZXBeeInfSend(char *p, int len)
{
    char buf[1024];
    
    int hlen = sprintf(buf, "%s=", sMac);
#if DEBUG
    printf("<<< %s\n", p);
#endif
    if ((hlen + len) >= sizeof(buf)) return;
    
    memcpy(&buf[hlen], p, len);
    len += hlen;
#if USE_CONNECT
    send(gsk, buf, len, 0);
#else
    sendto(gsk, buf, len, 0, (struct sockaddr *)&server_addr, sizeof(server_addr));
#endif
}
/*********************************************************************************************
* 名称：ZXBeeInfRecv
* 功能：接收到ZXbee数据报处理函数
* 参数：
* 返回：
* 修改：
* 注释：
*********************************************************************************************/
void ZXBeeInfRecv(char *buf, int len)
{
#if DEBUG
    buf[len] = 0;
    printf(">>> %s\n", buf);
#endif
  char *p = ZXBeeDecodePackage(buf, len);
  if (p != NULL) {
    ZXBeeInfSend(p, strlen(p));
  }
}

void ZXBeePoll(int ms)
{
    char buf[1024];
    struct sockaddr_in addr;
    int addr_len = sizeof (addr);
    int rlen;
    
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(gsk, &fds);
    
    int ret;
    struct timeval delay;
	delay.tv_sec = ms/1000;
	delay.tv_usec = (ms%1000) * 1000; 
	ret = select(gsk+1, &fds, NULL, NULL, &delay);
    if (ret < 0) {
        printf("Error: selece ret %d\n", ret);
        return;
    }
    if (FD_ISSET(gsk, &fds)) {
         
#if USE_CONNECT
        rlen = recv(gsk, buf, sizeof(buf), 0);
#else
        rlen = recvfrom(gsk, buf, sizeof(buf), 0, (struct sockaddr *)&addr, (socklen_t *)&addr_len);
#endif
        if (rlen < 0) {
            printf("Error: recvfrom %s\n", strerror(errno));
        }
        if (rlen > 0) {
            buf[rlen] = 0;
            printf(">>> %s\n", buf);
        }
        if (rlen > sMacLen) {
            if (memcmp(sMac, buf, sMacLen) == 0 && buf[sMacLen]=='=') {
               ZXBeeInfRecv(&buf[sMacLen+1], rlen-sMacLen-1);
            }
        }
    }
}
