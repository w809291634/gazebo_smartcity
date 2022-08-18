#ifndef __ZXBEE_INF_H__
#define __ZXBEE_INF_H__

void ZXBeeInfInit(char *mac, char* gwip, int gwport);
void  ZXBeeInfSend(char *p, int len);
void ZXBeeInfRecv(char *pkg, int len);
void ZXBeePoll(int ms);
#endif
