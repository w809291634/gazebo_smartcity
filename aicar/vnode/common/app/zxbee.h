#ifndef __ZXBEE_H__
#define __ZXBEE_H__

void ZXBeeBegin(void);
int ZXBeeAdd(char* tag, char* val);
char* ZXBeeEnd(void);

char* ZXBeeDecodePackage(char *pkg, int len);



void  _zxbee_send_call(char *pkg);
int ZXBeeSysCommandProc(char* ptag, char* pval);
int ZXBeeUserProcess(char* ptag, char* pval);
#endif