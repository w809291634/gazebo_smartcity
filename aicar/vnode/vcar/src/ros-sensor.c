/*********************************************************************************************
* 文件：ros-sensor.c
* 作者：zonesion
* 说明：ros接口传感器数据获取
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
#include <pthread.h>
#include "ros-sensor.h"
 

FILE *popen(char*, char *);

void* thread_topic_proc(void *args)
{
    topic_proc_args_t* argv = (topic_proc_args_t*)args;
    
    while((fgets(argv->buffer, argv->buflen, argv->fp))!=NULL){
#if 1
        char *p = strchr(argv->buffer, ',');
        unsigned long long t;
        if (p != NULL) {
            *p++ = '\0';
        } else p = "";
        t = atol(argv->buffer);
        /* p 的结尾\n并没有去掉*/
        argv->callback(t, p);
#else
        argv->callback(0, argv->buffer);
#endif
    }
    fclose(argv->fp);
    free(argv->buffer);
    free(argv);
    return NULL;
}

int ros_topic_register(char *top, on_topic_msg_cb fun, int bufsize)
{
    FILE * fp;
    char cmd[128];
    topic_proc_args_t *pa;
    
    sprintf(cmd,"rostopic echo -p %s", top); 
    fp = popen(cmd, "r");
    if (fp == NULL){
        return -1;
    }
    void *pb = malloc(bufsize);
    if (pb == NULL) return -2;
    pa = malloc(sizeof (topic_proc_args_t));
    if (pa == NULL){
        free(pb);
        return -3;
    }
    pa->fp = fp;
    pa->buffer = pb;
    pa->buflen = bufsize;
    pa->callback = fun;
    pthread_t *pt = malloc(sizeof (pthread_t));
    if (pt == NULL) {
        free(pb);
        free(pa);
        return -4;
    }
    pthread_create(pt, NULL, thread_topic_proc, pa); //pt 没有回收
    return 0;
}


