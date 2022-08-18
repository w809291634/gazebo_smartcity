#ifndef __ROS_SENSOR_H__
#define __ROS_SENSOR_H__

typedef void (*on_topic_msg_cb)(unsigned long long tm, char *msg);
typedef struct {
    FILE * fp;
    char * buffer;
    unsigned int buflen;
    on_topic_msg_cb callback;
} topic_proc_args_t;

int ros_topic_register(char *top, on_topic_msg_cb fun, int bufsize);
#endif
