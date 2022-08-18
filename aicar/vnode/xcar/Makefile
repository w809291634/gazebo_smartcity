#

#TARGET程序名称
TARGET := vnode-xcar
#SENSOR_TYPE传感器节点类型          
SENSOR_TYPE := 510
#NODE_MAC 节点MAC地址，类似 WIFI:02:42:14:a8:2d:4e，默认空，程序自动获取
NODE_MAC :=   

#CROSS_COMPILE=arm-linux-
CC := $(CROSS_COMPILE)gcc
LD := $(CROSS_COMPILE)ld

CFLAGS := -Wall -DSENSOR_TYPE=$(SENSOR_TYPE)  -DNODE_MAC=\"$(NODE_MAC)\"

CFLAGS += -std=c99

LDFLAGS := -lpthread 

SRCS := $(wildcard ./src/*.c)

SRCS += $(wildcard ../common/*.c)
SRCS += $(wildcard ../common/app/*.c)
SRCS += $(wildcard ../common/dev/*.c)

OBJS := $(patsubst %c, %o, $(SRCS))

all:$(TARGET)

$(TARGET):$(OBJS)
	$(CC)  *.o $(LDFLAGS)  -o $@

%.o:%.c
	$(CC) $(CFLAGS) -c $<

clean:
	rm -rf $(TARGET) *.o 
    
    
