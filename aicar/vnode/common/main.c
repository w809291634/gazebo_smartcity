#include <stdio.h>
#include <string.h>
#include <getopt.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <linux/if.h>
#include "app/zxbee-inf.h"

void sensorInit(void);
void sensorPoll();

static struct option options[] = {
    {"help", 0, NULL, 'h'},
    {"mac", required_argument, NULL, 'm'},
    {"gwip", required_argument, NULL, 'g'},
    {"port", required_argument, NULL, 'p'},
    {0,0,0,0}
};


char *get_mac(void)
{
    static char mac[32];
    struct ifreq req;
    int sock = 0;
    struct ifconf ifc;
    char buf[1024];

    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0){
        perror("error sock");
        return NULL;
    }

    
    ifc.ifc_len = sizeof(buf);
    ifc.ifc_buf = buf;
    if (ioctl(sock, SIOCGIFCONF, &ifc) == -1) {
        perror("SIOCGIFCONF");
        close(sock);
        return NULL;
    }

    struct ifreq* it = ifc.ifc_req;

    const struct ifreq* const end = it + (ifc.ifc_len / sizeof(struct ifreq));

    char szMac[64];

    int count = 0;

    for (; it != end; ++it) {
        strcpy(req.ifr_name, it->ifr_name);
        if (ioctl(sock, SIOCGIFFLAGS, &req) == 0) {
            if (!(req.ifr_flags & IFF_LOOPBACK)) { // don't count loopback
                if (ioctl(sock, SIOCGIFHWADDR, &req) < 0){
                    perror("error ioctl");
                    close(sock);
                    return NULL;
                }
                sprintf(mac, "%02X:%02X:%02X:%02X:%02X:%02X",
                req.ifr_hwaddr.sa_data[0]&0xff,req.ifr_hwaddr.sa_data[1]&0xff,
                req.ifr_hwaddr.sa_data[2]&0xff,req.ifr_hwaddr.sa_data[3]&0xff,
                req.ifr_hwaddr.sa_data[4]&0xff,req.ifr_hwaddr.sa_data[5]&0xff);
                close(sock);
                return mac;
            }
        }
    }
    return NULL;
}
static void usage(void)
{
    printf("usage:\n");
    printf("       -g <gwip> 网关ip\n");
    printf("       -p <port> 可选网关端口\n");
    printf("       -m <mac>  可选节点MAC地址\n");
    printf("       -h 帮助信息\n");
}
int main(int argc, char* argv[])
{
    char *pmac = NULL;
    char *host = NULL;
    int port = 7003;
    
    while (1) {
        int optidx;
        int ret;
        ret = getopt_long(argc, argv, "hm:g:p:", options, &optidx);
        if (ret < 0) break;
        switch(ret){
            case 'h':
                usage();
                return 0;
            case 'm':
                pmac = optarg;
                break;
            case 'g':
                host = optarg;
                break;
            case 'p':
                port = atoi(optarg);
                break;
        }
    }
    if (host == NULL) {
        host = "127.0.0.1";
    }
    if (pmac == NULL) {
        if (strlen(NODE_MAC) > 0){
            pmac = NODE_MAC;
        } else {
            pmac = get_mac( );
            if (pmac == NULL) {
                printf("错误: 获取mac地址失败，请使用-m 选项设置mac地址\n");
                return 1;
            }
            static char nmac[32];
            sprintf(nmac, "WIFI:%s", pmac);
            pmac = nmac;
        }
    }
    
    if (strlen(pmac) != 22){
        printf("错误: mac %s, mac地址长度应该为22.\n", pmac);
        return 1;
    }
    ZXBeeInfInit(pmac, host, port);
    
    sensorInit();
     
    while(1) {
        sensorPoll();
        ZXBeePoll(100);
    }
    return 0;
}
