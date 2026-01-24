#ifndef __UDP_APP_H__
#define __UDP_APP_H__

#include "main.h"

/* 定义端口配置 */
#define UDP_LOCAL_PORT  7000  // 开发板本机监听端口
#define UDP_REMOTE_PORT 7001  // 目标上位机(PC)端口

/* 定义目标IP (默认 PC IP，用于主动发送) */
#define DEST_IP_ADDR0   192
#define DEST_IP_ADDR1   168
#define DEST_IP_ADDR2   1
#define DEST_IP_ADDR3   10

void UDP_App_Init(void);
void UDP_Send_Data(uint8_t *data, uint16_t len);

#endif /* __UDP_APP_H__ */
