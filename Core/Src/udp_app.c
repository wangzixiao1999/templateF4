#include "udp_app.h"
#include "lwip/udp.h"
#include "string.h"

/* 全局 UDP 控制块 */
static struct udp_pcb *upcb;

/**
  * @brief  UDP 接收回调函数
  * @param  arg: 用户参数 (未用)
  * @param  pcb: UDP 控制块
  * @param  p:   接收到的数据包缓冲区 (pbuf)
  * @param  addr: 发送方的 IP 地址
  * @param  port: 发送方的端口号
  * @retval None
  */
void udp_receive_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
    if (p != NULL)
    {
        /* --- Echo 功能: 将接收到的数据原样发回给发送者 --- */

        /* 发送 UDP 数据包 */
        /* 注意：udp_sendto 不需要重新分配 pbuf，直接使用接收到的 p 即可 */
        udp_sendto(pcb, p, addr, port);

        /*
           如果需要处理数据，请在这里添加逻辑。
           数据指针: p->payload
           数据长度: p->len

           例如:
           uint8_t *data = (uint8_t *)p->payload;
           if (data[0] == 0x01) { ... }
        */

        uint8_t *data = (uint8_t *)p->payload;
        /* 重要：使用完 pbuf 后必须释放内存，否则会导致内存泄漏，最终无法接收新数据 */
        pbuf_free(p);
    }
}

/**
  * @brief  初始化 UDP 应用
  * @retval None
  */
void UDP_App_Init(void)
{
    err_t err;

    /* 1. 创建一个新的 UDP 控制块 */
    upcb = udp_new();

    if (upcb)
    {
        /* 2. 绑定本地 IP 和端口 */
        /* IP_ADDR_ANY 表示接收发送给本机任何 IP 的数据 */
        err = udp_bind(upcb, IP_ADDR_ANY, UDP_LOCAL_PORT);

        if (err == ERR_OK)
        {
            /* 3. 注册接收回调函数 */
            udp_recv(upcb, udp_receive_callback, NULL);
        }
        else
        {
            /* 绑定失败，移除控制块 */
            udp_remove(upcb);
        }
    }
}

/**
  * @brief  主动向指定目标发送 UDP 数据
  * @param  data: 数据指针
  * @param  len: 数据长度
  * @retval None
  */
void UDP_Send_Data(uint8_t *data, uint16_t len)
{
    struct pbuf *p;
    ip_addr_t destIP;

    if (upcb == NULL) return;

    /* 设置目标 IP 地址 */
    IP4_ADDR(&destIP, DEST_IP_ADDR0, DEST_IP_ADDR1, DEST_IP_ADDR2, DEST_IP_ADDR3);

    /* 分配 pbuf (Transport 层) */
    p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);

    if (p != NULL)
    {
        /* 将数据拷贝到 pbuf payload */
        pbuf_take(p, data, len);

        /* 发送 UDP 数据包 */
        udp_sendto(upcb, p, &destIP, UDP_REMOTE_PORT);

        /* 发送完毕后释放 pbuf */
        pbuf_free(p);
    }
}
