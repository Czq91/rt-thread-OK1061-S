#include "protocol.h"
#include "mpu6050.h"
#include <rtthread.h>
#include <sys/socket.h> /* 使用BSD socket，需要包含sockets.h头文件 */
#include <netdb.h>
#include <string.h>
#include <finsh.h>


#define RTCM_SUB_PACK_LEN 128
#define RTCM_DATA_SUB_PACK_LEN 120
rt_thread_t _udpclient01_tid = RT_NULL;
const char send_data[] = "This is UDP Client from RT-Thread.\n"; /* 发送用到的数据 */
extern MPU6050_t MPU6050;
imu_msg_t imu_msg = {0};
char _buf[40] = {0};
static int sock = -1;

char _udpclient01_url[16] = "192.168.1.20";
char _udpclient01_port[10] = "5000";
extern rt_sem_t rtcm_data_sem;

extern rt_uint8_t rtcm_raw_data[];
extern rt_uint16_t rtcm_data_len;

// void pack_rtcm_data(void)
// {
//     protocol_data_t rtcm_sub_pack = (protocol_data_t)rt_malloc(RTCM_SUB_PACK_LEN);
//     rtcm_sub_pack.timestamp = time(RT_NULL);
//     rtcm_sub_pack.sequence = 0;
//     rtcm_sub_pack.type = 0x20;
//     rtcm_sub_pack.length = RTCM_DATA_SUB_PACK_LEN;

//     for ()
//     {
        
//     }
// }

static void udpclient_task_entry(void *parameter)
{
    int port, count;
    struct hostent *host;
    struct sockaddr_in server_addr;
    const char *url;

    rt_thread_mdelay(3000);

    url = _udpclient01_url;
    port = strtoul(_udpclient01_port, 0, 10);

    host = (struct hostent *)gethostbyname(url);

    if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
    {
        rt_kprintf("Socket error\n");
        return;
    }

    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    server_addr.sin_addr = *((struct in_addr *)host->h_addr);
    rt_memset(&(server_addr.sin_zero), 0, sizeof(server_addr.sin_zero));

    while (1)
    {
        //        imu_msg.head.timestamp = rt_tick_get();

        //        imu_msg.Temperature = MPU6050.Temperature;
        //
        //        imu_msg.Accel_x = MPU6050.Ax;
        //        imu_msg.Accel_y = MPU6050.Ay;
        //        imu_msg.Accel_z = MPU6050.Az;

        //        imu_msg.Yaw  = 0;
        //        imu_msg.Roll = 0;
        //        imu_msg.Pitch = 0;

        // rt_kprintf("len is: %d\n", sizeof(imu_msg_t));
        if (rt_sem_take(rtcm_data_sem, RT_WAITING_FOREVER) == RT_EOK)
        {
            //					rt_kprintf("main trcm data:\r\n");
            //					for(int i = 0;i<rtcm_data_len;i++)
            //					{
            //						rt_kprintf("%x",rtcm_raw_data[i]);
            //					}
            // rt_kprintf("\r\n");
            rtcm_data_len = 900;

            protocol_data_t *rtcm_sub_pack = (protocol_data_t*)rt_malloc(RTCM_SUB_PACK_LEN);
            rtcm_sub_pack->timestamp = time(RT_NULL);
            rtcm_sub_pack->sequence = 0;
            rtcm_sub_pack->type = 0x20;
            rtcm_sub_pack->length = RTCM_DATA_SUB_PACK_LEN;


            int i = 0;
            for (i = 0; i < rtcm_data_len / RTCM_DATA_SUB_PACK_LEN; i++)
            {
                memcpy(rtcm_sub_pack->data, &rtcm_raw_data[i*RTCM_DATA_SUB_PACK_LEN], RTCM_DATA_SUB_PACK_LEN);
                rtcm_sub_pack->sequence = i + 1;
                sendto(sock, &rtcm_sub_pack, RTCM_SUB_PACK_LEN, 0,
                   (struct sockaddr *)&server_addr, sizeof(struct sockaddr));
                //rt_kprintf("****************i: %d\r\n", i);
                rt_thread_mdelay(1000);
            }

            int rtcm_last_pkg_data_len = rtcm_data_len % RTCM_DATA_SUB_PACK_LEN;
            if (rtcm_last_pkg_data_len > 0)
            {
                memcpy(rtcm_sub_pack->data, &rtcm_raw_data[i*RTCM_DATA_SUB_PACK_LEN], rtcm_last_pkg_data_len);
                rtcm_sub_pack->sequence = i + 1;
                sendto(sock, &rtcm_sub_pack, rtcm_last_pkg_data_len, 0,
                   (struct sockaddr *)&server_addr, sizeof(struct sockaddr));
                //rt_kprintf("****************i: %d\r\n", i);
                rt_thread_mdelay(1000);
            }
        }

        // rt_thread_mdelay(1000);
    }

    rt_kprintf("closesocket\r\n");
    closesocket(sock);
}

int cofig_udp_remote_id_port(char *id, char *port)
{
    memset(_udpclient01_url, 0, sizeof(_udpclient01_url));
    memcpy(_udpclient01_url, id, strlen(id));

    memset(_udpclient01_port, 0, sizeof(_udpclient01_port));
    memcpy(_udpclient01_port, port, strlen(port));

    udpclient_deinit();
    rt_thread_delay(10);
    udpclient_init();
}

int udpclient_deinit(void)
{
    if (_udpclient01_tid != RT_NULL)
    {
        rt_thread_delete(_udpclient01_tid);
        _udpclient01_tid = RT_NULL;
    }

    // if(sock != -1)
    // {
    //     closesocket(sock);
    //     sock = -1;
    // }
}

int udpclient_init(void)
{

    if (_udpclient01_tid != RT_NULL)
    {
        return -1;
    }

    _udpclient01_tid = rt_thread_create("udpclient01",
                                        udpclient_task_entry, (void *)1,
                                        2048,
                                        15, 10);
    if (_udpclient01_tid != RT_NULL)
        rt_thread_startup(_udpclient01_tid);

    return 0;
}

#ifndef GNSS_SLAVE_MODE
INIT_APP_EXPORT(udpclient_init);
#endif
