#include "rtthread.h"
#include "drv_ec200.h"
#include <stdlib.h>
#include "drv_e220.h"
#include "protocol.h"
#include "drv_gnss.h"
#include "mpu6050.h"

#define RT_USING_FINSH
#ifdef RT_USING_FINSH
#include <finsh.h>

char* chrIPAddress = RT_LWIP_IPADDR;
char* chrGateway = RT_LWIP_GWADDR;
char* chrSubnetMask = RT_LWIP_MSKADDR;
extern void set_if(const char *netif_name, const char *ip_addr, const char *gw_addr, const char *nm_addr);
dev_status_t gnss_dev_status;
extern gnss_conf_info_t gnss_conf_info;
extern bool gnss_slave_mode_ready;

extern rt_sem_t rtcm_data_sem;

/********************* gnss cmd ************************/
/* 设置GNSS模式，基准站或移动站, 经纬度，高程，数据刷新率 */
static int _gnss_set_mode(int argc, char **argv)
{
	set_gnss_mode();
//    if (6 == argc)
//    {
//        gnss_dev_status.gnss_mode = (rt_uint8_t)atoi(argv[1]); // gnss 一体机模式
//        gnss_conf_info.longitude = argv[2];     // 经度
//        gnss_conf_info.latitude = argv[3];      // 纬度
//        gnss_conf_info.elevation = argv[4];     // 高程
//        gnss_conf_info.refresh_rate = argv[5];  // 数据刷新率
//    }

    return 0;
}
MSH_CMD_EXPORT_ALIAS(_gnss_set_mode, gnss_set_mode, gnss cmd);

/* 设置星座，一般GPS + 北斗 */
static int _gnss_set_ss(int argc, char **argv)
{
    int ret = -1;
    if (argc == 2)
    {
        if(set_constellation((uint8_t)atoi(argv[1])))
        {
            ret = 0;
            rt_kprintf("GNSS Constellation setting succeeded\r\n");
        }
        else
        {
            ret = -1;
            rt_kprintf("GNSS Constellation setting failed\r\n");
        }
    }
    
    return ret;
}
MSH_CMD_EXPORT_ALIAS(_gnss_set_ss, gnss_set_ss, gnss cmd);

/* 获取星座状态 (最近一次搜星质量) 含时间戳 */
static int _gnss_get_ss(int argc, char **argv)
{
    if (argc > 1)
    {

    }
    
    return 0;
}
MSH_CMD_EXPORT_ALIAS(_gnss_get_ss, gnss_get_ss, gnss cmd);

/* 获取位置信息 (最近一次计算经纬度信息) 含时间戳 */
static int _gnss_get_loc(int argc, char **argv)
{
    if (argc > 1)
    {

    }
    
    return 0;
}
MSH_CMD_EXPORT_ALIAS(_gnss_get_loc, gnss_get_loc, gnss cmd);

/* 获取RTK信息 (最近一次计算) 含时间戳， 判断计算质量 */
static int _gnss_get_rtk(int argc, char **argv)
{
    if (argc > 1)
    {

    }
    
    return 0;
}
MSH_CMD_EXPORT_ALIAS(_gnss_get_rtk, gnss_get_rtk, gnss cmd);

/* 设置RTCM采用的网络 （LoRa，以太网，或4G MQTT） */
static int _gnss_set_net(int argc, char **argv)
{
    if (argc > 1)
    {

    }
    
    return 0;
}
MSH_CMD_EXPORT_ALIAS(_gnss_set_net, gnss_set_net, gnss cmd);

/* 在指定通道上发送，测试指令，输出基准站发送的 rctm 数据，测站此命令无效 */
static int _gnss_tx_net(int argc, char **argv)
{
    if (argc > 1)
    {

    }
    
    return 0;
}
MSH_CMD_EXPORT_ALIAS(_gnss_tx_net, gnss_tx_net, gnss cmd);

/* 在指定通道上接收，测试指令,输出测站接收到的 rctm 数据，基准站此命令无效 */
static int _gnss_rx_net(int argc, char **argv)
{
    if (argc > 1)
    {

    }
    
    return 0;
}
MSH_CMD_EXPORT_ALIAS(_gnss_rx_net, gnss_rx_net, gnss cmd);

/* 配置获取最近一次，GPA, GGA, RTCM原始数据， 含时间戳 */
static int _gnss_get_raw(int argc, char **argv)
{
    extern rt_uint8_t gpa_raw_data;
    extern rt_uint8_t gga_raw_data;
    extern rt_uint8_t rtcm_raw_data;
    if (1 == argc)
    {
        rt_kprintf("rtcm ram data:\r\n%s\r\n",rtcm_raw_data);
    }
    
    return 0;
}
MSH_CMD_EXPORT_ALIAS(_gnss_get_raw, gnss_get_raw, gnss cmd);

/* 休眠 GNSS 模块 */
static int _gnss_asleep(int argc, char **argv)
{
    if (1 == argc)
    {
        /* 关闭 Lora 电源 */
        //lora_power_off();
        /* 关闭 gnss 电源 */
				gnss_slave_mode_ready = false;
        gnss_power_off();
				
    }
    
    return 0;
}
MSH_CMD_EXPORT_ALIAS(_gnss_asleep, gnss_asleep, gnss cmd);

/* 唤醒 GNSS 模块 */
static int _gnss_wakeup(int argc, char **argv)
{
    if (1 == argc)
    {
        /* 打开 Lora 电源 */
        //lora_power_on();
        /* 打开 gnss 电源 */
        gnss_power_on();
				rt_thread_mdelay(5000);
				set_gnss_mode();
    }
    
    return 0;
}
MSH_CMD_EXPORT_ALIAS(_gnss_wakeup, gnss_wakeup, gnss cmd);

/********************* eth cmd ************************/

/* 设置以太网口IP地址，子网掩码 */
static int _eth_set_ip(int argc, char **argv)
{
    if (argc > 1)
    {
        chrIPAddress = argv[1];
        chrSubnetMask = argv[2];
        rt_kprintf("argv[1]: %s, argv[2]%s: \r\n", argv[1], argv[2]);
        set_if("e0", chrIPAddress, chrGateway, chrSubnetMask);
    }

    return 0;
}
MSH_CMD_EXPORT_ALIAS(_eth_set_ip, eth_set_ip, set the IP address and subnet mask of the Ethernet port);

/* 设置以太网网关地址 */
static int _eth_set_gw(int argc, char **argv)
{
    if (argc > 1)
    {
        rt_kprintf("argv[1]: %s,\r\n", argv[1]);
        chrGateway = argv[1];
        set_if("e0", chrIPAddress, chrGateway, chrSubnetMask);
    }

    return 0;
}
MSH_CMD_EXPORT_ALIAS(_eth_set_gw, eth_set_gw, set the Ethernet gateway address);

/* 配置TCP Server的IP，端口 */
static int _eth_cfg_tcp(int argc, char **argv)
{
    if (argc > 1)
    {

    }
    
    return 0;
}
MSH_CMD_EXPORT_ALIAS(_eth_cfg_tcp, eth_cfg_tcp, The IP address and port of the TCP Server are specified);

/* 在指定通道上发送，测试指令 */
static int _eth_tx_tcp(int argc, char **argv)
{
    if (argc > 1)
    {

    }
    
    return 0;
}
MSH_CMD_EXPORT_ALIAS(_eth_tx_tcp, eth_tx_tcp, Sends a test instruction on a specified channel);

/* 在指定通道上接收，测试指令 */
static int _eth_rx_tcp(int argc, char **argv)
{
    if (argc > 1)
    {

    }
    
    return 0;
}
MSH_CMD_EXPORT_ALIAS(_eth_rx_tcp, eth_rx_tcp, Receives and tests instructions on the specified channel);

extern int cofig_udp_remote_id_port(char *id, char* port);
/* 配置UDP Server的IP，端口 (本地端口 pc) remote */
static int _eth_cfg_udp(int argc, char **argv)
{
    if (argc > 1)
    {
        cofig_udp_remote_id_port(argv[1], argv[2]);
    }
    
    return 0;
}
MSH_CMD_EXPORT_ALIAS(_eth_cfg_udp, eth_cfg_udp, Configure the IP address and port of the UDP Server (local port));

/* 在指定通道上发送，测试指令 */
static int _eth_tx_udp(int argc, char **argv)
{
    if (argc > 1)
    {
				rt_sem_release(rtcm_data_sem);
    }
    
    return 0;
}
MSH_CMD_EXPORT_ALIAS(_eth_tx_udp, eth_tx_udp, Sends a test instruction on a specified channel);

/* 在指定通道上接收，测试指令 */
static int _eth_rx_udp(int argc, char **argv)
{
    if (argc > 1)
    {

    }
    
    return 0;
}
MSH_CMD_EXPORT_ALIAS(_eth_rx_udp, eth_rx_udp, Receives and tests instructions on the specified channel);

/********************* lora cmd ************************/
/* 设置Lora模块模式，主/从 */
static int _lora_set_mode(int argc, char **argv)
{
    if(!gnss_dev_status.pwr_lora)
    {
        rt_kprintf("The LORA module is powered off!\r\n");
        return 0;
    }

    if (argc > 1)
    {
        setMode(atoi(argv[1]));
    }
    
    return 0;
}
MSH_CMD_EXPORT_ALIAS(_lora_set_mode, lora_set_mode, lora cmd);

/* 设置Lora模块频点，SF */
static int _lora_set_channel(int argc, char **argv)
{
    if(!gnss_dev_status.pwr_lora)
    {
        rt_kprintf("The LORA module is powered off!\r\n");
        return 0;
    }

    if (argc > 1)
    {
        setChannel(atoi(argv[1]), true);
    }
    
    return 0;
}
MSH_CMD_EXPORT_ALIAS(_lora_set_channel, lora_set_channel, lora cmd);

/* 在指定频道上发送，测试指令 */
static int _lora_tx_channel(int argc, char **argv)
{
    if(!gnss_dev_status.pwr_lora)
    {
        rt_kprintf("The LORA module is powered off!\r\n");
        return 0;
    }

    if (argc > 1)
    {

    }
    
    return 0;
}
MSH_CMD_EXPORT_ALIAS(_lora_tx_channel, lora_tx_channel, lora cmd);

/* 在指定频道上接收，测试指令 */
static int _lora_rx_channel(int argc, char **argv)
{
    if(!gnss_dev_status.pwr_lora)
    {
        rt_kprintf("The LORA module is powered off!\r\n");
        return 0;
    }
    
    if (argc > 1)
    {

    }
    
    return 0;
}
MSH_CMD_EXPORT_ALIAS(_lora_rx_channel, lora_rx_channel, lora cmd);

/* 唤醒Lora模块 */
static int _lora_wakeup(int argc, char **argv)
{
    if (1 == argc)
    {
        /* 开启 lora 模块电源 */
        lora_power_on();
    }
    
    return 0;
}
MSH_CMD_EXPORT_ALIAS(_lora_wakeup, lora_wakeup, lora cmd);

/* 休眠Lora模块 */
static int _lora_asleep(int argc, char **argv)
{
    if (1 == argc)
    {
        /* 关闭 lora 模块电源 */
        lora_power_off();
    }
    
    return 0;
}
MSH_CMD_EXPORT_ALIAS(_lora_asleep, lora_asleep, lora cmd);

/********************* 4G Cat.1 cmd ************************/
/* 休眠4G模块 */
static int _cat1_asleep(int argc, char **argv)
{
    if (1 == argc)
    {

    }
    
    return 0;
}
MSH_CMD_EXPORT_ALIAS(_cat1_asleep, cat1_asleep, 4G net cmd);

/* 唤醒4G模块 */
static int _cat1_wakeup(int argc, char **argv)
{
    if (1 == argc)
    {

    }
    
    return 0;
}
MSH_CMD_EXPORT_ALIAS(_cat1_wakeup, cat1_wakeup, 4G net cmd);

/* 检查SIM卡状态 */
static int _cat1_simchk(int argc, char **argv)
{
    if (argc > 1)
    {

    }
    
    return 0;
}
MSH_CMD_EXPORT_ALIAS(_cat1_simchk, cat1_simchk, 4G net cmd);

/* 检查小区状态 */
static int _cat1_cellq(int argc, char **argv)
{
    if (argc > 1)
    {

    }
    
    return 0;
}
MSH_CMD_EXPORT_ALIAS(_cat1_cellq, cat1_cellq, 4G net cmd);

/* 4G模块联网注册 */
static int _cat1_register(int argc, char **argv)
{
    if (argc > 1)
    {

    }
    
    return 0;
}
MSH_CMD_EXPORT_ALIAS(_cat1_register, cat1_register, 4G net cmd);

/* 配置MQTT服务器地址，用户名，密码 */
static int _cat1_cfg_mqtt(int argc, char **argv)
{
    if (argc > 1)
    {

    }
    
    return 0;
}
MSH_CMD_EXPORT_ALIAS(_cat1_cfg_mqtt, cat1_cfg_mqtt, 4G net cmd);

/* 在指定mqtt的Topic上发送，测试指令 */
static int _cat1_pub_mqtt(int argc, char **argv)
{
    if (argc > 1)
    {

    }
    
    return 0;
}
MSH_CMD_EXPORT_ALIAS(_cat1_pub_mqtt, cat1_pub_mqtt, 4G net cmd);

/* 在指定mqtt的Topic上接收，测试指令 */
static int _cat1_sub_mqtt(int argc, char **argv)
{
    if (argc > 1)
    {

    }
    
    return 0;
}
MSH_CMD_EXPORT_ALIAS(_cat1_sub_mqtt, cat1_sub_mqtt, 4G net cmd);

/********************* imu cmd ************************/
/* 设置6轴加速度陀螺仪模式，采样频率, 滤波方式(数字滤波频率) */
static int _imu_set_mode(int argc, char **argv)
{
    if (3 == argc)
    {
        MPU_Set_Rate(atoi(argv[1]));
        MPU_Set_LPF(atoi(argv[2]));
    }
    
    return 0;
}
MSH_CMD_EXPORT_ALIAS(_imu_set_mode, imu_set_mode, set imu6050 mode);

/* IMU零偏校准 */
static int _imu_cfg_calib(int argc, char **argv)
{
    if (1 == argc)
    {

    }
    
    return 0;
}
MSH_CMD_EXPORT_ALIAS(_imu_cfg_calib, imu_cfg_calib, set zero offset calibration value);

/* 获取零偏校准值 */
static int _imu_get_calib(int argc, char **argv)
{
    if (1 == argc)
    {

    }
    
    return 0;
}
MSH_CMD_EXPORT_ALIAS(_imu_get_calib, imu_get_calib, set zero offset calibration value);

/* 启动测量 */
static int _imu_start(int argc, char **argv)
{
    if (1 == argc)
    {
        start_collect();
    }
    
    return 0;
}
MSH_CMD_EXPORT_ALIAS(_imu_start, imu_start, start measuring);

/* 停止测量 */
static int _imu_stop(int argc, char **argv)
{
    if (1 == argc)
    {
        stop_collect();
    }
    
    return 0;
}
MSH_CMD_EXPORT_ALIAS(_imu_stop, imu_stop, stop measurement);

/* 配置加速度，角速度阈值 */
static int _imu_cfg_thres(int argc, char **argv)
{
    if (3 == argc)
    {
        rt_uint8_t val = atoi(argv[1]);
        if(val >= 0 && val <= 3)
            MPU_Set_Gyro_Fsr(val);

        val = atoi(argv[2]);
        if(val >= 0 && val <= 3)
            MPU_Set_Accel_Fsr(val);       
    }
    
    return 0;
}
MSH_CMD_EXPORT_ALIAS(_imu_cfg_thres, imu_cfg_thres, set acceleration and angular velocity thresholds);

/* 获取加速度，角速度阈值 */
static int _imu_get_thres(int argc, char **argv)
{
    if (1 == argc)
    {
        rt_kprintf("Acceleration threshold: %d, Angular velocity threshold  %d\r\n", MPU_Get_Gyro_Fsr(), MPU_Get_Accel_Fsr());
    }
    
    return 0;
}
MSH_CMD_EXPORT_ALIAS(_imu_get_thres, imu_get_thres, get acceleration and angular velocity thresholds);

/********************* power management cmd ************************/
/* 获取当前整体供电状态 （外/内，电压），各模块唤醒，休眠状态 */
static int _pwr_get_status(int argc, char **argv)
{
    if (argc > 1)
    {

    }
    
    return 0;
}
MSH_CMD_EXPORT_ALIAS(_pwr_get_status, pwr_get_status, power management cmd);

/* 获取电池状态（有无电池，容量，充放电状态) */
static int _pwr_get_bat(int argc, char **argv)
{
    if (argc > 1)
    {

    }
    
    return 0;
}
MSH_CMD_EXPORT_ALIAS(_pwr_get_bat, pwr_get_bat, power management cmd);

/* 获取外接电源状态（电压，压降阈值) */
static int _pwr_get_dc(int argc, char **argv)
{
    if (argc > 1)
    {

    }
    
    return 0;
}
MSH_CMD_EXPORT_ALIAS(_pwr_get_dc, pwr_get_dc, power management cmd);

/* 配置电源->电池压降切换 阈值 */
static int _pwr_cfg_sw_down(int argc, char **argv)
{
    if (argc > 1)
    {

    }
    
    return 0;
}
MSH_CMD_EXPORT_ALIAS(_pwr_cfg_sw_down, pwr_cfg_sw_down, power management cmd);

/* 配置电池->电源压升切换 阈值，切换后转换为给电池充电 */
static int _pwr_cfg_sw_up(int argc, char **argv)
{
    if (argc > 1)
    {

    }
    
    return 0;
}
MSH_CMD_EXPORT_ALIAS(_pwr_cfg_sw_up, pwr_cfg_sw_up, power management cmd);

#endif
