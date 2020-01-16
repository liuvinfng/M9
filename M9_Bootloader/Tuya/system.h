/****************************************Copyright (c)*************************
**                               版权所有 (C), 2015-2020, 涂鸦科技
**
**                                 http://www.tuya.com
**
**--------------文件信息-------------------------------------------------------
**文   件   名: system.c
**描        述: wifi数据处理函数
**使 用 说 明 : 用户无需关心该文件实现内容
**
**
**--------------当前版本修订---------------------------------------------------
** 版  本: v2.5.3
** 日　期: 2019年11月26日
** 描　述: 1:增加红外状态通知
           2:增加红外进入收发产测
           3:修复串口数据接收函数uart_receive_input的BUG(不能判断出其中一种缓存区满的情况)
           4:修改串口数据缓存区的数组名称和长度宏定义的名称
           
** 版  本: v2.5.2
** 日　期: 2019年8月12日
** 描　述: 1:增加WiFi功能性测试（连接指定路由）
           2:简化流服务过程，用户只需要调用流服务传输接口即可使用流服务
           3:增加同步上报指令
           4:增加获取当前wifi联网状态指令
           5:增加获取格林时间指令
           6:优化ota升级流程，启动ota升级时用户可以选择传输包大小
           7:增加获取模块mac地址指令

** 版  本: v2.5.1
** 日　期: 2018年10月27日
** 描　述: 1:默认关闭流服务功能
           2:增加03协议wifi状态宏定义
		   3:更新与修改部分函数注释
		   
** 版  本: v2.5.0
** 日　期: 2018年4月18日
** 描　述: 1:协议版本改为0x03
           2:增加WIFI模组心跳关闭功能
           3:增加天气功能

** 版  本: v2.3.8
** 日　期: 2018年1月17日
** 描　述: 1:变量添加volatile防止编译器优化
           2:添加#error提示

** 版  本: v2.3.7
** 日　期: 2017年4月18日
** 描　述: 1:优化串口队列接收处理
		   
** 版  本: v2.3.6
** 日　期: 2016年7月21日
** 描　述: 1:修复获取本地时间错误
           2:添加hex_to_bcd转换函数
		   
** 版  本: v2.3.5
** 日　期: 2016年6月3日
** 描　述: 1:修改返回协议版本为0x01
           2:固件升级数据偏移量修改为4字节

** 版  本: v2.3.4
** 日　期: 2016年5月26日
** 描　述: 1:优化串口解析函数
           2:优化编译器兼容性,取消enum类型定义

** 版  本: v2.3.3
** 日　期: 2016年5月24日
** 描　述: 1:修改mcu获取本地时间函数
           2:添加wifi功能测试

** 版  本: v2.3.2
** 日　期: 2016年4月23日
** 描　述: 1:优化串口数据解析
           2:优化MCU固件升级流程
           3:优化上报流程

** 版  本: v2.3.1
** 日　期: 2016年4月15日
** 描　述: 1:优化串口数据解析

** 版  本: v2.3
** 日　期: 2016年4月14日
** 描　述: 1:支持MCU固件在线升级

** 版  本: v2.2
** 日　期: 2016年4月11日
** 描　述: 1:修改串口数据接收方式

** 版  本: v2.1
** 日　期: 2016年4月8日
** 描　述: 1:加入某些编译器不支持函数指针兼容选项

** 版  本: v2.0
** 日　期: 2016年3月29日
** 描　述: 1:优化代码结构
2:节省RAM空间
**
**-----------------------------------------------------------------------------
******************************************************************************/
#ifndef __SYSTEM_H_
#define __SYSTEM_H_

#ifdef SYSTEM_GLOBAL
  #define SYSTEM_EXTERN
#else
  #define SYSTEM_EXTERN   extern
#endif

//=============================================================================
//帧的字节顺序
//=============================================================================
#define         HEAD_FIRST                      0
#define         HEAD_SECOND                     1        
#define         PROTOCOL_VERSION                2
#define         FRAME_TYPE                      3
#define         LENGTH_HIGH                     4
#define         LENGTH_LOW                      5
#define         DATA_START                      6

//=============================================================================
//数据帧类型
//=============================================================================
#define         HEAT_BEAT_CMD                   0                               //心跳包
#define         PRODUCT_INFO_CMD                1                               //产品信息
#define         WORK_MODE_CMD                   2                               //查询MCU 设定的模块工作模式	
#define         WIFI_STATE_CMD                  3                               //wifi工作状态	
#define         WIFI_RESET_CMD                  4                               //重置wifi
#define         WIFI_MODE_CMD                   5                               //选择smartconfig/AP模式	
#define         DATA_QUERT_CMD                  6                               //命令下发
#define         STATE_UPLOAD_CMD                7                               //状态上报	 
#define         STATE_QUERY_CMD                 8                               //状态查询   
#define         UPDATE_START_CMD                0x0a                            //升级开始
#define         UPDATE_TRANS_CMD                0x0b                            //升级传输
#define         GET_ONLINE_TIME_CMD             0x0c                            //获取系统时间(格林威治时间)
#define         FACTORY_MODE_CMD                0x0d                            //进入产测模式    
#define         WIFI_TEST_CMD                   0x0e                            //wifi功能测试
#define         GET_LOCAL_TIME_CMD              0x1c                            //获取本地时间
#define         WEATHER_OPEN_CMD                0x20                            //打开天气          
#define         WEATHER_DATA_CMD                0x21                            //天气数据
#define         STATE_UPLOAD_SYN_CMD            0x22                            //状态上报（同步）
#define         STATE_UPLOAD_SYN_RECV_CMD       0x23                            //状态上报结果通知（同步）
#define         HEAT_BEAT_STOP                  0x25                            //关闭WIFI模组心跳
#define         OPEN_STREAM_TRANS_CMD           0x26                            //开启流服务
#define         START_STREAM_TRANS_CMD          0x27                            //开启流数据传输
#define         STREAM_TRANS_CMD                0x28                            //流数据传输
#define         STOP_STREAM_TRANS_CMD           0x29                            //结束流数据传输
#define         GET_WIFI_STATUS_CMD             0x2b                            //获取当前wifi联网状态
#define         WIFI_CONNECT_TEST_CMD           0x2c                            //wifi功能测试(连接指定路由)
#define         GET_MAC_CMD                     0x2d                            //获取模块mac
#define         GET_IR_STATUS_CMD               0x2e                            //红外状态通知
#define         IR_TX_RX_TEST_CMD               0x2f                            //红外进入收发产测



//=============================================================================
#define VERSION                 0x00                                            //协议版本号
#define PROTOCOL_HEAD           0x07                                            //固定协议头长度
#define FRAME_FIRST             0x55
#define FRAME_SECOND            0xaa
//============================================================================= 
SYSTEM_EXTERN volatile unsigned char wifi_data_process_buf[PROTOCOL_HEAD + WIFI_DATA_PROCESS_LMT];     //串口数据处理缓存
SYSTEM_EXTERN volatile unsigned char wifi_uart_rx_buf[PROTOCOL_HEAD + WIFI_UART_RECV_BUF_LMT];         //串口接收缓存
SYSTEM_EXTERN volatile unsigned char wifi_uart_tx_buf[PROTOCOL_HEAD + WIFIR_UART_SEND_BUF_LMT];        //串口发送缓存
//
SYSTEM_EXTERN volatile unsigned char *queue_in;
SYSTEM_EXTERN volatile unsigned char *queue_out;

SYSTEM_EXTERN volatile unsigned char stop_update_flag;                                                 //ENABLE:停止一切数据上传            DISABLE:恢复一切数据上传

#ifndef WIFI_CONTROL_SELF_MODE
SYSTEM_EXTERN volatile unsigned char reset_wifi_flag;                                                  //重置wifi标志(TRUE:成功/FALSE:失败)
SYSTEM_EXTERN volatile unsigned char set_wifimode_flag;                                                //设置WIFI工作模式标志(TRUE:成功/FALSE:失败)
SYSTEM_EXTERN volatile unsigned char wifi_work_state;                                                  //wifi模块当前工作状态
#endif

#ifdef WIFI_STREAM_ENABLE    
SYSTEM_EXTERN volatile char stream_status;                                                             //流服务发包返回状态
#endif

/*****************************************************************************
函数名称 : set_wifi_uart_byte
功能描述 : 写wifi_uart字节
输入参数 : dest:缓存区其实地址;
           byte:写入字节值
返回参数 : 写入完成后的总长度
*****************************************************************************/
unsigned short set_wifi_uart_byte(unsigned short dest, unsigned char byte);

/*****************************************************************************
函数名称 : set_wifi_uart_buffer
功能描述 : 写wifi_uart_buffer
输入参数 : dest:目标地址
           src:源地址
           len:数据长度
返回参数 : 无
*****************************************************************************/
unsigned short set_wifi_uart_buffer(unsigned short dest, unsigned char *src, unsigned short len);
/*****************************************************************************
函数名称 : wifi_uart_write_frame
功能描述 : 向wifi串口发送一帧数据
输入参数 : fr_type:帧类型
           len:数据长度
返回参数 : 无
*****************************************************************************/
void wifi_uart_write_frame(unsigned char fr_type, unsigned short len);

/*****************************************************************************
函数名称 : get_check_sum
功能描述 : 计算校验和
输入参数 : pack:数据源指针
           pack_len:计算校验和长度
返回参数 : 校验和
*****************************************************************************/
unsigned char get_check_sum(unsigned char *pack, unsigned short pack_len);

/*****************************************************************************
函数名称 : data_handle
功能描述 : 数据帧处理
输入参数 : offset:数据起始位
返回参数 : 无
*****************************************************************************/
void data_handle(unsigned short offset);

/*****************************************************************************
函数名称 : get_queue_total_data
功能描述 : 读取队列内数据
输入参数 : 无
返回参数 : 无
*****************************************************************************/
unsigned char get_queue_total_data(void);

/*****************************************************************************
函数名称 : Queue_Read_Byte
功能描述 : 读取队列1字节数据
输入参数 : 无
返回参数 : 无
*****************************************************************************/
unsigned char Queue_Read_Byte(void);

#endif
  
  
