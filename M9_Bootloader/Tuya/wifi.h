/****************************************Copyright (c)*************************
**                               版权所有 (C), 2015-2020, 涂鸦科技
**
**                                 http://www.tuya.com
**
**--------------文件信息-------------------------------------------------------
**文   件   名: wifi.h
**描        述: wifi文件头定义
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
** 日　期: 2019年7月5日
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
#ifndef __WIFI_H_
#define __WIFI_H_


//=============================================================================
/*定义常量*/
//=============================================================================
//#ifndef TRUE
//#define         TRUE                1
//#endif
////
//#ifndef FALSE
//#define         FALSE               0
//#endif
////
//#ifndef NULL
//#define         NULL                ((void *) 0)
//#endif

//#ifndef SUCCESS
//#define         SUCCESS             1
//#endif

//#ifndef ERROR
//#define         ERROR               0
//#endif

//#ifndef INVALID
//#define         INVALID             0xFF
//#endif

//#ifndef ENABLE
//#define         ENABLE                1
//#endif
////
//#ifndef DISABLE
//#define         DISABLE               0
//#endif
//=============================================================================
//dp数据点类型
//=============================================================================
#define         DP_TYPE_RAW                     0x00				//RAW型
#define         DP_TYPE_BOOL                    0x01	      //布尔型
#define         DP_TYPE_VALUE                   0x02	      //数值型
#define         DP_TYPE_STRING                  0x03				//字符串型
#define         DP_TYPE_ENUM                    0x04				//枚举型
#define         DP_TYPE_FAULT                   0x05				//故障型
#define         DP_TYPE_BITMAP                  0x06				

//=============================================================================
//WIFI工作状态
//=============================================================================
#define         SMART_CONFIG_STATE              0x00
#define         AP_STATE                        0x01
#define         WIFI_NOT_CONNECTED              0x02
#define         WIFI_CONNECTED                  0x03
#define					WIFI_CONN_CLOUD					0x04
#define					WIFI_LOW_POWER					0x05
#define         WIFI_SATE_UNKNOW                0xff
//=============================================================================
//wifi配网的方式
//=============================================================================
#define         SMART_CONFIG                    0x0  
#define         AP_CONFIG                       0x1   

//=============================================================================
//wifi复位状态
//=============================================================================
#define         RESET_WIFI_ERROR                0
#define         RESET_WIFI_SUCCESS              1

//=============================================================================
//wifi复位状态
//=============================================================================
#define         SET_WIFICONFIG_ERROR            0
#define         SET_WIFICONFIG_SUCCESS          1

//=============================================================================
//MCU固件升级状态
//=============================================================================
#define         FIRM_STATE_UN_SUPPORT           0x00                            //不支持 MCU 升级
#define         FIRM_STATE_WIFI_UN_READY        0x01                            //模块未就绪
#define         FIRM_STATE_GET_ERROR            0x02                            //云端升级信息查询失败
#define         FIRM_STATE_NO                   0x03                            //无需升级（云端无更新版本）
#define         FIRM_STATE_START                0x04                            //需升级，等待模块发起升级操作

//=============================================================================
//WIFI和mcu的工作方式 
//=============================================================================
#define         UNION_WORK                      0x0                             //mcu模块与wifi配合处理
#define         WIFI_ALONE                      0x1                             //wifi模块自处理

//=============================================================================
//系统工作模式
//=============================================================================
#define         NORMAL_MODE             0x00                                    //正常工作状态
#define         FACTORY_MODE            0x01                                    //工厂模式	
#define         UPDATE_MODE             0X02                                    //升级模式	 

//=============================================================================
//配网方式选择
//=============================================================================
#define         CONFIG_MODE_DEFAULT     "0"                                    //默认配网方式
#define         CONFIG_MODE_LOWPOWER    "1"                                    //低功耗配网方式
#define         CONFIG_MODE_SPECIAL     "2"                                    //特殊配网方式    

#include "protocol.h"
#include "system.h"
#include "mcu_api.h"
#include "stdint.h"

//=============================================================================
//下发命令
//=============================================================================
typedef struct {
  unsigned char dp_id;                        //dp序号
  unsigned char dp_type;                          //dp类型
} DOWNLOAD_CMD_S;

typedef struct 
{
	volatile uint8_t Ty_PowerSwitch;
	volatile uint8_t Ty_ErrorCode;
	volatile uint8_t Ty_CleanMode;
	volatile uint8_t Ty_RemoteDir;
	volatile uint8_t Ty_VacMode;
	volatile uint8_t Ty_CleanSwitch;
	volatile uint8_t Ty_CurrentState;		
	volatile uint8_t Ty_DumpEnergy;
	const unsigned char *Ty_CleanBuf;
	volatile uint8_t Ty_CleanRecord;
	volatile uint16_t Ty_CleanArea;
	volatile uint16_t Ty_CleanTime;	
	volatile uint8_t Ty_Volume;
	volatile uint8_t Ty_Hypa;
	volatile uint8_t Ty_Filter;
	volatile uint8_t Ty_Mop;	
	volatile uint8_t Ty_EdgeBrush;	
	volatile uint8_t Ty_RollBrush;	
	volatile uint8_t Ty_SweepMop;	
	volatile uint8_t Ty_WifiState;	
}ParameterStruct;
extern  ParameterStruct  SysConfig;

#endif
