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
		   4:流服务功能

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
#define SYSTEM_GLOBAL

#include "wifi.h"
#include "wifi2.h"
#include "debug.h"

void Flash_Erase(unsigned long datasize);

extern const DOWNLOAD_CMD_S download_cmd[];
unsigned short firm_size;                                                      //升级包一包的大小

/*****************************************************************************
函数名称 : set_wifi_uart_byte
功能描述 : 写wifi_uart字节
输入参数 : dest:缓存区其实地址;
           byte:写入字节值
返回参数 : 写入完成后的总长度
*****************************************************************************/
unsigned short set_wifi_uart_byte(unsigned short dest, unsigned char byte)
{
  unsigned char *obj = (unsigned char *)wifi_uart_tx_buf + DATA_START + dest;
  
  *obj = byte;
  dest += 1;
  
  return dest;
}
/*****************************************************************************
函数名称 : set_wifi_uart_buffer
功能描述 : 写wifi_uart_buffer
输入参数 : dest:目标地址
           src:源地址
           len:数据长度
返回参数 : 无
*****************************************************************************/
unsigned short set_wifi_uart_buffer(unsigned short dest, unsigned char *src, unsigned short len)
{
  unsigned char *obj = (unsigned char *)wifi_uart_tx_buf + DATA_START + dest;
  
  my_memcpy(obj,src,len);
  
  dest += len;
  return dest;
}

/*****************************************************************************
函数名称 : wifi_uart_write_data
功能描述 : 向wifi uart写入连续数据
输入参数 : in:发送缓存指针
           len:数据发送长度
返回参数 : 无
*****************************************************************************/
static void wifi_uart_write_data(unsigned char *in, unsigned short len)
{
  if((NULL == in) || (0 == len))
  {
    return;
  }
  
  while(len --)
  {
    uart_transmit_output(*in);
    in ++;
  }
}
/*****************************************************************************
函数名称 : get_check_sum
功能描述 : 计算校验和
输入参数 : pack:数据源指针
           pack_len:计算校验和长度
返回参数 : 校验和
*****************************************************************************/
unsigned char get_check_sum(unsigned char *pack, unsigned short pack_len)
{
  unsigned short i;
  unsigned char check_sum = 0;
  
  for(i = 0; i < pack_len; i ++)
  {
    check_sum += *pack ++;
  }
  
  return check_sum;
}
/*****************************************************************************
函数名称 : wifi_uart_write_frame
功能描述 : 向wifi串口发送一帧数据
输入参数 : fr_type:帧类型
           len:数据长度
返回参数 : 无
*****************************************************************************/
void wifi_uart_write_frame(unsigned char fr_type, unsigned short len)
{
  unsigned char check_sum = 0;
  
  wifi_uart_tx_buf[HEAD_FIRST] = 0x55;
  wifi_uart_tx_buf[HEAD_SECOND] = 0xaa;
  wifi_uart_tx_buf[PROTOCOL_VERSION] = 0x03;
  wifi_uart_tx_buf[FRAME_TYPE] = fr_type;
  wifi_uart_tx_buf[LENGTH_HIGH] = len >> 8;
  wifi_uart_tx_buf[LENGTH_LOW] = len & 0xff;
  
  len += PROTOCOL_HEAD;
  check_sum = get_check_sum((unsigned char *)wifi_uart_tx_buf, len - 1);
  wifi_uart_tx_buf[len - 1] = check_sum;
  //
  wifi_uart_write_data((unsigned char *)wifi_uart_tx_buf, len);
}
/*****************************************************************************
函数名称 : heat_beat_check
功能描述 : 心跳包检测
输入参数 : 无
返回参数 : 无
*****************************************************************************/
static void heat_beat_check(void)
{
  unsigned char length = 0;
  static unsigned char mcu_reset_state = FALSE;
  
  if(FALSE == mcu_reset_state)
  {
    length = set_wifi_uart_byte(length,FALSE);
    mcu_reset_state = TRUE;
  }
  else
  {
    length = set_wifi_uart_byte(length,TRUE);
  }
  
  wifi_uart_write_frame(HEAT_BEAT_CMD, length);
}
/*****************************************************************************
函数名称  : product_info_update
功能描述  : 产品信息上传
输入参数 : 无
返回参数 : 无
*****************************************************************************/
static void product_info_update(void)
{
  unsigned char length = 0;
  
  length = set_wifi_uart_buffer(length, "{\"p\":\"", my_strlen("{\"p\":\""));
  length = set_wifi_uart_buffer(length,(unsigned char *)PRODUCT_KEY,my_strlen((unsigned char *)PRODUCT_KEY));
  length = set_wifi_uart_buffer(length, "\",\"v\":\"", my_strlen("\",\"v\":\""));
  length = set_wifi_uart_buffer(length,(unsigned char *)MCU_VER,my_strlen((unsigned char *)MCU_VER));
  length = set_wifi_uart_buffer(length, "\",\"m\":", my_strlen("\",\"m\":"));
  length = set_wifi_uart_buffer(length, (unsigned char *)CONFIG_MODE, my_strlen((unsigned char *)CONFIG_MODE));
  length = set_wifi_uart_buffer(length, "}", my_strlen("}"));
  
  wifi_uart_write_frame(PRODUCT_INFO_CMD, length);
}
/*****************************************************************************
函数名称 : get_mcu_wifi_mode
功能描述 : 查询mcu和wifi的工作模式
输入参数 : 无
返回参数 : 无
*****************************************************************************/
static void get_mcu_wifi_mode(void)
{
  unsigned char length = 0;
  
#ifdef WIFI_CONTROL_SELF_MODE                                   //模块自处理
  length = set_wifi_uart_byte(length, WF_STATE_KEY);
  length = set_wifi_uart_byte(length, WF_RESERT_KEY);
#else                                                           
  //无需处理数据
#endif
  
  wifi_uart_write_frame(WORK_MODE_CMD, length);
}
/*****************************************************************************
函数名称 : get_update_dpid_index
功能描述 : 获取制定DPID在数组中的序号
输入参数 : dpid:dpid
返回参数 : index:dp序号
*****************************************************************************/
static unsigned char get_dowmload_dpid_index(unsigned char dpid)
{
  unsigned char index;
  unsigned char total = get_download_cmd_total();
  
  for(index = 0; index < total; index ++)
  {
    if(download_cmd[index].dp_id == dpid)
    {
      break;
    }
  }
  
  return index;
}
/*****************************************************************************
函数名称 : data_point_handle
功能描述 : 下发数据处理
输入参数 : value:下发数据源指针
返回参数 : ret:返回数据处理结果
*****************************************************************************/
static unsigned char data_point_handle(const unsigned char value[])
{
  unsigned char dp_id,index;
  unsigned char dp_type;
  unsigned char ret;
  unsigned short dp_len;
  
  dp_id = value[0];
  dp_type = value[1];
  dp_len = value[2] * 0x100;
  dp_len += value[3];
  
  index = get_dowmload_dpid_index(dp_id);

  if(dp_type != download_cmd[index].dp_type)
  {
    //错误提示
    return FALSE;
  }
  else
  {
    ret = dp_download_handle(dp_id,value + 4,dp_len);
  }
  
  return ret;
}

#ifdef WEATHER_ENABLE
/*****************************************************************************
函数名称 : data_point_handle
功能描述 : 下发数据处理
输入参数 : value:下发数据源指针
返回参数 : ret:返回数据处理结果
*****************************************************************************/
void mcu_open_weather(void)
{
  int i = 0;
  char temp[10], buffer[13] = "xw.xxxxxxxxxx";
  char result[WEATHER_CHOOSE_CNT * 13 + 8];
  int buf_len = 0, now_buf_len = 0, last_buf_len = 0;
  int weather_len = 0;
  unsigned char check_sum = 0;
  
  weather_len = sizeof(weather_choose);
  weather_len = sizeof(weather_choose) / 10;
  
  for(i=0;i<weather_len;i++)
  {
    my_memcpy(temp, weather_choose[i], strlen(weather_choose[i]));
    my_memcpy(buffer + 3, temp, strlen(weather_choose[i]));
    buffer[0] = strlen(weather_choose[i]) + 2;
    buf_len = strlen(weather_choose[i]) + 3;
    now_buf_len += buf_len;
    my_memcpy(result + DATA_START + last_buf_len, buffer, buf_len);
    last_buf_len = now_buf_len;
  }
  
  result[HEAD_FIRST] = 0x55;
  result[HEAD_SECOND] = 0xaa;
  result[PROTOCOL_VERSION] = 0x03;
  result[FRAME_TYPE] = 0x20;
  result[LENGTH_HIGH] = now_buf_len >> 8;
  result[LENGTH_LOW] = now_buf_len & 0xff;
  
  now_buf_len += PROTOCOL_HEAD;
  check_sum = get_check_sum((unsigned char *)result, now_buf_len - 1);
  result[now_buf_len - 1] = check_sum;
  
  wifi_uart_write_data((unsigned char *)result, now_buf_len);
}

/*****************************************************************************
函数名称 : weather_data_raw_handle
功能描述 : 天气数据解析
输入参数 : buffer:接收数据指针
返回参数 : 无
*****************************************************************************/
void weather_data_raw_handle(char *buffer)
{
  char return_send_Weather[7] = { 0x55, 0xaa, 0x00, 0x21, 0x00, 0x00, 0x20 };
  int length = buffer[LENGTH_HIGH] * 256 + buffer[LENGTH_LOW];
  int i = 7;
  int can_len = 0; 
  char can[15] = "";
  int type1 = 0;
  char value_string[100] = "";
  int val_cnt = 0;
  int val_len = 0;
  
  if(buffer[DATA_START] != 1 || length < 1)
  {
    //接收失败
  }
  else
  {
    if(length < 4)
    {
      //数据为空
    }
    
    while (i < length + 7 - 1)
    {
      can_len = buffer[i];
      my_memset(can, '\0', 15);
      my_memcpy(can, buffer + i + 1, can_len);

      type1 = buffer[i + 1 + can_len];
      if(type1 != 0 && type1 != 1)
      {
        return;
      }

      my_memset(value_string, '\0', 100);
      val_cnt = i + 1 + can_len + 1 + 1 - 1;
      val_len = buffer[val_cnt];
      if (type1 == 0)
      {//int32
        weather_data_user_handle(can+2, type1, buffer+val_cnt+1);
      }
      else if(type1 == 1)
      {//string
        my_memcpy(value_string, buffer + val_cnt + 1, val_len);
        weather_data_user_handle(can+2, type1, value_string);
      }

      i += 1 + can_len + 1 + val_len + 1;
    }
    
    my_memcpy((unsigned char *)wifi_uart_tx_buf, return_send_Weather, 7);
    wifi_uart_write_data((unsigned char *)wifi_uart_tx_buf, 7);
  }
}
#endif

/*****************************************************************************
函数名称 : data_handle
功能描述 : 数据帧处理
输入参数 : offset:数据起始位
返回参数 : 无
*****************************************************************************/
void data_handle(unsigned short offset)
{
#ifdef SUPPORT_MCU_FIRM_UPDATE
  unsigned char *firmware_addr;
  static unsigned long firm_length;                                             //MCU升级文件长度
  static unsigned char firm_update_flag;                                        //MCU升级标志
  unsigned long dp_len;
  unsigned char firm_flag;                                                      //升级包大小标志
#else
  unsigned short dp_len;
#endif
  
  unsigned char ret;
  unsigned short i,total_len;
  unsigned char cmd_type = wifi_data_process_buf[offset + FRAME_TYPE];

#ifdef WEATHER_ENABLE
  static unsigned char isWoSend = 0;                    //是否已经打开过天气数据,0:是;1:否
#endif

  unsigned char result;
#ifdef WIFI_TEST_ENABLE
  unsigned char rssi;
#endif
  
  switch(cmd_type)
  {
  case HEAT_BEAT_CMD:                                   //心跳包
    heat_beat_check();
    break;
    
  case PRODUCT_INFO_CMD:                                //产品信息
    product_info_update();
    break;
    
  case WORK_MODE_CMD:                                   //查询MCU设定的模块工作模式
    get_mcu_wifi_mode();
    break;
    
#ifndef WIFI_CONTROL_SELF_MODE
  case WIFI_STATE_CMD:                                  //wifi工作状态	
    wifi_work_state = wifi_data_process_buf[offset + DATA_START];
    get_wifi_status(wifi_work_state);//edit by vin
	  wifi_uart_write_frame(WIFI_STATE_CMD,0);
#ifdef WEATHER_ENABLE
    if(wifi_work_state == WIFI_CONNECTED && isWoSend == 0)   //当WIFI连接成功，打开天气数据且仅一次
    {
      mcu_open_weather();
      isWoSend = 1;
    }
#endif
    break;

  case WIFI_RESET_CMD:                                  //重置wifi(wifi返回成功)
    reset_wifi_flag = RESET_WIFI_SUCCESS;
    break;
    
  case WIFI_MODE_CMD:                                   //选择smartconfig/AP模式(wifi返回成功)	
    set_wifimode_flag = SET_WIFICONFIG_SUCCESS;
    break;
#endif
    
  case DATA_QUERT_CMD:                                  //命令下发
    total_len = wifi_data_process_buf[offset + LENGTH_HIGH] * 0x100;
    total_len += wifi_data_process_buf[offset + LENGTH_LOW];
    
    for(i = 0;i < total_len;)
    {
      dp_len = wifi_data_process_buf[offset + DATA_START + i + 2] * 0x100;
      dp_len += wifi_data_process_buf[offset + DATA_START + i + 3];
      //
      ret = data_point_handle((unsigned char *)wifi_data_process_buf + offset + DATA_START + i);
      
      if(SUCCESS == ret)
      {
        //成功提示
      }
      else
      {
        //错误提示
      }
      
      i += (dp_len + 4);
    }
    
    break;
    
  case STATE_QUERY_CMD:                                 //状态查询
    all_data_update();                               
    break;
    
#ifdef SUPPORT_MCU_FIRM_UPDATE
  case UPDATE_START_CMD:                                //升级开始
    //获取升级包大小全局变量
    firm_flag = PACKAGE_SIZE;
    if(firm_flag == 0) {
      firm_size = 256;
    }else if(firm_flag == 1) {
      firm_size = 512;
    }else if(firm_flag == 2) { 
      firm_size = 1024;
    }

    firm_length = wifi_data_process_buf[offset + DATA_START];
    firm_length <<= 8;
    firm_length |= wifi_data_process_buf[offset + DATA_START + 1];
    firm_length <<= 8;
    firm_length |= wifi_data_process_buf[offset + DATA_START + 2];
    firm_length <<= 8;
    firm_length |= wifi_data_process_buf[offset + DATA_START + 3];
    
//    upgrade_package_choose(PACKAGE_SIZE); 
		Wifi_Package_Choose();//edit by vin
    firm_update_flag = UPDATE_START_CMD;
     break;
    
  case UPDATE_TRANS_CMD:                                //升级传输
    if(firm_update_flag == UPDATE_START_CMD)
    {
      //停止一切数据上报
      stop_update_flag = ENABLE;
      
      total_len = wifi_data_process_buf[offset + LENGTH_HIGH] * 0x100;
      total_len += wifi_data_process_buf[offset + LENGTH_LOW];
      
      dp_len = wifi_data_process_buf[offset + DATA_START];
      dp_len <<= 8;
      dp_len |= wifi_data_process_buf[offset + DATA_START + 1];
      dp_len <<= 8;
      dp_len |= wifi_data_process_buf[offset + DATA_START + 2];
      dp_len <<= 8;
      dp_len |= wifi_data_process_buf[offset + DATA_START + 3];
      
      firmware_addr = (unsigned char *)wifi_data_process_buf;
      firmware_addr += (offset + DATA_START + 4);
      
      if((total_len == 4) && (dp_len == firm_length))
      {
        //最后一包
        ret = mcu_firm_update_handle(firmware_addr,dp_len,0);
        
        firm_update_flag = 0;
      }
      else if((total_len - 4) <= firm_size)//firm_size定义为256字节
      {
        ret = mcu_firm_update_handle(firmware_addr,dp_len,total_len - 4);
      }
      else
      {
        firm_update_flag = 0;
        ret = ERROR;
      }
      
      if(ret == SUCCESS)
      {
//        wifi_uart_write_frame(UPDATE_TRANS_CMD,0);
				Wifi_Rece_Package_Report();//edit by vin
      }
      //恢复一切数据上报
      stop_update_flag = DISABLE;    
    }
    break;
#endif      

#ifdef SUPPORT_GREEN_TIME
  case GET_ONLINE_TIME_CMD:                              //获取格林时间
    mcu_get_greentime((unsigned char *)(wifi_data_process_buf + offset + DATA_START));
  break;
#endif

#ifdef SUPPORT_MCU_RTC_CHECK
  case GET_LOCAL_TIME_CMD:                             //获取本地时间
      mcu_write_rtctime((unsigned char *)(wifi_data_process_buf + offset + DATA_START));
    break;
#endif
 
#ifdef WIFI_TEST_ENABLE
  case WIFI_TEST_CMD:                                   //wifi功能测试（扫描指定路由）
    result = wifi_data_process_buf[offset + DATA_START];
    rssi = wifi_data_process_buf[offset + DATA_START + 1];
    wifi_test_result(result, rssi);
    break;
#endif

#ifdef WEATHER_ENABLE
  case WEATHER_OPEN_CMD:
    weather_open_return_handle(wifi_data_process_buf[offset + DATA_START], wifi_data_process_buf[offset + DATA_START + 1]);
    
    break;
    
  case WEATHER_DATA_CMD:
    weather_data_raw_handle((char*)wifi_data_process_buf + offset);
    break;
#endif

#ifdef WIFI_STREAM_ENABLE
  case STREAM_TRANS_CMD:
    stream_status = wifi_data_process_buf[offset + DATA_START];//流服务传输返回接收
    break;
  
#endif

#ifdef WIFI_CONNECT_TEST_ENABLE
      case WIFI_CONNECT_TEST_CMD:                           //wifi功能测试（连接指定路由）
        result = wifi_data_process_buf[offset + DATA_START];
        wifi_connect_test_result(result);
        break;
#endif

#ifdef GET_MODULE_MAC_ENABLE
      case GET_MAC_CMD:                                     //获取模块mac
        mcu_get_mac((unsigned char *)(wifi_data_process_buf + offset + DATA_START));
      break;
#endif

#ifdef GET_WIFI_STATUS_ENABLE
      case GET_WIFI_STATUS_CMD:                             //获取当前wifi联网状态
        result = wifi_data_process_buf[offset + DATA_START];
        get_wifi_status(result);
      break;

#endif

#ifdef MCU_DP_UPLOAD_SYN
      case STATE_UPLOAD_SYN_RECV_CMD:                             //状态上报（同步）
        result = wifi_data_process_buf[offset + DATA_START];
        get_upload_syn_result(result);
      break;
      
#endif

#ifdef GET_IR_STATUS_ENABLE
      case GET_IR_STATUS_CMD:                             //红外状态通知
        result = wifi_data_process_buf[offset + DATA_START];
        get_ir_status(result);
      break;
      
#endif
      
#ifdef IR_TX_RX_TEST_ENABLE
      case IR_TX_RX_TEST_CMD:                             //红外进入收发产测
        result = wifi_data_process_buf[offset + DATA_START];
        ir_tx_rx_test_result(result);
      break;
      
#endif
      
  default:
    break;
  }
}
/*****************************************************************************
函数名称 : get_queue_total_data
功能描述 : 读取队列内数据
输入参数 : 无
返回参数 : 无
*****************************************************************************/
unsigned char get_queue_total_data(void)
{
  if(queue_in != queue_out)
    return 1;
  else
    return 0;
}
/*****************************************************************************
函数名称 : Queue_Read_Byte
功能描述 : 读取队列1字节数据
输入参数 : 无
返回参数 : 无
*****************************************************************************/
unsigned char Queue_Read_Byte(void)
{
  unsigned char value;
  
  if(queue_out != queue_in)
  {
    //有数据
    if(queue_out >= (unsigned char *)(wifi_uart_rx_buf + sizeof(wifi_uart_rx_buf)))
    {
      //数据已经到末尾
      queue_out = (unsigned char *)(wifi_uart_rx_buf);
    }
    
    value = *queue_out ++;   
  }
  
  return value;
}



