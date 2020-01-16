#include "charge.h"
#include "Usart_IAP.h"
#include "display.h"

#define START_ADDRESS		    DOWN_LOAD_ADDRESS

char Send_String_Buffer[TXBUF_MAX];
AC_TxPackStruct_t TxPackage;


pFunction Jump_To_Application;
uint32_t JumpAddress;


volatile uint8_t Rec_Data_Temp[RXBUF_MAX] = {0};//���յ����ݻ���
uint8_t Rec_Data[RXBUF_MAX] = {0};//���յ�����
volatile uint8_t Rec_Data_Length = 0;//��Ҫ���յ����ݳ���
volatile uint8_t Rec_Data_Flag = 0;//���ݽ���״̬��0-�ȴ����� 1-���ڽ��� 2-���ղ�У��ͨ�� 3-����ʧ��
volatile uint8_t Rec_Data_Step = 0;
volatile uint8_t Cnt = 0;//�ѽ��յ����ݳ���
volatile uint8_t Sum = 0;//�ۼӺ�
volatile uint8_t Send_Data_Sum = 0;
volatile uint16_t Package_Num = 1;
volatile uint16_t Package_Temp = 0;
volatile uint16_t Total_Pack = 0;
		
void Check_for_IAP(void)
{
	uint8_t i;
	uint32_t addr=0,buf=0;
	if(Get_Upgrade_Flag() == 0x55)
	{
//		SystemInitialize();			
//		delay(300);
//		Enable_PPower();
//		delay(5000);
		
		/////����flash 100KB
		fmc_unlock();
		for(i=0; i<100; i++)
		{
			fmc_page_erase(APPLICATION_ADDRESS+(i*1024));
		}
		fmc_lock();						

		for(addr=0;addr<102400;addr+=4)//copy flash 100KB
		{
			buf=FLASH_ReadU32(DOWN_LOAD_ADDRESS+addr);
			Flash_WriteU32(APPLICATION_ADDRESS+addr,buf);		
		}
		delay(5000);
		BootLoader_Jump(APPLICATION_ADDRESS);
		while(1)
		{
			//��תʧ��
			Set_Upgrade_Flag(0xaa);//����Ϊ����ģʽ
			BootLoader_Jump(Bootloader_ADDRESS);
		}		
	}
	else if(Get_Upgrade_Flag() == 0xAA)//���FLASH�е�������־λ
	{
//		SystemInitialize();			
//		delay(300);
//		Enable_PPower();
//		delay(5000);
		Set_LED_On_Blink(1,1,1,1,1,1);
		Set_LED_On_Switch(0,0,0,0,0,0);		
    Speaker(ROBOT_IS_START);	
		delay(10000);
		while(1)
		{
			if(Is_ChargerOn())
			{
				Charge_SetSwitch(ENABLE);
				Set_LED_On_Blink(0,0,0,0,0,0);
				Set_LED_On_Switch(0,0,0,1,1,0);	
				Set_LED_Table(0,0,0,g_breathe_led,g_breathe_led,0);
			}
			else				
			{
				Charge_SetSwitch(DISABLE);
				Set_LED_On_Blink(1,1,1,1,1,1);
				Set_LED_On_Switch(0,0,0,0,0,0);					
			}			
			Charge_Process();
			
			if(Rec_Data_Flag == 2)//���յ�����
			{
				//Analysis_Data();
				AC_Decode_Command(&Rec_Data[1],Rec_Data_Length);
				Rec_Data_Flag = 0;
				if(g_time_10ms>1000)
				{
					g_time_10ms=0;
					Speaker(SPK_SYS_UPDATE);	
				}			
			}
			if(Rec_Data_Flag == 3)
			{
				Rec_Data_Flag = 0;
			}
		}
	}
	else//��������״̬����ת���û�����ִ��
	{
		BootLoader_Jump(APPLICATION_ADDRESS);
		while(1)
		{
			//��תʧ��
			Set_Upgrade_Flag(0xaa);//����Ϊ����ģʽ
			BootLoader_Jump(Bootloader_ADDRESS);
		}
	}
}	
				

//��ת����ָ���ĵ�ַ����
void BootLoader_Jump(uint32_t addr)
{
	if (((*(__IO uint32_t*)addr) & 0x2FFE0000 ) == 0x20000000)
	{ 
		/* Jump to user application */
		JumpAddress = *(__IO uint32_t*) (addr + 4);
		Jump_To_Application = (pFunction) JumpAddress;
		
		/* Initialize user application's Stack Pointer */
		__set_MSP(*(__IO uint32_t*) addr);
		
		/* Jump to application */
		Jump_To_Application();
	}
}

//�����ж�������
void IAP_Set(uint32_t addr)
{
  uint32_t i = 0;
      
  for(i = 0; i < 48; i++)
  {
    *((uint32_t*)(0x20000000 + (i << 2)))=*(__IO uint32_t*)(addr + (i<<2));
	}
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE); 	
//  SYSCFG_MemoryRemapConfig(SYSCFG_MemoryRemap_SRAM);
}

//��ȡFLASH�е�����״̬��־λ
uint16_t Get_Upgrade_Flag(void)
{
	return FLASH_ReadU16(Upgrade_Flag_Address);	
}
//����FLASH�е�����״̬��־λΪ����ģʽ
void Set_Upgrade_Flag(uint16_t flag)
{
	fmc_unlock();
	fmc_page_erase(Upgrade_Flag_Address);
	Flash_WriteU16(Upgrade_Flag_Address, flag);
	fmc_lock();
}

//�����ļ���С������flash�ռ�
void FLASH_EraseALL(unsigned long datasize)//���뵥λ���ֽ�
{
	uint8_t i;
	uint8_t page = datasize/1024 + 1;
	fmc_unlock();
	for(i=0; i<page; i++)
	{
		fmc_page_erase(START_ADDRESS+(i*1024));
	}
	fmc_lock();
}

///////////////////////////////1616161616616161616166161616////////////////////////////////
//��flash��ָ����ַд16λ����
void Flash_WriteU16(uint32_t addr, uint16_t data)
{
	fmc_unlock();
	fmc_halfword_program(addr, data);
	fmc_lock();
}
//��ȡaddr��ַ�е�16λ����
uint16_t FLASH_ReadU16(uint32_t addr)
{
	return *(uint16_t *)addr;
}

///////////////////////////////3232323232323232323323232323////////////////////////////////
//��flash��ָ����ַд16λ����
void Flash_WriteU32(uint32_t addr, uint32_t data)
{
	fmc_unlock();
	fmc_word_program(addr, data);
	fmc_lock();
}

uint32_t FLASH_ReadU32(uint32_t addr)
{
	return *(uint32_t *)addr;
}





void Clear_Rec_Flags(void)
{
	Rec_Data_Temp[0] = 0;//��Ҫ���յ�����
	Rec_Data_Length = 0;//��Ҫ���յ����ݳ���
	Rec_Data_Step = 0;
	Cnt = 0;//�ѽ��յ����ݳ���
	Sum = 0;//�ۼӺ�
}

void Save_Rec_Data(uint8_t data)
{
	uint8_t i;
	if(Rec_Data_Step == 0)
	{
		Clear_Rec_Flags();
		if(data == 0x5A)//֡ͷ
		{			
			Rec_Data_Flag = 1;
			Rec_Data_Temp[0] = data;
			Sum = 0;
			Rec_Data_Step++;
		}		
	}
	else if(Rec_Data_Step == 1)//���ݳ���hight
	{
		Rec_Data_Length = data<<8;		
		Rec_Data_Temp[1] = data;		
		Sum += data;
		Rec_Data_Step++;
	}
	else if(Rec_Data_Step == 2)//���ݳ���low
	{
		Rec_Data_Length += data;
		Rec_Data_Temp[2] = data;
		Sum += data;
		Cnt=3;
		Rec_Data_Step++;		
	}
	else if(Rec_Data_Step == 3)//����
	{
		if(Cnt < (Rec_Data_Length-2))//���ݻ�û�н�����
		{
			Rec_Data_Temp[Cnt] = data;			
			Sum += data;
			Cnt++;			
		}
		else Rec_Data_Step++;
	}
	if(Rec_Data_Step == 4)//���ݽ������
	{
		if(Sum == data)
		{
			Rec_Data_Temp[Cnt] = data;
			for(i = 0; i <= Rec_Data_Length; i++)
			{
				Rec_Data[i] = Rec_Data_Temp[i];				
			}
			Clear_Rec_Flags();
			Rec_Data_Flag = 2;
		}
		else
		{
			Clear_Rec_Flags();
			Rec_Data_Flag = 3;
		}
	}
}

//��flash������д��64�ֽ����ݣ�pack_num ���ݵİ����
void WriteData(uint16_t pack_num,volatile uint8_t *rec_data)
{
	uint16_t i;
	uint16_t t = 0;
	uint16_t data_temp = 0;
	uint32_t addr = 0;
	
	pack_num -=1;
	addr = START_ADDRESS + pack_num * 64;
	
	data_temp = rec_data[6] + (rec_data[7] << 8);
	fmc_unlock();
	for(i=0; i<32; i++)
	{
		data_temp = rec_data[6 + t] + (rec_data[7 + t] << 8);
		
		fmc_halfword_program(addr, data_temp);
		
		addr += 2;
		t += 2;
	}
	fmc_lock();
}


uint8_t Calc_PackageSum(char *data_buf,uint16_t length)
{
	uint16_t idx = 0;
	uint8_t sum = 0 ;
	
	for(idx = 0;idx < length; idx++)
	{
		sum += *(data_buf +idx);
	}
	return sum;
}

void AC_DecodeRx_Buf(Ac_RxPackStruct_t *rx_pack,volatile uint8_t *rec_data)
{
	rx_pack->length = (*(rec_data + 0) << 8) + *(rec_data + 1);
	rx_pack->packnum = *(rec_data + 2);
	//rx_pack->network =  (*(rec_data + 3))? LAN_NETWORK : WAN_NETWORK ;
	rx_pack->msgcode = *((AC_MsgCode *)(rec_data + 4));
	rx_pack->rxbuf = (rec_data + 5);
	
}

void AC_Decode_Command(volatile uint8_t *Order_Data, uint16_t Length)
{
//	uint8_t i=0;
	Ac_RxPackStruct_t RxPack;
		
	AC_DecodeRx_Buf(&RxPack,&Order_Data[0]);
	
	//USPRINTF_WIFI("Rec msgcode:%x\n",RxPack.msgcode);
	switch(RxPack.msgcode)
	{
		case AC_OTA_START:  
								//Total_Pack = AC_Get_Total_Pack(RxPack.rxbuf);
								Total_Pack = ((RxPack.rxbuf[0]) << 8) + (RxPack.rxbuf[1]);
								FLASH_EraseALL((Total_Pack + 1) * 64);
								TxPackage.packnum = RxPack.packnum;
								TxPackage.msgcode = RxPack.msgcode;
								TxPackage.data_length = 1;
								TxPackage.databuf[0] = 1;	
		
								AC_SendData2Cloud();
								break;
		
		case AC_OTA_TRAN:
								Rec_Data_Length = ((RxPack.rxbuf[4]) << 8) + (RxPack.rxbuf[5]);
								Package_Temp = ((RxPack.rxbuf[2]) << 8) + (RxPack.rxbuf[3]);
								if(Package_Temp == Package_Num)
								{									
									WriteData(Package_Num,RxPack.rxbuf);
									Package_Num++;

									if(Package_Num > Total_Pack)
									{
										TxPackage.packnum = RxPack.packnum;
										TxPackage.msgcode = AC_OTA_CHECK;
										TxPackage.data_length = 1;
										TxPackage.databuf[0] = 1;										 
										delay(1000);
										Set_Upgrade_Flag(0x55);
										delay(5000);
										NVIC_SystemReset();						
									}
									TxPackage.packnum = RxPack.packnum;
									TxPackage.msgcode = RxPack.msgcode;
									TxPackage.data_length = 1;
									TxPackage.databuf[0] = 1;		
								}
								else
								{
									TxPackage.packnum = RxPack.packnum;
									TxPackage.msgcode = RxPack.msgcode;
									TxPackage.data_length = 1;
									TxPackage.databuf[0] = 0;										
								}
		
		
								AC_SendData2Cloud();
								break;	
		default:
								break;
	}
}

void AC_SendData2Cloud(void)
{
	uint16_t length_idx = 0;

	Send_String_Buffer[length_idx++] = 0x5A;//��ͷ

	length_idx += 2; 
	
	Send_String_Buffer[length_idx++] = TxPackage.packnum;//�����
	Send_String_Buffer[length_idx++] = 0;//�����루����������������
	Send_String_Buffer[length_idx++] = TxPackage.msgcode;//������
		
	TxPackage.data_length = (TxPackage.data_length > TxPACKBUF_MAX)? TxPACKBUF_MAX : TxPackage.data_length;
	
	memcpy((uint8_t*)&Send_String_Buffer[length_idx],(uint8_t *)TxPackage.databuf,TxPackage.data_length);
	
	length_idx = length_idx + TxPackage.data_length + 2;
	Send_String_Buffer[1] = (length_idx >> 8)&0xff;//����H
	Send_String_Buffer[2] = (length_idx)&0xff;//����L
	
	Send_String_Buffer[length_idx - 2] = (Calc_PackageSum(&Send_String_Buffer[1],length_idx - 3));//У��
	Send_String_Buffer[length_idx - 1] = 0x5B;//��β
	
	/*-----------Send Data-------------------*/
	WIFI_DMA_Write_String(length_idx,Send_String_Buffer);
}

