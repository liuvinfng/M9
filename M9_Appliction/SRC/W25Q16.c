/**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  Wfliu
  * @version V1.0
  * @date    17-Nov-2011
  * @brief   The touch pad and remote functions which related to the user input
  ******************************************************************************
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "include.h"
#include "main.h"
#include "charge.h"
#include "projecttask.h"
#include "userinterface.h"
#include "remote_mode.h"
#include "spot.h"
#include "standby.h"
#include "wallfollow.h"
#include "homestraight.h"
#include "cormove.h"
#include "pathplanning.h"
#include "shortestpath.h"
#include "map.h"
#include "wifi.h"
#include "movement.h"
#include "display.h"
#include "usart.h"
#include "spi.h"
#include "speaker.h"
#include "rcon.h"
#include "rtc.h"
#include "touchpad.h"
#include "gyro.h"
#include "wheel.h"
#include "obscliff.h"
#include "bldc.h"
#include "brush.h"
#include "w25q16.h"
#include "mymath.h"
#include "debug.h"
#include "cmsis_os.h"
#include "config.h"



//��ȡW25QXX��״̬�Ĵ���
//BIT7  6   5   4   3   2   1   0
//SPR   RV  TB BP2 BP1 BP0 WEL BUSY
//SPR:Ĭ��0,״̬�Ĵ�������λ,���WPʹ��
//TB,BP2,BP1,BP0:FLASH����д��������
//WEL:дʹ������
//BUSY:æ���λ(1,æ;0,����)
//Ĭ��:0x00
u8 W25QXX_ReadSR(void)   
{  
	u8 byte=0;   
	W25Q16_CS_ON;                            //ʹ������   
	SPI2_ReadWriteByte(W25X_ReadStatusReg); //���Ͷ�ȡ״̬�Ĵ�������    
	byte=SPI2_ReadWriteByte(0Xff);          //��ȡһ���ֽ�  
	W25Q16_CS_OFF;                            //ȡ��Ƭѡ     
	return byte;   
} 
//дW25QXX״̬�Ĵ���
//ֻ��SPR,TB,BP2,BP1,BP0(bit 7,5,4,3,2)����д!!!
void W25QXX_Write_SR(u8 sr)   
{   
	W25Q16_CS_ON;                            //ʹ������   
	SPI2_ReadWriteByte(W25X_WriteStatusReg);//����дȡ״̬�Ĵ�������    
	SPI2_ReadWriteByte(sr);               	//д��һ���ֽ�  
	W25Q16_CS_OFF;                            //ȡ��Ƭѡ     	      
} 


//W25QXXдʹ��	
//��WEL��λ   
void W25QXX_Write_Enable(void)   
{
	W25Q16_CS_ON;                          	//ʹ������   
  SPI2_ReadWriteByte(W25X_WriteEnable); 	//����дʹ��  
	W25Q16_CS_OFF;                           	//ȡ��Ƭѡ     	      
} 
//W25QXXд��ֹ	
//��WEL����  
void W25QXX_Write_Disable(void)   
{  
	W25Q16_CS_ON;                            //ʹ������   
  SPI2_ReadWriteByte(W25X_WriteDisable);  //����д��ָֹ��    
	W25Q16_CS_OFF;                            //ȡ��Ƭѡ     	      
} 


u16 W25QXX_ReadID(void)
{
	u16 Temp = 0;	  
	W25Q16_CS_ON;				    
	SPI2_ReadWriteByte(W25X_ManufactDeviceID);//���Ͷ�ȡID����	    
	SPI2_ReadWriteByte(0x00); 	    
	SPI2_ReadWriteByte(0x00); 	    
	SPI2_ReadWriteByte(0x00); 	 			   
	Temp=SPI2_ReadWriteByte(0xFF)<<8;  
	Temp|=SPI2_ReadWriteByte(0xFF);	
	W25Q16_CS_OFF;				    
	return Temp;
}   


//��ȡSPI FLASH  
//��ָ����ַ��ʼ��ȡָ�����ȵ�����
//pBuffer:���ݴ洢��
//ReadAddr:��ʼ��ȡ�ĵ�ַ(24bit)
//NumByteToRead:Ҫ��ȡ���ֽ���(���65535)
void W25QXX_Read(u8* pBuffer,u32 ReadAddr,u16 NumByteToRead)   
{ 
 	u16 i;   										    
	W25Q16_CS_ON;                            	//ʹ������   
	SPI2_ReadWriteByte(W25X_ReadData);         	//���Ͷ�ȡ����   
	SPI2_ReadWriteByte((u8)((ReadAddr)>>16));  	//����24bit��ַ    
	SPI2_ReadWriteByte((u8)((ReadAddr)>>8));   
	SPI2_ReadWriteByte((u8)ReadAddr);   
	for(i=0;i<NumByteToRead;i++)
	{ 
		pBuffer[i]=SPI2_ReadWriteByte(0XFF);   	//ѭ������  
	}
	W25Q16_CS_OFF;  				    	      
}  

uint8_t W25QXX_Read_One(u32 ReadAddr)   
{ 
 	u8 temp=0;   										    
	W25Q16_CS_ON;                            	//ʹ������   
	SPI2_ReadWriteByte(W25X_ReadData);         	//���Ͷ�ȡ����   
	SPI2_ReadWriteByte((u8)((ReadAddr)>>16));  	//����24bit��ַ    
	SPI2_ReadWriteByte((u8)((ReadAddr)>>8));   
	SPI2_ReadWriteByte((u8)ReadAddr);   
	temp=SPI2_ReadWriteByte(0XFF);   	
	W25Q16_CS_OFF; 
	return temp;
}  
//SPI��һҳ(0~65535)��д������256���ֽڵ�����
//��ָ����ַ��ʼд�����256�ֽڵ�����
//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)
//NumByteToWrite:Ҫд����ֽ���(���256),������Ӧ�ó�����ҳ��ʣ���ֽ���!!!	 
void W25QXX_Write_Page(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite)
{
 	u16 i;  
	W25QXX_Write_Enable();                  	//SET WEL 
	W25Q16_CS_ON;                            	//ʹ������   
	SPI2_ReadWriteByte(W25X_PageProgram);      	//����дҳ����   
	SPI2_ReadWriteByte((u8)((WriteAddr)>>16)); 	//����24bit��ַ    
	SPI2_ReadWriteByte((u8)((WriteAddr)>>8));   
	SPI2_ReadWriteByte((u8)WriteAddr);   
	for(i=0;i<NumByteToWrite;i++)SPI2_ReadWriteByte(pBuffer[i]);//ѭ��д�� 
	W25Q16_CS_OFF;                            	//ȡ��Ƭѡ 
	W25QXX_Wait_Busy();					   		//�ȴ�д�����
} 


//�޼���дSPI FLASH 
//����ȷ����д�ĵ�ַ��Χ�ڵ�����ȫ��Ϊ0XFF,�����ڷ�0XFF��д������ݽ�ʧ��!
//�����Զ���ҳ���� 
//��ָ����ַ��ʼд��ָ�����ȵ�����,����Ҫȷ����ַ��Խ��!
//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)
//NumByteToWrite:Ҫд����ֽ���(���65535)
//CHECK OK
void W25QXX_Write_NoCheck(u8* pBuffer,u32 WriteAddr,u32 NumByteToWrite)   
{ 			 		 
	u16 pageremain;	   
	pageremain=256-WriteAddr%256; //��ҳʣ����ֽ���		 	    
	if(NumByteToWrite<=pageremain)pageremain=NumByteToWrite;//������256���ֽ�
	while(1)
	{	   
		W25QXX_Write_Page(pBuffer,WriteAddr,pageremain);
		if(NumByteToWrite==pageremain)break;//д�������
	 	else //NumByteToWrite>pageremain
		{
			pBuffer+=pageremain;
			WriteAddr+=pageremain;	

			NumByteToWrite-=pageremain;			  //��ȥ�Ѿ�д���˵��ֽ���
			if(NumByteToWrite>256)pageremain=256; //һ�ο���д��256���ֽ�
			else pageremain=NumByteToWrite; 	  //����256���ֽ���
		}
	}	    
} 


//дSPI FLASH  
//��ָ����ַ��ʼд��ָ�����ȵ�����
//�ú�������������!
//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)						
//NumByteToWrite:Ҫд����ֽ���(���65535)   
u8 W25QXX_BUFFER[4096];		 
void W25QXX_Write(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite)   
{ 
	u32 secpos;
	u16 secoff;
	u16 secremain;	   
 	u16 i;    
	u8 * W25QXX_BUF;	    	
	W25QXX_BUF=W25QXX_BUFFER;	     
 	secpos=WriteAddr/4096;//������ַ  
	secoff=WriteAddr%4096;//�������ڵ�ƫ��
	secremain=4096-secoff;//����ʣ��ռ��С   
 	//printf("ad:%X,nb:%X\r\n",WriteAddr,NumByteToWrite);//������
 	if(NumByteToWrite<=secremain)secremain=NumByteToWrite;//������4096���ֽ�
	while(1) 
	{	
		W25QXX_Read(W25QXX_BUF,secpos*4096,4096);//������������������
		for(i=0;i<secremain;i++)//У������
		{
			if(W25QXX_BUF[secoff+i]!=0XFF)break;//��Ҫ����  	  
		}
		if(i<secremain)//��Ҫ����
		{
			W25QXX_Erase_Sector(secpos);		//�����������
			for(i=0;i<secremain;i++)	   		//����
			{
				W25QXX_BUF[i+secoff]=pBuffer[i];	  
			}
			W25QXX_Write_NoCheck(W25QXX_BUF,secpos*4096,4096);//д����������  

		}else W25QXX_Write_NoCheck(pBuffer,WriteAddr,secremain);//д�Ѿ������˵�,ֱ��д������ʣ������. 				   
		if(NumByteToWrite==secremain)break;//д�������
		else//д��δ����
		{
			secpos++;//������ַ��1
			secoff=0;//ƫ��λ��Ϊ0 	 

		  pBuffer+=secremain;  				//ָ��ƫ��
			WriteAddr+=secremain;				//д��ַƫ��	   
		  NumByteToWrite-=secremain;			//�ֽ����ݼ�
			if(NumByteToWrite>4096)secremain=4096;//��һ����������д����
			else secremain=NumByteToWrite;		//��һ����������д����
		}	 
	}	 
}


//��������оƬ		  
//�ȴ�ʱ�䳬��...
void W25QXX_Erase_Chip(void)   
{                                   
  W25QXX_Write_Enable();                 	 	//SET WEL 
  W25QXX_Wait_Busy();   
  W25Q16_CS_ON;                            	//ʹ������   
  SPI2_ReadWriteByte(W25X_ChipErase);        	//����Ƭ��������  
	W25Q16_CS_OFF;                            	//ȡ��Ƭѡ     	      
	W25QXX_Wait_Busy();   				   		//�ȴ�оƬ��������
} 


//����һ������
//Dst_Addr:������ַ ����ʵ����������
//����һ��ɽ��������ʱ��:150ms
void W25QXX_Erase_Sector(u32 Dst_Addr)   
{  
	//����falsh�������,������   
// 	printf("fe:%x\r\n",Dst_Addr);	  
 	Dst_Addr*=4096;
	W25QXX_Write_Enable();                  	//SET WEL 	 
	W25QXX_Wait_Busy();   
	W25Q16_CS_ON;                            	//ʹ������   
	SPI2_ReadWriteByte(W25X_SectorErase);      	//������������ָ�� 
	SPI2_ReadWriteByte((u8)((Dst_Addr)>>16));  	//����24bit��ַ    
	SPI2_ReadWriteByte((u8)((Dst_Addr)>>8));   
	SPI2_ReadWriteByte((u8)Dst_Addr);  
	W25Q16_CS_OFF;                            	//ȡ��Ƭѡ     	      
  W25QXX_Wait_Busy();   				   		//�ȴ��������
} 

 
//�ȴ�����
void W25QXX_Wait_Busy(void)   
{   
	while((W25QXX_ReadSR()&0x01)==0x01);  		// �ȴ�BUSYλ���
} 


//�������ģʽ
void W25QXX_PowerDown(void)   
{ 
  W25Q16_CS_ON;                           	 	//ʹ������   
  SPI2_ReadWriteByte(W25X_PowerDown);        //���͵�������  
	W25Q16_CS_OFF;                            	//ȡ��Ƭѡ     	       
}   


//����
void W25QXX_WAKEUP(void)   
{  
  W25Q16_CS_ON;                            	//ʹ������   
  SPI2_ReadWriteByte(W25X_ReleasePowerDown);	//  send W25X_PowerDown command 0xAB    
	W25Q16_CS_OFF;                            	//ȡ��Ƭѡ     	      
}   






