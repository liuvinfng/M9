/**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  Wfliu
  * @version V1.0
  * @date    17-Nov-2011
  * @brief   Initialize Usart function and prosessing the characters transmitting
  ******************************************************************************
  * Initialize the System Clock.ADC converter.EXTI.Timer and USART3
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "Rcon.h"


volatile Rcon_Element BRrcon = {0, 0, 0, 0, 0, 0};
volatile Rcon_Element FLrcon = {0, 0, 0, 0, 0, 0};
volatile Rcon_Element FRrcon = {0, 0, 0, 0, 0, 0};
volatile Rcon_Element Rrcon  = {0, 0, 0, 0, 0, 0};
volatile Rcon_Element Lrcon  = {0, 0, 0, 0, 0, 0};
 
volatile uint32_t g_rcon_remote_code=0,g_wifi_remote_code=0;
volatile uint32_t g_rcon_status = 0;



/*----------------------------------------charge--------------------------------*/
uint32_t Rcon_GetStatus(void)
{
  return g_rcon_status;
}

void Rcon_ResetStatus(void)
{
  g_rcon_status=0;
}

/*----------------------------------------Remote--------------------------------*/
uint32_t Rcon_GetRemoteCode(void)
{
  return g_rcon_remote_code;
}
void Rcon_SetRemoteCode(uint32_t Code)
{
  g_rcon_remote_code = Code;
}
void Rcon_ResetRemoteCode(void)
{
	Rcon_SetRemoteCode(0);
}

uint32_t Wifi_GetRemoteCode(void)
{
  return g_wifi_remote_code;
}
void Wifi_SetRemoteCode(uint32_t Code)
{
  g_wifi_remote_code = Code;
	g_rcon_remote_code = Code;
}

uint8_t Is_Turn_Remote(void)
{
	if(Rcon_GetRemoteCode()==Remote_Max)return 1;
	if(Rcon_GetRemoteCode()==Remote_Home)return 1;
  if(Rcon_GetRemoteCode()==Remote_Spot)return 1;
  if(Rcon_GetRemoteCode()==Remote_Wallfollow)return 1;
	return 0;
}

uint8_t Rcon_IsNearStation(void)
{
	static uint32_t Station_Counter=0;
	static uint32_t No_SH_Signal=0;
  static uint32_t Code=0;
		
	Code= Rcon_GetStatus();
	
	if(Code & Rcon_Signal_LRFLFR_T)
	{
		return 1;
	}
  if(Code&0x0330FF)
	{
		No_SH_Signal=0;
		Rcon_ResetStatus();
		Station_Counter++;
		if(Station_Counter>3)
		{
			Station_Counter=0;
			return 1;
		}
	}
	else
	{
		No_SH_Signal++;
		if(No_SH_Signal>50)
		{
			No_SH_Signal=0;
		  Station_Counter=0;
		}
	}
	return 0;
}


void Rcon_Timer(void)
{
	/*---------------------Rcon Counter----------------*/
	FLrcon.Time_Counter++;
	if(FLrcon.Time_Counter>0xfffd)FLrcon.Time_Counter=0xfffd;

	FRrcon.Time_Counter++;
	if(FRrcon.Time_Counter>0xfffd)FRrcon.Time_Counter=0xfffd;

	Rrcon.Time_Counter++;
	if(Rrcon.Time_Counter>0xfffd)Rrcon.Time_Counter=0xfffd;	

	Lrcon.Time_Counter++;
	if(Lrcon.Time_Counter>0xfffd)Lrcon.Time_Counter=0xfffd;

	BRrcon.Time_Counter++;
	if(BRrcon.Time_Counter>0xfffd)BRrcon.Time_Counter=0xfffd;		
}

void Rcon_BR(void)
{
		if(BRrcon.Time_Counter>500)
		{
			BRrcon.Receive_Start=0;
			BRrcon.Time_Counter=0;
			BRrcon.Temp_Counter=0;
			BRrcon.Temp_Code=0;
		}
    if(RCON_B_IDR)//high level
    {		 
			if(BRrcon.Receive_Start) 
			{
			  if((BRrcon.Time_Counter-BRrcon.Temp_Counter)>1)
				{
					BRrcon.Time_Counter=0;
					BRrcon.Temp_Counter=0;
				}
			}
			else//Check Low Head
			{		
        if((BRrcon.Time_Counter>75)&&(BRrcon.Time_Counter<100))//Base Code	 
				{
					BRrcon.Receive_Start=1;
					BRrcon.Time_Counter=0;
					BRrcon.Temp_Counter=0;
          BRrcon.Remote_Flag = 0;
				}		
				else if((BRrcon.Time_Counter>47)&&(BRrcon.Time_Counter<53))//Base Code	 
				{
					BRrcon.Receive_Start=1;
					BRrcon.Time_Counter=0;
					BRrcon.Temp_Counter=0;
          BRrcon.Remote_Flag = 1;
				}
				else
        {
          BRrcon.Temp_Counter=BRrcon.Time_Counter;
        }
			}
    }
    else//low level
    { 
			//-----------------------------------------------------Rcon Remote Code-----------------------------//
			if(BRrcon.Receive_Start==1)//Check if High Head
			{
			  if((BRrcon.Time_Counter>42)&&(BRrcon.Time_Counter<48))
				{
					BRrcon.Receive_Start=2;
					BRrcon.Temp_Code=0;
					BRrcon.Time_Counter=0;
					BRrcon.Temp_Counter=0;
          BRrcon.BitCounter=0;
				}
				else if((BRrcon.Time_Counter>47)&&(BRrcon.Time_Counter<53))
				{
					BRrcon.Receive_Start=2;
					BRrcon.Temp_Code=0;
					BRrcon.Time_Counter=0;
					BRrcon.Temp_Counter=0;
          BRrcon.BitCounter=0;				
				}
				else 
				{
					BRrcon.Receive_Start=0;
				}
			}
			else if(BRrcon.Receive_Start==2)
			{
				//-----------------------------------------------Standard remote---------------------------------//
			  if((BRrcon.Time_Counter>14)&&(BRrcon.Time_Counter<19))//1.69
				{
				  BRrcon.Temp_Code |= 0x1;
					BRrcon.BitCounter++;
					BRrcon.Temp_Code <<= 1;
				}
				else if((BRrcon.Time_Counter>3)&&(BRrcon.Time_Counter<7))//0.56
				{
				  BRrcon.Temp_Code &= ~0x1;					
					BRrcon.BitCounter++;					
					BRrcon.Temp_Code <<= 1;
				}
				//////////start/////////
			  else if((BRrcon.Time_Counter>7)&&(BRrcon.Time_Counter<13))//1
				{

				  BRrcon.Temp_Code |= 0x01;
					BRrcon.BitCounter++;
					BRrcon.Temp_Code <<= 1;					
				}
				else if((BRrcon.Time_Counter>27)&&(BRrcon.Time_Counter<33))//3
				{
				  BRrcon.Temp_Code &= ~0x01;					
					BRrcon.BitCounter++;					
					BRrcon.Temp_Code <<= 1;	
				}				
				///////////end/////////
				else 
				{
					if((BRrcon.Time_Counter>42)&&(BRrcon.Time_Counter<48))
					{
						BRrcon.Receive_Start=2;
						BRrcon.Temp_Code=0;
						BRrcon.Time_Counter=0;
						BRrcon.Temp_Counter=0;
						BRrcon.BitCounter=0;
					}
					else if((BRrcon.Time_Counter-BRrcon.Temp_Counter)>1)
  				{
  					BRrcon.Time_Counter=0;
  					BRrcon.Temp_Counter=0;
  				  BRrcon.Receive_Start=0;
  				}
				}
				
				if(BRrcon.BitCounter==7)
				{
          if(BRrcon.Remote_Flag )
          {
						BRrcon.BitCounter=0;
						BRrcon.Receive_Start=0;
            if((BRrcon.Temp_Code) == Charge_Home_Left)g_rcon_status |= RconBR_HomeL;
							
            else if((BRrcon.Temp_Code) == Charge_Home_Right)g_rcon_status |= RconBR_HomeR;
						
            else if((BRrcon.Temp_Code) == Charge_Home_Left_Side)g_rcon_status |= RconBR_LEFT;
							
            else if((BRrcon.Temp_Code) == Charge_Home_Right_Side)g_rcon_status |= RconBR_RIGHT;	
						
            else if((BRrcon.Temp_Code) == Charge_Home_Top)g_rcon_status |= RconBR_HomeT;
							
            else if((BRrcon.Temp_Code) == Vitual_Wall_Code)g_rcon_status |= RconBR_Wall;
						else if((BRrcon.Temp_Code) == Vitual_Wall_Code_Top)g_rcon_status |= RconBR_Wall_T;
						BRrcon.Temp_Code=0;	
					}					
				}
				if(BRrcon.BitCounter>=31)
				{
					if((BRrcon.Temp_Code&0x00FF0000)== 0x00FF0000)//remote command
					{
						Rcon_SetRemoteCode(BRrcon.Temp_Code);
						BRrcon.BitCounter=0;
					  BRrcon.Receive_Start=0;
					} 
					BRrcon.Temp_Code=0;	
					BRrcon.BitCounter=0;
					BRrcon.Receive_Start=0;					
				}	
        BRrcon.Temp_Counter = BRrcon.Time_Counter;
			}		
			else
			{
				if((BRrcon.Time_Counter-BRrcon.Temp_Counter)>1)
				{
					BRrcon.Time_Counter=0;
					BRrcon.Temp_Counter=0;
				}			
			}
		}
}

void Rcon_L(void)
{
		if(Lrcon.Time_Counter>500)
		{
			Lrcon.Receive_Start=0;
			Lrcon.Time_Counter=0;
			Lrcon.Temp_Counter=0;
			Lrcon.Temp_Code=0;
		}
    if(RCON_L_IDR)//high level
    {
		  //----------------------------------------Rmote Code------------//
			if(Lrcon.Receive_Start) 
			{
			  if((Lrcon.Time_Counter-Lrcon.Temp_Counter)>1)
				{
					Lrcon.Time_Counter=0;
					Lrcon.Temp_Counter=0;
				}
			}
			else//Check Low Head
			{		
        if((Lrcon.Time_Counter>75)&&(Lrcon.Time_Counter<100))//Base Code	 
				{
					Lrcon.Receive_Start=1;
					Lrcon.Time_Counter=0;
					Lrcon.Temp_Counter=0;
          Lrcon.Remote_Flag = 0;
				}	
				else if((Lrcon.Time_Counter>47)&&(Lrcon.Time_Counter<53))//Base Code	 
				{
					Lrcon.Receive_Start=1;
					Lrcon.Time_Counter=0;
					Lrcon.Temp_Counter=0;
          Lrcon.Remote_Flag = 1;
				}
				else
        {
          Lrcon.Temp_Counter=Lrcon.Time_Counter;
        }
			}
    }
    else//low level
    { 
			//-----------------------------------------------------Rcon Remote Code-----------------------------//
			if(Lrcon.Receive_Start==1)//Check if High Head
			{
			  if((Lrcon.Time_Counter>42)&&(Lrcon.Time_Counter<48))
				{
					Lrcon.Receive_Start=2;
					Lrcon.Temp_Code=0;
					Lrcon.Time_Counter=0;
					Lrcon.Temp_Counter=0;
          Lrcon.BitCounter=0;
				}
				else if((Lrcon.Time_Counter>47)&&(Lrcon.Time_Counter<53))
				{
					Lrcon.Receive_Start=2;
					Lrcon.Temp_Code=0;
					Lrcon.Time_Counter=0;
					Lrcon.Temp_Counter=0;
          Lrcon.BitCounter=0;				
				}
				else 
				{
					Lrcon.Receive_Start=0;
				}
			}
			else if(Lrcon.Receive_Start==2)
			{
				//-----------------------------------------------Standard remote---------------------------------//
			  if((Lrcon.Time_Counter>14)&&(Lrcon.Time_Counter<19))//1.69
				{
				  Lrcon.Temp_Code |= 0x1;
					Lrcon.BitCounter++;
					Lrcon.Temp_Code <<= 1;
				}
				else if((Lrcon.Time_Counter>3)&&(Lrcon.Time_Counter<7))//0.56
				{
				  Lrcon.Temp_Code &= ~0x1;					
					Lrcon.BitCounter++;					
					Lrcon.Temp_Code <<= 1;
				}
				//////////start/////////
			  else if((Lrcon.Time_Counter>7)&&(Lrcon.Time_Counter<13))//1
				{
	
				  Lrcon.Temp_Code |= 0x1;
					Lrcon.BitCounter++;
					Lrcon.Temp_Code <<= 1;					
				}
				else if((Lrcon.Time_Counter>27)&&(Lrcon.Time_Counter<33))//3
				{
				  Lrcon.Temp_Code &= ~0x1;					
					Lrcon.BitCounter++;					
					Lrcon.Temp_Code <<= 1;
				}				
				///////////end/////////
				else 
				{
					if((Lrcon.Time_Counter>42)&&(Lrcon.Time_Counter<48))
					{
						Lrcon.Receive_Start=2;
						Lrcon.Temp_Code=0;
						Lrcon.Time_Counter=0;
						Lrcon.Temp_Counter=0;
						Lrcon.BitCounter=0;
					}
					else if((Lrcon.Time_Counter-Lrcon.Temp_Counter)>1)
  				{
  					Lrcon.Time_Counter=0;
  					Lrcon.Temp_Counter=0;
  				  Lrcon.Receive_Start=0;
  				}
				}
				
				if(Lrcon.BitCounter==7)
				{
          if(Lrcon.Remote_Flag )
          {
						Lrcon.BitCounter=0;
						Lrcon.Receive_Start=0;
            if((Lrcon.Temp_Code) == Charge_Home_Left)g_rcon_status |= RconL_HomeL;
            else if((Lrcon.Temp_Code) == Charge_Home_Right)g_rcon_status |= RconL_HomeR;
            else if((Lrcon.Temp_Code) == Charge_Home_Left_Side)g_rcon_status |= RconL_LEFT;
            else if((Lrcon.Temp_Code) == Charge_Home_Right_Side)g_rcon_status |= RconL_RIGHT;						
            else if((Lrcon.Temp_Code) == Charge_Home_Top)g_rcon_status |= RconL_HomeT;
            else if((Lrcon.Temp_Code) == Vitual_Wall_Code)g_rcon_status |= RconL_Wall;
						else if((Lrcon.Temp_Code) == Vitual_Wall_Code_Top)g_rcon_status |= RconL_Wall_T;
						Lrcon.Temp_Code=0;	
					}					
				}
				if(Lrcon.BitCounter>=31)
				{
					if((Lrcon.Temp_Code&0x00FF0000)== 0x00FF0000)//remote command
					{
						Rcon_SetRemoteCode(Lrcon.Temp_Code);
						Lrcon.BitCounter=0;
					  Lrcon.Receive_Start=0;
					}
					Lrcon.Temp_Code=0;
					Lrcon.BitCounter=0;
					Lrcon.Receive_Start=0;					
				}	
        Lrcon.Temp_Counter = Lrcon.Time_Counter;
			}		
			else
			{				
				if((Lrcon.Time_Counter-Lrcon.Temp_Counter)>1)
				{
					Lrcon.Time_Counter=0;
					Lrcon.Temp_Counter=0;
				}			
			}
		}
}

void Rcon_FL(void)
{
		if(FLrcon.Time_Counter>500)
		{
			FLrcon.Receive_Start=0;
			FLrcon.Time_Counter=0;
			FLrcon.Temp_Counter=0;
			FLrcon.Temp_Code=0;
		}
    if(RCON_FL_IDR)//high level
    {
			if(FLrcon.Receive_Start) 
			{
			  if((FLrcon.Time_Counter-FLrcon.Temp_Counter)>1)
				{
					FLrcon.Time_Counter=0;
					FLrcon.Temp_Counter=0;
				}
			}
			else//Check Low Head
			{		
        if((FLrcon.Time_Counter>75)&&(FLrcon.Time_Counter<100))//Base Code	 
				{
					FLrcon.Receive_Start=1;
					FLrcon.Time_Counter=0;
					FLrcon.Temp_Counter=0;
          FLrcon.Remote_Flag = 0;
				}	
				else if((FLrcon.Time_Counter>47)&&(FLrcon.Time_Counter<53))//Base Code	 
				{
					FLrcon.Receive_Start=1;
					FLrcon.Time_Counter=0;
					FLrcon.Temp_Counter=0;
          FLrcon.Remote_Flag = 1;
				}
				else
        {
          FLrcon.Temp_Counter=FLrcon.Time_Counter;
        }
			}
    }
    else//low level
    { 
			 //-----------------------------------------------------Rcon Remote Code-----------------------------//
			if(FLrcon.Receive_Start==1)	//Check if High Head
			{
			  if((FLrcon.Time_Counter>42)&&(FLrcon.Time_Counter<48))
				{
					FLrcon.Receive_Start=2;
					FLrcon.Temp_Code=0;
					FLrcon.Time_Counter=0;
					FLrcon.Temp_Counter=0;
          FLrcon.BitCounter=0;
				}
				else if((FLrcon.Time_Counter>47)&&(FLrcon.Time_Counter<53))
				{
					FLrcon.Receive_Start=2;
					FLrcon.Temp_Code=0;
					FLrcon.Time_Counter=0;
					FLrcon.Temp_Counter=0;
          FLrcon.BitCounter=0;				
				}
				else 
				{
					FLrcon.Receive_Start=0;
				}
			}
			else if(FLrcon.Receive_Start==2)
			{
				//-----------------------------------------------Standard remote---------------------------------//
			  if((FLrcon.Time_Counter>14)&&(FLrcon.Time_Counter<19))//1.69
				{
				  FLrcon.Temp_Code |= 0x1;
					FLrcon.BitCounter++;
					FLrcon.Temp_Code <<= 1;
				}
				else if((FLrcon.Time_Counter>3)&&(FLrcon.Time_Counter<7))//0.56
				{
				  FLrcon.Temp_Code &= ~0x1;					
					FLrcon.BitCounter++;					
					FLrcon.Temp_Code <<= 1;
				}
				//////////start/////////
			  else if((FLrcon.Time_Counter>7)&&(FLrcon.Time_Counter<13))//1
				{
	
				  FLrcon.Temp_Code |= 0x1;
					FLrcon.BitCounter++;
					FLrcon.Temp_Code <<= 1;					
				}
				else if((FLrcon.Time_Counter>27)&&(FLrcon.Time_Counter<33))//3
				{
				  FLrcon.Temp_Code &= ~0x1;					
					FLrcon.BitCounter++;					
					FLrcon.Temp_Code <<= 1;
				}				
				///////////end/////////
				else 
				{
					if((FLrcon.Time_Counter>42)&&(FLrcon.Time_Counter<48))
					{
						FLrcon.Receive_Start=2;
						FLrcon.Temp_Code=0;
						FLrcon.Time_Counter=0;
						FLrcon.Temp_Counter=0;
						FLrcon.BitCounter=0;
					}
					else if((FLrcon.Time_Counter-FLrcon.Temp_Counter)>1)
  				{
  					FLrcon.Time_Counter=0;
  					FLrcon.Temp_Counter=0;
  				  FLrcon.Receive_Start=0;
  				}
				}
				
				if(FLrcon.BitCounter==7)
				{
          if(FLrcon.Remote_Flag )
          {
						FLrcon.BitCounter=0;
						FLrcon.Receive_Start=0;
            if((FLrcon.Temp_Code) == Charge_Home_Left)g_rcon_status |= RconFL_HomeL;
            else if((FLrcon.Temp_Code) == Charge_Home_Right)g_rcon_status |= RconFL_HomeR;
            else if((FLrcon.Temp_Code) == Charge_Home_Left_Side)g_rcon_status |= RconFL_LEFT;
            else if((FLrcon.Temp_Code) == Charge_Home_Right_Side)g_rcon_status |= RconFL_RIGHT;						
            else if((FLrcon.Temp_Code) == Charge_Home_Top)g_rcon_status |= RconFL_HomeT;
            else if((FLrcon.Temp_Code) == Vitual_Wall_Code)g_rcon_status |= RconFL_Wall;
						else if((FLrcon.Temp_Code) == Vitual_Wall_Code_Top)g_rcon_status |= RconFL_Wall_T;
						FLrcon.Temp_Code=0;	
					}					
				}
				if(FLrcon.BitCounter>=31)
				{
					if((FLrcon.Temp_Code&0x00FF0000)== 0x00FF0000)//remote command
					{
						Rcon_SetRemoteCode(FLrcon.Temp_Code);
						FLrcon.BitCounter=0;
					  FLrcon.Receive_Start=0;
					}
					FLrcon.Temp_Code=0;
					FLrcon.BitCounter=0;
					FLrcon.Receive_Start=0;					
				}	
        FLrcon.Temp_Counter = FLrcon.Time_Counter;
			}		
			else
			{					
				if((FLrcon.Time_Counter-FLrcon.Temp_Counter)>1)
				{
					FLrcon.Time_Counter=0;
					FLrcon.Temp_Counter=0;
				}			
			}
		}
}

void Rcon_R(void)
{
		if(Rrcon.Time_Counter>500)
		{
			Rrcon.Receive_Start=0;
			Rrcon.Time_Counter=0;
			Rrcon.Temp_Counter=0;
			Rrcon.Temp_Code=0;
		}
    if(RCON_R_IDR)//high level
    {
			if(Rrcon.Receive_Start) 
			{
			  if((Rrcon.Time_Counter-Rrcon.Temp_Counter)>1)
				{
					Rrcon.Time_Counter=0;
					Rrcon.Temp_Counter=0;
				}
			}
			else//Check Low Head
			{		
        if((Rrcon.Time_Counter>75)&&(Rrcon.Time_Counter<100))//Base Code	 
				{
					Rrcon.Receive_Start=1;
					Rrcon.Time_Counter=0;
					Rrcon.Temp_Counter=0;
          Rrcon.Remote_Flag = 0;
				}	
				else if((Rrcon.Time_Counter>47)&&(Rrcon.Time_Counter<53))//Base Code	 
				{
					Rrcon.Receive_Start=1;
					Rrcon.Time_Counter=0;
					Rrcon.Temp_Counter=0;
          Rrcon.Remote_Flag = 1;
				}
				else
        {
          Rrcon.Temp_Counter=Rrcon.Time_Counter;
        }
			}
    }
    else//low level
    { 
			//-----------------------------------------------------Rcon Remote Code-----------------------------//
			if(Rrcon.Receive_Start==1)	//Check if High Head
			{
			  if((Rrcon.Time_Counter>42)&&(Rrcon.Time_Counter<48))
				{
					Rrcon.Receive_Start=2;
					Rrcon.Temp_Code=0;
					Rrcon.Time_Counter=0;
					Rrcon.Temp_Counter=0;
          Rrcon.BitCounter=0;
				}
				else if((Rrcon.Time_Counter>47)&&(Rrcon.Time_Counter<53))
				{
					Rrcon.Receive_Start=2;
					Rrcon.Temp_Code=0;
					Rrcon.Time_Counter=0;
					Rrcon.Temp_Counter=0;
          Rrcon.BitCounter=0;				
				}
				else 
				{
					Rrcon.Receive_Start=0;
				}
			}
			else if(Rrcon.Receive_Start==2)
			{
				//-----------------------------------------------Standard remote---------------------------------//
			  if((Rrcon.Time_Counter>14)&&(Rrcon.Time_Counter<19))//1.69
				{
				  Rrcon.Temp_Code |= 0x1;
					Rrcon.BitCounter++;
					Rrcon.Temp_Code <<= 1;
				}
				else if((Rrcon.Time_Counter>3)&&(Rrcon.Time_Counter<7))//0.56
				{
				  Rrcon.Temp_Code &= ~0x1;					
					Rrcon.BitCounter++;					
					Rrcon.Temp_Code <<= 1;
				}
				//////////start/////////
			  else if((Rrcon.Time_Counter>7)&&(Rrcon.Time_Counter<13))//1
				{

				  Rrcon.Temp_Code |= 0x1;
					Rrcon.BitCounter++;
					Rrcon.Temp_Code <<= 1;						
				}
				else if((Rrcon.Time_Counter>27)&&(Rrcon.Time_Counter<33))//0
				{
					Rrcon.Temp_Code &= ~0x1;					
					Rrcon.BitCounter++;					
					Rrcon.Temp_Code <<= 1;			
				}				
				///////////end/////////
				else 
				{
					if((Rrcon.Time_Counter>42)&&(Rrcon.Time_Counter<48))
					{
						Rrcon.Receive_Start=2;
						Rrcon.Temp_Code=0;
						Rrcon.Time_Counter=0;
						Rrcon.Temp_Counter=0;
						Rrcon.BitCounter=0;
					}
					else if((Rrcon.Time_Counter-Rrcon.Temp_Counter)>1)
  				{
  					Rrcon.Time_Counter=0;
  					Rrcon.Temp_Counter=0;
  				  Rrcon.Receive_Start=0;
  				}
				}
				
				if(Rrcon.BitCounter==7)
				{
          if(Rrcon.Remote_Flag )
          {
						Rrcon.BitCounter=0;
						Rrcon.Receive_Start=0;
            if((Rrcon.Temp_Code) == Charge_Home_Left)g_rcon_status |= RconR_HomeL;
            else if((Rrcon.Temp_Code) == Charge_Home_Right)g_rcon_status |= RconR_HomeR;
            else if((Rrcon.Temp_Code) == Charge_Home_Left_Side)g_rcon_status |= RconR_LEFT;
            else if((Rrcon.Temp_Code) == Charge_Home_Right_Side)g_rcon_status |= RconR_RIGHT;						
            else if((Rrcon.Temp_Code) == Charge_Home_Top)g_rcon_status |= RconR_HomeT;
            else if((Rrcon.Temp_Code) == Vitual_Wall_Code)g_rcon_status |= RconR_Wall;
						else if((Rrcon.Temp_Code) == Vitual_Wall_Code_Top)g_rcon_status |= RconR_Wall_T;
						Rrcon.Temp_Code=0;	
					}					
				}
				if(Rrcon.BitCounter>=31)
				{
					if((Rrcon.Temp_Code&0x00FF0000)== 0x00FF0000)//remote command
					{
						Rcon_SetRemoteCode(Rrcon.Temp_Code);
						Rrcon.BitCounter=0;
					  Rrcon.Receive_Start=0;
					} 
					Rrcon.Temp_Code=0;
					Rrcon.BitCounter=0;
					Rrcon.Receive_Start=0;					
				}	
        Rrcon.Temp_Counter = Rrcon.Time_Counter;
			}		
			else
			{					
				if((Rrcon.Time_Counter-Rrcon.Temp_Counter)>1)
				{
					Rrcon.Time_Counter=0;
					Rrcon.Temp_Counter=0;
				}			
			}
		}
}

void Rcon_FR(void)
{
		if(FRrcon.Time_Counter>500)
		{
			FRrcon.Receive_Start=0;
			FRrcon.Time_Counter=0;
			FRrcon.Temp_Counter=0;
			FRrcon.Temp_Code=0;
		}
    if(RCON_FR_IDR)//high level
    {
			if(FRrcon.Receive_Start) 
			{
			  if((FRrcon.Time_Counter-FRrcon.Temp_Counter)>1)
				{
					FRrcon.Time_Counter=0;
					FRrcon.Temp_Counter=0;
				}
			}
			else	//Check Low Head
			{		
        if((FRrcon.Time_Counter>75)&&(FRrcon.Time_Counter<100))//Base Code	 
				{
					FRrcon.Receive_Start=1;
					FRrcon.Time_Counter=0;
					FRrcon.Temp_Counter=0;
          FRrcon.Remote_Flag = 0;
				}	
				else if((FRrcon.Time_Counter>47)&&(FRrcon.Time_Counter<53))//Base Code	 
				{
					FRrcon.Receive_Start=1;
					FRrcon.Time_Counter=0;
					FRrcon.Temp_Counter=0;
          FRrcon.Remote_Flag = 1;
				}
				else
        {
          FRrcon.Temp_Counter=FRrcon.Time_Counter;
        }
			}
    }
    else//low level
    { 
			 //-----------------------------------------------------Rcon Remote Code-----------------------------//
			if(FRrcon.Receive_Start==1)//Check if High Head
			{
			  if((FRrcon.Time_Counter>42)&&(FRrcon.Time_Counter<48))
				{
					FRrcon.Receive_Start=2;
					FRrcon.Temp_Code=0;
					FRrcon.Time_Counter=0;
					FRrcon.Temp_Counter=0;
          FRrcon.BitCounter=0;
				}	
				else if((FRrcon.Time_Counter>47)&&(FRrcon.Time_Counter<53))
				{
					FRrcon.Receive_Start=2;
					FRrcon.Temp_Code=0;
					FRrcon.Time_Counter=0;
					FRrcon.Temp_Counter=0;
          FRrcon.BitCounter=0;				
				}
				else 
				{
					FRrcon.Receive_Start=0;
				}
			}
			else if(FRrcon.Receive_Start==2)
			{
				//-----------------------------------------------Standard remote---------------------------------//
			  if((FRrcon.Time_Counter>14)&&(FRrcon.Time_Counter<19))//1.69
				{
				  FRrcon.Temp_Code |= 0x1;
					FRrcon.BitCounter++;
					FRrcon.Temp_Code <<= 1;
				}
				else if((FRrcon.Time_Counter>3)&&(FRrcon.Time_Counter<7))//0.56
				{
				  FRrcon.Temp_Code &= ~0x1;					
					FRrcon.BitCounter++;					
					FRrcon.Temp_Code <<= 1;
				}
				//////////start/////////
			  else if((FRrcon.Time_Counter>7)&&(FRrcon.Time_Counter<13))//1
				{
	
				  FRrcon.Temp_Code |= 0x1;
					FRrcon.BitCounter++;
					FRrcon.Temp_Code <<= 1;					
				}
				else if((FRrcon.Time_Counter>27)&&(FRrcon.Time_Counter<33))//3
				{
				  FRrcon.Temp_Code &= ~0x1;					
					FRrcon.BitCounter++;					
					FRrcon.Temp_Code <<= 1;	
				}				
				///////////end/////////
				else 
				{
					if((FRrcon.Time_Counter>42)&&(FRrcon.Time_Counter<48))
					{
						FRrcon.Receive_Start=2;
						FRrcon.Temp_Code=0;
						FRrcon.Time_Counter=0;
						FRrcon.Temp_Counter=0;
						FRrcon.BitCounter=0;
					}
					else if((FRrcon.Time_Counter-FRrcon.Temp_Counter)>1)
  				{
  					FRrcon.Time_Counter=0;
  					FRrcon.Temp_Counter=0;
  				  FRrcon.Receive_Start=0;
  				}
				}
				
				if(FRrcon.BitCounter==7)
				{
          if(FRrcon.Remote_Flag )//&& ((FRrcon.Temp_Code&0x30)== 0x30)
          {
						FRrcon.BitCounter=0;
						FRrcon.Receive_Start=0;
            if((FRrcon.Temp_Code) == Charge_Home_Left)g_rcon_status |= RconFR_HomeL;
            else if((FRrcon.Temp_Code) == Charge_Home_Right)g_rcon_status |= RconFR_HomeR;
            else if((FRrcon.Temp_Code) == Charge_Home_Left_Side)g_rcon_status |= RconFR_LEFT;
            else if((FRrcon.Temp_Code) == Charge_Home_Right_Side)g_rcon_status |= RconFR_RIGHT;						
            else if((FRrcon.Temp_Code) == Charge_Home_Top)g_rcon_status |= RconFR_HomeT;
            else if((FRrcon.Temp_Code) == Vitual_Wall_Code)g_rcon_status |= RconFR_Wall;
						else if((FRrcon.Temp_Code) == Vitual_Wall_Code_Top)g_rcon_status |= RconFR_Wall_T;
						FRrcon.Temp_Code=0;	
					}					
				}
				if(FRrcon.BitCounter>=31)
				{
					if((FRrcon.Temp_Code&0x00FF0000)== 0x00FF0000)//remote command
					{
						Rcon_SetRemoteCode(FRrcon.Temp_Code);
						FRrcon.BitCounter=0;
					  FRrcon.Receive_Start=0;
					}
					FRrcon.Temp_Code=0;
					FRrcon.BitCounter=0;
					FRrcon.Receive_Start=0;					
				}	
        FRrcon.Temp_Counter = FRrcon.Time_Counter;
			}		
			else
			{				
				if((FRrcon.Time_Counter-FRrcon.Temp_Counter)>1)
				{
					FRrcon.Time_Counter=0;
					FRrcon.Temp_Counter=0;
				}			
			}
		}
}





