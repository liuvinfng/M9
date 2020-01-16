 /**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  Wfliu
  * @version V1.0
  * @date    17-Nov-2018
  * @brief   Move near the wall on the left in a certain distance
  ******************************************************************************
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "WallFollow.h"
#include "Movement.h"
#include "Speaker.h"
#include "USART.h"
#include "Display.h"
#include "TouchPad.h"
#include "Charge.h"
#include "Spot.h"
#include "Rcon.h"
#include "Home.h"
#include "VirtualWall.h"
#include "WallFollowMulti.h"



volatile int32_t Wall_Follow_Distance = 0;

/*------------------------------------------------------------------ Wall Follow --------------------------*/
void Wall_Follow(void)
{
	volatile uint8_t Motor_Check_Code = 0,cycle=0;
  int32_t Proportion=0;
	int32_t Delta=0;
	int32_t Previous=0;
  volatile int32_t Wall_Distance=Wall_High_Limit;
	volatile int32_t Wall_Straight_Distance=DISTANCE_1CM;
	volatile int32_t Left_Wall_Speed=0,Right_Wall_Speed=0;
  volatile uint32_t L_B_Counter=0;
	uint8_t Temp_Counter=0;
	uint8_t Jam=0;
	int32_t R=0;
	uint8_t Temp_Dirt_Status=0;

	int16_t Left_Wall_Buffer[3]={0};
	#ifdef MOBILITY
  uint32_t Temp_Mobility_Distance=0;
  uint8_t Mobility_Temp_Error=0;
	#endif
	static uint8_t Vac_Mode_Buffer=0;
  uint32_t Temp_Rcon_Status = 0;
	uint8_t RandomRemoteWall_Flag=0;
	uint32_t WorkTime_Buffer=0;
	
	Reset_Bumper_Error();
	Work_Motor_Configure();  
	Set_RightWheel_Speed(15);
  Reset_Wall_Step();
	Move_Forward(RUN_SPEED_15,RUN_SPEED_15);
  Display_Clean_Status(Display_Wall);
	Reset_Touch();
  Reset_Rcon_Remote();
	Reset_Average_Counter();
	Set_Vac_Speed();

	if(Is_MoveWithRemote()||Is_RemoteRandomWall())
	{
		if(Is_RemoteRandomWall())
		{
			WorkTime_Buffer = Get_WorkTime();
			RandomRemoteWall_Flag=1;
			Reset_RemoteRandomWall();
		}
		while(1)
		{
		  Move_Forward(RUN_SPEED_15,RUN_SPEED_15);
			Motor_Check_Code=Check_Motor_Current();
			Wall_Dynamic_Base(300);
			delay(20);
			if(Motor_Check_Code)
			{
			  if(Self_Check(Motor_Check_Code))
				{
				  Set_Clean_Mode(Clean_Mode_Userinterface);
					return;
				}
				Initialize_Motor();
			}
			if(Get_Cliff_Trig()||Get_Bumper_Status())
			{
			  Move_Forward(0,0);
				delay(1000);
			  break;
			}
			if(Is_FrontOBS_Trig())
			{
				WFM_Left_move_back(RUN_SPEED_8,8*DISTANCE_1CM,900);
				Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
			  break;
			}
			
			#ifdef XP_WALL 
			if(Get_Xp_RWall_ADC()>600)
			#else
			if(Get_LWall_ADC()>600)
			#endif
			{
				Turn_Left(Turn_Speed,240);
				Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
			  break;
			}
     	#ifdef VIRTUAL_WALL
			if(Is_VirtualWall())break;
			#endif

			if(Touch_Detect())
			{
				Reset_Touch();
				Set_Clean_Mode(Clean_Mode_Userinterface);
				return;
			}
			if(Get_Rcon_Remote()!=0)
			{
				if(Remote_Key(Remote_Spot))
				{
				  if(Is_MoveWithRemote())
					{
					  Set_MoveWithRemote();
						Set_Clean_Mode(Clean_Mode_Spot);
						return;
					}
				}
				if(Remote_Key(Remote_Right))
				{
					Stop_Brifly();
					Turn_Right(Turn_Speed,320);
					Move_Forward(RUN_SPEED_5,RUN_SPEED_5);

				}
				if(Remote_Key(Remote_Left))
				{
					Stop_Brifly();
					Turn_Left(Turn_Speed,320);
					Move_Forward(RUN_SPEED_5,RUN_SPEED_5);

				}
				if(Remote_Key(Remote_Home))
				{
					Display_Content(LED_Home,100,100,0,7);
					Set_Clean_Mode(Clean_Mode_GoHome);
					SetHomeRemote();
					Reset_Rcon_Remote();
					return;
				}
			}
			if(Get_Rcon_Status()&Rcon_Signal_All_T)
			{
				Reset_Rcon_Status();
        break;
			}
			
			#ifdef BLDC_INSTALL
 
 
 
			if(Remote_Key(Remote_Max))
			{
				Switch_VacMode();
			}
			#endif
		}
	}
/*------------------------------------------------------Wall Follow ---------------------------------------------------------------------------------*/
	Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
  Reset_Rcon_Status();
	#ifdef MOBILITY
  Temp_Mobility_Distance = Get_Move_Distance();
	#endif
  Reset_Wheel_Step();
  Set_Mobility_Step(1000);
  Reset_Wall_Step();
	Reset_WallAccelerate();
	Wall_Straight_Distance=5*DISTANCE_1CM;
  while(1)
  { 
		#ifdef MOBILITY
    if(Get_LeftWheel_Step()<500)
    {
      Mobility_Temp_Error=0;
      Temp_Mobility_Distance = Get_Move_Distance();
    }
    else
    {
      if((Get_Move_Distance()-Temp_Mobility_Distance)>500)
      {
        Temp_Mobility_Distance = Get_Move_Distance();
        if(Get_Mobility_Step()<1)
        {
          Mobility_Temp_Error++;
          if(Mobility_Temp_Error>5)
          {
            Mob_Error_Add();
            if(Get_Mob_Error()<4)
            {
              Move_Back();
              Turn_Right(Turn_Speed,1200);
              Reset_Wheel_Step();
              Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
            }
            Mobility_Temp_Error=0;
            
          }
        }
        else
        {
          Mobility_Temp_Error=0;
        }
        Reset_Mobility_Step();
      }
    }
		#endif
	  /*------------------------------------------------------Check Current-----------------------*/
	   Motor_Check_Code=Check_Motor_Current();
		if(Motor_Check_Code)
		{
		  if(Self_Check(Motor_Check_Code))
			{
			  Set_Clean_Mode(Clean_Mode_Userinterface);
				break;
			}
			if(Is_MoveWithRemote())
			{
				Set_Clean_Mode(Clean_Mode_Userinterface);
			}
			if(RandomRemoteWall_Flag)
			{
				 Set_Clean_Mode(Clean_Mode_RandomMode);				 
			}
			Initialize_Motor();
      break;	  
		}
		/*------------------------------------------------------Touch and Remote event-----------------------*/
		if(Touch_Detect())
		{
		  Reset_Touch();
		  Set_Clean_Mode(Clean_Mode_Userinterface);
			break;
		}
		if(Get_Rcon_Remote()!=0)
		{
			if(Remote_Key(Remote_Spot))
			{
			  if(Is_MoveWithRemote())
				{
				  Set_MoveWithRemote();
					Set_Clean_Mode(Clean_Mode_Spot);
					break;
				}
				else
				{
					Vac_Mode_Buffer = Get_VacMode();
				  Temp_Dirt_Status=Random_Dirt_Event();
					Set_VacMode(Vac_Mode_Buffer);
					Set_VacMode(Vac_Normal);
					Set_Vac_Speed();
					if(Temp_Dirt_Status==1)
					{
						Set_Clean_Mode(Clean_Mode_Userinterface);
					}
					else
					{
						Set_Clean_Mode(Clean_Mode_RandomMode);			
					}
					Display_Content(LED_Clean,100,100,0,7);
					break;
				}
			}
			if(Remote_Key(Remote_Left))
			{
			  Turn_Left(Turn_Speed,560);
				Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
				if(!Is_MoveWithRemote())
				{
					Set_Clean_Mode(Clean_Mode_RandomMode);
					Display_Content(LED_Clean,100,100,0,7);
					break;
				}
				Reset_Rcon_Remote();
			}
			#ifdef BLDC_INSTALL 
			if(Remote_Key(Remote_Max))
			{
				Switch_VacMode();
			}
			#endif
			if(Remote_Key(Remote_Home))  
			{
					Display_Content(LED_Home,100,100,0,7);
					Set_Clean_Mode(Clean_Mode_GoHome);
          Reset_Rcon_Remote();
				  SetHomeRemote();
					return;
			}
		}
		/*------------------------------------------------------Check Battery-----------------------*/
		if(Check_Bat_SetMotors(Clean_Vac_Power,Clean_SideBrush_Power,Clean_MainBrush_Power))//Low Battery Event
    {
      Display_Battery_Status(Display_Low);//display low
      Set_Clean_Mode(Clean_Mode_Userinterface);
			break;
 		}	
		/*Run wall follow over 20minutes if random mode remote wallfollow*/
		
		if((Get_WorkTime()-WorkTime_Buffer)>2400)//wall follow over 20 minutes
		{
			if(RandomRemoteWall_Flag)
			{
				Set_Clean_Mode(Clean_Mode_RandomMode);
				break;
			}
			else
			{
				Set_Clean_Mode(Clean_Mode_GoHome);
				SetHomeRemote();
				break;
			}				
		}
		
		/*------------------------------------------------------Virtual Wall Event-----------------------*/
    Temp_Rcon_Status = Get_Rcon_Status();
    if(Temp_Rcon_Status)
    {
      Reset_Rcon_Status();
      if(Temp_Rcon_Status&Rcon_Signal_ALL)
      {
        if(Is_WorkFinish(Get_Room_Mode()))
        {
          Set_Clean_Mode(Clean_Mode_GoHome);
			    ResetHomeRemote();
    			break;
        }
      }
      if(Temp_Rcon_Status&Rcon_Signal_All_T)
      {
//        if(Is_MoveWithRemote())
//        {
//					if(Get_WorkTime()>2400)//wall follow over 20 minutes
//					{
//						Set_Clean_Mode(Clean_Mode_GoHome);
//						ResetHomeRemote();
//						break;
//					}
//        }
//				OBS_OFF();
				Stop_Brifly();
				if(Temp_Rcon_Status & RconFR_HomeT)
				{
					Turn_Left(Turn_Speed,1300);
				}
				else if(Temp_Rcon_Status & RconFL_HomeT)
				{
					Turn_Left(Turn_Speed,1200);
				}
				else if(Temp_Rcon_Status & RconL_HomeT)
				{
					Turn_Left(Turn_Speed,900);
				}
				else if(Temp_Rcon_Status & RconR_HomeT)
				{
					Turn_Left(Turn_Speed,1500);
				}
				Stop_Brifly();
				Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
				Reset_Rcon_Status();
				Wall_Straight_Distance=DISTANCE_1CM;
				Reset_WallAccelerate();
			}
			/*Virtual wall event*/
			#ifdef VIRTUAL_WALL
			if(VirtualWall_TurnRight(Temp_Rcon_Status))
			{
				OBS_OFF();
				Wall_Straight_Distance=3*DISTANCE_1CM;
				Reset_WallAccelerate();
			}
			#endif
    }  
		/*---------------------------------------------------Bumper Event-----------------------*/
		if(Get_Bumper_Status()&(LeftBumperTrig))
    {
			OBS_ON();
			WFM_Left_move_back(RUN_SPEED_8,5*DISTANCE_1CM,700);			
			Wall_Straight_Distance=5*DISTANCE_1CM;

		  if(Get_WallAccelerate()>100)
			{		
				if(Is_Bumper_Jamed())break;
			}
			Reset_WallAccelerate();	
      Reset_Wheel_Step();
			Wall_Distance=Wall_High_Limit;				
    }
					
		if(Get_Bumper_Status()&RightBumperTrig)
    {
			OBS_ON();
      L_B_Counter++;
			#ifdef XP_WALL
//			  if(Get_Xp_RWall_ADC()>(Wall_Low_Limit)){
//					Wall_Distance = Get_Xp_RWall_ADC()/2;//3
//				}
//				else {
//					Wall_Distance+=200;
//				}
			Wall_Distance-=20;
			#else
			  if(Get_RWall_ADC()>(Wall_Low_Limit)){
					Wall_Distance = Get_RWall_ADC()/1;
				}
				else {
					Wall_Distance+=200;
				}			
			#endif
				
			if((Get_Bumper_Status()&LeftBumperTrig)||Is_FrontOBS_Trig())
			{
				WFM_Left_move_back(RUN_SPEED_8,5*DISTANCE_1CM,700);				
				if(Is_Bumper_Jamed())break;;
				Wall_Straight_Distance=5*DISTANCE_1CM;
				Wall_Distance=Wall_High_Limit;					
			}		
			else//right
			{
//				Wall_Straight_Distance=5*DISTANCE_1CM;					
				Wall_Distance=Wall_Low_Limit;	
				if(Jam<2)
				{
					WFM_Right_move_back(RUN_SPEED_6,5*DISTANCE_1CM,500);
					if(Is_Bumper_Jamed())break;					
				}
				else
				{ 
					WFM_Left_move_back(RUN_SPEED_8,5*DISTANCE_1CM,700);
					if(Is_Bumper_Jamed())break;
				}									
			}
			
			if(Get_WallAccelerate()<1000)
			{
			  Jam++;
			}
			else
			{
				Jam=0;
			}
					
			Reset_WallAccelerate();			
      Move_Forward(RUN_SPEED_5,RUN_SPEED_5);			
			for (Temp_Counter = 0; Temp_Counter < 3; Temp_Counter++)
			{
			  Left_Wall_Buffer[Temp_Counter]=0;
			}	
      Reset_Wheel_Step();	
			USPRINTF("wall_dis:%d\n",Wall_Distance);	
			if (Wall_Distance < Wall_Low_Limit)Wall_Distance = Wall_Low_Limit;
			if(Wall_Distance>Wall_High_Limit)Wall_Distance=Wall_High_Limit;					
    }
		
		if(Get_WallAccelerate()>1000)
		{
			Jam=0;
		}   
		
		if(Jam>80)
		{
		  Set_Clean_Mode(Clean_Mode_Userinterface);
			Set_Error_Code(Error_Code_Stuck);
		  break;
		}
		if(L_B_Counter>10)
		{
			if(!Is_MoveWithRemote())
			{
				Set_Clean_Mode(Clean_Mode_RandomMode);
				break;
			}
		}

		/*------------------------------------------------------Cliff Event-----------------------*/
    if(Get_Cliff_Trig())
    {
			OBS_ON();
      Set_Wheel_Speed(0,0);
      Set_Dir_Backward();
	    delay(150);
//			if(Get_Cliff_Trig())
//			{
			  Cliff_Move_Back();
				if(Get_Cliff_Trig()==(Status_Cliff_All))
				{
					Set_Clean_Mode(Clean_Mode_Userinterface);
				  break;
				}
				if(Get_Cliff_Trig())
			  {
				  if(Cliff_Escape())
					{
					  Set_Clean_Mode(Clean_Mode_Userinterface);
				    break;
					}
				}
				if(Get_LeftWheel_Step()<12500)
				{
					Turn_Left(Turn_Speed,750);
					Stop_Brifly();
					Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
					Reset_WallAccelerate();
					Wall_Straight_Distance=5*DISTANCE_1CM;
				}
				else
				{
					if(Is_MoveWithRemote())Set_Clean_Mode(Clean_Mode_Userinterface);
					else Set_Clean_Mode(Clean_Mode_RandomMode);
					break;
				}
				for(Temp_Counter=0;Temp_Counter<3;Temp_Counter++)
				{
					Left_Wall_Buffer[Temp_Counter]=0;
				}	
				Reset_Wheel_Step();
//			}
    }

		cycle++;
		if(cycle>5)
		{
			cycle=0;
			if(Wall_Distance>=Wall_Low_Limit){//200
				Left_Wall_Buffer[2] = Left_Wall_Buffer[1];
				Left_Wall_Buffer[1] = Left_Wall_Buffer[0];				
				#ifdef XP_WALL
				Left_Wall_Buffer[0]=Get_Xp_RWall_ADC();
				#else
				Left_Wall_Buffer[0]=Get_RWall_ADC();
				#endif
				
				if (Left_Wall_Buffer[0] < 200) {
					if ((Left_Wall_Buffer[1] - Left_Wall_Buffer[0]) > (Wall_Distance / 25)) {
						if ((Left_Wall_Buffer[2] - Left_Wall_Buffer[1]) > (Wall_Distance / 25)) 
						{
							if (Get_WallAccelerate() > 300) {
								if ((Get_LeftWheel_Speed() - Get_RightWheel_Speed()) >= -3) {
									Move_Forward(RUN_SPEED_9, RUN_SPEED_9);
									delay(10);
									Reset_WallAccelerate();
									Wall_Straight_Distance = 6*DISTANCE_1CM;
								}
							}
						}
					}
				}
			}	

			/*------------------------------------------------------Short Distance Move-----------------------*/
			if (Get_WallAccelerate() < (uint32_t) Wall_Straight_Distance)
			{
				if (Get_RightWheel_Step() < 500) {
					if (Get_WallAccelerate() < 100) {
						Move_Forward(RUN_SPEED_5, RUN_SPEED_5);
					} else {
						Move_Forward(RUN_SPEED_9, RUN_SPEED_9);
					}
				} else {
					Move_Forward(RUN_SPEED_10, RUN_SPEED_10);
				}			
			}
			else
			{
				/*------------------------------------------------------Wheel Speed adjustment-----------------------*/
				if(!Is_FrontOBS_Trig())
				{
					#ifdef XP_WALL
					Proportion = Get_Xp_RWall_ADC();
					#else
					Proportion = Get_RWall_ADC();
					#endif
					
					Proportion = Proportion * 100 / Wall_Distance;	
					Proportion -= 100;
					
					Delta = Proportion - Previous;
					
					Delta /= 3;

					if (Wall_Distance > 300)//over left
					{
						Right_Wall_Speed = RUN_SPEED_12 + Proportion / 12 + Delta / 5;
						Left_Wall_Speed = RUN_SPEED_12 - Proportion / 10 - Delta / 5;
						if (Left_Wall_Speed > RUN_SPEED_14) {
//							Set_LED(0,0,0);
							Right_Wall_Speed = RUN_SPEED_4;
							Left_Wall_Speed = RUN_SPEED_14;
						}//else Set_LED(100,0,0);
					} else if (Wall_Distance > 200){//over left
						Right_Wall_Speed  = RUN_SPEED_10 + Proportion / 15 + Delta / 7;
						Left_Wall_Speed = RUN_SPEED_10 - Proportion / 12 - Delta / 7;

						if (Left_Wall_Speed > RUN_SPEED_12) {
//							Set_LED(0,0,0);
							Right_Wall_Speed = RUN_SPEED_3;
							Left_Wall_Speed = RUN_SPEED_13;
						}//else Set_LED(100,0,0);
					}
					else{
						Right_Wall_Speed  = RUN_SPEED_8 + Proportion / 18 + Delta / 10;
						Left_Wall_Speed = RUN_SPEED_8 - Proportion / 15 - Delta / 10;
						if(Right_Wall_Speed > RUN_SPEED_10)Right_Wall_Speed = RUN_SPEED_10;
						if(Left_Wall_Speed > RUN_SPEED_9) {
//							Set_LED(0,0,0);
							Right_Wall_Speed = RUN_SPEED_3;
							Left_Wall_Speed = RUN_SPEED_10;
						}//else Set_LED(100,0,0);

	//					if(Left_Wall_Speed>20)Left_Wall_Speed=20;
	//					if(Left_Wall_Speed<4)Left_Wall_Speed=4;
	//					if(Right_Wall_Speed<4)Right_Wall_Speed=4;
	//					if((Left_Wall_Speed-Right_Wall_Speed)>5)
	//					{
	//						Left_Wall_Speed = Right_Wall_Speed+5;
	//					}
					}
					/*slow move if left obs near a wall*/
					if(Get_RWall_ADC()>1200)
					{
						Wall_Distance+=20;
						if(Wall_Distance>Wall_High_Limit*2)Wall_Distance=Wall_High_Limit*2;
					}
					else if(Get_RWall_ADC()>Wall_High_Limit/2)
					{
						Wall_Distance+=10;
						if(Wall_Distance>Wall_High_Limit)Wall_Distance=Wall_High_Limit;
					}
					else if(Get_RWall_ADC()>Wall_Low_Limit)
					{
						Wall_Distance+=5;
						if(Wall_Distance>Wall_High_Limit/2)Wall_Distance=Wall_High_Limit/2;
					}
					if(Is_WallOBS_Near()){
							Left_Wall_Speed = Left_Wall_Speed/2;
							Right_Wall_Speed = Right_Wall_Speed/2;
					}

					Previous = Proportion;

					if (Left_Wall_Speed < 0) {
						Left_Wall_Speed = 0;
					}
					if (Left_Wall_Speed > RUN_SPEED_15) {
						Left_Wall_Speed = RUN_SPEED_15;
					}
					if (Right_Wall_Speed < 0) {
						Right_Wall_Speed = 0;
					}

	 
					Move_Forward(Left_Wall_Speed, Right_Wall_Speed);
						
		
					if(Get_LeftWall_Step()>Get_RightWall_Step())R=Get_LeftWall_Step()-Get_RightWall_Step();
				 
					if(R>TURN_AROUND_CNT)
					{
						if(RandomRemoteWall_Flag)
						{
							Set_Clean_Mode(Clean_Mode_RandomMode);
							break;
						}
						else
						{
							Set_Clean_Mode(Clean_Mode_WallFollow);
						}
						break;
					}

					if(Get_WallAccelerate()>750)
					{
						Set_Left_Brush(ENABLE);
						Set_Right_Brush(ENABLE);
					}
				}
				else
				{
					Stop_Brifly();
					WFM_Left_move_back(RUN_SPEED_8,8*DISTANCE_1CM,900);
					if(Get_LeftWheel_Step()<12500)
					{
						if(Is_FrontOBS_Trig())
						{
							if(Get_WallAccelerate()<2000)
							{
								Jam++;
							}
//							Turn_Left(Turn_Speed,880);
						}
						else
						{
//							Turn_Left(Turn_Speed,400);
						}
					}
//					else
//					{
//						Turn_Left(Turn_Speed,900);
//					}
					Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
					Reset_Wheel_Step();	
					Wall_Distance = Wall_High_Limit;
				}
			}
			
		}//cycle

  }
}

void Set_WallFollow_Distance(int32_t Dis)
{
  Wall_Follow_Distance = Dis;
}

//void Wall_Follow(void)
//{
//	volatile uint8_t Motor_Check_Code = 0;
//  int32_t Proportion=0;
//	int32_t Delta=0;
//	int32_t Previous=0;
//  volatile int32_t Wall_Distance=400;
//	volatile int32_t Wall_Straight_Distance=DISTANCE_1CM;
//	volatile int32_t Left_Wall_Speed=0,Right_Wall_Speed=0;
//  volatile uint32_t L_B_Counter=0;
//	uint8_t Temp_Counter=0;
//	uint8_t Jam=0;
//	int32_t R=0;
//	uint8_t Temp_Dirt_Status=0;
////	int32_t Wall_Angle=0;
//	int16_t Left_Wall_Buffer[3]={0};
//	#ifdef MOBILITY
//  uint32_t Temp_Mobility_Distance=0;
//  uint8_t Mobility_Temp_Error=0;
//	#endif
//	static uint8_t Vac_Mode_Buffer=0;
//  uint32_t Temp_Rcon_Status = 0;
//	uint8_t RandomRemoteWall_Flag=0;
//	uint32_t WorkTime_Buffer=0;
//	
//	Reset_Bumper_Error();
//	Work_Motor_Configure();  
//	Set_RightWheel_Speed(15);
//  Reset_Wall_Step();
//	Move_Forward(RUN_SPEED_9,RUN_SPEED_9);
//  Display_Clean_Status(Display_Wall);
//	Reset_Touch();
//  Reset_Rcon_Remote();
//	Reset_Average_Counter();
//	Set_Vac_Speed();


//	if(Is_MoveWithRemote()||Is_RemoteRandomWall())
//	{
//		if(Is_RemoteRandomWall())
//		{
//			WorkTime_Buffer = Get_WorkTime();
//			RandomRemoteWall_Flag=1;
//			Reset_RemoteRandomWall();
//		}
//		while(1)
//		{
//		  Move_Forward(RUN_SPEED_9,RUN_SPEED_9);
//			Motor_Check_Code=Check_Motor_Current();
//			Wall_Dynamic_Base(300);
//			delay(20);
//			if(Motor_Check_Code)
//			{
//			  if(Self_Check(Motor_Check_Code))
//				{
//				  Set_Clean_Mode(Clean_Mode_Userinterface);
//					return;
//				}
//				Initialize_Motor();
//			}
//			if(Get_Cliff_Trig()||Get_Bumper_Status())
//			{
//			  Move_Forward(0,0);
//				delay(1000);
//			  break;
//			}
//			if(Is_FrontOBS_Trig())
//			{
//				Turn_Right(Turn_Speed,880);
//				Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
//			  break;
//			}

//			if(Get_LWall_ADC()>600)
//			{
//				Turn_Right(Turn_Speed,240);
//				Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
//			  break;
//			}
//     	#ifdef VIRTUAL_WALL
//			if(Is_VirtualWall())break;
//			#endif

//			if(Touch_Detect())
//			{
//				Reset_Touch();
//				Set_Clean_Mode(Clean_Mode_Userinterface);
//				return;
//			}
//			if(Get_Rcon_Remote()!=0)
//			{
//				if(Remote_Key(Remote_Spot))
//				{
//				  if(Is_MoveWithRemote())
//					{
//					  Set_MoveWithRemote();
//						Set_Clean_Mode(Clean_Mode_Spot);
//						return;
//					}
//				}
//				if(Remote_Key(Remote_Right))
//				{
//					Stop_Brifly();
//					Turn_Right(Turn_Speed,320);
//					Move_Forward(RUN_SPEED_5,RUN_SPEED_5);

//				}
//				if(Remote_Key(Remote_Left))
//				{
//					Stop_Brifly();
//					Turn_Left(Turn_Speed,320);
//					Move_Forward(RUN_SPEED_5,RUN_SPEED_5);

//				}
//				if(Remote_Key(Remote_Home))
//				{
//					Display_Content(LED_Home,100,100,0,7);
//					Set_Clean_Mode(Clean_Mode_GoHome);
//					SetHomeRemote();
//					Reset_Rcon_Remote();
//					return;
//				}
//			}
//			if(Get_Rcon_Status()&Rcon_Signal_All_T)
//			{
//				Reset_Rcon_Status();
//        break;
//			}
//			
//			#ifdef BLDC_INSTALL
// 
// 
// 
//			if(Remote_Key(Remote_Max))
//			{
//				Switch_VacMode();
//			}
//			#endif
//		}
//	}
///*------------------------------------------------------Wall Follow ---------------------------------------------------------------------------------*/
//	Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
//  Reset_Rcon_Status();
//	#ifdef MOBILITY
//  Temp_Mobility_Distance = Get_Move_Distance();
//	#endif
//  Reset_Wheel_Step();
//  Set_Mobility_Step(1000);
//  Reset_Wall_Step();
//	Reset_WallAccelerate();
//	Wall_Straight_Distance=5*DISTANCE_1CM;
//  while(1)
//  { 
//		#ifdef MOBILITY
//    if(Get_LeftWheel_Step()<500)
//    {
//      Mobility_Temp_Error=0;
//      Temp_Mobility_Distance = Get_Move_Distance();
//    }
//    else
//    {
//      if((Get_Move_Distance()-Temp_Mobility_Distance)>500)
//      {
//        Temp_Mobility_Distance = Get_Move_Distance();
//        if(Get_Mobility_Step()<1)
//        {
//          Mobility_Temp_Error++;
//          if(Mobility_Temp_Error>5)
//          {
//            Mob_Error_Add();
//            if(Get_Mob_Error()<4)
//            {
//              Move_Back();
//              Turn_Right(Turn_Speed,1200);
//              Reset_Wheel_Step();
//              Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
//            }
//            Mobility_Temp_Error=0;
//            
//          }
//        }
//        else
//        {
//          Mobility_Temp_Error=0;
//        }
//        Reset_Mobility_Step();
//      }
//    }
//		#endif
//	  /*------------------------------------------------------Check Current-----------------------*/
//	   Motor_Check_Code=Check_Motor_Current();
//		if(Motor_Check_Code)
//		{
//		  if(Self_Check(Motor_Check_Code))
//			{
//			  Set_Clean_Mode(Clean_Mode_Userinterface);
//				break;
//			}
//			if(Is_MoveWithRemote())
//			{
//				Set_Clean_Mode(Clean_Mode_Userinterface);
//			}
//			if(RandomRemoteWall_Flag)
//			{
//				 Set_Clean_Mode(Clean_Mode_RandomMode);				 
//			}
//			Initialize_Motor();
//      break;	  
//		}
//		/*------------------------------------------------------Touch and Remote event-----------------------*/
//		if(Touch_Detect())
//		{
//		  Reset_Touch();
//		  Set_Clean_Mode(Clean_Mode_Userinterface);
//			break;
//		}
//		if(Get_Rcon_Remote()!=0)
//		{
//			if(Remote_Key(Remote_Spot))
//			{
//			  if(Is_MoveWithRemote())
//				{
//				  Set_MoveWithRemote();
//					Set_Clean_Mode(Clean_Mode_Spot);
//					break;
//				}
//				else
//				{
//					Vac_Mode_Buffer = Get_VacMode();
//				  Temp_Dirt_Status=Random_Dirt_Event();
//					Set_VacMode(Vac_Mode_Buffer);
//					Set_VacMode(Vac_Normal);
//					Set_Vac_Speed();
//					if(Temp_Dirt_Status==1)
//					{
//						Set_Clean_Mode(Clean_Mode_Userinterface);
//					}
//					else
//					{
//						Set_Clean_Mode(Clean_Mode_RandomMode);			
//					}
//					Display_Content(LED_Clean,100,100,0,7);
//					break;
//				}
//			}
//			if(Remote_Key(Remote_Right))
//			{
//			  Turn_Right(Turn_Speed,560);
//				Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
//				if(!Is_MoveWithRemote())
//				{
//					Set_Clean_Mode(Clean_Mode_RandomMode);
//					Display_Content(LED_Clean,100,100,0,7);
//					break;
//				}
//				Reset_Rcon_Remote();
//			}
//			#ifdef BLDC_INSTALL 
//			if(Remote_Key(Remote_Max))
//			{
//				Switch_VacMode();
//			}
//			#endif
//			if(Remote_Key(Remote_Home))  
//			{
//					Display_Content(LED_Home,100,100,0,7);
//					Set_Clean_Mode(Clean_Mode_GoHome);
//          Reset_Rcon_Remote();
//				  SetHomeRemote();
//					return;
//			}
//		}
//		/*------------------------------------------------------Check Battery-----------------------*/
//		if(Check_Bat_SetMotors(Clean_Vac_Power,Clean_SideBrush_Power,Clean_MainBrush_Power))//Low Battery Event
//    {
//      Display_Battery_Status(Display_Low);//display low
//      Set_Clean_Mode(Clean_Mode_Userinterface);
//			break;
// 		}	
//		/*Run wall follow over 20minutes if random mode remote wallfollow*/
//		
//		if((Get_WorkTime()-WorkTime_Buffer)>2400)//wall follow over 20 minutes
//		{
//			if(RandomRemoteWall_Flag)
//			{
//				Set_Clean_Mode(Clean_Mode_RandomMode);
//				break;
//			}
//			else
//			{
//				Set_Clean_Mode(Clean_Mode_GoHome);
//				SetHomeRemote();
//				break;
//			}				
//		}
//		
//		/*------------------------------------------------------Virtual Wall Event-----------------------*/
//    Temp_Rcon_Status = Get_Rcon_Status();
//    if(Temp_Rcon_Status)
//    {
//      Reset_Rcon_Status();
//      if(Temp_Rcon_Status&Rcon_Signal_ALL)
//      {
//        if(Is_WorkFinish(Get_Room_Mode()))
//        {
//          Set_Clean_Mode(Clean_Mode_GoHome);
//			    ResetHomeRemote();
//    			break;
//        }
//      }
//      if(Temp_Rcon_Status&Rcon_Signal_All_T)
//      {
////        if(Is_MoveWithRemote())
////        {
////					if(Get_WorkTime()>2400)//wall follow over 20 minutes
////					{
////						Set_Clean_Mode(Clean_Mode_GoHome);
////						ResetHomeRemote();
////						break;
////					}
////        }
////				OBS_OFF();
//				Stop_Brifly();
//				if(Temp_Rcon_Status & RconFR_HomeT)
//				{
//					Turn_Right(Turn_Speed,1300);
//				}
//				else if(Temp_Rcon_Status & RconFL_HomeT)
//				{
//					Turn_Right(Turn_Speed,1200);
//				}
//				else if(Temp_Rcon_Status & RconL_HomeT)
//				{
//					Turn_Right(Turn_Speed,900);
//				}
//				else if(Temp_Rcon_Status & RconR_HomeT)
//				{
//					Turn_Right(Turn_Speed,1500);
//				}
//				Stop_Brifly();
//				Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
//				Reset_Rcon_Status();
//				Wall_Straight_Distance=DISTANCE_1CM;
//				Reset_WallAccelerate();
//			}
//			/*Virtual wall event*/
//			#ifdef VIRTUAL_WALL
//			if(VirtualWall_TurnRight(Temp_Rcon_Status))
//			{
//				OBS_OFF();
//				Wall_Straight_Distance=3*DISTANCE_1CM;
//				Reset_WallAccelerate();
//			}
//			#endif
//    }  
//		/*---------------------------------------------------Bumper Event-----------------------*/
//    if(Get_Bumper_Status()&RightBumperTrig)
//    {
//			OBS_ON();
//			if(Get_WallAccelerate()>2000)
//			{
//				Jam=0;
//			}
//			
//			Stop_Brifly();
//			Wall_Move_Back();
//		  if(Get_WallAccelerate()>100)
//			{		
//				if(Is_Bumper_Jamed())break;
//			}
//			Turn_Right(Turn_Speed,900);
//			Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
//			Reset_WallAccelerate();
//			Wall_Straight_Distance=5*DISTANCE_1CM;
//      Reset_Wheel_Step();		
//    }

////		USART_Print("\n\r Jam ",Dev_USART3);
////		delay(10);
////		USART3_Transmit_Numbers(Jam);		
////		delay(10);
////		USART3_Transmit_Numbers(Get_WallAccelerate());
//		
//		if(Get_Bumper_Status()&LeftBumperTrig)
//    {
//			OBS_ON();
//      L_B_Counter++;
//      Set_Wheel_Speed(0,0);
//      Reset_TempPWM();
//      delay(300);
//      if(Get_LWall_ADC()>(Wall_Low_Limit)){
//				Wall_Distance = Get_LWall_ADC()/3;
//			}
//			else {
//					Wall_Distance+=200;
//			}

//			if (Wall_Distance < Wall_Low_Limit)Wall_Distance = Wall_Low_Limit;
//			if(Wall_Distance>Wall_High_Limit)Wall_Distance=Wall_High_Limit;			
//			
//			if(Get_Bumper_Status()&RightBumperTrig)
//			{
//				Move_Back();
//				if(Is_Bumper_Jamed())break;;
//				Turn_Right(Turn_Speed-5,600);
//				Wall_Straight_Distance=3*DISTANCE_1CM;
//			}
//			else
//			{
//				Wall_Move_Back();
//				if(Is_Bumper_Jamed())break;
//				if(Jam<3)
//				{
//					if(Wall_Distance<200)
//					{
//						if(Get_LeftOBS()>(Get_LeftOBST_Value()-200)){
//							Wall_Distance=Wall_High_Limit;
//							Turn_Right(Turn_Speed - 10, 300);
//						}
//						else{
//							Turn_Right(Turn_Speed - 10, 150);
//						}
//					}
//					else
//					{
//						Turn_Right(Turn_Speed - 10, 300);
//					}
//				}
//				else
//				{
//					Turn_Right(Turn_Speed-10,150);
//				}
//				Wall_Straight_Distance=5*DISTANCE_1CM;
//			}

//			
//			if(Get_WallAccelerate()<2000)
//			{
//			  Jam++;
//			}
//			else
//			{
//				Jam=0;
//			}
//		
//			
//			
//			
//			Reset_WallAccelerate();

//      Move_Forward(RUN_SPEED_5,RUN_SPEED_5);

//			for (Temp_Counter = 0; Temp_Counter < 3; Temp_Counter++)
//			{
//			  Left_Wall_Buffer[Temp_Counter]=0;
//			}	
//      Reset_Wheel_Step();	
//    }
//    

//		if(Jam>80)
//		{
//		  Set_Clean_Mode(Clean_Mode_Userinterface);
//			Set_Error_Code(Error_Code_Stuck);
//		  break;
//		}
//		if(L_B_Counter>10)
//		{
//			if(!Is_MoveWithRemote())
//			{
//				Set_Clean_Mode(Clean_Mode_RandomMode);
//				break;
//			}
//		}

//		/*------------------------------------------------------Cliff Event-----------------------*/
//    if(Get_Cliff_Trig())
//    {
//			OBS_ON();
//      Set_Wheel_Speed(0,0);
//      Set_Dir_Backward();
//	    delay(150);
////			if(Get_Cliff_Trig())
////			{
//			  Cliff_Move_Back();
//				if(Get_Cliff_Trig()==(Status_Cliff_All))
//				{
//					Set_Clean_Mode(Clean_Mode_Userinterface);
//					//Set_Error_Code(Error_Code_Cliff);
//				  break;
//				}
//				if(Get_Cliff_Trig())
//			  {
//				  if(Cliff_Escape())
//					{
//					  Set_Clean_Mode(Clean_Mode_Userinterface);
//						//Set_Error_Code(Error_Code_Cliff);
//				    break;
//					}
//				}
//				if(Get_LeftWheel_Step()<12500)
//				{
//					Turn_Right(Turn_Speed-10,750);
//					Stop_Brifly();
//					Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
//					Reset_WallAccelerate();
//					Wall_Straight_Distance=5*DISTANCE_1CM;
//				}
//				else
//				{
//					if(Is_MoveWithRemote())Set_Clean_Mode(Clean_Mode_Userinterface);
//					else Set_Clean_Mode(Clean_Mode_RandomMode);
//					break;
//				}
//				for(Temp_Counter=0;Temp_Counter<3;Temp_Counter++)
//				{
//					Left_Wall_Buffer[Temp_Counter]=0;
//				}	
//				Reset_Wheel_Step();
////			}
//    }
////		if(St)
////		{
////			Buffer_Array[DD]=OBS_Status.Left_Wall;
////			DD++;
////			End++;
////			if(DD>999)St=0;
////		}
//		
//		//delta_counter++;
//		//if(delta_counter>10)

//		//  delta_counter=0;
//		if(Wall_Distance>=200)
//		{
//			Left_Wall_Buffer[2]=Left_Wall_Buffer[1];
//			Left_Wall_Buffer[1]=Left_Wall_Buffer[0];
//			Left_Wall_Buffer[0]=Get_LWall_ADC();
//			if(Left_Wall_Buffer[0]<100)
//			{
//			  if((Left_Wall_Buffer[1]-Left_Wall_Buffer[0])>(Wall_Distance/25))
//				{
//				  if((Left_Wall_Buffer[2]-Left_Wall_Buffer[1])>(Wall_Distance/25))
//					{
//					  if(Get_WallAccelerate()>300)
//						{
//						  if((Get_RightWheel_Speed()-Get_LeftWheel_Speed())>=-3)
//							{
//								Move_Forward(RUN_SPEED_9,RUN_SPEED_9);
//								delay(1000);
//								Reset_WallAccelerate();
//								Wall_Straight_Distance=5*DISTANCE_1CM;
//							}
//						}
//					}
//				}
//			}
//		}
//		
//		/*------------------------------------------------------Short Distance Move-----------------------*/
//		if (Get_WallAccelerate() < (uint32_t) Wall_Straight_Distance)
//		{
//      if(Get_LeftWheel_Step()<500)
//      {
//        if(Get_WallAccelerate()<100)
//  			{
//  			  Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
//  			}
//  			else
//  			{
//  			  Move_Forward(RUN_SPEED_9,RUN_SPEED_9);
//  			}
//      }
//      else
//      {
//        Move_Forward(ROTATE_TOP_SPEED_10,ROTATE_TOP_SPEED_10);
//      }
//		}
//		else
//		{
//			/*------------------------------------------------------Wheel Speed adjustment-----------------------*/
//	    if(!Is_FrontOBS_Trig())
//	    {
//		    Proportion = Get_LWall_ADC();

//				Proportion = Proportion * 100 / Wall_Distance;

//				Proportion -= 100;

//				Delta = Proportion - Previous;

//					if (Wall_Distance > 300) {	//over left
//						Left_Wall_Speed  = RUN_SPEED_10 + Proportion / 12 + Delta / 5;
//						Right_Wall_Speed = RUN_SPEED_10 - Proportion / 10 - Delta / 5;
//						if (Right_Wall_Speed > RUN_SPEED_12) {
//							Left_Wall_Speed = RUN_SPEED_4; 
//							Right_Wall_Speed = RUN_SPEED_12;
//						}
//					} else if (0 && Wall_Distance > 150){	//over left
//						Left_Wall_Speed  = RUN_SPEED_8 + Proportion / 15 + Delta / 7;
//						Right_Wall_Speed = RUN_SPEED_8 - Proportion / 12 - Delta / 7;

//						if (Right_Wall_Speed > RUN_SPEED_10) {
//							Left_Wall_Speed  = RUN_SPEED_3; //8;
//							Right_Wall_Speed = RUN_SPEED_10;
//						}
//					} else {
//						Left_Wall_Speed  = RUN_SPEED_6 + Proportion / 22 + Delta / 10;
//						Right_Wall_Speed = RUN_SPEED_6 - Proportion / 18 - Delta / 10;

//						if (Right_Wall_Speed > RUN_SPEED_8) {
//							Left_Wall_Speed  = RUN_SPEED_3;
//							Right_Wall_Speed = RUN_SPEED_8;
//						}

////					if(Left_Wall_Speed>20)Left_Wall_Speed=20;
////					if(Left_Wall_Speed<4)Left_Wall_Speed=4;
////					if(Right_Wall_Speed<4)Right_Wall_Speed=4;
////					if((Left_Wall_Speed-Right_Wall_Speed)>5)
////					{
////						Left_Wall_Speed = Right_Wall_Speed+5;
////					}
//				}
//			  /*slow move if left obs near a wall*/
//				if(Get_LeftOBS()>Get_LeftOBST_Value()){
//					if(Wall_Distance<Wall_High_Limit)Wall_Distance++;
//				}
//				if(Is_WallOBS_Near()){
//						Left_Wall_Speed = Left_Wall_Speed/2;
//						Right_Wall_Speed = Right_Wall_Speed/2;
//				}

//				Previous = Proportion;

//				if (Left_Wall_Speed < 0) {
//					Left_Wall_Speed = 0;
//				}
//				if (Left_Wall_Speed > RUN_SPEED_17) {
//					Left_Wall_Speed = RUN_SPEED_17;
//				}
//				if (Right_Wall_Speed < 0) {
//					Right_Wall_Speed = 0;
//				}

// 
//				Move_Forward(Left_Wall_Speed, Right_Wall_Speed);
//					
//	
//	      if(Get_RightWall_Step()>Get_LeftWall_Step())R=Get_RightWall_Step()-Get_LeftWall_Step();
//			 
//	      if(R>TURN_AROUND_CNT)
//	      {
//					if(RandomRemoteWall_Flag)
//					{
//						Set_Clean_Mode(Clean_Mode_RandomMode);
//						break;
//					}
//					else
//					{
//						Set_Clean_Mode(Clean_Mode_WallFollow);
//					}
//	        break;
//	      }
////	      if((Get_RightWall_Step()>Wall_Follow_Distance)||(Get_LeftWall_Step()>Wall_Follow_Distance))//about 5 Meter    6000 = 1 m
////	      {
////					Reset_Wall_Step();
////					if(!Is_MoveWithRemote())
////          {
////            Set_Clean_Mode(Clean_Mode_RandomMode);
////            break;
////          }
////	      }
//				if(Get_WallAccelerate()>750)
//				{
//					Set_Left_Brush(ENABLE);
//			    Set_Right_Brush(ENABLE);
//				}
//	    }
//	    else
//	    {
//        Stop_Brifly();
//			  if(Get_LeftWheel_Step()<12500)
//				{
//					if(Is_FrontOBS_Trig())
//					{
//					  if(Get_WallAccelerate()<2000)
//						{
//							Jam++;
//						}
//						Turn_Right(Turn_Speed,880);
//				    Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
//					}
//					else
//					{
//				    Turn_Right(Turn_Speed,400);
//				    Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
//					}
//				}
//				else
//				{
//				  Turn_Right(Turn_Speed,900);
//			    Move_Forward(RUN_SPEED_5,RUN_SPEED_5);
//				 //	if(!Is_MoveWithRemote())Set_Clean_Mode(Clean_Mode_RandomMode);
//				}
//        Reset_Wheel_Step();	
//				Wall_Distance = Wall_High_Limit;
//	    }
//		}
//  }
//}


