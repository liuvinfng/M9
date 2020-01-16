#include "ShortestPath.h"
#include "PathPlanning.h"
#include "config.h"
#include "Map.h"
#include "Zone.h"
#include "CorMove.h"
#include "Gyro.h"

extern PositionType positions[];
extern int16_t xMin, xMax, yMin, yMax;
extern int16_t xMinSearch, xMaxSearch, yMinSearch, yMaxSearch;

volatile LineType	pos_line[POS_LINE_CNT];
volatile uint16_t line_cnt=0;
volatile int16_t Line_xMax=0,Line_yMax=0;

volatile Point16_t Next_Pos[NEXT_POS_SIZE];
volatile uint16_t next_pos_cnt=0;

volatile uint16_t ShortestPath_Time_Counter=0;
volatile uint8_t ShortestPath_Time_Over=0;
volatile Point16_t ShortestPath_Last_Cell;

volatile uint8_t next_move_step=1;
volatile uint8_t target_find=0;
volatile Point16_t Target_Cell;

volatile uint8_t find_global=0;

void Set_Search_Range(uint8_t value)
{
	find_global = value;
}

uint8_t Get_Search_Range(void)
{
	return find_global;
}

void Set_Target_Cell(Point16_t pos)
{
	Target_Cell = pos;
}

Point16_t Get_Target_Cell(void)
{
	return Target_Cell;
}

int16_t ABS(int16_t A, int16_t B)
{
	int16_t temp = A - B;
	
	if(temp<0)temp = -temp;
	
	return temp;
}

void Reset_Next_Pos(void)
{
	uint16_t i=0;
	
	for(i=0;i<NEXT_POS_SIZE;i++)
	{
		Next_Pos[i].X = 0;
		Next_Pos[i].Y = 0;
	}
	next_pos_cnt = 0;
}

Point16_t Get_Next_Pos_From_Id(uint16_t id)
{	
	return Next_Pos[id];
}

Point16_t Get_Next_Pos(void)
{
	return Next_Pos[0];
}

uint16_t Get_Next_Pos_Count(void)
{
	return next_pos_cnt;	
}

void Set_Next_Pos_Count(uint16_t id)
{
	next_pos_cnt = id;
}

void Delete_Next_Pos(uint16_t id)
{
	uint16_t i=Get_Next_Pos_Count();
	
	if(id<i)
	{
		while(id<i)
		{
			Next_Pos[id] = Next_Pos[id+1];
			id++;
		}
		if(Get_Next_Pos_Count()>0)
		{
			Set_Next_Pos_Count(Get_Next_Pos_Count()-1);
		}
		else
		{
			Set_Next_Pos_Count(0);
		}
	}
}

void Set_Next_Pos(uint16_t id,Point16_t pos)
{
	Next_Pos[id] = pos;
}

//每两个相邻的点有且只有一个X或者Y相同
Point16_t Updata_Next_Pos(Point16_t pos)
{
	uint16_t i=Get_Next_Pos_Count();
	uint8_t x=0,y=0;
	Point16_t temp_pos=pos;
	
	if(next_pos_cnt>1)
	{
		for(i=0;i<(Get_Next_Pos_Count()-1);i++)
		{
			if((Next_Pos[i].X==Next_Pos[i+1].X)&&(Next_Pos[i].Y==Next_Pos[i+1].Y))
			{
				Delete_Next_Pos(i);
				x = 0;
				y = 0;
			}
			else
			{
				if(Next_Pos[i].X==Next_Pos[i+1].X)
				{
					x++;
					if(y>0)y--;
					
					if(x>1)
					{
						Delete_Next_Pos(i);
						x = 0;
						y = 0;
					}					
				}
				else if(Next_Pos[i].Y==Next_Pos[i+1].Y)
				{
					y++;
					if(x>0)x--;
					if(y>1)
					{
						Delete_Next_Pos(i);
						x = 0;
						y = 0;
					}	
				}
				else
				{
					USPRINTF("%s %d: Next_Pos Error!!!\n", __FUNCTION__, __LINE__);
				}
			}
		}
	}
	
	if(next_pos_cnt>0)
	{
		temp_pos = Get_Next_Pos();
		Delete_Next_Pos(0);			
		while((temp_pos.X==pos.X)&&(temp_pos.Y==pos.Y))
		{
			if(Get_Next_Pos_Count()>0)
			{
				temp_pos = Get_Next_Pos();
				Delete_Next_Pos(0);	
			}
			else
			{
				break;
			}
		}		
	}
	
	return temp_pos;
}

void FAdd_Next_Pos(Point16_t next)
{
	uint8_t x=0,y=0;
	
	int16_t i=Get_Next_Pos_Count();
	
	if(Get_Next_Pos_Count()<(NEXT_POS_SIZE-2))
	{
		next_pos_cnt++;
	}	
	
	while(i)
	{
		Next_Pos[i] = Next_Pos[i-1];
		i--;
	}
	Next_Pos[0] = next;	
	
	if(next_pos_cnt>1)
	{
		for(i=0;i<(Get_Next_Pos_Count()-1);i++)
		{
			if((Next_Pos[i].X==Next_Pos[i+1].X)&&(Next_Pos[i].Y==Next_Pos[i+1].Y))
			{
				Delete_Next_Pos(i);
				x = 0;
				y = 0;
			}
			else
			{
				if(Next_Pos[i].X==Next_Pos[i+1].X)
				{
					x++;
					if(y>0)y--;
					
					if(x>1)
					{
						Delete_Next_Pos(i);
						x = 0;
						y = 0;
					}
					
				}
				else if(Next_Pos[i].Y==Next_Pos[i+1].Y)
				{
					y++;
					if(x>0)x--;
					if(y>1)
					{
						Delete_Next_Pos(i);
						x = 0;
						y = 0;
					}	
				}
				else
				{
					USPRINTF("%s %d: Next_Pos Error!!!\n", __FUNCTION__, __LINE__);
				}
			}
		}
	}
}



void Add_Next_Pos(Point16_t next)
{
	uint8_t x=0,y=0,i=0;
	
	Next_Pos[next_pos_cnt] = next;
	
	if(Get_Next_Pos_Count()<(NEXT_POS_SIZE-2))
	{
		next_pos_cnt++;
	}
	
	if(Get_Next_Pos_Count()>2)
	{
		for(i=0;i<(Get_Next_Pos_Count()-1);i++)
		{
			if((Next_Pos[i].X==Next_Pos[i+1].X)&&(Next_Pos[i].Y==Next_Pos[i+1].Y))
			{
				Delete_Next_Pos(i);
				x = 0;
				y = 0;
			}
			else
			{
				if(Next_Pos[i].X==Next_Pos[i+1].X)
				{
					x++;
					if(y>0)y--;
					
					if(x>1)
					{
						Delete_Next_Pos(i);
						x = 0;
						y = 0;
					}
					
				}
				else if(Next_Pos[i].Y==Next_Pos[i+1].Y)
				{
					y++;
					if(x>0)x--;
					if(y>1)
					{
						Delete_Next_Pos(i);
						x = 0;
						y = 0;
					}	
				}
				else
				{
					USPRINTF("%s %d: Next_Pos Error!!!\n", __FUNCTION__, __LINE__);
				}
			}
		}
	}
}

//------------------------------------------------------------------------
uint16_t Get_Path_Line_Count(void)
{
	return line_cnt;
}

void Set_Path_Line_Count(uint16_t id)
{
	if(id<=POS_LINE_CNT)
	{
		line_cnt = id;
	}
}
//为了省内存---缺点地图格子大小不能超过255*255
void Set_Path_Line_X(uint16_t id,int16_t x)
{
	int16_t temp=MAP_SIZE;	

	if(Line_xMax<x)
	{
		Line_xMax = x;
	}
	
	if(x<0)
	{
		temp += x;
	}
	else
	{
		temp = x;
	}
		
	pos_line[id].x = (uint8_t)temp;
}

int16_t Get_Path_Line_X(uint16_t id)
{
	int16_t temp=0;
	
	temp = pos_line[id].x;
	
	if(temp>Line_xMax)
	{
		temp -= MAP_SIZE;
	}
	
	return temp;
}

void Set_Path_Line_X_Off(uint16_t id,uint8_t x_off)
{
	pos_line[id].x_off = x_off;
}

uint8_t Get_Path_Line_X_Off(uint16_t id)
{
	return pos_line[id].x_off;
}

void Set_Path_Line_Y(uint16_t id,int16_t y)
{
	int16_t temp=MAP_SIZE;
	
	if(Line_yMax<y)
	{
		Line_yMax = y;
	}
	
	if(y<0)
	{
		temp += y;
	}
	else
	{
		temp = y;
	}

	pos_line[id].y = (uint8_t)temp;

}

int16_t Get_Path_Line_Y(uint16_t id)
{
	int16_t temp = 0;
	
	temp = pos_line[id].y;
	
	if(temp>Line_yMax)
	{
		temp -= MAP_SIZE;
	}	

	return temp;
}
//主要是为了省内存，将线条等级和选中标志合二为一，高位用来表示选中的线条
void Set_Path_Line_Level_Flag(uint16_t id,uint8_t flag)
{
	if(flag)
	{
		flag = 0x80;
		pos_line[id].level |= flag;
	}
	else
	{
		pos_line[id].level &= 0x7f;
	}
}

uint8_t Get_Path_Line_Level_Flag(uint16_t id)
{
	if(pos_line[id].level&0x80)
	{
		return 1;
	}
	return 0;
}

void Set_Path_Line_Level(uint16_t id,uint8_t level)
{
	uint8_t temp=level;
	
	temp &= 0x7f;
	pos_line[id].level = temp;
}

uint8_t Get_Path_Line_Level(uint16_t id)
{
	return (uint8_t)(pos_line[id].level&0x7f);
}

void Init_Line_Range(Point16_t pos)
{
	Line_xMax=pos.X;
	Line_yMax=pos.Y;
}

void Init_Path_Line(void)
{
	uint16_t i=0;		
	
	Set_Path_Line_Count(0);
	for(i=0;i<POS_LINE_CNT;i++)
	{
		Set_Path_Line_X(i,0);
		Set_Path_Line_X_Off(i,0);
		Set_Path_Line_Y(i,0);
		Set_Path_Line_Level(i,0);
		Set_Path_Line_Level_Flag(i,0);		
	}
	
	target_find = 0;
}

void Delete_Path_Line_Data(uint16_t id)
{
	uint16_t i=Get_Path_Line_Count();
	
	while(id<i)
	{
		pos_line[id] = pos_line[id+1];
		id++;
	}
	Set_Path_Line_Count(i-1);
}

//---------------------------------------------------------------

void Path_Line_Dump(void)
{
	#ifdef DEBUG_POS
	int16_t i=0;

	USPRINTF("\tFlag\tLevel\tY\tX\n");
	for (i = 0; i < Get_Path_Line_Count(); i++) 
	{
		USPRINTF("%d:\t%d:\t%d\t%d\t%d - %d\n", i, Get_Path_Line_Level_Flag(i),Get_Path_Line_Level(i), Get_Path_Line_Y(i), Get_Path_Line_X(i),  Get_Path_Line_X(i)+ Get_Path_Line_X_Off(i));
	}
	USPRINTF("Line List: %d\n", Get_Path_Line_Count());
	#endif
}


//-------------------------------------------------------------------------------------------------------------
//判断是否已经有已画的线包含点POS
uint8_t Path_Pos_Level_Is_Add(Point16_t pos)
{
	uint16_t i=0;
	uint8_t retval=0;	
	
	if(Get_Path_Line_Count()==0)
	{
		retval = 0;
	}
	else
	{
		for(i=0; i<Get_Path_Line_Count();i++) 
		{			
			if(pos.Y == Get_Path_Line_Y(i)) 
			{	
				if(((Get_Path_Line_X(i) <= pos.X )&&(pos.X <= (Get_Path_Line_X(i) + Get_Path_Line_X_Off(i)))))
				{
					retval = 1;
					break;	
				}					
			}
		}
	}
	
	return retval;	
}


//查找包含POS点的线的等级
uint8_t Get_Pos_Line_Level(Point16_t pos)
{
	uint16_t i=0;
	uint8_t retval=MAX_LEVEL+1;
	
	for(i=0;i<Get_Path_Line_Count();i++)
	{
		if(Get_Path_Line_Y(i)==pos.Y)
		{
			if((Get_Path_Line_X(i)<=pos.X)&&((Get_Path_Line_X(i)+Get_Path_Line_X_Off(i))>=pos.X))
			{
				retval = Get_Path_Line_Level(i);
				break;
			}
		}
	}		
	return retval;
}
//查找包含相同等级且Y也相同的线的ID（可以其中有多条是符合条件）
uint16_t Get_Line_Id(Point16_t pos,uint8_t level,int16_t y,uint16_t id)
{
	uint16_t retval=POS_LINE_CNT+1;	
	uint16_t k=0;
	uint16_t temp_k[10]={0};
	uint8_t i=0,j=0;
	int16_t temp_x=0;
//	int16_t temp_pos_x=0;
	int16_t min_dis=0;
	
	for(k=id;;k--)
	{
		if((Get_Path_Line_Level(k)==level))
		{
			if((Get_Path_Line_Y(k)==y))
			{
				if(!(((Get_Path_Line_X(id))>(Get_Path_Line_X(k)+(Get_Path_Line_X_Off(k))))||((Get_Path_Line_X(k))>(Get_Path_Line_X(id)+(Get_Path_Line_X_Off(id))))))
				{			
					i=0;
					while(((Get_Path_Line_Level(k)==level))&&(!(((Get_Path_Line_X(id))>(Get_Path_Line_X(k)+(Get_Path_Line_X_Off(k))))||((Get_Path_Line_X(k))>(Get_Path_Line_X(id)+(Get_Path_Line_X_Off(id)))))))
					{		
						if((Get_Path_Line_Y(k)==y))
						{
							temp_k[i] = k;
							
							i++;
							if(i>9)break;
						}
						
						if(k==0)break;
						if(k>0)k--;
					}
					
					if(i>1)
					{
					
//						temp_pos_x = pos.X;
						temp_x = (Get_Path_Line_X(temp_k[i-1]) + Get_Path_Line_X_Off(temp_k[i-1]))/2;								
//						min_dis = ABS(temp_pos_x,temp_x);						
						temp_x = pos.X - temp_x;
						if(temp_x<0)temp_x = -temp_x;
						min_dis = temp_x;
						
						k = temp_k[i-1];
						
						for(j=i-1;j>0;j--)
						{	
							temp_x = (Get_Path_Line_X(temp_k[j]) + Get_Path_Line_X_Off(temp_k[j]))/2;	
							temp_x = pos.X - temp_x;
							if(temp_x<0)temp_x = -temp_x;
							if((min_dis>temp_x)&&(Get_Path_Line_X_Off(temp_k[j])>2))
							{
								min_dis = temp_x;
								k = temp_k[j];
							}
						}
						temp_x = (Get_Path_Line_X(temp_k[0]) + Get_Path_Line_X_Off(temp_k[0]))/2;			
						temp_x = pos.X - temp_x;
						if(temp_x<0)temp_x = -temp_x;
						if((min_dis>temp_x)&&(Get_Path_Line_X_Off(temp_k[0])>2))
						{
							min_dis = temp_x;
							k = temp_k[0];
						}
						
						temp_k[0] = k;					
					}
					retval = temp_k[0];
					break;
				}				
			}		
		}
		if(k==0)break;
	}
	if(id==0)retval = 0;
	
	return retval;
}

uint16_t Get_Line_Id_Flag(Point16_t pos,uint8_t level,int16_t *y,uint16_t id)
{
	uint16_t retval=POS_LINE_CNT+1;	
	uint16_t k=0;
	
	for(k=id;;k--)
	{
		if(((Get_Path_Line_Level(k)==level))&&(Get_Path_Line_Level_Flag(k)))
		{		
			(*y) = Get_Path_Line_Y(k);
			retval = k;
			break;
		}
		if(k==0)break;
	}
	if(id==0)retval = 0;
	
	return retval;
}

//查找包含POS点的线的ID
uint16_t Get_Line_Id_POS(Point16_t pos)
{
	uint16_t retval=POS_LINE_CNT+1;	
	uint16_t k=0,id=0;
	
	id = Get_Path_Line_Count();
	if(id>0)id -= 1;
	
	for(k=id;;k--)
	{
		if((Get_Path_Line_Y(k)==pos.Y))
		{
			if(((pos.X<=(Get_Path_Line_X(k)+Get_Path_Line_X_Off(k)))&&(Get_Path_Line_X(k))<=pos.X))
			{
				retval = k;
				break;
			}				
		}	
		if(k==0)break;
	}
	
	return retval;
}
/***************************/
void Set_Line_Value(uint16_t count,uint8_t flag,uint8_t level,int16_t y,int16_t xmin,int16_t xmax)
{
	int16_t temp = xmax - xmin;
	
	Set_Path_Line_Count(count);	
	Set_Path_Line_X(count,xmin);
	Set_Path_Line_X_Off(count,temp);
	Set_Path_Line_Y(count,y);
	Set_Path_Line_Level(count,level);
	Set_Path_Line_Level_Flag(count,flag);		
}
/****************************/
//--------------------------------------------------------------------------------------------------------------
//画线条包含POS点，线两头是边界或者是障碍物
uint8_t Draw_Line(Point16_t pos,uint16_t id,uint8_t level,int16_t x_min,int16_t x_max,uint16_t last_id)
{
	uint8_t	x_off=0;
	int16_t x=0,y=0;	
	uint8_t retval=0;
	uint8_t clean=0;
	Point16_t temp_pos=pos;
	Point16_t temp_cell=Get_Target_Cell();
	
	x = pos.X;
	y = pos.Y;
//	USPRINTF("%s %d: \n", __FUNCTION__, __LINE__);
	if((Path_Pos_Level_Is_Add(pos)==0)&&(is_a_block(x,y)==0))
	{
		while(1)
		{				
			if((((is_a_block(x,y)==0)&&(is_a_block(x,y+1)==0))&&(is_a_block(x,y-1)==0))||(((id==0)&&(is_a_block(x,y)==0))&&((x>(temp_pos.X-2)&&(x<(temp_pos.X+2))))))
			{
				if(x==x_min)
				{
					
					break;
				}
				x--;
			}
			else
			{
				break;
			}
			
			if(clean<2)
			{
				if((Map_GetCell(MAP,x,y)==CLEANED)||(Map_GetCell(MAP,x,y-1)==CLEANED)||(Map_GetCell(MAP,x,y+1)==CLEANED)||(Map_GetCell(MAP,x,y-2)==CLEANED)||(Map_GetCell(MAP,x,y+2)==CLEANED))
				{
					clean++;
				}
			}
		}
		x = x + 1;		
		
		Set_Path_Line_X(id,x);
		
		x = pos.X;
		y = pos.Y;
		while(1)
		{				
			if((((is_a_block(x,y)==0)&&(is_a_block(x,y+1)==0))&&(is_a_block(x,y-1)==0))||(((id==0)&&(is_a_block(x,y)==0))&&((x>(temp_pos.X-2)&&(x<(temp_pos.X+2))))))
			{
				if(x==x_max)
				{
					break;
				}
				x++;
			}
			else
			{
				break;
			}
			
			if(clean<2)
			{
				if((Map_GetCell(MAP,x,y)==CLEANED)||(Map_GetCell(MAP,x,y-1)==CLEANED)||(Map_GetCell(MAP,x,y+1)==CLEANED)||(Map_GetCell(MAP,x,y-2)==CLEANED)||(Map_GetCell(MAP,x,y+2)==CLEANED))
				{
					clean++;
				}
			}
		}

		x = x - 1;		

		if((x-Get_Path_Line_X(id))>1)
		{
			x_off = x - Get_Path_Line_X(id);
		}
		else
		{
			x_off = 0;
		}
		
		if(id!=0)
		{
			if(!(((Get_Path_Line_X(id)<(Get_Path_Line_X(last_id)+Get_Path_Line_X_Off(last_id))+2)||(Get_Path_Line_X(last_id)-2)<x)))
			{
				x_off = 0;
			}			
		}

		if(((x_off>1)&&(Get_Search_Range()==0))||(((x_off>1)&&(!(clean<2)))&&(Get_Search_Range()==1)))
		{
			Set_Path_Line_X_Off(id,x_off);
			
			if(((pos.X<=(Get_Path_Line_X(id)+Get_Path_Line_X_Off(id)))&&(Get_Path_Line_X(id))<=pos.X))
			{
				Set_Path_Line_Y(id,y);
				Set_Path_Line_Level(id,level);
				Set_Path_Line_Count(id+1);
				retval = 1;	
				
				if(next_move_step==1)
				{
					if(y==temp_cell.Y)
					{
						if(((temp_cell.X<=(Get_Path_Line_X(id)+Get_Path_Line_X_Off(id)))&&(Get_Path_Line_X(id))<=temp_cell.X))
						{
							target_find = 1;
						}
					}
				}
				else if(next_move_step==2)
				{
					if(y==temp_cell.Y)
					{
						if(((temp_cell.X<=(Get_Path_Line_X(id)+Get_Path_Line_X_Off(id)))&&(Get_Path_Line_X(id))<=temp_cell.X))
						{
							target_find |= 0x01;
						}
					}
					else if(((y+1)==temp_cell.Y)||((y-1)==temp_cell.Y))
					{
						if(((temp_cell.X<=(Get_Path_Line_X(id)+Get_Path_Line_X_Off(id)))&&(Get_Path_Line_X(id))<=temp_cell.X))
						{
							target_find |= 0x02;
							Set_Path_Line_Level_Flag(id,1);	
						}
					}
				}
				else if(next_move_step==3)
				{
					if(y==temp_cell.Y)
					{
						if(((temp_cell.X<=(Get_Path_Line_X(id)+Get_Path_Line_X_Off(id)))&&(Get_Path_Line_X(id))<=temp_cell.X))
						{
							target_find |= 0x01;
						}
					}
					else if((((y+1)==temp_cell.Y)||((y-1)==temp_cell.Y))||(((y+2)==temp_cell.Y)||((y-2)==temp_cell.Y)))
					{
						if(((temp_cell.X<=(Get_Path_Line_X(id)+Get_Path_Line_X_Off(id)))&&(Get_Path_Line_X(id))<=temp_cell.X))
						{
							target_find |= 0x02;
							Set_Path_Line_Level_Flag(id,1);	
						}
					}
				}				
			}	
			else
			{
				Set_Path_Line_X(id,0);
				Set_Path_Line_X_Off(id,0);	
			}
		}
		else
		{
			Set_Path_Line_X(id,0);
			Set_Path_Line_X_Off(id,0);			
		}		
	}
	
	return retval;
}
//根据现有的一条线画旁边所能到达的线（有交集）
uint8_t One_Line_Spread(uint16_t id,uint8_t level,int16_t x_min,int16_t x_max,int16_t y_min,int16_t y_max)
{
	int16_t x=0,y=0;
	uint8_t last_x_min_f=0,last_x_max_f=0,temp_level=0,retval=0;;
	int16_t last_x_min=0,last_x_max=0;
	int16_t temp_x_min=0,temp_x_max=0;	
	uint16_t temp_id=0,temp=0;	
	Point16_t pos;
	
	temp_level = Get_Path_Line_Level(id);
	if(temp_level==MAX_LEVEL)
	{
		temp_level=MIN_LEVEL+1;
	}
	else
	{
		temp_level++;
	}
	y = Get_Path_Line_Y(id);
	
//	USPRINTF("%s %d: \n", __FUNCTION__, __LINE__);
	
	if((temp_level==level)&&((y<y_max)&&(y>y_min)))
	{
		temp_x_min = Get_Path_Line_X(id) ;
		temp_x_max = Get_Path_Line_X(id) + Get_Path_Line_X_Off(id) ;		
		
		do
		{	
			y = Get_Path_Line_Y(id);
						
			if(last_x_min_f==0)
			{
				x = Get_Path_Line_X(id) ;	
				last_x_min = x;
			}
			else
			{
				x = last_x_min;
			}
			
			if(x>(Get_Path_Line_X(id)+Get_Path_Line_X_Off(id)-1))
			{
				break;
			}
			
			while(1)
			{				
					
				if(is_a_block(x,y-next_move_step)==1)
				{
					
				}
				else
				{
					break;
				}
				if(x<temp_x_max)
				{
					x++;
				}				
				else
				{
					break;
				}
			}			
			
			if(x<temp_x_max)
			{
				pos.X = x;
				pos.Y = y-next_move_step;
				temp_id = Get_Path_Line_Count();	
				last_x_min = x;
				
				if(((pos.X>x_min)&&(pos.X<x_max))&&((pos.Y>y_min)&&(pos.Y<y_max)))
				{
					if(Path_Pos_Level_Is_Add(pos)==0)
					{
						if(Draw_Line(pos,temp_id,level,x_min,x_max,id))
						{
							retval = 1; 
							last_x_min = Get_Path_Line_X(temp_id)+Get_Path_Line_X_Off(temp_id)+2;	
							last_x_min_f = 1;				
						}
						else
						{
							last_x_min += 2;//break;
							last_x_min_f = 1;	
						}
					}
					else
					{
						temp = Get_Line_Id_POS(pos);
						if(temp<POS_LINE_CNT)
						{	
							last_x_min = Get_Path_Line_X(temp)+Get_Path_Line_X_Off(temp)+2;	
							last_x_min_f = 1;	
						}
						else
						{							
							break;
						}
					}
				}
				else
				{
					break;
				}
			}
			else
			{
				break;
			}				
			

			y = Get_Path_Line_Y(id);
			if(last_x_max_f==0)
			{
				x = Get_Path_Line_X(id)+Get_Path_Line_X_Off(id) ;	
				last_x_max = x;
			}
			else
			{
				x = last_x_max;
			}
			if(x<(Get_Path_Line_X(id)+1))
			{
				break;
			}
			
			while(1)
			{				
				if(is_a_block(x,y-next_move_step)==1)
				{
				
				}
				else
				{
					break;
				}
				
				if(x>temp_x_min)
				{
					x--;
				}
				else
				{
					break;
				} 
			}
			if(x>temp_x_min)
			{
				pos.X = x;
				pos.Y = y-next_move_step;
				temp_id = Get_Path_Line_Count();
				last_x_max = x;
				
				if(((pos.X>x_min)&&(pos.X<x_max))&&((pos.Y>y_min)&&(pos.Y<y_max)))
				{
					if(Path_Pos_Level_Is_Add(pos)==0)
					{
						if(Draw_Line(pos,temp_id,level,x_min,x_max,id))
						{
							retval = 1;
							last_x_max = Get_Path_Line_X(temp_id)-2;	
							last_x_max_f = 1;
						}
						else
						{
							last_x_max -= 2;//break;
							last_x_max_f = 1;
						}
					}
					else
					{
						temp = Get_Line_Id_POS(pos);
						if(temp<POS_LINE_CNT)
						{
							last_x_max = Get_Path_Line_X(temp)-2;
							last_x_max_f = 1;
						}
						else
						{							
							break;
						}
					}
				}
				else
				{
					break;
				}
			}
			else
			{
				break;
			}
		}
		while((last_x_max-last_x_min)>1);		
		
		//------------------------------------------------------------------------------------
		last_x_max = 0;
		last_x_min = 0;
		last_x_min_f = 0;
		last_x_max_f = 0;	
		temp_x_min = Get_Path_Line_X(id) ;
		temp_x_max = Get_Path_Line_X(id) + Get_Path_Line_X_Off(id) ;
		do
		{	
			y = Get_Path_Line_Y(id);
			
			if(last_x_min_f==0)
			{
				x = Get_Path_Line_X(id) ;	
				last_x_min = x;
			}
			else
			{
				x = last_x_min;
			}	

			if(x>(Get_Path_Line_X(id)+Get_Path_Line_X_Off(id)-1))
			{
				break;
			}
			
			while(1)
			{					
				if(is_a_block(x,y+next_move_step)==1)
				{
					
				}
				else
				{
					break;
				}
				
				if(x<temp_x_max)
				{
					x++;
				}
				else
				{
					break;
				}
			}
			if(x<temp_x_max)
			{
				pos.X = x;
				pos.Y = y+next_move_step;
				temp_id = Get_Path_Line_Count();
				last_x_min = x;
				
				if(((pos.X>x_min)&&(pos.X<x_max))&&((pos.Y>y_min)&&(pos.Y<y_max)))
				{
					if(Path_Pos_Level_Is_Add(pos)==0)
					{
						if(Draw_Line(pos,temp_id,level,x_min,x_max,id))
						{
							retval = 1;
							last_x_min = Get_Path_Line_X(temp_id)+Get_Path_Line_X_Off(temp_id)+2;	
							last_x_min_f = 1;	
						}
						else
						{
							last_x_min += 2;//break;
							last_x_min_f = 1;	
						}
					}
					else
					{
						temp = Get_Line_Id_POS(pos);
						if(temp<POS_LINE_CNT)
						{	
							last_x_min = Get_Path_Line_X(temp)+Get_Path_Line_X_Off(temp)+2;	
							last_x_min_f = 1;	
						}
						else
						{						
							break;
						}
					}
				}
				else
				{
					break;
				}
			}
			else
			{
				break;
			}				

			y = Get_Path_Line_Y(id);
			
			if(last_x_max_f==0)
			{
				x = Get_Path_Line_X(id)+Get_Path_Line_X_Off(id);	
				last_x_max = x;
			}
			else
			{
				x = last_x_max;
			}
			
			if(x<(Get_Path_Line_X(id)+next_move_step))
			{
				break;
			}
			
			while(1)
			{
				
				if(is_a_block(x,y+next_move_step)==1)
				{
				
				}
				else
				{
					break;
				}
				
				if(x>temp_x_min)
				{
					x--;
				}
				else
				{
					break;
				} 
				
			}
			if(x>temp_x_min)
			{
				pos.X = x;
				pos.Y = y+next_move_step;
				temp_id = Get_Path_Line_Count();	
				last_x_max = x;
				
				if(((pos.X>x_min)&&(pos.X<x_max))&&((pos.Y>y_min)&&(pos.Y<y_max)))
				{
					if(Path_Pos_Level_Is_Add(pos)==0)
					{
						if(Draw_Line(pos,temp_id,level,x_min,x_max,id))
						{
							retval = 1;
							last_x_max = Get_Path_Line_X(temp_id)-2;	
							last_x_max_f = 1;
						}
						else
						{
							last_x_max -= 2;//break;
							last_x_max_f = 1;
						}
					}
					else
					{
						temp = Get_Line_Id_POS(pos);
						if(temp<POS_LINE_CNT)
						{	
							last_x_max = Get_Path_Line_X(temp)-2;
							last_x_max_f = 1;
						}
						else
						{							
							break;
						}
					}
				}
				else
				{
					break;
				}
			}
			else
			{
				break;
			}
		}
		while((last_x_max-last_x_min)>1);
	}
	
	return retval;
}

//从机器位置开始过散到目标点所需要画的线
uint8_t All_Line_Spread(Point16_t pos,int16_t x,int16_t y,int16_t x_min,int16_t x_max,int16_t y_min,int16_t y_max)
{
	uint16_t i=0;
	uint16_t last_id=0;
	uint16_t last_level=0;
	Point16_t target_pos;
	uint8_t retval=0;
	uint8_t next=1;
	
	target_pos.X = x;
	target_pos.Y = y;
	
	Init_Line_Range(pos);
	Init_Path_Line();
	Set_Target_Cell(target_pos);
	
	if(Draw_Line(pos,0,1,x_min,x_max,0))
	{
		last_id = 0;
		last_level = 1;
		
		if(Path_Pos_Level_Is_Add(target_pos))
		{			
			retval = 1;
		}
		else
		{
			do
			{		
				i = Get_Path_Line_Count();
				
				if(i==(POS_LINE_CNT))
				{
					next=0;
					retval = 0;
					break;
				}
				
				last_level++;
				if(last_level>MAX_LEVEL)
				{
					last_level = MIN_LEVEL+1;
				}
				
				for(;last_id<i;last_id++)
				{
					if(0x02&target_find)
					{
						last_level = Get_Path_Line_Level(Get_Path_Line_Count()-1);
						
						last_level++;
						if(last_level>MAX_LEVEL)
						{
							last_level = MIN_LEVEL+1;
						}
				
						if(Draw_Line(target_pos,Get_Path_Line_Count(),last_level,x_min,x_max,last_id))
						{
							next=0;
							retval = 1;
							break;
						}
						else
						{
							next=0;
							retval = 0;
							break;
						}
					}
					
					if(One_Line_Spread(last_id,last_level,x_min,x_max,y_min,y_max)==0)
					{						
						if((last_id)==(Get_Path_Line_Count()-1))
						{
							next=0;
							retval = 0;
							break;
						}						
						continue;
					}
					
					if(0x01&target_find)
					{
						next=0;
						retval = 1;
						break;
					}
					
					if(0x02&target_find)
					{
						continue;
					}
					
				}
			}
			while(next);
		}
	}
	
	return retval;
}

//从目标点反向找回机器的位置，并把线标记上，把没有用的线删除掉
void Path_Trace_Path(Point16_t pos,Point16_t target_pos,uint8_t line_level,int16_t line_y,uint8_t cur_level,int16_t cur_y)
{
	int16_t next_y=0,temp_y=0;
	uint16_t i=0,k=1;
	uint16_t next_id=0;
	uint8_t flag=1;
	uint16_t temp_id=0;
	
//	USPRINTF("%s %d: \n", __FUNCTION__, __LINE__);
	
	Set_Path_Line_Level_Flag(0,1);	

	temp_id = Get_Line_Id_POS(target_pos);
	
	if(temp_id>POS_LINE_CNT)
	{
		USPRINTF("error temp_id %s %d\n", __FUNCTION__, __LINE__);
		return ;
	}
	
	while((!((Get_Path_Line_X(temp_id)<=target_pos.X)&&((Get_Path_Line_X(temp_id)+Get_Path_Line_X_Off(temp_id))>=target_pos.X)))&&(Get_Path_Line_Y(temp_id)==target_pos.Y))
	{
		USPRINTF("error temp_id %s %d:  %d\n", __FUNCTION__, __LINE__,temp_id);
		return ;	
	}
	
	Set_Path_Line_Level_Flag(temp_id,1);
	next_y = line_y;
	next_id = temp_id;
	temp_y = next_y;
	
	if(temp_id>0)
	{
		if(temp_id<20)k=0;
		if(flag==1)
		{
			flag = 0;
			line_level--;
			if(line_level==1)
			{
				if(k==1)
				{
					line_level = MAX_LEVEL;
				}				
			}
		}
		do
		{
			if(temp_id<20)k=0;
			
			temp_y = next_y;
			
			if(0x02&target_find)
			{
				next_id = Get_Line_Id_Flag(target_pos,line_level,&temp_y,temp_id);				
				target_find &= ~0x02;
				
				Set_Path_Line_Level_Flag(next_id,1);
				next_y = Get_Path_Line_Y(next_id);
				temp_id = next_id;
				if(temp_id<20)k=0;
				flag = 1;
				if(flag==1)
				{
					flag = 0;
					line_level--;
					if(line_level==1)
					{
						if(k==1)
						{
							line_level = MAX_LEVEL;
						}
						else
						{
							break;
						}
					}
				}				
				temp_y = next_y;
			}
			
			temp_y += next_move_step;
			next_id = Get_Line_Id(pos,line_level,temp_y,temp_id);					
			
			if((next_id<POS_LINE_CNT))
			{
				Set_Path_Line_Level_Flag(next_id,1);
				next_y = Get_Path_Line_Y(next_id);
				temp_id = next_id;
				if(temp_id<20)k=0;
				flag = 1;
				if(flag==1)
				{
					flag = 0;
					line_level--;
					if(line_level==1)
					{
						if(k==1)
						{
							line_level = MAX_LEVEL;
						}
						else
						{
							break;
						}
					}
				}
			}
			else
			{
				temp_y = next_y;		
				
				temp_y -= next_move_step;
				next_id = Get_Line_Id(pos,line_level,temp_y,temp_id);				
			
				if((next_id<POS_LINE_CNT))
				{
					Set_Path_Line_Level_Flag(next_id,1);
					next_y = Get_Path_Line_Y(next_id);
					temp_id = next_id;
					if(temp_id<20)k=0;
					flag = 1;
					if(flag==1)
					{
						flag = 0;
						line_level--;
						if(line_level==1)
						{
							if(k==1)
							{
								line_level = MAX_LEVEL;
							}
							else
							{
								break;
							}
						}
					}
				}		
			}		
		}
		while(cur_level<line_level);
		
		i=0;
		do
		{
			for(;i<Get_Path_Line_Count();i++)
			{
				if(Get_Path_Line_Level_Flag(i)==0)
				{
					Delete_Path_Line_Data(i);i--;
					continue;
				}
			}
		}
		while(i<Get_Path_Line_Count());	
	}
}

int16_t Points_Distance(int16_t x,int16_t x1,int16_t x2)
{	 	
	if((x1<=x)&&(x<=x2))
	{
		return x;
	}
	else
	{
		return (x1+x2)/2;
	}
}

//根据线条算出行走所需的拐点
uint8_t Path_To_Move(Point16_t pos, Point16_t target, Point16_t* next_pos,uint16_t last_dir)
{
	uint16_t i=0,k=0,next_id=0;
	uint8_t retval=1,equal=0;
	int16_t temp_x_min=0,temp_x_max=0,x_min=0,x_max=0,temp_y=0,last_y=0,next_y=0,move_temp_x_min=0,move_temp_x_max=0;
	Point16_t temp_pos;
	int16_t temp_offset=0;
	uint8_t offset=1;
	
	Reset_Next_Pos();
	
	i = Get_Path_Line_Count();
	next_id = 1;
	if(Get_Path_Line_Count()>1)
	{
		if(pos.Y==Get_Path_Line_Y(0))
		{
			last_y = pos.Y;
			temp_y = pos.Y;
			temp_pos.X = pos.X;
			temp_pos.Y = pos.Y;
			x_min = Get_Path_Line_X(next_id-1);
			x_max = x_min + Get_Path_Line_X_Off(next_id-1);			
			if(Get_Path_Line_X(next_id)>x_min)
			{
				x_min = Get_Path_Line_X(next_id);
			}
			
			if((Get_Path_Line_X(next_id)+Get_Path_Line_X_Off(next_id))<x_max)
			{
				x_max = Get_Path_Line_X(next_id)+Get_Path_Line_X_Off(next_id);
			}
			temp_x_min= x_min+1;
			temp_x_max= x_max-1;
			
			move_temp_x_min= temp_x_min;
			move_temp_x_max= temp_x_max;

			temp_offset = Get_Path_Line_X_Off(next_id);

			do
			{		
				equal = 1;
				
				k = 0;
				
//				USPRINTF("%s %d:temp_id:%d  last_y:%d Get_Path_Line_Y(next_id):%d\n", __FUNCTION__, __LINE__,temp_id,last_y,Get_Path_Line_Y(next_id));
				
				while(last_y>Get_Path_Line_Y(next_id))
				{	
					equal = 0;						
					k = 1;
					next_y = Get_Path_Line_Y(next_id);
					
					temp_offset = Get_Path_Line_X_Off(next_id);

					if(temp_offset>1)
					{
						if(offset>1)
						{
							offset = 1;
							break;
						}
					}
					else
					{
						offset = 0;
						Reset_Next_Pos();
						retval = 0;
						return retval;
					}
			
					if(x_min<(Get_Path_Line_X(next_id) + offset))
					{
						x_min = Get_Path_Line_X(next_id) + offset;

//						x_min += 1;
						if((move_temp_x_min<=x_min)&&((move_temp_x_max)>=x_min))
						{
							move_temp_x_min = x_min;
						}
						else
						{
							break;
						}						
					}
					
					if((x_max>(Get_Path_Line_X(next_id)+Get_Path_Line_X_Off(next_id) - 1*offset)))
					{
						x_max = Get_Path_Line_X(next_id)+Get_Path_Line_X_Off(next_id)- 1*offset;	

//						x_max -= 1;
						if((move_temp_x_min<=x_max)&&((move_temp_x_max)>=x_max))
						{
							move_temp_x_max = x_max;
						}
						else
						{
							break;
						}						
					}				
					
					if(x_min<=x_max)
					{
						if(next_id<i)
						{
							next_id++;
							if(next_id==i)break;
						}
						else
						{
							break;
						}
					}
					else
					{
						break;
					}
					last_y = next_y;
					k = 2;
				}			
//				USPRINTF("%s %d:temp_id:%d  last_y:%d Get_Path_Line_Y(next_id):%d\n", __FUNCTION__, __LINE__,temp_id,last_y,Get_Path_Line_Y(next_id));
				if(k!=0)
				{
					k = 0;

					if(next_id!=i)
					{
						if((temp_pos.X>=move_temp_x_min)&&(temp_pos.X<=move_temp_x_max))
						{
							
						}
						else
						{
//							temp_pos.X = (move_temp_x_min + move_temp_x_max)/2;
							
							temp_pos.X = Points_Distance(temp_pos.X,move_temp_x_min,move_temp_x_max);	
							if(!((temp_pos.X==pos.X)&&(temp_pos.Y==pos.Y)))
								{						
									Add_Next_Pos(temp_pos);	
								}	
						}
					}
					else
					{						
						if((target.X>=move_temp_x_min)&&(target.X<=move_temp_x_max))
						{
							
							if((temp_pos.X>=move_temp_x_min)&&(temp_pos.X<=move_temp_x_max))
							{
//								temp_y = target.Y;
							}
							else
							{
								if(temp_y!=target.Y)
								temp_pos.X = target.X;
							}
						}
						else
						{							
							if((temp_pos.X>=move_temp_x_min)&&(temp_pos.X<=move_temp_x_max))
							{
								
							}
							else
							{
								//							temp_pos.X = (move_temp_x_min + move_temp_x_max)/2;
							
								temp_pos.X = Points_Distance(temp_pos.X,move_temp_x_min,move_temp_x_max);	
								if(!((temp_pos.X==pos.X)&&(temp_pos.Y==pos.Y)))
								{						
									Add_Next_Pos(temp_pos);	
								}	
							}
						}					
					}
					//USPRINTF("%s %d:temp_pos.Y:%d  temp_pos.X:%d \n", __FUNCTION__, __LINE__,temp_pos.Y,temp_pos.X);
//					if(last_dir!=0)
//					{
//						
//						if(!((temp_pos.X==pos.X)&&(temp_pos.Y==pos.Y)))
//						{						
//							Add_Next_Pos(temp_pos);	
//						}
//					}
					
					temp_pos.Y = temp_y;
					if(!((temp_pos.X==pos.X)&&(temp_pos.Y==pos.Y)))
					{						
						Add_Next_Pos(temp_pos);	
					}	
					else
					{
						if(last_dir!=0)
						{
							if(next_id==i)
							{
								if((target.X>move_temp_x_min)&&(target.X<move_temp_x_max))
								temp_pos.X = target.X;
								if(!((temp_pos.X==pos.X)&&(temp_pos.Y==pos.Y)))
								{						
									Add_Next_Pos(temp_pos);	
								}	
							}
						}
					}
					
					if((temp_pos.X>(Get_Path_Line_X(next_id-1))+Get_Path_Line_X_Off(next_id-1))||(temp_pos.X<Get_Path_Line_X(next_id-1)))
					{
						temp_pos.X = (move_temp_x_min + move_temp_x_max)/2;
						if(!(temp_pos.X==pos.X))
						{
							Add_Next_Pos(temp_pos);	
						}
					}
					
					//USPRINTF_ZZ("%s %d:temp_pos.Y:%d  temp_pos.X:%d \n", __FUNCTION__, __LINE__,temp_pos.Y,temp_pos.X);			
					temp_pos.Y = Get_Path_Line_Y(next_id-1);			
					if(!((temp_pos.X==pos.X)&&(temp_pos.Y==temp_y)))
					{
						Add_Next_Pos(temp_pos);	
					}
					if(next_id==i)break;
					last_y = temp_pos.Y;
					temp_y = last_y;
					//USPRINTF("%s %d:temp_pos.Y:%d  temp_pos.X:%d \n", __FUNCTION__, __LINE__,temp_pos.Y,temp_pos.X);
					x_min = Get_Path_Line_X(next_id-1);
					x_max = x_min + Get_Path_Line_X_Off(next_id-1);			
					if(Get_Path_Line_X(next_id)>x_min)
					{
						x_min = Get_Path_Line_X(next_id);
						
					}
					
					if((Get_Path_Line_X(next_id)+Get_Path_Line_X_Off(next_id))<x_max)
					{
						x_max = Get_Path_Line_X(next_id)+Get_Path_Line_X_Off(next_id);
					}
					temp_x_min= x_min+1;
					temp_x_max= x_max-1;
					
					move_temp_x_min= temp_x_min;
					move_temp_x_max= temp_x_max;	
					
					temp_offset = Get_Path_Line_X_Off(next_id);
//					if(temp_offset>4)
//					{
//						offset = 2;
//						move_temp_x_min += 2;
//						move_temp_x_max -= 2;
//					}
//					else 
	/*
						if(temp_offset>2)
					{
						offset = 1;
						move_temp_x_min += 1;
						move_temp_x_max -= 1;
					}
					else
					{
//						offset = 0;
						offset = 0;
						Reset_Next_Pos();
						retval = 0;
						return retval;
					}
			*/
					if(next_id<i)
					{
						next_id++;
						if(next_id==i)
						{
							if(next_id!=i)
					{
						if((temp_pos.X>=move_temp_x_min)&&(temp_pos.X<=move_temp_x_max))
						{
							
						}
						else
						{
//							temp_pos.X = (move_temp_x_min + move_temp_x_max)/2;
							
							temp_pos.X = Points_Distance(temp_pos.X,move_temp_x_min,move_temp_x_max);
							if(!((temp_pos.X==pos.X)&&(temp_pos.Y==pos.Y)))
							{						
								Add_Next_Pos(temp_pos);	
							}	
						}
					}
					else
					{							
						if((target.X>=move_temp_x_min)&&(target.X<=move_temp_x_max))
						{
							if((temp_pos.X>=move_temp_x_min)&&(temp_pos.X<=move_temp_x_max))
							{
//								temp_y = target.Y;
							}
							else
							{
								if(temp_y!=target.Y)
								temp_pos.X = target.X;
							}
						}
						else
						{							
							if((temp_pos.X>=move_temp_x_min)&&(temp_pos.X<=move_temp_x_max))
							{
								
							}
							else
							{
//							temp_pos.X = (move_temp_x_min + move_temp_x_max)/2;
							
								temp_pos.X = Points_Distance(temp_pos.X,move_temp_x_min,move_temp_x_max);	
								if(!((temp_pos.X==pos.X)&&(temp_pos.Y==pos.Y)))
								{						
									Add_Next_Pos(temp_pos);	
								}	
							}
						}					
					}
//USPRINTF("%s %d:temp_pos.Y:%d  temp_pos.X:%d \n", __FUNCTION__, __LINE__,temp_pos.Y,temp_pos.X);
					temp_pos.Y = temp_y;
					if(!((temp_pos.X==pos.X)&&(temp_pos.Y==pos.Y)))
					{						
						Add_Next_Pos(temp_pos);	
					}
					else
					{
						if(last_dir!=0)
						{
							if(next_id==i)
							{
								if((target.X>move_temp_x_min)&&(target.X<move_temp_x_max))
								temp_pos.X = target.X;
								if(!((temp_pos.X==pos.X)&&(temp_pos.Y==pos.Y)))
								{						
									Add_Next_Pos(temp_pos);	
								}	
							}
						}
					}
//USPRINTF("%s %d:temp_pos.Y:%d  temp_pos.X:%d \n", __FUNCTION__, __LINE__,temp_pos.Y,temp_pos.X);
					temp_pos.Y = Get_Path_Line_Y(next_id-1);
					if(!((temp_pos.X==pos.X)&&(temp_pos.Y==temp_y)))
					{
						Add_Next_Pos(temp_pos);	
					}
						}
						if(next_id==i)break;
					}
				}			

//				USPRINTF("%s %d:temp_id:%d  last_y:%d Get_Path_Line_Y(next_id):%d\n", __FUNCTION__, __LINE__,temp_id,last_y,Get_Path_Line_Y(next_id));				
				while(last_y<Get_Path_Line_Y(next_id))
				{	
					equal = 0;					
					k = 1;
					next_y = Get_Path_Line_Y(next_id);
					
					temp_offset = Get_Path_Line_X_Off(next_id);
//					if(temp_offset>4)
//					{
//						if(offset>2)
//						{
//							offset = 2;
//						}
//					}
//					else 
						if(temp_offset>1)
					{
						if(offset>1)
						{
							offset = 1;
							break;
						}
					}
					else
					{
//						if(offset>0)
//						{
//							offset = 0;
//							break;
//						}
						offset = 0;
						Reset_Next_Pos();
						retval = 0;
						return retval;
					}
					
					if(x_min<(Get_Path_Line_X(next_id) + offset))
					{
						x_min = Get_Path_Line_X(next_id) + offset;

//						x_min += 1;
						if((move_temp_x_min<=x_min)&&((move_temp_x_max)>=x_min))
						{
							move_temp_x_min = x_min;
						}
						else
						{
							break;
						}						
					}
					
					if((x_max>(Get_Path_Line_X(next_id)+Get_Path_Line_X_Off(next_id) - 1*offset)))
					{
						x_max = Get_Path_Line_X(next_id)+Get_Path_Line_X_Off(next_id)- 1*offset;	

//						x_max -= 1;
						if((move_temp_x_min<=x_max)&&((move_temp_x_max)>=x_max))
						{
							move_temp_x_max = x_max;
						}
						else
						{
							break;
						}						
					}				
						
					if(x_min<=x_max)
					{
						if(next_id<i)
						{
							next_id++;
							if(next_id==i)break;
						}
						else
						{
							break;
						}
					}
					else
					{
						break;
					}
					last_y = next_y;
					k=2;
				}			
//				USPRINTF("%s %d:temp_id:%d  last_y:%d Get_Path_Line_Y(next_id):%d\n", __FUNCTION__, __LINE__,temp_id,last_y,Get_Path_Line_Y(next_id));
				if(k!=0)
				{
					k = 0;
					
					if(next_id!=i)
					{
						if((temp_pos.X>=move_temp_x_min)&&(temp_pos.X<=move_temp_x_max))
						{
							
						}
						else
						{
//							temp_pos.X = (move_temp_x_min + move_temp_x_max)/2;
							
							temp_pos.X = Points_Distance(temp_pos.X,move_temp_x_min,move_temp_x_max);	
							if(!((temp_pos.X==pos.X)&&(temp_pos.Y==pos.Y)))
							{						
								Add_Next_Pos(temp_pos);	
							}	
						}
					}
					else
					{							
						if((target.X>=move_temp_x_min)&&(target.X<=move_temp_x_max))
						{
							if((temp_pos.X>=move_temp_x_min)&&(temp_pos.X<=move_temp_x_max))
							{
//								temp_y = target.Y;
							}
							else
							{
								if(temp_y!=target.Y)
								temp_pos.X = target.X;
							}
						}
						else
						{							
							if((temp_pos.X>=move_temp_x_min)&&(temp_pos.X<=move_temp_x_max))
							{
								
							}
							else
							{
//							temp_pos.X = (move_temp_x_min + move_temp_x_max)/2;
							
							temp_pos.X = Points_Distance(temp_pos.X,move_temp_x_min,move_temp_x_max);	
								if(!((temp_pos.X==pos.X)&&(temp_pos.Y==pos.Y)))
								{						
									Add_Next_Pos(temp_pos);	
								}	
							}
						}					
					}
					//USPRINTF("%s %d:temp_pos.Y:%d  temp_pos.X:%d \n", __FUNCTION__, __LINE__,temp_pos.Y,temp_pos.X);
					temp_pos.Y = temp_y;
					if(!((temp_pos.X==pos.X)&&(temp_pos.Y==pos.Y)))
					{
						
						Add_Next_Pos(temp_pos);	
					}
					else
					{
						if(last_dir!=0)
						{
							if(next_id==i)
							{
								if((target.X>move_temp_x_min)&&(target.X<move_temp_x_max))
								temp_pos.X = target.X;
								if(!((temp_pos.X==pos.X)&&(temp_pos.Y==pos.Y)))
								{						
									Add_Next_Pos(temp_pos);	
								}	
							}
						}
					}
					if((temp_pos.X>(Get_Path_Line_X(next_id-1))+Get_Path_Line_X_Off(next_id-1))||(temp_pos.X<Get_Path_Line_X(next_id-1)))
					{
						temp_pos.X = (move_temp_x_min + move_temp_x_max)/2;
						if(!(temp_pos.X==pos.X))
						{
							Add_Next_Pos(temp_pos);	
						}
					}
//USPRINTF_ZZ("%s %d:temp_pos.Y:%d  temp_pos.X:%d \n", __FUNCTION__, __LINE__,temp_pos.Y,temp_pos.X);
					temp_pos.Y = Get_Path_Line_Y(next_id-1);
					if(!((temp_pos.X==pos.X)&&(temp_pos.Y==temp_y)))
					{						
						Add_Next_Pos(temp_pos);	
					}	
					if(next_id==i)break;
					last_y = temp_pos.Y;
					temp_y = last_y;
//USPRINTF("%s %d:temp_pos.Y:%d  temp_pos.X:%d \n", __FUNCTION__, __LINE__,temp_pos.Y,temp_pos.X);
					x_min = Get_Path_Line_X(next_id-1);
					x_max = x_min + Get_Path_Line_X_Off(next_id-1);			
					if(Get_Path_Line_X(next_id)>x_min)
					{
						x_min = Get_Path_Line_X(next_id);
					}
					
					if((Get_Path_Line_X(next_id)+Get_Path_Line_X_Off(next_id))<x_max)
					{
						x_max = Get_Path_Line_X(next_id)+Get_Path_Line_X_Off(next_id);
					}
					temp_x_min= x_min+1;
					temp_x_max= x_max-1;
					
					move_temp_x_min= temp_x_min;
					move_temp_x_max= temp_x_max;
					
					if((temp_pos.X>move_temp_x_max)||(temp_pos.X<move_temp_x_min))
					{
						temp_pos.X = (move_temp_x_min + move_temp_x_max)/2;
						if(!(temp_pos.X==pos.X))
						{
							Add_Next_Pos(temp_pos);	
						}
					}
					temp_offset = Get_Path_Line_X_Off(next_id);
//					if(temp_offset>4)
//					{
//						offset = 2;
//						move_temp_x_min += 2;
//						move_temp_x_max -= 2;
//					}
//					else 
	/*
						if(temp_offset>2)
					{
						offset = 1;
						move_temp_x_min += 1;
						move_temp_x_max -= 1;
					}
					else
					{
//						offset = 0;
						offset = 0;
				Reset_Next_Pos();
				retval = 0;
				return retval;
					}*/
			
					if(next_id<i)
					{
						next_id++;
						if(next_id==i)
						{
							if(next_id!=i)
					{
						if((temp_pos.X>=move_temp_x_min)&&(temp_pos.X<=move_temp_x_max))
						{
							
						}
						else
						{
//							temp_pos.X = (move_temp_x_min + move_temp_x_max)/2;
							
							temp_pos.X = Points_Distance(temp_pos.X,move_temp_x_min,move_temp_x_max);	
							if(!((temp_pos.X==pos.X)&&(temp_pos.Y==pos.Y)))
							{						
								Add_Next_Pos(temp_pos);	
							}	
						}
					}
					else
					{							
						if((target.X>=move_temp_x_min)&&(target.X<=move_temp_x_max))
						{
							if((temp_pos.X>=move_temp_x_min)&&(temp_pos.X<=move_temp_x_max))
							{
//								temp_y = target.Y;
							}
							else
							{
								if(temp_y!=target.Y)
								temp_pos.X = target.X;
							}
						}
						else
						{							
							if((temp_pos.X>=move_temp_x_min)&&(temp_pos.X<=move_temp_x_max))
							{
								
							}
							else
							{
//							temp_pos.X = (move_temp_x_min + move_temp_x_max)/2;
							
							temp_pos.X = Points_Distance(temp_pos.X,move_temp_x_min,move_temp_x_max);
							if(!((temp_pos.X==pos.X)&&(temp_pos.Y==pos.Y)))
							{						
								Add_Next_Pos(temp_pos);	
							}	
							}
						}					
					}
//USPRINTF("%s %d:temp_pos.Y:%d  temp_pos.X:%d \n", __FUNCTION__, __LINE__,temp_pos.Y,temp_pos.X);
					temp_pos.Y = temp_y;
					if(!((temp_pos.X==pos.X)&&(temp_pos.Y==pos.Y)))
					{
						Add_Next_Pos(temp_pos);	
					}
					else
					{
						if(last_dir!=0)
						{
							if(next_id==i)
							{
								if((target.X>move_temp_x_min)&&(target.X<move_temp_x_max))								
								temp_pos.X = target.X;
								if(!((temp_pos.X==pos.X)&&(temp_pos.Y==pos.Y)))
								{						
									Add_Next_Pos(temp_pos);	
								}	
							}
						}
					}
//USPRINTF("%s %d:temp_pos.Y:%d  temp_pos.X:%d \n", __FUNCTION__, __LINE__,temp_pos.Y,temp_pos.X);
					temp_pos.Y = Get_Path_Line_Y(next_id-1);
					if(!((temp_pos.X==pos.X)&&(temp_pos.Y==temp_y)))
					{						
						Add_Next_Pos(temp_pos);	
					}
					if(next_id==i)break;
						}
					}
				}				
				
				if(equal==1)
				{
					if(next_id!=i)
					{
						if((temp_pos.X>x_min)&&(temp_pos.X<x_max))
						{
							
						}
						else
						{
//							temp_pos.X = (move_temp_x_min + move_temp_x_max)/2;
							
							temp_pos.X = Points_Distance(temp_pos.X,move_temp_x_min,move_temp_x_max);	
							if(!((temp_pos.X==pos.X)&&(temp_pos.Y==pos.Y)))
							{						
								Add_Next_Pos(temp_pos);	
							}	
						}
					}
					else
					{							
						if((target.X>x_min)&&(target.X<x_max))
						{
							if(temp_y!=target.Y)
							temp_pos.X = target.X;
						}
						else
						{
//							temp_pos.X = (move_temp_x_min + move_temp_x_max)/2;
							
							temp_pos.X = Points_Distance(temp_pos.X,move_temp_x_min,move_temp_x_max);	
							if(!((temp_pos.X==pos.X)&&(temp_pos.Y==pos.Y)))
								{						
									Add_Next_Pos(temp_pos);	
								}	
						}					
					}
//USPRINTF("%s %d:temp_pos.Y:%d  temp_pos.X:%d \n", __FUNCTION__, __LINE__,temp_pos.Y,temp_pos.X);				
					temp_pos.Y = temp_y;
					if(!((temp_pos.X==pos.X)&&(temp_pos.Y==pos.Y)))
					{
						Add_Next_Pos(temp_pos);	
					}
					else
					{
						if(last_dir!=0)
						{
							if(next_id==i)
							{
								if((target.X>move_temp_x_min)&&(target.X<move_temp_x_max))								
								temp_pos.X = target.X;
								if(!((temp_pos.X==pos.X)&&(temp_pos.Y==pos.Y)))
								{						
									Add_Next_Pos(temp_pos);	
								}	
							}
						}
					}
//USPRINTF("%s %d:temp_pos.Y:%d  temp_pos.X:%d \n", __FUNCTION__, __LINE__,temp_pos.Y,temp_pos.X);										
					temp_pos.Y = Get_Path_Line_Y(next_id-1);			
					if(!((temp_pos.X==pos.X)&&(temp_pos.Y==temp_y)))
					{
						Add_Next_Pos(temp_pos);	
					}
					last_y = temp_pos.Y;
					temp_y = last_y;
//USPRINTF("%s %d:temp_pos.Y:%d  temp_pos.X:%d \n", __FUNCTION__, __LINE__,temp_pos.Y,temp_pos.X);
					x_min = Get_Path_Line_X(next_id-1);
					x_max = x_min + Get_Path_Line_X_Off(next_id-1);			
					if(Get_Path_Line_X(next_id)>x_min)
					{
						x_min = Get_Path_Line_X(next_id);
					}
					
					if((Get_Path_Line_X(next_id)+Get_Path_Line_X_Off(next_id))<x_max)
					{
						x_max = Get_Path_Line_X(next_id)+Get_Path_Line_X_Off(next_id);
					}
					temp_x_min= x_min-1;
					temp_x_max= x_max+1;
			
					move_temp_x_min= temp_x_min;
					move_temp_x_max= temp_x_max;
					
					if((temp_pos.X>move_temp_x_max)||(temp_pos.X<move_temp_x_min))
					{
						temp_pos.X = (move_temp_x_min + move_temp_x_max)/2;
						if(!(temp_pos.X==pos.X))
						{
							Add_Next_Pos(temp_pos);	
						}
					}
					
					temp_offset = Get_Path_Line_X_Off(next_id);
//					if(temp_offset>4)
//					{
//						offset = 2;
//						move_temp_x_min += 2;
//						move_temp_x_max -= 2;
//					}
//					else 
	/*
						if(temp_offset>2)
					{
						offset = 1;
						move_temp_x_min += 1;
						move_temp_x_max -= 1;
					}
					else
					{
//						offset = 0;
						offset = 0;
				Reset_Next_Pos();
				retval = 0;
				return retval;
					}*/
			
//					temp_id = next_id;
					if(next_id<i)
					{
						next_id++;
						if(next_id==i)
						{
							if(next_id!=i)
					{
						if((temp_pos.X>=move_temp_x_min)&&(temp_pos.X<=move_temp_x_max))
						{
							
						}
						else
						{
//							temp_pos.X = (move_temp_x_min + move_temp_x_max)/2;
							
							temp_pos.X = Points_Distance(temp_pos.X,move_temp_x_min,move_temp_x_max);	
							if(!((temp_pos.X==pos.X)&&(temp_pos.Y==pos.Y)))
								{						
									Add_Next_Pos(temp_pos);	
								}	
						}
					}
					else
					{							
						if((target.X>=move_temp_x_min)&&(target.X<=move_temp_x_max))
						{
							if((temp_pos.X>=move_temp_x_min)&&(temp_pos.X<=move_temp_x_max))
							{
//								temp_y = target.Y;
							}
							else
							{
								if(temp_y!=target.Y)
								temp_pos.X = target.X;
							}
						}
						else
						{							
							if((temp_pos.X>=move_temp_x_min)&&(temp_pos.X<=move_temp_x_max))
							{
								
							}
							else
							{
//							temp_pos.X = (move_temp_x_min + move_temp_x_max)/2;
							
							temp_pos.X = Points_Distance(temp_pos.X,move_temp_x_min,move_temp_x_max);
							if(!((temp_pos.X==pos.X)&&(temp_pos.Y==pos.Y)))
								{						
									Add_Next_Pos(temp_pos);	
								}	
							}
						}					
					}
//USPRINTF("%s %d:temp_pos.Y:%d  temp_pos.X:%d \n", __FUNCTION__, __LINE__,temp_pos.Y,temp_pos.X);
					temp_pos.Y = temp_y;
					if(!((temp_pos.X==pos.X)&&(temp_pos.Y==pos.Y)))
					{						
						Add_Next_Pos(temp_pos);	
					}
					else
					{
						if(last_dir!=0)
						{
							if(next_id==i)
							{
								if((target.X>move_temp_x_min)&&(target.X<move_temp_x_max))
								temp_pos.X = target.X;
								if(!((temp_pos.X==pos.X)&&(temp_pos.Y==pos.Y)))
								{						
									Add_Next_Pos(temp_pos);	
								}	
							}
						}
					}
//USPRINTF("%s %d:temp_pos.Y:%d  temp_pos.X:%d \n", __FUNCTION__, __LINE__,temp_pos.Y,temp_pos.X);
					temp_pos.Y = Get_Path_Line_Y(next_id-1);
					if(!((temp_pos.X==pos.X)&&(temp_pos.Y==temp_y)))
					{
						Add_Next_Pos(temp_pos);	
					}
						}
						if(next_id==i)break;
					}
					
					equal = 0;
				}	
					
			}
			while(next_id<i);
		}		
	}
	else
	{
		Add_Next_Pos(target);
	}
	
	if(Get_Next_Pos_Count())
	{
		Add_Next_Pos(target);
	}
	return  retval;
}

uint8_t Move_Draw_Line(Point16_t pos,Point16_t t_pos,uint8_t flag,uint8_t offset)
{
	int16_t x,y,min_x,min_y,max_x,max_y;	
	uint8_t i=0;
	
	if(pos.X>t_pos.X)
	{
		max_x = pos.X;
		min_x = t_pos.X;
	}
	else if(pos.X<t_pos.X)
	{
		min_x = pos.X;
		max_x = t_pos.X;		
	}
	else
	{
		min_x = pos.X;
		max_x = pos.X;
	}

	if(pos.Y>t_pos.Y)
	{
		max_y = pos.Y;
		min_y = t_pos.Y;
	}
	else if(pos.Y<t_pos.Y)
	{
		min_y = pos.Y;
		max_y = t_pos.Y;		
	}
	else
	{
		min_y = pos.Y;
		max_y = pos.Y;
	}
	
	if(flag==0)
	{
		if(offset==0)//x=x++  y++
		{
			for(x=min_x+1;x<=max_x+2;x++)
			{
				for(y=min_y;y<=max_y;y++)
				{
					if(is_a_block(x,y)==1)
					{
						break;
					}
				}
				if(y!=(max_y+1))
				{
					break;
				}
				i++;
			}
		}
		else//x=x-- y++
		{
			for(x=min_x-1;x>=max_x-2;x--)
			{
				for(y=min_y;y<=max_y;y++)
				{
					if(is_a_block(x,y)==1)
					{
						break;
					}
				}
				if(y!=(max_y+1))
				{
					break;
				}
				i++;
			}			
		}
	}
	else
	{
		if(offset==0)//y=y++ x++
		{
			for(y=min_y+1;y<=max_y+2;y++)
			{
				for(x=min_x;x<=max_x;x++)
				{
					if(is_a_block(x,y)==1)
					{
						break;
					}
				}
				if(x!=(max_x+1))
				{
					break;
				}
				i++;
			}			
		}
		else//y=y-- x++
		{
			for(y=min_y-1;y>=max_y-2;y--)
			{
				for(x=min_x;x<=max_x;x++)
				{
					if(is_a_block(x,y)==1)
					{
						break;
					}
				}
				if(x!=(max_x+1))
				{
					break;
				}
				i++;
			}			
		}		
	}
	
	return i;
}

//对拐点做偏移处理，防止机器行走时碰到障碍物
uint8_t Move_Offset(int16_t *x_next1, int16_t *y_next1,int16_t *x_next2, int16_t *y_next2)
{
	int16_t min,max;
	uint8_t retval=0;
	Point16_t pos,next_pos;
	
	pos.X = *x_next1;
	pos.Y = *y_next1;
	
	next_pos.X = *x_next2;
	next_pos.Y = *y_next2;
	
//	USPRINTF("%s %dnext_pos.X:%d next_pos.Y:%d\r\n", __FUNCTION__, __LINE__,next_pos.X,next_pos.Y);
	if((pos.X==next_pos.X)&&(pos.Y!=next_pos.Y))
	{
		max = Move_Draw_Line(pos,next_pos,0,0);
		min = Move_Draw_Line(pos,next_pos,0,1);
//		USPRINTF("%s %d  min:%d max:%d\r\n", __FUNCTION__, __LINE__,min,max);
		
		if((max>0)&&(min>0))
		{						
			retval = 1;
		}
		else if(max>1)
		{
			*x_next1 = pos.X+1;
			*y_next1 = pos.Y;
			
			*x_next2 = pos.X+1;
			*y_next2 = next_pos.Y;
			
			retval = 1;
		}
		else if(min>1)
		{
			*x_next1 = pos.X-1;
			*y_next1 = pos.Y;
			
			*x_next2 = pos.X-1;
			*y_next2 = next_pos.Y;
			
			retval = 1;		
		}
		else
		{
			retval = 0;
		}
	}
	else if((pos.X!=next_pos.X)&&(pos.Y==next_pos.Y))
	{
		max = Move_Draw_Line(pos,next_pos,1,0);
		min = Move_Draw_Line(pos,next_pos,1,1);
//		USPRINTF("%s %d  min:%d max:%d\r\n", __FUNCTION__, __LINE__,min,max);
		
		if((max>0)&&(min>0))
		{
			retval = 1;
		}
		else if(max>1)
		{
			*x_next1 = pos.X;
			*y_next1 = pos.Y+1;
			
			*x_next2 = next_pos.X;
			*y_next2 = pos.Y+1;
		
			retval = 1;
		}
		else if(min>1)
		{
			*x_next1 = pos.X;
			*y_next1 = pos.Y-1;
			
			*x_next2 = next_pos.X;
			*y_next2 = pos.Y-1;
		
			retval = 1;
		}
		else
		{
			retval = 0;
		}
	}
	
	return retval;
}

/*
 * By given a target, find the path to the target.
 * In this version, a up-side-down tree is built for the path searching, the target will always be
 * the root of the tree, the nodes are the line segments that can directly reach each other, a level
 * is set for each node, and the level of the target will always be 1, when the line segment of the
 * robot position is found, and the level of that line segment is set, start to trace the path back
 * to the root, which is the target.
 *
 * @param pos	The current robot position
 * @param x	The target X Coordinate that the robot wants to go
 * @param y	The target Y Coordinate that the robot wants to go
 * @param *x_next	The next X Coordinate that the robot should go before reaching the target
 * @param *y_next	The next Y Coordinate that the robot should go before reaching the target
 * @param last_dir	Last robot direction
 *
 * @return	-2: Robot is trapped
 * 		-1: Path to target is not found
 * 		1:  Path to target is found
 */
int8_t path_move_to_unclean_area(Point16_t pos, int16_t x, int16_t y, int16_t *x_next, int16_t *y_next, uint16_t last_dir)
{
	int8_t	found=-2;
	int16_t	 temp_angle=0,x_min=0, x_max=0, y_min=0, y_max=0;
	uint8_t line_level=0,cur_level=0;
	int16_t line_y=0, cur_y=0;
	int16_t i=0,f=0;
	Point16_t temp_pos,next_pos,temp_next_pos;
	
	last_dir=last_dir;
	temp_angle = Gyro_GetAngle(0);
	
	if(((temp_angle<450)||(temp_angle>=3150))||((temp_angle>1350)&&(temp_angle<=2250)))
	{
		last_dir = 0;
	}	
	else
	{
		last_dir = 1;
	}
	
//	last_dir = g_2nd_clean_flag;
	
	x_min = xMin;
	x_max = xMax;
	y_min = yMin;
	y_max = yMax;
	
	if((is_a_block(x,y)==1))
	{
		return found;
	}
	Reset_Next_Pos();
	Set_Search_Range(0);
#ifdef	ZONE_WALLFOLLOW
	Point16_t zone;
	if ( CM_IsSingleRoomMode() == 0 ) 
	{
		if (Zone_GetZoneWallFollowStatus() == 2) 
		{
			zone = Zone_GetCurrentZone();
			x_min = zone.X - ZONE_SIZE_HALF - 3;
			x_max = zone.X + ZONE_SIZE_HALF + 3;
			y_min = zone.Y - ZONE_SIZE_HALF - 3;
			y_max = zone.Y + ZONE_SIZE_HALF + 3;
		}
		else if (Zone_GetZoneWallFollowStatus() == 3) 
		{
			x_min = xMinSearch;
			x_max = xMaxSearch;
			y_min = yMinSearch;
			y_max = yMaxSearch;
			Set_Search_Range(1);
		}
	}
#endif

	USPRINTF("%s %d: %d (%d, %d) (%d, %d) x_min:%d x_max:%d y_min:%d y_max:%d\n", __FUNCTION__, __LINE__, found,pos.X,pos.Y, x,y,x_min,x_max,y_min,y_max);
	
	next_move_step = 1;
	
	i = pos.Y;
	i -= y;
	if(i<0)i=-i;
	
	if(i>POS_LINE_CNT)
	{
		next_move_step = 2;
		if((i/2)>POS_LINE_CNT)
		{
			next_move_step = 3;
		}
	}
	
	do
	{
		USPRINTF("%s %d: next_move_step：%d\n", __FUNCTION__, __LINE__,next_move_step);	
		if(All_Line_Spread(pos,x,y,x_min,x_max,y_min,y_max))
		{
			temp_pos = pos;
			cur_y = pos.Y;
			cur_level = Get_Pos_Line_Level(temp_pos);
			if(cur_level>MAX_LEVEL)
			{
				
				return found;
			}
			
			temp_pos.X = x;
			temp_pos.Y = y;
			line_y = y;
			line_level = Get_Pos_Line_Level(temp_pos);
			if(line_level>MAX_LEVEL)
			{
				return found;
			}		

			Path_Line_Dump();
			Path_Trace_Path(pos,temp_pos,line_level,line_y,cur_level,cur_y);
			Path_Line_Dump();
			Path_To_Move(pos,temp_pos,(Point16_t *)Next_Pos,0);
			
			break;
		}
		else
		{
			Reset_Next_Pos();
			
			if(Get_Path_Line_Count()==(POS_LINE_CNT))
			{
				next_move_step++;
				if(next_move_step>3)
				{
					USPRINTF("%s %d: next_move_step：%d\n", __FUNCTION__, __LINE__,next_move_step);	
					break;	
				}					
			}	
			else
			{
				break;
			}
		}
	}
	while(1);

	if(Get_Next_Pos_Count())
	{
		for(i=0;i<Get_Next_Pos_Count();i++)	
		{
			USPRINTF("%s %d: (%d, %d)\n", __FUNCTION__, __LINE__,Next_Pos[i].X, Next_Pos[i].Y);	

		}		
		found = 1;
				
		FAdd_Next_Pos(pos);	
		if(i==2)
		{
			temp_next_pos = Get_Next_Pos_From_Id(0);
			next_pos = Get_Next_Pos_From_Id(1);
			
			Reset_Next_Pos();			
			Add_Next_Pos(pos);
			Add_Next_Pos(temp_next_pos);
			Add_Next_Pos(next_pos);	
			temp_pos.X = x;
			temp_pos.Y = y;			
			Add_Next_Pos(temp_pos);			
		}
		else
		{
			i = Get_Next_Pos_Count();		
			i -= 2;
			f = 1;
			
			while(f<i)
			{
				temp_next_pos = Get_Next_Pos_From_Id(f);
				next_pos = Get_Next_Pos_From_Id(f+1);
				
//				if(Move_Offset(&temp_next_pos.X,&temp_next_pos.Y,&next_pos.X,&next_pos.Y)==0)
//				{				
//					found = -1;
//					break;
//				}
				
				Set_Next_Pos(f,temp_next_pos);
				Set_Next_Pos(f+1,next_pos);
				
				f++;
			}
		}
		
		for(i=0;i<Get_Next_Pos_Count();i++)	
		{
			USPRINTF("%s %d: (%d, %d)\n", __FUNCTION__, __LINE__,Next_Pos[i].X, Next_Pos[i].Y);	
		}		
		
		temp_pos = Updata_Next_Pos(pos);
		
		*x_next = temp_pos.X;
		*y_next = temp_pos.Y;	
	}
	
	if(found==-2)
	{
		USPRINTF("/r/n**************************Robot is trapped*****************************/r/n");
		Path_Line_Dump();
	}
	
	USPRINTF("%s %d: %d (%d, %d) (%d, %d) (%d, %d) (%d, %d)\n", __FUNCTION__, __LINE__, found,pos.X,pos.Y, x,y, *x_next, *y_next,temp_pos.X,temp_pos.Y);
		
	return found;
}

/*
 * Initialization for searching shortest path by using the up-side-down tree.
 *
 * @param dg	direction to go
 *
 * @return
 */
void path_position_init(uint8_t data)
{
	line_cnt = 0;	
	Init_Path_Line();
}

/*
 * This becomes a dummy function, the process for finding the path is in path_move_to_unclean_area().
 *
 * @param x	The target X Coordinate that the robot wants to go
 * @param y	The target Y Coordinate that the robot wants to go
 * @param *x_next	The next X Coordinate that the robot should go before reaching the target
 * @param *y_next	The next Y Coordinate that the robot should go before reaching the target
 * @param last_dir	Last robot direction
 *
 * @return
 */
int8_t path_find_shortest_path(int16_t xID, int16_t yID, int16_t endx, int16_t endy, uint8_t bound, uint16_t last_dir)
{
	int16_t *x_next, *y_next;
	Point16_t	pos;

	last_dir = last_dir;
	bound = bound;
	pos.X = xID;
	pos.Y = yID;

	x_next = &xID;
	y_next = &yID;

	return path_move_to_unclean_area(pos, endx, endy, x_next, y_next, last_dir);
}
