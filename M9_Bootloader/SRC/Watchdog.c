


#include "Watchdog.h"

#define IWDG_START            (uint32_t)0x0000cccc;
#define IWDG_WRITE_ACCESS     (uint32_t)0x00005555;
#define IWDG_RELOAD_VALUE     (uint32_t)0x000000ff;
#define IWDG_REFRESH          (uint32_t)0x0000aaaa;

void WatchDog_Configuration(void)
{
	#ifdef WD_ENABLE
	uint32_t i=0;
	
	IWDG->KR = IWDG_START;//Activate IWDG
	IWDG->KR = IWDG_WRITE_ACCESS;//Enable_Write_Access to IWDG register
	IWDG->PR = 0;//Set Prescaler by 4
	IWDG->RLR = IWDG_RELOAD_VALUE;//Set reload value to 
	while(IWDG->SR)
	{
		i++;
		if(i>0xfffffffe)break;
	}
	IWDG->KR  = IWDG_REFRESH;//refresh 
	#endif
}

void Feed_WatchDog(void)
{
	#ifdef WD_ENABLE
	IWDG->KR  = IWDG_REFRESH;//refresh 
	#endif
}

void Disable_WatchDog(void)
{
}






