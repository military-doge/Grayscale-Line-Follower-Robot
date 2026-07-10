#include "key.h"


/**************************************************************************
Function: Key scan
Input   : Double click the waiting time
Output  : 0: No action  1: click  2: Double click
**************************************************************************/
u8 click_N_Double (u8 time)
{
    static  u8 flag_key,count_key,double_key=0;
    static  u16 count_single,Forever_count;
    if(DL_GPIO_readPins(KEY_PORT,KEY_key_PIN)>0)  Forever_count++;
    else        Forever_count=0;
    if((DL_GPIO_readPins(KEY_PORT,KEY_key_PIN)>0)&&0==flag_key)     flag_key=1;
    if(0==count_key)
    {
            if(flag_key==1)
            {
                double_key++;
                count_key=1;
            }
            if(double_key==3)
            {
                double_key=0;
                count_single=0;
                return 2;
            }
    }
    if(0==DL_GPIO_readPins(KEY_PORT,KEY_key_PIN))          flag_key=0,count_key=0;
    if(1==double_key)
    {
        count_single++;
        if(count_single>time&&Forever_count<time)
        {
            double_key=0;
            count_single=0;
			return 1;
        }
        if(Forever_count>time)
        {
            double_key=0;
            count_single=0;
        }
    }
    return 0;
}
/**************************************************************************
Function: Long press detection
Input   : none
Output  : 0: No action  1: Long press for 2 seconds
**************************************************************************/
u8 Long_Press(void)
{
        static u16 Long_Press_count,Long_Press;
      if(Long_Press==0&&KEY==0)  Long_Press_count++;
    else                       Long_Press_count=0;
        if(Long_Press_count>200)
      {
            Long_Press=1;
            Long_Press_count=0;
            return 1;
        }
        if(Long_Press==1)
        {
            Long_Press=0;
        }
        return 0;
}


void Key(void)
{
	u8 tmp;
	tmp=click_N_Double(50);
	if(tmp==1)
	{
		Flag_Stop=!Flag_Stop;
		if(!Flag_Stop)
		{
			MotorA.Target_Encoder = MotorB.Target_Encoder = 1.0f;
		}
		else
		{
			MotorA.Target_Encoder = MotorB.Target_Encoder = 0.0f;
		}
	}
}
