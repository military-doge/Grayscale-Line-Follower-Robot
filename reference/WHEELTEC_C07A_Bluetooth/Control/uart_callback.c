#include "ti_msp_dl_config.h"
#include <string.h>
#include "board.h"
#include "uart_callback.h"
//Bluetooth remote control associated flag bits
//蓝牙遥控相关的标志位
int Flag_Left, Flag_Right, Flag_Direction=0, Turn_Flag;


extern uint8_t PID_Send;

#define BT_PACKET_SIZE (200)
volatile uint8_t gBTPacket[BT_PACKET_SIZE];
volatile uint8_t gBTCounts = 0 ;
volatile uint8_t lastBTCounts = 0 ;
volatile bool gCheckBT;

uint8_t RecvOverFlag = 0;

void bt_control(uint8_t recv);

void BT_DAMConfig(void)
{
    DL_DMA_disableChannel(DMA, DMA_CH0_CHAN_ID);
    DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)(&UART_1_INST->RXDATA));
    DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t) &gBTPacket[0]);
    DL_DMA_setTransferSize(DMA, DMA_CH0_CHAN_ID, BT_PACKET_SIZE);
    DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);
}

void BTBufferHandler(void)
{
    uint32_t tick = 0;
    static uint8_t handleflag = 0;
    static uint8_t handleSize = 0;
    static uint8_t lastSize = 0;

    //recvsize为已经接收的数据个数
    uint8_t recvsize = BT_PACKET_SIZE - DL_DMA_getTransferSize(DMA, DMA_CH0_CHAN_ID);

    uint32_t ticks = 0;
    if( recvsize != lastSize)
    {
        handleflag=1;
        tick = Systick_getTick();//正在接收数据,刷新时间
    }
    else
    {
        //数据已停止接收,超时后开始处理
        if( ((tick-Systick_getTick())&SysTickMAX_COUNT) >= SysTick_MS(1) && handleflag == 1)
        {
            handleflag = 0;

            //处理数据
            // for(uint8_t i=handleSize;i<recvsize;i++)
            //     printf("%c",gBTPacket[i]);
            // printf("\r\n");

            //处理串口数据
            for(uint8_t i=handleSize;i<recvsize;i++)
                bt_control(gBTPacket[i]);


            //记录本次数据处理的位置
            handleSize = recvsize;

            //处理到一定数据量时刷新一次dma搬运地址,不让dma完成搬运
            if( recvsize >= BT_PACKET_SIZE/2 )
            {
                recvsize=0;
                handleSize=0;
                lastSize = 0;
                BT_DAMConfig();
            }
        }
    }

    lastSize = recvsize;
}


void UART_1_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_1_INST)) {
        case DL_UART_IIDX_RX:
            gBTPacket[gBTCounts++] = DL_UART_Main_receiveData(UART_1_INST);
            break;
        case DL_UART_MAIN_IIDX_DMA_DONE_RX:
            BT_DAMConfig();
        default:
            break;
    }
}

void bt_control(uint8_t recv)
{
    static  int Usart_Receive=0;//蓝牙接收相关变量
    static uint8_t Flag_PID,i,j,Receive[50];
    static float Data;

    // 接收发送过来的数据保存
    Usart_Receive = recv;

	if(Usart_Receive==0x4B) 
			//Enter the APP steering control interface
		  //进入APP转向控制界面
			Turn_Flag=1;  
	  else	if(Usart_Receive==0x49||Usart_Receive==0x4A) 
      // Enter the APP direction control interface		
			//进入APP方向控制界面	
			Turn_Flag=0;	
		
		if(Turn_Flag==0) 
		{
			//App rocker control interface command
			//APP摇杆控制界面命令
			if(Usart_Receive>=0x41&&Usart_Receive<=0x48)  
			{	
				Flag_Direction=Usart_Receive-0x40;
			}
			else	if(Usart_Receive<=8)   
			{			
				Flag_Direction=Usart_Receive;
			}	
			else  Flag_Direction=0;
		}
		else if(Turn_Flag==1)
		{
			//APP steering control interface command
			//APP转向控制界面命令
			if     (Usart_Receive==0x43) Flag_Left=0,Flag_Right=1; //Right rotation //右自转
			else if(Usart_Receive==0x47) Flag_Left=1,Flag_Right=0; //Left rotation  //左自转
			else                         Flag_Left=0,Flag_Right=0;
			if     (Usart_Receive==0x41||Usart_Receive==0x45) Flag_Direction=Usart_Receive-0x40;
			else  Flag_Direction=0;
		}
		if(Usart_Receive==0x58)  RC_Velocity=RC_Velocity+100; //Accelerate the keys, +100mm/s //加速按键，+100mm/s
		if(Usart_Receive==0x59)  RC_Velocity=RC_Velocity-100; //Slow down buttons,   -100mm/s //减速按键，-100mm/s
	  
    if(Usart_Receive==0x7B) Flag_PID=1;   //APP参数指令起始位
    if(Usart_Receive==0x7D) Flag_PID=2;   //APP参数指令停止位
    if(Flag_PID==1)  //采集数据
    {
            Receive[i]=Usart_Receive;
            i++;
    }
    if(Flag_PID==2)  //分析数据
    {
            if(Receive[3]==0x50)               PID_Send=1;
            else if(Receive[1]!=0x23)
            {
                for(j=i;j>=4;j--)
                {
                    Data+=(Receive[j-1]-48)*pow(10,i-j);
                }
                switch(Receive[1])
                {
                    case 0x30:  Velocity_KP=Data;break;
                    case 0x31:  Velocity_KI=Data;break;
                    case 0x32: RC_Velocity=Data;break;//Velocity_Kp=Data;break;
                    case 0x33: break;//Velocity_Ki=Data;break;
                    case 0x34: break;//Turn_Kp=Data;break;
                    case 0x35:  break;//Turn_Kd=Data;break;
                    case 0x36:  break; //预留
                    case 0x37:  break; //预留
                    case 0x38:  break; //预留
                }
            }
                Flag_PID=0;
                i=0;
                j=0;
                Data=0;
                memset(Receive, 0, sizeof(uint8_t)*50);//数组清零
    }

}
