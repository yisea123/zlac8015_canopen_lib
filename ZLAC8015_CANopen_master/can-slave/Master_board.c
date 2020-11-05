/**
  ******************************************************************************
  * @file CAN/Normal/main.c 
  * @author  MCD Application Team
  * @version  V3.0.0
  * @date  04/06/2009
  * @brief  Main program body
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "platform_config.h"

/* test_master */
#include "canfestival.h"
#include "Master.h"
#include "Slave.h"
#include "TestMasterSlave.h"
#include "direct_can.h"
unsigned char CanreveiveFlag=0;
Message m;

#define KEY0  GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_4)//读取按键0
#define KEY1  GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3)//读取按键1
#define KEY2  GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_2)//读取按键2 
#define WK_UP   GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)//读取按键3(WK_UP) 

 

#define KEY0_PRES 	1	//KEY0按下
#define KEY1_PRES	2	//KEY1按下
#define KEY2_PRES	3	//KEY2按下
#define WKUP_PRES   4	//KEY_UP按下(即WK_UP/KEY_UP)


void delay_ms(u16 nms);
void EXTIX_Init(void);
void KEY_Init(void); 

//static u8  fac_us=0;							//us延时倍乘数			   
//static u16 fac_ms=0;							//ms延时倍乘数,在ucos下,代表每个节拍的ms数

/** @addtogroup StdPeriph_Examples
  * @{
  */

/** @addtogroup CAN_Normal
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
CAN_InitTypeDef        CAN_InitStructure;
CAN_FilterInitTypeDef  CAN_FilterInitStructure;
CanTxMsg TxMessage;
CanRxMsg RxMessage;

/* Private function prototypes -----------------------------------------------*/
static void RCC_Configuration(void);
static void GPIO_Configuration(void);
static void NVIC_Configuration(void);  
static void CAN_Config(void);
static void LED_Display(uint8_t Ledstatus);

extern void TIM4_start(void);
static int  test_master(void);

#define 	RX_BUF_LEN		128           //1024
Message   rx_msg_buf[RX_BUF_LEN];
uint32_t	rx_save, rx_read;

#define 	TX_BUF_LEN		128          //1024
uint32_t 	tx_save, tx_read;
CanTxMsg 	tx_msg_buf[TX_BUF_LEN];

//static rt_sem_t recv_sem = RT_NULL;
//static rt_sem_t tran_sem = RT_NULL;

/* Private functions ---------------------------------------------------------*/

void can_master_init(void)
{
  /* System clocks configuration ---------------------------------------------*/
  RCC_Configuration();

  /* NVIC configuration ------------------------------------------------------*/
  //NVIC_Configuration();

  /* GPIO configuration ------------------------------------------------------*/
  //GPIO_Configuration();

	//rx_save = 0;
	//rx_read = 0;

	//tx_save = 0;
	//tx_read = 0;

  /* CAN configuration */
  CAN_Config();

  //CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);

  /* turn off all leds*/
 // LED_Display(5);
}

/**
  * @brief  Main program
  * @param  None
  * @retval : None
  */
int cnnc = 0;
void can_recv_thread(void* parameter)
{
	//recv_sem = rt_sem_create("recvsem", 0, RT_IPC_FLAG_FIFO);
	//tran_sem = rt_sem_create("transem", 0, RT_IPC_FLAG_FIFO);
//  int i = 0;
  Message msg;
	can_master_init();

//	test_master();
	
  /* Infinite loop*/
//  while(1)
//  {	
//		//printf("hello world\r\n");
////		cnnc++;
////			if(cnnc== 500000)
////		{
//////			printf("***********************************************************\r\n");
////			msg.cob_id = (UNS16)0x602;
////      msg.len = (UNS8)0x08;
////      msg.rtr = (UNS8)0;
////      msg.data[0] = (UNS8)0x40;
////			msg.data[1] = (UNS8)0x0;
////			msg.data[2] = (UNS8)0x18;
////			msg.data[3] = (UNS8)0x0;
////			msg.data[4] = (UNS8)0x0;
////			msg.data[5] = (UNS8)0x0;
////			msg.data[6] = (UNS8)0x0;
////			msg.data[7] = (UNS8)0x0;		
//////      MSG_WAR(0x20000, "Producing heartbeat: ", 0x0);
////      canSend(&TestMaster_Data.canHandle,&msg );
////			cnnc = 0;
////		}
//		break;
//		//rt_sem_take(recv_sem, RT_WAITING_FOREVER);

//		/*{
//			uint32_t next;
//			Message *pmsg;
//			
//			next = rx_read;
//			pmsg = &rx_msg_buf[next];

//    	// Disable the Interrupt sources 
//    	TIM5->DIER &= (uint16_t)~TIM_IT_CC1;
//			canDispatch(&TestMaster_Data, pmsg);
//			// Enable the Interrupt sources
//    	TIM5->DIER |= TIM_IT_CC1;

//			next++;
//			if(next >= RX_BUF_LEN) next = 0;
//			rx_read = next;
//		}*/
//  }
}

void can_send_thread(void *parameter)
{
	while(1)
	{
		//rt_sem_take(tran_sem, RT_WAITING_FOREVER);zxb

		{
			uint32_t next;
			uint8_t  mailbox_no;
			CanTxMsg *ptx_msg;
			
			next = tx_read;
			ptx_msg = &tx_msg_buf[next];

			mailbox_no = CAN_Transmit(CAN1, ptx_msg);
			if(mailbox_no != CAN_NO_MB)
			{
				next++;
				if(next >= TX_BUF_LEN) next = 0;
				tx_read = next;			
			}
			else
			{
				//rt_sem_release(tran_sem);zxb
			} 
		}
	}		
}

/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval : None
  */
static void RCC_Configuration(void)
{
  /* Setup the microcontroller system. Initialize the Embedded Flash Interface,  
     initialize the PLL and update the SystemFrequency variable. */
  /* The System frequency should be set to HSE frequency */
  
  /* GPIO clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIO_CAN | RCC_APB2Periph_GPIO_Key , ENABLE);

  /* CAN1 Periph clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
}

/**
  * @brief  Configures the GPIO.
  * @param  None
  * @retval : None
  */
static void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

 
  /* Configure CAN pin: RX */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_RX;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIO_CAN, &GPIO_InitStructure);
  
  /* Configure CAN pin: TX */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIO_CAN, &GPIO_InitStructure);
  
  GPIO_PinRemapConfig(GPIO_Remap_CAN , ENABLE);

  /* (key button) as output push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 ;
  //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	
  GPIO_Init(GPIO_Key, &GPIO_InitStructure);
}

/**
  * @brief  Configures the NVIC for CAN and joystick
  * @param  None
  * @retval : None
  */
static void NVIC_Configuration(void)
{
  NVIC_InitTypeDef  NVIC_InitStructure;

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//NVIC_PriorityGroup_3
  
  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  Turn ON/OFF the dedicate led
  * @param Ledstatus: Led number from 0 to 3 
  * @retval : None
  */
//void LED_Display(uint8_t Ledstatus)
//{
//  /* turn off all leds*/
//  GPIO_WriteBit(GPIO_LED, GPIO_Pin_6, Bit_RESET);
//  GPIO_WriteBit(GPIO_LED, GPIO_Pin_7, Bit_RESET);
//  GPIO_WriteBit(GPIO_LED, GPIO_Pin_8, Bit_RESET);
//  GPIO_WriteBit(GPIO_LED, GPIO_Pin_9, Bit_RESET);
//  
//  switch(Ledstatus)
//  {
//   case(1): GPIO_WriteBit(GPIO_LED, GPIO_Pin_6, Bit_SET);
//            break;
//   
//   case(2): GPIO_WriteBit(GPIO_LED, GPIO_Pin_7, Bit_SET);
//            break;
// 
//   case(3): GPIO_WriteBit(GPIO_LED, GPIO_Pin_8, Bit_SET);
//            break;

//   case(4): GPIO_WriteBit(GPIO_LED, GPIO_Pin_9, Bit_SET);
//            break;
//  default:
//        break;
//   }
//}

/**
  * @brief  Configures the CAN.
  * @param  None
  * @retval : None
  */
static void CAN_Config(void)
{
  /* CAN register init */
  CAN_DeInit(CAN1);
	CAN_Mode_Init(CAN_SJW_1tq,CAN_BS2_1tq,CAN_BS1_10tq,12,CAN_Mode_Normal); 
	/*
  CAN_StructInit(&CAN_InitStructure); 
  CAN_InitStructure.CAN_TTCM = DISABLE;
  CAN_InitStructure.CAN_ABOM = DISABLE;
  CAN_InitStructure.CAN_AWUM = DISABLE;
  CAN_InitStructure.CAN_NART = DISABLE;
  CAN_InitStructure.CAN_RFLM = DISABLE;
  CAN_InitStructure.CAN_TXFP = DISABLE;
  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
  CAN_InitStructure.CAN_BS1 = CAN_BS1_12tq;
  CAN_InitStructure.CAN_BS2 = CAN_BS2_5tq;
  CAN_InitStructure.CAN_Prescaler = 2;
  CAN_Init(CAN1, &CAN_InitStructure);

  
  CAN_FilterInitStructure.CAN_FilterNumber = 0;
  CAN_FilterInitStructure.CAN_FilterMode   = CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale  = CAN_FilterScale_32bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterIdLow  = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow  = 0x0000;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);*/
  
   /* transmit */
  TxMessage.StdId = 0x321;
  TxMessage.ExtId = 0x00;
  TxMessage.RTR = CAN_RTR_DATA;
  TxMessage.IDE = CAN_ID_STD;
  TxMessage.DLC = 1;
}

#include <applicfg.h>
unsigned char canSend(CAN_PORT notused, Message *m)
{
	{
		uint32_t	i;
    CanTxMsg *ptx_msg;
			//ptx_msg = &tx_msg_buf[tx_save];
		  ptx_msg = (CanTxMsg *)malloc(sizeof(CanTxMsg));//zxb add this
			ptx_msg->StdId = m->cob_id;
	
			if(m->rtr)
	  		ptx_msg->RTR = CAN_RTR_REMOTE;
			else
				ptx_msg->RTR = CAN_RTR_DATA;

	  	ptx_msg->IDE = CAN_ID_STD;
  		ptx_msg->DLC = m->len;
			for(i = 0; i < m->len; i++)
				ptx_msg->Data[i] = m->data[i];
			MSG_WAR(0x00000,"ptx_msg->Data[0] is ", ptx_msg->Data[0]);
			MSG_WAR(0x00000,"ptx_msg->Data[1] is ", ptx_msg->Data[1]);
			MSG_WAR(0x00000,"ptx_msg->Data[2] is ", ptx_msg->Data[2]);
			MSG_WAR(0x00000,"ptx_msg->Data[3] is ", ptx_msg->Data[3]);
			MSG_WAR(0x00000,"ptx_msg->Data[4] is ", ptx_msg->Data[4]);
			MSG_WAR(0x00000,"ptx_msg->Data[5] is ", ptx_msg->Data[5]);
			MSG_WAR(0x00000,"ptx_msg->Data[6] is ", ptx_msg->Data[6]);
			MSG_WAR(0x00000,"ptx_msg->Data[7] is ", ptx_msg->Data[7]);
			 if( CAN_Transmit( CAN1, ptx_msg )==CAN_TxStatus_NoMailBox)
    { 
			//printf("1111111111\r\n");
         free(ptx_msg);
        return 0xff;
    }
    else
    {
			//printf("2222222\r\n");
         free(ptx_msg);
         return 0x00;
	  }
	
	}
}

unsigned char canReceive(Message *m) // 接收消息
/******************************************************************************
The driver pass a received CAN message to the stack
INPUT   Message *m pointer to received CAN message
OUTPUT  1 if a message received
******************************************************************************/
{
    u32 i = 0;
    CanRxMsg RxMessage;

    CAN_Receive( CAN1, CAN_FIFO0, &RxMessage );

    m->cob_id = (UNS16)RxMessage.StdId;        
    RxMessage.IDE   = CAN_ID_STD;      

    if( RxMessage.RTR == CAN_RTR_Remote )
    {
        m->rtr = 1; 
    }
    else
    {
        m->rtr = 0; 
    }

    m->len = RxMessage.DLC; 

    for( i=0; i<m->len; i++ )
    {
        m->data[i] = RxMessage.Data[i]; 
    }

    return TRUE; 
}

uint16_t InCnt =0;

void USB_LP_CAN1_RX0_IRQHandler(void)
{
	/*{
		Message *pmsg;
		uint32_t i, next;

		CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);

		next = (rx_save + 1) ;
		if(next >= RX_BUF_LEN) next = 0;
		if(next == rx_read) return;

		pmsg = &rx_msg_buf[rx_save];
		pmsg->cob_id = (UNS16)RxMessage.StdId;		//< message's ID 
		if(RxMessage.RTR == CAN_RTR_REMOTE)				//< remote transmission request. (0 if not rtr message, 1 if rtr message)
			pmsg->rtr = 1;		
		else
			pmsg->rtr = 0;

	  pmsg->len  = (UNS8)RxMessage.DLC;					//< message's length (0 to 8) 
		for(i = 0; i < pmsg->len; i++)
			pmsg->data[i] = RxMessage.Data[i];

		rx_save = next;
	//	rt_sem_release(recv_sem); zxb
	}*/
	__disable_irq();

    if( CAN_GetITStatus( CAN1, CAN_IT_FMP0 )== SET )
    {  
       
        CAN_ClearITPendingBit( CAN1, CAN_IT_FMP0 );

        while( CAN_MessagePending( CAN1, CAN_FIFO0 ) !=0 )
        {
            // a message was received pass it to the CANstack
           if(CanreveiveFlag==0)
           {               
                if( canReceive(&m) )
                {
									  //while(1) printf("is has receve the message........\r\n");
									  				
                    canDispatch(&TestMaster_Data, &m);//zxb
		//											GPIO_ResetBits(GPIOA,GPIO_Pin_5);
									  InCnt ++;
//									  CanreveiveFlag = 1;
                   // printf("%s(%d)TestSlave_Data->bDeviceNodeId is 0x[%x]\r\n ",__FILE__,__LINE__,*(TestMaster_Data.bDeviceNodeId));
                }
           }
        }   
    }

    if(CAN_GetITStatus( CAN1, CAN_IT_FF0) )
    {
        
        CAN_ClearITPendingBit(CAN1, CAN_IT_FF0 );
    }
    
    if(CAN_GetITStatus( CAN1, CAN_IT_FOV0) )
    {
        
        CAN_ClearITPendingBit(CAN1, CAN_IT_FOV0 );
    }
    __enable_irq();
}

static TimerCallback_t init_callback;

UNS32 OnMasterMap1Update(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex)
{
//	eprintf("OnMasterMap1Update:%d\n", MasterMap1);
	return 0;
}

s_BOARD MasterBoard = {"1", "1M"};

void help(void)
{
//  eprintf("**************************************************************\n");
//  eprintf("*  TestMasterSlave                                           *\n");
//  eprintf("*                                                            *\n");
//  eprintf("*  A simple example for PC. It does implement 2 CanOpen      *\n");
//  eprintf("*  nodes in the same process. A master and a slave. Both     *\n");
//  eprintf("*  communicate together, exchanging periodically NMT, SYNC,  *\n");
//  eprintf("*  SDO and PDO. Master configure heartbeat producer time     *\n");
//  eprintf("*  at 1000 ms for slave node-id 0x02 by concise DCF.         *\n");                                  
//  eprintf("*                                                            *\n");
//  eprintf("*   Usage:                                                   *\n");
//  eprintf("*   ./TestMasterSlave  [OPTIONS]                             *\n");
//  eprintf("*                                                            *\n");
//  eprintf("*   OPTIONS:                                                 *\n");
//  eprintf("*     -l : Can library [\"libcanfestival_can_virtual.so\"]     *\n");
//  eprintf("*                                                            *\n");
//  eprintf("*    Slave:                                                  *\n");
//  eprintf("*     -s : bus name [\"0\"]                                    *\n");
//  eprintf("*     -S : 1M,500K,250K,125K,100K,50K,20K,10K,none(disable)  *\n");
//  eprintf("*                                                            *\n");
//  eprintf("*    Master:                                                 *\n");
//  eprintf("*     -m : bus name [\"1\"]                                    *\n");
//  eprintf("*     -M : 1M,500K,250K,125K,100K,50K,20K,10K,none(disable)  *\n");
//  eprintf("*                                                            *\n");
//  eprintf("**************************************************************\n");
}

/***************************  INIT  *****************************************/
void InitNodes(CO_Data* d, UNS32 id)
{
	/****************************** INITIALISATION MASTER *******************************/
	if(strcmp(MasterBoard.baudrate, "none")){
 		RegisterSetODentryCallBack(&TestMaster_Data, 0x2000, 0, &OnMasterMap1Update);
		
		/* Defining the node Id */
		setNodeId(&TestMaster_Data, 0x01);

		/* init */
		setState(&TestMaster_Data, Initialisation);
	}
}

static StartTimerLoop(TimerCallback_t _init_callback) 
{
	init_callback = _init_callback;

	SetAlarm(NULL, 0, init_callback, 0, 0);
	TIM4_start();
}

/***************************  EXIT  *****************************************/
void Exit(CO_Data* d, UNS32 id)
{
	if(strcmp(MasterBoard.baudrate, "none")){
		
		masterSendNMTstateChange(&TestMaster_Data, 0x02, NMT_Reset_Node);    
    
   	//Stop master
		setState(&TestMaster_Data, Stopped);
	}
}

/****************************************************************************/
/****************************  test_master  *********************************/
/****************************************************************************/
static int test_master(void)
{
	if(strcmp(MasterBoard.baudrate, "none"))
	{
		TestMaster_Data.heartbeatError = TestMaster_heartbeatError;
		TestMaster_Data.initialisation = TestMaster_initialisation;
		TestMaster_Data.preOperational = TestMaster_preOperational;
		TestMaster_Data.operational = TestMaster_operational;
		TestMaster_Data.stopped = TestMaster_stopped;
		TestMaster_Data.post_sync = TestMaster_post_sync;
		TestMaster_Data.post_TPDO = TestMaster_post_TPDO;
		TestMaster_Data.post_emcy = TestMaster_post_emcy;
		TestMaster_Data.post_SlaveBootup=TestMaster_post_SlaveBootup;
	}

	// Start timer thread
	StartTimerLoop(&InitNodes);
	return 0;
}



/************************************************************************************/
/*************************************zxb add this***********************************/
/************************************************************************************/


void KEY_Init(void) //IO初始化
{ 
 	GPIO_InitTypeDef GPIO_InitStructure;
 
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOE,ENABLE);//使能PORTA,PORTE时钟

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4;//KEY0-KEY2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //设置成上拉输入
 	GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化GPIOE2,3,4

	//初始化 WK_UP-->GPIOA.0	  下拉输入
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //PA0设置成输入，默认下拉	  
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.0

}

void EXTIX_Init(void)
{
 
 	  EXTI_InitTypeDef EXTI_InitStructure;
 	  NVIC_InitTypeDef NVIC_InitStructure;
    //KEY_Init();	 //	按键端口初始化

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	//使能复用功能时钟

    //GPIOE.2 中断线以及中断初始化配置   下降沿触发
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource2);
  	EXTI_InitStructure.EXTI_Line=EXTI_Line2;	//KEY2
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);	 	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器

   //GPIOE.3	  中断线以及中断初始化配置 下降沿触发 //KEY1
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource3);
  	EXTI_InitStructure.EXTI_Line=EXTI_Line3;
  	EXTI_Init(&EXTI_InitStructure);	  	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器

   //GPIOE.4	  中断线以及中断初始化配置  下降沿触发	//KEY0
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource4);
  	EXTI_InitStructure.EXTI_Line=EXTI_Line4;
  	EXTI_Init(&EXTI_InitStructure);	  	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器


   //GPIOA.0	  中断线以及中断初始化配置 上升沿触发 PA0  WK_UP
 	    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource0); 
     	EXTI_InitStructure.EXTI_Line=EXTI_Line0;
    	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    	EXTI_Init(&EXTI_InitStructure);		//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器


  	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;			//使能按键WK_UP所在的外部中断通道
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2， 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;					//子优先级3
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
  	NVIC_Init(&NVIC_InitStructure); 

    NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;			//使能按键KEY2所在的外部中断通道
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2， 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;					//子优先级2
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
  	NVIC_Init(&NVIC_InitStructure);


  	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;			//使能按键KEY1所在的外部中断通道
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;					//子优先级1 
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
  	NVIC_Init(&NVIC_InitStructure);  	  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器

	  NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;			//使能按键KEY0所在的外部中断通道
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;					//子优先级0 
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
  	NVIC_Init(&NVIC_InitStructure);  	  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
}

//外部中断0服务程序 
/*void EXTI0_IRQHandler(void)
{
	//delay_ms(10);//消抖
	printf("aaaaaaaaaaaa\r\n");

	EXTI_ClearITPendingBit(EXTI_Line0); //清除LINE0上的中断标志位  
}
 
//外部中断2服务程序
void EXTI2_IRQHandler(void)
{
	//delay_ms(10);//消抖
	if(KEY2==0)	  //按键KEY2
	{
		//LED0=!LED0;
		printf("key2 is pushed....\r\n");
	}		 
	printf("key2 is pushed \r\n");		
	EXTI_ClearITPendingBit(EXTI_Line2);  //清除LINE2上的中断标志位  
}
//外部中断3服务程序
void EXTI3_IRQHandler(void)
{
	//delay_ms(10);//消抖
	if(KEY1==0)	 //按键KEY1
	{		
    printf("key1 is pushed....\r\n");		
		//LED1=!LED1;
	}		 
	 printf("key1 is pushed\r\n");		
	EXTI_ClearITPendingBit(EXTI_Line3);  //清除LINE3上的中断标志位  
}


void EXTI4_IRQHandler(void)
{
	delay_ms(10);//消抖
	if(KEY0==0)	 //按键KEY0
	{
		printf("key0 is pushed....\r\n");
		//LED0=!LED0;
		//LED1=!LED1; 
	}		 
	printf("key0 is pushed\r\n");
	EXTI_ClearITPendingBit(EXTI_Line4);  //清除LINE4上的中断标志位  
}
 
void delay_ms(u16 nms)
{	 		  	  
	u32 temp;		   
	SysTick->LOAD=(u32)nms*fac_ms;				//时间加载(SysTick->LOAD为24bit)
	SysTick->VAL =0x00;							//清空计数器
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;	//开始倒数  
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));		//等待时间到达   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//关闭计数器
	SysTick->VAL =0X00;       					//清空计数器	  	    
} 
*/

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
