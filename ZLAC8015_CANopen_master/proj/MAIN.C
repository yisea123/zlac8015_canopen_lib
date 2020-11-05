#include"main.h"


void Key_Init(void);
void WriteSpeed(int32_t leftspeed,int32_t rightspeed);
void WriteSpeed1(int16_t leftspeed,int16_t rightspeed);
void ReadSpeed(void);
extern void can_recv_thread(void);
extern unsigned char CanreveiveFlag;
extern Message m;

int cnt = 0;
int cob_id_index = 3,sendst=1;
int start = 7,cob_idbak =0,cob_id_last = 0;
/**************************************************************************

**************************************************************************/

uint16_t KeyState = 0,KeyStateCnt = 0,motion = 0,KeyStateOff = 0,KeyStateOffCnt = 0;
int32_t leftspeedwr = 0,rightspeedwr = 0,leftspeedrd = 10,rightspeedrd = -10,delaycnt = 0,delaycntMax = 500000,LEDcnt = 0;
int16_t leftspeedwr16 = 0,rightspeedwr16 = 0,leftspeedrd16 = 10,rightspeedrd16 = -10;
void delay_ns(__IO uint32_t nCount)
{
		for(; nCount != 0; nCount--);
}

static void RCC_Config(void)
{
		ErrorStatus HSEStartUpStatus;
		//RCC reset
		RCC_DeInit();
		//开启外部时钟 并执行初始化
		RCC_HSEConfig(RCC_HSE_ON);
		//等待外部时钟准备好
		HSEStartUpStatus = RCC_WaitForHSEStartUp();
		//启动失败 在这里等待
		while(HSEStartUpStatus == ERROR);
		//设置内部总线时钟
		RCC_HCLKConfig(RCC_SYSCLK_Div1);
		RCC_PCLK1Config(RCC_HCLK_Div1);
		RCC_PCLK2Config(RCC_HCLK_Div1);
		//外部时钟为8M 这里倍频到72M
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
		RCC_PLLCmd(ENABLE);
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		while(RCC_GetSYSCLKSource() != 0x08);
		//关闭内部时钟HSI
		RCC_HSICmd(DISABLE);
}
Message msgtemp;
int main(void)
{
	
	SystemInit(); //系统初始化
	RCC_Config();
	delay_init();

	Key_Init();
	KeyState = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12);

	delay_ns(8000000);		
	delay_ns(8000000);
	delay_ns(8000000);
	delay_ns(8000000);
	delay_ns(8000000);
	delay_ns(8000000);		
	delay_ns(8000000);
	delay_ns(8000000);
	delay_ns(8000000);
	delay_ns(8000000);
	can_recv_thread();
	
	
  if(start == 7){

			start = 0;
        cob_id_index   = 4;
				msgtemp.cob_id = (UNS16)0x600 + cob_id_index;                            //使能
				msgtemp.len = (UNS8)0x08;
				msgtemp.rtr = (UNS8)0;
				msgtemp.data[0] = (UNS8)0x2B;
				msgtemp.data[1] = (UNS8)0x3e;
				msgtemp.data[2] = (UNS8)0x68;
				msgtemp.data[3] = (UNS8)0x0;
				msgtemp.data[4] = (UNS8)0x01;
				msgtemp.data[5] = (UNS8)0x0;
				msgtemp.data[6] = (UNS8)0x0;
				msgtemp.data[7] = (UNS8)0x0;		
				canSend(&TestMaster_Data.canHandle,&msgtemp );

				msgtemp.cob_id = (UNS16)0x600 + cob_id_index;                            //使能
				msgtemp.len = (UNS8)0x08;
				msgtemp.rtr = (UNS8)0;
				msgtemp.data[0] = (UNS8)0x2B;
				msgtemp.data[1] = (UNS8)0x60;
				msgtemp.data[2] = (UNS8)0x68;
				msgtemp.data[3] = (UNS8)0x0;
				msgtemp.data[4] = (UNS8)0x03;
				msgtemp.data[5] = (UNS8)0x03;
				msgtemp.data[6] = (UNS8)0x0;
				msgtemp.data[7] = (UNS8)0x0;		
				canSend(&TestMaster_Data.canHandle,&msgtemp );
				
				msgtemp.cob_id = (UNS16)0x600 + cob_id_index;                            //启动
				msgtemp.len = (UNS8)0x08;
				msgtemp.rtr = (UNS8)0;
				msgtemp.data[0] = (UNS8)0x23;
				msgtemp.data[1] = (UNS8)0x40;
				msgtemp.data[2] = (UNS8)0x68;
				msgtemp.data[3] = (UNS8)0x0;
				msgtemp.data[4] = (UNS8)0x06;
				msgtemp.data[5] = (UNS8)0x0;
				msgtemp.data[6] = (UNS8)0x06;
				msgtemp.data[7] = (UNS8)0x0;		
				canSend(&TestMaster_Data.canHandle,&msgtemp );

				msgtemp.cob_id = (UNS16)0x600 + cob_id_index;                            //启动
				msgtemp.len = (UNS8)0x08;
				msgtemp.rtr = (UNS8)0;
				msgtemp.data[0] = (UNS8)0x23;
				msgtemp.data[1] = (UNS8)0x40;
				msgtemp.data[2] = (UNS8)0x68;
				msgtemp.data[3] = (UNS8)0x0;
				msgtemp.data[4] = (UNS8)0x07;
				msgtemp.data[5] = (UNS8)0x0;
				msgtemp.data[6] = (UNS8)0x07;
				msgtemp.data[7] = (UNS8)0x0;		
				canSend(&TestMaster_Data.canHandle,&msgtemp );
				
				msgtemp.cob_id = (UNS16)0x600 + cob_id_index;                            //启动
				msgtemp.len = (UNS8)0x08;
				msgtemp.rtr = (UNS8)0;
				msgtemp.data[0] = (UNS8)0x23;
				msgtemp.data[1] = (UNS8)0x40;
				msgtemp.data[2] = (UNS8)0x68;
				msgtemp.data[3] = (UNS8)0x0;
				msgtemp.data[4] = (UNS8)0x0F;
				msgtemp.data[5] = (UNS8)0x0;
				msgtemp.data[6] = (UNS8)0x0F;
				msgtemp.data[7] = (UNS8)0x0;		
				canSend(&TestMaster_Data.canHandle,&msgtemp );
				
				msgtemp.cob_id = (UNS16)0x600 + cob_id_index;                            //启动
				msgtemp.len = (UNS8)0x08;
				msgtemp.rtr = (UNS8)0;
				msgtemp.data[0] = (UNS8)0x23;
				msgtemp.data[1] = (UNS8)0xFF;
				msgtemp.data[2] = (UNS8)0x68;
				msgtemp.data[3] = (UNS8)0x0;
				msgtemp.data[4] = (UNS8)0x00;
				msgtemp.data[5] = (UNS8)0x0;
				msgtemp.data[6] = (UNS8)0x00;
				msgtemp.data[7] = (UNS8)0x0;		
				canSend(&TestMaster_Data.canHandle,&msgtemp );				

		}
	  while(1)
		{
			KeyState = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12);
			
			if(!KeyState){
				 
           KeyStateOffCnt = 0;
					 KeyStateCnt ++;
					 if(KeyStateCnt>=2){
							KeyStateCnt = 2;
							start = 1;
						  KeyStateOff = 0;
					 }

				   
			}else {
				// KeyStateOff = 0;
				 KeyStateCnt = 0;
				start =0;
			   KeyStateOffCnt ++;
				 if(KeyStateOffCnt>5){
		//		   KeyStateOff = 1;
				   KeyStateOffCnt = 5;
				 }
				 
			}
				
			cnt++;
//			eMBPoll();
			if(cnt== 1)
			{
				
				if(start == 1){
          motion = 1;
          delaycnt ++;

					if(delaycnt>delaycntMax){
						  delaycnt = 0;
             
              WriteSpeed1(leftspeedwr16,rightspeedwr16);
 							leftspeedwr16 = leftspeedwr16 + leftspeedrd16;
							rightspeedwr16 = rightspeedwr16 + rightspeedrd16;
						  if(leftspeedwr16>100){
							   leftspeedwr16 = 0;
								 leftspeedrd16 = -20;
							
							}else if(leftspeedwr16<-100){
							   leftspeedwr16 = 0;
								 leftspeedrd16 = 20;							
							
							}
						  if(rightspeedwr16<-100){
							   rightspeedwr16 = 0;
								 rightspeedrd16 = 20;
							
							}else if(rightspeedwr16>100){
							   rightspeedwr16 = 0;
								 rightspeedrd16 = -20;							
							
							}
//              if(leftspeedwr16 == 0 && rightspeedwr16 == 0)
//                  delaycntMax = 80000;
//              else
              		delaycntMax = 500000;	             

					}
					LEDcnt++;
					if(LEDcnt<=80000)
							GPIO_ResetBits(GPIOA,GPIO_Pin_5);
					else if(LEDcnt>80000&&LEDcnt<=160000)
						GPIO_SetBits(GPIOA,GPIO_Pin_5);
					else
						 LEDcnt = 0;
							
			  }
				else if(start == 0){
						LEDcnt++;
						if(LEDcnt<=1000)
								GPIO_ResetBits(GPIOA,GPIO_Pin_5);
						else if(LEDcnt>1000&&LEDcnt<=2000)
							GPIO_SetBits(GPIOA,GPIO_Pin_5);
						else
							 LEDcnt = 0;
					  motion = 0;
					  start = 0;
					  leftspeedwr16 = 0;
					  rightspeedwr16 = 0;
					  delaycntMax = 80000;
					  delaycnt = 0;
						KeyStateOff ++;
						if(KeyStateOff>1000){
							  KeyStateOff = 0;
								cob_id_index = 4;
								WriteSpeed1(leftspeedwr16,rightspeedwr16);	
					  }							
				
				}
				cnt = 0;	
       }
		}


}//end main
void WriteSpeed(int32_t leftspeed,int32_t rightspeed){
            cob_id_index = 4;
						msgtemp.cob_id = (UNS16)0x600 + cob_id_index;                            //启动
						msgtemp.len = (UNS8)0x08;
						msgtemp.rtr = (UNS8)0;
						msgtemp.data[0] = (UNS8)0x23;
						msgtemp.data[1] = (UNS8)0xFF;
						msgtemp.data[2] = (UNS8)0x60;
						msgtemp.data[3] = (UNS8)0x0;
						msgtemp.data[4] = (UNS8)leftspeed;
						msgtemp.data[5] = (UNS8)(leftspeed>>8);
						msgtemp.data[6] = (UNS8)(leftspeed>>16);
						msgtemp.data[7] = (UNS8)(leftspeed>>24);		
						canSend(&TestMaster_Data.canHandle,&msgtemp );
					
            cob_id_index = 5;
						msgtemp.cob_id = (UNS16)0x600 + cob_id_index;                            //启动
						msgtemp.len = (UNS8)0x08;
						msgtemp.rtr = (UNS8)0;
						msgtemp.data[0] = (UNS8)0x23;
						msgtemp.data[1] = (UNS8)0xFF;
						msgtemp.data[2] = (UNS8)0x60;
						msgtemp.data[3] = (UNS8)0x0;
						msgtemp.data[4] = (UNS8)rightspeed;
						msgtemp.data[5] = (UNS8)(rightspeed>>8);
						msgtemp.data[6] = (UNS8)(rightspeed>>16);
						msgtemp.data[7] = (UNS8)(rightspeed>>24);		
						canSend(&TestMaster_Data.canHandle,&msgtemp );

}
void WriteSpeed1(int16_t leftspeed,int16_t rightspeed){
            cob_id_index = 4;
						msgtemp.cob_id = (UNS16)0x600 + cob_id_index;                            //启动
						msgtemp.len = (UNS8)0x08;
						msgtemp.rtr = (UNS8)0;
						msgtemp.data[0] = (UNS8)0x23;
						msgtemp.data[1] = (UNS8)0xFF;
						msgtemp.data[2] = (UNS8)0x68;
						msgtemp.data[3] = (UNS8)0x0;
						msgtemp.data[4] = (UNS8)leftspeed;
						msgtemp.data[5] = (UNS8)(leftspeed>>8);
						msgtemp.data[6] = (UNS8)(rightspeed);
						msgtemp.data[7] = (UNS8)(rightspeed>>8);		
						canSend(&TestMaster_Data.canHandle,&msgtemp );

}
void ReadSpeed(void){
//				  GPIO_SetBits(GPIOA,GPIO_Pin_5);
			cob_id_index = 4;
			CanreveiveFlag = 0;
			msgtemp.cob_id = (UNS16)0x600 + cob_id_index;                            //启动
			msgtemp.len = (UNS8)0x08;
			msgtemp.rtr = (UNS8)0;
			msgtemp.data[0] = (UNS8)0x40;
			msgtemp.data[1] = (UNS8)0x6C;
			msgtemp.data[2] = (UNS8)0x60;
			msgtemp.data[3] = (UNS8)0x0;
			msgtemp.data[4] = (UNS8)0x0;
			msgtemp.data[5] = (UNS8)0x0;
			msgtemp.data[6] = (UNS8)0x0;
			msgtemp.data[7] = (UNS8)0x0;		
			canSend(&TestMaster_Data.canHandle,&msgtemp );

			cob_id_index = 5;
			CanreveiveFlag = 0;
			msgtemp.cob_id = (UNS16)0x600 + cob_id_index;                            //启动
			msgtemp.len = (UNS8)0x08;
			msgtemp.rtr = (UNS8)0;
			msgtemp.data[0] = (UNS8)0x40;
			msgtemp.data[1] = (UNS8)0x6C;
			msgtemp.data[2] = (UNS8)0x60;
			msgtemp.data[3] = (UNS8)0x0;
			msgtemp.data[4] = (UNS8)0x0;
			msgtemp.data[5] = (UNS8)0x0;
			msgtemp.data[6] = (UNS8)0x0;
			msgtemp.data[7] = (UNS8)0x0;		
			canSend(&TestMaster_Data.canHandle,&msgtemp );

}
void Key_Init(void)
{
 
 GPIO_InitTypeDef  GPIO_InitStructure;
 	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE);	 //使能PB,PE端口时钟


 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;	    		 //LED1-->PE.5 端口配置, 推挽输出
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		 //推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
 GPIO_Init(GPIOB, &GPIO_InitStructure);	  				 //推挽输出 ，IO口速度为50MHz
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;	    		 //LED1-->PE.5 端口配置, 推挽输出
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
 GPIO_Init(GPIOA, &GPIO_InitStructure);	  				 //推挽输出 ，IO口速度为50MHz	
	GPIO_SetBits(GPIOA,GPIO_Pin_5);
}
