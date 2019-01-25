#include "stm32f4xx.h"
#include "GLCD.h"

#define SW0_PUSH        0xFE00  //PH8
#define SW1_PUSH        0xFD00  //PH9
#define SW2_PUSH        0xFB00  //PH10
#define SW3_PUSH        0xF700  //PH11
#define SW4_PUSH        0xEF00  //PH12
#define SW5_PUSH        0xDF00  //PH13
#define SW6_PUSH        0xBF00  //PH14
#define SW7_PUSH        0x7F00  //PH15

#define N_Left               0x0DF0 //PI9
#define N_Right             0x0EF0 //PI8

void _RCC_Init(void);
void _GPIO_Init(void);
void DisplayTitle(void);

void Step_MOTOR_driver(void);
void USART1_Init(void); //����Ʈ1 ���
void USART_BRR_Configuration(uint32_t USART_BaudRate); //����Ʈ ��� �Լ�
void SerialPutChar(uint8_t c); //�ĸ����� �͹̳� ���� �Լ�
void Serial_PutString(char *s); //�ĸ����� �͹̳� ���ڿ��Լ�
void _EXTI_Init(void);
void current_pulse(); //���� �޽��� ��Ÿ���� �Լ�
void goal_pulse(); //��ǥ �޽��� �����ϴ� �Լ�
void goal_speed(); //��ǥ �ӵ��� �����ϴ� �Լ�

uint16_t Navi_KEY_Scan(void);  //Joystick key scan �Լ�
uint16_t KEY_Scan(void);


void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);
void BEEP(void);

short Usart_Flag = 0;
short SW4_Flag = 0;

int X[3]; //��ǥ  ������ ��� ����

uint8_t T = 0; //��ǥ �ð�
int Current_P = 0; //���� �޽�
int Goal_Speed = 0; //��ǥ �ӵ�
int Goal_Pulse = 0; // ��ǥ �޽�

char ch[20]; //2�ڸ� ������ �ޱ����� ����

char str5[20]; //sw5 100���ڸ� ���� �ޱ�����
char str6[20]; //sw6 10���ڸ� ���� �ޱ�����
char str7[20];//sw7 1���ڸ� ���� �ޱ�����

char str_L[20]; //Navi Left�� ���� �ޱ�����
char str_R[20]; //Navi Right���� �ޱ�����

char str[20];

int cnt = 0; //����Ʈ�� DR�� ������� �迭�� ũ�� ����
short stop = 0;

void USART1_IRQHandler(void)  //Usart1 Handler
{
    if ( (USART1->SR & USART_SR_RXNE) && (Usart_Flag == 0))        // DR �� ������ SR.RXNE bit(flag bit)�� clear �ȴ� 
    {
      ch[cnt] = (uint16_t)(USART1->DR & (uint16_t)0x01FF);  //���ͽ��� ����;
          Usart_Flag = 1; //�÷��� ���� 1�� ->����Ʈ ���ι� ���� ����
    }
}

void TIM2_IRQHandler(void)  // 1s Interrupt
{
  if((TIM2->SR & (1<<4)) != RESET) //Capture/compare 4 nterrupt flag
  {
    TIM2->SR &= ~(1<<4); // Interrupt flag Clear (ch4)
  }
  if((TIM2->SR & (1<<0)) != RESET) // Interrupt flag
  {
    if((SW4_Flag == 1) && (stop == 0))
    {
      current_pulse(); //�޽��� ��Ÿ����.->�� �Լ��� ������ SW4_Flag�� 0, stop = 1�� �ȴ�.
      //GPIOG->ODR |= 0x0002; //LED4
    }
    if((stop == 1) && (SW4_Flag == 0))
    {
      //��ǥ �޽��� �����ϸ� �����.
      stop = 0;
    }
    //current_pulse();
    TIM2->SR &= ~(1<<0); // Interrupt flag Clear
  }
}
  
void EXTI15_10_IRQHandler(void)		//��ž��ġ ��ɵ� // EXTI 15~10 ���ͷ�Ʈ �ڵ鷯
{
      if(EXTI->PR & 0x1000) // Pending bit Clear SW4
      {
        EXTI->PR |= 0x1000; //pending bit clear
        GPIOG->ODR |= 0x0010; //LED4
        SW4_Flag = 1;
        
        TIM2->CR1	|= (1<<0);	// CEN: Counter TIM2 enable
      }
      
      if((EXTI->PR & 0x2000) && (stop == 0)) // Pending bit Clear SW5
      {
        EXTI->PR |= 0x2000; //pending bit clear
        //GPIOG->ODR |= 0x0020; // LED5
        
          X[2]++; //100�� �ڸ�
          if(X[2] == 10) //10�� �Ǹ�
          {
            X[2] = 0; //0����
          }
          sprintf(str5, "%1d", X[2]); 
          LCD_SetTextColor(RGB_BLUE);//�Ķ���
          LCD_DisplayText(2,16, str5 );
      }
      
      if((EXTI->PR & 0x4000) && (stop == 0)) // Pending bit Clear SW6
      {
        EXTI->PR |= 0x4000; //pending bit clear
       // GPIOG->ODR |= 0x0040;
          X[1]++; //10�� �ڸ�
          if(X[1] == 10)
          {
            X[1] = 0;
          }
          sprintf(str6, "%1d", X[1]);
          LCD_SetTextColor(RGB_BLUE);//�Ķ���
          LCD_DisplayText(2,17, str6 );
        //BEEP();
      }
      
      if((EXTI->PR & 0x8000) && (stop == 0)) // Pending bit Clear SW7
      {
        EXTI->PR |= 0x8000; //pending bit clear
        //GPIOG->ODR |= 0x0080;
          X[0]++; //1�� �ڸ�
          if(X[0] == 10)
          {
            X[0] = 0;
          }
          sprintf(str7, "%1d", X[0]);
          LCD_SetTextColor(RGB_BLUE);//�Ķ���
          LCD_DisplayText(2,18, str7 );
      }
}
  
int main(void)
{
	_RCC_Init();
	LCD_Init();			// LCD ���� �Լ�
	DelayMS(10);			// LCD���� ������    
	DisplayTitle();		//LCD �ʱ�ȭ�鱸�� �Լ�	
	_GPIO_Init();
       _EXTI_Init();
	Step_MOTOR_driver(); // TIM2 �����Լ�
       
        USART1_Init(); //����Ʈ ���� �Լ�
	
        while(1)
	{
          if((Usart_Flag == 1) && (stop == 0)) //����Ʈ �ڵ鷰�� ���� ���� ��
          {
              if(ch[cnt]) //2�ڸ� ������ ���� ���
              {
                SerialPutChar(ch[cnt]); //�迭���� �ĸ����Ϳ� ���÷��� ���ش�.
                LCD_SetTextColor(RGB_BLUE);
                LCD_DisplayChar(1,15+cnt, ch[cnt]);

                cnt ++; //cnt�� ���� ����
                if(cnt == 2) //2�� �Ǹ� 
                {
                  cnt = 0; //0���� �ʱ�ȭ
                }
                Usart_Flag = 0; //Usart flag�� 0���� -> ����Ʈ �ڵ鷯�� ���� �ش�.
              }
          }
///////////////////////////////////////////////////////////////////////////          
          // if(stop == 0)
          // {
          //   switch(KEY_Scan())                                          
          //   {
          //   case SW0_PUSH:
          //     T++;
          //       if(T == 100)
          //       {
          //         T = 0;
          //       }
          //       sprintf(str, "%02d", T);
          //       LCD_SetTextColor(RGB_BLUE);
          //       LCD_DisplayText(3,14, str);
          //       break;
          //   case SW1_PUSH:
          //     T--;
          //     if(T == 0)
          //     {
          //       T =99;
          //     }
          //     if(T == 255)
          //       {
          //         T = 99;
          //       }
          //     sprintf(str, "%02d", T);
          //     LCD_SetTextColor(RGB_BLUE);
          //       LCD_DisplayText(3,14, str);
          //     break; 
          //   }
          // }
///////////////////////////////////////////////////////////////////////////////         
          if(stop == 0)
          {
            switch(Navi_KEY_Scan()) //Joystick Key Scan ����
            {
                case N_Left :
                T++;
                  if(T == 100)
                  {
                    T = 0;
                  }
                sprintf(str_L, "%02d", T);
                LCD_SetTextColor(RGB_BLUE);
                LCD_DisplayText(3,14, str_L);
                break;
                
              case N_Right :
              T--;
                if((T == 0) || (T == 255)) //���� T �� unsigned int 8��Ʈ �� �̹Ƿ� 
                //ó���� -1�� ��Ű�� 255�ΰ���. ���� �׶� T�� 99�� ������ش�.
                {
                  T =99;
                }
              sprintf(str_R, "%02d", T);
              LCD_SetTextColor(RGB_BLUE);
              LCD_DisplayText(3,14, str_R);
              break;
            }
          }
          goal_pulse(); // ��ǥ �޽� �Լ� ����
          goal_speed(); //��ǥ ���ǵ� �Լ� ����
	}
        
}

void _EXTI_Init(void)    //EXTI(12 ~ 15) (PH(12 ~ 15), SW(4 ~ 7))
{
    RCC->AHB1ENR    |= 0x80;	// RCC_AHB1ENR GPIOH Enable
    RCC->APB2ENR 	|= 0x4000;	// Enable System Configuration Controller Clock

    GPIOH->MODER 	&=  0x00FF;	// GPIOH PIN8~PIN15 Input mode (reset state)				 

    SYSCFG->EXTICR[3] |= 0x7777;    // EXTI12 ~ 15 �� ���� �ҽ� �Է��� GPIOH�� ���� (EXTICR4) (reset value: 0x0000)
    EXTI->FTSR |= 0x0000F100;		// Falling Trigger Enable  (EXTI8, 12 ~ 15 :PH8, 12 ~ 15)
    EXTI->IMR |= 0x0000F100;  	// EXTI11 ���ͷ�Ʈ mask (Interrupt Enable)

    NVIC->ISER[1] |= ( 1 << (40-32) ); // Enable Interrupt EXTI12 ~ 15 Vector table Position ����
}

void _GPIO_Init(void)
{
    // LED GPIO(PORT G) ����
    RCC->AHB1ENR    |= 0x00000040; 	// RCC_AHB1ENR(bit8~0) GPIOG(bit#6) Enable							
    GPIOG->MODER    = 0x00005555;	// GPIOG PIN0~PIN7 Output mode (0b01)						
    GPIOG->OTYPER   = 0x0000;	// GPIOG PIN0~PIN7 : Push-pull  (PIN8~PIN15) (reset state)	
    GPIOG->OSPEEDR  = 0x00005555;	// GPIOG PIN0~PIN7 Output speed (25MHZ Medium speed) 

    // SW GPIO(PORT H) ���� 
    RCC->AHB1ENR    |= 0x00000080;	// RCC_AHB1ENR(bit8~0) GPIOH(bit#7) Enable							
    GPIOH->MODER    = 0x00000000;	// GPIOH PIN8~PIN15 Input mode (reset state)				
    GPIOH->PUPDR    = 0x00000000;	// GPIOH PIN8~PIN15 Floating input (No Pull-up, pull-down) (reset state)

    // Buzzer GPIO(PORT F) ���� 
    RCC->AHB1ENR    |= 0x00000020; 	// RCC_AHB1ENR(bit8~0) GPIOF(bit#5) Enable							
    GPIOF->MODER    |= 0x00040000;	// GPIOF PIN9 Output mode (0b01)						
    GPIOF->OTYPER   &= 0xFDFF;	// GPIOF PIN9 : Push-pull  	
    GPIOF->OSPEEDR  |= 0x00040000;	// GPIOF PIN9 Output speed (25MHZ Medium speed) 

    //NAVI.SW(PORT I) ����
    RCC->AHB1ENR 	|= 0x00000100;	// RCC_AHB1ENR GPIOI Enable
    GPIOI->MODER 	= 0x00000000;	// GPIOI PIN0~PIN15 Input mode (reset state)
    GPIOI->PUPDR    = 0x11111111;	// GPIOI PIN0~PIN15 Floating input (No Pull-up, pull-down) (reset state)
}	

void Step_MOTOR_driver(void) //TIM2 CH4 -> LED PULSE
{   
        // Clock Enable : GPIOB & TIMER2
	RCC->AHB1ENR	|= 1<<1;	// RCC_AHB1ENR GPIOB Enable
	RCC->APB1ENR 	|= 1<<0;	// RCC_APB1ENR TIMER2 Enable 
    	
  NVIC->ISER[0] |= ( 1 << 28); // Enable Timer2 global Interrupt					
        
        // PB11�� ��¼����ϰ� Alternate function(TIM2_CH4)���� ��� ����
	GPIOB->MODER 	|= 2<<22;	// GPIOB PIN11 Output Alternate function mode					
	GPIOB->OSPEEDR 	|= 3<<22;	// GPIOB PIN11 Output speed (100MHz High speed)
	GPIOB->OTYPER	= 0x00000000;	// GPIOB PIN11 Output type push-pull (reset state)
	GPIOB->PUPDR	|= 1<<22;	// GPIOB PIN11 Pull-up
 	GPIOB->AFR[1]	|= 0x01<<12;	// AFRH(AFR[1]): Connect PB11 to AF1(TIM2,TIM1)
					// PB11 ==> TIM2_CH4
    
        // TIM2 Channel 4 
	TIM2->PSC	= 8400-1;	// Prescaler 84,000,000Hz/8400 = 10,000 Hz(0.1ms)  (1~65536)
	//TIM2->ARR	= 1000-1;	// Auto reload  (0.1ms * 1000= 100ms : PWM Period)
       // Duty Ratio 
	TIM2->CCR4	= 0;		// CCR4 value
	// CR1 : Up counting
	TIM2->CR1 	&= ~(1<<4);	// DIR: Countermode = Upcounter (reset state)
	TIM2->CR1 	&= ~(3<<8);	// CKD: Clock division = 1 (reset state)
	TIM2->CR1 	&= ~(3<<5); 	// CMS(Center-aligned mode Sel): No(reset state)
    	
  TIM2->DIER |= 1<<0; //UIE: Update interrupt enable //���ͷ�Ʈ �߻�
  TIM2->EGR |= 1<<0;//UG: update genertaion 
  
  TIM2->DIER |= 1<<4; //CC4IE: interrupt enable //CC4 ���ͷ�Ʈ �߻�
  TIM2->EGR |= 1<<4; //CC4G: (generation)  //CC4 event �߻�
	
        // Define the corresponding pin by 'Output'  
	TIM2->CCER	|= (1<<12);	// CC4E: OC4 Active(Capture/Compare 3 output enable)
	TIM2->CCER	&= ~(1<<13);	// CC4P: Capture/Compare 1 output Polarity High

	// 'Mode' Selection : Output/Compare Mode,
	TIM2->CCMR2 	&= ~(3<<8); 	// CC4S(CC3 channel): Output 
	TIM2->CCMR2 	&= ~(1<<11); 	// OC4PE: Output Compare 4 preload disable
	//TIM2->CCMR2	|= (3<<12);	// OC4M: Output compare 4 mode: toggle mode
	TIM2->CCMR2	|= (1<<15);	// OC4CE: Output compare 4 Clear enable
	TIM2->CCMR2	|= (6<<12); //pwm mode 1
       //TIM2->CCMR2	|= (7<<15); //pwm mode 2
	
        //Counter TIM4 enable
	TIM2->CR1	&= ~(1<<7);	// ARPE: Auto-reload preload Disnable
	//TIM2->CR1	|= (1<<0);	// CEN: Counter TIM2 enable
}

void USART1_Init(void)
{
    // USART1 : TX(PA9)
    RCC->AHB1ENR	|= 0x01;	// RCC_AHB1ENR GPIOA Enable
    GPIOA->MODER	|= 0x00080000;	// GPIOB PIN9 Output Alternate function mode					
    GPIOA->OSPEEDR	|= 0x000C0000;	// GPIOB PIN9 Output speed (100MHz Very High speed)
    GPIOA->OTYPER	|= 0x00000000;	// GPIOB PIN9 Output type push-pull (reset state)
    GPIOA->PUPDR	|= 0x00040000;	// GPIOB PIN9 Pull-up
    /**/GPIOA->AFR[1]	|= 0x70;	// Connect GPIOA pin9 to AF7(USART1)

    // USART1 : RX(PA10)
    GPIOA->MODER 	|= 0x200000;	// GPIOA PIN10 Output Alternate function mode
    GPIOA->OSPEEDR	|= 0x00300000;	// GPIOA PIN10 Output speed (100MHz Very High speed
    /**/GPIOA->AFR[1]	|= 0x700;	//Connect GPIOA pin10 to AF7(USART1)

    /**/RCC->APB2ENR	|= 0x0010;	// RCC_APB2ENR USART1 Enable

    /**/USART_BRR_Configuration(9600); // USART Baud rate Configuration

    /**/USART1->CR1	&= ~USART_CR1_M;	// USART_WordLength 8 Data bit
    /**/USART1->CR1	&= ~USART_CR1_PCE ;	// USART_Parity_none
    USART1->CR1	|= USART_CR1_RE;	// 0x0004, USART_Mode_RX Enable
    USART1->CR1	|= USART_CR1_TE ;	// 0x0008, USART_Mode_Tx Enable
    USART1->CR2	&= ~USART_CR2_STOP;	// USART_StopBits_1
    USART1->CR3	= 0x0000;	// No HardwareFlowControl, No DMA
    //cr3�� �÷ο� ��Ʈ�� �κ�

    USART1->CR1 	|= USART_CR1_RXNEIE;	//  0x0020, RXNE interrupt Enable /*****(�ڿ� IE�� ����ŷ ��Ʈ��)****/
    NVIC->ISER[1]	|= (1 << 5); 	// Enable Interrupt USART1 (NVIC 37��)
    USART1->CR1 	|= USART_CR1_UE;	//  0x2000, USART1 Enable
}

// Baud rate  
void USART_BRR_Configuration(uint32_t USART_BaudRate)
{ 
	uint32_t tmpreg = 0x00;
	uint32_t APB2clock = 84000000;	//PCLK2_Frequency
	uint32_t integerdivider = 0x00;
	uint32_t fractionaldivider = 0x00;

	// Determine the integer part 
	if ((USART1->CR1 & USART_CR1_OVER8) != 0) // USART_CR1_OVER8 : 0x8000
        {                                                                  // USART1->CR1.OVER8 = 1 (8 oversampling)
		// Integer part computing in case Oversampling mode is 8 Samples 
		integerdivider = ((25 * APB2clock) / (2 * USART_BaudRate));    
	}
	else  // USART1->CR1.OVER8 = 0 (16 oversampling)
	{	// Integer part computing in case Oversampling mode is 16 Samples 
		integerdivider = ((25 * APB2clock) / (4 * USART_BaudRate));    
	}
	tmpreg = (integerdivider / 100) << 4;
  
	// Determine the fractional part 
	fractionaldivider = integerdivider - (100 * (tmpreg >> 4));

	// Implement the fractional part in the register 
	if ((USART1->CR1 & USART_CR1_OVER8) != 0)
	{
		tmpreg |= (((fractionaldivider * 8) + 50) / 100) & (0x07);
	}
	else // if ((USARTx->CR1 & USART_CR1_OVER8) == 0) 
	{
		tmpreg |= (((fractionaldivider * 16) + 50) / 100) & (0x0F);
	}

	// Write to USART BRR register
	USART1->BRR = (uint16_t)tmpreg;
}

void SerialPutChar(uint8_t Ch) // 1���� ������ �Լ�
{
        while((USART1->SR & USART_SR_TXE) == RESET); //  USART_SR_TXE:0x0080, �۽� ������ ���±��� ���

	USART1->DR = (Ch & 0x01FF);	// ����
}

void Serial_PutString(char *str) // �������� ������ �Լ�
{
	while (*str != '\0') // ���Ṯ�ڰ� ������ ������ ����, ���Ṯ�ڰ� �����Ŀ��� ������ �޸� ���� �߻����ɼ� ����.
	{
		SerialPutChar(*str);	// �����Ͱ� ����Ű�� ���� �����͸� �۽�
		str++; 			// ������ ��ġ ����
	}
}

void goal_pulse() //
{
  int Degree; //��ǥ ���� ���� 
  int spec; //���� ���� ����
  char str_gp[20]; //��ǥ �޽��� ��� ����
 
  int i_ch; //�տ��� ���� ����Ʈ�� �����͸� int�� �����Ű�� ���� ����
  Degree = X[0]+ X[1]*10 + X[2]*100; //1�� �ڸ� 10�� �ڸ� 100�� �ڸ��� ���� ���Ѵ�.
 
  i_ch = atoi(ch); //char���� ���� int������ ��ȯ ��Ų��.
  spec = i_ch; //spec�� i_ch���� �ִ´�.
  
  Goal_Pulse = Degree/spec; //��ǥ �޽��� ��ǥ����/���ͽ����� �ִ´�.
  if(Goal_Pulse <=9 & Goal_Pulse > 0) //��ǥ �޽��� 0���� 99
  {
    sprintf(str_gp, "%02d", Goal_Pulse);
    LCD_SetTextColor(RGB_RED);//������
    LCD_DisplayText(4,20,str_gp);
  }
  if(Goal_Pulse <=99 & Goal_Pulse > 9) //��ǥ �޽��� 10���� 99
  {
    sprintf(str_gp, "%03d", Goal_Pulse);
    LCD_SetTextColor(RGB_RED);//������
    LCD_DisplayText(4,19,str_gp);
  }
  if(Goal_Pulse >99 & Goal_Pulse<1000) //��ǥ�޽��� 100���� 999
  {
    sprintf(str_gp, "%3d", Goal_Pulse);
    LCD_SetTextColor(RGB_RED);//������
    LCD_DisplayText(4,19,str_gp);
  }
}

void current_pulse()
{
  char str_cp[20];
  //Current_P = 0;
  Current_P++;
  if(Current_P == Goal_Pulse)
  {
    Current_P = 0; //
    stop = 1; //stop flag�� 1�� ����� �ش�.
    SW4_Flag = 0; //sw4 �÷��׸� 0���� ������ش�.
    GPIOG->ODR &= ~(0x0010); //LED4
    TIM2->CR1	&= ~(1<<0);	// CEN: Counter TIM2 Disable
  }
  LCD_SetTextColor(RGB_BLACK); //������
  LCD_DisplayText(4,17,": ");//�޽��� ���� �� ���ڰ� ���� ƨ��� ���󶧹��� 
  LCD_DisplayText(5,17,": "); //����Ͽ���.
  
  LCD_SetTextColor(RGB_RED); //������
  sprintf(str_cp, "%3d", Current_P); //��
  LCD_DisplayText(6,15,str_cp);
}

void goal_speed()
{
  /**** (PSC/CLK) * ARR = 1/GS *****/
  /**** ARR = (CLK/PSC)/GS *****/
  char str_gs[20]; //��ǥ���ǵ带 ��� �迭
  
  Goal_Speed = Goal_Pulse / T; //������Ʈ3�� �ִ� ���� �̿��Ѵ�.
  
  if(Goal_Speed <=9 & Goal_Speed > 0)
  {
    sprintf(str_gs, "%02d", Goal_Speed); //2�ڸ����� ������ 0�� �߰��Ͽ� 2�ڸ� Ȯ��
    LCD_SetTextColor(RGB_RED);//������
    LCD_DisplayText(5,20,str_gs);
    
    TIM2->ARR = TIM2->PSC / (Goal_Speed-1);// 0.1ms * ARR
    TIM2->CCR4 = TIM2->ARR/2; //��Ƽ�� 50%�� ������Ų��.
  }
  if(Goal_Speed >9 & Goal_Speed<100)
  {
    sprintf(str_gs, "%03d", Goal_Speed); //3�ڸ����� ������ 0�� �߰��Ͽ� 3�ڸ� Ȯ��
    LCD_SetTextColor(RGB_RED);//������
    LCD_DisplayText(5,19,str_gs);

    TIM2->ARR = TIM2->PSC / (Goal_Speed-1); // 0.1ms * ARR
    TIM2->CCR4 = TIM2->ARR/2; //��Ƽ�� 50%�� ������Ų��.
  }
  if(Goal_Speed >= 100 & Goal_Speed < 1000)
  {
    sprintf(str_gs, "%3d", Goal_Speed);
    LCD_SetTextColor(RGB_RED);//������
    LCD_DisplayText(5,19,str_gs);

    TIM2->ARR = TIM2->PSC / (Goal_Speed-1); // 0.1ms * ARR
    TIM2->CCR4 = TIM2->ARR/2; //��Ƽ�� 50%�� ������Ų��.
  }    
}

void BEEP(void)			// Beep for 20 ms 
{ 	
  GPIOF->ODR |= (1<<9);	// PF9 'H' Buzzer on
  DelayMS(20);		// Delay 20 ms
  GPIOF->ODR &= ~(1<<9);	// PF9 'L' Buzzer off
}

void DelayMS(unsigned short wMS)
{
	register unsigned short i;
	for (i=0; i<wMS; i++)
		DelayUS(1000);   // 1000us => 1ms
}

void DelayUS(unsigned short wUS)
{
	volatile int Dly = (int)wUS*17;
		for(; Dly; Dly--);
}

void DisplayTitle(void)
{
  LCD_Clear(RGB_WHITE);
  LCD_SetFont(&Gulim7);		//��Ʈ 
  LCD_SetBackColor(RGB_GREEN);	//���ڹ���
  LCD_SetTextColor(RGB_BLACK);	//���ڻ�
  LCD_SetBackColor(RGB_GREEN);	//���ڹ��� (�ʷ�)
  LCD_DisplayText(0,0,"Step Motor Pulse Gen.");

  LCD_SetBackColor(RGB_WHITE);	//���ڹ���
  LCD_DisplayText(1,0,"Motor spec(D): 00(deg/p)");
  LCD_DisplayText(2,0,"Goal Degree(X): 000(deg)");
  LCD_DisplayText(3,0,"Goal Time(T): 00(sec)");
  LCD_DisplayText(4,0,"Goal Pulse(P=X/D): 000(p)");
  LCD_DisplayText(5,0,"Goal Speed(V=P/T): 000(p/s)");
  
  LCD_SetTextColor(RGB_BLUE);//�Ķ���
  LCD_DisplayText(1,15,"00");
  LCD_DisplayText(2,16,"000");
  LCD_DisplayText(3,14,"00");
  LCD_SetTextColor(RGB_RED);//������
  LCD_DisplayText(4,19,"000");
  LCD_DisplayText(5,19,"000");
  
  
  LCD_SetTextColor(RGB_RED);//������
  LCD_DisplayText(6,0,"Current Pulse: 000(p)");
}

uint8_t key_flag = 0;
uint16_t KEY_Scan(void)	// input key SW0 - SW7 
{ 
	uint16_t key;
	key = GPIOH->IDR & 0xFF00;	// any key pressed ?
	if(key == 0xFF00)		// if no key, check key off
	{  	if(key_flag == 0)
        		return key;
      		else
		{	DelayMS(10);
        		key_flag = 0;
        		return key;
        	}
    	}
  	else				// if key input, check continuous key
	{	if(key_flag != 0)	// if continuous key, treat as no key input
        		return 0xFF00;
      		else			// if new key,delay for debounce
		{	key_flag = 1;
			DelayMS(10);
 			return key;
        	}
	}
}

int8_t Navi_key_flag = 0; //Ű ��ĵ�� ���� �÷��� ����

uint16_t Navi_KEY_Scan(void)	// input key SW0 - SW7 
{ 
	uint16_t  Navi_key;
	Navi_key = GPIOI->IDR & 0x0FF0;	// any key pressed ?
	if(Navi_key == 0x0FF0)		// if no key, check key off
	{  	if(Navi_key_flag == 0)
        		return Navi_key;
      		else
		{	DelayMS(100);
        		Navi_key_flag = 0;
        		return Navi_key;
        	}
    	}
  	else				// if key input, check continuous key
	{	if(Navi_key_flag != 0)	// if continuous key, treat as no key input
        		return 0x0FF0;
      		else			// if new key,delay for debounce
		{	Navi_key_flag = 1;
			DelayMS(100);
 			return Navi_key;
        	}
	}
}

/******************************************************************************/
/*     RCC Set up                                                             */
/******************************************************************************/
void _RCC_Init(void)
{
    // PLL (clocked by HSE) used as System clock source                    

    __IO uint32_t StartUpCounter = 0, HSEStatus = 0;

    // Enable HSE : �ܺ� �Է� Ŭ�����ļ�: 5MHz(ȸ�ε� ����)
    RCC->CR |= 0x00010000;	// Set bit#16 of CR
 
    // Wait till HSE is ready and if Time out is reached exit 
    do
    {
	HSEStatus = RCC->CR & 0x00020000;	// CHECK bit#17 of CR (HSE RDY?) 
	StartUpCounter++;
    } while((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

    if ((RCC->CR & 0x00020000) != RESET)	// CHECK bit#17 of CR (HSE RDY?) // RESET is 0
    {
	HSEStatus = 0x01;	// HSE is Ready!
    }
    else
    {
	HSEStatus = 0x00;	// HSE is NOT Ready!
    }

    if (HSEStatus == 0x01)	// HSE clock Enable
    {
	// HCLK = SYSCLK / 1 (HCLK = 168MHz)
	RCC->CFGR |= 0x00000000;
 
	// PCLK2 = HCLK / 2  (PCLK2 = 84MHz)
	RCC->CFGR |= 0x00008000;	// PPRE2: APB(Advanced Peripheral Bus)(APB2) High-speed prescaler
					// 100: AHB clock divided by 2 

	// PCLK1 = HCLK / 4  (PCLK1 = 42MHz)
	RCC->CFGR |= 0x00001400;	// PPRE1: APB(Advanced Peripheral Bus)(APB1) Low-speed prescaler
					// 101: AHB clock divided by 4 

    	// Configure the main PLL 
	// Reset vlaue: 0x2400 3010 (PPLQ:4, PLLSR:0, PLL_M:16, PLL_N:192, PLL_P: 2(00))
        RCC->PLLCFGR = 0;
	RCC->PLLCFGR |= 8;		// PLL_M(6bits): 8(0b001000): /8
	RCC->PLLCFGR |= (336 << 6);	// PLL_N(9bits): 336 : *336
	RCC->PLLCFGR |= (0<<16);	// PLL_P(2bits): (2 >> 1) -1=0b00 : 2 : /2 
	RCC->PLLCFGR |= 0x00400000; 	// PLL_SR(1bit): 1 : HSE oscillator clock selected as PLL and PLLI2S clock
//      RCC->PLLCFGR = 0x24405408;
	// SYSCLK ��� (HSE �Է�Ŭ�����ļ�: 8MHz)
	// SYSCLK = 8M * 336(N) /8(M) /2(P) = 168MHz	
    
	// Enable the main PLL 
	RCC->CR |= 0x01000000;	// Set bit#24 of CR : PLL ON

	// Wait till the main PLL is ready 
	while((RCC->CR & 0x02000000) == 0)	// CHECK bit#25 : PLL RDY?  
	{}
   
	// Configure Flash prefetch, Instruction cache, Data cache and wait state 
	FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_5WS;

	// Select the main PLL as system clock source 
	// Reset value of RCC->CFGR: 0x0000 0000
	RCC->CFGR &= ~0x00000003;	// clear, (Reset value: HSI) 
	RCC->CFGR |= 0x00000002;	// PLL

	// Wait till the main PLL is used as system clock source 
	while ((RCC->CFGR & 0x0000000C ) != 0x00000008);	// CHECK bit#2~3 : PLL as system clock is RDY?  
	{}
    }
    else
    { // If HSE fails to start-up, the application will have wrong clock
        // configuration. User can add here some code to deal with this error 
    }
}

