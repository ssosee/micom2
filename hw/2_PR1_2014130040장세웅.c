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

void _RCC_Init(void);
void _GPIO_Init(void);
void DisplayTitle(void);

/**/void _ADC1_Init(void);
/**/void _ADC2_Init(void);
/**/void TIMER3_Init(void);
/**/void TIMER6_Init(void);

/**/void USART1_Init(void);
/**/void USART_BRR_Configuration(uint32_t USART_BaudRate);
/**/void SerialPutChar(uint8_t c);
/**/void Serial_PutString(char *s);


uint16_t KEY_Scan(void);
void _EXTI_Init(void);

void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);
void BEEP(void);

/**/unsigned short ADC_Value, ADC2_Value2;
/**/int ADC2_Flag = 0;
/**/int ADC1_Flag = 0;
/**/int RX = 0; // Computer의 신호를 받는 변수
/**/char str[20]; // 16진수를 담을 배열
/**/char str2[20];// 10진수를 담을 배열


void USART1_IRQHandler(void) //
{
  if((USART1->SR & USART_SR_RXNE)) //Read Data Register Not Empty
  {
    if(USART1 ->  DR == '1') //DR에 1을 받으면
    {
      RX = 1; //RX에 1을 넣는다.(TIM6 핸들러의 조건 만족)
    }
    else if(USART1 ->  DR == '2') //DR에 2을 받으면
    {
      RX = 2;
    }
    else if(USART1 ->  DR == '3') //DR에 3을 받으면
    {
      RX = 3;
    }
    else if(USART1 ->  DR == '4') //DR에 4을 받으면
    {
      RX = 4;
    }
  }
}
void TIM6_DAC_IRQHandler(void)
{
  TIM6->SR &= ~0x01;
  
  if(RX == 1) //RX가 1이면
  {
        Serial_PutString("H:0x"); //터미널 연결 프로그램 창에 현재 지자기센서 값(가변저항값) 출력
        Serial_PutString(str);
        Serial_PutString("\r\n");    //줄 바꿈
  }
   else if(RX == 2)
   {
        Serial_PutString("A:"); //터미널 연결 프로그램 창에 현재 가속도센서 값(가변저항값) 출력
        Serial_PutString(str2);
        Serial_PutString("\r\n");    //줄 바꿈
   }
  else if(RX == 3)
  {
    Serial_PutString("H:0x"); //터미널 연결 프로그램 창에 현재 지자기센서 값(가변저항값) 출력
    Serial_PutString(str);
    Serial_PutString(" "); //한 칸 띄어쓰기
    Serial_PutString("A:"); //터미널 연결 프로그램 창에 현재 가속도센서 값(가변저항값) 출력
    Serial_PutString(str2);
    Serial_PutString("\r\n"); // 줄바꿈
  }
  else if(RX ==4)
  {
    //Stop(동작 중지)
  }
}

void EXTI9_5_IRQHandler(void)		// EXTI 9~5인터럽트 핸들러
{ // SW0,1은 EXTI 8,9로 
    if(EXTI->PR & 0x0100) //SW0 pending?
    {
        EXTI->PR |= 0x0100; //pending bit clear
        GPIOG->ODR &= ~0x00FF; //LED ALL OFF
        GPIOG->ODR |= 0x0002; // LED1 ON
        ADC1->CR2 &= ~(1<<0); //ADC1 Disable
    }
     if(EXTI->PR & 0x0200) //SW1 pending?
    {
        EXTI->PR |= 0x0200; //pending bit clear
        GPIOG->ODR &= ~0x00FF; //LED ALL OFF
        GPIOG->ODR |= 0x0004; // LED2 ON
        ADC2->CR2 &= ~(1<<0);  ////ADC2 Disable
    }
}

void EXTI15_10_IRQHandler(void)		// EXTI 15~10인터럽트 핸들러
{ // SW3,4은 EXTI 11,12로 
    if(EXTI->PR & 0x0800) //SW3 pending?
    {
        EXTI->PR |= 0x0800; //pending bit clear
        GPIOG->ODR &= ~0x00FF; //LED ALL OFF
        GPIOG->ODR |= 0x0008; // LED3 ON
        ADC1->CR2 |= (1<<0); //ADC1 Enable
    }
     if(EXTI->PR & 0x1000) //SW4 pending?
    {
        EXTI->PR |= 0x1000; //pending bit clear
        GPIOG->ODR &= ~0x00FF; //LED ALL OFF
        GPIOG->ODR |= 0x0010; // LED4 ON
        ADC2->CR2 |= (1<<0); //ADC2 Enable
        
        ADC2->CR2 |= ADC_CR2_SWSTART; //가속도 센서 시작! (소프트웨어적으로)
    }
}

void ADC_IRQHandler(void) //ADC 인터럽트 수행문
{   
   if(ADC1->SR & 0x02) //ADC SR EOC bit Enable
   {
     ADC1->SR &= ~(1<<1); // EOC flag clear
     ADC1_Flag = 1; //Flag에 1을 넣어줌-> 메인문 실행 조건
   }
   
   if(ADC2->SR & 0x02) //ADC SR EOC bit Enable
   {
     ADC2->SR &= ~(1<<1); // EOC flag clear
     ADC2_Flag = 1;
   }
}

int main(void)
{
	_RCC_Init();
	LCD_Init();		// LCD 구동 함수
	DelayMS(10);	// LCD구동 딜레이
 	DisplayTitle();	//LCD 초기화면구동 함수
    _EXTI_Init() ; //인터럽트 설정함수
    TIMER3_Init();        //타이머3 설정 함수
    TIMER6_Init();//타이머6 설정 함수
    _GPIO_Init();
	_ADC1_Init(); //ADC1 설정함수
    _ADC2_Init(); //ADC2 설정함수
    
    USART1_Init();
    
    
    ADC2->CR2 |= ADC_CR2_SWSTART; //가속도 센서 시작! (소프트웨어적으로)
       
 	while(1)
	{
          if( ADC1_Flag == 1) //ADC 의 인터럽트 핸들러가 실행되고 ADC1 EOC가 Enable 할 때
          {
            ADC_Value = ADC1->DR; //ADC1의 Data Reg 값을 ADC_Value에 넣는다.
            sprintf(str,"%03x",ADC_Value);    // 3자리까지
            LCD_DisplayText(1,9,str);
            ADC1_Flag = 0; //플래그 값 초기화
          }
          
           if( ADC2_Flag == 1) //ADC 의 인터럽트 핸들러가 실행되고 ADC2 EOC가 Enable 할 때
          {
            ADC2_Value2 = ADC2->DR; //ADC2의 Data Reg 값을 ADC2_Value2에 넣는다.
            sprintf(str2,"%04d",ADC2_Value2); // 4자리까지
            LCD_DisplayText(2,5,str2);
            ADC2_Flag = 0; //플래그 값 초기화
          }
	}
}

void _ADC1_Init(void) //지자기센서(가변저항)
{   
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; 	// 0x00000001  // ENABLE GPIOA CLK
	GPIOA->MODER |= GPIO_MODER_MODER1;       // 0x0000000C	// CONFIG GPIOA PIN1(PA1) TO ANALOG IN MODE
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;	// 0x00000100  // ENABLE ADC1 CLK

	ADC->CCR &= ~0X0000001F;	// MULTI[4:0]: ADC_Mode_Independent
	ADC->CCR |= 0x00010000;		// ADCPRE: ADC_Prescaler_Div4 (ADC MAX Clock 36MHz, 84Mhz(APB2)/4 = 21MHz)
	ADC->CCR &= ~0x0000C000;	// DMA: Disable
    ADC->CCR |= 0x00000F00;		// ADC_TwoSamplingDelay_20Cycles
        
	ADC1->CR1 &= ~(3<<24);		// RES[1:0]: 12bit Resolution
	ADC1->CR1 &= ~(1<<8);		// SCAN: ADC_ScanCovMode Disable
	ADC1->CR1 |= 1<<5;		// EOCIE: Interrupt enable for EOC

	/**/ADC1->CR2 &= ~(1<<1);		// CONT: ADC_ContinuousConvMode Disable
    //단일모드로
    
    //단일이냐 연속모드냐(현재 연속 ~(1<<1) ) 연속모드: 시작 명령 한 번은 줘양함 -> 대신 한 번 시작하면 자동시작
    
	ADC1->CR2 |= (2<<28);		// EXTEN[1:0]: ADC_ExternalTrigConvEdge_Enable(Falling Edge)
	/**/ADC1->CR2 |= (7<<24);	// EXTSEL[3:0]: ADC_ExternalTrig (EXTI11)
    //타이머3 CC1 event 0x0111
	ADC1->CR2 &= ~(1<<11);		// ALIGN: ADC_DataAlign_Right
	ADC1->CR2 |= (1<<10);		// EOCS: The EOC bit is set at the end of each regular conversion
	//연속 모드, 스케일 모드 EOCS를 고쳐야함 (0<<10) 
    //연속모드, 스캔모드 eoc:0
    ADC1->CR2 |= 1<<0;		// ADON: ADC ON

	ADC1->SQR1 &= ~0x00F00000;	// L[3:0]: ADC Regular channel sequece length = 1 conversion
    
    /**/ADC1->SMPR2	|= 0x07 << (3*1);	// ADC1_CH1 Sample TIme_480Cycles (3*Channel_1)
 	//Channel selection, The Conversion Sequence of PIN1(ADC1_CH1) is first, Config sequence Range is possible from 0 to 17
	ADC1->SQR3 |= 0x01<<0;

	NVIC->ISER[0] |= (1<<18);	// Enable ADC global Interrupt
    
    //연속 모드, 스케일 모드 EOCS를 고쳐야함 (0<<10)
}

void _ADC2_Init(void) //가속도센서
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; 	// 0x00000004  // ENABLE GPIOA CLK
	GPIOC->MODER |= GPIO_MODER_MODER1;       // 0x0000000C	// CONFIG GPIOC PIN1(PC1) TO ANALOG IN MODE
	RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;	// 0x00000200  // ENABLE ADC2 CLK

	ADC->CCR &= ~0X0000001F;	// MULTI[4:0]: ADC_Mode_Independent
	ADC->CCR |= 0x00010000;		// ADCPRE: ADC_Prescaler_Div4 (ADC MAX Clock 36MHz, 84Mhz(APB2)/4 = 21MHz)
	ADC->CCR &= ~0x0000C000;	// DMA: Disable
    ADC->CCR |= 0x00000F00;		// ADC_TwoSamplingDelay_20Cycles
        
	ADC2->CR1 &= ~(3<<24);		// RES[1:0]: 12bit Resolution
	ADC2->CR1 &= ~(1<<8);		// SCAN: ADC_ScanCovMode Disable
	ADC2->CR1 |= 1<<5;		// EOCIE: Interrupt enable for EOC

	/**/ADC2->CR2 |= (1<<1);		// CONT: ADC_ContinuousConvMode Disable
    //연속모드로
    
    //단일이냐 연속모드냐(현재 연속 ~(1<<1) ) 연속모드: 시작 명령 한 번은 줘양함 -> 대신 한 번 시작하면 자동시작
    
	/**/ADC2->CR2 &= ~(3<<28);		// EXTEN[1:0]: ADC_ExternalTrigConvEdge_Disable
    //타이머3 CC1 event 0x0111
	ADC2->CR2 &= ~(1<<11);		// ALIGN: ADC_DataAlign_Right
	/**/ADC2->CR2 &= ~(1<<10);		// EOCS: The EOC bit is set at the end of each regular conversion
	//연속 모드, 스케일 모드 EOCS를 고쳐야함 (0<<10) 
    //연속모드, 스캔모드 eoc:0
    ADC2->CR2 |= 1<<0;		// ADON: ADC ON (Enable)

	ADC2->SQR1 &= ~0x00F00000;	// L[3:0]: ADC Regular channel sequece length = 1 conversion
    
    /**/ADC2->SMPR1	|= 0x00 << (3*1);	// ADC2_CH11 Sample TIme_480Cycles (3*Channel_1)
 	//Channel selection, The Conversion Sequence of PIN1(ADC1_CH1) is first, Config sequence Range is possible from 0 to 17
	ADC2->SQR2 |= 0x01<<20; //SQ11
	NVIC->ISER[0] |= (1<<18);	// Enable ADC global Interrupt
    
    //연속 모드, 스케일 모드 EOCS를 고쳐야함 (0<<10)
}

void TIMER3_Init(void) //타이머3 채널1 CC_Event
{
   //Variable Resistor (EVU-F3AF30B14)
	///**/RCC->AHB1ENR	|= 0x00000001;	// RCC_AHB1ENR GPIOA Enable
    						
	 //PA1을 출력설정하고 Alternate function(TIM3_CH1)으로 사용 선언
    /**/GPIOA->MODER 	|= 0x00000008;// GPIOA PIN1 Output Alternate function mode					
	/**/GPIOA->OSPEEDR |= 0x0000000C;	// GPIOA PIN1 Output speed (100MHz High speed)
	/**/GPIOA->OTYPER	&= 0x00;	// GPIOA PIN1 Output type push-pull (reset state)
	/**/GPIOA->PUPDR	|= 0x00000008;	// GPIOA PIN1 Pull-down
    /**/GPIOA->AFR[0]	|= 0x00000020;// AFRL(AFR[0]): Connect PA1 to AF2 (TIM3)
	 //PA1 ==> TIM3_CH1
    
    // Timerbase Mode
    /**/RCC->APB1ENR 	|= 0x00000002;	// RCC_APB2ENR TIMER3 Enable 
	/**/NVIC->ISER[0] |= (1<<29 );// Enable Timer3_CC Caputre Compare Interrupt on NVIC

	TIM3->PSC = 840-1;	// Prescaler 84,000,000Hz/840 = 10,0000 Hz (0.01ms)  (1~65536) //-1은 rule임
	TIM3->ARR = 12500-1;	// Auto reload  0.01ms * 12500 = 125ms  
	
    TIM3->CR1 &= ~(1<<4);	//DIR: Countermode = Upcounter (reset state)
	TIM3->CR1 &= ~(3<<8); 	// CKD(Clock division)=1(reset state)
	TIM3->CR1 &= ~(3<<5); 	// CMS(Center-aligned mode Sel): No(reset state)   
	
    // Output/Compare Mode
	/**/TIM3->CCER |= (1<<0);	// CC1E: OC1 Active 
	/**/TIM3->CCER &= ~(1<<1);	// CC1P: OCPolarity_High
	TIM3->CCR1 = 0;	// TIM3_CH1_Pulse

	/**/TIM3->EGR |= (1<<1);    // CC1G: Capture/Compare 1 event generation    

	/**/TIM3->CCMR1 &= ~(3<<0); // CC1S(CC1 channel): Output 
	/**/TIM3->CCMR1 &= ~(1<<3); // OC1PE: Output Compare 1 preload disable
    /**/TIM3->CCMR1 |= (3<<4);	// OC1M: Output Compare 1 Mode : toggle

	/**/TIM3->CR1 &= ~(1<<7);	// ARPE: Auto reload preload disable
	TIM3->CR1 |= (1<<0);	// CEN: Enable the Tim3 Counter
}

void TIMER6_Init(void)
{   
    /**/RCC->APB1ENR 	|= (1<<4);	// RCC_APB2ENR TIMER6 Enable 
	/**/NVIC->ISER[1] |= (1<<(54-32) );// Enable Timer6_DAC Interrupt on NVIC

	/**/TIM6->PSC = 840-1;	// Prescaler 84,000,000Hz/840 = 10,0000 Hz (0.01ms)  (1~65536) //-1은 rule임
	/**/TIM6->ARR = 500-1;	// Auto reload  0.01ms * 500 = 5ms  
    
	/**/TIM6->EGR |= (1<<0);    //  Update Generation Enable
    /**/TIM6->DIER |= (1<<0); // Update intterupt Enable
    
	TIM6->CR1 |= (1<<0);	// CEN: Enable the Tim6 Counter
}

void _GPIO_Init(void)
{
	// LED GPIO(PORT G) 설정
    RCC->AHB1ENR    |= 0x00000040; 	// RCC_AHB1ENR(bit8~0) GPIOG(bit#6) Enable							
	GPIOG->MODER    = 0x00005555;	// GPIOG PIN0~PIN7 Output mode (0b01)						
	GPIOG->OTYPER   = 0x0000;	// GPIOG PIN0~PIN7 : Push-pull  (PIN8~PIN15) (reset state)	
 	GPIOG->OSPEEDR  = 0x00005555;	// GPIOG PIN0~PIN7 Output speed (25MHZ Medium speed) 
   
	// Buzzer GPIO(PORT F) 설정 
    RCC->AHB1ENR    |= 0x00000020; 	// RCC_AHB1ENR(bit8~0) GPIOF(bit#5) Enable							
	GPIOF->MODER    |= 0x00040000;	// GPIOF PIN9 Output mode (0b01)						
	GPIOF->OTYPER   &= 0xFDFF;	// GPIOF PIN9 : Push-pull  	
 	GPIOF->OSPEEDR  |= 0x00040000;	// GPIOF PIN9 Output speed (25MHZ Medium speed) 
	
	//NAVI.SW(PORT I) 설정
	RCC->AHB1ENR 	|= 0x00000100;	// RCC_AHB1ENR GPIOI Enable
	GPIOI->MODER 	= 0x00000000;	// GPIOI PIN8~PIN15 Input mode (reset state)
	GPIOI->PUPDR    = 0x00000000;	// GPIOI PIN8~PIN15 Floating input (No Pull-up, pull-down) (reset state)
	
	// DC_MOTOR DIR
	RCC->AHB1ENR	|= 0x00000002; 	// RCC_AHB1ENR GPIOB Enable							
	GPIOB->MODER 	= 0x00040000;	// GPIOB PIN9 Output mode						
	GPIOB->OSPEEDR 	= 0x00040000;	// GPIOB PIN9 Output speed (25MHZ Medium speed) 
}	

/**/void _EXTI_Init(void)    //EXTI12 ~ 15(PH12 ~ 15, SW (0,1,3,4) )
{
    RCC->AHB1ENR    |= 0x80;	// RCC_AHB1ENR GPIOH Enable
	RCC->APB2ENR 	|= 0x4000;	// Enable System Configuration Controller Clock
	
	GPIOH->MODER 	&=  0x00FF;	// GPIOH PIN8~PIN15 Input mode (reset state)				 
	SYSCFG->EXTICR[2] |= 0x00007077;      // EXTI8,9,10에 대한 소스 입력은 GPIOH로 설정 (EXTICR3) (reset value: 0x0000)	
	SYSCFG->EXTICR[3] |= 0x00000007; 	// EXTI11에 대한 소스 입력은 GPIOH로 설정 (EXTICR4) (reset value: 0x0000)	
	
	EXTI->FTSR |= 0x001B00;		// Falling Trigger Enable  (EXTI8 ~ 12:PH 8 ~ 12)
    EXTI->IMR |= 0x001B00;  	// EXTI8 ~ 12 인터럽트 mask (Interrupt Enable)
		
	NVIC->ISER[1] |= ( 1 << (40-32) );// Enable Interrupt EXTI 10 ~ 15 Vector table Position 참조
    NVIC->ISER[0] |= ( 1 << 23 );// Enable Interrupt EXTI 9 ~ 5Vector table Position 참조
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
    
	USART_BRR_Configuration(19200); // USART Baud rate Configuration
    
	/**/USART1->CR1	&= ~USART_CR1_M;	// USART_WordLength 8 Data bit
	/**/USART1->CR1	&= ~USART_CR1_PCE ;	// USART_Parity_none
    USART1->CR1	|= USART_CR1_RE;	// 0x0004, USART_Mode_RX Enable
	USART1->CR1	|= USART_CR1_TE ;	// 0x0008, USART_Mode_Tx Enable
	USART1->CR2	&= ~USART_CR2_STOP;	// USART_StopBits_1
	USART1->CR3	= 0x0000;	// No HardwareFlowControl, No DMA
    //cr3가 플로우 컨트롤 부분
	
    USART1->CR1 	|= USART_CR1_RXNEIE;	//  0x0020, RXNE interrupt Enable /*****(뒤에 IE는 마스킹 비트다)****/
	NVIC->ISER[1]	|= (1 << 5); 	// Enable Interrupt USART1 (NVIC 37번)
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
void SerialPutChar(uint8_t Ch) // 1문자 보내기 함수
{
        while((USART1->SR & USART_SR_TXE) == RESET); //  USART_SR_TXE:0x0080, 송신 가능한 상태까지 대기

	USART1->DR = (Ch & 0x01FF);	// 전송
}
void Serial_PutString(char *str) // 여러문자 보내기 함수
{
	while (*str != '\0') // 종결문자가 나오기 전까지 구동, 종결문자가 나온후에도 구동시 메모리 오류 발생가능성 있음.
	{
		SerialPutChar(*str);	// 포인터가 가르키는 곳의 데이터를 송신
		str++; 			// 포인터 수치 증가
	}
}
void BEEP(void)			// Beep for 20 ms 
{ 	GPIOF->ODR |= (1<<9);	// PF9 'H' Buzzer on
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
    LCD_Clear(RGB_YELLOW);
    LCD_SetFont(&Gulim10);		//폰트 
    LCD_SetBackColor(RGB_GREEN);	//글자배경색
    LCD_SetTextColor(RGB_BLACK);	//글자색
    LCD_DisplayText(0,0,"Sensor               ");
    LCD_SetBackColor(RGB_YELLOW);	//글자배경색
    LCD_DisplayText(1,0,"Height:0x000");
    LCD_DisplayText(2,0,"AccX:0000");

    LCD_SetBackColor(RGB_YELLOW);	//글자배경색
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


/******************************************************************************/
/*     RCC Set up                                                             */
/******************************************************************************/
void _RCC_Init(void)
{
    // PLL (clocked by HSE) used as System clock source                    

    __IO uint32_t StartUpCounter = 0, HSEStatus = 0;

    // Enable HSE : 외부 입력 클락주파수: 5MHz(회로도 참조)
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
	// SYSCLK 계산 (HSE 입력클럭주파수: 8MHz)
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

