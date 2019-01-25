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

void _ADC_Init(void);
/**/void TIMER5_Init(void);
/**/void TIMER14_Init(void);
/**/void _EXTI_Init(void);
uint16_t KEY_Scan(void);

void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);
//void BEEP(void);

unsigned short ADC_Value, Voltage, Temp;
uint8_t sw7_flag = 1;
//char str[20];


void ADC_IRQHandler(void)
{
	ADC1->SR &= ~(1<<1);	// EOC flag clear

	ADC_Value = ADC1->DR;

    //sprintf(str,"%4d",ADC_Value);
    //LCD_DisplayText(5,6,str);
	Voltage = ADC_Value * (3.3 * 100) / 4095;   // 3.3 : 4095 =  Volatge : ADC_Value 
                                                                // 100:  소수점아래 두자리까지 표시하기 위한 값  
	LCD_DisplayChar(0,4,Voltage/100 + 0x30); //10의 자리
	LCD_DisplayChar(0,6,(Voltage%100)/10 + 0x30); //1의 자리
	LCD_DisplayChar(0,7,Voltage%10+ 0x30); //소수점 첫 째 자리

    Temp = ((3.5 * (Voltage * Voltage)/10000) + 1)* 10; //온도 식 현재 Voltage (온도에 소수점을 만들기 위해 10000을 나누고 10을 곱한다.)
    LCD_DisplayChar(1,5,Temp/100 + 0x30); //10의 자리
	LCD_DisplayChar(1,6,(Temp%100)/10 + 0x30); // 1의 자리
    LCD_DisplayChar(1,8,Temp%10 + 0x30);//소수점 첫 째자리
    
    //DR = CCR/ARR *100
            if(Temp  <= 10 && Temp >= 1) 
            {
                LCD_DisplayText(2,5,"100");
                LCD_SetBrushColor(GET_RGB(102,255,102)); //green
                LCD_DrawFillRect(3,55,12,22); //네모를 그린다.
                
                LCD_SetBrushColor(RGB_YELLOW); //YELLOW
                LCD_DrawFillRect(20,55,12,22); //네모를 그린다.
                LCD_DrawFillRect(37,55,12,22); //네모를 그린다.
                LCD_DrawFillRect(54,55,12,22); //네모를 그린다.
                LCD_DrawFillRect(71,55,12,22); //네모를 그린다.
                TIM14->PSC =8400-1;
                TIM14->ARR	= 100-1;
                TIM14->CCR1 =10;
            }
            else if( Temp >10 && Temp <= 97 )
            {
                LCD_DisplayText(2,5,"200");
                LCD_SetBrushColor(GET_RGB(102,255,102)); //green
                LCD_DrawFillRect(3,55,12,22); //네모를 그린다.
                LCD_DrawFillRect(20,55,12,22); //네모를 그린다.
               
                LCD_SetBrushColor(RGB_YELLOW); //YELLOW
                LCD_DrawFillRect(37,55,12,22); //네모를 그린다.
                LCD_DrawFillRect(54,55,12,22); //네모를 그린다.
                LCD_DrawFillRect(71,55,12,22); //네모를 그린다.
                
                TIM14->PSC =8400-1;
                TIM14->ARR	= 50-1;
                TIM14->CCR1 =5;
            }
            else if( Temp >97 && Temp <= 195 )
            {
                LCD_DisplayText(2,5,"300");
                LCD_SetBrushColor(GET_RGB(102,255,102)); //green
                LCD_DrawFillRect(3,55,12,22); //네모를 그린다.
                LCD_DrawFillRect(20,55,12,22); //네모를 그린다.
                LCD_DrawFillRect(37,55,12,22); //네모를 그린다.
                
                LCD_SetBrushColor(RGB_YELLOW); //YELLOW
                LCD_DrawFillRect(54,55,12,22); //네모를 그린다.
                LCD_DrawFillRect(71,55,12,22); //네모를 그린다.

                TIM14->PSC =8400-1;
                TIM14->ARR	= 100/3-1;
                TIM14->CCR1 =10/3;
            }
            else if( Temp >195 && Temp <= 292 )
            {
                LCD_DisplayText(2,5,"400");
                LCD_SetBrushColor(GET_RGB(102,255,102)); //green
                LCD_DrawFillRect(3,55,12,22); //네모를 그린다.
                LCD_DrawFillRect(20,55,12,22); //네모를 그린다.
                LCD_DrawFillRect(37,55,12,22); //네모를 그린다.
                LCD_DrawFillRect(54,55,12,22); //네모를 그린다.
                
                LCD_SetBrushColor(RGB_YELLOW); //YELLOW
                LCD_DrawFillRect(71,55,12,22); //네모를 그린다.

                TIM14->PSC =8400-1;
                TIM14->ARR	= 25-1;
                TIM14->CCR1 =25/10;
            }
            else if( Temp >292 && Temp <= 389 )
            {
                LCD_DisplayText(2,5,"500");
                LCD_SetBrushColor(GET_RGB(102,255,102)); //green
                LCD_DrawFillRect(3,55,12,22); //네모를 그린다.
                LCD_DrawFillRect(20,55,12,22); //네모를 그린다.
                LCD_DrawFillRect(37,55,12,22); //네모를 그린다.
                LCD_DrawFillRect(54,55,12,22); //네모를 그린다.
                LCD_DrawFillRect(71,55,12,22); //네모를 그린다.
                
                TIM14->PSC =8400-1;
                TIM14->ARR	= 20-1;
                TIM14->CCR1 =2;
            }
      

}

int main(void)
{
	_RCC_Init();
	LCD_Init();		// LCD 구동 함수
	DelayMS(10);	// LCD구동 딜레이
 	DisplayTitle();	//LCD 초기화면구동 함수
        
        _GPIO_Init();
        /**/_EXTI_Init(); //인터럽트 초기화 함수
        /**/TIMER5_Init(); //타이머5 초기화 함수
        /**/TIMER14_Init(); //타이머14 초기화 함수
	_ADC_Init();
       
 	//Starts conversion of regular channels
       	//ADC1->CR2 |= ADC_CR2_SWSTART; // 0x40000000 
	while(1)
	{
        
    
    }
}

void _ADC_Init(void)
{   
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; 	// 0x00000001  // ENABLE GPIOA CLK
    GPIOA->MODER |= GPIO_MODER_MODER1;       // 0x0000000C	// CONFIG GPIOA PIN1(PA1) TO ANALOG IN MODE
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;	// 0x00000100  // ENABLE ADC1 CLK
    
//GPIOA->MODER |= 0x00000003;
  
    ADC->CCR &= ~0X0000001F;	// MULTI[4:0]: ADC_Mode_Independent
	ADC->CCR |= 0x00010000;		// ADCPRE: ADC_Prescaler_Div4 (ADC MAX Clock 36MHz, 84Mhz(APB2)/4 = 21MHz)
	ADC->CCR &= ~0x0000C000;	// DMA: Disable
    ADC->CCR	|= 0x00000F00;	// ADC_TwoSamplingDelay_20Cycles
    
    ///**/ADC->CCR |= (1<<23); //Enable the temperature sensor    
    
	
    ADC1->CR1 &= ~(3<<24);		// RES[1:0]: 12bit Resolution
	ADC1->CR1 &= ~(1<<8);		// SCAN: ADC_ScanCovMode Disable
	ADC1->CR1 |= 1<<5;		// EOCIE: Interrupt enable for EOC

	ADC1->CR2 &= ~(1<<1);		// CONT: ADC_ContinuousConvMode Disable
	/**/ADC1->CR2 |= (2<<28);		// EXTEN[1:0]: ADC_ExternalTrigConvEdge_None
    ADC1->CR2 |= (0x0C<<24);	// EXTSEL[3:0]: ADC_ExternalTrig (EXTI11)
    //ADC1->CR2 |= 0x0C00 0000; //Bits 27:24 EXTSEL[3:0]: External event select for regular group Bits
	
    //1100: TIMER5 CC3 EVENT
    ADC1->CR2 &= ~(1<<11);		// ALIGN: ADC_DataAlign_Right
	ADC1->CR2 |= 1<<10;		// EOCS: The EOC bit is set at the end of each regular conversion
	ADC1->CR2 |= 1<<0;		// ADON: ADC ON
    ADC1->SMPR2	|= 0x07 << (3*1);	// ADC1_CH1 Sample TIme_480Cycles (3*Channel_1)
 	//Channel selection, The Conversion Sequence of PIN1(ADC1_CH1) is first, Config sequence Range is possible from 0 to 17
	ADC1->SQR3 |= 0x01<<0;

	NVIC->ISER[0] |= (1<<18);	// Enable ADC global Interrupt
}

void _EXTI_Init(void)
{
    RCC->AHB1ENR 	|= 0x80;	// RCC_AHB1ENR GPIOH Enable
	RCC->APB2ENR 	|= 0x4000;	//Enable System Configuration Controller Clock
	
	GPIOH->MODER 	&=  0x00FF;	// GPIOH PIN8~PIN15 Input mode (reset state)				 

    SYSCFG->EXTICR[3] |= 0x7700;  //EXTI14,15 대한 소스 입력은 GPIOH로 설정  
	
    EXTI->FTSR |= 0x00C300;		// Falling Trigger Enable  (EXTI8:PH8)
    //EXTI->RTSR |= 0x004000;		// Rising Trigger  Enable  (EXTI14:PH14)
    EXTI->IMR |= 0xC000;  	// EXTI14,15 인터럽트 mask (Interrupt Enable)
		
    NVIC->ISER[1] |= ( 1 << (40 - 32) ); // Enable Interrupt EXTI14,15 Vector table Position 참조
}

void _GPIO_Init(void)
{
	// LED GPIO(PORT G) 설정
    	RCC->AHB1ENR    |= 0x00000040; 	// RCC_AHB1ENR(bit8~0) GPIOG(bit#6) Enable							
	GPIOG->MODER    = 0x00005555;	// GPIOG PIN0~PIN7 Output mode (0b01)						
	GPIOG->OTYPER   = 0x0000;	// GPIOG PIN0~PIN7 : Push-pull  (PIN8~PIN15) (reset state)	
 	GPIOG->OSPEEDR  = 0x00005555;	// GPIOG PIN0~PIN7 Output speed (25MHZ Medium speed) 
    
	// SW GPIO(PORT H) 설정 
	RCC->AHB1ENR    |= 0x00000080;	// RCC_AHB1ENR(bit8~0) GPIOH(bit#7) Enable							
	GPIOH->MODER    = 0x00000000;	// GPIOH PIN8~PIN15 Input mode (reset state)				
	GPIOH->PUPDR    = 0x00000000;	// GPIOH PIN8~PIN15 Floating input (No Pull-up, pull-down) (reset state)

//	// Buzzer GPIO(PORT F) 설정 
//    	RCC->AHB1ENR    |= 0x00000020; 	// RCC_AHB1ENR(bit8~0) GPIOF(bit#5) Enable							
//	GPIOF->MODER    |= 0x00040000;	// GPIOF PIN9 Output mode (0b01)						
//	GPIOF->OTYPER   &= 0xFDFF;	// GPIOF PIN9 : Push-pull  	
// 	GPIOF->OSPEEDR  |= 0x00040000;	// GPIOF PIN9 Output speed (25MHZ Medium speed) 
	
	//NAVI.SW(PORT I) 설정
	RCC->AHB1ENR 	|= 0x00000100;	// RCC_AHB1ENR GPIOI Enable
	GPIOI->MODER 	= 0x00000000;	// GPIOI PIN8~PIN15 Input mode (reset state)
	GPIOI->PUPDR    = 0x00000000;	// GPIOI PIN8~PIN15 Floating input (No Pull-up, pull-down) (reset state)
	
	// DC_MOTOR DIR
	RCC->AHB1ENR	|= 0x00000002; 	// RCC_AHB1ENR GPIOB Enable							
	GPIOB->MODER 	= 0x00040000;	// GPIOB PIN9 Output mode						
	GPIOB->OSPEEDR 	= 0x00040000;	// GPIOB PIN9 Output speed (25MHZ Medium speed) 
}

void TIMER5_Init(void)
{
   //Variable Resistor (EVU-F3AF30B14)
	/**/RCC->AHB1ENR	|= 0x00000001;	// RCC_AHB1ENR GPIOA Enable
    						
	// PA2을 출력설정하고 Alternate function(TIM5_CH3)으로 사용 선언
    /**/GPIOA->MODER 	|= 0x00000020;// GPIOA PIN2 Output Alternate function mode					
	/**/GPIOA->OSPEEDR |= 0x0C000030;	// GPIOA PIN2 Output speed (100MHz High speed)
	/**/GPIOA->OTYPER	&= 0x00;	// GPIOA PIN2 Output type push-pull (reset state)
	/**/GPIOA->PUPDR	|= 0x00000010;	// GPIOE PIN2 Pull-up
    /**/GPIOA->AFR[0]	|= 0x00000200;// AFRL(AFR[0]): Connect PA2 to AF2 (TIM5)
	// PA2 ==> TIM5_CH3
    
    // Timerbase Mode
    /**/RCC->APB1ENR 	|= 0x00000008;	// RCC_APB2ENR TIMER5 Enable 
	/**/NVIC->ISER[1] |= (1<<(50-32) );// Enable Timer1_CC Caputre Compare Interrupt on NVIC

	TIM5->PSC = 840-1;	// Prescaler 84,000,000Hz/840 = 10,0000 Hz (0.01ms)  (1~65536) //-1은 rule임
	TIM5->ARR = 10000-1;	// Auto reload  0.01ms * 10000 = 100ms  
	TIM5->CR1 &= ~(1<<4);	//DIR: Countermode = Upcounter (reset state)
	TIM5->CR1 &= ~(3<<8); 	// CKD(Clock division)=1(reset state)
	TIM5->CR1 &= ~(3<<5); 	// CMS(Center-aligned mode Sel): No(reset state)
	
    TIM5->EGR |= (1<<0);    // UG: Update generation    
	/***/TIM5->DIER &= (0<<0);	// UIE: Enable Tim5 Update interrupt
	
    // Output/Compare Mode
	TIM5->CCER |= (1<<8);	// CC3E: OC3 Active 
	TIM5->CCER &= ~(1<<9);	// CC3P: OCPolarity_High
	TIM5->CCR3 = 0;	// TIM5_Pulse

	TIM5->EGR |= (1<<3);    // CC1G: Capture/Compare 1 event generation    

	/**/TIM5->CCMR2 &= ~(3<<0); // CC3S(CC3 channel): Output 
	/**/TIM5->CCMR2 &= ~(1<<3); // OC3PE: Output Compare 3 preload disable
	/**/TIM5->CCMR2 |= (3<<4);	// OC3M: Output Compare 3 Mode : toggle

	/**/TIM5->CR1 &= ~(1<<7);	// ARPE: Auto reload preload disable
	/***/TIM5->DIER &= (0<<3);	// CC3IE: Enable the Tim5 CC3 interrupt
	TIM5->CR1 |= (1<<0);	// CEN: Enable the Tim5 Counter
}

void TIMER14_Init(void)
{   
	/**/RCC->AHB1ENR	 |= 0x00000020;	// RCC_AHB1ENR GPIOF Disable
	/**/RCC->APB1ENR 	|= 0x00000100;	// RCC_APB1ENR TIMER14 Enable 
    						
	// PF9을 출력설정하고 Alternate function(TIM14_CH1)으로 사용 선언
	/**/GPIOF->MODER 	|= 0x00080000;	// GPIOF PIN9 Output Alternate function mode					
	/**/GPIOF->OSPEEDR 	|= 0x000C0000;	// GPIOF PIN9 Output speed (100MHz High speed)
	GPIOF->OTYPER	= 0x00000000;	// GPIOF PIN9 Output type push-pull (reset state)
	/**/GPIOF->PUPDR	|= 0x00070000;	// GPIOF PIN9 Pull-up  
	/**/GPIOF->AFR[1]	|= 0x00000090;	// AFRH: Connect TIM14 pins(PB9) to AF2(TIM14)
					// PF9 ==> TIM14_CH3
    
	// TIMER14 Timerbase Mode
	TIM14->PSC	= 840-1;	// Prescaler 84,000,000Hz/840 = 10,0000 Hz (0.01ms)  (1~65536) //-1은 rule임
	TIM14->ARR	= 10000-1;	// Auto reload  0.01ms * 10000 = 100ms 
	TIM14->CR1 	&= ~(1<<4);	// DIR: Countermode = Upcounter (reset state)
	TIM14->CR1 	&= ~(3<<8);	// CKD: Clock division = 1 (reset state)
	TIM14->CR1 	&= ~(3<<5); 	// CMS(Center-aligned mode Sel): No(reset state)
    
    TIM14->EGR |= (1<<0);    // UG: Update generation    
	TIM14->DIER |= (1<<0);	// UIE: Enable Tim14 Update interrupt
    
    // Output/Compare Mode
	/**/TIM14->CCER	|= (1<<0);	// CC1E: OC3 Active(Capture/Compare 3 output enable)
	/**/TIM14->CCER	&= ~(1<<1);	// CC3P: Capture/Compare 1 output Polarity High
	/**/TIM14->CCR1	= 5000;		// CCR1 value
    
	TIM14->CCMR1 	&= ~(3<<0); 	// CC3S(CC3 channel): Output 
	TIM14->CCMR1 	&= ~(1<<3); 	// OC3PE: Output Compare 3 preload disable
	TIM14->CCMR1	|= (3<<4); // OC3M: Output Compare 3 Mode : toggle
    TIM14->CCMR1	|= (1<<7);	// OC3CE: Output compare 3 Clear enable
    
    //TIM14->DIER |= (1<<1);	// UIE: Enable Tim14 Update interrupt
	TIM14->CR1	|= (1<<7);	// ARPE: Auto-reload preload enable
    TIM14->CR1	&= ~(1<<0);	// CEN: Counter TIM14 enable
}



void EXTI15_10_IRQHandler(void)		// EXTI 15~10 인터럽트 핸들러
{
    if((EXTI->PR & 0x8000) && (sw7_flag == 1))  //  EXTI15 Interrupt Pending? //SW7눌렀을때 부저시작
    {
        //EXTI->PR |= 0x4000;     // Pending bit Clear
         sw7_flag = 0;
        TIM14->CR1	|= (1<<0);	// CEN: Counter TIM14 Enable
         EXTI->PR |= 0x4000;     // Pending bit Clear
        
        //TIM14->CR1 &= (0<<1); 

    }
    else if((EXTI->PR & 0x8000) && (sw7_flag == 0)) //sw7 한 번 더 누르면 부저멈춤
    {
        //EXTI->PR |= 0x4000; // Pending bit Clear
        sw7_flag = 1;
        TIM14->CR1	&= ~(1<<0);	// CEN: Counter TIM14 Disable  
        TIM14->CR1	&= ~(1<<0);	// CEN: Counter TIM14 Disable  
        TIM14->CR1	&= ~(1<<0);	// CEN: Counter TIM14 Disable  
        TIM14->CR1	&= ~(1<<0);	// CEN: Counter TIM14 Disable  
        TIM14->CR1	&= ~(1<<0);	// CEN: Counter TIM14 Disable  
        TIM14->CR1	&= ~(1<<0);	// CEN: Counter TIM14 Disable  
         EXTI->PR |= 0x4000;     // Pending bit Clear

    }

}

//void BEEP(void)			// Beep for 20 ms 
//{ 	GPIOF->ODR |= (1<<9);	// PF9 'H' Buzzer on
//	DelayMS(20);		// Delay 20 ms
//	GPIOF->ODR &= ~(1<<9);	// PF9 'L' Buzzer off
//}

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
    LCD_SetBackColor(RGB_YELLOW);	//글자배경색
    LCD_SetTextColor(RGB_BLACK);	//글자색
    LCD_DisplayText(0,0,"Vol:0.00V");
    LCD_DisplayText(1,0,"Temp:00.0C");
    LCD_DisplayText(2,0,"Freq:100Hz");
    //LCD_DisplayText(3,0,"■"); 소리크기 100hz당 1개

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

