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

void DC_MOTOR_driver(void);
void TIMER1_Init(void);
uint16_t KEY_Scan(void);
void _EXTI_Init(void);
void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);

uint16_t Speed_Level = 0;
uint16_t SW7_Flag = 0;


int main(void)
{
	_RCC_Init();
	LCD_Init();			// LCD 구동 함수
	DelayMS(10);			// LCD구동 딜레이
 	DisplayTitle();		//LCD 초기화면구동 함수
	_GPIO_Init();
    _EXTI_Init(); //인터럽트 실행
	GPIOG->ODR &= ~0x00FF;	// 초기값: LED0~7 Off
    TIMER1_Init(); //타이머1 초기화함수
    DC_MOTOR_driver(); //DC모터(타이머14) 초기화함수
	while(1)
	{
     switch(KEY_Scan())
		{
          //CCR1의 값을 바꿈으로써 Duty ratio(DR) 값을 변경한다.
            case SW0_PUSH : 	//SW0
            TIM14->CCR1 =0;
            GPIOG->ODR &= ~0x00FF; //LED ALL OFF
            GPIOG->ODR |= 0x0001; //LED0 on
            LCD_SetTextColor(RGB_RED); //글자색 빨강
            LCD_DisplayText(4,14,"0 ");
            break;

            case SW1_PUSH : 	//SW1
            TIM14->CCR1= 10;
            GPIOG->ODR &= ~0x00FF;
            GPIOG->ODR |= 0x0002; //LED1 on
            LCD_SetTextColor(RGB_RED);
            LCD_DisplayText(4,14,"10");
            break;

            case SW2_PUSH  : 	//SW2
            TIM14->CCR1 = 20;	
            GPIOG->ODR &= ~0x00FF;
            GPIOG->ODR |= 0x0004; //LED2 on
            LCD_SetTextColor(RGB_RED);
            LCD_DisplayText(4,14,"20");
            break;

            case SW3_PUSH  : 	//SW3
            TIM14->CCR1 = 30;
            GPIOG->ODR &= ~0x00FF;
            GPIOG->ODR |= 0x0008;//LED3 on
            LCD_SetTextColor(RGB_RED);
            LCD_DisplayText(4,14,"30");
            break;

            case SW4_PUSH  : 	//SW4
            TIM14->CCR1 = 40;
            GPIOG->ODR &= ~0x00FF;
            GPIOG->ODR |= 0x00010;//LED4 on
            LCD_SetTextColor(RGB_RED);
            LCD_DisplayText(4,14,"40");
            break;

            case SW5_PUSH  : 	//SW5
            TIM14->CCR1 = 50;
            GPIOG->ODR &= ~0x00FF;
            GPIOG->ODR |= 0x00020;//LED5 on
            LCD_SetTextColor(RGB_RED);
            LCD_DisplayText(4,14,"50");
            break;
          }      
    }
}

void TIMER1_Init(void)
{   
    // Clock Enable : GPIOE & TIMER1
	/**/RCC->AHB1ENR	|= 0x00000010;	// RCC_AHB1ENR GPIOE Enable
    						
	// PE13을 출력설정하고 Alternate function(TIM1_CH3)으로 사용 선언
    /**/GPIOE->MODER 	|= 0x08000000;// GPIOE PIN13 Output Alternate function mode					
	/**/GPIOE->OSPEEDR |= 0x0C000000;	// GPIOE PIN0 Output speed (100MHz High speed)
	/**/GPIOE->OTYPER	&= 0x00;	// GPIOA PIN13 Output type push-pull (reset state)
	/**/GPIOE->PUPDR	|= 0x04000000;	// GPIOE PIN13 Pull-up
    /**/GPIOE->AFR[1]	|= 0x00100000;// AFRL(AFR[1]): Connect PE13 to AF1(TIM1)
	// PE13 ==> TIM1_CH3
    
    // Timerbase Mode
	//RCC->APB1ENR	|= 0x04;// RCC_APB1ENR TIMER4 Enable
    /**/RCC->APB2ENR 	|= 0x00000001;	// RCC_APB2ENR TIMER1 Enable 
	/**/NVIC->ISER[0] |= (1<<27);// Enable Timer1_CC Caputre Compare Interrupt on NVIC

	TIM1->PSC	= 42000-1;	// Prescaler 84,000,000Hz/42000 = 2000Hz(0.5ms)  (1~65536) 168
	TIM1->ARR	= 1000-1;	// Auto reload  (0.5ms * 1000= 0.5s : PWM Period) 0.25
	TIM1->CR1 &= ~(1<<4);	// DIR: Countermode = Upcounter (reset state)
	TIM1->CR1 &= ~(3<<8);	// CKD: Clock division = 1 (reset state)
	TIM1->CR1 &= ~(3<<5); 	// CMS(Center-aligned mode Sel): No(reset state)

	TIM1->EGR |= (1<<0);    // UG: Update generation    
	TIM1->DIER |= (1<<0);	// UIE: Enable Tim1 Update interrupt
	
    // Output/Compare Mode
	TIM1->CCER |= (1<<8);	// CC3E: OC3 Active 
	TIM1->CCER &= ~(1<<9);	// CC3P: OCPolarity_High
	TIM1->CCR3 = 500;	// TIM1_Pulse

	TIM1->EGR |= (1<<3);    // CC1G: Capture/Compare 1 event generation    

	/**/TIM1->CCMR2 &= ~(3<<0); // CC3S(CC3 channel): Output 
	/**/TIM1->CCMR2 &= ~(1<<3); // OC3PE: Output Compare 3 preload disable
	/**/TIM1->CCMR2 |= (3<<4);	// OC3M: Output Compare 3 Mode : toggle

	/**/TIM1->CR1 &= ~(1<<7);	// ARPE: Auto reload preload disable
	TIM1->DIER |= (1<<3);	// CC3IE: Enable the Tim1 CC3 interrupt
	TIM1->CR1 |= (1<<0);	// CEN: Enable the Tim1 Counter
}
	
void DC_MOTOR_driver(void)
{   
	/**/RCC->AHB1ENR	|= 0x00000020;	// RCC_AHB1ENR GPIOF Enable
	/**/RCC->APB1ENR 	|= 0x00000100;	// RCC_APB1ENR TIMER14 Enable 
    						
	// PF9을 출력설정하고 Alternate function(TIM14_CH1)으로 사용 선언
	/**/GPIOF->MODER 	|= 0x00080000;	// GPIOF PIN9 Output Alternate function mode					
	/**/GPIOF->OSPEEDR 	|= 0x000C0000;	// GPIOF PIN9 Output speed (100MHz High speed)
	GPIOF->OTYPER	= 0x00000000;	// GPIOF PIN9 Output type push-pull (reset state)
	/**/GPIOF->PUPDR	|= 0x00070000;	// GPIOF PIN9 Pull-up  
	/**/GPIOF->AFR[1]	|= 0x00000090;	// AFRH: Connect TIM14 pins(PB9) to AF2(TIM14)
					// PF9 ==> TIM14_CH3
    
	// TIM14 Channel 1 : PWM 1 mode
	TIM14->PSC	= 42-1;	// Prescaler 84,000,000Hz/42= 200000Hz(0.005ms)  (1~65536)
	TIM14->ARR	= 100-1;	// Auto reload  (0.005ms * 100 = 0.5ms : PWM Period) -> 500micro second
	TIM14->CR1 	&= ~(1<<4);	// DIR: Countermode = Upcounter (reset state)
	TIM14->CR1 	&= ~(3<<8);	// CKD: Clock division = 1 (reset state)
	TIM14->CR1 	&= ~(3<<5); 	// CMS(Center-aligned mode Sel): No(reset state)
    
	/**/TIM14->CCER	|= (1<<0);	// CC1E: OC3 Active(Capture/Compare 3 output enable)
	/**/TIM14->CCER	&= ~(1<<1);	// CC3P: Capture/Compare 1 output Polarity High
	/**/TIM14->CCR1	= 0;		// CCR1 value

	TIM14->CCMR1 	&= ~(3<<0); 	// CC3S(CC3 channel): Output 
	TIM14->CCMR1 	&= ~(1<<3); 	// OC3PE: Output Compare 3 preload disable
	TIM14->CCMR1	|= (6<<4);	// OC3M: Output compare 3 mode: PWM 1 mode
	TIM14->CCMR1	|= (1<<7);	// OC3CE: Output compare 3 Clear enable

	TIM14->CR1	|= (1<<7);	// ARPE: Auto-reload preload enable
	TIM14->CR1	|= (1<<0);	// CEN: Counter TIM14 enable
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

	// Buzzer GPIO(PORT F) 설정 
    /*RCC->AHB1ENR    |= 0x00000020; 	// RCC_AHB1ENR(bit8~0) GPIOF(bit#5) Enable							
	GPIOF->MODER    |= 0x00040000;	// GPIOF PIN9 Output mode (0b01)						
	GPIOF->OTYPER   &= 0xFDFF;	// GPIOF PIN9 : Push-pull  	
 	GPIOF->OSPEEDR  |= 0x00040000;	// GPIOF PIN9 Output speed (25MHZ Medium speed) 
	*/
	//NAVI.SW(PORT I) 설정
	RCC->AHB1ENR 	|= 0x00000100;	// RCC_AHB1ENR GPIOI Enable
	GPIOI->MODER 	= 0x00000000;	// GPIOI PIN8~PIN15 Input mode (reset state)
	GPIOI->PUPDR    = 0x00000000;	// GPIOI PIN8~PIN15 Floating input (No Pull-up, pull-down) (reset state)
	
	// DC_MOTOR DIR
	/*RCC->AHB1ENR	|= 0x00000002; 	// RCC_AHB1ENR GPIOB Enable							
	GPIOB->MODER 	= 0x00040000;	// GPIOB PIN9 Output mode						
	GPIOB->OSPEEDR  = 0x00040000;	// GPIOB PIN9 Output speed (25MHZ Medium speed) */
}

void TIM1_CC_IRQHandler(void)      
{
    if((TIM1->SR & 0x08) != RESET) //CC3I compare counter 3 interupt
    {
        TIM1->SR &= ~0x08; //CC3I compare counter 3 interupt clear
        GPIOG->ODR ^= 0x80; //LED7을 토글 시켜 점멸한다. 
    }
}


void _EXTI_Init(void)
{
    RCC->AHB1ENR 	|= 0x80;	// RCC_AHB1ENR GPIOH Enable
	RCC->APB2ENR 	|= 0x4000;	//Enable System Configuration Controller Clock
	
	GPIOH->MODER 	&=  0x00FF;	// GPIOH PIN8~PIN15 Input mode (reset state)				 
	
    SYSCFG->EXTICR[3] |= 0x7000;  //EXTI15 대한 소스 입력은 GPIOH로 설정  
	
    EXTI->FTSR |= 0x8000;		// Falling Trigger Enable  (EXTI15:PE15)
    EXTI->IMR |= 0x8000;  	// EXTI15 인터럽트 mask (Interrupt Enable)
		
	//NVIC->ISER[0] |= ( 1 << 23 ); // Enable Interrupt EXTI8,9 Vector table Position 참조
    NVIC->ISER[1] |= ( 1 << (40 - 32) ); // Enable Interrupt EXTI15 Vector table Position 참조
}

void EXTI15_10_IRQHandler(void)		// EXTI 15~10 인터럽트 핸들러
{
    if(EXTI->PR & 0x8000) 		// SW7 눌렸을 때 EXTI15 nterrupt Pending?
    {
        EXTI->PR |= 0x8000; 		// Pending bit Clear
        Speed_Level++; 
       if(Speed_Level == 1)
        {
          TIM1->CCR3 = 1000; //Duty ratio 50%를 유지해줘야한다. DR = CCR3/ARR *100
          TIM1->ARR  = 2000-1; //1초
        }
        else if(Speed_Level == 2)
        {
          TIM1->CCR3 = 1500;
          TIM1->ARR  = 3000-1; //1.5초  
        }
         else if(Speed_Level == 3)
        {
          TIM1->CCR3 = 2000;
          TIM1->ARR  = 4000-1; //2초 
        }
         else if(Speed_Level == 4)
        {
          TIM1->CCR3 = 2500;
          TIM1->ARR  = 5000-1; //2.5초 
        }
         else if(Speed_Level == 5)
        {
          TIM1->CCR3 = 3000;
          TIM1->ARR  = 6000-1;//3초 
        }
         else if(Speed_Level == 6)
        {
          TIM1->CCR3 = 3500;
          TIM1->ARR  = 7000-1;//3.5초 
        }
         else if(Speed_Level == 7)
        {
          TIM1->CCR3 = 4000;
          TIM1->ARR  = 8000-1;//4초
        }
         else if(Speed_Level == 8)
        {
          TIM1->CCR3 = 4500;
          TIM1->ARR  = 9000-1;//4.5초
        }
         else if(Speed_Level == 9)
        {
          TIM1->CCR3 = 5000;
          TIM1->ARR  = 10000-1;//5초
        }
        else if(Speed_Level == 10) //Speed Level이 10일때
        {
           Speed_Level = 0; //Speed Level을 0으로
           TIM1->CCR3 = 500; //초기 타이머1 CCR3 설정 값
           TIM1->ARR  = 1000-1;    // 0.5초
        }
    }
    LCD_SetTextColor(RGB_RED); //글자색 빨강
    LCD_DisplayChar(2, 13, Speed_Level+0x30);   //Speed level의 값을 디스플레이 해준다.
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
	LCD_SetBackColor(RGB_BLACK);	//글자배경색
	LCD_SetTextColor(RGB_WHITE);	//글자색
    LCD_DisplayText(0,0,"Motor Control");

    
	LCD_SetBackColor(RGB_YELLOW);	//글자배경색
	LCD_SetTextColor(RGB_BLUE);
    LCD_DisplayText(1,0,"<Step Motor>");
	LCD_DisplayText(3,0,"<DC Motor>");
    
    LCD_SetTextColor(RGB_BLACK);
    LCD_DisplayText(2,0,"*Speed Level:");
    LCD_DisplayText(4,0,"*Torque Level:");
    LCD_SetTextColor(RGB_RED); //글자색 빨강
    LCD_DisplayChar(4,14,'0');
    LCD_DisplayChar(2, 13,'0');
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

