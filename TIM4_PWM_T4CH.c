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
uint16_t KEY_Scan(void);

void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);
void BEEP(void);

int main(void)
{
	_RCC_Init();
	LCD_Init();			// LCD 구동 함수
	DelayMS(10);			// LCD구동 딜레이
    
	DisplayTitle();		//LCD 초기화면구동 함수
	GPIOG->ODR &= ~0x00FF;		// 초기값: LED0~7 Off
	GPIOB->ODR &= ~0x0200;
	
	GPIOG->ODR |= 0x01; 		//GPIO Example(1) LED0 ON
	_GPIO_Init();
	DC_MOTOR_driver();
    
	while(1)
	{
		/* SW0~SW3 Duty 변경*/
		switch(KEY_Scan())
		{
			case SW0_PUSH : 	//SW0
				TIM4->CCR3 = 10;	  
				LCD_DisplayText(1,6,"10");
 			break;

			case SW1_PUSH : 	//SW1
				TIM4->CCR3 = 35;	  
				LCD_DisplayText(1,6,"35");
			break;

                        case SW2_PUSH  : 	//SW2
				TIM4->CCR3 = 70;	  
				LCD_DisplayText(1,6,"70");
			break;

                        case SW3_PUSH  : 	//SW3
				TIM4->CCR3 = 90;	  
				LCD_DisplayText(1,6,"90");
			break;

                }      
      
		/*Navi.SW로  DIR 변경*/
		if((GPIOI->IDR & 0x0200) == 0)	//NAVI.SW_LEFT (PI_9)
		{
			GPIOB->ODR |= 0x0200;	//DIR = CW(HIGH) (PB_9)
			LCD_DisplayText(2,5," CW");	 	   
		}
		if((GPIOI->IDR & 0x0100) == 0)	//NAVI.SW_RIGHT (PI_8)	
		{
			GPIOB->ODR &= ~0x0200;	//DIR = CCW(LOW)(PB_9)
			LCD_DisplayText(2,5,"CCW");
		}
	}
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

void DC_MOTOR_driver(void)
{   
// Clock Enable : GPIOB & TIMER4
	RCC->AHB1ENR	|= 0x00000002;	// RCC_AHB1ENR GPIOB Enable
	RCC->APB1ENR 	|= 0x00000004;	// RCC_APB1ENR TIMER4 Enable 
    						
// PB8을 출력설정하고 Alternate function(TIM4_CH3)으로 사용 선언
	GPIOB->MODER 	|= 0x00020000;	// GPIOB PIN8 Output Alternate function mode					
	GPIOB->OSPEEDR 	|= 0x00030000;	// GPIOB PIN8 Output speed (100MHz High speed)
	GPIOB->OTYPER	= 0x00000000;	// GPIOB PIN8 Output type push-pull (reset state)
	GPIOB->PUPDR	|= 0x00010000;	// GPIOB PIN8 Pull-up
 	GPIOB->AFR[1]	|= 0x00000002;	// AFRH(AFR[1]): Connect PA0 to AF2(TIM3..5)
					// PB8 ==> TIM4_CH3
    
// TIM4 Channel 3 : PWM 1 mode
	// Assign 'PWM Pulse Period'
	TIM4->PSC	= 8400-1;	// Prescaler 84,000,000Hz/8400 = 10,000 Hz(0.1ms)  (1~65536)
	TIM4->ARR	= 100-1;	// Auto reload  (0.1ms * 100 = 10ms : PWM Period)

	// CR1 : Up counting
	TIM4->CR1 	&= ~(1<<4);	// DIR: Countermode = Upcounter (reset state)
	TIM4->CR1 	&= ~(3<<8);	// CKD: Clock division = 1 (reset state)
	TIM4->CR1 	&= ~(3<<5); 	// CMS(Center-aligned mode Sel): No(reset state)
    	
	// Define the corresponding pin by 'Output'  
	TIM4->CCER	|= (1<<8);	// CC3E: OC3 Active(Capture/Compare 3 output enable)
	TIM4->CCER	&= ~(1<<9);	// CC3P: Capture/Compare 1 output Polarity High

	// Duty Ratio 
	TIM4->CCR3	= 10;		// CCR3 value

	// 'Mode' Selection : Output mode, PWM 1
	TIM4->CCMR2 	&= ~(3<<0); 	// CC3S(CC3 channel): Output 
	TIM4->CCMR2 	&= ~(1<<3); 	// OC3PE: Output Compare 3 preload disable
	TIM4->CCMR2	|= (6<<4);	// OC3M: Output compare 3 mode: PWM 1 mode
	TIM4->CCMR2	|= (1<<7);	// OC3CE: Output compare 3 Clear enable
	
	//Counter TIM5 enable
	TIM4->CR1	|= (1<<7);	// ARPE: Auto-reload preload enable
	TIM4->CR1	|= (1<<0);	// CEN: Counter TIM4 enable
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
	LCD_Clear(RGB_WHITE);
	LCD_SetFont(&Gulim10);		//폰트 
	LCD_SetBackColor(RGB_GREEN);	//글자배경색
	LCD_SetTextColor(RGB_BLACK);	//글자색
       	LCD_DisplayText(0,0,"TIM4 PWM MODE");

	LCD_SetBackColor(RGB_YELLOW);	//글자배경색
	LCD_DisplayText(1,0,"DUTY:  5%");
	LCD_DisplayText(2,0,"DIR :  CW");
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

