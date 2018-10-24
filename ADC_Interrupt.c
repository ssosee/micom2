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
uint16_t KEY_Scan(void);

void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);
void BEEP(void);

unsigned short ADC_Value, Voltage;
char str[20];


void ADC_IRQHandler(void)
{
	ADC->SR &= ~(1<<1);	// EOC flag clear

	ADC_Value = ADC1->DR;

    sprintf(str,"%4d",ADC_Value);
    LCD_DisplayText(0,6,str);
	Voltage = ADC_Value * (3.3 * 100) / 4095;   // 3.3 : 4095 =  Volatge : ADC_Value 
                                                                // 100:  소수점아래 두자리까지 표시하기 위한 값  
	LCD_DisplayChar(1,9,Voltage/100 + 0x30);
	LCD_DisplayChar(1,11,Voltage%100/10 + 0x30);
	LCD_DisplayChar(1,12,Voltage%10 + 0x30);

 	//Starts conversion of regular channels
	ADC1->CR2 |= ADC_CR2_SWSTART; // 0x40000000 
}

int main(void)
{
	_RCC_Init();
	LCD_Init();		// LCD 구동 함수
	DelayMS(10);	// LCD구동 딜레이
 	DisplayTitle();	//LCD 초기화면구동 함수
        
        _GPIO_Init();
	_ADC_Init();
       
 	//Starts conversion of regular channels
       	ADC1->CR2 |= ADC_CR2_SWSTART; // 0x40000000 
	while(1)
	{
                GPIOG->ODR ^= 0x01;
                DelayMS(300);
          
	}
}

void _ADC_Init(void)
{   /*****************************************************************************************************************
    온도센서 측정할때는 내부센서이기 때문에 필요없다.
	
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; 	// 0x00000001  // ENABLE GPIOA CLK
	GPIOA->MODER |= GPIO_MODER_MODER1;       // 0x0000000C	// CONFIG GPIOA PIN1(PA1) TO ANALOG IN MODE
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;	// 0x00000100  // ENABLE ADC1 CLK
    //GPIOA->MODER |= 0x00000003;
    ********************************************************************************************************************/
	
    ADC->CCR &= ~0X0000001F;	// MULTI[4:0]: ADC_Mode_Independent
	ADC->CCR |= 0x00010000;		// ADCPRE: ADC_Prescaler_Div4 (ADC MAX Clock 36MHz, 84Mhz(APB2)/4 = 21MHz)
	ADC->CCR &= ~0x0000C000;	// DMA: Disable
    ADC->CCR	|= 0x00000F00;	// ADC_TwoSamplingDelay_20Cycles
    
    /**/ADC->CCR |= (1<<23); //Enable the temperature sensor    
	
    ADC1->CR1 &= ~(3<<24);		// RES[1:0]: 12bit Resolution
	ADC1->CR1 &= ~(1<<8);		// SCAN: ADC_ScanCovMode Disable
	ADC1->CR1 |= 1<<5;		// EOCIE: Interrupt enable for EOC

	ADC1->CR2 &= ~(1<<1);		// CONT: ADC_ContinuousConvMode Disable
	ADC1->CR2 &= ~(3<<28);		// EXTEN[1:0]: ADC_ExternalTrigConvEdge_None
	ADC1->CR2 &= ~(1<<11);		// ALIGN: ADC_DataAlign_Right
	ADC1->CR2 |= 1<<10;		// EOCS: The EOC bit is set at the end of each regular conversion
	ADC1->CR2 |= 1<<0;		// ADON: ADC ON

	ADC1->SQR1 &= ~0x00F00000;	// L[3:0]: ADC Regular channel sequece length = 1 conversion
    
        ADC1->SMPR2	|= 0x07 << (3*1);	// ADC1_CH1 Sample TIme_480Cycles (3*Channel_1)
 	//Channel selection, The Conversion Sequence of PIN1(ADC1_CH1) is first, Config sequence Range is possible from 0 to 17
	ADC1->SQR3 |= 0x01<<0;

	NVIC->ISER[0] |= (1<<18);	// Enable ADC global Interrupt
    
    

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
       	LCD_DisplayText(0,0,"ADC1: 0000");
      	LCD_DisplayText(1,0,"Voltage: 0.00V");

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

