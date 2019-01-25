#include "stm32f4xx.h"
#include "GLCD.h"

void _RCC_Init(void);
void _GPIO_Init(void);
void _EXTI_Init(void);

void BEEP(void); //소리함수 30ms 간격
void Coin_flag(void);
void RunMenu(void);//초기 LCD화면 함수

void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);
uint16_t KEY_Scan();

uint8_t	SW7_Flag;//인터럽트를 위해 선언한 SW0 변수
uint8_t current_floor = 1; //현재 층 값 저장 변수 (초기값은 1층으로)
uint8_t  arrive_floor = 1; //도착 층 값 저장 변수

int main(void)
{
	_RCC_Init();
	_GPIO_Init();
	_EXTI_Init();
	LCD_Init();	//LCD함수 호출
	BEEP(); //BEEP함수 호출
	DelayMS(100);

	GPIOG->ODR &= 0x11111100;		// 초기값: LED0~7 Off 
	RunMenu(); //LCD 초기화면구동 함수

        while (1)
        {
                switch(KEY_Scan()) 
                {
                    case 0xFE00: //스위치 0번 B층
                    LCD_DisplayChar(0, 3, 'B');
                    arrive_floor = 0;//도착층을 0으로 만든다.
                    break;

                    case 0xFD00:  //스위치 1번 1층
                    LCD_DisplayChar(0, 3, '1');
                    arrive_floor = 1;
                    break;

                    case 0xFB00: //스위치 2번 2층
                    LCD_DisplayChar(0, 3, '2');
                    arrive_floor = 2;
                    break;

                    case 0xF700:  //스위치 3번 3층
                    LCD_DisplayChar(0, 3, '3');
                    arrive_floor = 3;
                    break;

                    case 0xEF00:  //스위치 4번 4층
                    LCD_DisplayChar(0, 3, '4');
                    arrive_floor = 4;
                    break;

                    case 0xDF00:  //스위치 5번 5층
                    LCD_DisplayChar(0, 3, '5');
                    arrive_floor = 5;
                    break;

                    case 0xBF00:  //스위치 6번 6층
                    LCD_DisplayChar(0, 3, '6');
                    arrive_floor = 6;
                    break;
                }
                
                if(SW7_Flag)//EXTI SW7가 High에서 Low가 될 때 (Falling edge Trigger mode) LED0 toggle
                {
                    if(arrive_floor < current_floor) //도착 층이 현재 층보다 낮은 경우   
                    {
                      for(int i = 0; i <= abs(current_floor - arrive_floor); i++)
                        {
                          LCD_DisplayChar(1,(2*current_floor) + 5 -(2*i),'<'); 
                          //(2 * 현재 층) + 5(더하기 5하는 이유가 층 시작이 5번 자리이기 때문) - (2*i) (2칸씩 움직이기 때문에 2를 곱하고 마지막 i는 층 수의 차이기 때문에 거기까지 <를 움직이게 한다.)
                        DelayMS(500);
                          LCD_DisplayChar(1,(2*current_floor) + 5 -(2*i),'  ');//같은 방법으로 남아 있는 <을 지워준다. 
                        }
                        
                        LCD_DisplayChar(1, 0, 'S');
                        current_floor = arrive_floor; //현재 층에 도착 층의 값을 넣어준다.
                        LCD_DisplayChar(1, (2 * current_floor) + 5, '*'); //현재 층수 화면에 *을 넣어준다.
                        SW7_Flag = 0; // 0을 넣어준다. 동작을 멈춘다.
                        BEEP();
                        DelayMS(500);
                        BEEP();
                        DelayMS(500);
                        BEEP();
                    }
                    else if(arrive_floor > current_floor) //도착 층이  현재 층보다 높은 경우
                    {
                        for (int i =0; i <= abs(current_floor - arrive_floor); i++)
                        {
                        LCD_DisplayChar(1,(2*current_floor) + 5 +(2*i),'>'); 
                        //(2 * 현재 층) + 5(더하기 5하는 이유가 층 시작이 5번 자리이기 때문) +(2*i) (2칸씩 움직이기 때문에 2를 곱하고 마지막 i는 층 수의 차이기 때문에 거기까지 >를 움직이게 한다.)
                        DelayMS(500);
                        LCD_DisplayChar(1,(2*current_floor) + 5 +(2*i),'  ');//같은 방법으로 남아 있는 >을 지워준다. 
                        }
                        LCD_DisplayChar(1, 0, 'S');
                        current_floor = arrive_floor;
                        LCD_DisplayChar(1, (2 * current_floor) + 5, '*');
                        SW7_Flag = 0; // 0을 넣어준다. 동작을 멈춘다.
                        BEEP();
                        DelayMS(500);
                        BEEP();
                        DelayMS(500);
                        BEEP();
                    }
                    else if(arrive_floor == current_floor) //현재 층수와 도착 층수가 같을 경우
                    { 
                     SW7_Flag = 0;
                    LCD_DisplayChar(1, 0, 'S');
                    BEEP();
                    }
                }

        }
}

  

uint8_t key_flag = 0;

uint16_t KEY_Scan(void)	// input key SW0 - SW7 
{ 
	uint16_t key;
	key = GPIOH->IDR & 0xFF00;	// any key pressed ? 모든 ky 확인
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



void _GPIO_Init(void)
{
	// LED GPIO(PORT G) 설정
    RCC->AHB1ENR	|= 0x00000040; 	// RCC_AHB1ENR(bit8~0) GPIOG(bit#6) Enable							
	GPIOG->MODER 	= 0x00005555;	// GPIOG PIN0~PIN7 Output mode (0b01)						
	GPIOG->OTYPER	= 0x0000;	// GPIOG PIN0~PIN7 : Push-pull  (PIN8~PIN15) (reset state)	
 	GPIOG->OSPEEDR 	= 0x00005555;	// GPIOG PIN0~PIN7 Output speed (25MHZ Medium speed) 
    
	// SW GPIO(PORT H) 설정 
	RCC->AHB1ENR    |= 0x00000080;	// RCC_AHB1ENR(bit8~0) GPIOH(bit#7) Enable							
	GPIOH->MODER 	= 0x00000000;	// GPIOH PIN8~PIN15 Input mode (reset state)				
	GPIOH->PUPDR 	= 0x00000000;	// GPIOH PIN8~PIN15 Floating input (No Pull-up, pull-down) (reset state)

	// Buzzer GPIO(PORT F) 설정 
    RCC->AHB1ENR	|= 0x00000020; 	// RCC_AHB1ENR(bit8~0) GPIOF(bit#5) Enable							
	GPIOF->MODER 	|= 0x00040000;	// GPIOF PIN9 Output mode (0b01)						
	GPIOF->OTYPER 	&= 0xFDFF;	// GPIOF PIN9 : Push-pull  	
 	GPIOF->OSPEEDR 	|= 0x00040000;	// GPIOF PIN9 Output speed (25MHZ Medium speed) 
}	

void _EXTI_Init(void) 
{
    RCC->AHB1ENR 	|= 0x80;	// RCC_AHB1ENR GPIOH Enable
	RCC->APB2ENR 	|= 0x4000;	//Enable System Configuration Controller Clock
	
	GPIOH->MODER 	&=  0x00FF;	// GPIOH PIN8~PIN15 Input mode (reset state)				 
	
	SYSCFG->EXTICR[3] |= 0x7000; 	// EXTI10,15에 대한 소스 입력은 GPIOH로 설정 (EXTICR4) (reset value: 0x0000)	
	
	EXTI->FTSR |= 0x8000;		// Falling Trigger Enable  (EXTI15:PH15)
	//EXTI->RTSR |= 0x8000;		// Rising Trigger  Enable  (EXTI15:PH15) 
    EXTI->IMR |= 0x8000;  	// EXTI15 인터럽트 mask (Interrupt Enable)
		
	NVIC->ISER[1] 	|= ( 1 << 40 -32 ); // *Enable Interrupt EXTI15 Vector table Position 참조*
}

void EXTI15_10_IRQHandler(void)		// EXTI 15~10 인터럽트 핸들러
{
    if(EXTI->PR & 0x8000) 		// EXTI8 nterrupt Pending?
    {
        EXTI->PR |= 0x8000; 		// Pending bit Clear
	SW7_Flag = 1;
    }
    LCD_DisplayChar(1, 0, 'W');
}

void BEEP(void)			/* beep for 30 ms */
{ 	
    GPIOF->ODR |= 0x0200;	// PF9 'H' Buzzer on
	DelayMS(30);		// Delay 30 ms
	GPIOF->ODR &= ~0x0200;	// PF9 'L' Buzzer off
}

void DelayMS(unsigned short wMS)
{
    register unsigned short i;
    for (i=0; i<wMS; i++)
        DelayUS(1000);         		// 1000us => 1ms
}

void DelayUS(unsigned short wUS)
{
    volatile int Dly = (int)wUS*17;
    for(; Dly; Dly--);
}

void RunMenu(void) //초기 화면 함수
{
        LCD_Clear(RGB_YELLOW);
        LCD_SetFont(&Gulim8);		//폰트 
        LCD_SetBackColor(RGB_YELLOW);	//글자배경색
        LCD_SetTextColor(RGB_BLACK);	//글자색

        LCD_DisplayText(0,0,"FL:"); //0: 첫 번째 줄(y축), 0: 첫 번째 줄(x축), " ":(출력 문자열)
        LCD_DisplayChar(0,3,'1');
        LCD_DisplayChar(0,5,'B');
        LCD_DisplayChar(0,7,'1');
        LCD_DisplayChar(0,9,'2');
        LCD_DisplayChar(0,11,'3');
        LCD_DisplayChar(0,13,'4');
        LCD_DisplayChar(0,15,'5');
        LCD_DisplayChar(0,17,'6');

        LCD_DisplayChar(1,0,'S');
        LCD_DisplayChar(1,7,'*');
}

/******************************************************************************/
/*     RCC Set up                                                             */
/******************************************************************************/
void _RCC_Init(void)
{
// PLL (clocked by HSE) used as System clock source                    

    __IO uint32_t StartUpCounter = 0, HSEStatus = 0;

    // Enable HSE */
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
	// HCLK = SYSCLK / 1 
	RCC->CFGR |= 0x00000000;
 
	// PCLK2 = HCLK / 2 
	RCC->CFGR |= 0x00008000;	// PPRE2: APB(Advanced Peripheral Bus)(APB2) High-speed prescaler
					// 100: AHB clock divided by 2 

	// PCLK1 = HCLK / 4 
	RCC->CFGR |= 0x00001400;	// PPRE1: APB(Advanced Peripheral Bus)(APB1) Low-speed prescaler
					// 101: AHB clock divided by 4 

    	// Configure the main PLL 
	// Reset vlaue: 0x2400 3010 (PPLQ:4, PLLSR:0, PLL_M:16, PLL_N:192, PLL_P: 2(00))
	RCC->PLLCFGR |= 25;			// PLL_M(6bits): 25(0b011001): /25
	RCC->PLLCFGR |= (336 << 6);		// PLL_N(9bits): 336 : *336
	RCC->PLLCFGR |= (((2 >> 1) -1)<<16);// PLL_P(2bits): (2 >> 1) -1=0b00 :  2 : /2 
	RCC->PLLCFGR |= 0x00400000; 	// PLL_SR(1bit): 1 : HSE oscillator clock selected as PLL and PLLI2S clock
	    
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

/**************************************************************************
// 보충 설명자료
// 다음은 stm32f4xx.h에 있는 RCC관련 주요 선언문임 
#define HSE_STARTUP_TIMEOUT    ((uint16_t)0x05000)   // Time out for HSE start up 
typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

#define FLASH_BASE            ((uint32_t)0x08000000) // FLASH(up to 1 MB) base address in the alias region                          
#define CCMDATARAM_BASE       ((uint32_t)0x10000000) // CCM(core coupled memory) data RAM(64 KB) base address in the alias region   
#define SRAM1_BASE            ((uint32_t)0x20000000) // SRAM1(112 KB) base address in the alias region                              

#if defined(STM32F40_41xxx) 
#define SRAM2_BASE            ((uint32_t)0x2001C000) // SRAM2(16 KB) base address in the alias region                               
#define SRAM3_BASE            ((uint32_t)0x20020000) // SRAM3(64 KB) base address in the alias region                               
#endif   

#define PERIPH_BASE           ((uint32_t)0x40000000) // Peripheral base address in the alias region                                 
#define BKPSRAM_BASE          ((uint32_t)0x40024000) // Backup SRAM(4 KB) base address in the alias region                          

// Peripheral memory map  
#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000)
#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x10000000)

// AHB1 peripherals  
#define GPIOA_BASE            (AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASE            (AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASE            (AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASE            (AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASE            (AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASE            (AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASE            (AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASE            (AHB1PERIPH_BASE + 0x1C00)
#define GPIOI_BASE            (AHB1PERIPH_BASE + 0x2000)
#define GPIOJ_BASE            (AHB1PERIPH_BASE + 0x2400)
#define GPIOK_BASE            (AHB1PERIPH_BASE + 0x2800)
#define CRC_BASE              (AHB1PERIPH_BASE + 0x3000)
#define RCC_BASE              (AHB1PERIPH_BASE + 0x3800)
#define FLASH_R_BASE          (AHB1PERIPH_BASE + 0x3C00)
#define SYSCFG_BASE           (APB2PERIPH_BASE + 0x3800)
#define EXTI_BASE             (APB2PERIPH_BASE + 0x3C00)

typedef struct
{
  __IO uint32_t CR;            // RCC clock control register, Address offset: 0x00  
  __IO uint32_t PLLCFGR;       // RCC PLL configuration register, Address offset: 0x04  
  __IO uint32_t CFGR;          // RCC clock configuration register, Address offset: 0x08  
  __IO uint32_t CIR;           // RCC clock interrupt register, Address offset: 0x0C  
  __IO uint32_t AHB1RSTR;      // RCC AHB1 peripheral reset register, Address offset: 0x10  
  __IO uint32_t AHB2RSTR;      // RCC AHB2 peripheral reset register, Address offset: 0x14  
  __IO uint32_t AHB3RSTR;      // RCC AHB3 peripheral reset register, Address offset: 0x18  
  uint32_t      RESERVED0;     // Reserved, 0x1C                                                                    
  __IO uint32_t APB1RSTR;      // RCC APB1 peripheral reset register, Address offset: 0x20  
  __IO uint32_t APB2RSTR;      // RCC APB2 peripheral reset register, Address offset: 0x24  
  uint32_t      RESERVED1[2];  // Reserved, 0x28-0x2C                                                                
  __IO uint32_t AHB1ENR;       // RCC AHB1 peripheral clock register, Address offset: 0x30  
  __IO uint32_t AHB2ENR;       // RCC AHB2 peripheral clock register, Address offset: 0x34  
  __IO uint32_t AHB3ENR;       // RCC AHB3 peripheral clock register, Address offset: 0x38  
  uint32_t      RESERVED2;     // Reserved, 0x3C                                                                     
  __IO uint32_t APB1ENR;       // RCC APB1 peripheral clock enable register, Address offset: 0x40  
  __IO uint32_t APB2ENR;       // RCC APB2 peripheral clock enable register, Address offset: 0x44  
  uint32_t      RESERVED3[2];  // Reserved, 0x48-0x4C                                                                
  __IO uint32_t AHB1LPENR;     // RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50  
  __IO uint32_t AHB2LPENR;     // RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54  
  __IO uint32_t AHB3LPENR;     // RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58  
  uint32_t      RESERVED4;     // Reserved, 0x5C                                                                     
  __IO uint32_t APB1LPENR;     // RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60  
  __IO uint32_t APB2LPENR;     // RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64  
  uint32_t      RESERVED5[2];  // Reserved, 0x68-0x6C                                                                
  __IO uint32_t BDCR;          // RCC Backup domain control register, Address offset: 0x70  
  __IO uint32_t CSR;           // RCC clock control & status register, Address offset: 0x74  
  uint32_t      RESERVED6[2];  // Reserved, 0x78-0x7C                                                                
  __IO uint32_t SSCGR;         // RCC spread spectrum clock generation register, Address offset: 0x80  
  __IO uint32_t PLLI2SCFGR;    // RCC PLLI2S configuration register, Address offset: 0x84  
  __IO uint32_t PLLSAICFGR;    // RCC PLLSAI configuration register, Address offset: 0x88  
  __IO uint32_t DCKCFGR;       // RCC Dedicated Clocks configuration register, Address offset: 0x8C  
} RCC_TypeDef;
	

typedef struct
{
  __IO uint32_t ACR;      // FLASH access control register,   Address offset: 0x00  
  __IO uint32_t KEYR;     // FLASH key register,              Address offset: 0x04  
  __IO uint32_t OPTKEYR;  // FLASH option key register,       Address offset: 0x08  
  __IO uint32_t SR;       // FLASH status register,           Address offset: 0x0C  
  __IO uint32_t CR;       // FLASH control register,          Address offset: 0x10  
  __IO uint32_t OPTCR;    // FLASH option control register ,  Address offset: 0x14  
  __IO uint32_t OPTCR1;   // FLASH option control register 1, Address offset: 0x18  
} FLASH_TypeDef;

typedef struct
{
  __IO uint32_t MODER;    // GPIO port mode register,               Address offset: 0x00       
  __IO uint32_t OTYPER;   // GPIO port output type register,        Address offset: 0x04       
  __IO uint32_t OSPEEDR;  // GPIO port output speed register,       Address offset: 0x08       
  __IO uint32_t PUPDR;    // GPIO port pull-up/pull-down register,  Address offset: 0x0C       
  __IO uint32_t IDR;      // GPIO port input data register,         Address offset: 0x10       
  __IO uint32_t ODR;      // GPIO port output data register,        Address offset: 0x14       
  __IO uint16_t BSRRL;    // GPIO port bit set/reset low register,  Address offset: 0x18       
  __IO uint16_t BSRRH;    // GPIO port bit set/reset high register, Address offset: 0x1A       
  __IO uint32_t LCKR;     // GPIO port configuration lock register, Address offset: 0x1C       
  __IO uint32_t AFR[2];   // GPIO alternate function registers,     Address offset: 0x20-0x24  
} GPIO_TypeDef;

typedef struct
{
  __IO uint32_t IMR;    // EXTI Interrupt mask register, Address offset: 0x00 
  __IO uint32_t EMR;    // EXTI Event mask register, Address offset: 0x04 
  __IO uint32_t RTSR;   // EXTI Rising trigger selection register,  Address offset: 0x08
  __IO uint32_t FTSR;   // EXTI Falling trigger selection register, Address offset: 0x0C
  __IO uint32_t SWIER;  // EXTI Software interrupt event register,  Address offset: 0x10 
  __IO uint32_t PR;     // EXTI Pending register, Address offset: 0x14 
} EXTI_TypeDef;

typedef struct
{
  __IO uint32_t MEMRMP;       // SYSCFG memory remap register, Address offset: 0x00 
  __IO uint32_t PMC;          // SYSCFG peripheral mode configuration register, Address offset: 0x04
  __IO uint32_t EXTICR[4];    // SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14 
  uint32_t      RESERVED[2];  // Reserved, 0x18-0x1C     
  __IO uint32_t CMPCR;        // SYSCFG Compensation cell control register,Address offset: 0x20

} SYSCFG_TypeDef;

#define GPIOA 	((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB	((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC   ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD   ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE  	((GPIO_TypeDef *) GPIOE_BASE)
#define GPIOF   ((GPIO_TypeDef *) GPIOF_BASE)
#define GPIOG   ((GPIO_TypeDef *) GPIOG_BASE)
#define GPIOH   ((GPIO_TypeDef *) GPIOH_BASE)
#define GPIOI   ((GPIO_TypeDef *) GPIOI_BASE)
#define GPIOJ   ((GPIO_TypeDef *) GPIOJ_BASE)
#define GPIOK   ((GPIO_TypeDef *) GPIOK_BASE)

#define CRC             ((CRC_TypeDef *) CRC_BASE)
#define RCC             ((RCC_TypeDef *) RCC_BASE)
#define FLASH           ((FLASH_TypeDef *) FLASH_R_BASE)

#define SYSCFG          ((SYSCFG_TypeDef *) SYSCFG_BASE)
#define EXTI            ((EXTI_TypeDef *) EXTI_BASE)

#define FLASH_ACR_PRFTEN             ((uint32_t)0x00000100)
#define FLASH_ACR_ICEN               ((uint32_t)0x00000200)
#define FLASH_ACR_DCEN               ((uint32_t)0x00000400)
#define FLASH_ACR_ICRST              ((uint32_t)0x00000800)
#define FLASH_ACR_DCRST              ((uint32_t)0x00001000)
#define FLASH_ACR_BYTE0_ADDRESS      ((uint32_t)0x40023C00)
#define FLASH_ACR_BYTE2_ADDRESS      ((uint32_t)0x40023C03)

#define FLASH_ACR_LATENCY_5WS        ((uint32_t)0x00000005)

typedef struct {
  __IO uint32_t ISER[8];  // Offset: 0x000 Interrupt Set Enable Register    
       uint32_t RESERVED0[24];                                   
  __IO uint32_t ICER[8];  // Offset: 0x080 Interrupt Clear Enable Register  
       uint32_t RSERVED1[24];                                    
  __IO uint32_t ISPR[8];  // Offset: 0x100 Interrupt Set Pending Register   
       uint32_t RESERVED2[24];                                   
  __IO uint32_t ICPR[8];  // Offset: 0x180 Interrupt Clear Pending Register
       uint32_t RESERVED3[24];                                   
  __IO uint32_t IABR[8];  // Offset: 0x200 Interrupt Active bit Register      
       uint32_t RESERVED4[56];                                   
  __IO uint8_t  IP[240];  // Offset: 0x300 Interrupt Priority Register (8Bit) 
       uint32_t RESERVED5[644];                                  
  __O  uint32_t STIR;  // Offset: 0xE00 Software Trigger Interrupt Register    
}  NVIC_Type; 
     
// Memory mapping of Cortex-M4 Hardware 
#define SCS_BASE     (0xE000E000)    // System Control Space Base Address 
#define NVIC_BASE   (SCS_BASE +  0x0100)  // NVIC Base Address  
#define NVIC        ((NVIC_Type *)  NVIC_BASE) // NVIC configuration struct                                           

*/ 