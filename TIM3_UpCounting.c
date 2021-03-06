#include "stm32f4xx.h"
#include "GLCD.h"

void _RCC_Init(void);
void _GPIO_Init(void);
void _EXTI_Init(void);
uint16_t KEY_Scan(void);
void TIMER3_Init(void);
void TIMER5_Init(void);
void RunMenu(void);

void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);
void BEEP(void);
void Watch_System(void); //시계함수 선언
void StopWatch_System(void); //stop워치 함수 선언
void AM_PM(int Day); //AM PM 함수 선언

uint8_t	SW0_Flag, SW1_Flag, SW6_Flag, SW7_Flag;
uint8_t   S100M = 0, S1 = 5, S10 = 5, M1 = 9, M10 = 5, H1 = 1, H10 = 1; //Watch_System 함수의 변수
uint8_t   s100M = 0, s1 = 0, s10 = 0,  m1 = 0, m10 = 0; //StopWatch_System 함수의 변수
uint8_t Day = 1; //am과 pm을 구분해주는 변수
uint8_t stop = 0; //Watch_System의 멈춤을 위한 변수
uint8_t  stop2 = 1;//StopWatch_System의 멈춤을 위한 변수(SW7과 SW6번 구분)
uint8_t toggle = 0; //StopWatch_System의 멈춤을 위한 변수(SW6번 안에서 토글 구분)

int main(void)
{
	_RCC_Init();
	_GPIO_Init();
	_EXTI_Init();
	TIMER3_Init();
    TIMER5_Init();
	LCD_Init();	
	//BEEP();
	DelayMS(100);

	
    GPIOG->ODR |= 0x01; //LED0 on
	RunMenu();//LCD 초기화면구동 함수
 
	while(1)
	{
      
	}
}


void TIMER3_Init(void) //★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★
{
	RCC->APB1ENR |= 0x02;	//RCC_APB1ENR TIMER3 Enable
    
	NVIC->ISER[0] |= ( 1 << 29 ); // Enable Timer3 global Interrupt
    //분주 840, period 100ms
	TIM3->PSC = 840-1;	// Prescaler 84,000,000Hz/840 = 10,0000 Hz (0.01ms)  (1~65536) //-1은 rule임
	TIM3->ARR = 10000-1;	// Auto reload  0.01ms * 10000 = 100ms 
	TIM3->CR1 &= ~(1<<4);	// Upcounter(reset state)
	TIM3->CR1 &= ~(3<<8); 	// CKD(Clock division)=1(reset state)
	TIM3->CR1 &= ~(3<<5); 	// CMS(Center-aligned mode Sel): No(reset state)  
		 //= 0xff9f;
        //바로위 TIM3 3개는 없어도 된다.  교수님이 설명을 위해 추가한 것
        //0을 넣고 싶을 땐 &연산자
        //1을 넣고 싶을 땐 |연산자
        //인터럽트 벡터 테이블 항상 참고

	TIM3->EGR |=(1<<0);	//Update generation    

	TIM3->DIER |= (1<<0);	//Enable the Tim3 Update interrupt
	TIM3->CR1 |= (1<<0);	//Enable the Tim3 Counter    
}//★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★

void TIMER5_Init(void) //★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★
{
	RCC->APB1ENR |= 0x08;	//RCC_APB1ENR TIMER5 Enable
    
	NVIC->ISER[1] |= ( 1 << (50 - 32) ); // Enable Timer5 global Interrupt (번호가 50번이고 프로세서가32비트이므로 -32를 해준다)
    //분주 840, period 100ms
	TIM5->PSC = 840-1;	// Prescaler 84,000,000Hz/840 = 10,0000 Hz (0.01ms)  (1~65536) //-1은 rule임
	TIM5->ARR = 10000;	// Auto reload  0.01ms * 10000 = 100ms  //다운카운팅이라서 -1은 하지 않는다.
	TIM5->CR1 &=  (1<<4);	// Down - counter(reset state)
	TIM5->CR1 &= ~(3<<8); 	// CKD(Clock division)=1(reset state)
	TIM5->CR1 &= ~(3<<5); 	// CMS(Center-aligned mode Sel): No(reset state)  

	TIM5->EGR |=(1<<0);	//Update generation    

	TIM5->DIER |= (1<<0);	//Enable the Tim5 Update interrupt
	TIM5->CR1 |= (1<<0);	//Enable the Tim5 Counter    
}//★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★

void _GPIO_Init(void)
{
	// LED GPIO(PORT G) 설정
    RCC->AHB1ENR	|= 0x00000040; 	// RCC_AHB1ENR(bit8~0) GPIOG(bit#6) Enable							
	GPIOG->MODER 	= 0x00005555;	// GPIOG PIN0~PIN7 Output mode (0b01)						
	GPIOG->OTYPER	= 0x0000;	// GPIOG PIN0~PIN7 : Push-pull  (PIN8~PIN15) (reset state)	
 	GPIOG->OSPEEDR = 0x00005555;	// GPIOG PIN0~PIN7 Output speed (25MHZ Medium speed) 
    
	// SW GPIO(PORT H) 설정 
	RCC->AHB1ENR   |= 0x00000080;	// RCC_AHB1ENR(bit8~0) GPIOH(bit#7) Enable							
	GPIOH->MODER 	= 0x00000000;	// GPIOH PIN8~PIN15 Input mode (reset state)				
	GPIOH->PUPDR = 0x00000000;	// GPIOH PIN8~PIN15 Floating input (No Pull-up, pull-down) (reset state)

	// Buzzer GPIO(PORT F) 설정 
    RCC->AHB1ENR	|= 0x00000020; 	// RCC_AHB1ENR(bit8~0) GPIOF(bit#5) Enable							
	GPIOF->MODER 	|= 0x00040000;	// GPIOF PIN9 Output mode (0b01)						
	GPIOF->OTYPER &= 0xFDFF;	// GPIOF PIN9 : Push-pull  	
 	GPIOF->OSPEEDR |= 0x00040000;	// GPIOF PIN9 Output speed (25MHZ Medium speed) 
}	

void _EXTI_Init(void)
{
    RCC->AHB1ENR 	|= 0x80;	// RCC_AHB1ENR GPIOH Enable
	RCC->APB2ENR 	|= 0x4000;	//Enable System Configuration Controller Clock
	
	GPIOH->MODER 	&=  0x00FF;	// GPIOH PIN8~PIN15 Input mode (reset state)				 
	
	SYSCFG->EXTICR[2] |= 0x0077; 	// EXTI8,9에 대한 소스 입력은 GPIOH로 설정 (EXTICR3) (reset value: 0x0000)	
	//인터럽트 표 참고 77이 무슨 의미인지?
    SYSCFG->EXTICR[3] |= 0x7700;  //EXTI14,15 대한 소스 입력은 GPIOH로 설정  
	
    EXTI->FTSR |= 0x00C300;		// Falling Trigger Enable  (EXTI8:PH8)
    //EXTI->RTSR |= 0x004000;		// Rising Trigger  Enable  (EXTI14:PH14)
    EXTI->IMR |= 0xC300;  	// EXTI8,9,14,15 인터럽트 mask (Interrupt Enable)
		
	NVIC->ISER[0] |= ( 1 << 23 ); // Enable Interrupt EXTI8,9 Vector table Position 참조
    NVIC->ISER[1] |= ( 1 << (40 - 32) ); // Enable Interrupt EXTI14,15 Vector table Position 참조
}

void TIM3_IRQHandler(void)  // 100ms Interrupt
{
	TIM3->SR &= ~(1<<0);	// Interrupt flag Clear
    if(stop == 1) // stop(SW1) 키를 눌렀을 때
    {
      //멈춤
    }
    else if(stop == 0) // clear( SW0) 키를 눌렀을 때
    {
      Watch_System(); // Watch_System 함수 동작
    }
}

void TIM5_IRQHandler(void)  // 100ms Interrupt
{
	TIM5->SR &= ~(1<<0);	// Interrupt flag Clear
    if(stop2 == 1) // stop2(SW7) 키를 눌렀을 때
    {
      //클리어
    }
    else if((stop2 == 0) && (toggle == 1))// clear( SW6) 키를 눌렀을 때
    {
      StopWatch_System();// StopWatch_System 함수 동작
    }
    else if((stop2 == 0) && (toggle == 0))
    {
      //시간 멈춤
    }
}


void EXTI9_5_IRQHandler(void)		// EXTI 5~9 인터럽트 핸들러
{
	if(EXTI->PR & 0x0100) 		// EXTI8 nterrupt Pending? //Clear key sw0
	{
        EXTI->PR |= 0x0100; 	// Pending bit Clear // 다른 것과 다르게 1을 써야 지워진다.
        S100M = S1 = S10 = M1 = M10 = H1 =  H10 = 0;
        stop = 0;
        LCD_DisplayText(1,3,"00:00:00:0");
        BEEP(); //BZ ON
        GPIOG->ODR |= 0x01; //LED0 on
        GPIOG->ODR &=  ~0x02; //LED1 off
        //SW0_Flag = 1;
	}
    
	else if(EXTI->PR & 0x0200) 	// EXTI9 Interrupt Pending? //Stop key sw1
	{
        EXTI->PR |= 0x0200; 	// Pending bit Clear
        GPIOG->ODR |= 0X02; //LED1 on
        GPIOG->ODR &= ~0X01; //LED0 off
        stop = 1;
        BEEP(); //BZ ON
        //SW1_Flag = 1;	  
	}
    
}
void EXTI15_10_IRQHandler(void)		// EXTI 15~10 인터럽트 핸들러
{
    if((EXTI->PR & 0x4000) && (toggle == 0))      //EXTI14 Interrupt Pending?//SW6
    {
        EXTI->PR |= 0x4000; 		//  Pending bit Clear
        GPIOG->ODR  ^= 0x40; // LED6 on (toggle)
        GPIOG->ODR &=  ~0x80; // LED7 off
        BEEP();//BZ ON
        stop2 = 0;
        toggle = 1; // 처음 눌렀을때
        //SW6_Flag = 1;
    }
    else if((EXTI->PR & 0x4000) && (toggle == 1))
    {
      EXTI->PR |= 0x4000; 		// Pending bit Clear
        GPIOG->ODR  ^= 0x40; // LED6 on (toggle)
        GPIOG->ODR &=  ~0x80; // LED7 off
        BEEP();//BZ ON
        stop2 = 0;
        toggle = 0; // 한 번 더 눌렀을때
        //SW6_Flag = 1;
    }
    else if(EXTI->PR & 0x8000)  //  EXTI15 Interrupt Pending? //SW7
    {
        EXTI->PR |= 0x4000; 		// Pending bit Clear
        GPIOG->ODR |= 0x80; //LED7 on
        GPIOG->ODR &=  ~0x40; //LED6 off
        BEEP();//BZ ON
        s100M = s1 = s10 = m1 =m10 = 0; // 분,초 0으로 초기화
        LCD_DisplayText(2,3,"00:00:0");
        stop2 = 1;
        toggle = 0; //7번 스위치를 누른 후 6번스위치를 눌러도 스탑워치가 바로 실행될수 있도록 하기위해....
       // SW7_Flag = 1;
    }
}

void Watch_System()
{
  	S100M++; // 1/10초 시계 1씩 증가
    LCD_DisplayChar(1,12,S100M+0x30);
    if(S100M == 10) // 1/10초 시계가 10이면
    {	
      S100M = 0; // 1/10초 시계 0으로 초기화
      S1++;  // 1초시계 1씩 증가 시계

            if(S1 == 10) //1초 시계가 10이면
            {
                S1 = 0;
                S10++; //10초 시계 1씩 증가
               
                if(S10 == 6) // 10초시계가 6이 되면 0으로 : why? -> 60초이면 1분이므로 6부분이 0으로 되어야한다.
                {
					S10 = 0;
					M1++;
					
                 	if(M1 == 10)
                 	{
	                	 M1 = 0;
	                	 M10++;
	                	 
	                	 if(M10 == 6)
	                	 {
	                		M10 = 0;
	                		H1++;
	                		
	                		if(H1 == 10)
	                		{
	                			H1 = 0;
	                			H10++;
	                			
	                			if(H10 == 2)
	                			{
	                				H10 = 0;
                                }
								//LCD_DisplayChar(1,3,H10+0x30);
							}
							//LCD_DisplayChar(1,4,H1+0x30);
						 }
						// LCD_DisplayChar(1,6,M10+0x30);
				 	}
				 	//LCD_DisplayChar(1,7,M1+0x30);
                }
               // LCD_DisplayChar(1,9,S10+0x30);	
            }
            //LCD_DisplayChar(1,10,S1+0x30);
        LCD_DisplayChar(1,12,S100M+0x30);
        LCD_DisplayChar(1,10,S1+0x30);
        LCD_DisplayChar(1,9,S10+0x30);
        LCD_DisplayChar(1,7,M1+0x30);
        LCD_DisplayChar(1,6,M10+0x30);	
        LCD_DisplayChar(1,4,H1+0x30);
        LCD_DisplayChar(1,3,H10+0x30);
    }
    else if((H1== 2) && (H10 == 1) && (Day == 1)) // (Day == 1은 PM이라는 의미) PM이고 12시가 되면 AM으로
    {
        Day = 0; //AM
        H10 = 0;
        H1 = 0; // 00시간으로 표기 //AM(0~11)
        AM_PM(Day); //AM으로
        LCD_DisplayChar(1,4,H1+0x30);
        LCD_DisplayChar(1,3,H10+0x30);
    }
    else if((H1 == 2) && (H10 == 1) && (Day == 0)) //AM이고 12시가 되면 PM으로
    {
        Day = 1; //PM
        H10 = 1;
        H1 = 2;  // 12시간으로 표기 //PM(12, 1 ~ 11)
        AM_PM(Day);// PM으로
        LCD_DisplayChar(1,4,H1+0x30);
        LCD_DisplayChar(1,3,H10+0x30);
        
        if((H1 == 2) && (H10 == 1)) 
        {
            Day = 1; //PM
            H10 = 0;
            H1 = 1;  // 01시간으로 표기 //PM(12, 1 ~ 11)
            AM_PM(Day);// PM으로
            LCD_DisplayChar(1,4,H1+0x30);
            LCD_DisplayChar(1,3,H10+0x30);
        }
    }
}
void StopWatch_System()
{
    s100M++; // 1/10초 시계 1씩 증가
    LCD_DisplayChar(2,9,s100M+0x30);
    if(s100M == 10) // 1/10초 시계가 10이면
    {	
      s100M = 0; // 1/10초 시계 0으로 초기화
      s1++;  // 1초시계 1씩 증가 시계

            if(s1 == 10) //1초 시계가 10이면
            {
                s1 = 0;
                s10++; //10초 시계 1씩 증가
                if(s10 == 6) // 10초시계가 6이 되면 0으로 : why? -> 60초이면 1분이므로 6부분이 0으로 되어야한다.
                {
                    s10 = 0;
                    m1++;
                    if(m1 == 10)
                    {
                    m1 = 0;
                    m10++;
                        if(m10 == 6)
                        {
                            m10 = 0;	
                        }	
                    }	
                }
            }
        }
        LCD_DisplayChar(2,9,s100M+0x30);
        LCD_DisplayChar(2,3,m10+0x30);
        LCD_DisplayChar(2,4,m1+0x30);
        LCD_DisplayChar(2,6,s10+0x30);	
        LCD_DisplayChar(2,7,s1+0x30);
    
}

void AM_PM(int Day) //AM과 PM 구분 함수
{
	if(Day == 1)
	{
		LCD_DisplayText(1,0,"PM");
	}
	else if(Day == 0)
	{
		LCD_DisplayText(1,0,"AM");	
	}
}

void BEEP(void)			/* beep for 30 ms */
{ 	GPIOF->ODR |= 0x0200;	// PF9 'H' Buzzer on
	DelayMS(30);		// Delay 30 ms
	GPIOF->ODR &= ~0x0200;	// PF9 'L' Buzzer off
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

void WriteTitle(char* str)
{
	LCD_Clear(RGB_WHITE);
	LCD_SetFont(&Gulim10);
	LCD_SetBackColor(RGB_YELLOW);
	LCD_SetTextColor(RGB_BLACK);
    
	LCD_DisplayText(0,0,str);
}

void RunMenu(void)
{
	WriteTitle("Digital Clock");
    LCD_SetFont(&Gulim10);	
    LCD_DisplayText(2,0,"SW 00:00:0");
    LCD_DisplayText(1,0,"PM 11:59:55:0");
	//LCD_SetFont(&Gulim10);		//폰트 &Gulim10으로 
	LCD_SetBackColor(RGB_YELLOW);	//글자배경색
	LCD_SetTextColor(RGB_BLACK);	//글자색
      
	/*
     LCD_DisplayText(1,0,"0.LED0 On ");
	LCD_DisplayText(2,0,"1.LED1 Off");
	LCD_DisplayText(3,0,"2.LED2 Off");
       */
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

    // Enable HSE : 외부 입력 클락주파수: 8MHz(회로도 참조)
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
					// 100: AHB clock divided by 2 (APB2 Timer Clock = APB2_CLK * 2= 168MHz) 

	// PCLK1 = HCLK / 4  (PCLK1 = 42MHz)
	RCC->CFGR |= 0x00001400;	// PPRE1: APB(Advanced Peripheral Bus)(APB1) Low-speed prescaler
					// 101: AHB clock divided by 4 (APB1 Timer Clock = APB1_CLK * 2= 84MHz)  

    	// Configure the main PLL 
	// Reset vlaue: 0x2400 3010 (PPLQ:4, PLLSR:0, PLL_M:16, PLL_N:192, PLL_P: 2(00))
        RCC->PLLCFGR = 0;
	RCC->PLLCFGR |= 8;	// PLL_M(6bits): 8(0b001000): /8
	RCC->PLLCFGR |= (336 << 6);		// PLL_N(9bits): 336 : *336
	RCC->PLLCFGR |= (((2 >> 1) -1)<<16);// PLL_P(2bits): (2 >> 1) -1=0b00 :  2 : /2 
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

// APB1 peripherals address
#define TIM2_BASE             (APB1PERIPH_BASE + 0x0000)
#define TIM3_BASE             (APB1PERIPH_BASE + 0x0400)
#define TIM4_BASE             (APB1PERIPH_BASE + 0x0800)
#define TIM5_BASE             (APB1PERIPH_BASE + 0x0C00)
#define TIM6_BASE             (APB1PERIPH_BASE + 0x1000)
#define TIM7_BASE             (APB1PERIPH_BASE + 0x1400)
#define TIM12_BASE            (APB1PERIPH_BASE + 0x1800)
#define TIM13_BASE            (APB1PERIPH_BASE + 0x1C00)
#define TIM14_BASE            (APB1PERIPH_BASE + 0x2000)

// APB2 peripherals address
#define TIM1_BASE             (APB2PERIPH_BASE + 0x0000)
#define TIM8_BASE             (APB2PERIPH_BASE + 0x0400)
#define SYSCFG_BASE           (APB2PERIPH_BASE + 0x3800)
#define EXTI_BASE             (APB2PERIPH_BASE + 0x3C00)
#define TIM9_BASE             (APB2PERIPH_BASE + 0x4000)
#define TIM10_BASE            (APB2PERIPH_BASE + 0x4400)
#define TIM11_BASE            (APB2PERIPH_BASE + 0x4800)

// RCC Structue
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
	
// FLASH Structue 
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

// GPIO Structue 
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

// EXTI Structue 
typedef struct
{
  __IO uint32_t IMR;    // EXTI Interrupt mask register, Address offset: 0x00 
  __IO uint32_t EMR;    // EXTI Event mask register, Address offset: 0x04 
  __IO uint32_t RTSR;   // EXTI Rising trigger selection register,  Address offset: 0x08
  __IO uint32_t FTSR;   // EXTI Falling trigger selection register, Address offset: 0x0C
  __IO uint32_t SWIER;  // EXTI Software interrupt event register,  Address offset: 0x10 
  __IO uint32_t PR;     // EXTI Pending register, Address offset: 0x14 
} EXTI_TypeDef;

// SYSCFG Structue 
typedef struct
{
  __IO uint32_t MEMRMP;       // SYSCFG memory remap register, Address offset: 0x00 
  __IO uint32_t PMC;          // SYSCFG peripheral mode configuration register, Address offset: 0x04
  __IO uint32_t EXTICR[4];    // SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14 
  uint32_t      RESERVED[2];  // Reserved, 0x18-0x1C     
  __IO uint32_t CMPCR;        // SYSCFG Compensation cell control register,Address offset: 0x20

} SYSCFG_TypeDef;

// Timer Structue 
typedef struct
{
  __IO uint16_t CR1;         // TIM control register 1, Address offset: 0x00
  uint16_t      RESERVED0;   // Reserved, 0x02   
  __IO uint16_t CR2;         // TIM control register 2, 0x04 
  uint16_t      RESERVED1;   // Reserved, 0x06                                            
  __IO uint16_t SMCR;        // TIM slave mode control register, 0x08
  uint16_t      RESERVED2;   // Reserved, 0x0A                                            
  __IO uint16_t DIER;        // TIM DMA/interrupt enable register, 0x0C 
  uint16_t      RESERVED3;   // Reserved, 0x0E                                            
  __IO uint16_t SR;          // TIM status register, 0x10 
  uint16_t      RESERVED4;   // Reserved, 0x12                                            
  __IO uint16_t EGR;         // TIM event generation register, 0x14 
  uint16_t      RESERVED5;   // Reserved, 0x16                                            
  __IO uint16_t CCMR1;       // TIM capture/compare mode register 1, 0x18 
  uint16_t      RESERVED6;   // Reserved, 0x1A                                            
  __IO uint16_t CCMR2;       // TIM capture/compare mode register 2, 0x1C 
  uint16_t      RESERVED7;   // Reserved, 0x1E                                            
  __IO uint16_t CCER;        // TIM capture/compare enable register, 0x20 
  uint16_t      RESERVED8;   // Reserved, 0x22                                            
  __IO uint32_t CNT;         // TIM counter register, 0x24 
  __IO uint16_t PSC;         // TIM prescaler, 0x28 
  uint16_t      RESERVED9;   // Reserved, 0x2A                                            
  __IO uint32_t ARR;         // TIM auto-reload register, 0x2C 
  __IO uint16_t RCR;         // TIM repetition counter register, 0x30 
  uint16_t      RESERVED10;  // Reserved, 0x32                                            
  __IO uint32_t CCR1;        // TIM capture/compare register 1, 0x34 
  __IO uint32_t CCR2;        // TIM capture/compare register 2, 0x38 
  __IO uint32_t CCR3;        // TIM capture/compare register 3, 0x3C 
  __IO uint32_t CCR4;        // TIM capture/compare register 4, 0x40 
  __IO uint16_t BDTR;        // TIM break and dead-time register, 0x44 
  uint16_t      RESERVED11;  // Reserved, 0x46                                            
  __IO uint16_t DCR;         // TIM DMA control register, 0x48 
  uint16_t      RESERVED12;  // Reserved, 0x4A                                            
  __IO uint16_t DMAR;        // TIM DMA address for full transfer, 0x4C 
  uint16_t      RESERVED13;  // Reserved, 0x4E                                            
  __IO uint16_t OR;          // TIM option register, 0x50 
  uint16_t      RESERVED14;  // Reserved, 0x52                                            
} TIM_TypeDef;

// 각 주변장치 모듈 선언
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

#define CRC     ((CRC_TypeDef *) CRC_BASE)
#define RCC     ((RCC_TypeDef *) RCC_BASE)
#define FLASH   ((FLASH_TypeDef *) FLASH_R_BASE)

#define SYSCFG  ((SYSCFG_TypeDef *) SYSCFG_BASE)
#define EXTI    ((EXTI_TypeDef *) EXTI_BASE)

#define TIM2    ((TIM_TypeDef *) TIM2_BASE)
#define TIM3    ((TIM_TypeDef *) TIM3_BASE)
#define TIM4    ((TIM_TypeDef *) TIM4_BASE)
#define TIM5    ((TIM_TypeDef *) TIM5_BASE)
#define TIM6    ((TIM_TypeDef *) TIM6_BASE)
#define TIM7    ((TIM_TypeDef *) TIM7_BASE)
#define TIM12   ((TIM_TypeDef *) TIM12_BASE)
#define TIM13   ((TIM_TypeDef *) TIM13_BASE)

#define TIM1    ((TIM_TypeDef *) TIM1_BASE)
#define TIM8    ((TIM_TypeDef *) TIM8_BASE)
#define TIM9    ((TIM_TypeDef *) TIM9_BASE)
#define TIM10   ((TIM_TypeDef *) TIM10_BASE)
#define TIM11   ((TIM_TypeDef *) TIM11_BASE)

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