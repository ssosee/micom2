#include "stm32f4xx.h"
#include "GLCD.h"
#include "ACC.h"

void DisplayTitle(void);
void _GPIO_Init(void);
void SPI1_Init(void);
void TIMER1_Init(void);
void Display_Process(int16 *pBuf);
void BEEP(void); //부저 설정
void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);

//void SPI1_Process(UINT16 *pBuf);  // ACC.c 
//void ACC_Init(void); // ACC.c
//void LCD_Init(void); // GLCD.c

UINT8 bControl;

int main(void)
{
        int16 buffer[3];
    
        LCD_Init();	// LCD 구동 함수
        DelayMS(10);	// LCD구동 딜레이
        DisplayTitle();	// LCD 초기화면구동 함수
    
       	_GPIO_Init();
        SPI1_Init();        
        ACC_Init();
        TIMER1_Init();
           
        while(1)
        {
                if(bControl)  //300ms 마다 참
                {
                        bControl = FALSE;     
                        SPI1_Process(&buffer[0]);	// SPI통신을 이용하여 가속도센서 측정
                        Display_Process(&buffer[0]);	// 측정값 LCD에 표시
                }
        }
}

void SPI1_Init(void)
{
        /*!< Enable the SPI clock */
        RCC->APB2ENR |=	0x1000;

        /*!< Enable GPIOA clocks */
        RCC->AHB1ENR |=	0x0001;
  
        /*!< SPI pins configuration *************************************************/
	
        /*!< SPI NSS pin(PA8) configuration */
        GPIOA->MODER |= 0x00010000;	// GPIOA PIN8 Output mode
        GPIOA->OTYPER &= ~0x0100;	// push-pull(reset state)
        GPIOA->OSPEEDR |= 0x00030000;	// GPIOA PIN8 Output speed (100MHZ) 
        GPIOA->PUPDR &= ~0x00030000;	// NO Pullup Pulldown(reset state)
    
        /*!< SPI SCK pin(PA5) configuration */
        GPIOA->MODER |= 0x00000800;	// GPIOA PIN5 Output Alternate function mode
        GPIOA->OTYPER &= ~0x0020;	// GPIOA PIN5 Output type push-pull (reset state)
        GPIOA->OSPEEDR |= 0x00000C00;	// GPIOA PIN5 Output speed (100MHz Very High speed)
        GPIOA->PUPDR |= 0x00000800;	// GPIOA PIN5 Pull-down
        GPIOA->AFR[0] |= 0x00500000;	// Connect GPIOA pin5 to AF5(SPI1)
    
        /*!< SPI MOSI pin(PA7) configuration */    
        GPIOA->MODER |= 0x00008000;	// GPIOA PIN7 Output Alternate function mode
        GPIOA->OTYPER &= ~0x0080;	// GPIOA PIN7 Output type push-pull (reset state)
        GPIOA->OSPEEDR |= 0x0000C000;	// GPIOA PIN7 Output speed (100MHz Very High speed)
        GPIOA->PUPDR |= 0x00008000;	// GPIOA PIN7 Pull-down
        GPIOA->AFR[0] |= 0x50000000;	// Connect GPIOA pin7 to AF5(SPI1)
    
        /*!< SPI MISO pin(PA6) configuration */
        GPIOA->MODER |= 0x00002000;	// GPIOA PIN6 Output Alternate function mode
        GPIOA->OTYPER &= ~0x0040;	// GPIOA PIN6 Output type push-pull (reset state)
        GPIOA->OSPEEDR |= 0x00003000;	// GPIOA PIN6 Output speed (100MHz Very High speed)
        GPIOA->PUPDR |= 0x00002000;	// GPIOA PIN6 Pull-down
        GPIOA->AFR[0] |= 0x05000000;	// Connect GPIOA pin6 to AF5(SPI1)

       // Init SPI1 Registers 
        SPI1->CR1 &= ~(1<<15);	//SPI_Direction_2Lines_FullDuplex
        SPI1->CR1 &= ~(1<<11);	//SPI_DataSize_8b
        /**/SPI1->CR1 |= (1<<9);   //0x0200;	//SPI_NSS_Soft
        SPI1->CR1 |= (1<<8);   //0x0100;	//SPI_Internal_slave_select
        SPI1->CR1 &= ~(1<<7);	//SPI_FirstBit_MSB
        SPI1->CR1 |= (4<<3);   //0x0020;	//SPI_BaudRatePrescaler_32 
        /**/SPI1->CR1 |= (1<<2);   //0x0004;	//SPI_Mode_Master
        SPI1->CR1 |= (1<<1);   //0x0002;	//SPI_CPOL_High
        SPI1->CR1 |= (1<<0);   //0x0001;	//SPI_CPHA_2Edge
 
        SPI1->CR1 |= (1<<6);   //0x40;	// Enable SPI1 
}



void TIMER1_Init(void)
{
        RCC->APB2ENR 	|= (1<<0);	// RCC_APB2ENR TIMER1 Enable 
        NVIC->ISER[0] |= ( 1 << 27 );	//Enable Timer3 global Interrupt
    
        /**/TIM1->PSC	= 16800-1;	// Prescaler 168,000,000Hz/16800 = 10,000Hz(0.1ms)  (1~65536) 168
        /**/TIM1->ARR	= 10-1;	// Auto reload  (0.1ms * 10= 1ms : PWM Period)   1ms
        
       	TIM1->CR1 &= ~(1<<4);	// DIR: Countermode = Upcounter (reset state)
        TIM1->CR1 &= ~(3<<8);	// CKD: Clock division = 1 (reset state)
        TIM1->CR1 &= ~(3<<5); 	// CMS(Center-aligned mode Sel): No(reset state)
   
        TIM1->EGR |= (1<<0);    // UG: Update generation 
        TIM1->DIER |= (1<<0);		//Enable Tim1 Update interrupt   
        
        //채널에 대한 언급이 없어서 임의로 CH3으로 했다.
        // Output/Compare Mode
        TIM1->CCER |= (1<<8);	// CC3E: OC3 Active 
        TIM1->CCER &= ~(1<<9);	// CC3P: OCPolarity_High
        
        TIM1->EGR |= (1<<3);    // CC3G: Capture/Compare 1 event generation    

        /**/TIM1->CCMR2 &= ~(3<<0); // CC3S(CC3 channel): Output 
        /**/TIM1->CCMR2 &= ~(1<<3); // OC3PE: Output Compare 3 preload disable
        /**/TIM1->CCMR2 |= (3<<4);	// OC3M: Output Compare 3 Mode : toggle

        /**/TIM1->CR1 &= ~(1<<7);	// ARPE: Auto reload preload disable
        TIM1->DIER |= (1<<3);	// CC3IE: Enable the Tim1 CC3 interrupt
        TIM1->CR1 |= (1<<0);	// CEN: Enable the Tim1 Counter
}

void TIM1_CC_IRQHandler(void)	// 1ms int
{
        static UINT16 LED_cnt = 0;
        static UINT16 ControlCnt = 0;

    
        TIM1->SR &= ~(1<<3);		//Interrupt flag Clear
    
        LED_cnt++;								
        ControlCnt++;
        
        if(ControlCnt >= 300)		//300ms
        {
          ControlCnt = 0; //0으로 초기화
                bControl = TRUE;	//300ms마다 센서 측정
                
        }
        
}

void Display_Process(int16 *pBuf) //부호를 살려서 표기해줘야함
{
        char str[20];
        int i;
        
         LCD_SetPenColor(GET_RGB(199,21,33)); //보라색
        for(i=0; i<4; i++)
        {
        LCD_DrawVerLine(84+i, 50, 60); //세로축을 그린다.
        }
        
        if(pBuf[0] < 0) //음수일 경우
        {
            sprintf(str, "Ax:%06d", pBuf[0]);	//X AXIS 
            LCD_DisplayText(1,0,str);
                
                LCD_SetPenColor(RGB_WHITE); //x축 bar를 지운다
                if(pBuf[0] < (65536/2)*(0.75)*(-1)) // Ax < -1.5g 일때
                {
                  BEEP(); //부저를 울린다.
                }
                for(i = 0; i<4; i++)
                {
                 LCD_DrawHorLine(15,60+i,70); 
                 LCD_DrawHorLine(90,60+i,70); 
                }
                
                for(i = 0; i<4; i++)
                {
                  LCD_SetPenColor(RGB_RED); //빨간색
                  LCD_DrawHorLine(15,60+i ,70); //x축 bar 전체 그린다.
                }
                for(i = 0; i<4; i++) 
                {
                LCD_SetPenColor(RGB_WHITE); //흰색
                 LCD_DrawHorLine(15,60+i , 70  + pBuf[0]/(32768/70)); //흰색으로 값에 일치하게 지워준다.
                }

              
        }
        else if(pBuf[0] >=0)
        {
            sprintf(str, "Ax: %5d", pBuf[0]);	//X AXIS 
            LCD_DisplayText(1,0,str);
            LCD_SetPenColor(RGB_WHITE); //x축 bar를 지운다.
                if(pBuf[0] > (65536/2)*(0.75)) // Ax > 1.5g 일때
                {
                  BEEP();
                }
                for(i = 0; i<4; i++)
                {
                    LCD_DrawHorLine(90,60+i,70); 
                    LCD_DrawHorLine(15,60+i,70);
                }
                    LCD_SetPenColor(RGB_RED);
                for(i = 0; i<4; i++)
                {
                    LCD_DrawHorLine(90,60+i ,pBuf[0]/(32768/70)); //x축 bar를 그린다.
                }
                 
        }
        
         if(pBuf[1] < 0) //음수일 경우
        {
            sprintf(str, "Ax:%06d", pBuf[1]);	//Y AXIS 
            LCD_DisplayText(2,0,str);
                if(pBuf[1] < (65536/2)*(0.75)*(-1)) // Ay < -1.5g 일때
                {
                  BEEP();
                }
                LCD_SetPenColor(RGB_WHITE); //y축 bar를 지운다
                for(i = 0; i<4; i++)
                {
                 LCD_DrawHorLine(15,75+i,70); 
                 LCD_DrawHorLine(90,75+i,70); 
                }
                
                for(i = 0; i<4; i++)
                {
                  LCD_SetPenColor(RGB_GREEN);
                  LCD_DrawHorLine(15,75+i ,70); //y축 bar 전체 그린다.
                }
                for(i = 0; i<4; i++) 
                {
                LCD_SetPenColor(RGB_WHITE);
                 LCD_DrawHorLine(15,75+i , 70  + pBuf[1]/(32768/70)); //흰색으로 값에 맞게지워준다.
                }

              
        }
        else if(pBuf[1] >=0)
        {
            sprintf(str, "Ax: %5d", pBuf[1]);	//Y AXIS 
            LCD_DisplayText(2,0,str);
            LCD_SetPenColor(RGB_WHITE); //y축 bar를 지운다
                if(pBuf[1] > (65536/2)*(0.75)) // Ay > 1.5g 일때
                {
                  BEEP();
                }
                for(i = 0; i<4; i++)
                {
                    LCD_DrawHorLine(90,75+i,70); 
                    LCD_DrawHorLine(15,75+i,70);
                }
                    LCD_SetPenColor(RGB_GREEN);
                for(i = 0; i<4; i++)
                {
                    LCD_DrawHorLine(90,75+i ,pBuf[1]/(32768/70)); //y축 bar를 그린다.
                }
        }
        
         if(pBuf[2] < 0) //음수일 경우
        {
            sprintf(str, "Ax:%06d", pBuf[2]);	//Z AXIS 
            LCD_DisplayText(3,0,str);
                
               
                LCD_SetPenColor(RGB_WHITE); //z축 bar를 지운다

                if(pBuf[2] < (65536/2)*(0.75)*(-1)) // Az < -1.5g 일때
                {
                  BEEP(); 
                }
                for(i = 0; i<4; i++)
                {
                 LCD_DrawHorLine(15,90+i,70); 
                 LCD_DrawHorLine(90,90+i,70); 
                }
                
                for(i = 0; i<4; i++)
                {
                  LCD_SetPenColor(RGB_BLUE);
                  LCD_DrawHorLine(15,90+i ,70); //z축 bar 전체 그린다.
                }
                for(i = 0; i<4; i++) 
                {
                LCD_SetPenColor(RGB_WHITE);
                 LCD_DrawHorLine(15,90+i , 70  + pBuf[2]/(32768/70)); //흰색으로 값에 맞게지워준다.
                }
    
        }
        else if(pBuf[2] >=0)
        {
            sprintf(str, "Ax: %5d", pBuf[2]);	//Z AXIS 
            LCD_DisplayText(3,0,str);
            LCD_SetPenColor(RGB_WHITE); //z축 bar를 지운다
             if(pBuf[2] > (65536/2)*(0.75)) // Az > 1.5g 일때
                {
                  BEEP();
                }
                for(i = 0; i<4; i++)
                {
                    LCD_DrawHorLine(90,90+i,70); 
                    LCD_DrawHorLine(15,90+i,70);
                }
                    LCD_SetPenColor(RGB_BLUE);
                for(i = 0; i<4; i++)
                {
                    LCD_DrawHorLine(90,90+i ,pBuf[2]/(32768/70)); //z축 bar를 그린다.
                }
        }     
}

void _GPIO_Init(void)
{
        // LED GPIO 설정
        RCC->AHB1ENR	|= 0x40;	// RCC_AHB1ENR GPIOG Enable		
        GPIOG->MODER 	|= 0x5555;	// GPIOG PIN0~PIN7 Output mode	
        GPIOG->OSPEEDR |= 0x5555;	// GPIOG PIN0~PIN7 Output speed (25MHZ Medium speed) 
    
        // SW GPIO 설정
        RCC->AHB1ENR 	|= 0x80;	// RCC_AHB1ENR GPIOH Enable
        GPIOH->MODER 	|= 0x00000000;	// GPIOH PIN8~PIN15 Input mode (reset state)				
        GPIOG->ODR &= 0x00;	// LED0~7 Off
        
        // Buzzer GPIO(PORT F) 설정 
        RCC->AHB1ENR	|= 0x00000020; 	// RCC_AHB1ENR(bit8~0) GPIOF(bit#5) Enable							
        GPIOF->MODER 	|= 0x00040000;	// GPIOF PIN9 Output mode (0b01)						
        GPIOF->OTYPER &= 0xFDFF;	// GPIOF PIN9 : Push-pull  	
        GPIOF->OSPEEDR |= 0x00040000;	// GPIOF PIN9 Output speed (25MHZ Medium speed) 
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
                DelayUS(1000);		//1000us => 1ms
}

void DelayUS(unsigned short wUS)
{
        volatile int Dly = (int)wUS*17;
         for(; Dly; Dly--);
}

void DisplayTitle(void)
{
        LCD_Clear(RGB_WHITE);
        LCD_SetFont(&Gulim8);		//폰트 
        LCD_SetTextColor(RGB_BLACK);    //글자색
        LCD_SetBackColor(RGB_YELLOW);
        LCD_DisplayText(0,0,"ACC sendsor(SPI)");
        LCD_SetBackColor(RGB_WHITE);    //글자배경색
        LCD_DisplayText(4,0,"Ax:");
        LCD_DisplayText(5,0,"Ay:");
        LCD_DisplayText(6,0,"Az:");
        
        
}
