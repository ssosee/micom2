#include "stm32f4xx.h"
#include "GLCD.h"
(Jeongwang-dong, Gyunggi Univ of Science & Technology 2nd Small & Medium Business Center)
 Gyeonggi-do Shieung City GyeonggiSienceTech Street 270, Room 106
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
#define N_Down            0x0F70 //PI7
#define N_Up                0x0FB0 //PI6
#define N_Push             0x0FD0 //PI5

#define pi 3.141591 //pi에 대한 정의
void _RCC_Init(void);
void _GPIO_Init(void);

void Alarm_Mode(); //알람 모드 함수
void Stop_Watch_Mode(); //스탑워치 모드 함수
void Thermostat_Mode();// 온도모드 함수
void Calculator_Mode();//계산기 함수
void Clear_LCD(); //텍스트 색상을 기본값으로 바꾼다.
void Clear_ClockDispaly(); //시계를 그려주는 부분을

void onTimer(); //우측상단의 현재 시각을 구현하는 함수
void onTimer2();//아날로그 시계의 시각을 구현하는 함수
void onTimerDisplay(); //모드1 일때 현재시각을 디스플레이 해주는 함수
void onTimerDisplay2(); //모드1일때 아날로그 시계를 표시해주는 디스플레이 함수
void Alarm_Display(); //모드1의 아날로그 시계의 알람 시침을 디스플레이하는 함수
void TIMER7_Init(void); //타이머7 초기화 함수
void TIMER9_Init(void); //타이머9 초기화 함수
void TIMER3_Init(void); //타이머3 초기화 함수
void TIMER4_Init(void); //타이머4 초기화 함수
void StopWatch(); //스탑워치구동 함수
void StopWatch_Display(); //스탑워치 디스플레이 함수
void USART1_Init(void); //유사트1 통신
void USART_BRR_Configuration(uint32_t USART_BaudRate); //유사트 통신 함수
void SerialPutChar(uint8_t c); //컴마스터 터미널 문자 함수
void Serial_PutString(char *s); //컴마스터 터미널 문자열함수
void _ADC1_Init(void); //ADC1 초기화 함수

uint16_t Navi_KEY_Scan(void); //Navi switch에 대한 key scan 함수
void _EXTI_Init(void); //인터럽트 초기화 함수

void DelayMS(unsigned short wMS); //딜레이 함수
void DelayUS(unsigned short wUS); //딜레이 함수
void BEEP(void);// 부저 함수

int SW0cnt = 1; //SW0가 몇번 눌려서 모드 몇 인지 구별 해주는 변수
int S = 10; //초 A
int M = 15; //분 F

int S2 =10; //초

short A_S = 0;  //알람 초
short A_M = 0; //알람 분
short A_S2 = 0; //알람 초
short A_M2 = 0; //알람 분

int stop_msec = 0; //스탑워치 msec
int stop_sec = 0; //스탑워치 sec
int stop_min = 0; //스탑워치 min

int StopWatch_Start = 0; //스탑워치 시작 구별 변수

int RX = 0; // Computer의 신호를 받는 변수

int ADC_Value, Voltage, Temp; //ADC 값, Voltage 값, 온도 값

int ADC1_Flag = 0; //ADC 핸들러가 동작 했는가를 알려주는 flag
int Usart_Flag = 0; //유사트 모드 flag
int Alarm_Flag = 0;//알람 모드 일 때 flag
short Push_Flag = 0; //push 버튼에 대한 flag

/*문자를 저장하기위한 변수*/
char str[20];
char str2[20];
char str3[20];
char str4[20];
char str5[20];
char str_A_M [20];
char str_A_S [20];
/*************************/

char ch; //유사트 통신에서 데이터를 담는 변수
int sum; //결과적으로 op1+op2
int door1 = 0;//유사트 통신에서 op1, op2 , '=' 의 입력 순서를 구별하기위한 변수
int op1, op2;//op1값, op2값

int i = 0; //for문 수행을 위한 변수

void USART1_IRQHandler(void) // 유사트1 핸들러
{
    if ( (USART1->SR & USART_SR_RXNE) )
    {
		ch = (uint16_t)(USART1->DR & (uint16_t)0x01FF);	// 수신된 문자 저장

        Usart_Flag = 1; //메인문에서 실행시키기위해 플래그를 사용한다.
    }
}

void TIM1_BRK_TIM9_IRQHandler(void) //타이머9 핸들러
{
  TIM9->SR &= ~(1<<0);	// Interrupt flag Clear
  TIM9->SR &= ~(1<<1);	// Interrupt flag Clear (ch1)

  if((StopWatch_Start == 1) && (SW0cnt == 2)) //sw4가 눌리고 스탑워치 작동
  //SWcnt == 2는 2번째 모드에서만 디스플레이 하게 하기 위하여
  {
    StopWatch(); //스탑워치 함수 동작
    LCD_SetTextColor(RGB_GREEN);	//글자색
    LCD_DisplayText(0,0,"2.Stop Watch"); // 가끔씩 S-W 의 시간이 위로 튕기는 현상을 막기위해 추가
  }
}

void TIM7_IRQHandler(void)  // 1s Interrupt
{
    TIM7->SR &= ~(1<<0);	// Interrupt flag Clear
    onTimer(); //타이머가  작동
    onTimer2();//아날로그 시계 동작
    Alarm_Flag = 1;
}

void EXTI15_10_IRQHandler(void)		//스탑워치 기능들 // EXTI 15~10 인터럽트 핸들러
{
      if((EXTI->PR & 0x1000) &&  (SW0cnt == 2)) // Pending bit Clear SW4
      {
      EXTI->PR |= 0x1000; //pending bit clear
      GPIOG->ODR |= 0x0010; //LED4
      StopWatch_Start = 1; //스탑워치 시작
      BEEP();
      }
      if((EXTI->PR & 0x2000) &&  (SW0cnt ==2)) // Pending bit Clear SW5
      {
      EXTI->PR |= 0x2000; //pending bit clear
      GPIOG->ODR |= 0x0020; // LED5

      LCD_SetTextColor(RGB_RED);

      sprintf(str2, "%1X", stop_msec); //0.1초 단위
      LCD_DisplayText(2,10, str2);

      sprintf(str2, "%1X", stop_sec); //분
      LCD_DisplayText(2,8, str2);//분

      sprintf(str2, "%1X", stop_min); //초
      LCD_DisplayText(2,7, str2);//초
      BEEP();
      }
      if((EXTI->PR & 0x4000) &&(SW0cnt == 2)) // Pending bit Clear SW6
      {
      EXTI->PR |= 0x4000; //pending bit clear
      GPIOG->ODR &= ~0x0010;
      GPIOG->ODR &= ~0x0020;
      GPIOG->ODR |= 0x0040; //LED6
      StopWatch_Start = 0; //스탑워치 중지
      BEEP();
      }
      if((EXTI->PR & 0x8000) && (SW0cnt == 2)) // Pending bit Clear SW7
      {
      EXTI->PR |= 0x8000; //pending bit clear
      GPIOG->ODR &= ~0x00FF; //LED ALL CLEAR
      stop_msec = 0; //초기화
      stop_sec = 0; //초기화
      stop_min = 0; //초기화
      LCD_SetTextColor(RGB_RED);
      LCD_DisplayText(1,4,"00:0");
      LCD_DisplayText(2,7,"00:0");
      BEEP();
      }
}

void EXTI9_5_IRQHandler(void)		// EXTI 5~9 인터럽트 핸들러
{
    //SW0cnt ++;
	if((EXTI->PR & 0x0100) && (SW0cnt == 0) ) 	// Pending bit Clear SW0
	{
        EXTI->PR |= 0x0100; //pending bit clear
        Clear_LCD();
        DelayMS(10);	// LCD구동 딜레이
        Alarm_Mode(); //알람 모드 디스플레이
         GPIOG->ODR &= ~0x00FF; //ALL LED OFF
        SW0cnt = 1;
        Push_Flag = 0;
        Clear_ClockDispaly();
        BEEP();

	}
	else if((EXTI->PR & 0x0100) && (SW0cnt == 1 )) 	// Pending bit Clear SW0
	{
        EXTI->PR |= 0x0100; //pending bit clear
        Clear_LCD();
        DelayMS(10);	// LCD구동 딜레이
        Stop_Watch_Mode(); //스탑워치 모드 디스플레이
         GPIOG->ODR &= ~0x00FF;
        SW0cnt = 2;
        Push_Flag = 0;
        Clear_ClockDispaly();
        BEEP();

	}
	else if((EXTI->PR & 0x0100) && (SW0cnt == 2) ) 	// Pending bit Clear SW0
	{
        EXTI->PR |= 0x0100; //pending bit clear
        Clear_LCD();
        DelayMS(10);	// LCD구동 딜레이
        Calculator_Mode(); //계산기 모드 디스플레이
         GPIOG->ODR &= ~0x00FF;
        BEEP();
        SW0cnt = 3;
        Push_Flag = 0;
        Clear_ClockDispaly();
	}

	else if((EXTI->PR & 0x0100) && (SW0cnt == 3) ) 	// Pending bit Clear SW0
	{
        EXTI->PR |= 0x0100; //pending bit clear
        Clear_LCD();
        DelayMS(10);	// LCD구동 딜레이
        Thermostat_Mode(); //온도 모드 디스플레이
         BEEP();
        SW0cnt = 0;
        Push_Flag = 0;
        Clear_ClockDispaly();
    }
}

void ADC_IRQHandler(void)
{
  if(ADC1->SR & 0x02)
  {
    //LCD_DisplayText(0,0,"4.Thermostat");
    ADC1->SR &= ~(1<<1);
    ADC1_Flag = 1;
  }
}

int main(void)
{
        _RCC_Init();
        LCD_Init();		// LCD 구동 함수
        DelayMS(10);	// LCD구동 딜레이
        Alarm_Mode();	//LCD 초기화면구동 함수
        TIMER7_Init(); //타이머7 구동 함수
        TIMER9_Init(); //타이머9 구동 함수
        TIMER3_Init(); //타이머3 구동 함수
        TIMER4_Init(); //타이머4 구동 함수
        _EXTI_Init() ; //인터럽트 구동 함수
        _GPIO_Init(); //gpio구동 함수
        _ADC1_Init(); //ADC1 구동함수

        USART1_Init(); //유사트 구동 함수

 	while(1)
	{
        onTimerDisplay(); //현재시각 구현 함수
        if((Alarm_Flag == 1) && (SW0cnt == 1)) //알람 모드일때
        {
            switch(Navi_KEY_Scan())
            {
              case N_Left :
              A_M--;
              if(A_M < 0) //아날로그
              {
                A_M = 15;
              }
              sprintf(str_A_M, "%1X", A_M);
              LCD_DisplayText(1,6, str_A_M);
              break;

              case N_Right :
              A_M++;
              if(A_M == 16)
              {
                A_M = 0;
              }
              sprintf(str_A_M, "%1X", A_M);
              LCD_DisplayText(1,6, str_A_M);
              break;

              case N_Down :
              A_S--;
              if(A_S < 0)
              {
                A_S = 15;
              }
               sprintf(str_A_S, "%1X", A_S);
              LCD_DisplayText(1,8, str_A_S);
              break;

              case N_Up:
              A_S++;
              if(A_S == 16)
              {
              A_S = 0;
              }
              sprintf(str_A_S, "%1X", A_S);
              LCD_DisplayText(1,8, str_A_S);
              break;

              case N_Push :
            /***************************************
            push버튼을 눌렀을 때
            A_S2에 A_S값을 넣어주고
            A_M2에 A_M값을 넣어준다.
            알람시간을 up down right left로 표현 할 때
            디스플레이 되는 것을 방지하기위해
            *****************************************/

              Push_Flag = 1; //Push를 눌렀을때만 동작하게 하기위해서 플래그를 설정
              A_S2 = A_S;
              A_M2 = A_M;
              break;
            }
            onTimerDisplay2(); //모드1일때 아날로그 시계를 표시해주는 디스플레이 함수

            if(Push_Flag == 1) //Push가 눌렸을 때만
            {
            Alarm_Display(); //알람 시계 시침 표시
            }
        }

        if((Usart_Flag == 1) &&(SW0cnt == 3)) //유사트 핸들러를 동작하고 모드3일때
        {
          if (door1== 0) //door1의 값이 0일때 //door1의 초기값은 0이다.
            {
                if( ch >= 0x30 & ch <= 0x39) //0에서 9값을 받을 때
                {
                    op1 = ch - 0x30;	// 수신된 문자를 op1에 저장
                    //
                    SerialPutChar(op1);
                    sprintf(str3, "%1X", op1);
                    LCD_SetTextColor(RGB_BLACK);
                    LCD_DisplayText(1,1,str3);
                    door1 = 1; //두번째값을 받기위해 1을 넣어준다.
                    BEEP();
                }
                else if(ch >= 0x41 && ch <= 0x46) //A에서 F값을 받을 때
                {
                    op1 = ch - 0x37; //수신된 문자를 op1에 저장
                    /**************************************************************************
                    현재 op1은 int 정수형이고 ch는 문자형이라서 A의 16진수가 0x41이므로 0x37을 빼면
                    0x0A가 나온다. 0x0A는 정수형으로 10임을 알 수 있다.
                ****************************************************************************/
                    SerialPutChar(op1);
                    sprintf(str3, "%1X", op1);
                    LCD_SetTextColor(RGB_BLACK);
                    LCD_DisplayText(1,1,str4);
                    door1 = 1; //두번째값을 받기위해 1을 넣어준다.
                    BEEP();
                }
                else if(ch >= 0x61 && ch <= 0x66) //a에서 f값을 받을 때
                {
                    op1 = ch - 0x57; //수신된 문자를 op1에 저장
                    //a의 16진수가 0x61이므로 *앞에서 설명한 내용과 동일하게 0x57을 빼면 0x0A가 나온다.
                    SerialPutChar(op1);
                    sprintf(str3, "%1X", op1);
                    LCD_SetTextColor(RGB_BLACK);
                    LCD_DisplayText(1,1,str4);
                    door1 = 1;  //세번째값을 받기위해 1을 넣어준다.
                    BEEP();
                }
            }
            else if(door1 == 1) //door1 의 값이 1일 때
            {
                if( ch >= 0x30 & ch <=0x39) //0에서 9의 값을 받을 때
                {
                    op2 = ch -0x30; //수신된 문자를 op2에 저장
                    SerialPutChar(op2);
                    sprintf(str4, "%1X", op2);
                    LCD_SetTextColor(RGB_BLACK);
                    LCD_DisplayText(1,5,str4);
                    door1 = 2; //세번째값을 받기위해 1을 넣어준다.
                    BEEP();
                }
                else if(ch >= 0x41 && ch <= 0x46) //A에서 F값을 받을 때
                {
                    op2 = ch -0x37; //수신된 문자를 op2에 저장
                    SerialPutChar(op2);
                    sprintf(str4, "%1X", op2);
                    LCD_SetTextColor(RGB_BLACK);
                    LCD_DisplayText(1,5,str4);
                    door1 = 2; //세번째값을 받기위해 1을 넣어준다.
                    BEEP();
                }
                else if(ch >= 0x61 && ch <= 0x66) //a에서 f값을 받을 때
                {
                    op2 = ch -0x57; //수신된 문자를 op2에 저장
                    SerialPutChar(op2);
                    sprintf(str4, "%1X", op2);
                    LCD_SetTextColor(RGB_BLACK);
                    LCD_DisplayText(1,5,str4);
                    door1 = 2; //세번째값을 받기위해 1을 넣어준다.
                    BEEP();
                }
            }
            else if(door1 == 2) //door1의 값이 2일 때
            {         //(uint16_t)0x01FF
                if(ch == '=') //데이터가 = 일 때
                {
                sum = op1 + op2; //앞에서 수신받은 op1값과 op2값을 더한다.
                sprintf(str5, "%2X", sum);
                LCD_SetTextColor(RGB_BLACK);
                LCD_DisplayText(1,9,str5);
                door1 = 0;
                BEEP();
                }
            }
        }
         if((ADC1_Flag == 1) && (SW0cnt == 0)) //ADC 핸들러를 실행했고, 모드4 일때
            {
                LCD_SetTextColor(RGB_GREEN);	//초록 글자색
                LCD_DisplayText(0,0,"4.Thermostat");
                char temp[20];

                ADC_Value = ADC1->DR; //ADC_Value에 ADC1의 데이터 값을 넣어준다.
                Voltage = ADC_Value * 50 / 4095 -10;   // ADC값을 4095로 나누어 주고 50을 곱하면
                //0에서 50까지의 범위가 나온다. 이때 -값의 온도를 표기해주기 위해 -10을 해준다.
                ADC1_Flag = 0;
                if(Voltage <0)
                {
                  if(Voltage > -10) //Voltage의 값이 -10 보다 클 때
                  {
                     LCD_SetTextColor(RGB_BLACK);
                    sprintf(temp, "%01d", Voltage); //500ms 단위
                     LCD_DisplayText(1,4," ");
                    LCD_DisplayText(1,2, temp);
                  }
                   LCD_SetTextColor(RGB_BLACK);
                    sprintf(temp, "%01d", Voltage); //500ms 단위
                    LCD_DisplayText(1,2, temp);
                }
                else if(Voltage >= 0) //Voltage의 값이 0보다 크거나 같을 때
                {
                  if(Voltage >0 & Voltage <= 9)
                  {
                    LCD_SetTextColor(RGB_BLACK);
                    sprintf(temp, "%1d", Voltage); //500ms 단위
                    LCD_DisplayText(1,2, temp);
                    LCD_DisplayText(1,4,"  ");
                  }
                    LCD_SetTextColor(RGB_BLACK);
                    sprintf(temp, "%2d", Voltage); //500ms 단위
                    LCD_DisplayText(1,2, temp);
                    LCD_DisplayText(1,4,"  ");
                }

                if(Voltage <= 0 && Voltage >= -10) //1단계
                {
                    LCD_SetPenColor(RGB_WHITE); //x축 bar를 지운다
                    for(i = 0; i<4; i++)
                    {
                    LCD_DrawHorLine(50,25+i,70);
                    }
                    for(i = 0; i<4; i++)
                    {
                    LCD_SetPenColor(RGB_BLUE); //빨간색
                    LCD_DrawHorLine(50,25+i ,Voltage+10); //x축 bar를 그린다.
                    }

                    LCD_SetTextColor(RGB_BLACK);
                    LCD_DisplayText(2,2,"2"); //H:2
                    LCD_DisplayText(2,6,"0");//C:0

                    GPIOG->ODR &= ~0x00FFf; //LED All off
                    GPIOG->ODR |= 0x0001; //LED0 ON

                    TIM4->CCR1 = 18000;  //DR = CCR/ARR을 이용한다. //DR = 90%
                }
                else if( Voltage > 0 && Voltage <= 10) //2단계
                {
                   LCD_SetPenColor(RGB_WHITE); //x축 bar를 지운다
                    for(i = 0; i<4; i++)
                    {
                    LCD_DrawHorLine(50,25+i,70);
                    }
                    for(i = 0; i<4; i++)
                    {
                    LCD_SetPenColor(RGB_BLUE); //빨간색
                    LCD_DrawHorLine(50,25+i ,Voltage+10); //x축 bar를 그린다.
                    }

                    LCD_SetTextColor(RGB_BLACK);
                     LCD_DisplayText(2,2,"1"); //H:1
                     LCD_DisplayText(2,6,"0");//C:0

                     GPIOG->ODR &= ~0x00FFf; //LED All off
                    GPIOG->ODR |= 0x0001; //LED0 ON

                     TIM4->CCR1 = 2000;//DR = 10%
                }
                else if( Voltage > 10 && Voltage <= 20) //3단계
                {
                   LCD_SetPenColor(RGB_WHITE); //x축 bar를 지운다
                    for(i = 0; i<4; i++)
                    {
                    LCD_DrawHorLine(50,25+i,50);
                    }
                    for(i = 0; i<4; i++)
                    {
                    LCD_SetPenColor(RGB_GREEN); //빨간색
                    LCD_DrawHorLine(50,25+i ,Voltage +10); //x축 bar를 그린다.
                    }

                    LCD_SetTextColor(RGB_BLACK);
                     LCD_DisplayText(2,2,"0"); //H:0
                     LCD_DisplayText(2,6,"0");//C:0

                     GPIOG->ODR &= ~0x00FFf; //LED All off

                     TIM4->CCR1 = 0; //DR = 100%
                }
                else if( Voltage > 21 && Voltage <= 30) //4단계
                {
                   LCD_SetPenColor(RGB_WHITE); //x축 bar를 지운다
                    for(i = 0; i<4; i++)
                    {
                    LCD_DrawHorLine(50,25+i,50);
                    }
                    for(i = 0; i<4; i++)
                    {
                    LCD_SetPenColor(RGB_RED); //빨간색
                    LCD_DrawHorLine(50,25+i ,Voltage+10); //x축 bar를 그린다.
                    }

                    LCD_SetTextColor(RGB_BLACK);
                     LCD_DisplayText(2,2,"0"); //H:0
                    LCD_DisplayText(2,6,"1"); //C:1

                    GPIOG->ODR &= ~0x00FFf; //LED All off
                    GPIOG->ODR |= 0x0002; //LED1 ON

                    TIM4->CCR1 = 2000; //DR = 10%

                }
                else if( Voltage > 31 && Voltage <= 40)
                {
                   LCD_SetPenColor(RGB_WHITE); //x축 bar를 지운다
                    for(i = 0; i<4; i++)
                    {
                    LCD_DrawHorLine(50,25+i,70);
                    }
                    for(i = 0; i<4; i++)
                    {
                    LCD_SetPenColor(RGB_RED); //빨간색
                    LCD_DrawHorLine(50,25+i ,Voltage+10); //x축 bar를 그린다.
                    }
                    LCD_SetTextColor(RGB_BLACK);
                     LCD_DisplayText(2,2,"0"); //H:0
                    LCD_DisplayText(2,6,"2"); //C:2

                    GPIOG->ODR &= ~0x00FFf; //LED All off
                    GPIOG->ODR |= 0x0002; //LED1 ON

                    TIM4->CCR1 = 18000; //DR = 90%
                }
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

void TIMER3_Init(void) //타이머3 채널1 CC_Event //ADC1으로 변환시작
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
    TIM3->ARR = 25000-1;	// Auto reload  0.01ms * 25000 = 250ms

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

void TIMER7_Init(void) //모드1의 시계를 동작시키기 위한 타이머
{
      RCC->APB1ENR |= (1<<5);	//RCC_APB1ENR TIMER7 Enable

      NVIC->ISER[1] |= ( 1 << (55-32)); // Enable Timer7 global Interrupt
  //분주 840, period 100ms
      TIM7->PSC = 8400-1;	// Prescaler 84,000,000Hz/8400 = 10,000 Hz (0.1ms)  (1~65536) //-1은 rule임
      TIM7->ARR = 10000-1;	// Auto reload  0.1ms * 10000 = 1000ms
      TIM7->CR1 &= ~(1<<4);	// Upcounter(reset state)
      TIM7->CR1 &= ~(3<<8); 	// CKD(Clock division)=1(reset state)
      TIM7->CR1 &= ~(3<<5); 	// CMS(Center-aligned mode Sel): No(reset state)

      TIM7->EGR |=(1<<0);	//Update generation

      TIM7->DIER |= (1<<0);	//Enable the Tim7 Update interrupt
      TIM7->CR1 |= (1<<0);	//Enable the Tim7Counter
}

void TIMER9_Init(void) //모드2의 스탑워치를 동작시키기 위한 타이머 (ch1로 연결)
{
    RCC->APB2ENR    |= 0x00010000;   // RCC_APB1ENR TIMER3 Enable
    NVIC->ISER[0] |= (1<<24);   //Enable Timer9 global Interrupt

    TIM9->PSC = 16800-1;           // Prescaler 168MHz/16800 = 10000Hz (0.0001s)
    TIM9->ARR = 1000-1;         // Auto reload  : 0.0001s * 1000 = 0.1s(period)
    //TIM9->CR1 &= ~(1<<4);   // DIR: Countermode = Upcounter (reset state)
    TIM9->CR1 &= ~(3<<8);   // CKD: Clock division = 1 (reset state)
    //TIM9->CR1 &= ~(3<<5);    // CMS(Center-aligned mode Sel): No(reset state)

    TIM9->EGR |= (1<<0);        // UG: Update generation
    TIM9->DIER |= (1<<0);   // UIE: Enable Tim9 Update interrupt

    TIM9->CCER |= (1<<0);   // CC1E: OC1 Active
    TIM9->CCER &= ~(1<<1);   // CC1P: OCPolarity_High

    TIM9->EGR |= (1<<1);        // CC1G: Capture/Compare 1 event generation
    TIM9->DIER |=(1<<1); //CC1 interrupt enabled

    TIM9->CCMR1 &= ~(3<<0); // CC1S(CC1 channel): Output
    TIM9->CCMR1 &= ~(1<<3); // OC1PE: Output Compare 1 preload disable
    TIM9->CCMR1 |= (3<<4);   // OC1M: Output Compare 1 Mode : OC mode

    TIM9->CR1 &= ~(1<<7);   // ARPE: Auto reload preload disable
    TIM9->CR1 |= (1<<0);   // CEN: Enable the Tim9 Counter
}

void TIMER4_Init(void) //모드4의 pwm을 동작시키기 위한 타이머 (ch1로 연결)
{
      RCC->AHB1ENR	|= (1<<1);	// RCC_AHB1ENR GPIOB Enable

      GPIOB->MODER 	|= (2<<12);	//중요 GPIOB PIN6 Output Alternate function mode //왜 20000인가
      GPIOB->OSPEEDR 	|= (2<<12);	// GPIOB PIN6 Output speed (100MHz High speed) //보통 25MHz
      GPIOB->OTYPER	= 0x00000000;	// GPIOB PIN6 Output type push-pull (reset state)
      GPIOB->PUPDR	|= (1<<12);	// GPIOB PIN6 Pull-up
      GPIOB->AFR[0]	|= (2<<24);	// AFRH(AFR[1]): Connect PA0 to AF2(TIM3..5)

      RCC->APB1ENR    |= (1<<2);   // RCC_APB1ENR TIMER4 Enable
      NVIC->ISER[0] |= (1<<30);   //Enable Timer4 global Interrupt

      TIM4->PSC = 8400-1;           // Prescaler 84MHz/8400 = 10,000Hz (0.0001s)
      TIM4->ARR = 20000-1;         // Auto reload  : 0.0001s * 20000= 2s(period)
      TIM4->CR1 &= ~(1<<4);   // DIR: Countermode = Upcounter (reset state)
      TIM4->CR1 &= ~(3<<8);   // CKD: Clock division = 1 (reset state)
      TIM4->CR1 &= ~(3<<5);    // CMS(Center-aligned mode Sel): No(reset state)
      TIM4->CCR1 = 0;
      //TIM4->EGR |= (1<<0);        // UG: Update generation
      //TIM4->DIER |= (1<<0);   // UIE: Enable Tim4 Update interrupt

      TIM4->CCER |= (1<<0);   // CC1E: OC1 Active
      TIM4->CCER &= ~(1<<1);   // CC1P: OCPolarity_High

      //TIM4->EGR |= (1<<1);        // CC1G: Capture/Compare 1 event generation
      //TIM4->DIER |=(1<<1); //CC1 interrupt enabled

      TIM4->CCMR1 &= ~(3<<0); // CC1S(CC1 channel): Output
      TIM4->CCMR1 &= ~(1<<3); // OC1PE: Output Compare 1 preload disable
      TIM4->CCMR1 |= (6<<4);   // OC1M: PWM MODE 1

      TIM4->CR1 &= ~(1<<7);   // ARPE: Auto reload preload disable
      TIM4->CR1 |= (1<<0);   // CEN: Enable the Tim4 Counter
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
    GPIOH->MODER    = 0x00000000;	// GPIOH PIN0~PIN15 Input mode (reset state)
    GPIOH->PUPDR    = 0x00000000;	// GPIOH PIN0~PIN15 Floating input (No Pull-up, pull-down) (reset state)

    // Buzzer GPIO(PORT F) 설정
    RCC->AHB1ENR    |= 0x00000020; 	// RCC_AHB1ENR(bit8~0) GPIOF(bit#5) Enable
    GPIOF->MODER    |= 0x00040000;	// GPIOF PIN9 Output mode (0b01)
    GPIOF->OTYPER   &= 0xFDFF;	// GPIOF PIN9 : Push-pull
    GPIOF->OSPEEDR  |= 0x00040000;	// GPIOF PIN9 Output speed (25MHZ Medium speed)

    //NAVI.SW(PORT I) 설정
    RCC->AHB1ENR 	|= 0x00000100;	// RCC_AHB1ENR GPIOI Enable
    GPIOI->MODER 	= 0x00000000;	// GPIOI PIN0~PIN15 Input mode (reset state)
    GPIOI->PUPDR    = 0x11111111;	// GPIOI PIN0~PIN15 Floating input (No Pull-up, pull-down) (reset state)

    RCC->AHB1ENR	|= 0x00000002; 	// RCC_AHB1ENR GPIOB Enable
    GPIOB->MODER 	= 0x00040000;	// GPIOB PIN9 Output mode
    GPIOB->OSPEEDR 	= 0x00040000;	// GPIOB PIN9 Output speed (25MHZ Medium speed)
}

/**/void _EXTI_Init(void)    //EXTI(8, 12 ~ 15) (PH(8, 12 ~ 15), SW(0, 4 ~ 7))
{
    RCC->AHB1ENR    |= 0x80;	// RCC_AHB1ENR GPIOH Enable
    RCC->APB2ENR 	|= 0x4000;	// Enable System Configuration Controller Clock

    GPIOH->MODER 	&=  0x00FF;	// GPIOH PIN8~PIN15 Input mode (reset state)

    SYSCFG->EXTICR[2] |= 0x0007; 	// EXTI8에 대한 소스 입력은 GPIOH로 설정 (EXTICR3) (reset value: 0x0000)
    SYSCFG->EXTICR[3] |= 0x7777;    // EXTI12 ~ 15 에 대한 소스 입력은 GPIOH로 설정 (EXTICR4) (reset value: 0x0000)
    EXTI->FTSR |= 0x0000F100;		// Falling Trigger Enable  (EXTI8, 12 ~ 15 :PH8, 12 ~ 15)
    EXTI->IMR |= 0x0000F100;  	// EXTI11 인터럽트 mask (Interrupt Enable)

    NVIC->ISER[1] |= ( 1 << (40-32) ); // Enable Interrupt EXTI12 ~ 15 Vector table Position 참조
    NVIC->ISER[0] |= ( 1 << 23 ); // Enable Interrupt EXTI8 Vector table Position 참조
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

void StopWatch() //tim9의 stop watch
{
  stop_msec++; //ms증가

      if(stop_msec == 16) //ms가 16일때
      {
        stop_msec = 0; //0으로 초기화
        stop_sec++; //s증가
        if(stop_sec == 16) //s가 16일때
        {
          stop_sec =0; //s 0으로 초기화
          stop_min++; //min 증가
           if(stop_min ==16) //min가 16일때
           {
             stop_min = 0; //min 0으로 초기화
           }
        }
      }
     StopWatch_Display(); //스탑워치 화면출력 함수 실행
}

void StopWatch_Display() //스탑워치 화면출력 함수
{
    char str2[20];
    LCD_SetTextColor(RGB_RED); //글자색은 빨간색

    sprintf(str2, "%1X", stop_msec); //0.1초 단위
    LCD_DisplayText(1,7, str2); //초단위

    sprintf(str2, "%1X", stop_sec); //분
    LCD_DisplayText(1,5, str2);//분

    sprintf(str2, "%1X", stop_min); //초
    LCD_DisplayText(1,4, str2);//초
}

void onTimer()  // 현재 시간 함수
{
    S++;
    if(S == 16) //16일 때
    {
      S = 0; //0으로 초기화
      M++;
      if(M == 16) //16일 때
      {
        M = 0; //0으로 초기화
      }
    }
    onTimerDisplay(); //현재 시각 display 함수

    if((A_S == S) && (A_M == M)) //현재시각과 알람시각이 같아지면 beep음 3번 발생
    {
      BEEP();
      DelayMS(100);
      BEEP();
      DelayMS(100);
      BEEP();
      DelayMS(100);
    }
}

void onTimer2() // 아날로그 시계 현재시각 함수2 (GUI용)
{
  S2++;
  if(S2 == 256)
  {
    S2 = 0;
  }
}

void onTimerDisplay() //현재 시각 display 함수
{
    LCD_SetTextColor(RGB_BLACK);
    LCD_DisplayText(0,15,":");
    sprintf(str, "%1X", M); //분
    LCD_DisplayText(0,14,str);//분
    LCD_SetTextColor(RGB_BLUE);
    sprintf(str, "%1X", S); //초
    LCD_DisplayText(0,16,str);//초
}

void onTimerDisplay2() // 아날로그 시계 현재시각 화면에대한 디스플레이
{
    double barX = 0; //x축 x 좌표
    double barY= 0; //y축 y좌표

    barX = 20*cos( 2*pi*(S2+176)/256 ) +125; //20은 시침의 길이, 뒤의 숫자 125는 xbar가 시작하는 x좌표
    barY =20*sin(2*pi*(S2+176)/256 )+ 75;
    //원으로 생각했을 때 원의 각도는 2pi이므로 이것을 256등분하여
    //시침을 나타낸다. S2변수에 176을 하는 이유는
    //DrawLine의 함수가 1사분면과 4사분면의 양의 x축에서 시작 하기때문이다.
    //이떄 176을 더하면 처음시각인 A분에 위치하게 된다.

    LCD_SetBrushColor(RGB_YELLOW); //노랑색
    LCD_DrawFillRect(100,50,50,50); //시게 겉모양을 그린다.
    LCD_SetPenColor(RGB_BLUE); // 파란색
    LCD_DrawLine(125, 75, barX, barY); //  X축과 Y축을 그린다.
}

void Alarm_Display()
{
      /* **********************
      push버튼을 눌렀을 때
      A_S2에 A_S값을 넣어주고
      A_M2에 A_M값을 넣어준다.
      알람시간을 up down right left로 표현 할 때 디스플레이 되는 것을 방지하기위해
      *************************/

      double barAX = 0; //x축 x좌표
      double barAY= 0; //y축 y좌표

      barAX = 20*cos( 2*pi*(A_S2 + A_M2*16 +192)/256 ) +125; //20은 시침의 길이, 뒤의 숫자 125는 xbar가 시작하는 x좌표
      barAY =20*sin(2*pi*(A_S2 + A_M2*16 +192)/256 )+ 75;
      //원으로 생각했을 때 원의 각도는 2pi이므로 이것을 256등분하여
      //시침을 나타낸다. S2변수에 192을 하는 이유는
      //DrawLine의 함수가 1사분면과 4사분면의 양의 x축에서 시작 하기때문이다.
      //이때 +192를 하면 1사분면과 2사분면의 양의 y축에 위치하게 된다.
      LCD_SetPenColor(RGB_RED); // 빨간색
      LCD_DrawLine(125, 75, barAX, barAY);
}

void Alarm_Mode() //1. 알람 모드 display func
{
    LCD_Clear(RGB_WHITE);
    LCD_SetFont(&Gulim10);		//폰트
    LCD_SetBackColor(RGB_WHITE);	//글자배경색
    LCD_SetTextColor(RGB_GREEN);	//글자색
    LCD_DisplayText(0,0,"1.ALARM");
    LCD_SetTextColor(RGB_BLACK);
    LCD_DisplayText(1,0,"Alarm");
    LCD_SetTextColor(RGB_RED);
    sprintf(str_A_M, "%1X", A_M);
    LCD_DisplayText(1,6, str_A_M);
    sprintf(str_A_S, "%1X", A_S);
    LCD_DisplayText(1,8, str_A_S);
    LCD_DisplayText(1,7,":");
}

void Stop_Watch_Mode() //2. 스탑워치 모드 display func
{
    LCD_Clear(RGB_WHITE);
    LCD_SetFont(&Gulim10);		//폰트
    LCD_SetBackColor(RGB_WHITE);	//글자배경색
    LCD_SetTextColor(RGB_GREEN);	//글자색
    LCD_DisplayText(0,0,"2.Stop Watch");
    LCD_SetTextColor(RGB_BLACK);
    LCD_DisplayText(1,0,"S-W");
    LCD_DisplayText(2,0,"Record");
    LCD_SetTextColor(RGB_RED);
    LCD_DisplayText(1,4,"00:0");
    LCD_DisplayText(2,7,"00:0");
}

void Calculator_Mode() // 3. 계산기 모드 display func
{
    LCD_Clear(RGB_WHITE);
    LCD_SetFont(&Gulim10);		//폰트
    LCD_SetBackColor(RGB_WHITE);	//글자배경색
    LCD_SetTextColor(RGB_GREEN);	//글자색
    LCD_DisplayText(0,0,"3.Calculator");
    LCD_SetTextColor(RGB_BLACK);
    LCD_DisplayText(1,0," 0 + 0 = 00");
}

void Thermostat_Mode() // 4. 온도 모드 display func
{
    LCD_Clear(RGB_WHITE);
    LCD_SetFont(&Gulim10);		//폰트
    LCD_SetBackColor(RGB_WHITE);	//글자배경색
    LCD_SetTextColor(RGB_GREEN);	//글자색
    LCD_DisplayText(0,0,"4.Thermostat");
    LCD_SetTextColor(RGB_BLACK);
    LCD_DisplayText(1,0,"T:23");
    LCD_DisplayText(2,0,"H:");
    LCD_DisplayText(2,4,"C:");
}

void Clear_LCD()
{
    LCD_Clear(RGB_WHITE);
    LCD_SetFont(&Gulim10);		//폰트
    LCD_SetBackColor(RGB_WHITE);	//글자배경색
    LCD_SetTextColor(RGB_GREEN);	//글자색
}
void Clear_ClockDispaly() //아날로그 시계부분을 흰색으로 덮는 함수
{
  //시계 부분을 흰색으로 그려준다.
   LCD_SetBrushColor(RGB_WHITE); //흰색
    LCD_DrawFillRect(100,50,50,50); //시게 겉모양을 그린다.
    double barX = 0; //x축 x 좌표
    double barY= 0; //y축 y좌표

    barX = 20*cos( 2*pi*(S2+176)/256 ) +125;
    barY =20*sin(2*pi*(S2+176)/256 )+ 75;

    LCD_SetPenColor(RGB_WHITE); // 흰색
    LCD_DrawLine(125, 75, barX, barY); //  X축과 Y축을 그린다.

}

uint8_t Navi_key_flag = 0; //키 스캔을 위한 플래그 변수

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
