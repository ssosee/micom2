#define __ACC_C__
#include "ACC.h"
#undef  __ACC_C__

UINT8 SPI1_GetData(int8 *pBuf);
void SPI1_CalData(int8 *pBuf, int16 *pTxBuf);

#define DUMMY   0x00

int8_t SPI_Send(int8_t byte)
{
        /*!< Loop while DR register in not empty */
        while ((SPI1->SR & 0x02) == RESET); //TXE 보낸다
        
        /*!< Send a byte through the SPI1 peripheral */
        SPI1->DR = byte;

        /*!< Wait to receive a byte */
        while ((SPI1->SR & 0x01) == RESET); //RXE
	
        /*!< Return the byte read from the SPI bus */
        SPI1->DR;
}

void ACC_Init(void)
{
        SET_ACC_NSS1();         // NSS : H

        CLR_ACC_NSS1();	        // NSS : L (enable: 통신 시작)
        SPI_Send(CMD_WRITE | CMD_REG_CTRL1); // Master -> Slave(sensor)
                                             // writing a Command to CTRL1(20h) 
        SPI_Send(HR_HIGH_RESOLUTION          // Command
                | ODR_800HZ
                | BDU_CONTINUOUS_UPDATE
                | ZEN_Z_ENABLE  
                | YEN_Y_ENABLE
                | XEN_X_ENABLE);
        SET_ACC_NSS1();         // NSS : H
        UTIL_DelayUS(1);
  
        CLR_ACC_NSS1();	       // NSS : L (enable: 통신 시작)
        SPI_Send(CMD_WRITE | CMD_REG_CTRL2); // Master -> Slave(sensor)
                                             // writing a Command to CTRL2(21h)   
        SPI_Send(ZERO                        // command
                | DFC_ODR_9
		| FDS_FILTER_BYPASSED
		| HPIS1_FILTER_BYPASSED  
		| HPIS2_FILTER_BYPASSED);
        SET_ACC_NSS1();
        UTIL_DelayUS(1);
  
        CLR_ACC_NSS1();	
        SPI_Send(CMD_WRITE | CMD_REG_CTRL3);  
        SPI_Send(0x00);
        SET_ACC_NSS1();
        UTIL_DelayUS(1);
  
        CLR_ACC_NSS1();	
        SPI_Send(CMD_WRITE | CMD_REG_CTRL4); // Master -> Slave(sensor)
                                             // writing a Command to CTRL4(24h) 
        SPI_Send(BW_400HZ 
		| FS_2G
		| BW_SCALE_ODR_AUTOMATICALLY
		| IF_ADD_INC_ENABLE  
		| I2C_DISABLE
		| SIM_4WIRE);
        SET_ACC_NSS1();
        UTIL_DelayUS(1);
  
        CLR_ACC_NSS1();	
        SPI_Send(CMD_WRITE | CMD_REG_CTRL5);  
        SPI_Send(0x00);
        SET_ACC_NSS1();
        UTIL_DelayUS(1);
  
        CLR_ACC_NSS1();	
        SPI_Send(CMD_WRITE | CMD_REG_CTRL6);  
        SPI_Send(0x00);
        SET_ACC_NSS1();
  
        CLR_ACC_NSS1();	
        SPI_Send(CMD_WRITE | CMD_REG_CTRL7);  
        SPI_Send(0x00);
        SET_ACC_NSS1();
  
        UTIL_DelayUS(100);
}

UINT8 SPI1_GetData(int8 *pBuf) //3축에 대한 가속도 값은 읽어옴
{
        CLR_ACC_NSS1();  
        SPI_Send(CMD_READ | CMD_REG_OUT_X_L);// Master <- Slave(sensor)
                                             // reading a data from OUT_X
        pBuf[0] = SPI_Send(DUMMY); //
        pBuf[1] = SPI_Send(DUMMY);
        SET_ACC_NSS1();
        UTIL_DelayUS(1);
  
        CLR_ACC_NSS1();  
        SPI_Send(CMD_READ | CMD_REG_OUT_Y_L);// Master <- Slave(sensor)
                                             // reading a data from OUT_Y
        pBuf[2] = SPI_Send(DUMMY);
        pBuf[3] = SPI_Send(DUMMY);
        SET_ACC_NSS1();
        UTIL_DelayUS(1);
  
        CLR_ACC_NSS1();  
        SPI_Send(CMD_READ | CMD_REG_OUT_Z_L);// Master <- Slave(sensor)
                                             // reading a data from OUT_Z
        pBuf[4] = SPI_Send(DUMMY);
        pBuf[5] = SPI_Send(DUMMY);
        SET_ACC_NSS1();
        UTIL_DelayUS(1);  
  
        return true;
}

void SPI1_Process(int16 *pBuf)
{
        int8 buffer[6]; //6으로 수정했음
  
        SPI1_GetData(&buffer[0]);
  
        SPI1_CalData(&buffer[0],&pBuf[0]);
}

void SPI1_CalData(int8 *pBuf, int16 *pTxBuf) // 16bit data로 전환 //-값을 추력 하고싶으면 절댓값과 U를 지워야함
{
        int16 TempUniX0, TempUniY0, TempUniZ0;

        int16 TempX0, TempY0, TempZ0;

        TempX0 = (int16)(((int16)pBuf[1]<<8) | ((int16)pBuf[0]&0xFF));
        TempY0 = (int16)(((int16)pBuf[3]<<8) | ((int16)pBuf[2]&0xFF));
        TempZ0 = (int16)(((int16)pBuf[5]<<8) | ((int16)pBuf[4]&0xFF));
  
        TempUniX0 = TempX0;
        TempUniY0 = TempY0;
        TempUniZ0 = TempZ0;

        pTxBuf[0] = TempUniX0;
        pTxBuf[1] = TempUniY0;
        pTxBuf[2] = TempUniZ0;
        
}