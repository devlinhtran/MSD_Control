// Header:
// File Name: 
// Author:        	LinhTran
// Mail:		linhtran.ccs@gmail.com
// Company:	Cc-Smart.net
// Date:		7/1/2021


//#include "rs485.h"

#include "main.h"



//struct avariable:

//extern volatile STATE_MECHINE asStateMechine;

GPIO_InitTypeDef GPIO_InitStruct;

void init_buff(void)
{
    RINGBUF_Init(&sRingBuffer, serialRxBuffer,SERIAL_BUFFER_LENGH);

}
//Function:
int8_t Uart1SendData(uint8_t *pData,uint16_t len){



    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;         	//Config pin as Tx mode.
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    if(HAL_UART_Transmit(&huart1, pData, len, 1000)==HAL_BUSY)
    {

    }
//    GPIO_InitStructure.Pull      = GPIO_PULLUP;
//    GPIO_InitStructure.Mode      = GPIO_MODE_INPUT;
//    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
}
/*----------------------------------------------------------------------------
  USART2_IRQHandler
  Handles USART1 global interrupt request.
*----------------------------------------------------------------------------*/
int indexData =0;
uint8_t  termDataBuffer[200];
void Serial_PollingData(void){

    uint8_t c;

    while(RINGBUF_Get(&sRingBuffer,&c) ==0)
    {
        if(indexData >= 200){
            indexData =0;
        }

        if(c == '\r' || c == '\n'){ //new line == new command.
            termDataBuffer[indexData++] = '\n';
            termDataBuffer[indexData++] = 0;
//            asStateMechine.aHostCommunicationSource = CM_RS485;
//            ASCII_COMMAND_Process(termDataBuffer);
            indexData =0;
            break;
        }
        else{
            termDataBuffer[indexData++] = c;
        }
    }
}

// Receive color sensor data
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        RINGBUF_Put(&sRingBuffer, data_uart);
        HAL_UART_Receive_IT(&huart1, &data_uart, 1);

        //        if(asStateMechine.aHostRequestMovingInfor >1){//If sending data with frequency, stop
        //            asStateMechine.aHostRequestMovingInfor = 1;
        //        }
    }

}
