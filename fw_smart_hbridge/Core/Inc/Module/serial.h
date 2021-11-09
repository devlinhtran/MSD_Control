// Header:
// File Name: 
// Author:		LinhTran
// Date:			12/01/2021
#ifndef _SERIAL_H_
#define _SERIAL_H_

#define UART_TX_PIN		GPIO_PIN_2
#define UART_TX_PORT	GPIOA

#define UART_RX_PIN		GPIO_PIN_3
#define UART_RX_PORT	GPIOA

#define SERIAL_BUFFER_LENGH	200


void UART1_RxDataIRQ(void);
int8_t Uart1SendData(uint8_t *pData,uint16_t len);
void Serial_PollingData(void);
void init_buff(void);
#endif
