/**
* \file
*		Ring Buffer library
* \author
*		Thanh M. Phan <thanhpm@mttc.vn>
*/
#include "main.h"

//#define NULL					(0)
#define	SYS_EnterCritical()		 __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE)
#define	SYS_LeaveCritical()		 __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE)

/**
* \brief init a RINGBUF object
* \param r pointer to a RINGBUF object
* \param buf pointer to a byte array
* \param size size of buf
* \return 0 if successfull, otherwise failed
*/
void RINGBUF_Init(RINGBUF *r, uint8_t* buf, uint16_t size)
{
	r->p_o = r->p_r = r->p_w = buf;
	r->fill_cnt = 0;
	r->size = size;
}
/**
* \brief put a character into ring buffer
* \param r pointer to a ringbuf object
* \param c character to be put
* \return 0 if successfull, otherwise failed
*/
void RINGBUF_Put(RINGBUF *r, uint8_t c)
{
	if(r->fill_cnt>=r->size){
		return;								// ring buffer is full, this should be atomic operation
	}
	
	SYS_EnterCritical();					// enter critical session
	r->fill_cnt++;							// increase filled slots count, this should be atomic operation
	
	*r->p_w++ = c;							// put character into buffer
	
	if(r->p_w >= r->p_o + r->size)			// rollback if write pointer go pass
		r->p_w = r->p_o;					// the physical boundary
	
	SYS_LeaveCritical();					// leave critical session
}
/**
* \brief get a character from ring buffer
* \param r pointer to a ringbuf object
* \param c read character
* \return 0 if successfull, otherwise failed
*/
int8_t RINGBUF_Get(RINGBUF *r, uint8_t* c)
{
	if(r->fill_cnt<=0){
		return -1;								// ring buffer is empty, this should be atomic operation
	}
	
	SYS_EnterCritical();						// enter critical session
	
	r->fill_cnt--;								// decrease filled slots count
	
	SYS_LeaveCritical();						// leave critical session
	
	*c = *r->p_r++;								// get the character out
	
	if(r->p_r >= r->p_o + r->size)				// rollback if write pointer go pass
		r->p_r = r->p_o;						// the physical boundary
	
	return 0;
}
