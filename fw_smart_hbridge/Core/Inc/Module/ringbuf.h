#ifndef _RING_BUF_H_
#define _RING_BUF_H_


typedef struct{
	uint8_t* p_o;				/**< Original pointer */
	uint8_t* p_r;		/**< Read pointer */
	uint8_t* p_w;		/**< Write pointer */
	uint8_t fill_cnt;	/**< Number of filled slots */
	uint8_t size;				/**< Buffer size */
}RINGBUF;

void RINGBUF_Init(RINGBUF *r, uint8_t* buf, uint16_t size);
void RINGBUF_Put(RINGBUF *r, uint8_t c);
int8_t RINGBUF_Get(RINGBUF *r, uint8_t* c);
#endif
