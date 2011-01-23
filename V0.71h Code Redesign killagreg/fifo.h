#ifndef _FIFO_H_
#define _FIFO_H_

#include <avr/io.h>
#include <avr/interrupt.h>

// the fifo object
typedef struct
{
	uint8_t volatile count;       // # number of characters in FIFO
	uint8_t size;                 // buffer size
	uint8_t *pread;               // read pointer
	uint8_t *pwrite;              // write pointer
	uint8_t read2end, write2end;  // number of characters for buffer overflow for read/write pointers
} fifo_t;

/*
The initialization of the FIFO sets the read/write pointers etc..
The FIFO uses the buffer 'buf' which byte length must 'size'.
*/
extern void fifo_init (fifo_t*, uint8_t* buf, const uint8_t size);

/*
Puts a byte into the FIFO. Returns 1 on success ans 0 in case of FIFO overflow.
*/
extern uint8_t fifo_put (fifo_t*, const uint8_t data);

/*
Get the next byte out of the FIFO. If the FIFO is empty the function blocks
until the next byte is put into the FIFO.
*/
extern uint8_t fifo_get_wait (fifo_t*);

/*
Get the next byte from the FIFO as int. Returns -1 if the FIFO is empty.
*/
extern int16_t fifo_get_nowait (fifo_t*);


/*
The same like fifo_put
*/
static inline uint8_t _inline_fifo_put (fifo_t *f, const uint8_t data)
{
	if (f->count >= f->size)
		return 0;

	uint8_t * pwrite = f->pwrite;

	*(pwrite++) = data;

	uint8_t write2end = f->write2end;

	if (--write2end == 0)
	{
		write2end = f->size;
		pwrite -= write2end;
	}

	f->write2end = write2end;
	f->pwrite = pwrite;

	uint8_t sreg = SREG;
	cli();
	f->count++;
	SREG = sreg;

	return 1;
}

/*
Get the next byte from FIFO. Before this functionis called
it must be checked that there is a byte in the FIFO to get.
*/
static inline uint8_t _inline_fifo_get (fifo_t *f)
{
	uint8_t *pread = f->pread;
	uint8_t data = *(pread++);
	uint8_t read2end = f->read2end;

	if (--read2end == 0)
	{
		read2end = f->size;
		pread -= read2end;
	}

	f->pread = pread;
	f->read2end = read2end;

	uint8_t sreg = SREG;
	cli();
	f->count--;
	SREG = sreg;

	return data;
}

#endif /* _FIFO_H_ */
