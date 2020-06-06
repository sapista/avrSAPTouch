/*
 * circular_buffer.c
 *
 *  Created on: 25 may. 2020
 *      Author: sapista addapted from /embeddedartistry.com/
 */

#include <stdlib.h>
#include "circular_buffer.h"

// The hidden definition of our circular buffer structure
struct circular_buf_t {
	uint16_t * buffer; //pointer to the actual buffer
	uint8_t head; //buffer write index, max is 255 since a uint8_t data type is used
	uint8_t tail; //buffer read index, max is 255 since a uint8_t data type is used
	uint8_t max; //of the buffer
	uint8_t full; //1 if full, 0 otherwise
};

cbuf_handle_t circular_buf_init(size_t size)
{
	if(size > 256)
	{
		//ERROR, the max buffer size is 256 since we are using uint8_t to index its elements
		return NULL;
	}

	cbuf_handle_t cbuf = malloc(sizeof(circular_buf_t));

	cbuf->buffer = malloc(sizeof(uint16_t)*size);
	cbuf->max = size;
	circular_buf_reset(cbuf);

	return cbuf;
}

void circular_buf_free(cbuf_handle_t cbuf)
{
	free(cbuf->buffer);
	free(cbuf);
}

void circular_buf_reset(cbuf_handle_t cbuf)
{
    cbuf->head = 0;
    cbuf->tail = 0;
    cbuf->full = 0;
}

static void advance_pointer(cbuf_handle_t cbuf)
{
	if(cbuf->full)
   	{
		cbuf->tail = (cbuf->tail + 1) % cbuf->max;
	}

	cbuf->head = (cbuf->head + 1) % cbuf->max;
	cbuf->full = (cbuf->head == cbuf->tail);
}

static void retreat_pointer(cbuf_handle_t cbuf)
{
	cbuf->full = 0;
	cbuf->tail = (cbuf->tail + 1) % cbuf->max;
}

void circular_buf_put(cbuf_handle_t cbuf, uint16_t data)
{
    cbuf->buffer[cbuf->head] = data;
    advance_pointer(cbuf);
}

uint8_t circular_buf_get(cbuf_handle_t cbuf, uint16_t * data)
{
	int r = 1;

	if(!circular_buf_empty(cbuf))
	{
		*data = cbuf->buffer[cbuf->tail];
		retreat_pointer(cbuf);

		r = 0;
	}

	return r;
}

uint8_t circular_buf_empty(cbuf_handle_t cbuf)
{
	return (!cbuf->full && (cbuf->head == cbuf->tail));
}

uint8_t circular_buf_full(cbuf_handle_t cbuf)
{
	return cbuf->full;
}

uint8_t circular_buf_capacity(cbuf_handle_t cbuf)
{
	return cbuf->max;
}

size_t circular_buf_size(cbuf_handle_t cbuf)
{
	size_t size = cbuf->max;

	if(!cbuf->full)
	{
		if(cbuf->head >= cbuf->tail)
		{
			size = (cbuf->head - cbuf->tail);
		}
		else
		{
			size = (cbuf->max + cbuf->head - cbuf->tail);
		}
	}

	return size;
}

