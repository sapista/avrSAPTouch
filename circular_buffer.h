/*
 * circular_buffer.h
 *
 *  Created on: 25 may. 2020
 *      Author: sapista addapted from /embeddedartistry.com/
 */

#ifndef CIRCULAR_BUFFER_H_
#define CIRCULAR_BUFFER_H_
#include <stdint.h>

// Opaque circular buffer structure
typedef struct circular_buf_t circular_buf_t;

// Handle type, the way users interact with the API
typedef circular_buf_t* cbuf_handle_t;

/// Pass in a buffer size
/// Returns a circular buffer handle
cbuf_handle_t circular_buf_init(size_t size);

/// Free a circular buffer structure.
/// Does not free data buffer; owner is responsible for that
void circular_buf_free(cbuf_handle_t cbuf);

/// Reset the circular buffer to empty, head == tail
void circular_buf_reset(cbuf_handle_t cbuf);

/// Put continues to add data if the buffer is full
/// Old data is overwritten
void circular_buf_put(cbuf_handle_t cbuf, uint16_t data);

/// Retrieve a value from the buffer
/// Returns 0 on success, 1 if the buffer is empty
uint8_t circular_buf_get(cbuf_handle_t cbuf, uint16_t * data);

/// Returns true if the buffer is empty
uint8_t circular_buf_empty(cbuf_handle_t cbuf);

/// Returns true if the buffer is full
uint8_t circular_buf_full(cbuf_handle_t cbuf);

/// Returns the maximum capacity of the buffer
uint8_t circular_buf_capacity(cbuf_handle_t cbuf);

/// Returns the current number of elements in the buffer
size_t circular_buf_size(cbuf_handle_t cbuf);

#endif /* CIRCULAR_BUFFER_H_ */
