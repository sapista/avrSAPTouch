/*
 * raspSerial.h
 *
 *  Created on: 22 may. 2020
 *      Author: sapista
 */

#ifndef RASPSERIAL_H_
#define RASPSERIAL_H_

#include "circular_buffer.h"

//Serial Baudrate used for raspberry pi comunication
#define PICOM_BAUDRATE 38400
#define PICOM_RX_BUFFER_SIZE 256
#define PICOM_TX_BUFFER_SIZE 256

//PiCom Messages ID's
#define PICOMID_UNTOUCH	0x00
#define PICOMID_MUTE	0x01
#define PICOMID_SOLO	0x02
#define PICOMID_REC		0x02
#define PICOMID_SELECT	0x04

// Opaque picom structure
typedef struct picom_t picom_t;

// Handle type, the way users interact with the API
typedef picom_t* picom_handler_t;

//Init the Serial1 interface with the PICOM_BAUDRATE, 8 data bits, 1 stop bit, no parity
//Returns a pointer to the picom handler
picom_handler_t picom_init();

//Destroy a picom object
void picom_free(picom_handler_t picom);

// Process incoming messages and call the message corresponding callback
void picom_process_RX_queue(picom_handler_t picom);

// Process outcoming messages and send it to pi
void picom_process_TX_queue(picom_handler_t picom);

// Add setFaderValue callback function
void picom_add_setFaderValueCallback(picom_handler_t picom, void (*fun_ptr)(uint8_t ch, uint16_t value) );
//TODO add the callbacks for all possible incoming messages

// Append a fader value at the TX queue
void picom_TXqueue_append_faderValue(picom_handler_t picom, uint8_t fader_id, uint16_t value);

// Append a fader untouched flags at the TX queue
void picom_TXqueue_append_faderUntouched(picom_handler_t picom, uint8_t untouch_flags);

#endif /* RASPSERIAL_H_ */
