/*
 * raspSerial.c
 *
 *  Created on: 22 may. 2020
 *      Author: sapista
 */

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "raspSerial.h"

// The hidden definition of picom structure
struct picom_t {
	cbuf_handle_t RX_buffer; //RX buffer
	cbuf_handle_t TX_buffer; //TX buffer
	void (*setFader)(uint8_t ch, uint16_t value); //Set Fader Value callback function
};

//Global pointer to the picom object to be used in the USART ISR routine
volatile picom_t* ptr_picom;

picom_handler_t picom_init()
{
	//Init the picom object
	picom_handler_t picom = malloc(sizeof(picom_t));
	picom->RX_buffer = circular_buf_init(PICOM_RX_BUFFER_SIZE);
	picom->TX_buffer = circular_buf_init(PICOM_TX_BUFFER_SIZE);

	//Set the global picom pointer to be used by the ISR routine
	ptr_picom = picom;

	//Configure UART1
	#undef BAUD  // avoid compiler warning
	#define BAUD PICOM_BAUDRATE
	#include <util/setbaud.h>

	// Set baud rate
	UBRR1H = UBRRH_VALUE;
	UBRR1L = UBRRL_VALUE;

	#if USE_2X
		UCSR1A |= (1<<U2X1);
	#else
		UCSR1A &= ~(1<<U2X1);
	#endif

    UCSR1C = (1<<UCSZ11) | (1<<UCSZ10); // 8-bit data
    UCSR1B = (1<<RXEN1) | (1<<TXEN1) | (1<<RXCIE1);   // Enable RX, TX and RXinterrupt

    return picom;
}

void picom_free(picom_handler_t picom)
{
	circular_buf_free(picom->RX_buffer);
	free(picom);
}

void picom_process_RX_queue(picom_handler_t picom)
{
	uint16_t packet;
	uint8_t channel;
	uint8_t value_8bit;
	uint16_t value_16bit;
	uint8_t result;
	uint8_t RX_first_byte, RX_second_byte;

	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		result = circular_buf_get(picom->RX_buffer, &packet);
	}

	if( result == 0 )
	{
		//Decode the message
		RX_first_byte = (uint8_t)(packet >> 8);
		RX_second_byte = (uint8_t)packet;
		if(RX_first_byte & 0x40) //Check first byte
		{
			//Fader-related message
			channel = (RX_first_byte & 0x38) >> 3;
			value_16bit =  (((uint16_t)RX_first_byte & 0x07) << 7 ) | ((uint16_t)RX_second_byte & 0x7F);
			picom->setFader(channel, value_16bit);
		}
		else
		{
			//Non-fader message
			if( RX_first_byte & 0x20 )
			{
				//Encoder/LEDring message
				channel = (RX_first_byte & 0x0E) >> 1;
				value_8bit = (RX_second_byte & 0x7F) | ((RX_first_byte & 0x01) << 7);
				//TODO call the callback!
			}
			else
			{
				//Buttons/LEDs message
				value_8bit = (RX_second_byte & 0x7F) | ((RX_first_byte & 0x01) << 7);
				//TODO decode and call the callback!
			}
		}
	}
}

void picom_add_setFaderValueCallback(picom_handler_t picom, void (*fun_ptr)(uint8_t ch, uint16_t value) )
{
	picom->setFader = fun_ptr;
}

void picom_TXqueue_append_faderValue(picom_handler_t picom, uint8_t fader_id, uint16_t value)
{
	//Prepare the serial packet
	uint16_t packet = 0x4080; //Set just byte markers
	packet |= ((uint16_t)fader_id) << 11;
	packet |= value & 0x7F;
	packet |= (value & 0x380) << 1;

	//Append to the TX queue
	circular_buf_put(picom->TX_buffer, packet);
}

void picom_TXqueue_append_faderUntouched(picom_handler_t picom, uint8_t untouch_flags)
{
	//Prepare the serial packet
	uint16_t packet = 0x0080; //Set just byte markers
	packet |= untouch_flags & 0x7F;
	packet |= (untouch_flags & 0x80) << 1;

	//Append to the TX queue
	circular_buf_put(picom->TX_buffer, packet);
}

#define TX_QUEUE_IDLE 0
#define TX_QUEUE_SENDING_FIRST_BYTE 1
#define TX_QUEUE_SENDING_SECOND_BYTE 2
void picom_process_TX_queue(picom_handler_t picom)
{
	static uint16_t packet;
	static uint8_t tx_queue_state = TX_QUEUE_IDLE;

	if(tx_queue_state == TX_QUEUE_IDLE)
	{
		if(circular_buf_get(picom->TX_buffer, &packet) == 0)
		{
			tx_queue_state = TX_QUEUE_SENDING_FIRST_BYTE;
		}
	}

	if(tx_queue_state == TX_QUEUE_SENDING_FIRST_BYTE && ( UCSR1A & (1<<UDRE1)))
	{
		UDR1 = (uint8_t)((packet & 0xFF00) >> 8);
		tx_queue_state = TX_QUEUE_SENDING_SECOND_BYTE;
	}

	if(tx_queue_state == TX_QUEUE_SENDING_SECOND_BYTE && (UCSR1A & (1<<UDRE1)))
	{
		UDR1 = (uint8_t)(packet & 0x00FF);
		tx_queue_state = TX_QUEUE_IDLE;
	}
}

//USART1 RX complete interrupt
ISR(USART1_RX_vect)
{
	static uint8_t RX_first_byte_set = 0; //Flag if the first byte is already received
	static uint16_t packet; //2 bytes buffer to handle a complete message
	uint8_t dataByte = UDR1; //a place to store the received bytes

	if( dataByte & 0x80 ) //Check the MSB
	{
		//Second byte received, append to circular buffer if first byte is already there
		if(RX_first_byte_set)
		{
			packet |= ((uint16_t)dataByte);
			circular_buf_put(ptr_picom->RX_buffer, packet);
			RX_first_byte_set = 0;
		}
	}
	else
	{
		//First byte received, just keep it till the next iteration
		packet = ((uint16_t)dataByte) << 8;
		RX_first_byte_set = 1;
	}
}
