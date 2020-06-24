/*
 * main.c
 *
 *  Created on: 17 dic. 2017
 *      Author: sapista
 *      comm..
 */


#define F_CPU 16000000UL // 16 MHz

#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <stdio.h>
#include "uart.h"
#include "fader.h"
#include "raspSerial.h"

#define ENCODER_THRESHOLD 8 //Minimum encoder increment to filter out bouncing artefacts
#define REVERSE_ENCODER //Define this to reverse the encoder wiring
volatile int8_t encoder_value = 0;

//INT0 interrupt used for encoder
ISR(INT0_vect )
{
	if(!bit_is_clear(PIND, PD1))
	{
#ifdef REVERSE_ENCODER
		encoder_value++;
		//printf("+");
#else
		encoder_value--;
#endif
	}
	else
	{
#ifdef REVERSE_ENCODER
		encoder_value--;
		//printf("-");
#else
		encoder_value++;
#endif
	}

	encoder_value = encoder_value > 100 ? 100 : encoder_value; //100 to ensure to saturate at 8 bits (sign bit + 7 data bits)
	encoder_value = encoder_value < -100 ? -100 : encoder_value; //-100 to ensure to saturate at 8 bits (sign bit + 7 data bits)
}

//INT1 interrupt used for encoder
ISR(INT1_vect )
{
	if(!bit_is_clear(PIND, PD0))
	{
#ifdef REVERSE_ENCODER
		encoder_value++;
		//printf("+");
#else
		encoder_value--;
#endif
	}
	else
	{
#ifdef REVERSE_ENCODER
		encoder_value--;
		//printf("-");
#else
		encoder_value++;
#endif
	}

	encoder_value = encoder_value > 100 ? 100 : encoder_value; //100 to ensure to saturate at 8 bits (sign bit + 7 data bits)
	encoder_value = encoder_value < -100 ? -100 : encoder_value; //-100 to ensure to saturate at 8 bits (sign bit + 7 data bits)
}



int main()
{
	DDRC |= (1<<PF0); //Testing LED pin as output;


	// Config and init UART
    uart_init();
    stdout = &uart_output;
    stdin = &uart_input;

    //Fader Vars
	uint16_t _targets[NUM_OF_FADERS];
	uint16_t _position[NUM_OF_FADERS];
	uint16_t _touchValues[NUM_OF_FADERS];
	uint8_t faderTouched_ant = 0x00; //Store the previous touch state

	//Initialize all data fields requested by faders
	set_fposition_ptr(_position);
	set_ftarget_ptr(_targets);
	set_touchValues_ptr(_touchValues);

	//Init Faders
	initFaders();
	startMotorController();

	//Init raspberry pi serial communications
	picom_handler_t picom = picom_init();
	picom_add_setFaderValueCallback(picom, setFaderTarget);

	//Encoder initialization
	DDRD &=~ (1 << PD0);				// PD2 and PD3 as input
	DDRD &=~ (1 << PD1);
	PORTD |= (1 << PD0)|(1 << PD1);   // PD0 and PD1 pull-up enabled

	EIMSK |= (1<<INT0)|(1<<INT1);		// enable INT0 and INT1
	EICRA |= (1<<ISC01)|(1<<ISC11)|(1<<ISC10); // INT0 - falling edge, INT1 - reising


	//Global Interrupts enable
	sei();

	calibrateTouchSensors(); //always calibrate touch sensors at the start

    while(1)
	{
/*DEBUG prints
		PORTF |= (1<<PF0); //LED on
		_delay_ms(500);
		if( getTouchedFader() )
		{
			_delay_ms(500);
		}
		PORTF &= ~(1<<PF0); //LED off
		if( !getTouchedFader() )
		{
			_delay_ms(500);
		}


		uint8_t icounttest = 0;
		printf("Int Number test: %d\n", icounttest);
		icounttest++;

		for( uint8_t i = 0; i < NUM_OF_FADERS; i++)
		{
			printf("Fader%u: Pos:%u\tTocuh(%u):%u\n", i+1, _position[i], 0x01&(getTouchedFader()>>i), _touchValues[i]);
		}

*/

		//Process incoming queue
		picom_process_RX_queue(picom);

		//Check for encoder changes to send
		if( encoder_value > ENCODER_THRESHOLD)
		{
			ATOMIC_BLOCK(ATOMIC_FORCEON)
			{
				picom_TXqueue_append_EncoderValue(picom, encoder_value - ENCODER_THRESHOLD);
				encoder_value = 0;
			}
			picom_process_TX_queue(picom); //Try to send at least the first byte
		}

		else if( encoder_value < -ENCODER_THRESHOLD )
		{
			ATOMIC_BLOCK(ATOMIC_FORCEON)
			{
				picom_TXqueue_append_EncoderValue(picom, encoder_value + ENCODER_THRESHOLD);
				encoder_value = 0;
			}
			picom_process_TX_queue(picom); //Try to send at least the first byte
		}

		//Check for fader changes to send
		uint8_t fader_changed = getFaderChanged();
		uint8_t faderTouched = getTouchedFader();
		uint8_t fader_UnTouchFlags = 0;
		for( uint8_t i = 0; i < NUM_OF_FADERS; i++)
		{
			 //Check if current fader has data to send
			if(fader_changed & (1<<i))
			{
				picom_TXqueue_append_faderValue(picom, i, getFaderPosition(i));
				acknowledgeFaderChanged(i); //Avoid re-sending the same data by clearing the change state
				picom_process_TX_queue(picom); //Try to send at least the first byte
			}

			 //Check if current fader has been untouched
			if( (faderTouched_ant & (1<<i)) && !(faderTouched & (1<<i)) )
			{
				//Fader untouched event
				fader_UnTouchFlags |= (1<<i);
			}
		}

		if(fader_UnTouchFlags)
		{
			picom_TXqueue_append_faderUntouched(picom, fader_UnTouchFlags);
			picom_process_TX_queue(picom);
		}

		picom_process_TX_queue(picom); //Ensure second byte is sent
		faderTouched_ant = faderTouched; //Update fader touch ant
	}

	return 0;
}
