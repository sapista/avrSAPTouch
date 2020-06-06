/*
 * main.c
 *
 *  Created on: 17 dic. 2017
 *      Author: sapista
 */


#define F_CPU 16000000UL // 16 MHz

#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <stdio.h>
#include "uart.h"
#include "fader.h"
#include "raspSerial.h"


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

	//Global Interrupts enable
	sei();

	calibrateTouchSensors(); //always calibrate touch sensors at the start

    //uint8_t icounttest = 0;
	while(1)
	{
		/*
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


		printf("Int Number test: %d\n", icounttest);
		icounttest++;

		for( uint8_t i = 0; i < NUM_OF_FADERS; i++)
		{
			printf("Fader%u: Pos:%u\tTocuh(%u):%u\n", i+1, _position[i], 0x01&(getTouchedFader()>>i), _touchValues[i]);
		}
		*/

		//Process incoming queue
		picom_process_RX_queue(picom);

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
