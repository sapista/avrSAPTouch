/*
 * fader.c
 *
 *  Created on: 29/12/2011
 *      Author: sapista
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "fader.h"

volatile uint16_t *ftarget;
volatile uint16_t *fposition;
volatile uint16_t *touchValues; //Pointer to a 8-element vectors to store the last touch measurements
volatile uint8_t touched; //One bit for each fader touch sensor 1 is touched
volatile uint8_t changed; //One bit for each fader indicating if the value has changed

volatile uint8_t AdcStarted = 0; //Adc state
volatile uint8_t Follower_flag = 0; //The follower test flag
volatile uint16_t *pwm_ptr[NUM_OF_FADERS]; //Pointers to OCR register to fast access from ADC ISR routine
volatile uint8_t *tccra_ptr[NUM_OF_FADERS]; //Pointer to TCCRnA registers to disconnect OCR pins form inside the touch isr
volatile uint8_t  tccra_com_bits[NUM_OF_FADERS]; //The bit that controls the OCR output pin of each pwm channel
volatile uint8_t *pwm_port[NUM_OF_FADERS]; //Pointer where the pwm outputs are connected
volatile uint8_t pwm_pin[NUM_OF_FADERS]; //Pins where the pwm outputs are connected

volatile uint16_t touchBackground[NUM_OF_FADERS]; //Stores the background value (no-touched) for each fader
volatile uint8_t touchCalibrationCounter; //Used to determine if touch calibration is taking place (grater than zero)

uint16_t adcValue;
uint8_t adcCh;
uint8_t faderPositioned[NUM_OF_FADERS]; //Fader is positioned at target if 1

//initFaders - Initialize the fader controller module
void initFaders()
{
	//Init motor controller port pointers
	pwm_ptr[0] = &OCR3B;
	pwm_ptr[1] = &OCR3C;
	pwm_ptr[2] = &OCR4A;
	pwm_ptr[3] = &OCR4B;
	pwm_ptr[4] = &OCR4C;
	pwm_ptr[5] = &OCR5A;
	pwm_ptr[6] = &OCR5B;
	pwm_ptr[7] = &OCR5C;

	tccra_ptr[0] = &TCCR3A;
	tccra_ptr[1] = &TCCR3A;
	tccra_ptr[2] = &TCCR4A;
	tccra_ptr[3] = &TCCR4A;
	tccra_ptr[4] = &TCCR4A;
	tccra_ptr[5] = &TCCR5A;
	tccra_ptr[6] = &TCCR5A;
	tccra_ptr[7] = &TCCR5A;

	tccra_com_bits[0] = COM3B1;
	tccra_com_bits[1] = COM3C1;
	tccra_com_bits[2] = COM4A1;
	tccra_com_bits[3] = COM4B1;
	tccra_com_bits[4] = COM4C1;
	tccra_com_bits[5] = COM5A1;
	tccra_com_bits[6] = COM5B1;
	tccra_com_bits[7] = COM5C1;

	pwm_port[0] = &PORTE;
	pwm_port[1] = &PORTE;
	pwm_port[2] = &PORTH;
	pwm_port[3] = &PORTH;
	pwm_port[4] = &PORTH;
	pwm_port[5] = &PORTL;
	pwm_port[6] = &PORTL;
	pwm_port[7] = &PORTL;

	pwm_pin[0] = PE4;
	pwm_pin[1] = PE5;
	pwm_pin[2] = PH3;
	pwm_pin[3] = PH4;
	pwm_pin[4] = PH5;
	pwm_pin[5] = PL3;
	pwm_pin[6] = PL4;
	pwm_pin[7] = PL5;

	//Initialize all data fields
	for(uint8_t i = 0; i<NUM_OF_FADERS; i++)
	{
		ftarget[i]= DEFAULT_FADER_POS;
		faderPositioned[i] = 0;
		*pwm_ptr[0] = 0;
	}

	touched = 0;

	//Config pins for the touch sensors
	TOUCH_PULSE_DDR |= (1 << TOUCH_PULSE_PIN);
	TOUCH_MUX_DDR |= (1 << TOUCH_MUX_PINA) | (1 << TOUCH_MUX_PINB) | (1 << TOUCH_MUX_PINC);
	TOUCH_MUX_PORT &= ~((1<<TOUCH_MUX_PINA) | (1<<TOUCH_MUX_PINB) | (1<<TOUCH_MUX_PINC));

	//Config Pins
	DDRE |= (1<<PE4) | (1<<PE5); //PWM0, PWM1
	DDRH |= (1<<PH3) | (1<<PH4) | (1<<PH5); //PWM2, PWM3, PWM4
	DDRL |= (1<<PL3) | (1<<PL4) | (1<<PL5); //PWM5, PWM6, PWM7
	PWM_DIR_DDR = 0xFF; //Direction pins
	PWM_DIR_PORT = 0;

	//PWM on TCCR 3,4,5: Fast PWM 9-bits and config OCnN pins to clear on compare match
	TCCR3A = (1<<COM3B1) | (1<<COM3C1) | (1<<WGM31);
	TCCR4A = (1<<COM4A1) | (1<<COM4B1) | (1<<COM4C1) | (1<<WGM41);
	TCCR5A = (1<<COM5A1) | (1<<COM5B1) | (1<<COM5C1) | (1<<WGM51);

	//PWM on TCCR 3,4,5: Fast PWM 9-bits and prescaler = 1 => pwm of  31.25 kHz
	TCCR3B = (1<<WGM32) | (1<<CS30);
	TCCR4B = (1<<WGM42) | (1<<CS40);
	TCCR5B = (1<<WGM52) | (1<<CS50);

	//Config ADC
	ADMUX = (1<<REFS0); //external AVCC ref, AD8 input selected
	ADCSRA = (1<<ADPS2) | (1<<ADPS1) | (1<<ADIE); //Prescaler to 64 to get a sample rate of 19,2 kHz (2,4 kHz for each fader)
												  //This sample rate corresponds to a CPU usage of 18%
	ADCSRB = (1<<MUX5); //MUX5 bit must be always 1 to use ADC channels 8 to 15, then only MUX4:0 is used

	//Disable digital input buffer for ADC pins
	DIDR0 = 0; //No fader is connected to ADC channels 0 to 7
	DIDR2 = 0xFF; //All 8 faders are connected to to ADC channels 8 to 15

	//Touch sensors using analog comparator and timer1 input capture mode
	ACSR |= (1<<ACIC)| (1<<ACIS1); //Analog comparator connected t timer1 and triggered by falling edge.
	DIDR1 |= (1<<AIN1D) | (1<<AIN0D); //Disable digital input buffers at analog inputs of the comparator.

	//Touch sensor using input capture feature of timer1 (16 bits mode)
	TCCR1B = (1<<ICNC1) | (1<<CS10); //Use a prescaler of 1 for the measurment to obtain high precission
	TIMSK1 |= (1<<TOIE1); //Overflow interrupt enable
	TCNT1 = 0;
	TOUCH_PULSE_PORT |= (1 << TOUCH_PULSE_PIN); //Pulse output ON to disable touch measurement till next timer overflow

	//ISR test output pin to measure CPU used by interruptions
	ISR_TEST_DDR = (1<<ISR_TEST_PIN);
	ISR_TEST_PORT &= ~(1<<ISR_TEST_PIN);

	//Motor Enable pin (motor is enabled by setting output to zero)
	MOTOR_ENABLE_DDR |= (1<<MOTOR_ENABLE_PIN);
	MOTOR_ENABLE_PORT |= (1<<MOTOR_ENABLE_PIN); //Initially set to one to dissable the motors
}

//Set ftarget pointer
void set_ftarget_ptr(uint16_t* ptr)
{
	ftarget = ptr;
}

//Set fposition pointer
void set_fposition_ptr(uint16_t* ptr)
{
	fposition = ptr;
}

//Set touchValues pointer
void set_touchValues_ptr(uint16_t* ptr)
{
	touchValues = ptr;
}

//startMotorController - Start the ADC Interrupt system to start the motor controller
void startMotorController()
{
	AdcStarted = 1;
	ADCSRA |=  (1<<ADEN) | (1<<ADSC); //Enable motor controller
	MOTOR_ENABLE_PORT &= ~(1<<MOTOR_ENABLE_PIN); //Enable motors drivers
}

//stopMotorController - Stop the ADC Interrupt system to stop the motor controller
void stopMotorController()
{
	AdcStarted = 0;
	ADCSRA &=  ~(1<<ADEN); //Disable motor controller
	for(uint8_t i=0; i< NUM_OF_FADERS; i++)
	{
		*pwm_ptr[i] = 0;
		PWM_DIR_PORT = 0;
	}
	MOTOR_ENABLE_PORT |= (1<<MOTOR_ENABLE_PIN); //Disable motor controllers
}

//setFaderTarget - Set a target position for a fader
//iFader - number of fader to set target position
//iTarget - The new target for iFader
void setFaderTarget(uint8_t iFader, uint16_t iTarget)
{
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		ftarget[iFader] = iTarget;
		faderPositioned[iFader] = 0; //Fader is not positioned!
	}
}

//getFaderPosition
//iFader - The fader number to get the current position
//Returns - The iFader current position
uint16_t getFaderPosition(uint8_t iFader)
{
	uint16_t fpos;
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		fpos = fposition[iFader];
	}
	return fpos;
}

//getFaderTargets
//iFader - The fader number to get the current position
//Returns - The iFader trarget position
uint16_t getFaderTarget(uint8_t iFader)
{
	uint16_t fpos;
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		fpos =  ftarget[iFader];
	}
	return fpos;
}

//getTouchedFader
//Return - A byte containing the touched faders
uint8_t getTouchedFader()
{
	return touched;
}

//getFaderChanged
//Return - A byte containing "1" for the faders whose position changed and must be send trough serial interface
uint8_t getFaderChanged()
{
	return changed;
}

//acknowledgeFaderChanged
//clear the fader changed bit
void acknowledgeFaderChanged(uint8_t iFader)
{
	changed &= ~(1<<iFader);
}

//startFollowerTest - Start the follower test setting to 1 the followers flag
void startFollowerTest()
{
	Follower_flag = 1;
}

//stopFollowerTest - Stop the follower test setting to 0 the followers flag
void stopFollowerTest()
{
	Follower_flag = 0;
}
//calibrateTouchSensors - Use the next TOUCH_CALIBRATE_AVERAGE measurements of each fader to
// calibrate the background of each fader.
void calibrateTouchSensors()
{
	for(uint8_t i = 0; i<NUM_OF_FADERS; i++)
	{
		touchBackground[i] = 0;
	}
	touchCalibrationCounter = TOUCH_CALIBRATE_AVERAGE * NUM_OF_FADERS;
	touched = 0;
}

//Interrupt for ADC convertion
ISR(ADC_vect)
{
	//ISR test pin
	#ifdef ISR_ADC_TEST_PIN
	ISR_TEST_PORT |= (1<<ISR_TEST_PIN);
	#endif

	int16_t duty = 0;
	adcValue = ADC;
	adcCh = ADMUX & 0x1F;
	fposition[adcCh] = adcValue;

	//Set next ADC Ch
	uint8_t adcCh_next = (adcCh + 1)%NUM_OF_FADERS;
	ADMUX &= 0b11100000;
	ADMUX |= adcCh_next;



	//Limits for Faders position
	ftarget[adcCh] = ftarget[adcCh] > UP_LIMIT ? UP_LIMIT : ftarget[adcCh];
	ftarget[adcCh] = ftarget[adcCh] < DOWN_LIMIT ? DOWN_LIMIT : ftarget[adcCh];

    //PID controller and refresh touched fader value
	int16_t pError = ftarget[adcCh] - adcValue;
	if( pError > FADER_DEADBAND || pError < -FADER_DEADBAND )
	{
		//Set new target value if touched
		if((touched & (1<<adcCh)))
		{
			ftarget[adcCh] = adcValue ;
			changed |= (1<<adcCh); //Signal data to send trough serial port
		}

		//P controller
		duty = P_CONTROLLER * pError;

		//Duty limits low
		duty = duty < DUTY_LIMIT_LOW && duty > 0 ? DUTY_LIMIT_LOW : duty;
		duty = duty > -DUTY_LIMIT_LOW && duty < 0 ? -DUTY_LIMIT_LOW : duty;

		//Duty limits high
		duty = duty > DUTY_LIMIT_HIGH ? DUTY_LIMIT_HIGH : duty;
		duty = duty < -DUTY_LIMIT_HIGH ? -DUTY_LIMIT_HIGH : duty;
	}

	//Set duty to OCR registers using 9-bits encoding thus... 511
	*pwm_ptr[adcCh] = (duty>>15)*511 + duty; //If is positive (duty>>15) is zero so multiplication has no effect.

	//Set direction pin depending whether duty is positive or negative
	uint8_t aux = PWM_DIR_PORT;
	aux &= ~(1<<adcCh);
	aux |= ( (0x01&(duty>>15))<<adcCh);
	aux &= ~touched;
	PWM_DIR_PORT = aux;

	//Start the next convertion
	ADCSRA |= (1<<ADSC);

	//ISR test pin
	#ifdef ISR_ADC_TEST_PIN
	ISR_TEST_PORT &= ~(1<<ISR_TEST_PIN);
	#endif
}


ISR(TIMER1_OVF_vect)
{
	//ISR test pin
	#ifdef ISR_TOUCH_TEST_PIN
	ISR_TEST_PORT |= (1<<ISR_TEST_PIN);
	#endif

	if( TOUCH_PULSE_PORT & (1 << TOUCH_PULSE_PIN) )
	{
		//Touch pulse output is ON so pulse is not being sended, just start a new measurement
		TOUCH_PULSE_PORT &= ~(1 << TOUCH_PULSE_PIN); //Pulse output OFF to start next input capture event
		TCNT1 = 0;
	}
	else
	{
		//Touch pulse output is OFF so this overlow must caputre the touch measuremnt
		uint8_t touch_channel = TOUCH_MUX_PORT & ((1<<TOUCH_MUX_PINA) | (1<<TOUCH_MUX_PINB) | (1<<TOUCH_MUX_PINC));

		if (TIFR1 & (1 << ICF1))
		{
			//Input capture detected
			touchValues[touch_channel] = ICR1;
			TIFR1 |= (1 << ICF1); //Clear the input capture interrupt flag
		}
		else
		{
			//Timer overflow without input caputre, thus fader is touched for safety reasons
			touchValues[touch_channel] = 0xFFFF;
		}

		if( touchCalibrationCounter > 0)
		{
			//Calibration is taking place...
			touchBackground[touch_channel] += (touchValues[touch_channel] / TOUCH_CALIBRATE_AVERAGE);
			touchCalibrationCounter--;
		}
		else
		{
			//Set the touch register
			if(touchValues[touch_channel]  > (TOUCH_SENSOR_THRESHOLD + touchBackground[touch_channel]))
			{
				touched |= (1<<touch_channel);

				//Disable motor PWM output, dir pin is also set to zero in the motor controller
				*tccra_ptr[touch_channel] &= ~(1<<tccra_com_bits[touch_channel]);
				*pwm_port[touch_channel] &= ~(1<< pwm_pin[touch_channel]);
				PWM_DIR_PORT &= ~(1<<touch_channel);
			}
			else
			{
				touched &= ~(1<<touch_channel);

				//Not touched, enable motor
				*tccra_ptr[touch_channel] |= (1<<tccra_com_bits[touch_channel]);
			}
		}

		TCNT1 = TOUCH_WAIT_TIMMER_START; //Reset timmer to higher value to produce a sample rate of 40Hz approx

		//Config the touch MUX for the next run
		touch_channel = (touch_channel + 1 )%NUM_OF_FADERS;
		TOUCH_MUX_PORT &= ~((1<<TOUCH_MUX_PINA) | (1<<TOUCH_MUX_PINB) | (1<<TOUCH_MUX_PINC));
		TOUCH_MUX_PORT |= touch_channel;

		//Pulse output ON to dissable the touch measurement
		TOUCH_PULSE_PORT |= (1 << TOUCH_PULSE_PIN);
	}

	//ISR test pin
	#ifdef ISR_TOUCH_TEST_PIN
	ISR_TEST_PORT &= ~(1<<ISR_TEST_PIN);
	#endif
}
