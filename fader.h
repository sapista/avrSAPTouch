/*
 * fader.h
 *
 *  Created on: 29/12/2011
 *      Author: sapista
 */
#ifndef FADER_H_
#define FADER_H_

#define NUM_OF_FADERS 8

#define PWM_DIR_PORT PORTC
#define PWM_DIR_DDR DDRC
#define MOTOR_ENABLE_PORT PORTB
#define MOTOR_ENABLE_PIN PB5
#define MOTOR_ENABLE_DDR DDRB

//Touch sensors
#define TOUCH_PULSE_PORT PORTA
#define TOUCH_PULSE_PIN  PA3
#define TOUCH_PULSE_DDR  DDRA

#define TOUCH_MUX_PORT   PORTA
#define TOUCH_MUX_PINA   PA0
#define TOUCH_MUX_PINB   PA1
#define TOUCH_MUX_PINC   PA2
#define TOUCH_MUX_DDR	 DDRA

#define TOUCH_CALIBRATE_AVERAGE 10 //Number of measurements to average to calibrate a touch sensors background
#define TOUCH_SENSOR_THRESHOLD 500 //The fader is touched if a touch value above background+threshold is sensed
#define	TOUCH_WAIT_TIMMER_START 49536 //This is to set a touch polling of 1000 Hz (timmer_start_value = 2^16 - 16e6/poll_freq)

#define P_CONTROLLER  2
#define UP_LIMIT 1020 //Mechanical upper position limit
#define DOWN_LIMIT 10 //Mechanical lower position limit
#define DEFAULT_FADER_POS 600 //Initial fader position
#define DUTY_LIMIT_HIGH  500
#define DUTY_LIMIT_LOW 380
#define FADER_DEADBAND 15

//ISR test output pin to measure CPU used by interruptions
//#define ISR_ADC_TEST_PIN //Comment this line to disable adc isr test pin
//#define ISR_TOUCH_TEST_PIN //Comment this line to disable touch-sensor isr test pin
#define ISR_TEST_PORT PORTG
#define ISR_TEST_PIN  PG5
#define ISR_TEST_DDR  DDRG

//initFaders - Initialize the fader controller module
void initFaders();

//Set pointers
void set_ftarget_ptr(uint16_t* ptr);
void set_fposition_ptr(uint16_t* ptr);
void set_touchValues_ptr(uint16_t* ptr);

//startMotorController - Start the ADC Interrupt system to start the motor controller
void startMotorController();

//stopMotorController - Stop the ADC Interrupt system to stop the motor controller
void stopMotorController();

//setFaderTarget - Set a target position for a fader
//iFader - number of fader to set target position
//iTarget - The new target for iFader
void setFaderTarget(uint8_t iFader, uint16_t iTarget);

//getFaderPosition
//iFader - The fader number to get the current position
//Returns - The iFader current position
uint16_t getFaderPosition(uint8_t iFader);

//getFaderTargets
//iFader - The fader number to get the current position
//Returns - The iFader trarget position
uint16_t getFaderTarget(uint8_t iFader);

//getTouchedFader
//Return - A byte containing the touched faders
uint8_t getTouchedFader();

//getFaderChanged
//Return - A byte containing "1" for the faders whose position changed and must be send trough serial interface
uint8_t getFaderChanged();

//acknowledgeFaderChanged
//clear the fader changed bit
void acknowledgeFaderChanged(uint8_t iFader);

//startFollowerTest - Start the follower test setting to 1 the followers flag
void startFollowerTest();

//stopFollowerTest - Stop the follower test setting to 0 the followers flag
void stopFollowerTest();

//calibrateTouchSensors - USe the next TOUCH_CALIBRATE_AVERAGE measurements of each fader to
// calibrate the background of each fader.
// this routine must be exectued with interrupts enabled.
void calibrateTouchSensors();

#endif /* FADER_H_ */
