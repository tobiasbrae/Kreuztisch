// standard libraries
#include <avr/io.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <math.h>

// self-made libraries
#include "port_configuration.h"
#include "circular_buffer.h"
#include "simulation_elements.h"

// ==================================== [basic defines] ==========================================

// switch message-types on or off
#define ERRORS 1
#define WARNINGS 1
#define DEBUGS 0
#define ECHO 0
#define CALLS 0

// timers
#define TIME_STEP 0.0005

#define NUM_TIMES 5
#define T_RUNTIME 0
#define T_CAL_WAIT 1
#define T_MOTOR_CHECK 2
#define T_TARGET_DELAY 3
#define T_DEBUGS 4

#define X 0
#define Y 1
#define Z 2

// ==================================== [hardware configuration] ==========================================

// motors ============================
#define NUM_MOTORS 3

#define MOT_X_INVERTED 0
#define MOT_Y_INVERTED 1
#define MOT_Z_INVERTED 0

#define VAL_PER_MM 100 // for the Servo

// checkMotors
#define MIN_POWER 60 // the minimum power for checking the motor
#define MIN_SPEED 1.0  // the minimum speed, the motor should have
#define MAX_TIME 4.0 // the max time, this state is enabled

// encoders ==========================

#define NUM_AXIS 2

// handling channels
#define A 0
#define B 1
#define A_NEW 0
#define B_NEW 1
#define A_OLD 2
#define B_OLD 3
#define RISING 1
#define FALLING 2

#define AXIS_X_INVERTED 0
#define AXIS_Y_INVERTED 1

#define STEPS_PER_MM 34 // 15ZähneProUmdrehung*2.032mmProZahn/1024SchritteProUmdrehung

// ==================================== [limits] ==========================================

#define MIN_VELOCITY 100
#define MAX_VELOCITY 800
#define MIN_X_AXIS 0
#define MAX_X_AXIS 10000
#define MIN_Y_AXIS 0
#define MAX_Y_AXIS 10000
#define MIN_Z_AXIS 0
#define MAX_Z_AXIS 10

#define MAX_MOT_X 255
#define MAX_MOT_Y 255

#define MIN_MOT_Z 128.0
#define MAX_MOT_Z 300.0
#define OFF_MOT_Z 200.0

// ==================================== [controlling] ==========================================

// targets ======================
#define COORDINATE 1
#define DELAY 2

typedef struct
{
	uint8_t type;
	int16_t x;
	int16_t y;
	int16_t z;
	int16_t vel;
}target;

// calibration ==================
#define CAL_VEL_X -50 // the moving speed of the x-axis
#define CAL_VEL_Y -50 // the moving speed of the y-axis
#define CAL_TIME 1 // the time the motor needs to be standing
#define CAL_TOL 1 // area in mm which is considered as "not moving" for the calibration
#define CAL_TAR_X 0 // the position of the limiters on the x-axis
#define CAL_TAR_Y 0 // the position of the limiters on the y-axis

// controlling - settings ================

#define SAMPLE_TIME 0.005
#define PRECISION 1.0 // the mm the tool is allowed to be away from the target
#define TARGET_WAIT 0.2 // the time, the tool has to be in the target area to be confirmed

#define MIN_MOV_SPEED 5
#define ACCELERATION 20

#define AXIS_X_KP 0.8 // parameter for the speed regulation
#define AXIS_X_KI 1.0 // parameter for the speed regulation
#define AXIS_X_KD 0.0 // parameter for the speed regulation
#define AXIS_X_FAC 1.9

#define AXIS_Y_KP 0.8 // parameter for the speed regulation
#define AXIS_Y_KI 1.0 // parameter for the speed regulation
#define AXIS_Y_KD 0 // parameter for the speed regulation
#define AXIS_Y_FAC 1.7

#define MAX_ESUM 4000 // parameter for the speed regulation
#define AXIS_T1 0.3 // time constant for filtering the velocity

// controllers ============================

#define NUM_CONTROLLERS 22

#define C_X_VEL      0
#define C_Y_VEL      1
#define C_X_VEL_FIL  2
#define C_Y_VEL_FIL  3
#define C_X_E_POS    4
#define C_Y_E_POS    5
#define C_E_POS      6
#define C_VEL_TAR    7
#define C_VEL_DIV    8
#define C_X_VEL_MUL  9
#define C_Y_VEL_MUL 10
#define C_X_E_VEL   11
#define C_Y_E_VEL   12
#define C_X_MOT_PID 13
#define C_Y_MOT_PID 14
#define C_X_MOT_MUL 15
#define C_Y_MOT_MUL 16
#define C_X_MOT_SUM 17
#define C_Y_MOT_SUM 18
#define C_X_MOT_LIM 19
#define C_Y_MOT_LIM 20
#define C_Z_MOT_PID 21

// ==================================== [buffers] ==========================================

#define NUM_BUFFERS 3
#define INDATA 0
#define OUTDATA 1
#define TARGETS 2

#define INOUT_BUFF_SIZE 150
#define TARGET_BUFF_SIZE 100
#define STRING_BUFF_SIZE 20

// ==================================== [variables] ==========================================

uint64_t timeCounter; // gets incremented via timer-interrupt
double time[NUM_TIMES]; // stores time as double for different purposes

double targetCoordinates[NUM_MOTORS]; // stores the current target position as double

uint8_t motorState = 0; // stores whether the motors are switched on or off
uint8_t invertedMotor[NUM_MOTORS]; // option to invert the motor rotation direction
uint16_t motorBuffer[NUM_MOTORS]; // buffer for setting the power; update at next overflow

volatile uint8_t encoderState[NUM_AXIS][4]; // stores the encoder-state
uint8_t invertedEncoder[NUM_AXIS]; // option to invert the encoder direction
int32_t encoderValue[NUM_AXIS]; // stores the increments of the encoders
double axisValue[NUM_AXIS]; // stores the position in mm

sim_object *controllers[NUM_CONTROLLERS]; // PID-controllers
double outputs[NUM_CONTROLLERS]; // outputs for controlling
double motorMulti[NUM_AXIS]; // motor-controlling...
double acceleration; // stores the target acceleration for moving around
uint32_t controlCounter; // counts the number of calculations
double elog[NUM_MOTORS]; // logs the difference from target speed to actual speed

buffer *buffers[NUM_BUFFERS]; // stores the buffers
char stringBuffer[STRING_BUFF_SIZE]; // buffer for formatting strings

uint8_t isControlling; // stores whether the controllers are currently running or not
uint8_t isSending; // indicates if the output buffer is currently being emptied

// ==================================== [port configuration] ==========================================

	portpin motorZpwm = { &DDRB, &PORTB, &PINB, 4 }; // the pwm-output-pin for the motor Z
	portpin motorXpwm = { &DDRB, &PORTB, &PINB, 5 }; // the pwm-output-pin for the motor X
	portpin motorYpwm = { &DDRB, &PORTB, &PINB, 6 }; // the pwm-output-pin for the motor Y
	
	portpin encoder[NUM_AXIS][2] = // the encoder-input pins
	{
		{ {&DDRD, &PORTD, &PIND, 0}, {&DDRD, &PORTD, &PIND, 1} },
		{ {&DDRD, &PORTD, &PIND, 2}, {&DDRD, &PORTD, &PIND, 3} }
	};
	
	portpin motorXpin1 = { &DDRF, &PORTF, &PINF, 0 }; // used to contol the motor controller
	portpin motorXpin2 = { &DDRF, &PORTF, &PINF, 1 }; // used to contol the motor controller
	portpin motorYpin1 = { &DDRF, &PORTF, &PINF, 2 }; // used to contol the motor controller
	portpin motorYpin2 = { &DDRF, &PORTF, &PINF, 3 }; // used to contol the motor controller
	
	portpin timer1CS10 = { 0, &TCCR1B, 0, CS10 }; // for switching the pwm-timer off and on
	portpin timer1CS11 = { 0, &TCCR1B, 0, CS11 }; // for switching the pwm-timer off and on

// ==================================== [function declaration] ==========================================

void initialize(); // setting the timers, uart, etc.
void createControllers(); // create the controllers
void calibrate(); // calibrate the encoders
void updateTime(void); // turns the increments into seconds (double-operation)
double getTime(uint8_t timer); // returns the runtime in seconds
double getTimeDiff(uint8_t timer); // returns the time since the last reset for the given timer
void resetTime(uint8_t timer); // resets the time for the given timer
void handleData(); // checks the received data for valid commands
void setControlling(uint8_t active); // sets the controlling routine on or off
void addNewTarget(uint8_t type, int16_t x, int16_t y, int16_t z, int16_t vel); // used to add a new target position
void clearTargetBuffer(void); // clears the buffer for the targets
uint8_t isTargetReached(); // returns 1, if the current target position is reached (within limits)
void setSpeedControllers(); // configures the speed controllers for the current target
void switchMotors(uint8_t on); // used to turn the motors on or off
void setMotorPower(uint8_t motor, int16_t power); // sets the power for the given motor
void handleControllers(); // let the controllers compute
void checkMotors(); // checks if the motors are moving as they should
void sendString(char* data); // send a string via uart
void sendDouble(double value); // send a double via uart (cut down to int)
void sendRegister(char *name, volatile uint8_t *port); // send a register value via uart
uint8_t getEdge(uint8_t axis, uint8_t new, uint8_t old); // returns the edge for the given channel
void updateEncoder(uint8_t axis); // updates the counter value for the given axis
void updateAxis(); // updates the axis value

// ==================================== [program start] ==========================================

int main()
{
	initialize();
	createControllers();
	sendString("mcalibrating...\r\n");
	calibrate();
	sendString("mcalibrating finished...\r\n");
	
	while(1)
	{
		updateTime();
		handleData();
		updateAxis();
		if(isControlling)
		{
			if(isTargetReached())
			{
				if(getTimeDiff(T_CAL_WAIT) > TARGET_WAIT)
				{
					deleteItem(buffers[TARGETS], target);
					if(hasNextItem(buffers[TARGETS], target))
					{
						setControlling(1);
					}
					else
					{
						setControlling(0);
						resetBuffer(buffers[TARGETS]);
						sendString("tfinal target reached\r\n");
						sendString("mfinal target reached\r\n");
					}
				}
			}
			else
			{
				resetTime(T_CAL_WAIT);
			}
			
			if(isControlling)
			{
				handleControllers();
				checkMotors();
			}
			
			if(DEBUGS && getTimeDiff(T_DEBUGS) > 0.8) // shouldn't be used, sending much data continous causes crashes
			{
				sendString("m");
				sendDouble(outputs[C_X_E_POS] * 10);
				sendString("\t");
				sendDouble(outputs[C_Y_E_POS] * 10);
				sendString("\r\n");
				resetTime(T_DEBUGS);
			}
		}
	}
}

void initialize()
{
	cli(); // disable global interrupts
  
// ==================================== [pin configuration] ==========================================
	
	setPin(motorZpwm, DDR, 1); // set the pwm-pins as outputs
	setPin(motorXpwm, DDR, 1);
	setPin(motorYpwm, DDR, 1);
	setPin(encoder[X][A], DDR, 0); // set the encoder-pins as inputs
	setPin(encoder[X][B], DDR, 0);
	setPin(encoder[Y][A], DDR, 0);
	setPin(encoder[Y][B], DDR, 0);
	setPin(motorXpin1, DDR, 1); // set the motor-control pins as outputs
	setPin(motorXpin2, DDR, 1);
	setPin(motorYpin1, DDR, 1);
	setPin(motorYpin2, DDR, 1);
	
	setBit(&DDRE, 0, 0); // RX0 as input
	setBit(&DDRE, 1, 1); // TX0 as output
	
	setPin(encoder[X][A], PORT, 1); // turn pull-up on for x-encoder channel A
	setPin(encoder[X][B], PORT, 1); // turn pull-up on for x-encoder channel B
	setPin(encoder[Y][A], PORT, 1); // turn pull-up on for y-encoder channel A
	setPin(encoder[Y][B], PORT, 1); // turn pull-up on for y-encoder channel B

// ==================================== [encoder configuration] ==========================================

	setBit(&EICRA, ISC00, 1); // any edge of pin INT0 generates an interrupt
	setBit(&EICRA, ISC10, 1); // any edge of pin INT1 generates an interrupt
	setBit(&EICRA, ISC20, 1); // any edge of pin INT2 generates an interrupt
	setBit(&EICRA, ISC30, 1); // any edge of pin INT3 generates an interrupt
	setBit(&EIMSK, INT0, 1); // enable interrupt for pin INT0
	setBit(&EIMSK, INT1, 1); // enable interrupt for pin INT1
	setBit(&EIMSK, INT2, 1); // enable interrupt for pin INT2
	setBit(&EIMSK, INT3, 1); // enable interrupt for pin INT3
	
	encoderState[X][A_NEW] = readPin(encoder[X][A], PIN); // read the current encoder status
	encoderState[X][B_NEW] = readPin(encoder[X][B], PIN);
	encoderState[Y][A_NEW] = readPin(encoder[Y][A], PIN);
	encoderState[Y][B_NEW] = readPin(encoder[Y][B], PIN);
	
	invertedEncoder[X] = AXIS_X_INVERTED;
	invertedEncoder[Y] = AXIS_Y_INVERTED;
  
// ==================================== [timer1 - motor PWM] ==========================================
	
	//setBit(&TCCR1A, COM1A1, 1); // port A operation as non inverted PWM -> does not work, interrupts instead
	//setBit(&TCCR1A, COM1B1, 1); // port B operation as non inverted PWM	
	setBit(&TCCR1A, WGM10, 1); // timer mode 5 - 8-bit fast PWM
	setBit(&TCCR1A, WGM11, 0);
	setBit(&TCCR1B, WGM12, 1);
	setBit(&TCCR1B, WGM13, 0);
	setBit(&TIMSK1, OCIE1A, 1); // enable compare match A interrupt (X-axis)
	setBit(&TIMSK1, OCIE1B, 1); // enable compare match B interrupt (Y-axis)
	setBit(&TIMSK1, TOIE1, 1); // enable overflow interrupt
	
	invertedMotor[X] = MOT_X_INVERTED;
	invertedMotor[Y] = MOT_Y_INVERTED;
	invertedMotor[Z] = MOT_Z_INVERTED;
	
// ==================================== [timer3 - time measuring] ==========================================
  
	setBit(&TCCR3B, WGM32, 1); // timer mode 4 - CTC
	setBit(&TCCR3B, CS30, 1); // set prescaler to 1
	setBit(&TCCR3B, CS31, 0);
	setBit(&TCCR3B, CS32, 0);
	OCR3A = 1580; // -> the interrupt is generated every 0.5ms
	setBit(&TIMSK3, OCIE3A, 1); // enable interrupt on compare match
	
// ==================================== [timer5 - servo control] ==========================================
	
	setBit(&TCCR5B, WGM52, 1); // timer mode 4 - CTC
	setBit(&TCCR5B, CS50, 1); // set prescaler to 64 -> setting OCR4B to 255 leads to duration of 2ms for the servo
	setBit(&TCCR5B, CS51, 1);
	setBit(&TCCR5B, CS52, 0);
	OCR5A = 2499; // -> the period is ~20ms
	setBit(&TIMSK5, OCIE5A, 1); // enable interrupt on compare match
	setBit(&TIMSK5, OCIE5B, 1); // enable interrupt on compare match
	setMotorPower(Z, OFF_MOT_Z); // set the pen up
	
// ==================================== [uart configuration] ==========================================	
	
	UBRR0 = 12; // baudrate 76800
	setBit(&UCSR0B, RXCIE0, 1); // enable RX complete interrupt
	setBit(&UCSR0B, TXCIE0, 1); // enable TX complete interrupt
	setBit(&UCSR0C, UMSEL01, 0); // asynchronous USART
	setBit(&UCSR0C, UMSEL00, 0); //
	setBit(&UCSR0C, UPM01, 0); // no Parity
	setBit(&UCSR0C, UPM00, 0); //
	setBit(&UCSR0C, USBS0, 0); // 1 stop-bit
	setBit(&UCSR0C, UCSZ01, 1); // 8-bit Frames
	setBit(&UCSR0C, UCSZ00, 1); //
	setBit(&PRR0, PRUSART0, 0); // switch USART0 on
	setBit(&UCSR0B, TXEN0, 1); // Enable TX
	setBit(&UCSR0B, RXEN0, 1); // Enable RX
	
// ==================================== [buffer setup] =========================================

	buffers[INDATA] = createBuffer(char, INOUT_BUFF_SIZE);
	buffers[OUTDATA] = createBuffer(char, INOUT_BUFF_SIZE);
	buffers[TARGETS] = createBuffer(target, TARGET_BUFF_SIZE);

	sei(); // enable global interrupts
	
}

void createControllers()
{
	if(CALLS) sendString("qcreateControllers()\r\n");
	
	controllers[C_X_VEL] = createDIF(SAMPLE_TIME);
	controllers[C_Y_VEL] = createDIF(SAMPLE_TIME);
	controllers[C_X_VEL_FIL] = createPT1(SAMPLE_TIME);
	controllers[C_Y_VEL_FIL] = createPT1(SAMPLE_TIME);
	controllers[C_X_E_POS] = createLink(SAMPLE_TIME);
	controllers[C_Y_E_POS] = createLink(SAMPLE_TIME);
	controllers[C_E_POS] = createNORM(SAMPLE_TIME);
	controllers[C_VEL_TAR] = createRAM2(SAMPLE_TIME);
	controllers[C_VEL_DIV] = createLink(SAMPLE_TIME);
	controllers[C_X_VEL_MUL] = createLink(SAMPLE_TIME);
	controllers[C_Y_VEL_MUL] = createLink(SAMPLE_TIME);
	controllers[C_X_E_VEL] = createLink(SAMPLE_TIME);
	controllers[C_Y_E_VEL] = createLink(SAMPLE_TIME);
	controllers[C_X_MOT_PID] = createPID(SAMPLE_TIME);
	controllers[C_Y_MOT_PID] = createPID(SAMPLE_TIME);
	controllers[C_X_MOT_MUL] = createLink(SAMPLE_TIME);
	controllers[C_Y_MOT_MUL] = createLink(SAMPLE_TIME);
	controllers[C_X_MOT_SUM] = createLink(SAMPLE_TIME);
	controllers[C_Y_MOT_SUM] = createLink(SAMPLE_TIME);
	controllers[C_X_MOT_LIM] = createLIM(SAMPLE_TIME);
	controllers[C_Y_MOT_LIM] = createLIM(SAMPLE_TIME);
	controllers[C_Z_MOT_PID] = createPID(SAMPLE_TIME);
	
	setInterfaces(    controllers[C_X_VEL],          &axisValue[X],                     0,     &outputs[C_X_VEL]);
	setInterfaces(    controllers[C_Y_VEL],          &axisValue[Y],                     0,     &outputs[C_Y_VEL]);
	setInterfaces(controllers[C_X_VEL_FIL],      &outputs[C_X_VEL],                     0, &outputs[C_X_VEL_FIL]);
	setInterfaces(controllers[C_Y_VEL_FIL],      &outputs[C_Y_VEL],                     0, &outputs[C_Y_VEL_FIL]);
	setInterfaces(  controllers[C_X_E_POS],  &targetCoordinates[X],         &axisValue[X],   &outputs[C_X_E_POS]);
	setInterfaces(  controllers[C_Y_E_POS],  &targetCoordinates[Y],         &axisValue[Y],   &outputs[C_Y_E_POS]);
	setInterfaces(    controllers[C_E_POS],    &outputs[C_X_E_POS],   &outputs[C_Y_E_POS],     &outputs[C_E_POS]);
	setInterfaces(  controllers[C_VEL_TAR],      &outputs[C_E_POS],                     0,   &outputs[C_VEL_TAR]);
	setInterfaces(  controllers[C_VEL_DIV],    &outputs[C_VEL_TAR],     &outputs[C_E_POS],   &outputs[C_VEL_DIV]);
	setInterfaces(controllers[C_X_VEL_MUL],    &outputs[C_X_E_POS],   &outputs[C_VEL_DIV], &outputs[C_X_VEL_MUL]);
	setInterfaces(controllers[C_Y_VEL_MUL],    &outputs[C_Y_E_POS],   &outputs[C_VEL_DIV], &outputs[C_Y_VEL_MUL]);
	setInterfaces(  controllers[C_X_E_VEL],  &outputs[C_X_VEL_MUL], &outputs[C_X_VEL_FIL],   &outputs[C_X_E_VEL]);
	setInterfaces(  controllers[C_Y_E_VEL],  &outputs[C_Y_VEL_MUL], &outputs[C_Y_VEL_FIL],   &outputs[C_Y_E_VEL]);
	setInterfaces(controllers[C_X_MOT_PID],    &outputs[C_X_E_VEL],                     0, &outputs[C_X_MOT_PID]);
	setInterfaces(controllers[C_Y_MOT_PID],    &outputs[C_Y_E_VEL],                     0, &outputs[C_Y_MOT_PID]);
	setInterfaces(controllers[C_X_MOT_MUL],  &outputs[C_X_VEL_MUL],        &motorMulti[X], &outputs[C_X_MOT_MUL]);
	setInterfaces(controllers[C_Y_MOT_MUL],  &outputs[C_Y_VEL_MUL],        &motorMulti[Y], &outputs[C_Y_MOT_MUL]);
	setInterfaces(controllers[C_X_MOT_SUM],  &outputs[C_X_MOT_PID], &outputs[C_X_MOT_MUL], &outputs[C_X_MOT_SUM]);
	setInterfaces(controllers[C_Y_MOT_SUM],  &outputs[C_Y_MOT_PID], &outputs[C_Y_MOT_MUL], &outputs[C_Y_MOT_SUM]);
	setInterfaces(controllers[C_X_MOT_LIM],  &outputs[C_X_MOT_SUM],                     0, &outputs[C_X_MOT_LIM]);
	setInterfaces(controllers[C_Y_MOT_LIM],  &outputs[C_Y_MOT_SUM],                     0, &outputs[C_Y_MOT_LIM]);
	setInterfaces(controllers[C_Z_MOT_PID],  &targetCoordinates[Z],                     0, &outputs[C_Z_MOT_PID]);
	
	pt1SetParams( controllers[C_X_VEL_FIL], AXIS_T1, 1);
	pt1SetParams( controllers[C_Y_VEL_FIL], AXIS_T1, 1);
	linkSetParams(  controllers[C_X_E_POS], SUBSTRACT);
	linkSetParams(  controllers[C_Y_E_POS], SUBSTRACT);
	linkSetParams(controllers[C_VEL_DIV], DIVIDE);
	linkSetParams(controllers[C_X_VEL_MUL], MULTIPLY);
	linkSetParams(controllers[C_Y_VEL_MUL], MULTIPLY);
	linkSetParams(controllers[C_X_E_VEL], SUBSTRACT);
	linkSetParams(controllers[C_Y_E_VEL], SUBSTRACT);
	linkSetParams(controllers[C_X_MOT_MUL], MULTIPLY);
	linkSetParams(controllers[C_Y_MOT_MUL], MULTIPLY);
	linkSetParams(controllers[C_X_MOT_SUM], ADD);
	linkSetParams(controllers[C_Y_MOT_SUM], ADD);
	
	acceleration = ACCELERATION;
	
	motorMulti[X] = AXIS_X_FAC;
	pidSetParams( controllers[C_X_MOT_PID], AXIS_X_KP, AXIS_X_KI, AXIS_X_KD);
	pidSetLimits( controllers[C_X_MOT_PID], -MAX_MOT_X, MAX_MOT_X, MAX_ESUM);
	limSetParams( controllers[C_X_MOT_LIM], -MAX_MOT_X, MAX_MOT_X);
	
	motorMulti[Y] = AXIS_Y_FAC;
	pidSetParams( controllers[C_Y_MOT_PID], AXIS_Y_KP, AXIS_Y_KI, AXIS_Y_KD);
	pidSetLimits( controllers[C_Y_MOT_PID], -MAX_MOT_Y, MAX_MOT_Y, MAX_ESUM);
	limSetParams( controllers[C_Y_MOT_LIM], -MAX_MOT_Y, MAX_MOT_Y);
	
	pidSetParams( controllers[C_Z_MOT_PID], VAL_PER_MM, 0, 0);
	pidSetLimits( controllers[C_Z_MOT_PID], MIN_MOT_Z, MAX_MOT_Z, 0);
	pidSetOffset( controllers[C_Z_MOT_PID], OFF_MOT_Z);		
}

void calibrate()
{
	if(CALLS) sendString("qcalibrate()\r\n");
	
	switchMotors(1);
	setMotorPower(X, CAL_VEL_X); // drive towards origin
	setMotorPower(Y, CAL_VEL_Y);
	
	double lastX = 0; // stores the last position
	double lastY = 0;
		
	resetTime(T_CAL_WAIT); // give the motors some time to start moving
	while(getTimeDiff(T_CAL_WAIT) < CAL_TIME)
		updateTime();
	resetTime(T_CAL_WAIT);
		
	while(1)
	{
		updateTime();
		updateAxis();
		if(getTimeDiff(T_CAL_WAIT) >= CAL_TIME) // time to wait till the position gets checked
		{
			if(abs(axisValue[X] - lastX) <= CAL_TOL && abs(axisValue[Y] - lastY) <= CAL_TOL)
				break; // the motors aren't moving, they should be at the origin (or didn't move at all -> power supply?)
			else
			{
				lastX = axisValue[X]; // store the last position
				lastY = axisValue[Y];
				resetTime(T_CAL_WAIT);
			}
		}
	}
	
	encoderValue[X] = CAL_TAR_X * STEPS_PER_MM; // set the encoder-increments to the defined value
	encoderValue[Y] = CAL_TAR_Y * STEPS_PER_MM;
	updateAxis(); // calculate the position in mm new
	
	switchMotors(0);
}

void updateTime(void)
{
	if(CALLS) sendString("qupdateTime()\r\n");
	
	time[T_RUNTIME] = (double) timeCounter * TIME_STEP;	
}

double getTime(uint8_t timer)
{
	if(CALLS) sendString("qgetTime()\r\n");
	
	return time[timer];
}

double getTimeDiff(uint8_t timer)
{
	if(CALLS) sendString("qgetTimeDiff()\r\n");
	
	return time[T_RUNTIME] - time[timer];	
}

void resetTime(uint8_t timer)
{
	if(CALLS) sendString("qresetTime()\r\n");
	
	time[timer] = getTime(T_RUNTIME);	
}

void handleData()
{
	if(CALLS) sendString("qhandleData()\r\n");	
	
	if(hasNextItem(buffers[INDATA], char))
	{
		char command = getItem(buffers[INDATA], char);
		
		if(command == 'b')
		{
			if(hasNextItems(buffers[INDATA], char, 3))
			{
				if(getNextItem(buffers[INDATA], char, 2) == 'e')
				{
					int8_t value = getNextItem(buffers[INDATA], int8_t, 1);
					if(value > 1)
					{
						acceleration = (double) value;
						sendString("mAcceleration: ");
						sendDouble(acceleration);
						sendString("\r\n");
					}
					else
						sendString("fvalue has to be higher than 1.\r\n");
				}
				else
				{
					sendString("finvalid command!\r\n");	
				}
				deleteItems(buffers[INDATA], char, 3);
			}
		}
		else if(command == 'c')
		{
			if(hasNextItems(buffers[INDATA], char, 2))
			{
				if(getNextItem(buffers[INDATA], char, 1) == 'e')
				{
					clearTargetBuffer();
				}
				else
				{
					sendString("finvalid command!\r\n");
				}
				deleteItems(buffers[INDATA], char, 2);
			}
		}
		else if(command == 'd')
		{
			if(hasNextItems(buffers[INDATA], char, 4))
			{
				if(getNextItem(buffers[INDATA], char, 3) == 'e')
				{
					deleteItem(buffers[INDATA], char);
					int16_t delay = getItem(buffers[INDATA], int16_t);
					deleteItems(buffers[INDATA], char, 3);
					addNewTarget(DELAY, delay, 0, 0, 0);
				}
				else
				{
					deleteItems(buffers[INDATA], char, 4);
					sendString("finvalid command!\r\n");
				}
			}
		}
		else if(command == 'g')
		{
			if(hasNextItems(buffers[INDATA], char, 10))
			{
				if(getNextItem(buffers[INDATA], char, 9) == 'e')
				{
					deleteItem(buffers[INDATA], char);
					int16_t values[4];
					for(int i = 0; i < 4; i++)
						values[i] = getNextItem(buffers[INDATA], int16_t, i);
					deleteItems(buffers[INDATA], char, 9);
					addNewTarget(COORDINATE, values[0], values[1], values[2], values[3]); 
				}
				else
				{
					deleteItems(buffers[INDATA], char, 10);
					sendString("finvalid command!\r\n");
				}
			}
		}
		else if(command == 'h')
		{
			if(hasNextItems(buffers[INDATA], char, 2))
			{
				if(getNextItem(buffers[INDATA], char, 1) == 'e')
				{
					sendString("mmoving paused...\r\n");
					setControlling(0);
				}
				else
				{
					sendString("finvalid command!\r\n");
				}
				deleteItems(buffers[INDATA], char, 2);
			}
		}
		else if(command == 'j')
		{
			if(hasNextItems(buffers[INDATA], char, 2))
			{
				if(getNextItem(buffers[INDATA], char, 1) == 'e')
				{
					if(controlCounter > 0)
					{
						elog[X] = elog[X] / controlCounter;
						elog[Y] = elog[Y] / controlCounter;
						sendString("mx: ");
						sendDouble(elog[X]);
						sendString(" y: ");
						sendDouble(elog[Y]);
						sendString("\r\n");
						elog[X] = 0;
						elog[Y] = 0;
						controlCounter = 0;
					}
					else
					{
						sendString("wthere were no calculations.\r\n");	
					}
				}
				else
				{
					sendString("finvalid command!\r\n");
				}
				deleteItems(buffers[INDATA], char, 2);
			}
		}
		else if(command == 'k')
		{
			if(hasNextItems(buffers[INDATA], char, 2))
			{
				if(getNextItem(buffers[INDATA], char, 1) == 'e')
				{
					for(int i = 0; i < 2; i++)
					{
						sendString("m");
						sendDouble(axisValue[i]);
						sendString("\r\nm");
						sendDouble(targetCoordinates[i]);
						sendString("\r\n");
						sendString("m--------------------\r\n");
					}
					
					sendString("mcontrollers ---------------\r\n");
					for(int i = 0; i < NUM_CONTROLLERS; i++)
					{
						sendString("m");
						sendDouble(outputs[i]);
						sendString("\r\n");
						if(i % 2 != 0)
							sendString("m-------------------\r\n");
					}
				}
				else
				{
					sendString("finvalid command!\r\n");
				}
				deleteItems(buffers[INDATA], char, 2);
			}
		}
		else if(command == 'l')
		{
			if(hasNextItems(buffers[INDATA], char, 2))
			{
				if(getNextItem(buffers[INDATA], char, 1) == 'e')
				{
					setMotorPower(Z, OFF_MOT_Z);
				}
				else
				{
					sendString("finvalid command!\r\n");
				}
				deleteItems(buffers[INDATA], char, 2);
			}
		}
		else if(command == 'o')
		{
			if(hasNextItems(buffers[INDATA], char, 2))
			{
				if(getNextItem(buffers[INDATA], char, 1) == 'e')
				{
					clearTargetBuffer();
					sendString("mdriving towards origin...\r\n");
					addNewTarget(COORDINATE, (int16_t) axisValue[X] * 10, (int16_t) axisValue[Y] * 10, 0, 400);
					addNewTarget(DELAY, 1000, 0, 0, 0);
					addNewTarget(COORDINATE, 0, 0, 0, 400);
					setControlling(1);
				}
				else
				{
					sendString("finvalid command!\r\n");
				}
				deleteItems(buffers[INDATA], char, 2);
			}
		}
		else if(command == 'p')
		{
			if(hasNextItems(buffers[INDATA], char, 7))
			{
				if(getNextItem(buffers[INDATA], char, 6) == 'e')
				{
					deleteItem(buffers[INDATA], char);
					
					char motor[2];
					motor[0] = getItem(buffers[INDATA], char);
					motor[1] = 0;
					
					if(motor[0] == 'x' || motor[0] == 'y')
					{
						double kp = (double) getNextItem(buffers[INDATA], uint8_t, 1) / 100; 
						double ki = (double) getNextItem(buffers[INDATA], uint8_t, 2) / 100;
						double kd = (double) getNextItem(buffers[INDATA], uint8_t, 3) / 100;
						double kmul = (double) getNextItem(buffers[INDATA], uint8_t, 4) / 100;
					
						if(motor[0] == 'x')
						{
							pidSetParams(controllers[C_X_MOT_PID], kp, ki, kd);
							motorMulti[X] = kmul;
						}
						else if(motor[0] == 'y')
						{
							pidSetParams(controllers[C_Y_MOT_PID], kp, ki, kd);
							motorMulti[Y] = kmul;
						}
					
						sendString("mMotor: ");
						sendString(motor);
						sendString("\tkP: ");
						sendDouble(kp * 100);
						sendString("\tkI: ");
						sendDouble(ki * 100);
						sendString("\tkD: ");
						sendDouble(kd * 100);
						sendString("\tMulti: ");
						sendDouble(kmul * 100);
						sendString("\r\n");
					}
					else
					{
						sendString("funknown motor!\r\n");	
					}
					deleteItems(buffers[INDATA], char, 6);
				}
				else
				{
					deleteItems(buffers[INDATA], char, 7);
					sendString("finvalid command!\r\n");
				}
			}
		}
		else if(command == 'r')
		{
			if(hasNextItems(buffers[INDATA], char, 2))
			{
				if(getNextItem(buffers[INDATA], char, 1) == 'e')
				{
					clearTargetBuffer();
					sendString("mrecalibrating...\r\n");
					setMotorPower(Z, OFF_MOT_Z);
					calibrate();
					sendString("mrecalibrating finished...\r\n");
				}
				else
				{
					sendString("finvalid command!\r\n");
				}
				deleteItems(buffers[INDATA], char, 2);
			}
		}
		else if(command == 's')
		{
			if(hasNextItems(buffers[INDATA], char, 2))
			{
				if(getNextItem(buffers[INDATA], char, 1) == 'e')
				{
					if(isControlling)
					{
						sendString("wcontrollers are already running...\r\n");
					}
					else if(!hasNextItem(buffers[TARGETS], target))
					{
						sendString("fno targets available!\r\n");	
					}
					else
					{
						sendString("mstarting to move...\r\n");
						setControlling(1);
					}
				}
				else
				{
					sendString("finvalid command!\r\n");
				}
				deleteItems(buffers[INDATA], char, 2);
					
			}
		}
		else
		{
			sendString("funknown command!\r\n");
			deleteItem(buffers[INDATA], char);	
		}
	}
}

void setControlling(uint8_t active)
{
	if(CALLS) sendString("qsetControlling()\r\n");
	
	if(active)
		setSpeedControllers();
	isControlling = active;
	for(int i = 0; i < NUM_CONTROLLERS; i++)
	{
		if(active)
			reset(controllers[i], getTime(T_RUNTIME));
		setEnabled(controllers[i], active);
	}
	switchMotors(active);
}

void addNewTarget(uint8_t type, int16_t x, int16_t y, int16_t z, int16_t vel)
{
	if(CALLS) sendString("qaddNewTarget()\r\n");
	
	target cache;
	
	if(type == COORDINATE && x < MIN_X_AXIS)
	{
		if(ERRORS)
			sendString("wx is lower than minimum. target won't be added.\r\n");
	}
	else if(type == COORDINATE && x > MAX_X_AXIS)
	{
		if(ERRORS)
			sendString("wx is higher than maximum. target won't be added.\r\n");
	}
	else if(type == COORDINATE && y < MIN_Y_AXIS)
	{
		if(ERRORS)
			sendString("wy is lower than minimum. target won't be added.\r\n");
	}
	else if(type == COORDINATE && y > MAX_Y_AXIS)
	{
		if(ERRORS)
			sendString("wy is higher than maximum. target won't be added.\r\n");
	}
	else if(type == COORDINATE && z < MIN_Z_AXIS)
	{
		if(ERRORS)
			sendString("wz is lower than minimum. target won't be added.\r\n");
	}
	else if(type == COORDINATE && z > MAX_Z_AXIS)
	{
		if(ERRORS)
			sendString("wz is higher than maximum. target won't be added.\r\n");
	}
	else if(type == COORDINATE && vel < MIN_VELOCITY)
	{
		if(ERRORS)
			sendString("wvelocity is lower than minimum. target won't be added.\r\n");
	}
	else if(type == COORDINATE && vel > MAX_VELOCITY)
	{
		if(ERRORS)
			sendString("wvelocity is higher than maximum. target won't be added.\r\n");
	}
	else if(type == DELAY && x <= 0)
	{
		if(ERRORS)
			sendString("wthe delay must be positive. target won't be added.\r\n");
	}
	else
	{
		cache.type = type;
		cache.x = x;
		cache.y = y;
		cache.z = z;
		cache.vel = vel;
		if(isControlling && WARNINGS)
			sendString("wtargets shouldn't be added while running...\r\n");
		if(hasSpaceForItem(buffers[TARGETS], target))
		{
			putItem(buffers[TARGETS], target, cache);
		}
		else if(WARNINGS)
		{
			sendString("ftarget buffer full!\r\n");
		}
	}
}

void clearTargetBuffer(void)
{
	if(CALLS) sendString("qclearTargetBuffer()\r\n");
	
	sendString("mbuffer gets cleared...\r\n");
	setControlling(0);
	resetBuffer(buffers[TARGETS]);
}

uint8_t isTargetReached()
{
	if(CALLS) sendString("qisTargetReached()\r\n");
	
	target cache = getItem(buffers[TARGETS], target);
	if(cache.type == COORDINATE)
	{
		if(sqrt((targetCoordinates[X]-axisValue[X])*(targetCoordinates[X]-axisValue[X])+(targetCoordinates[Y]-axisValue[Y])*(targetCoordinates[Y]-axisValue[Y])) <= PRECISION)
			return 1;
		return 0;
	}
	else if(cache.type == DELAY)
	{
		if(getTimeDiff(T_TARGET_DELAY) >= targetCoordinates[X])
			return 1;
		return 0;
	}
	return 1;
}

void setSpeedControllers()
{
	if(CALLS) sendString("qsetSpeedControllers()\r\n");
	
	target cache = getItem(buffers[TARGETS], target);
	if(cache.type == COORDINATE)
	{
		double v;
		
		targetCoordinates[X] = (double) cache.x / 10;
		targetCoordinates[Y] = (double) cache.y / 10;
		targetCoordinates[Z] = (double) cache.z / 10;
		v = (double) cache.vel / 10;
		
		ramSetParams(controllers[C_VEL_TAR], acceleration, v);
		
		for(int i = 0; i < NUM_CONTROLLERS; i++)
			reset(controllers[i], getTime(T_RUNTIME));
	}
	else if(cache.type == DELAY)
	{
		targetCoordinates[X] = (double) cache.x;
		targetCoordinates[X] /= 1000.0;
		ramSetParams(controllers[C_VEL_TAR], acceleration, 0);
		resetTime(T_TARGET_DELAY);
	}
}

void switchMotors(uint8_t on)
{
	if(CALLS) sendString("qswitchMotors()\r\n");
	
	if(on && !motorState)
	{
		motorState = 1;
		setPin(motorXpin1, PORT, 0); // disable all controllers
		setPin(motorXpin2, PORT, 0);
		setPin(motorYpin1, PORT, 0);
		setPin(motorYpin2, PORT, 0);
		setPin(timer1CS10, PORT, 1); // set prescaler to 64, PWM frequency ~1kHZ
		setPin(timer1CS11, PORT, 1);
	}
	else if(!on && motorState)
	{
		motorState = 0;
		setPin(timer1CS10, PORT, 0); // set prescaler to zero, switches timer off
		setPin(timer1CS11, PORT, 0);
		
		setPin(motorXpin1, PORT, 1); // short-circuit all motors
		setPin(motorXpin2, PORT, 1);
		setPin(motorYpin1, PORT, 1);
		setPin(motorYpin2, PORT, 1);
	}
}

void setMotorPower(uint8_t motor, int16_t power)
{
	if(CALLS) sendString("qsetMotorPower()\r\n");
	
	if(invertedMotor[motor])
		power *= -1;
	int8_t direction = 0;
	if(power < 0)
	{
		direction = 1;
		power *= -1;
	}
	if(power > 0 && power < 15) // otherwise pwm won't work properly
		power = 15;	
	
	motorBuffer[motor] = power;
	
	if(motor == X)
	{
		setPin(motorXpin1, PORT, direction);
		setPin(motorXpin2, PORT, !direction);
	}
	else if(motor == Y)
	{
		setPin(motorYpin1, PORT, direction);
		setPin(motorYpin2, PORT, !direction);
	}
}

void handleControllers()
{
	if(CALLS) sendString("qhandleControllers()\r\n");
	
	for(int i = 0; i < NUM_CONTROLLERS; i++)
		controllers[i]->base.compute(controllers[i], getTime(T_RUNTIME));
	
	setMotorPower(X, (int16_t) outputs[C_X_MOT_LIM]);
	setMotorPower(Y, (int16_t) outputs[C_Y_MOT_LIM]);
	setMotorPower(Z, (int16_t) outputs[C_Z_MOT_PID]);
	
	elog[X] = elog[X] + outputs[C_X_E_VEL];
	elog[Y] = elog[Y] + outputs[C_Y_E_VEL];
	controlCounter++;
}

void checkMotors()
{
	if(CALLS) sendString("qcheckMotors()\r\n");
	
	if((abs(outputs[C_X_MOT_LIM]) > MIN_POWER && abs(outputs[C_X_VEL_FIL]) < MIN_SPEED) || (abs(outputs[C_Y_MOT_LIM]) > MIN_POWER && abs(outputs[C_Y_VEL_FIL]) < MIN_SPEED))
	{
		if(getTimeDiff(T_MOTOR_CHECK) > MAX_TIME)
		{
			if(ERRORS)
				sendString("fmotor error!\r\n");
			clearTargetBuffer();
		}
	}
	else
	{
		resetTime(T_MOTOR_CHECK);	
	}
}

void sendString(char *data)
{
	while(*data)
	{
		putItem(buffers[OUTDATA], char, *data);
		data++;
	}
	if(!isSending)
	{
		isSending = 1;
		UDR0 = getItem(buffers[OUTDATA], char);
		deleteItem(buffers[OUTDATA], char);
	}
}

void sendDouble(double value)
{
	if(CALLS) sendString("qsendDouble()\r\n");
	
	sprintf(stringBuffer, "%ld", (int32_t) value);
	sendString(stringBuffer);
}

void sendRegister(char *name, volatile uint8_t *port)
{
	if(CALLS) sendString("qsendRegister()\r\n");
	
	char output[40];
	output[0] = 'm';
	uint8_t cnt = 1;
	while(*name)
	{
		output[cnt] = *name;
		name++;
		cnt++;	
	}
	for(int i = 7; i > -1; i--) // counting down
	{
		if(readBit(port, i))
			output[cnt] = '1';
		else
			output[cnt] = '0';
		cnt++;	
	}
	output[cnt++] = '\r';
	output[cnt++] = '\n';
	output[cnt] = 0;
	sendString(output);
}

uint8_t getEdge(uint8_t axis, uint8_t new, uint8_t old)
{
	if(CALLS) sendString("qgetEdge()\r\n");
	
	if(encoderState[axis][new] == encoderState[axis][old])
		return 0;
	if(encoderState[axis][new] == 1)
		return RISING;
	return FALLING;
}

void updateEncoder(uint8_t axis)
{
	int8_t inc = 1;
	if(invertedEncoder[axis])
		inc = -1;
	
	encoderState[axis][A_OLD] = encoderState[axis][A_NEW]; // move states from section new to section old
	encoderState[axis][B_OLD] = encoderState[axis][B_NEW];
	encoderState[axis][A_NEW] = readPin(encoder[axis][A], PIN); // copy the current state to section new
	encoderState[axis][B_NEW] = readPin(encoder[axis][B], PIN);
	
	if(getEdge(axis, A_NEW, A_OLD) == RISING && 0 == encoderState[axis][B_NEW])
		encoderValue[axis] += inc;
	else if(getEdge(axis, A_NEW, A_OLD) == FALLING && 1 == encoderState[axis][B_NEW])
		encoderValue[axis] += inc;
	else if(getEdge(axis, B_NEW, B_OLD) == RISING && 1 == encoderState[axis][A_NEW])
		encoderValue[axis] += inc;
	else if(getEdge(axis, B_NEW, B_OLD) == FALLING && 0 == encoderState[axis][A_NEW])
		encoderValue[axis] += inc;
	else
		encoderValue[axis] -= inc;
}

void updateAxis()
{
	if(CALLS) sendString("qupdateAxis()\r\n");
	
	axisValue[X] = encoderValue[X] / STEPS_PER_MM;
	axisValue[Y] = encoderValue[Y] / STEPS_PER_MM;
}

ISR(INT0_vect)
{
	updateEncoder(X);
}

ISR(INT1_vect)
{
	updateEncoder(X);
}

ISR(INT2_vect)
{  
	updateEncoder(Y);
}

ISR(INT3_vect)
{
	updateEncoder(Y);
}

ISR(USART0_RX_vect) // RX complete
{
	uint8_t test = UDR0;
	if(ECHO)
	{
		putItem(buffers[OUTDATA], char, 'i');
		putItem(buffers[OUTDATA], uint8_t, test);
		sendString("\r\n");
	}
	putItem(buffers[INDATA], uint8_t, test);
}

ISR(USART0_TX_vect)
{
	if(hasNextItem(buffers[OUTDATA], char))
	{
		UDR0 = getItem(buffers[OUTDATA], char);
		deleteItem(buffers[OUTDATA], char);
	}
	else
		isSending = 0;
}

ISR(TIMER1_OVF_vect) // set the motors on
{
	OCR1A = motorBuffer[X];
	OCR1B = motorBuffer[Y];
	
	if(OCR1A > 0)
		setPin(motorXpwm, PORT, 1);
	if(OCR1B > 0)
		setPin(motorYpwm, PORT, 1);
}

ISR(TIMER1_COMPA_vect) // set motor X off
{
	setPin(motorXpwm, PORT, 0);
}

ISR(TIMER1_COMPB_vect) // set motor Y off
{
	setPin(motorYpwm, PORT, 0);
}

ISR(TIMER3_COMPA_vect) // counting up the time
{
	timeCounter++;
}

ISR(TIMER5_COMPA_vect)
{
	OCR5B = motorBuffer[Z];
	if(OCR5B > 0)
		setPin(motorZpwm, PORT, 1); // set the servo pin high at overflow (CTC)
}

ISR(TIMER5_COMPB_vect)
{
	setPin(motorZpwm, PORT, 0); // set the servo pin low for given value in COMPB
}