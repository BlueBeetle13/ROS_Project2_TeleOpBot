#include "adafruitServoController.h"
#include <stdio.h>
#include <math.h>

// PCA9685 Registers
#define REG_MODE1			0x00
#define REG_MODE2			0x01
#define REG_SUBADR1			0x02
#define REG_SUBADR2			0x03
#define REG_SUBADR3			0x04
#define REG_PRESCALE		0xFE
#define REG_LED0_ON_L		0x06
#define REG_LED0_ON_H		0x07
#define REG_LED0_OFF_L		0x08
#define REG_LED0_OFF_H		0x09
#define REG_ALL_LED_ON_L	0xFA
#define REG_ALL_LED_ON_H	0xFB
#define REG_ALL_LED_OFF_L	0xFC
#define REG_ALL_LED_OFF_H	0xFD

// PCA9685 Bits
#define BIT_RESTART			0x80
#define BIT_SLEEP			0x10
#define BIT_ALLCALL			0x01
#define BIT_INVRT			0x10
#define BIT_OUTDRV			0x04

// Debug verbose output
#define DEBUG_MODE			1

// Constructor
AdafruitServoController::AdafruitServoController()
{
	// Initialization
	m_WiringPi_I2C = -1;


	// Set initial values
	for (int servoNum = 0; servoNum < TOTAL_SERVOS; servoNum ++)
	{
		// Servo Min value
		m_Servo_Min[servoNum] = 0;

		// Servo Max value
		m_Servo_Max[servoNum] = 4095;

		// Servo Position
		m_Servo_Position[servoNum] = 4096;
	}
}

// Destructor
AdafruitServoController::~AdafruitServoController()
{
	// Stop everything
	SetAllPWM(0, 0);
}

// Min - Max values - these are servo dependent and the position is scaled from 0->180 using these values
void AdafruitServoController::Servo_SetMinMax(unsigned char servoNum, unsigned short min, unsigned short max)
{
	// Ensure valid servo number
	if (servoNum >= 0 && servoNum < TOTAL_SERVOS)
	{
		if (DEBUG_MODE)
			printf("AdafruitServoController - Servo: %d   Min:%d   Max:%d\n", servoNum, min, max);

		// Set min and max, constraining to limits

		m_Servo_Min[servoNum] = min;
		if (m_Servo_Min[servoNum] < 0)
			m_Servo_Min[servoNum] = 0;
		if (m_Servo_Min[servoNum] > 4095)
			m_Servo_Min[servoNum] = 4095;

		m_Servo_Max[servoNum] = max;
		if (m_Servo_Max[servoNum] < 0)
			m_Servo_Max[servoNum] = 0;
		if (m_Servo_Max[servoNum] > 4095)
			m_Servo_Max[servoNum] = 4095;
	}
}


// Motor direction and speed
void AdafruitServoController::Servo_SetPosition(unsigned char servoNum, unsigned short position)
{
	// Ensure valid servo number
	if (servoNum >= 0 && servoNum < TOTAL_SERVOS)
	{
		// Map the position (0-180) using the servo min/max values
		double diff = ((double)m_Servo_Max[servoNum]) - ((double)m_Servo_Min[servoNum]);
		double pulse = ((((double)position) / 180.0) * diff) + ((double)m_Servo_Min[servoNum]);
		unsigned short pulseShort = (unsigned short)pulse;

		if (DEBUG_MODE)
			printf("AdafruitServoController - #: %d   Pos: %d   Pulse: %d\n", servoNum, position, pulseShort);


		// Update the position if it has changed
		if (m_Servo_Position[servoNum] != pulseShort)
		{
			if (DEBUG_MODE)
				printf("AdafruitServoController - Update Servo: %d Pulse: %d\n", servoNum, pulseShort);

			m_Servo_Position[servoNum] = pulseShort;
			SetPWM(servoNum, 0, pulseShort);
		}
	}
}

// Connect to the device on I2C and ensure it exists
int AdafruitServoController::ConnectToI2C(int address)
{
	// Setup the WiringPi I2C library and connect to the motor controller
	m_WiringPi_I2C = wiringPiI2CSetup(address);
	if (m_WiringPi_I2C == -1)
	{
		if (DEBUG_MODE)
			printf("AdafruitServoController - wiringPi setup failed\n");

		return -1;
	}

	if (DEBUG_MODE)
		printf("AdafruitServoController - wiringPi connected to Adafruit Servo Controller\n");

	Init();

	SetFrequency(60.0 * 0.9);

	return 1;
}

void AdafruitServoController::Init()
{
	// Reseting PCA9685 MODE1 (without SLEEP) and MODE
	SetAllPWM(0, 0);
	wiringPiI2CWriteReg8(m_WiringPi_I2C, REG_MODE2, BIT_OUTDRV);
	wiringPiI2CWriteReg8(m_WiringPi_I2C, REG_MODE1, BIT_ALLCALL);
	delay(5);
	char mode1 = wiringPiI2CReadReg8(m_WiringPi_I2C, REG_MODE1);
	mode1 = mode1 & BIT_SLEEP;
	wiringPiI2CWriteReg8(m_WiringPi_I2C, REG_MODE1, mode1);
	delay(5);

	if (DEBUG_MODE)
		printf("AdafruitServoController - PCA9685 init completed\n");
}

void AdafruitServoController::SetFrequency(double frequency)
{
	// Set the PWM frequency
	double prescale = floor((25000000.0 / 4096.0 / frequency) - 1.0 + 0.5);
	char prescaleChar = (char)prescale;

	unsigned char oldMode = wiringPiI2CReadReg8(m_WiringPi_I2C, REG_MODE1);
	char newMode = (oldMode & 0x7F) | 0x10;
	wiringPiI2CWriteReg8(m_WiringPi_I2C, REG_MODE1, newMode);
	wiringPiI2CWriteReg8(m_WiringPi_I2C, REG_PRESCALE, prescaleChar);
	wiringPiI2CWriteReg8(m_WiringPi_I2C, REG_MODE1, oldMode);
	delay(5);
	wiringPiI2CWriteReg8(m_WiringPi_I2C, REG_MODE1, oldMode | 0x80);

	if (DEBUG_MODE)
		printf("AdafruitServoController - PCA9685 PWM frequency set\n");
}

// Set specific pwm
void AdafruitServoController::SetPWM(unsigned char channel, unsigned short on, unsigned short off)
{
	wiringPiI2CWriteReg8(m_WiringPi_I2C, REG_LED0_ON_L + (4 * channel), on & 0xFF);
	wiringPiI2CWriteReg8(m_WiringPi_I2C, REG_LED0_ON_H + (4 * channel), on >> 8);
	wiringPiI2CWriteReg8(m_WiringPi_I2C, REG_LED0_OFF_L + (4 * channel), off & 0xFF);
	wiringPiI2CWriteReg8(m_WiringPi_I2C, REG_LED0_OFF_H + (4 * channel), off >> 8);
}

// Set all servo pwm at the same time
void AdafruitServoController::SetAllPWM(unsigned short on, unsigned short off)
{
	wiringPiI2CWriteReg8(m_WiringPi_I2C, REG_ALL_LED_ON_L, on & 0xFF);
	wiringPiI2CWriteReg8(m_WiringPi_I2C, REG_ALL_LED_ON_H, on >> 8);
	wiringPiI2CWriteReg8(m_WiringPi_I2C, REG_ALL_LED_OFF_L, off & 0xFF);
	wiringPiI2CWriteReg8(m_WiringPi_I2C, REG_ALL_LED_OFF_H, off >> 8);
}
