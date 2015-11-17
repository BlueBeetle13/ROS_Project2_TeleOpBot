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

	// Min, max values
	m_Servo_Min = 150;
	m_Servo_Max = 600;

	m_Servo1_Position = 4096;
	m_Servo2_Position = 4096;
}

// Destructor
AdafruitServoController::~AdafruitServoController()
{
	// Stop everything
	SetAllPWM(0, 0);
}

// Min - Max values - these are servo dependent and the position is scaled from 0->180 using these values
void AdafruitServoController::Set_Min_Max(unsigned short min, unsigned short max)
{
	m_Servo_Min = min;
	if (m_Servo_Min < 0)
		m_Servo_Min = 0;
	if (m_Servo_Min > 4096)
		m_Servo_Min = 4096;

	m_Servo_Max = max;
	if (m_Servo_Max < 0)
		m_Servo_Max = 0;
	if (m_Servo_Max > 4096)
		m_Servo_Max = 4096;
}


// Motor direction and speed
void AdafruitServoController::Servo_SetPosition(unsigned char servoNum, unsigned short position)
{
	double diff = ((double)m_Servo_Max) - ((double)m_Servo_Min);
	double pulse = ((((double)position) / 180.0) * diff) + ((double)m_Servo_Min);
	unsigned short pulseShort = (unsigned short)pulse;

	if (DEBUG_MODE)
		printf("AdafruitServoController - Pos: %d   Pulse: %d\n", position, pulseShort);

	// Update or not
	bool update = false;
	if (servoNum == SERVO_1 && m_Servo1_Position != pulseShort)
	{
		m_Servo1_Position = pulseShort;
		update = true;
	}
	else if (servoNum == SERVO_2 && m_Servo2_Position != pulseShort)
	{
		m_Servo2_Position = pulseShort;
		update = true;
	}

	if (update)
	{
		if (DEBUG_MODE)
			printf("AdafruitServoController - Update PWM: %d Pulse: %d\n", servoNum, pulseShort);

		SetPWM(servoNum, 0, pulseShort);
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

void AdafruitServoController::SetPWM(unsigned char channel, unsigned short on, unsigned short off)
{
	wiringPiI2CWriteReg8(m_WiringPi_I2C, REG_LED0_ON_L + (4 * channel), on & 0xFF);
	wiringPiI2CWriteReg8(m_WiringPi_I2C, REG_LED0_ON_H + (4 * channel), on >> 8);
	wiringPiI2CWriteReg8(m_WiringPi_I2C, REG_LED0_OFF_L + (4 * channel), off & 0xFF);
	wiringPiI2CWriteReg8(m_WiringPi_I2C, REG_LED0_OFF_H + (4 * channel), off >> 8);
}

void AdafruitServoController::SetAllPWM(unsigned short on, unsigned short off)
{
	wiringPiI2CWriteReg8(m_WiringPi_I2C, REG_ALL_LED_ON_L, on & 0xFF);
	wiringPiI2CWriteReg8(m_WiringPi_I2C, REG_ALL_LED_ON_H, on >> 8);
	wiringPiI2CWriteReg8(m_WiringPi_I2C, REG_ALL_LED_OFF_L, off & 0xFF);
	wiringPiI2CWriteReg8(m_WiringPi_I2C, REG_ALL_LED_OFF_H, off >> 8);
}
