#include "adafruitMotorController.h"
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

// Motor Identifiers
#define MOTOR_1				0
#define MOTOR_2				1
#define MOTOR_3				2
#define MOTOR_4				3

// Debug verbose output
#define DEBUG_MODE			1


// Constructor
AdafruitMotorController::AdafruitMotorController()
{
	// Initialization
	m_WiringPi_I2C = -1;

	m_SpeedScale = 1.0;

	m_Motor1_Direction = MOTOR_DIRECTION_STOP;
	m_Motor1_Speed = 0;
	m_Motor2_Direction = MOTOR_DIRECTION_STOP;
	m_Motor2_Speed = 0;
	m_Motor3_Direction = MOTOR_DIRECTION_STOP;
	m_Motor3_Speed = 0;
	m_Motor4_Direction = MOTOR_DIRECTION_STOP;
	m_Motor4_Speed = 0;
}

// Destructor
AdafruitMotorController::~AdafruitMotorController()
{
	// Stop everything
	SetPWM(8, 0, 0);
	SetPWM(13, 0, 0);
	SetPWM(2, 0, 0);
	SetPWM(7, 0, 0);

	SetAllPWM(0, 0);
}

void AdafruitMotorController::SetSpeedScale(double speedScale)
{
	m_SpeedScale = speedScale;
}

// Motor direction and speed
void AdafruitMotorController::Motor1_Set(unsigned short direction, double speed)
{
	Motor_Set(MOTOR_1, 10, 9, 8, direction, speed);
}

void AdafruitMotorController::Motor2_Set(unsigned short direction, double speed)
{
	Motor_Set(MOTOR_2, 11, 12, 13, direction, speed);
}

void AdafruitMotorController::Motor3_Set(unsigned short direction, double speed)
{
	Motor_Set(MOTOR_3, 4, 3, 2, direction, speed);
}

void AdafruitMotorController::Motor4_Set(unsigned short direction, double speed)
{
	Motor_Set(MOTOR_4, 5, 6, 7, direction, speed);
}

void AdafruitMotorController::Motor_Set(unsigned char motorNum, unsigned char in1, unsigned char in2, unsigned char pwm, unsigned short direction, double speed)
{
	unsigned char currentMotorDirection = MOTOR_DIRECTION_STOP;
	unsigned short currentMotorSpeed = 0;

	// *** Get the current direction and speed, and only adjust if a change is made
	if (motorNum == MOTOR_1)
	{
		currentMotorDirection = m_Motor1_Direction;
		currentMotorSpeed = m_Motor1_Speed;
	}
	else if (motorNum == MOTOR_2)
	{
		currentMotorDirection = m_Motor2_Direction;
		currentMotorSpeed = m_Motor2_Speed;
	}
	else if (motorNum == MOTOR_3)
	{
		currentMotorDirection = m_Motor3_Direction;
		currentMotorSpeed = m_Motor3_Speed;
	}
	else if (motorNum == MOTOR_4)
	{
		currentMotorDirection = m_Motor4_Direction;
		currentMotorSpeed = m_Motor4_Speed;
	}


	// *** Direction
	if (direction != currentMotorDirection)
	{
		if (direction == MOTOR_DIRECTION_FORWARD)
		{
			if (DEBUG_MODE)
				printf("AdafruitMotorController - Set direction: forward\n");

			SetPWM(in2, 0, 4096);
			SetPWM(in1, 4096, 0);
		}

		else if (direction == MOTOR_DIRECTION_BACKWARD)
		{
			if (DEBUG_MODE)
				printf("AdafruitMotorController - Set direction: backward\n");

			SetPWM(in2, 4096, 0);
			SetPWM(in1, 0, 4096);
		}

		else if (direction == MOTOR_DIRECTION_STOP)
		{
			if (DEBUG_MODE)
				printf("AdafruitMotorController - Set direction: stop\n");

			SetPWM(in2, 0, 4096);
			SetPWM(in1, 0, 4096);
		}

		// Update the current motor direction
		if (motorNum == MOTOR_1)
			m_Motor1_Direction = direction;
		else if (motorNum == MOTOR_2)
			m_Motor2_Direction = direction;
		else if (motorNum == MOTOR_3)
			m_Motor3_Direction = direction;
		else if (motorNum == MOTOR_4)
			m_Motor4_Direction = direction;
	}


	// *** Speed

	// Convert from 0.0 -> 1.0 to 0 -> 4095
	speed *= 4095.0;

	// Scale down if needed
	speed *= m_SpeedScale;

	// Convert to short
	unsigned short speedShort = (unsigned short)speed;

	// Set the speed, if it changed for this motor
	if (speedShort != currentMotorSpeed)
	{
		if (DEBUG_MODE)
			printf("AdafruitMotorController - Set speed: %d\n", speedShort);

		SetPWM(pwm, 0, speedShort);

		// Update the current motor speed
		if (motorNum == MOTOR_1)
			m_Motor1_Speed = speedShort;
		else if (motorNum == MOTOR_2)
			m_Motor2_Speed = speedShort;
		else if (motorNum == MOTOR_3)
			m_Motor3_Speed = speedShort;
		else if (motorNum == MOTOR_4)
			m_Motor4_Speed = speedShort;
	}
}

// Connect to the device on I2C and ensure it exists
int AdafruitMotorController::ConnectToI2C(int address)
{
	// Setup the WiringPi I2C library and connect to the motor controller
	m_WiringPi_I2C = wiringPiI2CSetup(address);
	if (m_WiringPi_I2C == -1)
	{
		if (DEBUG_MODE)
			printf("AdafruitMotorController - wiringPi setup failed\n");

		return -1;
	}

	if (DEBUG_MODE)
		printf("AdafruitMotorController - wiringPi connected to Adafruit Motor Controller\n");

	Init();

	SetFrequency(1600.0);

	return 1;
}

void AdafruitMotorController::Init()
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
		printf("AdafruitMotorController - PCA9685 init completed\n");
}

void AdafruitMotorController::SetFrequency(double frequency)
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
		printf("AdafruitMotorController - PCA9685 PWM frequency set\n");
}

void AdafruitMotorController::SetPWM(unsigned char channel, unsigned short on, unsigned short off)
{
	wiringPiI2CWriteReg8(m_WiringPi_I2C, REG_LED0_ON_L + (4 * channel), on & 0xFF);
	wiringPiI2CWriteReg8(m_WiringPi_I2C, REG_LED0_ON_H + (4 * channel), on >> 8);
	wiringPiI2CWriteReg8(m_WiringPi_I2C, REG_LED0_OFF_L + (4 * channel), off & 0xFF);
	wiringPiI2CWriteReg8(m_WiringPi_I2C, REG_LED0_OFF_H + (4 * channel), off >> 8);
}

void AdafruitMotorController::SetAllPWM(unsigned short on, unsigned short off)
{
	wiringPiI2CWriteReg8(m_WiringPi_I2C, REG_ALL_LED_ON_L, on & 0xFF);
	wiringPiI2CWriteReg8(m_WiringPi_I2C, REG_ALL_LED_ON_H, on >> 8);
	wiringPiI2CWriteReg8(m_WiringPi_I2C, REG_ALL_LED_OFF_L, off & 0xFF);
	wiringPiI2CWriteReg8(m_WiringPi_I2C, REG_ALL_LED_OFF_H, off >> 8);
}