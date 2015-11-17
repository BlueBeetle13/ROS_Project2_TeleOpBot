// Class to control the Adafruit CD Motor + Stepper HAT for the Raspberry Pi
#ifndef ADAFRUIT_SERVO_CONTROLLER_H
#define ADAFRUIT_SERVO_CONTROLLER_H

#include <wiringPi.h>
#include <wiringPiI2C.h>

#define SERVO_1			0
#define SERVO_2			1

class AdafruitServoController
{
private:
	int		m_WiringPi_I2C;

	unsigned short	m_Servo_Min, m_Servo_Max;

	unsigned short	m_Servo1_Position, m_Servo2_Position;

	void Init();
	void SetFrequency(double frequency);

	void Servo_SetPosition(unsigned char motorNum, unsigned char in1, unsigned char in2, unsigned char pwm, unsigned short direction, double speed);
	void SetPWM(unsigned char channel, unsigned short on, unsigned short off);
	void SetAllPWM(unsigned short on, unsigned short off);


public:
	AdafruitServoController();
	~AdafruitServoController();

	int ConnectToI2C(int address);

	// Used to scale 0->180 degrees to pulse length for a specific servo
	void Set_Min_Max(unsigned short min, unsigned short max);

	// Set Servo Position (0->180 degrees)
	void Servo_SetPosition(unsigned char servoNum, unsigned short positio);
};

#endif
