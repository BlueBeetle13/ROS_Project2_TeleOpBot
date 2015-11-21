// Class to control the Adafruit CD Motor + Stepper HAT for the Raspberry Pi
#ifndef ADAFRUIT_SERVO_CONTROLLER_H
#define ADAFRUIT_SERVO_CONTROLLER_H

#include <wiringPi.h>
#include <wiringPiI2C.h>

#define TOTAL_SERVOS	16

#define SERVO_1			0
#define SERVO_2			1
#define SERVO_3			2
#define SERVO_4			3
#define SERVO_5			4
#define SERVO_6			5
#define SERVO_7			6
#define SERVO_8			7
#define SERVO_9			8
#define SERVO_10		9
#define SERVO_11		10
#define SERVO_12		11
#define SERVO_13		12
#define SERVO_14		13
#define SERVO_15		14
#define SERVO_16		15

class AdafruitServoController
{
private:
	int		m_WiringPi_I2C;

	unsigned short	m_Servo_Min[TOTAL_SERVOS], m_Servo_Max[TOTAL_SERVOS];

	unsigned short	m_Servo_Position[TOTAL_SERVOS];

	void Init();
	void SetFrequency(double frequency);

	void SetPWM(unsigned char channel, unsigned short on, unsigned short off);
	void SetAllPWM(unsigned short on, unsigned short off);


public:
	AdafruitServoController();
	~AdafruitServoController();

	// Connect to I2C given the address
	int ConnectToI2C(int address);

	// Used to scale 0->180 degrees to pulse length for a specific types of servos, each servo can
	// have different values so all 16 servos could be different
	void Servo_SetMinMax(unsigned char servoNum, unsigned short min, unsigned short max);

	// Set Servo Position (0->180 degrees)
	void Servo_SetPosition(unsigned char servoNum, unsigned short position);
};

#endif
