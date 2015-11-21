// Class to control the Adafruit CD Motor + Stepper HAT for the Raspberry Pi
#ifndef ADAFRUIT_MOTOR_CONTROLLER_H
#define ADAFRUIT_MOTOR_CONTROLLER_H

// Wiring Pi libraries for accessing I2C
#include <wiringPi.h>
#include <wiringPiI2C.h>

// Motor Identifiers
#define TOTAL_MOTORS				4
#define MOTOR_1						0
#define MOTOR_2						1
#define MOTOR_3						2
#define MOTOR_4						3


// Motor directions
#define MOTOR_DIRECTION_FORWARD		0
#define MOTOR_DIRECTION_BACKWARD	1
#define MOTOR_DIRECTION_STOP		2

class AdafruitMotorController
{
private:
	int		m_WiringPi_I2C;

	// Scale is used if the motors can only accept less voltage than the input voltage. For
	// example if the max voltage for the motor is 1.5V and the input voltage is 5V then
	// a speed scale of 0.3 is used to ensure the voltage is scaled properly for the motor.
	double	m_Motor_Scale[TOTAL_MOTORS];

	// Direction and speed are saved so information is only sent when it changes
	unsigned char	m_Motor_Direction[TOTAL_MOTORS];
	unsigned short	m_Motor_Speed[TOTAL_MOTORS];

	void Init();
	void SetFrequency(double frequency);

	void Motor_Set(unsigned char motorNum, unsigned char in1, unsigned char in2, unsigned char pwm, unsigned short direction, double speed);
	void SetPWM(unsigned char channel, unsigned short on, unsigned short off);
	void SetAllPWM(unsigned short on, unsigned short off);


public:
	AdafruitMotorController();
	~AdafruitMotorController();

	// Connect to I2C given the address
	int ConnectToI2C(int address);

	// Speed scale allows a speed of 1.0 to work with varying power source voltage and motor voltages`
	void Set_Motor_Scale(unsigned char motorNum, double motorScale);

	// Set direction (from #define above) and speed (0.0->1.0)
	void Motor_Update(unsigned char motorNum, unsigned short direction, double speed);
};

#endif
