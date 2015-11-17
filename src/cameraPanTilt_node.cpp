#include <stdio.h>
#include <time.h>
#include <signal.h>

#include "ros/ros.h"
#include <sensor_msgs/Joy.h>

#include "adafruitServoController.h"


AdafruitServoController _servoController;
unsigned short _servoPanPosition;
unsigned short _servoTiltPosition;


// Called when the node is about to shut down
void NodeShutdown(int sig)
{
	ROS_INFO("Camera Pan-Tilt Node - Shutdown");

	// Center servos
	_servoController.Servo_SetPosition(SERVO_1, 90);
	_servoController.Servo_SetPosition(SERVO_2, 90);

	ros::shutdown();
}

// Callback for receiving joystick messages
void JoystickCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
	// Center camera
	if (msg->buttons[11] != 0)
	{
		_servoPanPosition = 90;
		_servoTiltPosition = 90;
	}

	// Move camera
	else
	{
		// Pan
		if (msg->buttons[3] != 0)
			_servoPanPosition -= 1;
		if (msg->buttons[1] != 0)
			_servoPanPosition += 1;

		// Tilt
		if (msg->buttons[0] != 0)
			_servoTiltPosition += 1;
		if (msg->buttons[2] != 0)
			_servoTiltPosition -= 1;
	}

	// Limits
	if (_servoPanPosition < 0)
		_servoPanPosition = 0;
	else if (_servoPanPosition > 180)
		_servoPanPosition = 180;

	if (_servoTiltPosition < 60)
		_servoTiltPosition = 60;
	else if (_servoTiltPosition > 180)
		_servoTiltPosition = 180;

	// Servo Position
	_servoController.Servo_SetPosition(SERVO_1, _servoPanPosition);
	_servoController.Servo_SetPosition(SERVO_2, _servoTiltPosition);


	ROS_INFO("Pan: %d   Tilt: %d", _servoPanPosition, _servoTiltPosition);
}

int main(int argc, char ** argv)
{
	// Init ROS
	ros::init(argc, argv, "cameraPanTilt_node");
	ros::NodeHandle nodeHandle;

	// Handle the shutdown event
	signal(SIGINT, NodeShutdown);

	// Subscribe to Joystick messages
	ros::Subscriber joystickSubscriber = nodeHandle.subscribe("/joy", 1, JoystickCallback);

	// Adafruit Servo controller - connect and set scale
	if (_servoController.ConnectToI2C(0x40) == -1)
	{
		ROS_INFO("Could not connect to I2C");
		return -1;
	}
	
	// Min, max values are servo dependent
	_servoController.Set_Min_Max(150, 600);

	// Center servos
	_servoPanPosition = 90;
	_servoTiltPosition = 90;
	_servoController.Servo_SetPosition(SERVO_1, 90);
	_servoController.Servo_SetPosition(SERVO_2, 90);

	
	// Receive ROS messages
	ros::spin();

	return 0;
}
