#include <stdio.h>
#include <time.h>
#include <signal.h>

#include "ros/ros.h"
#include <sensor_msgs/Joy.h>

#include "adafruitMotorController.h"


AdafruitMotorController _motorController;


// Called when the node is about to shut down
void NodeShutdown(int sig)
{
	ROS_INFO("Tank Tracks Node - Shutdown");

	ros::shutdown();
}

// Callback for receiving joystick messages
void JoystickCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
	float leftRightAxis = msg->axes[0];
	float upDownAxis = msg->axes[1];

	float V = (1.0 - fabs(leftRightAxis)) * (upDownAxis/ 1.0) + upDownAxis;
	float W = (1.0 - fabs(upDownAxis)) * (leftRightAxis / 1.0) + leftRightAxis;

	float rightTrackSpeed = (V + W) / 2.0;
	float leftTrackSpeed = (V - W)/ 2.0;

	// Left Track
	if (leftTrackSpeed == 0)
		_motorController.Motor4_Set(MOTOR_DIRECTION_STOP, 0.0);
	else if (leftTrackSpeed > 0)
		_motorController.Motor4_Set(MOTOR_DIRECTION_FORWARD, fabs(leftTrackSpeed));
	else if (leftTrackSpeed < 0)
		_motorController.Motor4_Set(MOTOR_DIRECTION_BACKWARD, fabs(leftTrackSpeed));

	// Right Track
	if (rightTrackSpeed == 0)
		_motorController.Motor3_Set(MOTOR_DIRECTION_STOP, 0.0);
	else if (rightTrackSpeed > 0)
		_motorController.Motor3_Set(MOTOR_DIRECTION_FORWARD, fabs(rightTrackSpeed));
	else if (rightTrackSpeed < 0)
		_motorController.Motor3_Set(MOTOR_DIRECTION_BACKWARD, fabs(rightTrackSpeed));

	ROS_INFO("L: %.4f   R: %.4f", leftTrackSpeed, rightTrackSpeed);
}

int main(int argc, char ** argv)
{
	// Init ROS
	ros::init(argc, argv, "tankTracks_node");
	ros::NodeHandle nodeHandle;

	// Handle the shutdown event
	signal(SIGINT, NodeShutdown);

	// Subscribe to Joystick messages
	ros::Subscriber joystickSubscriber = nodeHandle.subscribe("/joy", 1, JoystickCallback);

	// Adafruit Motor controller - connect and set scale
	if (_motorController.ConnectToI2C(0x60) == -1)
	{
		ROS_INFO("Could not connect to I2C");
		return -1;
	}
	_motorController.SetSpeedScale(0.3);

	ros::spin();

	/*delay(5000);

	motorController.Motor2_Set(MOTOR_DIRECTION_FORWARD, 1.0);
	motorController.Motor3_Set(MOTOR_DIRECTION_FORWARD, 0.5);
	motorController.Motor4_Set(MOTOR_DIRECTION_BACKWARD, 0.25);

	printf("1.0\n");
	motorController.Motor1_Set(MOTOR_DIRECTION_FORWARD, 1.0);

	delay(5000);

	printf("0.75\n");
	motorController.Motor1_Set(MOTOR_DIRECTION_FORWARD, 0.75);

	delay(5000);

	printf("0.75\n");
	motorController.Motor1_Set(MOTOR_DIRECTION_FORWARD, 0.75);

	delay(5000);

	printf("0.5\n");
	motorController.Motor1_Set(MOTOR_DIRECTION_FORWARD, 0.5);

	delay(5000);

	printf("-0.5\n");
	motorController.Motor1_Set(MOTOR_DIRECTION_BACKWARD, 0.5);

	delay(5000);

	printf("0\n");
	motorController.Motor1_Set(MOTOR_DIRECTION_STOP, 0);



	while (ros::ok())
	{
		ros::spinOnce();

		delay(5000);
	}*/

	return 0;
}
																	
