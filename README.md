ROS - Project2 - TeleOpBot
==========================
ROS tele-operated robot with joystick. Tracked robot with camera, telemetry data connects to 2nd computer with joystick and HUD


Purpose
-------

The purpose of this project was to continue my learning of ROS by creating something more complicated and more robot-like than the previous project. Also I wanted to create a project distributed over more than 1 computer.


Description
-----------

I've always thought that robots look cool with tank tracks so this robot will be using them. In addition there will be a camera mounted that can pan and tilt in order to look around. I will be controlling this robot using a joystick connected to my laptop and will view the camera feed on my laptop screen.


Design
------

The main computer on the robot will be a Raspberry Pi 2, model B. This computer is MUCH faster than the original RPi and I installed Ubuntu Trusty Tehr along with ROS.

For tank tracks I used the track and wheel set from Tamiya (http://www.robotshop.com/ca/en/tamiya-track-and-wheel-set.html). The tracks are controlled with this DC motor and gear box (http://www.robotshop.com/ca/en/tamiya-double-gearbox-70168.html) that allow for controlling each track independently.

To control the tracks I choose a motor hat from Adafruit (https://www.adafruit.com/products/2348) that allows me to connect up to 4 DC motors and controls everything through I2C. I experimented with an L293D H-Bridge chip myself but this is a much cleaner and easier to use option. Adafruit supplied a python script but I prefer to work in C++ so I created a C++ class that I feel gives a cleaner interface. I also allow for motors that don't match perfectly with the power source (in this case max 1.5V DC motors and 5V power).

To control the pan/tilt servos I used the Adafruit PWM servo driver (https://www.adafruit.com/products/815). This also comes in hat form but I choose just the breakout version. This also uses I2C. I had an Arduino sketch to work off, but once again opted to create a C++ class as I will likely be using both this and the motor hat often in the future.

For the camera pan/tilt I went with the RPi camera and a servo pan/tilt kit (http://www.robotshop.com/ca/en/lynxmotion-pan-and-tilt-kit-aluminium2.html).

In order to communicate over wifi effectively I needed to power the wifi dongle separately from the RPi USB connector, as it does not provide enough amps and the wifi dongle keeps powering down and losing too many packets. I modified a powered USB hub to lengthen the USB cord, block out the power line (forcing it to use the hubs power), and added wire leads to connect to my batteries.

For a power source I used a 5V 4400mAh power bank to power the RPi, motor hat, and servo driver logic. I wanted to use this for the motors, but it has a shutoff that shuts down power when too few amps are being drawn. To power the DC motors, servos, and wifi dongle I used a 3.7V 2000mAh LiPo battery connected to a LiPo charger and a power booster that gives 5V and 1 amp.

The pictures below show the assembled robot:

![alt text](http://www.typhoonsoftware.com/GitHub/ROS_Project2_TeleOpBot/teleOpProject1.jpg "Project Design 1")

- 1) Tank Track kit
- 2) 5V 4400mAh battery
- 3) 3.7V 2000mAh battery (under the white battery)
- 4) Powered USB hub and wifi dongle
- 5) 2 servos with pan/tilt kit
- 6) RPi camera


![alt text](http://www.typhoonsoftware.com/GitHub/ROS_Project2_TeleOpBot/teleOpProject2.jpg "Project Design 2")

- 7) Raspberry Pi 2, model B
- 8) Motor hat
- 9) Servo driver


![alt text](http://www.typhoonsoftware.com/GitHub/ROS_Project2_TeleOpBot/teleOpProject3.jpg "Project Design 3")

- 10) Gearbox with 1.5V max DC Motors
- 11) Powerboost for providing 5V
- 12) LiPo charger



