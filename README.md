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


ROS
---

One of the many great things about ROS is that there is already many useful nodes already created. Keeping this in mind I wanted to quickly develop a project using as many existing technologies as possible, rather than re-invent the wheel myself. So instead of writing code to capture and send the camera feed, display the received images, and joystick control, I used existing code.

The Robot

- I wrote a node for controlling the tank tracks. This node subscribes to the joystick messages controls the DC motors accordingly. I translate the x-y axis information from the joystick into left/right track speed and direction. I created a C++ class for interfacing the Adafruit motor hat which allows me to easily control 4 DC motos by simply giving a direction and speed. I also allow for a scaling factor in case the power input doesn't match perfectly with the max voltage for the DC motors. In this case I set the scaling to 0.3 since I have 5V power and the motors use 1.5V max.

- I wrote another node for controlling the pan/tilt of the servos. This node also subscribes to the joystick messages and listens for buttons, then sets the pan and tilt servo positions. Once again I wrote a C++ class to interface with the Adafruit servo driver allowing me to easily set the servo position.

- For capturing the RPi camera images I found a node online (https://github.com/fpasteau/raspicam_node) that works great.

Laptop

- I am using my Macbook Pro but I installed Virtual Box and The Ubuntu Trusty Tehr and then installed ROS. Someone wrote a package for ROS already that reads the values from the joystick, and publishes messages when the values change, so this saved me writing this myself.

- I also found a package for displaying images which also works great and subscribes to the image messages from the raspicam_node on the robot.

Setup

Once again this is very easy with ROS, I just modified the /etc/hosts file on both the robot and my laptop so each knows the hostname of the RPi which will be the master. In the past I had written my own networking code, which was fun to write but ROS makes this so much easier. I created launch files on both the RPi and my laptop and it was a pleasant surprise how well everything came together. In total it only took a week for this project, including all the time building the robot, which was also a lot of fun.

/etc/hosts on RPi
```
192.168.0.20    localhost
192.168.0.20    pibot
192.168.0.21    Mac-Ubuntu-ROS
```

/etc/hosts on laptop
```
192.168.0.21    localhost
192.168.0.21    Mac-Ubuntu-ROS
192.168.0.20    pibot
```

On laptop:
ROS_MASTER_URI=http://pibot:11311
ROS_IP=192.168.0.21


Additional Thoughts
-------------------

- The antennae on the wifi dongle isn't very good so it would help the range to modify this.

- Tank Tracks look cool but they seem to come off too often. This might have to do with my cheap $10 track kit but for the next project I might just go with wheels.

- It would be nice if I could give topics priority, so that the joystick control of the tracks and camera pan/tilt had a higher priority than the camera feed. Again the wifi dongle worked just ok, not great so every once in a while  the camera feed got backed up or slowed the networking down and caused the joystick control to be unresponsive. When I created my own networking library in the past (before ROS) I added communication priority for just this situation.
