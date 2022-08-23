# Haptic_Device_Arduino

## Drain Pipe Cleaning Robot

We made a robot that can clean inside drain pipe by moving through it

Reference : Atsushi Kakogawa(2016), Design of a Multilink-articulated Wheeled Inspection Robot for Winding Pipelines:AIRO-Ⅱ, IEEE

<img src="https://user-images.githubusercontent.com/86711384/185090482-feae1dd0-aaf1-4dd9-bba4-3ca795b865d5.png" width="100" height="100"/>

## Controller : motor_haptic_1
<img src="https://user-images.githubusercontent.com/86711384/185094723-173a92b6-c9d8-4331-bbb4-4b33bd3f4b2e.png" width="100" height="100"/>
   
With Arduino Uno & Dynamixel XH430V350R (https://www.robotis.com/shop/item.php?it_id=902-0129-000)
we made a controller that can control front two links intuitively & minutely.   
   
We added haptic function which helps the controller to share senses with the robot.   
   
There were some limits imitating haptic function with arduino. 
But we implemented haptic functionality by only sharing controller's sense to the robot. 
   
## link_robcon
With using motor_haptic_1 code, we linked controller and robot.   
We got the medium angle of the controller by forward kinematics.   
We substituted the angles of controller to the motor in the robot, and achieved same movement with each other.   
   
In the code,   
a,b : Each length of controller links   
theta2, theta5 : Upper & Lower motor's position of the controller   
theta3 : Angle between two links of controller's upper part   

## Other files
They are for checking & updating dynamixel's id and baudrate.   
"debug" is for debugging the current velocity, position, etc of the dynamixel.   

