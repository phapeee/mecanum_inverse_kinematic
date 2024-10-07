Mecanum_inverse_kinematic is a tool to calculate the inverse kinematic of a 4-wheel Mecanum robot. Given the desired linear and angular velocities in the global or local frame, the tool calculates the angular velocities of the 4 motors.

# Getting started: Assume you have a workspace named catkin_ws at the root, steps to download the library:
  1. $ cd ~/catkin_ws/src
  2. $ git clone https://github.com/phapeee/mecanum_inverse_kinematic.git
  3. $ cd ~/catkin_ws
  4. $ catkin_make
# If the package is successfully built, you can start to implement the library by:  
#include "mecanum_ik/mecanum_ik.h";  
#include <mecanum_ik/vector4_msg.h>;
#define WHEEL_RADIUS (<your_wheel_radius>)    // the radius of the Mecanum wheel.
#define V_DISTANCE (<your_vertical_distance>)  // distance between the center of the front and rear axles.
#define H_DISTANCE (<your_horizontal_distance>)  // distance between the centers of the front/rear wheels.

mecanum_ik::Mecanum4Wheels mecanum_op(WHEEL_RADIUS, V_DISTANCE, H_DISTANCE);  //Initiate the tool as a global variable
mecanum_ik::vector4_msg wheel_angular_velocities;  // 4D vector contains the angular velocity of each motor (wheel_angular_velocities.x is front left, wheel_angular_velocities.y is front right, wheel_angular_velocities.z is back right, and wheel_angular_velocities.w is back left).

# There are 2 main functions of the Mecanum4Wheels class, getLocalSpeed(geometry_msgs::Vector3 velocity) and getGlobalSpeed(geometry_msgs::Vector3 velocity, double angle_position).
The getLocalSpeed function accepts a geometry_msgs::Vector3 type variable in which the x and y components are the desired local x and y velocities relative to the robot's coordinate, and the z component is the rotational velocity of the robot.
The getGlobalSpeed function is similar to the getLocalSpeed, however, the function treats input velocity as the desired global velocity relative to the global static frame, and an extra input of angle_position which tells the robot's angular position relative to the global frame.
Both functions return a 4D vector containing each motor's angular velocity.
# Example:
geometry_msgs::Vector3 velocity;
// The distance unit is defined by the unit you use in the WHEEL_RADIUS, V_DISTANCE, H_DISTANCE. The time unit is defined by the sample_period that you sample the robot's pose. The angular unit is in radian.
velocity.x = 5;
velocity.y = 5;
velocity.z = 3;
wheel_angular_velocities = getLocalSpeed(velocity);
// Then you can publish the wheel_angular_velocities as mecanum_ik::vector4_msg message;
