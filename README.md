Mecanum_inverse_kinematic is a tool to calculate the inverse kinematic of a 4-wheel Mecanum robot. Given the desired linear and angular velocities in the global or local frame, the tool calculates the angular velocities of the 4 motors.

# Getting started: Assume you have a workspace named catkin_ws at the root, steps to download the library:
  1. $ cd ~/catkin_ws/src
  2. $ git clone https://github.com/phapeee/mecanum_inverse_kinematic.git
  3. $ cd ~/catkin_ws
  4. $ catkin_make
# If the package is successfully built, you can implement the library by:
  #include "mecanum_ik/mecanum_ik.h";  
  #include <mecanum_ik/vector4_msg.h>;
  
  // the radius of the Mecanum wheel<br/>
  #define WHEEL_RADIUS (<your_wheel_radius>)    
  
  // distance between the center of the front and rear axles<br/>
  #define V_DISTANCE (<your_vertical_distance>)  
  
  // distance between the centers of the front/rear wheels<br/>
  #define H_DISTANCE (<your_horizontal_distance>)  

  //Initiate the tool as a global variable<br/>
  mecanum_ik::Mecanum4Wheels mecanum_op(WHEEL_RADIUS, V_DISTANCE, H_DISTANCE);
  
  // 4D vector contains the angular velocity of each motor (wheel_angular_velocities.x is front left, wheel_angular_velocities.y is front right,<br/> 
  // wheel_angular_velocities.z is back right, and wheel_angular_velocities.w is back left)<br/>
  mecanum_ik::vector4_msg wheel_angular_velocities;

# There are 2 main functions of the Mecanum4Wheels class, getLocalSpeed(geometry_msgs::Vector3 velocity) and getGlobalSpeed(geometry_msgs::Vector3 velocity, double angle_position).
The getLocalSpeed function accepts a geometry_msgs::Vector3 type variable in which the x and y components are the desired local x and y velocities relative to the robot's coordinate, and the z component is the rotational velocity of the robot.<br/>
The getGlobalSpeed function is similar to the getLocalSpeed, however, the function treats input velocity as the desired global velocity relative to the global static frame, and an extra input of angle_position which is the robot's angular position relative to the global frame.<br/>
Both functions return a 4D vector containing each motor's angular velocity.
# Example:
  // desired local velocity<br/> 
  geometry_msgs::Vector3 velocity;
  
  // The distance unit is defined by the unit you use in the WHEEL_RADIUS, V_DISTANCE, H_DISTANCE.<br/>
  // The time unit is defined by the sample_period that you sample the robot's pose.<br/> 
  // The angular unit is in radian.<br/>
  velocity.x = 5;  // psudo linear velocity in local x direction.<br/>
  velocity.y = 5;  // psudo linear velocity in local y direction.<br/>
  velocity.z = 3;  // psudo angular velocity.<br/>
  
  // Calculate the angular velocities of 4 motors.<br/>
  wheel_angular_velocities = getLocalSpeed(velocity);<br/>
  // Then you can publish the wheel_angular_velocities as mecanum_ik::vector4_msg message;
