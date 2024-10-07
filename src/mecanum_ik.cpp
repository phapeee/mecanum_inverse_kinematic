#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <mecanum_ik/vector4_msg.h>
#include <cmath>
#include "mecanum_ik/mecanum_ik.h"

namespace mecanum_ik{

// constructor
Mecanum4Wheels::Mecanum4Wheels(double radius, double v_dist, double h_dist)
{
	r = radius;
	vd = v_dist;
	hd = h_dist;
	vd_hd = (vd/2 + hd/2) / r;	// pre calculate constant
}

// calculate the local speed of bot with given 3D vector velocity in which x and y components are desired local linear velocities and z component desried rotaiional velocity
//  | 1 -1 -(l+w)/r | |x|
//  | 1  1  (l+w)/r |*|y|
//  | 1 -1  (l+w)/r | |z|
//  | 1  1 -(l+w)/r |
// return value is a 4D vector in which x is top left, y is top right, z is bottom right, w is bottom left
mecanum_ik::vector4_msg Mecanum4Wheels::getLocalSpeed(geometry_msgs::Vector3 velocity)
{
	mecanum_ik::vector4_msg wheel_ang_vel;	// variable to hold result
	// pre calculation
	double mul_z = vd_hd * velocity.z;
	double div_x = velocity.x / r;
	double div_y = velocity.y / r;
	wheel_ang_vel.x = div_x - div_y - mul_z;
	wheel_ang_vel.y = div_x + div_y + mul_z;
	wheel_ang_vel.z = div_x - div_y + mul_z;
	wheel_ang_vel.w = div_x + div_y - mul_z;
	return wheel_ang_vel;
}

// Similar with above function except the given vector is a global desired velocities
mecanum_ik::vector4_msg Mecanum4Wheels::getGlobalSpeed(geometry_msgs::Vector3 velocity, double angle_position)
{
	double cos_phi = cos(angle_position);
	double sin_phi = sin(angle_position);
	geometry_msgs::Vector3 local_vel;
	local_vel.z = velocity.z;
	local_vel.x = cos_phi * velocity.x + sin_phi * velocity.y;
	local_vel.y = cos_phi * velocity.y - sin_phi * velocity.x;
	return getLocalSpeed(local_vel);
}

}
