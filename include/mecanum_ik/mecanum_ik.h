#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include "mecanum_ik/vector4_msg.h"

#ifndef MECANUM_IK_H
#define MECANUM_IK_H

namespace mecanum_ik
{
class Mecanum4Wheels {
	private:

                double r;       // wheel radius (assume using the same wheels >
                double vd;      // vertical distance between wheels on the sam>
                double hd;      // horizontal distance between wheess on the s>
                double vd_hd;   // sum of vd and hd

	public:

	Mecanum4Wheels(double r, double vd, double hd);

	mecanum_ik::vector4_msg getLocalSpeed(geometry_msgs::Vector3 velocity);

	mecanum_ik::vector4_msg getGlobalSpeed(geometry_msgs::Vector3 velocity, double angle_position);
};
}

#endif
