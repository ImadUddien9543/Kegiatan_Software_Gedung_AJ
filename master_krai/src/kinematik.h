#ifndef KINEMATIK__
#define KINEMATIK__

#define _USE_MATH_DEFINES

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <master_krai/Wheel.h>
#include <iostream>
#include <cmath>
#include <cstdint>
#include "common/math_func.h"

//manual joystick

enum class button {
	A = 2, //segitiga
	L2 = 6,
	R2 = 7
};

enum class axes {
	LX = 0,
	LY = 1,
	L2 = 4,
	R2 = 5
};

class kinematik{
public:
	kinematik(
		ros::NodeHandle &nh,
		const float &l,
		const std::vector<float> &alpha,
		const std::vector<float> &vel_coeff,
		const std::vector<float> &joy_range,
		const std::vector<float> &omni_range
		);
	~kinematik(){};
	void stm_pub();

private:
	ros::NodeHandle &nh_;
	ros::Publisher omni_pwm;
	ros::Subscriber joy_sub;
	ros::Timer timer_pub;

	float vel_x, vel_y, vel_z;	
	float l_;
	std::vector<float> alpha_;
	float rf, rb, lf, lb;
	std::vector<float> joy_range_;
    std::vector<float> omni_range_;
    std::vector<float> vel_coeff_;
	master_krai::Wheel rpm;
	float nos;
	std::vector<float> nos_in;
	std::vector<float> nos_out;


	void joy_callback(const sensor_msgs::Joy::ConstPtr &joy);
};


#endif