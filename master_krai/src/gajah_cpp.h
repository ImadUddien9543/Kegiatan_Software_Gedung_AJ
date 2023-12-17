#ifndef MEKANISME__ 
#define MEKANISME__

#define _USE_MATH_DEFINES
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <master_krai/Mechanism.h>
#include <iostream>
#include <cmath>
#include <cstdint>
#include "common/math_func.h"

//manual joystick

enum class button {
	O = 1, //bulat
	X = 0, //silang
	K = 3, //kotak
	SH = 8, //share
	OP = 9, //options
	PS = 10,
	L1 = 4,
	R1 = 5,
	L = 13, //kiri
	R = 14, //kanan
	U = 15, //atas
	D = 16, //bawah
};

class mekanisme{
public:
	mekanisme(
		ros::NodeHandle &nh,		
		const std::vector<float> &roller_vel,
		const std::vector<float> &roller_x,
		const std::vector<float> &roller_y,
		const std::vector<float> &bldc_range,
		const std::vector<float> &bldc_offset,
		const std::vector<float> &boost_bldc_up,
		const std::vector<float> &boost_bldc_down
		);
	~mekanisme(){};
	void stm_pub();

private:
	ros::NodeHandle &nh_;
	ros::Publisher m_pub;
	ros::Subscriber joy_sub;
	ros::Timer timer_pub;

	bool shoot_mode, bulat, silang, kotak;
	bool is_initialized;
	float chain_lift, inc_chain_lift;
	int shooter_position;

	std::vector<float> roller_vel_;
	std::vector<float> roller_x_;
	std::vector<float> roller_y_;
	std::vector<float> bldc_range_;
	std::vector<float> bldc_offset_;
	std::vector<float> boost_bldc_up_;
	std::vector<float> boost_bldc_down_;
	
	
	master_krai::Mechanism m_;
	sensor_msgs::Joy prev_joy;
	float nos;

	void joy_callback(const sensor_msgs::Joy::ConstPtr &msg);
};


#endif
