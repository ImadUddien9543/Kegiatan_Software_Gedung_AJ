#include <ros/ros.h>
#include <kinematik/omni.h>

#define _USE_MATH_DEFINES
#include <cmath>

class LL_ctrl {
	public:
		LL_ctrl(ros::NodeHandle &nh);
		~LL_ctrl(){};
		void pwm_motor_publish();		
	private:
		ros::NodeHandle &nh_;
		ros::Publisher pwm_pub;				
		
		float w1, w2, w3, w4, w5, w6;
		float vx, vy, vz;
		float prev_vx, prev_vy, prev_vz;	
		kinematik::omni wheel_vel;
								
		//degree to radian
		float _cos(float &x);
		float _sin(float &x);
		float limit_rpm();
};

LL_ctrl::LL_ctrl(ros::NodeHandle &nh): nh_(nh){
	ROS_INFO("creating kinematik");

	pwm_pub = nh_.advertise<kinematik::omni>("/robot/motor", 10);		
	vx = 0, vy = 0, vz = 0;
	prev_vx = 0, prev_vy = 0, prev_vz = 0;
	w1 = 0, w2 = 0, w3 = 0, w4 = 0, w5 = 0, w6 = 0;

};

float LL_ctrl::_cos(float &x){	
	float converted =  (x / 180.0 * M_PI) * 1.0f;
	float ab = std::fabs(converted - 0);
	float tolerance = (1e-08 + (1e-05 * converted));
	if(ab <= tolerance){
		return 0;
	}
	else{
		return std::cos(converted);
	}
}

float LL_ctrl::_sin(float &x){	
	float converted =  (x / 180.0 * M_PI) * 1.0f;
	float ab = std::fabs(converted - 0);
	float tolerance = (1e-08 + (1e-05 * converted));
	if(ab <= tolerance){
		return 0;
	}
	else{
		return std::sin(converted);
	}
}

void LL_ctrl::pwm_motor_publish(){	
}

int main(int argc, char **argv){
	ros::init(argc, argv, "kinematik");
	ros::NodeHandle nh;
	ros::NodeHandle arg_n("~");
	
	LL_ctrl motor_node(nh);
	ros::Timer timer_pub = nh.createTimer(ros::Duration(1/50), std::bind(&LL_ctrl::pwm_motor_publish, motor_node));

	ros::spin();

	return 0;
}