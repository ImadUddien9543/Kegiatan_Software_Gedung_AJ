#include "kinematik.h"

kinematik::kinematik(
	ros::NodeHandle &nh,
	const float &l,	
	const std::vector<float> &alpha,
	const std::vector<float> &vel_coeff,
	const std::vector<float> &joy_range,
	const std::vector<float> &omni_range
	) 
	:nh_(nh), l_(l), alpha_(alpha), vel_coeff_(vel_coeff), joy_range_(joy_range), omni_range_(omni_range)
{
	ROS_INFO("Creating kinematik");
	omni_pwm = nh_.advertise<master_krai::Wheel>("/robot/omni_pwm", 10);
	joy_sub = nh_.subscribe("/joy", 1, &kinematik::joy_callback, this);
	timer_pub = nh_.createTimer(ros::Duration(1.0/50.0), std::bind(&kinematik::stm_pub, this));
}

void kinematik::joy_callback(const sensor_msgs::Joy::ConstPtr &joy){
	vel_x = joy->axes[static_cast<int>(axes::LX)] * 1.0f * vel_coeff_[0];
	vel_y = joy->axes[static_cast<int>(axes::LY)] * 1.0f * vel_coeff_[1];

	if(joy->buttons[static_cast<int>(button::R2)] == 1 || joy->buttons[static_cast<int>(button::L2)] == 1){
		vel_z = (joy->axes[static_cast<int>(axes::R2)] - joy->axes[static_cast<int>(axes::L2)]) * vel_coeff_[2];;
	}	
	else if(joy->buttons[static_cast<int>(button::R2)] == 0 || joy->buttons[static_cast<int>(button::L2)] == 0){
		vel_z = 0.0f;
	}

	if(joy->buttons[static_cast<int>(button::A)] == 1) {
		nos += 0.025f;		
	}
	else if(joy->buttons[static_cast<int>(button::A)] == 0) nos -= 0.085f;

}

void kinematik::stm_pub(){
	//inverse kinematik ini sudah sangat disederhanakan dari master_krai/scripts/kinematics.py 
	rf = (-1.0f * math::sin(alpha_[0]) * vel_x) + (1.0f * math::cos(alpha_[0]) * vel_y) + (l_ * vel_z);
	lf = (-1.0f * math::sin(alpha_[1]) * vel_x) + (1.0f * math::cos(alpha_[1]) * vel_y) + (l_ * vel_z);
	lb = (-1.0f * math::sin(alpha_[2]) * vel_x) + (1.0f * math::cos(alpha_[2]) * vel_y) + (l_ * vel_z);
	rb = (-1.0f * math::sin(alpha_[3]) * vel_x) + (1.0f * math::cos(alpha_[3]) * vel_y) + (l_ * vel_z);

	rpm.R_Front	= math::r_to_int(rf * 100);
	rpm.L_Front	= math::r_to_int(lf * 100);
	rpm.L_Back 	= math::r_to_int(lb * 100);
	rpm.R_Back  = math::r_to_int(rb * 100);
	omni_pwm.publish(rpm);
}

int main(int argc, char **argv){
	std::string node_name = "kinematik";
	ros::init(argc, argv, node_name);

	ros::NodeHandle nh;
	ros::NodeHandle arg_n("~");

	float l;
	std::vector<float> joy_range;
	std::vector<float> omni_range;
	std::vector<float> alpha;
	std::vector<float> vel_coeff;

	arg_n.getParam("l_omni", l);
	arg_n.getParam("alpha_omni", alpha);
	arg_n.getParam("vel_coeff", vel_coeff);
	arg_n.getParam("joy_range", joy_range);
	arg_n.getParam("omni_range", omni_range);    

	kinematik robot(nh, l, alpha, vel_coeff, joy_range, omni_range);
	ros::spin();
	return 0;
}
