#include "gajah_cpp.h"

#define A 10

mekanisme::mekanisme(
	ros::NodeHandle &nh,	
	const std::vector<float> &roller_vel,
	const std::vector<float> &roller_x,
	const std::vector<float> &roller_y,
	const std::vector<float> &bldc_range,
	const std::vector<float> &bldc_offset,
	const std::vector<float> &boost_bldc_up,
	const std::vector<float> &boost_bldc_down,
	):
	nh_(nh), roller_vel_(roller_vel), roller_x_(roller_x), roller_y_(roller_y),
	bldc_range_(bldc_range), bldc_offset_(bldc_offset), 
	boost_bldc_up_(boost_bldc_up), boost_bldc_down_(boost_bldc_down)
{
	ROS_INFO("Creating mekanisme");
	
	shoot_mode = bulat = silang = false;
	chain_lift = inc_chain_lift = 0.0f;
	shooter_position = 0;
	is_initialized = false;

	m_pub = nh_.advertise<master_krai::Mechanism>("/robot/mekanisme", 10);
	joy_sub = nh_.subscribe("/joy", 1, &kinematik::joy_callback, this);	
}

void kinematik::joy_callback(const sensor_msgs::Joy::ConstPtr &msg){
	if(!is_initialized){
		prev_joy = *msg;
		is_initialized = true;
		return;
	}

	if(msg->buttons != prev_joy.buttons){
		prev_joy = *msg;
		if(msg->buttons[static_cast<int>(button::PS)] != prev_joy.buttons[static_cast<int>(button::PS)]){
			if(msg->buttons[static_cast<int>(button::PS)] == 1) shoot_mode = !shoot_mode;
			ROS_INFO("PS");
		}
		else if(msg->buttons[static_cast<int>(button::O)] != prev_joy.buttons[static_cast<int>(button::O)]){
			if(msg->buttons[static_cast<int>(button::O)] == 1) bulat = !bulat;
			ROS_INFO("BULAT");
		}
		else if(msg->buttons[static_cast<int>(button::X)] != prev_joy.buttons[static_cast<int>(button::X)]){
			if(msg->buttons[static_cast<int>(button::X)] == 1) silang = !silang;
			ROS_INFO("SILANG");
		}
		else if(msg->buttons[static_cast<int>(button::U)] != prev_joy.buttons[static_cast<int>(button::U)]){
			if(msg->buttons[static_cast<int>(button::U)] == 1){
				shooter_position += 1;
				shooter_position = static_cast<int>(math::clamp(shooter_position, 0, roller_x_.size() - 1));
				ROS_INFO("ATAS");
			}
		}
		else if(msg->buttons[static_cast<int>(button::D)] != prev_joy.buttons[static_cast<int>(button::D)]){
			if(msg->buttons[static_cast<int>(button::D)] == 1){
				shooter_position -= 1;
				shooter_position = static_cast<int>(math::clamp(shooter_position, 0, roller_x_.size() - 1));
				ROS_INFO("BAWAH");
			}
		}
	}
	
	//lontar ring
	else if(msg->buttons[static_cast<int>(button::K)] != prev_joy.buttons[static_cast<int>(button::K)]){
		if(msg->buttons[static_cast<int>(button::K)] == 1) kotak = true;
		else kotak = false;
	}

	//gripper rantai
	else if(msg->buttons[static_cast<int>(button::L1)] != prev_joy.buttons[static_cast<int>(button::L1)]){
		if(msg->buttons[static_cast<int>(button::L1)] == 1){
			inc_chain_lift += 0.25f;
			inc_chain_lift = math::clamp(inc_chain_lift, 0.0f, 1.0f);
			chain_lift = 1.0f * std::pow(700.0f, inc_chain_lift);
		}
		else{
			inc_chain_lift = chain_lift = 0.0f;
		}
	}

	else if(msg->buttons[static_cast<int>(button::R1)] != prev_joy.buttons[static_cast<int>(button::R1)]){
		if(msg->buttons[static_cast<int>(button::R1)] == 1){
			inc_chain_lift += 0.25f; 
			inc_chain_lift = math::clamp(inc_chain_lift, 0.0f, 1.0f);
			chain_lift = -1.0f * std::pow(700.0f, inc_chain_lift);
		}
		else{
			inc_chain_lift = chain_lift = 0.0f;
		}
	}
}

void kinematik::stm_pub(){
	if(shoot_mode == false){
		m_.y_penembak = 0;
		m_.x_penembak = 0;
	}
	else if(shoot_mode == true){
		m_.y_penembak = roller_y_[shooter_position];
		m_.y_penembak = roller_x_[shooter_position];
	}

	m_.kotak = kotak;
	m_.bulat = bulat;
	m_.silang = silang;
	m_.lift_gripper = math::r_to_int(chain_lift);
	m_.lift_gripper2 = 0;
	m_pub.publish(rpm);
}


int main(int argc, char **argv){
	std::string node_name = "mekanisme";
	ros::init(argc, argv, node_name);

    ros::NodeHandle nh;
    ros::NodeHandle arg_n("~");
	
    std::vector<float> roller_vel;
	std::vector<float> &roller_x;
	std::vector<float> &roller_y;
	std::vector<float> bldc_range;
	std::vector<float> bldc_offset;
	std::vector<float> boost_bldc_up;
	std::vector<float> boost_bldc_down;

    arg_n.getParam("ring_mri", roller_vel);
    arg_n.getParam("sudut_x", roller_x);
    arg_n.getParam("sudut_y", roller_y);
    arg_n.getParam("bldc_pwm_range", bldc_range);
    arg_n.getParam("bldc_offset", bldc_offset);
    arg_n.getParam("boost_bldc_1", boost_bldc_up);
    arg_n.getParam("boost_bldc_2", boost_bldc_down);

    kinematik robot(nh, roller_vel, roller_x, roller_y, bldc_range, bldc_offset, boost_bldc_up, boost_bldc_down);
    ros::Timer timerpublish = nh.createTimer(ros::Duration(1.0/50.0), std::bind(&kinematik::stm_pub, robot));
    ros::spin();
    return 0;
}
//\b[[:alpha:]_][[:alnum:]_]*\b

//'(?:(::)\s*)?{{\b[[:alpha:]_][[:alnum:]_]*\b}}\s*(::)\s*'

//source.c++ meta.function.c++ meta.block.c++ meta.function-call.c++ meta.group.c++ punctuation.section.group.begin.c++
//"scope": "(keyword.operator.comparison.c | keyword.operator.ternary.c | keyword.operator.assignment.c | keyword.operator.assignment.augmented.c | keyword.operator.arithmetic.c 