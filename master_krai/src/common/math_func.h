#ifndef MATH_FUNC__
#define MATH_FUNC__

#define _USE_MATH_DEFINES
#include <cstdint>
#include <cmath>
#include <iostream>
#include <vector>
#include <ros/ros.h>

namespace math {
	class smooth_step{
	public:
		smooth_step(
			const std::vector<float> &in_range,
			const std::vector<float> &out_range,
			const std::string &type
			);
		~smooth_step(){};
		float get_val(const float x);
	
	private:
		std::vector<float> in_range_;
		std::vector<float> out_range_;
		std::vector<float> range;
		std::string type_;		
		float in, out, calc;		
	};

	class logistic{
	public:
		logistic(
			const float &L,
			const float &k,
			const float &x0,
			const std::vector<float> &x_range			
			);
		~logistic(){};
		float get_val(const float input);

	private:
		float L_, k_, x0_;
		std::vector<float> x_range_;
		float in, out;
	};

	int_fast16_t r_to_int(const float val);
	float cos(const float deg);
	float sin(const float deg);	
	float clamp(float val, float min, float max);
	float interp(const float x, std::vector<float> in_range, std::vector<float> out_range);
}

#endif