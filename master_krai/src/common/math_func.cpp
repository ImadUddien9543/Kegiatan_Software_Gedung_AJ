#include "math_func.h"


namespace math {
	smooth_step::smooth_step(
		const std::vector<float> &in_range,
		const std::vector<float> &out_range,
		const std::string &type
		): in_range_(in_range), out_range_(out_range), type_(type){
		in = out = 0;		
	}

	float smooth_step::get_val(const float x){ //smooth_step (0 <= x <= 1)
		in = clamp(x, in_range_[0], in_range_[1]);

		if(type_.compare("order_2") == 0){
			calc = 6.0f * std::pow(x, 5) - 15.0f * std::pow(x, 4) + 10 * std::pow(x, 3);
		}
		else if(type_.compare("order_3") == 0){
			calc = -20.0f * std::pow(x, 7) + 70.0f * std::pow(x, 6) - 84.0f * std::pow(x, 5) + 35 * std::pow(x, 4);
		}
		else if(type_.compare("order_4") == 0){
			calc = 70.0f * std::pow(x, 9) - 315.0f * std::pow(x, 8) + 540.0f * std::pow(x, 7) - 420.0f * std::pow(x, 6) + 126.0f * std::pow(x, 5);
		}
		else{
			calc = -2.0f * std::pow(x, 3) + 3.0f * std::pow(x, 2);
		}
		float out_ = interp(calc, in_range_, out_range_);
		out = clamp(out_, out_range_[0], out_range_[1]);
		return out;
	}


	logistic::logistic(
		const float &L,
		const float &k,
		const float &x0,
		const std::vector<float> &x_range		
		): L_(L), k_(k), x0_(x0), x_range_(x_range){
		out = 0;
	}

	float logistic::get_val(const float input){
		in = clamp(input, x_range_[0], x_range_[1]);
		float exp = -k_ * (in - x0_);
		out = L_ / (1 + std::pow(M_E, exp));		
		return out;
	}

	int_fast16_t r_to_int(const float val){
		return (val < 0.0f) ? static_cast<int_fast16_t>(std::floor(val)) : static_cast<int_fast16_t>(std::ceil(val));
	}

	float cos(const float deg){
		float converted =  (deg / 180.0f * M_PI) * 1.0f;
		float ab = std::fabs(converted);
		return (ab <= 1e-10f) ? 0.0f : std::cos(converted);	
	}

	float sin(const float deg){
		float converted =  (deg / 180.0f * M_PI) * 1.0f;
		float ab = std::fabs(converted);	
		return (ab <= 1e-10f) ? 0.0f : std::sin(converted);
	}

	float clamp(float val, float min, float max){
		if(val > max) return max;
		else if(val < min) return min;
		else return val;
	}

	float interp(const float x, std::vector<float> in_range, std::vector<float> out_range){
		float in_ = clamp(x, in_range[0], in_range[1]);
		float out_ = ((x - in_range[0]) * (out_range[1] - out_range[0]) / (in_range[1] - in_range[0])) + out_range[0];
		return clamp(out_, out_range[0], out_range[1]);
	}	
}