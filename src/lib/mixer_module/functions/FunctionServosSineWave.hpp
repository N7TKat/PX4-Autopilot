/****************************************************************************
 *
 *   Copyright (c) 2021-2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include "FunctionProviderBase.hpp"

//#include <uORB/topics/actuator_servos.h>

/**
 * SineWave (_/-\_-\_/-,_/-\_-\_/-) 
 */
class FunctionServoSineWave : public FunctionProviderBase
{
public:

	//static FunctionProviderBase *allocate(const Context &context) { return new FunctionServoSineWave(); }

	// void update() override
	// {
	// 	if (_topic.update(&_data)) {
	// 		updateValues(_data.reversible_flags, _thrust_factor, _data.control, actuator_motors_s::NUM_CONTROLS);
	// 	}
	// }

	//float value(OutputFunction func) override { return _data.control[(int)func - (int)OutputFunction::Motor1]; }

	bool allowPrearmControl() const override { return false; }

	//uORB::SubscriptionCallbackWorkItem *subscriptionCallback() override { return &_topic; }

	//bool getLatestSampleTimestamp(hrt_abstime &t) const override { t = _data.timestamp_sample; return t != 0; }

	static inline void updateValues(float *values, int in_sequence_num, int sv_loop, int servo_max)
	{
		const int one_round_num = 1500;
		const int half_round_num = 750;
		int revers_wave = 0;
		int relative_seq = 0;
		int one_seq = 0;
		const float v[9] = {0.0, 0.25, 0.5, 0.75, 1, 0.75, 0.5, 0.25, 0};
		for (int i = 1; i < sv_loop; ++i) {
			if ((in_sequence_num-(one_round_num*i)>0)&&(in_sequence_num-(one_round_num*i)<=one_round_num)) {
				//short 2nd wave and all to 1st wave   				(_/-\_-\_/-,_/-\_-\_/-) 	>> [_/-\_-\_/-]
				
				one_seq = in_sequence_num-(one_round_num*i);	
			}

			if((in_sequence_num>=0)&&(in_sequence_num<one_round_num)){
				//this is 1st wave    								(_/-\_-\_/-)
				one_seq = in_sequence_num;
			}

			if((one_seq-half_round_num>0)&&((one_seq-half_round_num)<=half_round_num)){
					//reverse from 0>> -0.5 >> -1 >> -0.5 >>0     	[_/-\_-\_/-] 				>> [-\_/-]
					relative_seq = one_seq-half_round_num;
					revers_wave = -1;
			}
			else{
					//normal from 0>> 0.5 >> 1 >> 0.5 >>0   		[_/-\_-\_/-] 				>> [_/-\_]
					relative_seq = one_seq;
					revers_wave = 1;
			}


		}

		for (int j = 0; j < servo_max; ++j) {
			if((0<=relative_seq)&&(relative_seq<=750))
			{
				if((0<=relative_seq)&&(relative_seq<=50)){
						//0
						values[j] = v[0]*revers_wave;
				}
				if((51<=relative_seq)&&(relative_seq<=100)){
						//0.25
						values[j] = v[1]*revers_wave;
				}
				if((101<=relative_seq)&&(relative_seq<=150)){
						//0.5
						values[j] = v[2]*revers_wave;
				}
				if((151<=relative_seq)&&(relative_seq<=200)){
						//0.75
						values[j] = v[3]*revers_wave;
				}
				if((201<=relative_seq)&&(relative_seq<=500)){
						//1.00
						values[j] = v[4]*revers_wave;
				}
				if((501<=relative_seq)&&(relative_seq<=550)){
						//0.75
						values[j] = v[5]*revers_wave;
				}
				if((551<=relative_seq)&&(relative_seq<=600)){
						//0.5
						values[j] = v[6]*revers_wave;
				}
				if((601<=relative_seq)&&(relative_seq<=650)){
						//0.25
						values[j] = v[7]*revers_wave;
				}
				if((651<=relative_seq)&&(relative_seq<=750)){
						//0.0
						values[j] = v[8]*revers_wave;
				}
			}
		}
	}

private:
	
};
