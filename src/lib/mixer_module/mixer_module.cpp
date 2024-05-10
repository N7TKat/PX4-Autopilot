/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include "mixer_module.hpp"

#include <uORB/Publication.hpp>
#include <px4_platform_common/log.h>

using namespace time_literals;


struct FunctionProvider {
	using Constructor = FunctionProviderBase * (*)(const FunctionProviderBase::Context &context);
	FunctionProvider(OutputFunction min_func_, OutputFunction max_func_, Constructor constructor_)
		: min_func(min_func_), max_func(max_func_), constructor(constructor_) {}
	FunctionProvider(OutputFunction func, Constructor constructor_)
		: min_func(func), max_func(func), constructor(constructor_) {}

	OutputFunction min_func;
	OutputFunction max_func;
	Constructor constructor;
};

static const FunctionProvider all_function_providers[] = {
	// Providers higher up take precedence for subscription callback in case there are multiple
	{OutputFunction::Constant_Min, &FunctionConstantMin::allocate},
	{OutputFunction::Constant_Max, &FunctionConstantMax::allocate},
	{OutputFunction::Motor1, OutputFunction::MotorMax, &FunctionMotors::allocate},
	{OutputFunction::Servo1, OutputFunction::ServoMax, &FunctionServos::allocate},
	{OutputFunction::Offboard_Actuator_Set1, OutputFunction::Offboard_Actuator_Set6, &FunctionActuatorSet::allocate},
	{OutputFunction::Landing_Gear, &FunctionLandingGear::allocate},
	{OutputFunction::Landing_Gear_Wheel, &FunctionLandingGearWheel::allocate},
	{OutputFunction::Parachute, &FunctionParachute::allocate},
	{OutputFunction::Gripper, &FunctionGripper::allocate},
	{OutputFunction::RC_Roll, OutputFunction::RC_AUXMax, &FunctionManualRC::allocate},
	{OutputFunction::Gimbal_Roll, OutputFunction::Gimbal_Yaw, &FunctionGimbal::allocate},
};

MixingOutput::MixingOutput(const char *param_prefix, uint8_t max_num_outputs, OutputModuleInterface &interface,
			   SchedulingPolicy scheduling_policy, bool support_esc_calibration, bool ramp_up) :
	ModuleParams(&interface),
	_output_ramp_up(ramp_up),
	_scheduling_policy(scheduling_policy),
	_support_esc_calibration(support_esc_calibration),
	_max_num_outputs(max_num_outputs < MAX_ACTUATORS ? max_num_outputs : MAX_ACTUATORS),
	_interface(interface),
	_control_latency_perf(perf_alloc(PC_ELAPSED, "control latency")),
	_param_prefix(param_prefix)
{
	/* Safely initialize armed flags */
	_armed.armed = false;
	_armed.prearmed = false;
	_armed.ready_to_arm = false;
	_armed.lockdown = false;
	_armed.force_failsafe = false;
	_armed.in_esc_calibration_mode = false;

	px4_sem_init(&_lock, 0, 1);

	initParamHandles();

	for (unsigned i = 0; i < MAX_ACTUATORS; i++) {
		_failsafe_value[i] = UINT16_MAX;
	}

	updateParams();

	_outputs_pub.advertise();
}

MixingOutput::~MixingOutput()
{
	perf_free(_control_latency_perf);
	px4_sem_destroy(&_lock);

	cleanupFunctions();

	_outputs_pub.unadvertise();
}

void MixingOutput::initParamHandles()
{
	char param_name[17];

	for (unsigned i = 0; i < _max_num_outputs; ++i) {
		snprintf(param_name, sizeof(param_name), "%s_%s%d", _param_prefix, "FUNC", i + 1);
		_param_handles[i].function = param_find(param_name);
		snprintf(param_name, sizeof(param_name), "%s_%s%d", _param_prefix, "DIS", i + 1);
		_param_handles[i].disarmed = param_find(param_name);
		snprintf(param_name, sizeof(param_name), "%s_%s%d", _param_prefix, "MIN", i + 1);
		_param_handles[i].min = param_find(param_name);
		snprintf(param_name, sizeof(param_name), "%s_%s%d", _param_prefix, "MAX", i + 1);
		_param_handles[i].max = param_find(param_name);
		snprintf(param_name, sizeof(param_name), "%s_%s%d", _param_prefix, "FAIL", i + 1);
		_param_handles[i].failsafe = param_find(param_name);
	}

	snprintf(param_name, sizeof(param_name), "%s_%s", _param_prefix, "REV");
	_param_handle_rev_range = param_find(param_name);
}

void MixingOutput::printStatus() const
{
	PX4_INFO("Param prefix: %s", _param_prefix);
	perf_print_counter(_control_latency_perf);

	if (_wq_switched) {
		PX4_INFO("Switched to rate_ctrl work queue");
	}

	PX4_INFO_RAW("Channel Configuration:\n");

	for (unsigned i = 0; i < _max_num_outputs; i++) {
		PX4_INFO_RAW("Channel %i: func: %3i, value: %i, failsafe: %d, disarmed: %d, min: %d, max: %d\n", i,
			     (int)_function_assignment[i], _current_output_value[i],
			     actualFailsafeValue(i), _disarmed_value[i], _min_value[i], _max_value[i]);
	}
}

void MixingOutput::updateParams()
{
	ModuleParams::updateParams();

	bool function_changed = false;

	for (unsigned i = 0; i < _max_num_outputs; i++) {
		int32_t val;

		if (_param_handles[i].function != PARAM_INVALID && param_get(_param_handles[i].function, &val) == 0) {
			if (val != (int32_t)_function_assignment[i]) {
				function_changed = true;
			}

			// we set _function_assignment[i] later to ensure _functions[i] is updated at the same time
		}

		if (_param_handles[i].disarmed != PARAM_INVALID && param_get(_param_handles[i].disarmed, &val) == 0) {
			_disarmed_value[i] = val;
		}

		if (_param_handles[i].min != PARAM_INVALID && param_get(_param_handles[i].min, &val) == 0) {
			_min_value[i] = val;
		}

		if (_param_handles[i].max != PARAM_INVALID && param_get(_param_handles[i].max, &val) == 0) {
			_max_value[i] = val;
		}

		if (_min_value[i] > _max_value[i]) {
			uint16_t tmp = _min_value[i];
			_min_value[i] = _max_value[i];
			_max_value[i] = tmp;
		}

		if (_param_handles[i].failsafe != PARAM_INVALID && param_get(_param_handles[i].failsafe, &val) == 0) {
			_failsafe_value[i] = val;
		}
	}

	_reverse_output_mask = 0;
	int32_t rev_range_param;

	if (_param_handle_rev_range != PARAM_INVALID && param_get(_param_handle_rev_range, &rev_range_param) == 0) {
		_reverse_output_mask = rev_range_param;
	}

	if (function_changed) {
		_need_function_update = true;
	}
}

void MixingOutput::cleanupFunctions()
{
	if (_subscription_callback) {
		_subscription_callback->unregisterCallback();
		_subscription_callback = nullptr;
	}

	for (int i = 0; i < MAX_ACTUATORS; ++i) {
		delete _function_allocated[i];
		_function_allocated[i] = nullptr;
		_functions[i] = nullptr;
	}
}

bool MixingOutput::updateSubscriptions(bool allow_wq_switch)
{
	if (!_need_function_update || _armed.armed) {
		return false;
	}

	// must be locked to potentially change WorkQueue
	lock();

	_has_backup_schedule = false;

	if (_scheduling_policy == SchedulingPolicy::Auto) {
		// first clear everything
		unregister();
		_interface.ScheduleClear();

		bool switch_requested = false;

		// potentially switch work queue if we run motor outputs
		for (unsigned i = 0; i < _max_num_outputs; i++) {
			// read function directly from param, as _function_assignment[i] is updated later
			int32_t function;

			if (_param_handles[i].function != PARAM_INVALID && param_get(_param_handles[i].function, &function) == 0) {
				if (function >= (int32_t)OutputFunction::Motor1 && function <= (int32_t)OutputFunction::MotorMax) {
					switch_requested = true;
				}
			}
		}

		if (allow_wq_switch && !_wq_switched && switch_requested) {
			if (_interface.ChangeWorkQueue(px4::wq_configurations::rate_ctrl)) {
				// let the new WQ handle the subscribe update
				_wq_switched = true;
				_interface.ScheduleNow();
				unlock();
				return false;
			}
		}
	}

	// Now update the functions
	PX4_DEBUG("updating functions");

	cleanupFunctions();

	const FunctionProviderBase::Context context{_interface, _param_thr_mdl_fac.reference()};
	int provider_indexes[MAX_ACTUATORS] {};
	int next_provider = 0;
	int subscription_callback_provider_index = INT_MAX;
	bool all_disabled = true;

	for (int i = 0; i < _max_num_outputs; ++i) {
		int32_t val;

		if (_param_handles[i].function != PARAM_INVALID && param_get(_param_handles[i].function, &val) == 0) {
			_function_assignment[i] = (OutputFunction)val;

		} else {
			_function_assignment[i] = OutputFunction::Disabled;
		}

		for (int p = 0; p < (int)(sizeof(all_function_providers) / sizeof(all_function_providers[0])); ++p) {
			if (_function_assignment[i] >= all_function_providers[p].min_func &&
			    _function_assignment[i] <= all_function_providers[p].max_func) {
				all_disabled = false;
				int found_index = -1;

				for (int existing = 0; existing < next_provider; ++existing) {
					if (provider_indexes[existing] == p) {
						found_index = existing;
						break;
					}
				}

				if (found_index >= 0) {
					_functions[i] = _function_allocated[found_index];

				} else {
					_function_allocated[next_provider] = all_function_providers[p].constructor(context);

					if (_function_allocated[next_provider]) {
						_functions[i] = _function_allocated[next_provider];
						provider_indexes[next_provider++] = p;

						// lowest provider takes precedence for scheduling
						if (p < subscription_callback_provider_index && _functions[i]->subscriptionCallback()) {
							subscription_callback_provider_index = p;
							_subscription_callback = _functions[i]->subscriptionCallback();
						}

					} else {
						PX4_ERR("function alloc failed");
					}
				}

				break;
			}
		}
	}

	hrt_abstime fixed_rate_scheduling_interval = 4_ms; // schedule at 250Hz

	if (_max_topic_update_interval_us > fixed_rate_scheduling_interval) {
		fixed_rate_scheduling_interval = _max_topic_update_interval_us;
	}

	if (_scheduling_policy == SchedulingPolicy::Auto) {
		if (_subscription_callback) {
			if (_subscription_callback->registerCallback()) {
				PX4_DEBUG("Scheduling via callback");
				_has_backup_schedule = true;
				_interface.ScheduleDelayed(50_ms);

			} else {
				PX4_ERR("registerCallback failed, scheduling at fixed rate");
				_interface.ScheduleOnInterval(fixed_rate_scheduling_interval);
			}

		} else if (all_disabled) {
			_interface.ScheduleOnInterval(_lowrate_schedule_interval);
			PX4_DEBUG("Scheduling at low rate");

		} else {
			_interface.ScheduleOnInterval(fixed_rate_scheduling_interval);
			PX4_DEBUG("Scheduling at fixed rate");
		}
	}

	setMaxTopicUpdateRate(_max_topic_update_interval_us);
	_need_function_update = false;

	_actuator_test.reset();

	unlock();

	_interface.mixerChanged();

	return true;
}

void MixingOutput::setMaxTopicUpdateRate(unsigned max_topic_update_interval_us)
{
	_max_topic_update_interval_us = max_topic_update_interval_us;

	if (_subscription_callback) {
		_subscription_callback->set_interval_us(_max_topic_update_interval_us);
	}
}

void MixingOutput::setAllMinValues(uint16_t value)
{
	for (unsigned i = 0; i < MAX_ACTUATORS; i++) {
		_param_handles[i].min = PARAM_INVALID;
		_min_value[i] = value;
	}
}

void MixingOutput::setAllMaxValues(uint16_t value)
{
	for (unsigned i = 0; i < MAX_ACTUATORS; i++) {
		_param_handles[i].max = PARAM_INVALID;
		_max_value[i] = value;
	}
}

void MixingOutput::setAllFailsafeValues(uint16_t value)
{
	for (unsigned i = 0; i < MAX_ACTUATORS; i++) {
		_param_handles[i].failsafe = PARAM_INVALID;
		_failsafe_value[i] = value;
	}
}

void MixingOutput::setAllDisarmedValues(uint16_t value)
{
	for (unsigned i = 0; i < MAX_ACTUATORS; i++) {
		_param_handles[i].disarmed = PARAM_INVALID;
		_disarmed_value[i] = value;
	}
}

void MixingOutput::unregister()
{
	if (_subscription_callback) {
		_subscription_callback->unregisterCallback();
	}
}

bool MixingOutput::update()
{
	// check arming state
	if (_armed_sub.update(&_armed)) {
		_armed.in_esc_calibration_mode &= _support_esc_calibration;

		if (_ignore_lockdown) {
			_armed.lockdown = false;
		}

		/* Update the armed status and check that we're not locked down.
		 * We also need to arm throttle for the ESC calibration. */
		_throttle_armed = (_armed.armed && !_armed.lockdown) || _armed.in_esc_calibration_mode;
	}

	// only used for sitl with lockstep
	bool has_updates = _subscription_callback && _subscription_callback->updated();

	// update topics
	for (int i = 0; i < MAX_ACTUATORS && _function_allocated[i]; ++i) {
		_function_allocated[i]->update();
	}

	if (_has_backup_schedule) {
		_interface.ScheduleDelayed(50_ms);
	}

	automatic_hardware_testing_s automatic_hardware_testing;
	// waiting for automatic hardware testing command
	// if(_automatic_hardware_testing_sub.update(&automatic_hardware_testing)){
	// 	ACTUATOR_RUN = true;
	// }

	//fake QGC Start Test
	if (!_param_auto_pfl_en.get()){
		// PX4_WARN("AUTO HW TEST params : disbled");			//(_param_auto_pfl_en.get() == 0)
		_in_actuator_test_mode = false;
	}									//_param_auto_pfl_en Disable
	else {									//(_param_auto_pfl_en.get() == 1)
		_in_actuator_test_mode = true;
		motor_run_time_ms = _param_auto_pfl_mt_time.get()*1000;
		servo_loop = _param_auto_pfl_sv_loop.get();
	}

	is_motor_function = (bool)(((int)_function_assignment[0]>=actuator_test_s::FUNCTION_MOTOR1) && ((int)_function_assignment[0]<actuator_test_s::FUNCTION_SERVO1));
	is_servo_function = (bool)(((int)_function_assignment[0]>= actuator_test_s::FUNCTION_SERVO1) && ((int)_function_assignment[0]<(actuator_test_s::FUNCTION_SERVO1+actuator_test_s::MAX_NUM_SERVOS)));

	// PX4_ERR(" is_motor_function is : %s", is_motor_function ? "true" : "false");
	// PX4_ERR(" is_servo_function is : %s", is_servo_function ? "true" : "false");
	// PX4_ERR("ACTUATOR_RUN loop = %i: ********************************************",loop_num_2);
	++loop_num_2;

	if(_automatic_hardware_testing_sub.update(&automatic_hardware_testing) && ((is_motor_function)||(is_servo_function)))
	{	/*if automatic_hardware_testing is update and this is motor and servo fuction*/
		// PX4_ERR("Test mode at Mixer : %i",automatic_hardware_testing.test_mode);
		/*Stamp update time*/
		update_timeout = hrt_absolute_time();
		timeout_timer = true;
				// PX4_ERR("1: ACTUATOR_RUN is : %s", ACTUATOR_RUN ? "true" : "false");
				// PX4_ERR("2: test_mode is : %d", automatic_hardware_testing.test_mode_int);
				// PX4_WARN("3: _function_assignment[i] = %.2f ***************",(double)_function_assignment[1]);
				// PX4_ERR("ACTUATOR_RUN loop = %i: ***************",loop_num_2);
				// 	++loop_num_2;
		//PX4_ERR(" inprogress is : %s", automatic_hardware_testing.inprogress ? "true" : "false");


		if ((int)automatic_hardware_testing.test_mode == 0){
			//reset motor_test
			motor_test = false;
			do_next_motor = true;

			//reset servo_test
			servo_test = false;
			do_next_servo = true;

			//ACTUATOR_SHOULD_EXIT = true;
			//PX4_INFO("4.1: I NA SOON");
		}

		else if (((int)automatic_hardware_testing.test_mode == automatic_hardware_testing_s::TEST_MODE_MOTOR_ONLY) && (is_motor_function))
		{
			//PX4_INFO("MIXER : AUTO_PFL_MODE 1");
			motor_test = true;
			done_all_motor = false;
			// PX4_WARN(" motor_test in case 1 is : %s", motor_test ? "true" : "false");
			//PX4_ERR("Motor loop = %i: ***************",loop_num);
			++loop_num;
			// PX4_INFO("4.2 : I NA NUEANG");
		}


		else if (((int)automatic_hardware_testing.test_mode == automatic_hardware_testing_s::TEST_MODE_SERVO_ONLY) && (is_servo_function))
		{
			PX4_INFO("AUTO_PFL_MODE 2");
			servo_test = true;

			// PX4_ERR("Servo loop = %i: ***************",loop_num);
			// ++loop_num_2;
		}

		else if (((int)automatic_hardware_testing.test_mode == automatic_hardware_testing_s::TEST_MODE_MOTOR_THEN_SERVO)&&(is_motor_function))
		{
			PX4_INFO("AUTO_PFL_MODE 3");
			//motor_test = true;
			//servo_test = true;
		}

		else if (((int)automatic_hardware_testing.test_mode == automatic_hardware_testing_s::TEST_MODE_STOP) && ((is_motor_function)||(is_servo_function)))
		{
			// PX4_INFO("AUTO_PFL_MODE 4");
			servo_test = false;
			motor_test = false;
			done_all_motor = true;
			PX4_INFO("Stop Testing via FlightLync Command");
		}

		else{
			//reset motor_test
			motor_test = false;
			do_next_motor = true;
			//reset servo_test
			servo_test = false;
			do_next_servo = true;
		}

	}
	if(!_automatic_hardware_testing_sub.update(&automatic_hardware_testing) && (timeout_timer) && (hrt_elapsed_time(& update_timeout) > 12_ms) && ((is_motor_function)||(is_servo_function))){
		/*auto reset*/
		//PX4_WARN("AUTO RESET TRIGGERED : NO Update");

		// //reset motor_test
		// motor_test = false;
		// do_next_motor = true;
		done_all_motor = true;
		// //reset servo_test
		// servo_test = false;
		// do_next_servo = true;
		// timeout_timer = false;

	}

	if(!_automatic_hardware_testing_sub.update(&automatic_hardware_testing)){
		//PX4_WARN("automatic_hardware_testing not update");
	}

	// actuator_test.update (Default)
	if (!_in_actuator_test_mode){
		_actuator_test.update(_max_num_outputs, _param_thr_mdl_fac.get());

		// get output values
		float outputs[MAX_ACTUATORS];
		bool all_disabled = true;
		_reversible_mask = 0;

		for (int i = 0; i < _max_num_outputs; ++i) {

			if (_functions[i]) {
				all_disabled = false;
				//PX4_WARN("Hi");
				//PX4_INFO("outputs[MAX_ACTUATORS] = %.2f[%i],_functions = %s, _function_assignment[i] = %.2f",(double)outputs[i],MAX_ACTUATORS,_functions[i] ? "true" : "false",(double)_function_assignment[i]);
				if (_armed.armed || (_armed.prearmed && _functions[i]->allowPrearmControl())) {
					outputs[i] = _functions[i]->value(_function_assignment[i]);

				} else {
					outputs[i] = NAN;
				}
				//PX4_INFO("outputs[MAX_ACTUATORS] = %.2f[%i],_functions = %s, _function_assignment[i] = %.2f",(double)outputs[i],MAX_ACTUATORS,_functions[i] ? "true" : "false",(double)_function_assignment[i]);
				_reversible_mask |= (uint32_t)_functions[i]->reversible(_function_assignment[i]) << i;

			} else {	//not use actuator (Like servo 2,3,...,max servo)
				outputs[i] = NAN;
			}

		}

		if (!all_disabled) {
			if (!_armed.armed && !_armed.manual_lockdown) {
					_actuator_test.overrideValues(outputs, _max_num_outputs);
				}

			limitAndUpdateOutputs(outputs, has_updates);
		}
	}//_param_auto_pfl_en Disable

	/*Actuator_Test Logic Function (Auto Preflight test)
		update and generate actuator value to motor and servo
	*/
	if (_in_actuator_test_mode){

		/*update parameter to ActuatorValueMix Function*/
		//PX4_WARN("in actuator test mode");
		_actuator_value_mix.update(_max_num_outputs,
						 _param_thr_mdl_fac.get(),
						 _param_auto_pfl_mode.get(),
						 _param_auto_pfl_sv_loop.get(),
						 _param_auto_pfl_mt_time.get(),
						 _param_auto_pfl_en.get(),
						 in_sequence_num,
						 servo_max);


		/*Actuator_Test Logic Function : Get output values*/
		float outputs[MAX_ACTUATORS];
		bool all_disabled = true;
		_reversible_mask = 0;

		/*Defualt*/
		for (int i = 0; i < _max_num_outputs; ++i) {

			if (_functions[i]) {
				all_disabled = false;
				//PX4_WARN("Hi");
				//PX4_INFO("outputs[MAX_ACTUATORS] = %.2f[%i],_functions = %s, _function_assignment[i] = %.2f",(double)outputs[i],MAX_ACTUATORS,_functions[i] ? "true" : "false",(double)_function_assignment[i]);
				if (_armed.armed || (_armed.prearmed && _functions[i]->allowPrearmControl())) {
					outputs[i] = _functions[i]->value(_function_assignment[i]);

				} else {
					outputs[i] = NAN;
				}
				//PX4_INFO("outputs[MAX_ACTUATORS] = %.2f[%i],_functions = %s, _function_assignment[i] = %.2f",(double)outputs[i],MAX_ACTUATORS,_functions[i] ? "true" : "false",(double)_function_assignment[i]);
				_reversible_mask |= (uint32_t)_functions[i]->reversible(_function_assignment[i]) << i;

			} else {	//not use actuator (Like servo 2,3,...,max servo)
				outputs[i] = NAN;
			}

		}

		// /*Set point*/
		// for (int i = 0; i < _max_num_outputs; ++i)
		// {

		// 	/*Get output values > Motor Command Check*/
		// 	if(is_motor_function) //automatic_hardware_testing.test_mode == 1
		// 	{
		// 		if(((int)_function_assignment[i]>=actuator_test_s::FUNCTION_MOTOR1)&&(int)_function_assignment[i]<actuator_test_s::FUNCTION_SERVO1)
		// 		{
		// 			if (_functions[i]) {
		// 				all_disabled = false;
		// 				//PX4_WARN("Hi");
		// 				//PX4_INFO("outputs[MAX_ACTUATORS] = %.2f[%i],_functions = %s, _function_assignment[i] = %.2f",(double)outputs[i],MAX_ACTUATORS,_functions[i] ? "true" : "false",(double)_function_assignment[i]);
		// 				if (_armed.armed || (_armed.prearmed && _functions[i]->allowPrearmControl())) {
		// 					outputs[i] = _functions[i]->value(_function_assignment[i]);

		// 				} else {
		// 					outputs[i] = NAN;
		// 				}
		// 				//PX4_INFO("outputs[MAX_ACTUATORS] = %.2f[%i],_functions = %s, _function_assignment[i] = %.2f",(double)outputs[i],MAX_ACTUATORS,_functions[i] ? "true" : "false",(double)_function_assignment[i]);
		// 				_reversible_mask |= (uint32_t)_functions[i]->reversible(_function_assignment[i]) << i;

		// 			} else {	//not use actuator (Like motor 2,3,...,max motor)
		// 				outputs[i] = NAN;
		// 			}
		// 		}
		// 	}

		// 	/*Get output values > Servo Command Check*/
		// 	if(is_servo_function)
		// 	{
		// 		if(((int)_function_assignment[i]>=actuator_test_s::FUNCTION_SERVO1)&&(int)_function_assignment[i]<(actuator_test_s::FUNCTION_SERVO1+actuator_test_s::MAX_NUM_SERVOS))
		// 		{
		// 			if (_functions[i]) {
		// 				all_disabled = false;
		// 				//PX4_WARN("Hi");
		// 				//PX4_INFO("outputs[MAX_ACTUATORS] = %.2f[%i],_functions = %s, _function_assignment[i] = %.2f",(double)outputs[i],MAX_ACTUATORS,_functions[i] ? "true" : "false",(double)_function_assignment[i]);
		// 				if (_armed.armed || (_armed.prearmed && _functions[i]->allowPrearmControl())) {
		// 					outputs[i] = _functions[i]->value(_function_assignment[i]);

		// 				} else {
		// 					outputs[i] = NAN;
		// 				}
		// 				//PX4_INFO("outputs[MAX_ACTUATORS] = %.2f[%i],_functions = %s, _function_assignment[i] = %.2f",(double)outputs[i],MAX_ACTUATORS,_functions[i] ? "true" : "false",(double)_function_assignment[i]);
		// 				_reversible_mask |= (uint32_t)_functions[i]->reversible(_function_assignment[i]) << i;

		// 			} else {	//not use actuator (Like servo 2,3,...,max servo)
		// 				outputs[i] = NAN;
		// 			}
		// 		}
		// 	}

		// }

		// /*End of set point*/

		if (!all_disabled) {
			if (!_armed.armed && !_armed.manual_lockdown) {
				//PX4_ERR(" motor_test is : %s *******************************************", motor_test ? "true" : "false");
				//_actuator_value_mix.overrideValues(outputs, _max_num_outputs, _param_auto_pfl_mode.get(),motor_oder[1]);
				//PX4_ERR("actuator_test");

				/*Actuator_Test Logic Function : Do motor test*/
				if((motor_test)&&(!done_all_motor)&&(is_motor_function)){

					// PX4_ERR("actuator_test : motor_test starting");
					// PX4_ERR("do next motor is : %s", do_next_motor ? "true" : "false");

					if(do_next_motor){			/*Set time and reset logic to do next motor*/
						/*kill logic*/
						do_next_motor = false;
						PX4_INFO("actuator_test : motor_test starting **this message will show one time");
						/*Open Next logic*/
						do_motor_sequence = true;
						in_motor_sequence = true;
						in_sequence_num = 1;

						/*Time Stamped*/
						motor_sequence_delay = hrt_absolute_time();

						/*Debug*/
						//PX4_INFO("DO Test MOTOR [%i]",motor_id+1);
						PX4_INFO("Do Test MOTOR [%i] ", motor_id+1);
					}

					if((do_motor_sequence)&&(hrt_elapsed_time(& motor_sequence_delay) < ((double)motor_run_time_ms*1000))){
						/*When do_motor_sequence is true ,Do this Command Until motor_run_time */

						/*motor_oder[motor_id]*/
						_actuator_value_mix.overrideValues(outputs, _max_num_outputs, _param_auto_pfl_mode.get(),motor_oder[motor_id],false);
						++in_sequence_num;

						/*Time Stamped*/
						//in_motor_sequence_delay = hrt_absolute_time();

						/*Debug*/
						//PX4_WARN("[%i]%.0f ms",in_sequence_num,(double)(hrt_absolute_time()-in_motor_sequence_delay)/1000);
						// PX4_ERR("outputs[MAX_ACTUATORS] = %5.2f[0],_functions = %s, _function_assignment[0] = %.2f",(double)outputs[0],_functions[0] ? "true" : "false",(double)_function_assignment[0]);
						// PX4_WARN("outputs[MAX_ACTUATORS] = %5.2f[1],_functions = %s, _function_assignment[1] = %.2f",(double)outputs[1],_functions[1] ? "true" : "false",(double)_function_assignment[1]);
						// PX4_WARN("outputs[MAX_ACTUATORS] = %5.2f[2],_functions = %s, _function_assignment[2] = %.2f",(double)outputs[2],_functions[2] ? "true" : "false",(double)_function_assignment[2]);
						// PX4_WARN("outputs[MAX_ACTUATORS] = %5.2f[3],_functions = %s, _function_assignment[3] = %.2f",(double)outputs[3],_functions[3] ? "true" : "false",(double)_function_assignment[3]);
						// PX4_WARN("outputs[MAX_ACTUATORS] = %5.2f[4],_functions = %s, _function_assignment[4] = %.2f",(double)outputs[4],_functions[4] ? "true" : "false",(double)_function_assignment[4]);
						// PX4_WARN("outputs[MAX_ACTUATORS] = %5.2f[5],_functions = %s, _function_assignment[5] = %.2f",(double)outputs[5],_functions[5] ? "true" : "false",(double)_function_assignment[5]);
						// PX4_INFO("Test MOTOR [%i] at SEQUENCE[%i]",MOTOR_ID+1,MOTOR_SEQUENCE);
					}


					if((do_motor_sequence)&&(hrt_elapsed_time(& motor_sequence_delay) > ((double)motor_run_time_ms*1000))){
						/*When do_motor_sequence is true and now is motor_run_time end, Stop this Command */

						/*kill logic*/
						do_motor_sequence = false;

						/*Open Next logic*/
						do_next_motor_delay_timer = true;

						/*Time Stamped*/
						do_next_motor_delay = hrt_absolute_time();

					}


					if((do_next_motor_delay_timer)&&(hrt_elapsed_time(& do_next_motor_delay) > 5_s)){
						/*When do_next_motor_delay_timer is true and now is delay time, Send to Next motor and Stop this Command */

						/*kill logic*/
						do_next_motor_delay_timer = false;

						/*Open Next logic*/

						if(motor_id < (motor_max - 1)){
							do_next_motor = true;
							++motor_id;

							/*Status*/
							PX4_INFO("Test Next MOTOR [%i] ", motor_id + 1);

						}
						else{ //(motor_id>=motor_max)
							do_next_motor = false;
							done_all_motor = true;

							/*Status*/
							PX4_INFO("Finish last motor");
						}

					}

					if(done_all_motor){
						/*kill logic*/
						//done_all_motor = false;

						motor_test_is_done = true;


						/*Reset to Default*/
						motor_test = false;
						do_next_motor = true;
						do_motor_sequence = false;
						do_next_motor_delay_timer = false;
						motor_sequence_delay = 0;
						//in_motor_sequence_delay = 0;
						motor_id = 0;


						/*set for pub*/
						automatic_hardware_testing.inprogress = false;
						automatic_hardware_testing.success = true;

						/*Status*/
						// PX4_INFO("_____________________________________________________");
						// PX4_INFO("Finish");
						// PX4_ERR(" motor_test is : %s", motor_test ? "true" : "false");
						// PX4_INFO("_____________________________________________________");

					}
					_automatic_hardware_testing_pub.publish(automatic_hardware_testing);
					//PX4_ERR(" Test motor | inprogress is : %s", automatic_hardware_testing.inprogress ? "true" : "false");
					//PX4_ERR(" Test motor | success is : %s", automatic_hardware_testing.success ? "true" : "false");


				}

				if(servo_test){
					//PX4_ERR("motor_test");
					if(do_next_servo){
						do_servo_sequence = true;
						//IN_MOTOR_SEQUENCE = true;
						do_next_servo = false;
						servo_sequence_delay= hrt_absolute_time();
						in_sequence_num = 1;

						//PX4_INFO("DO Test MOTOR [%i]",MOTOR_ID+1);
					}

					if((do_servo_sequence)&&(hrt_elapsed_time(& servo_sequence_delay) < ((long unsigned int)(servo_loop_time_ms*1000*servo_loop)))){
						_actuator_value_mix.overrideValues(outputs, _max_num_outputs, _param_auto_pfl_mode.get(),servo_oder[servo_id],true);

						PX4_WARN("[%i]%.0f ms",in_sequence_num,(double)(hrt_absolute_time()-in_servo_sequence_delay)/1000);
						in_servo_sequence_delay = hrt_absolute_time();
						++in_sequence_num;
							PX4_ERR("outputs[MAX_ACTUATORS] = %5.2f[0],_functions = %s, _function_assignment[0] = %.2f",(double)outputs[0],_functions[0] ? "true" : "false",(double)_function_assignment[0]);
							PX4_WARN("outputs[MAX_ACTUATORS] = %5.2f[1],_functions = %s, _function_assignment[1] = %.2f",(double)outputs[1],_functions[1] ? "true" : "false",(double)_function_assignment[1]);
							// PX4_WARN("outputs[MAX_ACTUATORS] = %5.2f[2],_functions = %s, _function_assignment[2] = %.2f",(double)outputs[2],_functions[2] ? "true" : "false",(double)_function_assignment[2]);
							// PX4_WARN("outputs[MAX_ACTUATORS] = %5.2f[3],_functions = %s, _function_assignment[3] = %.2f",(double)outputs[3],_functions[3] ? "true" : "false",(double)_function_assignment[3]);
							// PX4_WARN("outputs[MAX_ACTUATORS] = %5.2f[4],_functions = %s, _function_assignment[4] = %.2f",(double)outputs[4],_functions[4] ? "true" : "false",(double)_function_assignment[4]);
							// PX4_WARN("outputs[MAX_ACTUATORS] = %5.2f[5],_functions = %s, _function_assignment[5] = %.2f",(double)outputs[5],_functions[5] ? "true" : "false",(double)_function_assignment[5]);

							// PX4_INFO("Test MOTOR [%i] at SEQUENCE[%i]",MOTOR_ID+1,MOTOR_SEQUENCE);
					}

							//((double)motor_run_time_ms*1000) >> servo sequence time
					if((do_servo_sequence)&&(hrt_elapsed_time(& servo_sequence_delay) > ((long unsigned int)(servo_loop_time_ms*1000*servo_loop)))){
						do_servo_sequence = false;
						do_next_servo_delay_timer = true;
						do_next_servo_delay = hrt_absolute_time();

					}


					if((do_next_servo_delay_timer)&&(hrt_elapsed_time(& do_next_servo_delay) > 5_s)){
						do_next_servo_delay_timer = false;
						do_next_servo = true;
						++servo_id;
						if(servo_id<servo_max){
							PX4_INFO("Test Next Servo [%i] ",servo_id+1);
						}

					}

					if(servo_id>=servo_max){
						servo_test = false;
						servo_test_done = true;
						PX4_INFO("****************************************");
						PX4_INFO("Finish");

						//Reset
						do_next_servo = true;
						do_servo_sequence = false;
						do_next_servo_delay_timer = false;
						servo_id = 0;
						in_servo_sequence_delay = 0;

						//MOTOR_SEQUENCE_DELAY_TIMER = false;
						//IN_MOTOR_SEQUENCE = false;

						if(_param_auto_pfl_mode.get()==2){
							PX4_INFO("EXIT ACTUATOR SERVO TEST");
							//ACTUATOR_SHOULD_EXIT = true;
						}
					}
					//motor_test = false;
					//PX4_ERR("motor_test = false");
				}

			}

			limitAndUpdateOutputs(outputs, has_updates);
			// PX4_ERR("outputs[MAX_ACTUATORS] = %5.2f[0],_functions = %s, _function_assignment[0] = %.2f",(double)outputs[0],_functions[0] ? "true" : "false",(double)_function_assignment[0]);
			// PX4_WARN("outputs[MAX_ACTUATORS] = %5.2f[1],_functions = %s, _function_assignment[1] = %.2f",(double)outputs[1],_functions[1] ? "true" : "false",(double)_function_assignment[1]);
			// PX4_WARN("outputs[MAX_ACTUATORS] = %5.2f[2],_functions = %s, _function_assignment[2] = %.2f",(double)outputs[2],_functions[2] ? "true" : "false",(double)_function_assignment[2]);
			// PX4_WARN("outputs[MAX_ACTUATORS] = %5.2f[3],_functions = %s, _function_assignment[3] = %.2f",(double)outputs[3],_functions[3] ? "true" : "false",(double)_function_assignment[3]);
			// PX4_WARN("outputs[MAX_ACTUATORS] = %5.2f[4],_functions = %s, _function_assignment[4] = %.2f",(double)outputs[4],_functions[4] ? "true" : "false",(double)_function_assignment[4]);
			// PX4_WARN("outputs[MAX_ACTUATORS] = %5.2f[5],_functions = %s, _function_assignment[5] = %.2f",(double)outputs[5],_functions[5] ? "true" : "false",(double)_function_assignment[5]);
		}

	}//_param_auto_pfl_en Enable



	return true;
}

void
MixingOutput::limitAndUpdateOutputs(float outputs[MAX_ACTUATORS], bool has_updates)
{
	bool stop_motors = !_throttle_armed && !_actuator_test.inTestMode();

	if (_armed.lockdown || _armed.manual_lockdown) {
		// overwrite outputs in case of lockdown with disarmed values
		for (size_t i = 0; i < _max_num_outputs; i++) {
			_current_output_value[i] = _disarmed_value[i];
		}

		stop_motors = true;

	} else if (_armed.force_failsafe) {
		// overwrite outputs in case of force_failsafe with _failsafe_value values
		for (size_t i = 0; i < _max_num_outputs; i++) {
			_current_output_value[i] = actualFailsafeValue(i);
		}

	} else {
		// the output limit call takes care of out of band errors, NaN and constrains
		output_limit_calc(_throttle_armed || _actuator_test.inTestMode(), _max_num_outputs, outputs);
	}

	// We must calibrate the PWM and Oneshot ESCs to a consistent range of 1000-2000us (gets mapped to 125-250us for Oneshot)
	// Doing so makes calibrations consistent among different configurations and hence PWM minimum and maximum have a consistent effect
	// hence the defaults for these parameters also make most setups work out of the box
	if (_armed.in_esc_calibration_mode) {
		static constexpr uint16_t PWM_CALIBRATION_LOW = 1000;
		static constexpr uint16_t PWM_CALIBRATION_HIGH = 2000;

		for (int i = 0; i < _max_num_outputs; i++) {
			if (_current_output_value[i] == _min_value[i]) {
				_current_output_value[i] = PWM_CALIBRATION_LOW;
			}

			if (_current_output_value[i] == _max_value[i]) {
				_current_output_value[i] = PWM_CALIBRATION_HIGH;
			}
		}
	}

	/* now return the outputs to the driver */
	if (_interface.updateOutputs(stop_motors, _current_output_value, _max_num_outputs, has_updates)) {
		actuator_outputs_s actuator_outputs{};
		setAndPublishActuatorOutputs(_max_num_outputs, actuator_outputs);

		updateLatencyPerfCounter(actuator_outputs);
	}
}

uint16_t MixingOutput::output_limit_calc_single(int i, float value) const
{
	// check for invalid / disabled channels
	if (!PX4_ISFINITE(value)) {
		return _disarmed_value[i];
	}

	if (_reverse_output_mask & (1 << i)) {
		value = -1.f * value;
	}

	uint16_t effective_output = value * (_max_value[i] - _min_value[i]) / 2 + (_max_value[i] + _min_value[i]) / 2;

	// last line of defense against invalid inputs
	return math::constrain(effective_output, _min_value[i], _max_value[i]);
}

void
MixingOutput::output_limit_calc(const bool armed, const int num_channels, const float output[MAX_ACTUATORS])
{
	const bool pre_armed = armNoThrottle();

	// time to slowly ramp up the ESCs
	static constexpr hrt_abstime RAMP_TIME_US = 500_ms;

	/* first evaluate state changes */
	switch (_output_state) {
	case OutputLimitState::INIT:
		if (armed) {
			// set arming time for the first call
			if (_output_time_armed == 0) {
				_output_time_armed = hrt_absolute_time();
			}

			// time for the ESCs to initialize (this is not actually needed if the signal is sent right after boot)
			if (hrt_elapsed_time(&_output_time_armed) >= 50_ms) {
				_output_state = OutputLimitState::OFF;
			}
		}

		break;

	case OutputLimitState::OFF:
		if (armed) {
			if (_output_ramp_up) {
				_output_state = OutputLimitState::RAMP;

			} else {
				_output_state = OutputLimitState::ON;
			}

			// reset arming time, used for ramp timing
			_output_time_armed = hrt_absolute_time();
		}

		break;

	case OutputLimitState::RAMP:
		if (!armed) {
			_output_state = OutputLimitState::OFF;

		} else if (hrt_elapsed_time(&_output_time_armed) >= RAMP_TIME_US) {
			_output_state = OutputLimitState::ON;
		}

		break;

	case OutputLimitState::ON:
		if (!armed) {
			_output_state = OutputLimitState::OFF;
		}

		break;
	}

	/* if the system is pre-armed, the limit state is temporarily on,
	 * as some outputs are valid and the non-valid outputs have been
	 * set to NaN. This is not stored in the state machine though,
	 * as the throttle channels need to go through the ramp at
	 * regular arming time.
	 */
	auto local_limit_state = _output_state;

	if (pre_armed) {
		local_limit_state = OutputLimitState::ON;
	}

	// then set _current_output_value based on state
	switch (local_limit_state) {
	case OutputLimitState::OFF:
	case OutputLimitState::INIT:
		for (int i = 0; i < num_channels; i++) {
			_current_output_value[i] = _disarmed_value[i];
		}

		break;

	case OutputLimitState::RAMP: {
			hrt_abstime diff = hrt_elapsed_time(&_output_time_armed);

			static constexpr int PROGRESS_INT_SCALING = 10000;
			int progress = diff * PROGRESS_INT_SCALING / RAMP_TIME_US;

			if (progress > PROGRESS_INT_SCALING) {
				progress = PROGRESS_INT_SCALING;
			}

			for (int i = 0; i < num_channels; i++) {

				float control_value = output[i];

				/* check for invalid / disabled channels */
				if (!PX4_ISFINITE(control_value)) {
					_current_output_value[i] = _disarmed_value[i];
					continue;
				}

				uint16_t ramp_min_output;

				/* if a disarmed output value was set, blend between disarmed and min */
				if (_disarmed_value[i] > 0) {

					/* safeguard against overflows */
					auto disarmed = _disarmed_value[i];

					if (disarmed > _min_value[i]) {
						disarmed = _min_value[i];
					}

					int disarmed_min_diff = _min_value[i] - disarmed;
					ramp_min_output = disarmed + (disarmed_min_diff * progress) / PROGRESS_INT_SCALING;

				} else {
					/* no disarmed output value set, choose min output */
					ramp_min_output = _min_value[i];
				}

				if (_reverse_output_mask & (1 << i)) {
					control_value = -1.f * control_value;
				}

				_current_output_value[i] = control_value * (_max_value[i] - ramp_min_output) / 2 + (_max_value[i] + ramp_min_output) /
							   2;

				/* last line of defense against invalid inputs */
				_current_output_value[i] = math::constrain(_current_output_value[i], ramp_min_output, _max_value[i]);
			}
		}
		break;

	case OutputLimitState::ON:
		for (int i = 0; i < num_channels; i++) {
			_current_output_value[i] = output_limit_calc_single(i, output[i]);
		}

		break;
	}
}

void
MixingOutput::setAndPublishActuatorOutputs(unsigned num_outputs, actuator_outputs_s &actuator_outputs)
{
	actuator_outputs.noutputs = num_outputs;

	for (size_t i = 0; i < num_outputs; ++i) {
		actuator_outputs.output[i] = _current_output_value[i];
	}

	actuator_outputs.timestamp = hrt_absolute_time();
	_outputs_pub.publish(actuator_outputs);
}

void
MixingOutput::updateLatencyPerfCounter(const actuator_outputs_s &actuator_outputs)
{
	// Just check the first function. It means we only get the latency if motors are assigned first, which is the default
	if (_function_allocated[0]) {
		hrt_abstime timestamp_sample;

		if (_function_allocated[0]->getLatestSampleTimestamp(timestamp_sample)) {
			perf_set_elapsed(_control_latency_perf, actuator_outputs.timestamp - timestamp_sample);
		}
	}
}

uint16_t
MixingOutput::actualFailsafeValue(int index) const
{
	uint16_t value = 0;

	if (_failsafe_value[index] == UINT16_MAX) { // if set to default, use the one provided by the function
		float default_failsafe = NAN;

		if (_functions[index]) {
			default_failsafe = _functions[index]->defaultFailsafeValue(_function_assignment[index]);
		}

		value = output_limit_calc_single(index, default_failsafe);

	} else {
		value = _failsafe_value[index];
	}

	return value;
}
