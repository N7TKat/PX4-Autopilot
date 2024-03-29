/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "work_item_tutorialT.hpp"

work_item_tutorialT::work_item_tutorialT() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
}

work_item_tutorialT::~work_item_tutorialT()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool work_item_tutorialT::init()
{
	// execute Run() on every sensor_accel publication
	if (!_sensor_accel_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	// alternatively, Run on fixed interval
	// ScheduleOnInterval(5000_us); // 2000 us interval, 200 Hz rate

	return true;
}

void work_item_tutorialT::parameters_updated() {
	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams(); // update module parameters (in DEFINE_PARAMETERS)
	}
}

void work_item_tutorialT::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	// Example
	//  update vehicle_status to check arming state
	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.copy(&vehicle_status)) {

			const bool armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);

			if (armed && !_armed) {
				PX4_WARN("vehicle armed due to %d", vehicle_status.latest_arming_reason);
				switch (vehicle_status.latest_disarming_reason)
				{
				case 1 :
					PX4_INFO("vehicle armed due to RC_STICK");
					break;
				case 2 :
					PX4_INFO("vehicle armed due to RC_SWITCH");
					break;
				case 3 :
					PX4_INFO("vehicle armed due to COMMAND INTERNAL");
					break;
				case 4 :
					PX4_INFO("vehicle armed due to COMMAND EXTERNAL");
					break;
				case 5 :
					PX4_INFO("vehicle armed due to MISSION START");
					break;
				case 6 :
					PX4_INFO("vehicle armed due to SAFETY BUTTON");
					break;
				case 7 :
					PX4_INFO("vehicle armed due to DISRAM LAND ");
					break;
				case 8 :
					PX4_INFO("vehicle armed due to DISARM PREFLIGHT");
					break;
				case 9 :
					PX4_INFO("vehicle armed due to KILL SWITCH");
					break;
				case 10 :
					PX4_INFO("vehicle armed due to LOCKDOWN");
					break;
				case 11 :
					PX4_INFO("vehicle armed due to FAILURE DETECTOR");
					break;
				case 12 :
					PX4_INFO("vehicle armed due to SHUTDOWN");
					break;
				case 13 :
					PX4_INFO("vehicle armed due to UNIT TEST");
					break;
				default:
					PX4_INFO("vehicle armed due to TRANSITION TO STANDBY");
					break;
				}
			} else if (!armed && _armed) {
				PX4_INFO("vehicle disarmed due to %d", vehicle_status.latest_disarming_reason);
				switch (vehicle_status.latest_disarming_reason)
				{
				case 1 :
					PX4_INFO("vehicle disarmed due to RC_STICK");
					break;
				case 2 :
					PX4_INFO("vehicle disarmed due to RC_SWITCH");
					break;
				case 3 :
					PX4_INFO("vehicle disarmed due to COMMAND INTERNAL");
					break;
				case 4 :
					PX4_INFO("vehicle disarmed due to COMMAND EXTERNAL");
					break;
				case 5 :
					PX4_INFO("vehicle disarmed due to MISSION START");
					break;
				case 6 :
					PX4_INFO("vehicle disarmed due to SAFETY BUTTON");
					break;
				case 7 :
					PX4_INFO("vehicle disarmed due to DISRAM LAND ");
					break;
				case 8 :
					PX4_INFO("vehicle disarmed due to DISARM PREFLIGHT");
					break;
				case 9 :
					PX4_INFO("vehicle disarmed due to KILL SWITCH");
					break;
				case 10 :
					PX4_INFO("vehicle disarmed due to LOCKDOWN");
					break;
				case 11 :
					PX4_INFO("vehicle disarmed due to FAILURE DETECTOR");
					break;
				case 12 :
					PX4_INFO("vehicle disarmed due to SHUTDOWN");
					break;
				case 13 :
					PX4_INFO("vehicle disarmed due to UNIT TEST");
					break;
				default:
					PX4_INFO("vehicle disarmed due to TRANSITION TO STANDBY");
					break;
				}
			}

			_armed = armed;
		}
	}

	//Update Parameter Data
	parameters_updated();
	// Example
	//  grab latest accelerometer data

	if (_sensor_accel_sub.updated()) {
		sensor_accel_s accel;

		if (_sensor_accel_sub.copy(&accel)) {
			// DO WORK

			// access parameter value (SYS_AUTOSTART)
			if (_param_sys_autostart.get() == 1234) {
				int test_param = _param_sys_autostart.get();
				// do something if SYS_AUTOSTART is 1234
				/*if (_sys_printing) {
					PX4_INFO("Hello %d", _param_sys_autostart.get());
					_sys_printing = false;
					}*/
					if (_time_reset){
						_previous_time = hrt_absolute_time();
						_time_reset = false;
					}
					if (hrt_elapsed_time(&_previous_time) > 1_s){
						_time_reset = true;
						PX4_INFO("SYS_AUTOSTART SET to %d", test_param);
					}
				}
			}

		}





	// Example
	//  publish some data
	/*
	orb_test_s data{};
	data.val = 314159;
	data.timestamp = hrt_absolute_time();
	_orb_test_pub.publish(data);
	*/


	perf_end(_loop_perf);
}

int work_item_tutorialT::task_spawn(int argc, char *argv[])
{
	work_item_tutorialT *instance = new work_item_tutorialT();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int work_item_tutorialT::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int work_item_tutorialT::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int work_item_tutorialT::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Example of a simple module running out of a work queue.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("work_item_tutorialT", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int work_item_tutorialT_main(int argc, char *argv[])
{
	return work_item_tutorialT::main(argc, argv);
}
