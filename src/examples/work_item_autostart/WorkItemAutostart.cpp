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

#include "WorkItemAutostart.hpp"

WorkItemAutostart::WorkItemAutostart() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
}

WorkItemAutostart::~WorkItemAutostart()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool WorkItemAutostart::init()
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

//Parameters Updater function
void WorkItemAutostart::Parameters_updated()
{
	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams(); // update module parameters (in DEFINE_PARAMETERS)
	}
}

void WorkItemAutostart::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	Parameters_updated();

	// declare servo info Struct
	struct servo_info_s servo_info;

	// access WORK_AUTO_EN value
	if (_param_work_auto_en.get() == 1) {

		servo_info.timestamp = hrt_absolute_time();
		servo_info.connectiontype = 2;
		//access WORK_AUTO_MU value
		servo_info.temperature[0] = _param_work_auto_mu.get() ;
		servo_info.errorcount[0] = _param_work_auto_mu.get() ;

		if (_time_reset){ //true and start if
			_previous_time = hrt_absolute_time();
			_time_reset = false;
		}

                if (hrt_elapsed_time(& _previous_time)> 1_s){  //set every n second
                        _time_reset = true;
			switch (_param_work_auto_mode.get())
			{
				case 0:
					break;
				case 1:
					PX4_INFO("Hello : Printing in Mode 1 : %.2f",(double)_param_work_auto_mu.get());
					break;
				case 2:
					PX4_INFO("Hello : Printing in Mode 2 : %.2f",(double)_param_work_auto_mu.get() * 10);
					break;
				case 3:
					PX4_INFO("Hello : Printing in Mode 3 : %.2f",(double)_param_work_auto_mu.get() * 100);
					break;
				default:
					break;
			}

		}

		_servo_info_pub.publish(servo_info);
	}

	perf_end(_loop_perf);
}

int WorkItemAutostart::task_spawn(int argc, char *argv[])
{
	WorkItemAutostart *instance = new WorkItemAutostart();

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

int WorkItemAutostart::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int WorkItemAutostart::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int WorkItemAutostart::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Example of a simple module running out of a work queue.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("work_item_example", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int work_item_autostart_main(int argc, char *argv[])
{
	return WorkItemAutostart::main(argc, argv);
}
