/****************************************************************************
 *
 *   Copyright (C) 2021 PX4 Development Team. All rights reserved.
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

/**
 * @file auto_preflight_check.cpp
 *
 * CLI to publish the actuator_test msg
 */
#include "auto_preflight_check.hpp"

AutoPreflightCheck::AutoPreflightCheck() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1) //test1 lp_default
{
}

AutoPreflightCheck::~AutoPreflightCheck()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool AutoPreflightCheck::init()
{
	// // execute Run() on every sensor_accel publication
	if (!_sensor_accel_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	// // alternatively, Run on fixed interval
	// // ScheduleOnInterval(5000_us); // 2000 us interval, 200 Hz rate

	return true;
}

//Parameters Updater function
void AutoPreflightCheck::Parameters_updated()
{
	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams(); // update module parameters (in DEFINE_PARAMETERS)
	}
}

//Actuator_test_default_logic
void AutoPreflightCheck::actuator_test(int function, float value, int timeout_ms, bool release_control)
{
	actuator_test_s actuator_test{};
	actuator_test.timestamp = hrt_absolute_time();
	actuator_test.function = function;
	actuator_test.value = value;
	actuator_test.action = release_control ? actuator_test_s::ACTION_RELEASE_CONTROL : actuator_test_s::ACTION_DO_CONTROL;
	actuator_test.timeout_ms = timeout_ms;

	uORB::Publication<actuator_test_s> actuator_test_pub{ORB_ID(actuator_test)};
	actuator_test_pub.publish(actuator_test);
}


void AutoPreflightCheck::Run()
{
	if(ACTUATOR_SHOULD_EXIT) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	
	motor_run_time_ms = _param_auto_pfl_mt_time.get()*1000; //default 1.5s AUTO_PFL_MT_TIME
	servo_sequence_max = _param_auto_pfl_sv_loop.get();

	if(param_check){
		if(_param_auto_pfl_mt_time.get()>5.0f||_param_auto_pfl_mt_time.get()<0.5f)
		{
			ACTUATOR_RUN = false;
			PX4_INFO("auto preflight check Fail");
			PX4_INFO("param AUTO_PFL_MT_TIME is out of range 0.5-5.0 sec");
			PX4_INFO("auto_preflight_check exit");
			ACTUATOR_SHOULD_EXIT = true;
		}
		if(_param_auto_pfl_mode.get()>3||_param_auto_pfl_mode.get()<0)
		{
			ACTUATOR_RUN = false;
			PX4_INFO("auto preflight mode Fail");
			PX4_INFO("param AUTO_PFL_MODE is out of range 0-3");
			PX4_INFO("auto_preflight_check exit");
			ACTUATOR_SHOULD_EXIT = true;
		}
		if(_param_auto_pfl_sv_loop.get()>5||_param_auto_pfl_sv_loop.get()<2)
		{
			ACTUATOR_RUN = false;
			PX4_INFO("auto preflight check Fail");
			PX4_INFO("param AUTO_PFL_SV_LOOP is out of range 2-5");
			PX4_INFO("auto_preflight_check exit");
			ACTUATOR_SHOULD_EXIT = true;
		}
		if(_param_auto_pfl_mt_time.get()<=5.0f||_param_auto_pfl_mt_time.get()>=0.5f||_param_auto_pfl_sv_loop.get()<=5||_param_auto_pfl_sv_loop.get()>=2||_param_auto_pfl_mode.get()<=3||_param_auto_pfl_mode.get()>=0){
			ACTUATOR_RUN = true;
			PX4_INFO("auto preflight check pass");
		}

		param_check = false;
	}			
	
	automatic_hardware_testing_s automatic_hardware_testing;
	
	// PX4_ERR(" is_motor_function is : %s", is_motor_function ? "true" : "false");
	// PX4_ERR(" is_servo_function is : %s", is_servo_function ? "true" : "false");
	// PX4_ERR("ACTUATOR_RUN loop = %i: ********************************************",loop_num_2);

	if(_automatic_hardware_testing_sub.update(&automatic_hardware_testing))
	{	

		update_timeout = hrt_absolute_time();
		timeout_timer = true;

		if ((int)automatic_hardware_testing.test_mode == 0){
			/*---reset motor_test---*/
			motor_test 		= false;
			DO_NEXT_MOTOR 	= true;
			done_all_motor 	= false;

			/*---reset servo_test---*/
			servo_test = false;
			DO_NEXT_SERVO = true;

			PX4_INFO("4.1: I NA SOON");
		}

		else if (((int)automatic_hardware_testing.test_mode == automatic_hardware_testing_s::TEST_MODE_MOTOR_ONLY))
		{
			//PX4_INFO("MIXER : AUTO_PFL_MODE 1");
			motor_test = true;
			//PX4_WARN(" ACTUATOR_RUN in case 1 is : %s", ACTUATOR_RUN ? "true" : "false");
			//PX4_ERR("Motor loop = %i: ***************",loop_num);
			//++loop_num;
			// PX4_INFO("4.2 : I NA NUEANG");
		}


		else if (((int)automatic_hardware_testing.test_mode == automatic_hardware_testing_s::TEST_MODE_SERVO_ONLY))
		{
			PX4_INFO("AUTO_PFL_MODE 2");
			servo_test = true;

			// PX4_ERR("Servo loop = %i: ***************",loop_num);
			// ++loop_num_2;
		}

		else if (((int)automatic_hardware_testing.test_mode == automatic_hardware_testing_s::TEST_MODE_MOTOR_THEN_SERVO))
		{
			PX4_INFO("AUTO_PFL_MODE 3");
			//motor_test = true;
			//servo_test = true;
		}

		else if (((int)automatic_hardware_testing.test_mode == automatic_hardware_testing_s::TEST_MODE_STOP))
		{
			PX4_INFO("AUTO_PFL_MODE 4");
			
			/*Reset to Default*/
			done_all_motor = false;

			motor_test = false;
			DO_NEXT_MOTOR = true;
			DO_MOTOR_SEQUENCE = false;
			IN_MOTOR_SEQUENCE = false;
			DO_NEXT_MOTOR_DELAY_TIMER = false;
			MOTOR_SEQUENCE_DELAY_TIMER = false;
			MOTOR_SEQUENCE_DELAY = 0;
			DO_NEXT_MOTOR_DELAY = 0;

			MOTOR_ID = 0;
			MOTOR_SEQUENCE = 1;
			//PX4_INFO("4.4: I NA SEE");
		}

		else{
			//reset motor_test
			// motor_test = false;
			// DO_NEXT_MOTOR = true;
			// //reset servo_test
			// servo_test = false;
			// DO_NEXT_SERVO = true;
			PX4_INFO("4.5: I NA Default");
		}

	}
	if(!_automatic_hardware_testing_sub.update(&automatic_hardware_testing) && (timeout_timer) && (hrt_elapsed_time(& update_timeout) > 12_ms)){
		/*auto reset*/
		//PX4_WARN("AUTO RESET TRIGGERED : NO Update");

		// //reset motor_test
		// motor_test = false;
		// do_next_motor = true;
		//done_all_motor = true;
		// //reset servo_test
		// servo_test = false;
		// do_next_servo = true;
		// timeout_timer = false;

	}
	
	if(motor_test){
		//PX4_ERR(" DO_NEXT_MOTOR is : %s", DO_NEXT_MOTOR ? "true" : "false");
		if(DO_NEXT_MOTOR){
			DO_MOTOR_SEQUENCE = true;
			IN_MOTOR_SEQUENCE = true;
			DO_NEXT_MOTOR = false;
			//PX4_INFO("DO Test MOTOR [%i]",j);
			PX4_INFO("DO Test MOTOR [%i]",MOTOR_ID+1);
			//++j;
		}

		if((DO_MOTOR_SEQUENCE)&&(IN_MOTOR_SEQUENCE)){
			DO_MOTOR_SEQUENCE = false;
			value = 0.15f;
			PX4_WARN("DO_MOTOR_SEQUENCE [%i] , MOTOR_SEQUENCE[%i/%i]",j,MOTOR_SEQUENCE,MOTOR_SEQUENCE_MAX);
			++j;

			if(MOTOR_SEQUENCE>MOTOR_SEQUENCE_MAX){
				IN_MOTOR_SEQUENCE = false;
				MOTOR_SEQUENCE = 1;
				
				if((MOTOR_ID)<MOTOR_MAX){
					PX4_INFO("Wait Next MOTOR 5 sec.");
				}
				DO_NEXT_MOTOR_DELAY = hrt_absolute_time();
				DO_NEXT_MOTOR_DELAY_TIMER = true;
				//PX4_ERR(" DO_NEXT_MOTOR_DELAY_TIMER is : %s ", DO_NEXT_MOTOR_DELAY_TIMER ? "true" : "false");
			}
			else{
				actuator_test(actuator_test_s::FUNCTION_MOTOR1+motor_oder[MOTOR_ID], value, motor_run_time_ms, false);
				PX4_ERR("Test MOTOR [%i] at SEQUENCE[%i]",MOTOR_ID+1,MOTOR_SEQUENCE);

				PX4_INFO("Wait MOTOR sequence %d sec.",motor_run_time_ms/1000);

				++MOTOR_SEQUENCE;

				MOTOR_SEQUENCE_DELAY = hrt_absolute_time();
				MOTOR_SEQUENCE_DELAY_TIMER = true;
			}
		}
		
		// PX4_ERR(" MSDT[%s], lapstime[%s], Insq[%s] ",
		// 			MOTOR_SEQUENCE_DELAY_TIMER ? "true" : "false",
		// 			(hrt_elapsed_time(& MOTOR_SEQUENCE_DELAY) > ((double)motor_run_time_ms*1000)) ? "true" : "false",
		// 			IN_MOTOR_SEQUENCE ? "true" : "false");
		if((MOTOR_SEQUENCE_DELAY_TIMER)&&(hrt_elapsed_time(& MOTOR_SEQUENCE_DELAY) >= ((double)motor_run_time_ms*1000))&&(IN_MOTOR_SEQUENCE)){
			MOTOR_SEQUENCE_DELAY_TIMER = false;
			DO_MOTOR_SEQUENCE = true;
			PX4_INFO("^^^^^^^^Wait MOTOR Runtime 3 sec.");
		}

		//PX4_INFO("DO_NEXT_MOTOR_DELAY : %lu",(hrt_absolute_time()-DO_NEXT_MOTOR_DELAY)/1000000);
		if((DO_NEXT_MOTOR_DELAY_TIMER)&&(hrt_elapsed_time(& DO_NEXT_MOTOR_DELAY) > 5_s)){
			DO_NEXT_MOTOR_DELAY_TIMER = false;
			// DO_NEXT_MOTOR = true;
			// //++MOTOR_ID;
			// if(MOTOR_ID<MOTOR_MAX){
			// 	PX4_INFO("Test Next MOTOR [%i] ",MOTOR_ID+1);
			// }
			if(MOTOR_ID < (MOTOR_MAX - 1)){
				DO_NEXT_MOTOR = true;
				++MOTOR_ID;

				/*Status*/
				PX4_WARN("****Test Next MOTOR [%i] ", MOTOR_ID + 1);

			}
			else{ //(motor_id>=motor_max)
				DO_NEXT_MOTOR = false;
				done_all_motor = true;

				/*Status*/
				PX4_WARN("Finish last motor");
			}

				
		}

		if(done_all_motor){
			/*kill logic*/
			//done_all_motor = false;

			motor_test_is_done = true;


			/*Reset to Default*/
			motor_test = false;
			DO_NEXT_MOTOR = false;
			DO_MOTOR_SEQUENCE = false;
			IN_MOTOR_SEQUENCE = false;
			DO_NEXT_MOTOR_DELAY_TIMER = false;
			MOTOR_SEQUENCE_DELAY_TIMER = false;
			MOTOR_SEQUENCE_DELAY = 0;
			DO_NEXT_MOTOR_DELAY = 0;

			MOTOR_ID = 0;
			MOTOR_SEQUENCE = 1;

			/*Reset to Default -lower*/
			//motor_test = false;
			//do_next_motor = true;
			//do_motor_sequence = false;
			//do_next_motor_delay_timer = false;
			//motor_sequence_delay = 0;
			//in_motor_sequence_delay = 0;
			//motor_id = 0;
		}
	}

	if((servo_test)&&(motor_test_is_done)&&(!ACTUATOR_SHOULD_EXIT)){
		if(DO_NEXT_SERVO){
			//DO_SERVO_SEQUENCE = true;
			//IN_SERVO_SEQUENCE = true;
			DO_NEXT_SERVO = false;
			DO_SERVO_SINE_WAVE = true;
			//IN_SINE_WAVE = true;

			PX4_INFO("DO Test Servo [%i]",SERVO_ID+1);
		}

		if((DO_SERVO_SINE_WAVE)){
			DO_SERVO_SINE_WAVE = false;
			if(one_sine_wave_step<=one_sine_wave_step_max){

				PX4_INFO("Test Servo [%i] SEQUENCE[%i] STEP[%i] AMP[%f]",SERVO_ID+1,SERVO_SEQUENCE,one_sine_wave_step,(double)one_sine_wave[one_sine_wave_step]);
				actuator_test(actuator_test_s::FUNCTION_SERVO1+SERVO_ID, one_sine_wave[one_sine_wave_step], one_sine_wave_step_time, false);
				++one_sine_wave_step;
				SERVO_SINE_WAVE_STEP_TIMEOUTS = hrt_absolute_time();
				SERVO_SINE_WAVE_STEP_TIMER = true;

			}
			else{
				one_sine_wave_step = 1;
				DO_SERVO_SEQUENCE = true;
				IN_SERVO_SEQUENCE = true;
			}
			
		}

		if((SERVO_SINE_WAVE_STEP_TIMER)&&(hrt_elapsed_time(& SERVO_SINE_WAVE_STEP_TIMEOUTS) > 250_ms)){
			SERVO_SINE_WAVE_STEP_TIMER	= false;
			DO_SERVO_SINE_WAVE = true;
		}



		if((DO_SERVO_SEQUENCE)&&(IN_SERVO_SEQUENCE)){
			DO_SERVO_SEQUENCE = false;

			//PX4_INFO("Test Servo [%i] at SEQUENCE[%i]",SERVO_ID+1,SERVO_SEQUENCE);
			//++SERVO_SEQUENCE;

			if(SERVO_SEQUENCE>=servo_sequence_max){
				IN_SERVO_SEQUENCE = false;
				SERVO_SEQUENCE = 1;
				
				if((SERVO_ID+2)<SERVO_MAX){
					PX4_INFO("Wait Next SERVO 5 sec.");
				}
				
				DO_NEXT_SERVO_DELAY = hrt_absolute_time();
				DO_NEXT_SERVO_DELAY_TIMER = true;
			}
			else{
				++SERVO_SEQUENCE;
				PX4_INFO("Wait Next sequence 2 sec.");
				SERVO_SEQUENCE_DELAY = hrt_absolute_time();
				SERVO_SEQUENCE_DELAY_TIMER = true;
			}
		}

		if((SERVO_SEQUENCE_DELAY_TIMER)&&(hrt_elapsed_time(& SERVO_SEQUENCE_DELAY) > 2_s)&&(IN_SERVO_SEQUENCE)){
			SERVO_SEQUENCE_DELAY_TIMER	= false;
			//DO_SERVO_SEQUENCE = true;
			DO_SERVO_SINE_WAVE = true;
			
		}

		if((DO_NEXT_SERVO_DELAY_TIMER)&&(hrt_elapsed_time(& DO_NEXT_SERVO_DELAY) > 5_s)){
			
			DO_NEXT_SERVO_DELAY_TIMER = false;
			DO_NEXT_SERVO = true;
			++SERVO_ID;
			if(SERVO_ID<SERVO_MAX){
				PX4_INFO("Test Next Servo [%i] ",SERVO_ID+1);
			}
				
		}

		if(SERVO_ID>=SERVO_MAX){
			servo_test = false;

			PX4_INFO("****************************************");
			PX4_INFO("Finish");

			//Reset
			DO_NEXT_SERVO = true;
            DO_SERVO_SEQUENCE = false;
            IN_SERVO_SEQUENCE = false;
            DO_NEXT_SERVO_DELAY_TIMER = false;
            SERVO_SEQUENCE_DELAY_TIMER = false;

			//Reset
			ACTUATOR_RUN = false;
			if((_param_auto_pfl_mode.get()==3)){
				PX4_INFO("EXIT ACTUATOR SERVO TEST");
				ACTUATOR_SHOULD_EXIT = true;
			}
			if((_param_auto_pfl_mode.get()==3)){
				PX4_INFO("EXIT ACTUATOR MOTOR SERVO TEST");
				ACTUATOR_SHOULD_EXIT = true;
			}
			
		}
	}

	perf_end(_loop_perf);
}

int AutoPreflightCheck::task_spawn(int argc, char *argv[])
{
	AutoPreflightCheck *instance = new AutoPreflightCheck();

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

int AutoPreflightCheck ::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int AutoPreflightCheck ::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int AutoPreflightCheck::print_usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
Utility to test actuators.

WARNING: remove all props before using this command.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("Actuator_test", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int auto_preflight_check_main(int argc, char *argv[])
{
	return AutoPreflightCheck::main(argc, argv);
}
