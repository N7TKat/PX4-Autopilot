#include "ActuatorValueMix.hpp"

#include "functions/FunctionMotors.hpp"
#include "functions/FunctionServosSineWave.hpp"

using namespace time_literals;

ActuatorValueMix::ActuatorValueMix(const OutputFunction function_assignments[MAX_ACTUATORS])
	: _function_assignments(function_assignments)
{
	reset();
}


void ActuatorValueMix::update(int num_outputs,
				 float thrust_curve,
 				 int pfl_mode, 
				 int servo_loop, 
				 int motor_run_time, 
				 bool pfl_en,
				 int in_sequence_num,
				 int servo_max)
				// float update_function_assignment[MAX_ACTUATORS])
{
	//			
	// generate value motor
	// handle servo
	if ((int)OutputFunction::Servo1 <= 201 && 201 <= (int)OutputFunction::ServoMax) {
		//actuator_motors_s motors;
		servo_value = 1.0;
		//motors.reversible_flags = 0;
		//int motor_idx = 1;
		FunctionServoSineWave::updateValues(&servo_value, in_sequence_num, servo_loop, servo_max);
		_current_servo_value = servo_value;
	}


	// handle motors
	if ((int)OutputFunction::Motor1 <= 101 && 101 <= (int)OutputFunction::MotorMax) {
		actuator_motors_s motors;
		motor_value = 0.15;
		motors.reversible_flags = 0;
		int motor_idx = 1;
		FunctionMotors::updateValues(motors.reversible_flags >> motor_idx, thrust_curve, &motor_value, 1);
		_current_motor_value = motor_value;
	}
}

void ActuatorValueMix::overrideValues(
				 float value[MAX_ACTUATORS],
				 int num_outputs,
				 int pfl_mode,
				 int override_index,
				 bool servo)//, bool pfl_en
{
	if (servo) {
		_output_value	= _current_servo_value;//0.15f +((pfl_mode/3.0f)*0.0f + mix_value*0.0f); //_current_servo_value
	}
	else{
		_output_value 	= _current_motor_value;
	}

	for (int i = 0; i < num_outputs; ++i) {
		if(i == override_index){
			value[i] = _output_value;
		}
		else{
			value[i] = NAN;
		}
	}
	
	// 	for (int i = 0; i < num_outputs; ++i) {

	// 		if(i == override_index){
	// 			value[i] = (pfl_mode/3.0f)*0.0f + new_value*0.0f + 0.15f;
	// 		}
	// 		else{
	// 			value[i] = NAN;
	// 		}
	// 	}
	// }
}

void ActuatorValueMix::reset()
{
	// _in_test_mode = false;
	// _next_timeout = 0;

	// for (int i = 0; i < MAX_ACTUATORS; ++i) {
	// 	mix_value = NAN;

	// }
}


// const hrt_abstime now = hrt_absolute_time();
// 	actuator_test_s actuator_test;

// 	// for (int i = 0; i < num_outputs; ++i) {
// 	// 		new_value = pfl_mode + i;	
// 	// }

// 	for (int i = 0; i < num_outputs; ++i) {
// 		if ((int)_function_assignments[i] == actuator_test.function) {

// 			_output_overridden[i] = in_test_mode;

// 			if (in_test_mode) {
// 				if (actuator_test.timeout_ms > 0) {
// 					_next_timeout = now + actuator_test.timeout_ms * 1000;
// 				}

// 				float mix_value = 0.0f;

// 				// handle motors
// 				if ((int)OutputFunction::Motor1 <= actuator_test.function && actuator_test.function <= (int)OutputFunction::MotorMax) {
// 					actuator_motors_s motors;
// 					mix_value = 0.15f;
// 					motors.reversible_flags = 0;
// 					_actuator_motors_sub.copy(&motors);
// 					int motor_idx = actuator_test.function - (int)OutputFunction::Motor1;
// 					FunctionMotors::updateValues(motors.reversible_flags >> motor_idx, thrust_curve, &mix_value, 1);
// 				}

// 				// handle servos: add trim
// 				if ((int)OutputFunction::Servo1 <= actuator_test.function && actuator_test.function <= (int)OutputFunction::ServoMax) {
// 					actuator_servos_trim_s trim{};
// 					mix_value = 0.85f;
// 					_actuator_servos_trim_sub.copy(&trim);
// 					int idx = actuator_test.function - (int)OutputFunction::Servo1;

// 					if (idx < actuator_servos_trim_s::NUM_CONTROLS) {
// 						mix_value += trim.trim[idx];
// 					}
// 				}

// 				_current_outputs[i] = mix_value;

// 			} else {
// 				_current_outputs[i] = NAN;
// 			}
// 		}
// 	}

// 	if (_in_test_mode) {
// 		// check if all disabled
// 		bool any_overridden = false;

// 		for (int i = 0; i < num_outputs; ++i) {
// 			any_overridden |= _output_overridden[i];
// 		}

// 		if (!any_overridden) {
// 			_in_test_mode = false;
// 		}
// 	}

// 	// check for timeout
// 	if (_in_test_mode && _next_timeout != 0 && now > _next_timeout) {
// 		_in_test_mode = false;
// 	}
	
