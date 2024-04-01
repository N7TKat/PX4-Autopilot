#pragma once

#include <mixer_module/output_functions.hpp>

#include <drivers/drv_pwm_output.h>
#include <uORB/topics/actuator_test.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_servos_trim.h>
#include <uORB/Subscription.hpp>

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/px4_config.h>

#include <lib/perf/perf_counter.h>

#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/orb_test.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>

#include <drivers/drv_hrt.h>

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <uORB/Publication.hpp>
#include <math.h>
#include <cmath>

static_assert(actuator_test_s::FUNCTION_MOTOR1 == (int)OutputFunction::Motor1, "define mismatch");
static_assert(actuator_test_s::MAX_NUM_MOTORS == (int)OutputFunction::MotorMax - (int)OutputFunction::Motor1 + 1,
	      "count mismatch");
static_assert(actuator_test_s::FUNCTION_SERVO1 == (int)OutputFunction::Servo1, "define mismatch");
static_assert(actuator_test_s::MAX_NUM_SERVOS == (int)OutputFunction::ServoMax - (int)OutputFunction::Servo1 + 1,
	      "count mismatch");

using namespace time_literals;

//class ActuatorValueMix:  public px4::ScheduledWorkItem, public ModuleParams
class ActuatorValueMix
{
public:
	static constexpr int MAX_ACTUATORS = PWM_OUTPUT_MAX_CHANNELS;

	ActuatorValueMix(const OutputFunction function_assignments[MAX_ACTUATORS]);

	//~ActuatorValueMix();
	
	void reset();

	//virtual bool updateOutputs(uint16_t value[MAX_ACTUATORS]) = 0;

	void update(int num_outputs,
				 float thrust_curve,
				 int pfl_mode, 
				 int servo_loop, 
				 int motor_run_time, 
				 bool pfl_en,
				 int in_sequence_num,
				 int servo_max);
				// float update_function_assignment[MAX_ACTUATORS]);

	void overrideValues(float value[MAX_ACTUATORS],
				 int num_outputs,
				 int pfl_mode,
				 int override_index,
				 bool servo);//, int servo_sequence_max, int motor_run_time, );

	//bool inTestMode() const { return _in_test_mode; }
	//void updateParams();
	
private:

	// Performance (perf) counters
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};


	const OutputFunction *_function_assignments;

	float motor_value;
	float servo_value;
	float _current_motor_value;
	float _current_servo_value;
	float _output_value;

	//Ref from actuator_test of update function
	bool _in_test_mode{false};
	hrt_abstime _next_timeout{0};

	float _current_outputs[MAX_ACTUATORS];
	bool _output_overridden[MAX_ACTUATORS];
};
