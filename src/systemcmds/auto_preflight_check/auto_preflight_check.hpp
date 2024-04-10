#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/px4_config.h>

#include <lib/perf/perf_counter.h>

#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/orb_test.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>

#include <drivers/drv_hrt.h>

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/actuator_test.h>
#include <math.h>
#include <cmath>

#include <uORB/topics/automatic_hardware_testing.h>


using namespace time_literals;

class AutoPreflightCheck : public ModuleBase<AutoPreflightCheck>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	AutoPreflightCheck();
	~AutoPreflightCheck() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	void Run() override;

	void Parameters_updated();

	static void actuator_test(int function, float value, int timeout_ms, bool release_control);
	static void usage(const char *reason);

	// // Publications
	//uORB::Publication<orb_test_s> _orb_test_pub{ORB_ID(orb_test)};
	// uORB::Publication<servo_info_s> _servo_info_pub{ORB_ID(servo_info)};

	// // Subscriptions
	uORB::SubscriptionCallbackWorkItem _sensor_accel_sub{this, ORB_ID(sensor_accel)};        // subscription that schedules WorkItemExample when updated
	// uORB::SubscriptionInterval         _parameter_update_sub{ORB_ID(parameter_update), 1_s}; // subscription limited to 1 Hz updates
	// uORB::Subscription                 _vehicle_status_sub{ORB_ID(vehicle_status)};          // regular subscription for additional data

	uORB::Subscription _automatic_hardware_testing_sub{ORB_ID(automatic_hardware_testing)}; ///< automatic Hardware Testing Subscriptions
	

	// Performance (perf) counters
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};


	// Publications

	
	// Subscriptions
	uORB::SubscriptionInterval         _parameter_update_sub{ORB_ID(parameter_update), 1_s}; // subscription limited to 1 Hz updates
	
	// Parameters
	DEFINE_PARAMETERS(
		(ParamBool<px4::params::AT_PFL_EN>) _param_auto_pfl_en,
		/*(ParamBool<px4::params::A_AUTO_PFL_MT_LOOP>) _param_work_auto_en, */
		(ParamInt<px4::params::AT_PFL_SV_LOOP>) _param_auto_pfl_sv_loop,
        (ParamFloat<px4::params::AT_PFL_MT_TIME>) _param_auto_pfl_mt_time,
		(ParamInt<px4::params::AT_PFL_MODE>) _param_auto_pfl_mode
	)


	float value = 0.10f;
	int motor_oder[5] = {0, 3, 1, 2, 4};
	int motor_run_time; //how long each motor spinning in once time (default 1.5 sec, max 5.0 sec)
	
	bool ACTUATOR_SHOULD_EXIT{false};
	bool ACTUATOR_RUN{true};
	
	bool servo_test{false};
	bool param_check{true};


	//MOTOR LOGIC
	bool motor_test{false}; //FUCTION MOTOR motor_test
	bool motor_test_is_done{false}; 
	bool DO_NEXT_MOTOR{true};
	bool DO_MOTOR_SEQUENCE{false};
	bool IN_MOTOR_SEQUENCE{false};
	bool DO_NEXT_MOTOR_DELAY_TIMER{false};
	bool MOTOR_SEQUENCE_DELAY_TIMER{false};

	hrt_abstime MOTOR_SEQUENCE_DELAY{0};  //(MOTOR RUN TIME)
	hrt_abstime DO_NEXT_MOTOR_DELAY{0}; //(WAITING TIME)

	int j = 0;
	int MOTOR_ID = 0;
	int MOTOR_MAX = 5;
	int MOTOR_SEQUENCE = 1;
	int MOTOR_SEQUENCE_MAX = 1;


	//SERVO LOGIC
	bool DO_NEXT_SERVO{true};
	bool DO_SERVO_SEQUENCE{false};
	bool IN_SERVO_SEQUENCE{false};
	bool DO_SERVO_SINE_WAVE{false};
	bool DO_NEXT_SERVO_DELAY_TIMER{false};
	bool SERVO_SEQUENCE_DELAY_TIMER{false};
	bool SERVO_SINE_WAVE_STEP_TIMER{false};

	hrt_abstime SERVO_SEQUENCE_DELAY{0};  //(MOTOR RUN TIME)
	hrt_abstime SERVO_SINE_WAVE_STEP_TIMEOUTS{0};  //(MOTOR RUN TIME)
	hrt_abstime DO_NEXT_SERVO_DELAY{0}; //(WAITING TIME)

	float one_sine_wave[17] = {0,0.382,0.707,0.924,1,0.924,0.707,0.382,0,-0.382,-0.707,-0.924,-1,-0.924,-0.707,-0.382,0};
	int one_sine_wave_step_max = 16;
	int one_sine_wave_step = 1;
	int one_sine_wave_step_time = 250; //   4000/16= 250 ms

	int SERVO_ID = 0;
	int SERVO_MAX = 2;
	int SERVO_SEQUENCE = 1;
	//int SERVO_SEQUENCE_MAX = 5;
	int servo_sequence_max;

	
	//mod
	hrt_abstime update_timeout{0};
	bool done_all_motor{false};
	bool timeout_timer{false};
				
};
