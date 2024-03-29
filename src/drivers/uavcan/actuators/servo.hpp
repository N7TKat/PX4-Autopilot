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

#pragma once

#include <uavcan/uavcan.hpp>
#include <uavcan/equipment/actuator/Status.hpp> /*include for servo_status*/
#include <uavcan/equipment/actuator/ArrayCommand.hpp>
#include <lib/perf/perf_counter.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/actuator_outputs.h>
#include <drivers/drv_hrt.h>
#include <lib/mixer_module/mixer_module.hpp>

/*ADDITIONAL include for Transceiver from CAN to uORB to Get Message from CAN Servo*/
#include <drivers/drv_hrt.h>
#include <lib/mixer_module/mixer_module.hpp>
	
	/*Debug Vector for Debug*/
#include <uORB/topics/debug_vect.h>

	/*Custom Additional uORB message*/
#include <uORB/topics/servo_status.h>
#include <uORB/topics/servo_power_status.h>
#include <uORB/topics/servo_temp_status.h>

	/*Calling UAVCAN Message*/
#include <uavcan/equipment/power/CircuitStatus.hpp>
#include <uavcan/equipment/device/Temperature.hpp>


class UavcanServoController
{
public:

	static constexpr int MAX_ACTUATORS = 8;
	static constexpr unsigned MAX_RATE_HZ = 50;
	static constexpr unsigned UAVCAN_COMMAND_TRANSFER_PRIORITY = 6;	///< 0..31, inclusive, 0 - highest, 31 - lowest

	UavcanServoController(uavcan::INode &node);
	~UavcanServoController() = default;

	void update_outputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs);

	int init();

	/* Sets the number of servo and enable timer */
    void set_servo_count(uint8_t count);
	servo_status_s &servo_status() {return _servo_status;}
	
	void set_servo_power_count(uint8_t power_count) ;
	servo_power_status_s &servo_power_status() {return _servo_power_status; }
	
	void set_servo_temp_count(uint8_t temp_count) ;
	servo_temp_status_s &servo_temp_status() {return _servo_temp_status; }

private:
	uavcan::INode &_node;
	
	uavcan::Publisher<uavcan::equipment::actuator::ArrayCommand> _uavcan_pub_array_cmd;
	typedef uavcan::MethodBinder<UavcanServoController *, 
		void (UavcanServoController::*)(const uavcan::ReceivedDataStructure<uavcan::equipment::actuator::Status>&)> StatusCbBinder;
	typedef uavcan::MethodBinder<UavcanServoController *,
		void (UavcanServoController::*)(const uavcan::ReceivedDataStructure<uavcan::equipment::power::CircuitStatus>&)> PowerStatusCbBinder;
	typedef uavcan::MethodBinder<UavcanServoController *,	
		void (UavcanServoController::*)(const uavcan::ReceivedDataStructure<uavcan::equipment::device::Temperature>&)> TempStatusCbBinder;

	uavcan::Subscriber<uavcan::equipment::actuator::Status, StatusCbBinder> _uavcan_sub_status;	
	uavcan::Subscriber<uavcan::equipment::power::CircuitStatus, PowerStatusCbBinder> _uavcan_sub_power_status;	
	uavcan::Subscriber<uavcan::equipment::device::Temperature, TempStatusCbBinder> _uavcan_sub_temp_status;	

	uORB::PublicationMulti<servo_status_s> _servo_status_pub{ORB_ID(servo_status)};
	uORB::PublicationMulti<servo_power_status_s> _servo_power_status_pub{ORB_ID(servo_power_status)};
	uORB::PublicationMulti<servo_temp_status_s> _servo_temp_status_pub{ORB_ID(servo_temp_status)};

    uORB::Publication<debug_vect_s> _debug_status_pub{ORB_ID(debug_vect)};
	
	/* Servo status message reception will be reported via this callback. */
	void servo_status_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::actuator::Status> &msg);
	void servo_power_status_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::power::CircuitStatus> &msg);
	void servo_temp_status_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::device::Temperature> &msg);

	/* Checks all the Servo's freshness based on timestamp, if an servo exceeds the timeout then is flagged offline. */
    uint8_t check_servos_status();
	uint8_t check_servos_power_status();
	uint8_t check_servos_temp_status();
	
	/*Declare ServoStatus Structure*/
	servo_status_s _servo_status{};
	servo_power_status_s _servo_power_status{};
	servo_temp_status_s _servo_temp_status{};

	/*Declare Variable to handle servo*/
	uint8_t _servo_count{0};
	uint8_t _servo_power_count{0};
	uint8_t _servo_temp_count{0};

	/*Declare Debug_vect for debugging data*/
    debug_vect_s _debug_vect = {};

};
