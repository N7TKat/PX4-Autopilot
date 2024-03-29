/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#ifndef SERVO_INFO_HPP
#define SERVO_INFO_HPP

#include <uORB/topics/esc_status.h>
#include <uORB/topics/debug_vect.h>
#include <uORB/topics/servo_info.h>

class MavlinkStreamServoInfo : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamServoInfo(mavlink); }

	static constexpr const char *get_name_static() { return "SERVO_INFO"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_SERVO_INFO; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		static constexpr unsigned size_per_batch = MAVLINK_MSG_ID_SERVO_INFO_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		return _servo_info_sub.advertised() ? size_per_batch * _number_of_batches : 0;
	}

private:
	explicit MavlinkStreamServoInfo(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _servo_info_sub{ORB_ID(servo_info)};
	uint8_t _number_of_batches{0};

	bool send() override
	{
		servo_info_s servo_info;

		if (_servo_info_sub.update(&servo_info)) {

			mavlink_servo_info_t msg{};

			msg.time_usec = servo_info.timestamp;
			msg.counter = servo_info.counter;
			msg.count = servo_info.count;
			msg.connection_type = servo_info.connectiontype;
			msg.error_count[0] = servo_info.errorcount[0];
			msg.temperature[0] = servo_info.temperature[0];

				mavlink_msg_servo_info_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // ESC_INFO_HPP
