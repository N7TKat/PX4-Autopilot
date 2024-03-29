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

#ifndef SERVO_STATUS_HPP
#define SERVO_STATUS_HPP

#include <uORB/topics/servo_status.h>
#include <uORB/topics/servo_report.h>

class MavlinkStreamServoStatus : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamServoStatus(mavlink); }

	static constexpr const char *get_name_static() { return "SERVO_STATUS"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_SERVO_STATUS; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		static constexpr unsigned size_per_batch = MAVLINK_MSG_ID_SERVO_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		return _servo_status_sub.advertised() ? size_per_batch * _number_of_batches : 0;
	}

private:
	explicit MavlinkStreamServoStatus(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _servo_status_sub{ORB_ID(servo_status)};
	uint8_t _number_of_batches{0};

	bool send() override
	{
		static constexpr uint8_t batch_size = MAVLINK_MSG_SERVO_STATUS_FIELD_SERVO_ID_LEN;
		servo_status_s servo_status;

		if (_servo_status_sub.update(&servo_status)) {
			mavlink_servo_status_t msg{};

			msg.timestamp = servo_status.timestamp;
			msg.counter = servo_status.counter;
			msg.servo_count = servo_status.servo_count;
			msg.servo_connectiontype = servo_status.servo_connectiontype;
			msg.servo_online_flags = servo_status.servo_online_flags;

			// Ceil value of integer division. For 1-4 esc => 1 batch, 5-8 esc => 2 batches etc
			_number_of_batches = ceilf((float)servo_status.servo_count / batch_size);

			for (int batch_number = 0; batch_number < _number_of_batches; batch_number++) {
				msg.index = batch_number * batch_size;

				for (int servo_index = 0; servo_index < batch_size ; servo_index++) {
					msg.load_pct[servo_index] = servo_status.servo[servo_index].load_pct;
					msg.servo_id[servo_index] = servo_status.servo[servo_index].servo_id;
					msg.servo_function[servo_index] = servo_status.servo[servo_index].servo_function;
					msg.servo_address[servo_index] = servo_status.servo[servo_index].servo_address;
					msg.servo_position[servo_index] = servo_status.servo[servo_index].servo_position;
					msg.servo_speed[servo_index] = servo_status.servo[servo_index].servo_speed;
					msg.servo_force[servo_index] = servo_status.servo[servo_index].servo_force;
				}

				mavlink_msg_servo_status_send_struct(_mavlink->get_channel(), &msg);
			}

			return true;
		}

		return false;
	}
};

#endif // servo_STATUS_HPP
