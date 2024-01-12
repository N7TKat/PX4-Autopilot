/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file px4_tutorial_mavlink_debug.cpp
 * Debug application example for PX4 autopilot
 * Example for Subsribe Local position and velocity and send to debug message
 * @author N7TKat User <mail@example.com>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <poll.h>

#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/debug_value.h>
#include <uORB/topics/debug_vect.h>
#include <uORB/topics/debug_array.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/example_message.h>
#include <uORB/topics/vehicle_global_position.h>

extern "C" __EXPORT int px4_tutorial_mavlink_debug_main(int argc, char *argv[]);

int px4_tutorial_mavlink_debug_main(int argc, char *argv[])
{
	printf("Hello Debug Tutorials!\n");

	/* advertise topic to subscribe vehicle_local_position*/
	int localposition_sub_fd = orb_subscribe(ORB_ID(vehicle_local_position));
	/* limit the update rate to 2 Hz */
	orb_set_interval(localposition_sub_fd, 500);

	/* advertise topic to subscribe BatteryStatus*/
	int batterystatus_sub_fd = orb_subscribe(ORB_ID(battery_status));
	/* limit the update rate to 1 Hz */
	orb_set_interval(localposition_sub_fd, 1000);

	/* advertise topic to subscribe VehicleGlobalPosition*/
	int vehicle_global_position_sub_fd = orb_subscribe(ORB_ID(vehicle_global_position));
	/* limit the update rate to 1 Hz */
	orb_set_interval(vehicle_global_position_sub_fd, 1000);

	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[] = {
		{ .fd = localposition_sub_fd,   .events = POLLIN },
		{ .fd = batterystatus_sub_fd,   .events = POLLIN },
		{ .fd = vehicle_global_position_sub_fd,   .events = POLLIN }
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};

	/* advertise named debug value */
	struct debug_key_value_s dbg_key;
	strncpy(dbg_key.key, "velx", 10);
	dbg_key.value = 0.0f;
	orb_advert_t pub_dbg_key = orb_advertise(ORB_ID(debug_key_value), &dbg_key);

	/* advertise indexed debug value */
	struct debug_value_s dbg_ind;
	dbg_ind.ind = 42;
	dbg_ind.value = 0.5f;
	orb_advert_t pub_dbg_ind = orb_advertise(ORB_ID(debug_value), &dbg_ind);

	/* advertise debug vect */
	struct debug_vect_s dbg_vect;
	strncpy(dbg_vect.name, "Local3D", 10);
	dbg_vect.x = 1.0f;
	dbg_vect.y = 2.0f;
	dbg_vect.z = 3.0f;
	orb_advert_t pub_dbg_vect = orb_advertise(ORB_ID(debug_vect), &dbg_vect);

	/* advertise debug array */
	struct debug_array_s dbg_array;
	dbg_array.id = 1;
	strncpy(dbg_array.name, "dbg_array", 10);
	orb_advert_t pub_dbg_array = orb_advertise(ORB_ID(debug_array), &dbg_array);

	/* advertise ExampleMessage */
	struct example_message_s exam_msg;
	memset(&exam_msg, 0, sizeof(exam_msg));
	orb_advert_t pub_exam_msg = orb_advertise(ORB_ID(example_message), &exam_msg);

	int error_counter = 0;
	int value_counter = 0;

	while (value_counter < 51) {

		uint64_t timestamp_us = hrt_absolute_time();

		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = px4_poll(fds, 1, 1000);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a second");

		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}

			error_counter++;

		} else {

			if (fds[0].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				struct vehicle_local_position_s local_pose;
				/* copy local position data into local buffer */
				orb_copy(ORB_ID(vehicle_local_position), localposition_sub_fd, &local_pose);
				PX4_INFO("Local Position of JUAV is : x\t%8.4f y\t%8.4f z\t%8.4f",
					 (double)local_pose.x,
					 (double)local_pose.y,
					 (double)local_pose.z);

				/* copy Battery Status data into local buffer */
				struct battery_status_s batt_stats;
				orb_copy(ORB_ID(battery_status), batterystatus_sub_fd, &batt_stats);
				PX4_INFO("Battery Status : \n Voltage :\t%8.4f\n Current :\t%8.4f\n Batt.Cell:\t%8.4f",
					 (double)batt_stats.voltage_v,
					 (double)batt_stats.current_a,
					 (double)batt_stats.cell_count);

				/* copy Global position data into local buffer */
				struct vehicle_global_position_s glob_pose;
				orb_copy(ORB_ID(vehicle_global_position), vehicle_global_position_sub_fd, &glob_pose);

				/* copy copied data to ExampleMessage */
				exam_msg.timestamp = timestamp_us;
				strncpy(exam_msg.name, "Example Location Message", 25);
				exam_msg.local_position[0] = local_pose.x;
				exam_msg.local_position[1] = local_pose.y;
				exam_msg.local_position[2] = local_pose.z;
				exam_msg.glob_pose_lat = glob_pose.lat;
				exam_msg.glob_pose_lon = glob_pose.lon;
				exam_msg.glob_pose_alt = glob_pose.alt;

				/* send one named value */
				dbg_key.value = value_counter;
				dbg_key.timestamp = timestamp_us;
				orb_publish(ORB_ID(debug_key_value), pub_dbg_key, &dbg_key);

				/* send one indexed value */
				dbg_ind.value = 0.5f * value_counter;
				dbg_ind.timestamp = timestamp_us;
				orb_publish(ORB_ID(debug_value), pub_dbg_ind, &dbg_ind);

				/* send one vector */
				dbg_vect.x = local_pose.x;
				dbg_vect.y = local_pose.y;
				dbg_vect.z = local_pose.z;
				dbg_vect.timestamp = timestamp_us;
				orb_publish(ORB_ID(debug_vect), pub_dbg_vect, &dbg_vect);

				/* send one array */
				strncpy(dbg_array.name, "Debug_array_edited", 10);
				for (size_t i = 0; i < debug_array_s::ARRAY_SIZE; i++) {
					dbg_array.data[i] = value_counter + i * 0.01f;
				}
				dbg_array.timestamp = timestamp_us;
				orb_publish(ORB_ID(debug_array), pub_dbg_array, &dbg_array);

				/* Publish Example Message*/
				orb_publish(ORB_ID(example_message), pub_exam_msg, &exam_msg);

				/*warnx("sent one more value..");*/

				value_counter++;

				px4_usleep(100000);

			}
		}
	}

	PX4_INFO("exiting");

	return 0;
}
