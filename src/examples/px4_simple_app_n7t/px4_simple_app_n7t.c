/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>
#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
//#include <uORB/topics/vehicle_acceleration.h>
//#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/debug_vect.h>

__EXPORT int px4_simple_app_n7t_main(int argc, char *argv[]);

int px4_simple_app_n7t_main(int argc, char *argv[])
{
	PX4_INFO("Hello Sky!");
	PX4_INFO("You're on branch v1.14.0-N7TKatdev");

	/* subscribe to vehicle_local_position topic */
	int localposition_sub_fd = orb_subscribe(ORB_ID(vehicle_local_position));
	/* limit the update rate to 2 Hz */
	orb_set_interval(localposition_sub_fd, 500);

	/* advertise debug_vect topic */
	struct debug_vect_s dbg_vect;
	memset(&dbg_vect, 0, sizeof(dbg_vect));
	orb_advert_t dbg_vect_pub = orb_advertise(ORB_ID(debug_vect), &dbg_vect);

	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[] = {
		{ .fd = localposition_sub_fd,   .events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};

	int error_counter = 0;

	for (int i = 0; i < 101; i++) {
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
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(vehicle_local_position), localposition_sub_fd, &local_pose);
				PX4_INFO("Local Position of JUAV is : x\t%8.4f y\t%8.4f z\t%8.4f",
					 (double)local_pose.x,
					 (double)local_pose.y,
					 (double)local_pose.z);

				PX4_INFO("1st Phase pass : Preparing to Debug_Vect");

				/* set att and publish this information for other apps
				 the following does not have any meaning, it's just an example
				*/
				dbg_vect.timestamp = timestamp_us;
				strncpy(dbg_vect.name, "Local_Pos", 10);
				dbg_vect.x = local_pose.x;
				dbg_vect.y = local_pose.y;
				dbg_vect.z = local_pose.z;

				orb_publish(ORB_ID(debug_vect), dbg_vect_pub, &dbg_vect);
			}

			/* there could be more file descriptors here, in the form like:
			 * if (fds[1..n].revents & POLLIN) {}
			 */
		}
	}

	PX4_INFO("exiting");

	return 0;
}
