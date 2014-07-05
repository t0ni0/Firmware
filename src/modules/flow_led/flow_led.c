/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Antonio Sanniravong <antonio.sanniravong@polymtl.ca>
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
 * @file flow_led.c
 * PX4Flow assistive LED lighting control.
 */
 
#include <nuttx/config.h>
#include <stdio.h>
#include <errno.h>
#include <poll.h>
#include <uORB/uORB.h>
#include <uORB/topics/optical_flow.h>

__EXPORT int flow_led_main(int argc, char *argv[]);

int flow_led_main(int argc, char *argv[])
{
	/* Initialize structs */
	struct optical_flow_s flow;
	memset(&flow, 0, sizeof(flow));

	struct actuator_controls_s actuators;
	memset(&actuators, 0, sizeof(actuators));

	/* Subscribe to flow topic */
	int flow_sub = orb_subscribe(ORB_ID(optical_flow));

	/* Advertise on actuators topic */
	orb_advert_t actuators_pub = orb_advertise(ORB_ID(actuator_outputs), &actuators);

	struct pollfd fds[] = {
		{ .fd = flow_sub,   .events = POLLIN }
	};

	int error_counter = 0;
	int flow_q = 0;
	float sonar = 0.0f;
	float sonar_avg = 0.0f;
	float led_out = 0.0f;

	while (1) {
		/* wait for update for 1000 ms */
		int poll_result = poll(fds, 1, 1000);
 
		if (poll_result == 0) {
			/* No new flow data */
			printf("[flow_led] Got no data within a second\n");

		} else if (poll_result < 0) {
			/* ERROR */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* Use error counter to prevent flooding */
				printf("[flow_led] ERROR return value from poll(): %d\n", poll_result);
			}
			error_counter++;

		} else {
 
			if (fds[0].revents & POLLIN) {
				/* Flow data obtained */

				/* Copy flow data to local buffer */
				orb_copy(ORB_ID(optical_flow), flow_sub, &flow);
				flow_q = flow.quality;
				sonar = flow.ground_distance_m;

			}
		}
	}

	return 0;
}