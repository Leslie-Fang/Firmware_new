#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <platforms/px4_defines.h>

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <px4_time.h>
#include <nuttx/sched.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>

#include <drivers/drv_hrt.h>

#include <poll.h>
#include <uORB/uORB.h>
#include <uORB/topics/vision_position_estimate.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/localsense_kalman.h>


#ifdef __cplusplus
extern "C" {
#endif
#include "MarkerEKF/MarkerEKF.h"
#include "MarkerEKF/MarkerEKF_initialize.h"
//#include "attitude_estimator_ekf_params.h"
#ifdef __cplusplus
}
#endif

static bool thread_should_exit = false;		/**< localsense_kalman exit flag */
static bool thread_running = false;		/**< localsense_kalman status flag */
static int localsense_kalman_task;				/**< Handle of localsense_kalman task / thread */

/**
 * localsense_kalman management function.
 */
extern "C" __EXPORT int localsense_kalman_main(int argc, char *argv[]);

/**
 * Mainloop of localsense_kalman.
 */
int localsense_kalman_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}

	warnx("usage: localsense_kalman {start|stop|status} [-p <additional params>]\n\n");
}

/**
 * The localsense_kalman app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int localsense_kalman_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("localsense_kalman already running\n");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		localsense_kalman_task = px4_task_spawn_cmd("localsense_kalman",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_DEFAULT,
						 2000,
						 localsense_kalman_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");

		} else {
			warnx("\tnot started\n");
		}

		return 0;
	}

	usage("unrecognized command");
	return 1;
}

int localsense_kalman_thread_main(int argc, char *argv[])
{
	//dt set a initial value, then would be changed 
	//float dt = 0.005f;
	warnx("[localsense_kalman] starting\n");

	//bool R_valid=false;
	//bool acc_valid=false;
	//bool zk_valid=false;
	bool wait_localsense=true;
	hrt_abstime t;
	hrt_abstime t_prev=0;
	//measurement: gloabl accx,global accy,localsense_x,localsense_y
	float z_k[4] = {0.0f, 0.0f, 10.0f, 10.0f};
	float q_a=0.05;
	float q_v=0.05;
	float q_x=0.05;
	float r_a=0.05;
	float r_x=0.5;
	/*output value is not important*/
	float xa_apo[6]={0.0f,0.0f,0.0f,0.0f,10.0f,10.0f};
	//hrt_abstime accel_timestamp = 0;
	//hrt_abstime localsense_timestamp = 0;

	/*initlized position*/
	float localsense_initx=10;
	float localsense_inity=10;
	float acc[] = { 0.0f, 0.0f, 0.0f };

	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	struct sensor_combined_s raw;
	memset(&raw, 0, sizeof(raw));
	struct vision_position_estimate_s localsense;
	memset(&localsense, 0, sizeof(localsense));

	int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int sub_raw = orb_subscribe(ORB_ID(sensor_combined));
	int sub_localsense = orb_subscribe(ORB_ID(vision_position_estimate));


	struct localsense_kalman_s localsense_kalman_filter;
	memset(&localsense_kalman_filter, 0, sizeof(localsense_kalman_filter));
	orb_advert_t pos_pub = orb_advertise(ORB_ID(localsense_kalman), &localsense_kalman_filter);

	thread_running = true;

	//subscribe onece to init some data
	/*wait localsense to initilized 
	then get the init position measurement data
	*/
	px4_pollfd_struct_t fds_init[1] = {};
	fds_init[0].fd = sub_localsense;
	fds_init[0].events = POLLIN;
	
	while (wait_localsense && !thread_should_exit)
	{
		int ret = px4_poll(&fds_init[0], 1, 1000);
		if (ret < 0) {
			/* poll error */
			//PX4_WARN("INAV poll error");
		} else if (ret > 0) {
			/*get init localsense data*/
			if (fds_init[0].revents & POLLIN) {
				orb_copy(ORB_ID(vision_position_estimate), sub_localsense, &localsense);
				localsense_initx=localsense.x;
				localsense_inity=localsense.y;
				t = hrt_absolute_time();
				t_prev=t;
				wait_localsense=false;
			}
		}
	}
	//initialize state variable (intilized position is (10,10))
	//initiallize covariance 
	//set R and Q's initialed flag to false
	MarkerEKF_initialize(localsense_initx,localsense_inity);

	/* Main loop*/
	while (!thread_should_exit) {
		warnx("Hello localsense_kalman!\n");

		bool ATT_updated=false;
		bool ACC_updated=false;

		px4_pollfd_struct_t fds[1];
		fds[0].fd = sub_localsense;
		fds[0].events = POLLIN;
		int ret = px4_poll(fds, 1, 1000);


		//hrt_abstime t = hrt_absolute_time();
		/*here check localsense is valid or not*/
		if (ret < 0) {
			/* XXX this is seriously bad - should be an emergency */
		} else if (ret == 0) {
			/* check if we're in HIL - not getting sensor data is fine then */
			//orb_copy(ORB_ID(vehicle_control_mode), sub_control_mode, &control_mode);
			//if (!control_mode.flag_system_hil_enabled) {
			//	warnx("WARNING: Not getting sensor data - sensor app running?");
			//}
		}
		else
		{
			/*sensordata is fine*/
			/*here only judge localsense is updated or not*/
			if (fds[0].revents & POLLIN)
			{
				warnx("get localsense data!\n");
				orb_check(vehicle_attitude_sub, &ATT_updated);
				if(ATT_updated)
				{
					warnx("get ATT data!\n");
					orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &att);
					orb_check(sub_raw, &ACC_updated);
					if (ACC_updated)
					{
						warnx("get ACC data!\n");
						/* code */
						orb_copy(ORB_ID(sensor_combined), sub_raw, &raw);
						/* transform acceleration vector from body frame to NED frame */
						for (int i = 0; i < 3; i++) {
							acc[i] = 0.0f;
							for (int j = 0; j < 3; j++) {
							acc[i] += PX4_R(att.R, i, j) * raw.accelerometer_m_s2[j];
							}
						}
						z_k[0]=acc[0];
						z_k[1]=acc[1];
						orb_copy(ORB_ID(vision_position_estimate), sub_localsense, &localsense);
						z_k[2]=localsense.x;
						z_k[3]=localsense.y;
						
						t = hrt_absolute_time();
						float dt = t_prev > 0 ? (t - t_prev) / 1000000.0f : 0.0f;
						t_prev=t;
						MarkerEKF(dt,z_k,q_a,q_v,q_x,r_a,r_x,xa_apo);
						localsense_kalman_filter.vx=xa_apo[2];
						localsense_kalman_filter.vy=xa_apo[3];
						localsense_kalman_filter.x=xa_apo[4];
						localsense_kalman_filter.y=xa_apo[5];
						printf("%f,%f,%f,%f\n",(double)xa_apo[2],(double)xa_apo[3],(double)xa_apo[4],(double)xa_apo[5]);
						if (pos_pub == nullptr) {
							//pos_pub = orb_advertise(ORB_ID(vision_position_estimate), &localsense);
							pos_pub = orb_advertise(ORB_ID(localsense_kalman), &localsense_kalman_filter);
						} else {
							orb_publish(ORB_ID(localsense_kalman), pos_pub, &localsense_kalman_filter);
							//mavlink_log_info(mavlink_fd, "[localsense] position x:%f  y:%f", (double)localsense.x,(double)localsense.y);
						}	
					}
				}
			}
		}

		//sleep(10)
		usleep(10000);
	}

	thread_running = false;

	return 0;
}