#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_time.h>
#include <nuttx/sched.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#include <uORB/uORB.h>
#include <uORB/topics/vision_position_estimate.h>


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

	//initialize state variable and covariance 
	//set R and Q's flag
	MarkerEKF_initialize();
	thread_running = true;


	while (!thread_should_exit) {
		warnx("Hello localsense_kalman!\n");
		//subscribe global accx,accy, localsense_x,localsense_y

		//subscribe dt time
		//dt=
		//sleep(10)
		sleep(10);
	}

	warnx("[localsense_kalman] exiting.\n");

	thread_running = false;

	return 0;
}