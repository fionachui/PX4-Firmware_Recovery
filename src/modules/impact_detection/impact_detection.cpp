/**
 * @file impact_detection.cpp
 * 
 * Detects whether an impact has occured
 * 
 * modules impact_characterization and mc_att_control make use of this flag.
 *
 * @author Gareth Dicker<dicker.gareth@gmail.com>

 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <px4_defines.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <errno.h>
#include <lib/mathlib/mathlib.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/control_state.h>

#include <uORB/topics/impact_detection.h>
#include <uORB/topics/impact_recovery_stage.h>

#include <uORB/topics/debug.h>

#include <nuttx/sched.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>

static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;			/**< daemon status flag */
static int daemon_task;						/**< Handle of daemon task / thread */

extern "C" __EXPORT int impact_detection_main(int argc, char *argv[]);

int impact_detection_thread_main(int argc, char *argv[]);

static void
usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}

	warnx("usage: impact_detection {start|stop|status} [-p <additional params>]\n\n");
}

int impact_detection_main(int argc, char *argv[]){
		if (argc < 2) {
		PX4_INFO("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			PX4_INFO("impact_detection already running\n");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		daemon_task = px4_task_spawn_cmd("impact_detection",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_DEFAULT,
						 2000,
						 impact_detection_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			PX4_INFO("\trunning\n");

		} else {
			PX4_INFO("\tnot started\n");
		}

		return 0;
	}

	usage("unrecognized command");
	return 1;
}


int impact_detection_thread_main(int argc, char *argv[])
{

	PX4_INFO("impact_detection starting\n");

	thread_running = true;

	// set up subscribers
	int _sensor_accel_sub = orb_subscribe(ORB_ID(sensor_accel));
	int _ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
	int _recovery_stage_sub = orb_subscribe(ORB_ID(impact_recovery_stage));
	int _armed_sub = orb_subscribe(ORB_ID(actuator_armed));

	// declare local copies
	struct sensor_accel_s             	  _sensor_accel;
	struct control_state_s					_ctrl_state;
	struct impact_detection_s 				 _detection;
	struct impact_recovery_stage_s  	_recovery_stage;
	struct actuator_armed_s						 _armed;				

	// set them to zero initially
	memset(&_sensor_accel, 0, sizeof(_sensor_accel));	
	memset(&_ctrl_state, 0, sizeof(_ctrl_state));
	memset(&_detection, 0, sizeof(_detection));
	memset(&_recovery_stage, 0, sizeof(_recovery_stage));
	memset(&_armed, 0, sizeof(_armed));

	// declare update flags
	bool updated_sensor_accel;
	bool updated_ctrl_state;
	bool updated_recovery_stage;
	bool updated_armed;
	bool accel_is_done_spike = true;

	orb_advert_t _detection_pub = orb_advertise(ORB_ID(impact_detection), &_detection);

	struct pollfd fds[1];

	fds[0].fd = _sensor_accel_sub;
	fds[0].events = POLLIN;
                
    while(!thread_should_exit){  
    	//TODO: check if armed

		int ret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);
		if (ret == 0) {
			continue;
		}

		if (fds[0].revents & POLLIN){		
		   	// poll for subscription updates
			orb_check(_sensor_accel_sub, &updated_sensor_accel);
			orb_check(_ctrl_state_sub, &updated_ctrl_state);
			orb_check(_recovery_stage_sub, &updated_recovery_stage);
			orb_check(_armed_sub, &updated_armed);
			
			// make local copies of uORB topics
			if(updated_sensor_accel){
				orb_copy(ORB_ID(sensor_accel), _sensor_accel_sub, &_sensor_accel);
			}
			if(updated_ctrl_state){
				orb_copy(ORB_ID(control_state), _ctrl_state_sub, &_ctrl_state);
			}
			if(updated_recovery_stage){
				orb_copy(ORB_ID(impact_recovery_stage), _recovery_stage_sub, &_recovery_stage);
			}
			if (updated_armed) {
				orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);
			}

			math::Vector<3> accel(_sensor_accel.x, _sensor_accel.y, _sensor_accel.z);
			math::Quaternion q_att(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
			//get rotation matrix
			math::Matrix<3, 3> R = q_att.to_dcm();
			//rotate accelerometer readings into world frame
			math::Vector<3> accelWorldFrame = R * accel;
			math::Vector<2> accelHorizontalComponents(accelWorldFrame(0), accelWorldFrame(1));

			if(_armed.armed){
				if (!accel_is_done_spike){
					if (accelHorizontalComponents.length() <= 9.81f){
						accel_is_done_spike = true;
					}
				}
				if(!_detection.inRecovery && accel_is_done_spike){
					if (accelHorizontalComponents.length() > 9.81f){
						 _detection.inRecovery = true;
						 accel_is_done_spike = false;

					}
		        }
		        else if(_detection.inRecovery){ // could just do 'else'
		        	if(_recovery_stage.recoveryIsReset){
		        		_detection.inRecovery = false;
		        	}
				}
			}	

        	orb_publish(ORB_ID(impact_detection), _detection_pub, &_detection);
			 
    	} //end polling

    } //end while

	PX4_INFO("impact_detection exiting.\n");

	thread_running = false;

    return OK;
} //end main


