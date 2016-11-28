/*
 * @file recovery_stage.cpp
 * 
 * takes sensor data as input and computes the recovery stage, passed to mc_att_control
 *
 * Stage 1: Impact has occured, not stable yet
 * Stage 2: Has successfully pointed away from the wall
 * Stage 3: Has achieved hover
 * Stage 4: Has stablized vertical velocity --> return to normal flight
 *
 * @author Gareth Dicker<dicker.gareth@gmail.com>
 *
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
#include <uORB/topics/impact_characterization.h>
#include <uORB/topics/impact_recovery_stage.h>

#include <uORB/topics/debug.h>
#include <uORB/topics/parameter_update.h>

static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;			/**< daemon status flag */
static int daemon_task;						/**< Handle of daemon task / thread */

extern "C" __EXPORT int recovery_stage_main(int argc, char *argv[]);

int recovery_stage_thread_main(int argc, char *argv[]);

static void usage(const char *reason){
	if (reason) {
		warnx("%s\n", reason);
	}

	warnx("usage: recovery_stage {start|stop|status} [-p <additional params>]\n\n");
}

int recovery_stage_main(int argc, char *argv[]){
	if (argc < 2) {
		PX4_INFO("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {
		if (thread_running) {
			PX4_INFO("impact detection already running\n");
			/* this is not an error */
			return 0;
		}
		thread_should_exit = false;
		daemon_task = px4_task_spawn_cmd("recovery_stage",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_DEFAULT,
						 2000,
						 recovery_stage_thread_main,
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

int recovery_stage_thread_main(int argc, char *argv[])
{

	PX4_INFO("recovery_stage starting\n");

	thread_running = true;
	bool switchStage[3] = {0, 0, 1};

	// set up subscribers
	int _sensor_accel_sub = orb_subscribe(ORB_ID(sensor_accel));
	int _ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
	int _detection_sub = orb_subscribe(ORB_ID(impact_detection));
	int _characterization_sub = orb_subscribe(ORB_ID(impact_characterization));

	// declare local copies
	struct sensor_accel_s            		_sensor_accel;
	struct control_state_s					  _ctrl_state;
	struct impact_detection_s				   _detection;
	struct impact_characterization_s 	_characterization;
	struct impact_recovery_stage_s		  _recovery_stage;
	struct debug_s 								   _debug;

	// set them to zero initially
	memset(&_sensor_accel, 0, sizeof(_sensor_accel));	
	memset(&_ctrl_state, 0, sizeof(_ctrl_state));
	memset(&_detection, 0, sizeof(_detection));
	memset(&_characterization, 0, sizeof(_characterization));
	memset(&_recovery_stage, 0, sizeof(_recovery_stage));
	memset(&_debug, 0, sizeof(_debug));

	// declare update flags
	bool updated_sensor_accel;
	bool updated_ctrl_state;
	bool updated_detection;
	bool updated_characterization;

	orb_advert_t _recovery_stage_pub = orb_advertise(ORB_ID(impact_recovery_stage), &_recovery_stage);
	orb_advert_t _debug_pub = orb_advertise(ORB_ID(debug), &_debug);

	struct pollfd fds[1];
	fds[0].fd = _sensor_accel_sub;
	fds[0].events = POLLIN;

    while(!thread_should_exit){  

		int ret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);
		if (ret == 0) {
			continue;
		}

		//PX4_WARN("Over here's %d\n",_recovery_stage.recoveryStage);

		if (fds[0].revents & POLLIN) {
	    	// poll for subscription updates
			orb_check(_sensor_accel_sub, &updated_sensor_accel);
			orb_check(_ctrl_state_sub, &updated_ctrl_state);
			orb_check(_detection_sub, &updated_detection);
			orb_check(_characterization_sub, &updated_characterization);

			// make local copies of uORB topics
			if(updated_sensor_accel){
				orb_copy(ORB_ID(sensor_accel), _sensor_accel_sub, &_sensor_accel);
			}
			if(updated_ctrl_state){
				orb_copy(ORB_ID(control_state), _ctrl_state_sub, &_ctrl_state);
			}
			if(updated_detection){
				orb_copy(ORB_ID(impact_detection), _detection_sub, &_detection);
			}
			if(updated_characterization){
				orb_copy(ORB_ID(impact_characterization), _characterization_sub, &_characterization);
			}

			float dotNormalwithZAxis = 0.0f;

			// eventually check _characterization.accelRefIsComputed as well
			if(_characterization.accelRefIsComputed && _recovery_stage.recoveryIsReset == false){
				// recovery can start

				// check if pointing away from the wall (equivalent to checking hover right now)
				math::Quaternion q_att(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
				math::Quaternion zAxisAppended(0.0f, 0.0f, 0.0f, 1.0f);
				math::Quaternion bodyZAppended = ((q_att * zAxisAppended) * q_att.inversed());
				math::Vector<3> bodyZAxis(bodyZAppended(1), bodyZAppended(2), bodyZAppended(3));
				math::Vector<3> wallNormal(_characterization.wallNormal[0], _characterization.wallNormal[1], _characterization.wallNormal[2]);
				dotNormalwithZAxis = wallNormal * bodyZAxis;
				if(_characterization.fuzzyInput[1] > 0.0f){
					switchStage[0] = (dotNormalwithZAxis < -0.0f);
				} else{
					switchStage[0] = (dotNormalwithZAxis > 0.0f);
				}
				
				// check if hover has been achived
				//double RP_SWITCH = param_find("RP_SWITCH");
				//double RATES_SWITCH = param_find("RATES_SWITCH");
				double RP_SWITCH = 0.2;
				double RATES_SWITCH = 1.0;
				math::Vector<3> angles = q_att.to_euler();
				float roll = angles(0);
				float pitch = angles(1);
				float roll_rate = _ctrl_state.roll_rate;
				float pitch_rate = _ctrl_state.pitch_rate;
				switchStage[1] = fabs(roll) < RP_SWITCH && fabs(pitch) < RP_SWITCH && \
						         fabs(roll_rate) < RATES_SWITCH  && fabs(pitch_rate) < RATES_SWITCH;
    			_debug.floats[1] = roll;
    			_debug.floats[2] = roll_rate;
    			_debug.floats[3] = pitch;
    			_debug.floats[4] = pitch_rate;

				if (_recovery_stage.recoveryStage == 1){
		            if (switchStage[0]) {
	            		_recovery_stage.recoveryStage = 2;
		            }
		        } 
		        else if (_recovery_stage.recoveryStage == 2){
		            if (switchStage[1]) {
	            		_recovery_stage.recoveryStage = 3;
		            }
		        } 
		        else if (_recovery_stage.recoveryStage == 3){
		            if (switchStage[2]) {
						_recovery_stage.recoveryIsReset = true;
						_recovery_stage.recoveryStage = 0;	
						switchStage[0] = false;
						switchStage[1] = false;
					}	            		
		        }
		        else{
					_recovery_stage.recoveryStage = 1; // executes upon opening this loop
		        }
		    }
			
			if (_characterization.accelRefIsComputed == false && _detection.inRecovery == false &&_recovery_stage.recoveryIsReset == true){
				_recovery_stage.recoveryIsReset = false;
				dotNormalwithZAxis = 0.0f;
			}

			_debug.boolean[0] = switchStage[0];
			_debug.boolean[1] = switchStage[1];
			_debug.floats[0] = dotNormalwithZAxis;

        	orb_publish(ORB_ID(impact_recovery_stage), _recovery_stage_pub, &_recovery_stage);
			orb_publish(ORB_ID(debug), _debug_pub, &_debug);

    	}
    }

	PX4_INFO("recovery_stage exiting.\n");
	thread_running = false;

    return OK;
}




