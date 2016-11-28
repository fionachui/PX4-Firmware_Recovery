
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
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/control_state.h>

#include <uORB/topics/impact_detection.h>
#include <uORB/topics/impact_recovery_stage.h>
#include <uORB/topics/impact_characterization.h>
#ifndef IMPACT_CHARACTERIZATION_H
#define IMPACT_CHARACTERIZATION_H

#include <uORB/topics/debug.h>

#include <nuttx/sched.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>

//fuzzy logic libraries
#include <lib/eFLL/Fuzzy.h>
#include <lib/eFLL/FuzzyComposition.h>
#include <lib/eFLL/FuzzyInput.h>
#include <lib/eFLL/FuzzyIO.h>
#include <lib/eFLL/FuzzyOutput.h>
#include <lib/eFLL/FuzzyRule.h>
#include <lib/eFLL/FuzzyRuleAntecedent.h>
#include <lib/eFLL/FuzzyRuleConsequent.h>
#include <lib/eFLL/FuzzySet.h>

#define QUATQUEUESIZE 5
#define PREIMPACTCYCLES 2

//class definitions
class QuaternionQueue{
	math::Quaternion queue[QUATQUEUESIZE];
	int head, tail;
public:
	QuaternionQueue();
	void quatqueue(math::Quaternion quat);
	math::Quaternion readquatqueue(int queueidx);
};

//main
int impact_characterization_thread_main(int argc, char *argv[]);

//custom math functions
math::Vector<3> crossProduct(const math::Vector<3> vect1, const math::Vector<3> vect2);
float signf(const float number);
float rad2deg(const float angle);

#endif
