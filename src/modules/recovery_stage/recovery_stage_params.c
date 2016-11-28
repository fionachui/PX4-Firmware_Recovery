/**
 * ROLL_PITCH_SWITCH
 *
 * for switching recovery stages (rad)
 *
 * @unit s
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Attitude Control
 */

PARAM_DEFINE_FLOAT(RP_SWITCH, 0.2f);


/**
 * RATES_SWITCH
 *
 * for switching recovery stages, (rad/s)
 *
 * @unit s
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Attitude Control
 */

PARAM_DEFINE_FLOAT(RATES_SWITCH, 0.2f);
