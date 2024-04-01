/**
 * @file ActuatorValueMix.c
 * Parameters for custom ActuatorValueMix Module
 *
 * @author Twominus
 **/

/**
 * Custom debug Parameter
 *
 * @group AUTO_PREFLIGHT_CHECK
 * @value 2 - Mode 1: 2 Sine Wave
 * @value 3 - Mode 2: 3 Sine Wave
 * @value 4 - Mode 3: 4 Sine Wave
 * @value 5 - Mode 4: 5 Sine Wave
 * @min 2
 * @max 5
 */
PARAM_DEFINE_INT32(A_PFL_SV_LOOP, 2);


/**
 * Custom INT for AUTO_PREFLIGHT_CHECK Boolean
 *
 * 0 for Disable
 * 1 for Enable
 *
 * @boolean
 * @group AUTO_PREFLIGHT_CHECK
 */
PARAM_DEFINE_INT32(A_PFL_EN, 0);

// /**
//  *
//  * Variations of Printing Behavior
//  *
//  * @group AUTO_PREFLIGHT_CHECK
//  * @value 0 - Not Print
//  * @value 1 - Print Case1
//  * @value 2 - Print Case2
//  * @value 3 - Print Case3
//  * @min 0
//  * @max 3
// */
// PARAM_DEFINE_INT32(AUTO_PFL_MT_LOOP, 0);

/**
 * Custom debug Parameter
 *
 * Default Motor Spin time is 1.5 Sec. 
 *
 * @min 0.50
 * @max 3.00
 * @decimal 5
 * @increment 0.001
 * @group AUTO_PREFLIGHT_CHECK
 */
PARAM_DEFINE_FLOAT(A_PFL_MT_TIME, 1.5f);

/**
 *
 * Actuator Testing Case
 *
 * @group AUTO_PREFLIGHT_CHECK
 * @value 0 - Not Print
 * @value 1 - Motor Testing
 * @value 2 - Servo Testing
 * @value 3 - Motor and Servo Testing
 * @min 0
 * @max 3
*/
PARAM_DEFINE_INT32(A_PFL_MODE, 0);