/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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
 * @file auto_preflight_check_params.c
 * Parameters for custom auto_preflight_check_params Module
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
PARAM_DEFINE_INT32(AT_PFL_SV_LOOP, 2);


/**
 * Custom INT for AUTO_PREFLIGHT_CHECK Boolean
 *
 * 0 for Disable
 * 1 for Enable
 *
 * @boolean
 * @group AUTO_PREFLIGHT_CHECK
 */
PARAM_DEFINE_INT32(AT_PFL_EN, 0);

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
PARAM_DEFINE_FLOAT(AT_PFL_MT_TIME, 1.5f);

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
PARAM_DEFINE_INT32(AT_PFL_MODE, 0);

/**
 * Custom INT for AUTO_PREFLIGHT_CHECK Boolean
 *
 * 0 for Disable
 * 1 for Enable
 *
 * @boolean
 * @group AUTO_PREFLIGHT_CHECK
 */
PARAM_DEFINE_INT32(APP_AT_PFL_EN, 0);