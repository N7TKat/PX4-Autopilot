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
 * @file work_item_autostart_params.c
 * Parameters for custom work_item_autostart Module
 *
 * @author Thanaphat Khattawee <lorenz@px4.io>
 **/

/**
 * Custom debug Parameter
 *
 * The Base of Literature Multiplier
 *
 * @min 0.01
 * @max 0.99
 * @decimal 5
 * @increment 0.001
 * @group tutorials
 */
PARAM_DEFINE_FLOAT(WORK_AUTO_MU, 0.15f);

/**
 * Custom INT for AutoStart Boolean
 *
 * 0 for Disable
 * 1 for Enable
 *
 * @boolean
 * @group tutorials
 */
PARAM_DEFINE_INT32(WORK_AUTO_EN, 0);

/**
 *
 * Variations of Printing Behavior
 *
 * @group tutorials
 * @value 0 - Not Print
 * @value 1 - Print Case1
 * @value 2 - Print Case2
 * @value 3 - Print Case3
 * @min 0
 * @max 3
*/
PARAM_DEFINE_INT32(WORK_AUTO_MODE, 0);
