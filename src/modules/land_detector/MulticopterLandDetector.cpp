/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file MulticopterLandDetector.cpp
 * Land detection algorithm for multicopters
 *
 * @author Johan Jansen <jnsn.johan@gmail.com>
 * @author Morten Lysgaard <morten@lysgaard.no>
 */

#include "MulticopterLandDetector.h"

#include <cmath>
#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>

MulticopterLandDetector::MulticopterLandDetector() : LandDetector(),
	_paramHandle(),
	_params(),
#if __LAND_FIX__
	_vehicleLocalSub(-1),
#else
	_vehicleGlobalPositionSub(-1),
#endif/*__LAND_FIX__*/
	_vehicleStatusSub(-1),
	_actuatorsSub(-1),
	_armingSub(-1),
	_parameterSub(-1),
	_attitudeSub(-1),
#if __LAND_FIX__
	_update(0),
	_vehicleLocal{},
#else/*__LAND_FIX__*/
	_vehicleGlobalPosition{},
#endif/*__LAND_FIX__*/
	_vehicleStatus{},
	_actuators{},
	_arming{},
	_vehicleAttitude{},
	_landTimer(0)
{
	_paramHandle.maxRotation = param_find("LNDMC_ROT_MAX");
	_paramHandle.maxVelocity = param_find("LNDMC_XY_VEL_MAX");
	_paramHandle.maxClimbRate = param_find("LNDMC_Z_VEL_MAX");
	_paramHandle.maxThrottle = param_find("LNDMC_THR_MAX");
}

void MulticopterLandDetector::initialize()
{
	// subscribe to position, attitude, arming and velocity changes
#if __LAND_FIX__
	_vehicleLocalSub = orb_subscribe(ORB_ID(vehicle_local_position));
#else
	_vehicleGlobalPositionSub = orb_subscribe(ORB_ID(vehicle_global_position));
#endif/*__LAND_FIX__*/
	_attitudeSub = orb_subscribe(ORB_ID(vehicle_attitude));
	_vehicleStatusSub = orb_subscribe(ORB_ID(vehicle_status));
	_actuatorsSub = orb_subscribe(ORB_ID_VEHICLE_ATTITUDE_CONTROLS);
	_armingSub = orb_subscribe(ORB_ID(actuator_armed));
	_parameterSub = orb_subscribe(ORB_ID(parameter_update));

	// download parameters
	updateParameterCache(true);
}

void MulticopterLandDetector::updateSubscriptions()
{
#if __LAND_FIX__
	orb_check(_vehicleLocalSub,&_update);
	if(_update){
		orb_copy(ORB_ID(vehicle_local_position),_vehicleLocalSub, &_vehicleLocal);
		if(!_vehicleLocal.xy_valid){
			_vehicleLocal.vx = 0;
			_vehicleLocal.vy = 0;
		}
	}
#else/*__LAND_FIX__*/
	orb_update(ORB_ID(vehicle_global_position), _vehicleGlobalPositionSub, &_vehicleGlobalPosition);
#endif/*__LAND_FIX__*/
	orb_update(ORB_ID(vehicle_attitude), _attitudeSub, &_vehicleAttitude);
	orb_update(ORB_ID(vehicle_status), _vehicleStatusSub, &_vehicleStatus);
	orb_update(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, _actuatorsSub, &_actuators);
	orb_update(ORB_ID(actuator_armed), _armingSub, &_arming);
}

bool MulticopterLandDetector::update()
{
	// first poll for new data from our subscriptions
	updateSubscriptions();

	updateParameterCache(false);

	return get_landed_state();
}

bool MulticopterLandDetector::get_landed_state()
{
	// only trigger flight conditions if we are armed
	if (!_arming.armed) {
		_arming_time = 0;
		return true;

	} else if (_arming_time == 0) {
		_arming_time = hrt_absolute_time();
	}
#if __LAND_FIX__
#else
	// return status based on armed state if no position lock is available
	if (_vehicleGlobalPosition.timestamp == 0 ||
	    hrt_elapsed_time(&_vehicleGlobalPosition.timestamp) > 500000) {

		// no position lock - not landed if armed
		return !_arming.armed;
	}
#endif/*__LAND_FIX__*/
	const uint64_t now = hrt_absolute_time();

	float armThresholdFactor = 1.0f;

	// Widen acceptance thresholds for landed state right after arming
	// so that motor spool-up and other effects do not trigger false negatives
	if (hrt_elapsed_time(&_arming_time) < LAND_DETECTOR_ARM_PHASE_TIME) {
		armThresholdFactor = 2.5f;
	}

	// check if we are moving vertically - this might see a spike after arming due to
	// throttle-up vibration. If accelerating fast the throttle thresholds will still give
	// an accurate in-air indication
#if __LAND_FIX__

		// check if we are moving vertically
	bool verticalMovement = fabsf(_vehicleLocal.vz) > _params.maxClimbRate;
	// check if we are moving horizontally
	bool horizontalMovement = sqrtf(_vehicleLocal.vx * _vehicleLocal.vx + _vehicleLocal.vy * _vehicleLocal.vy) > _params.maxVelocity;
#else/*__LAND_FIX__*/
	bool verticalMovement = fabsf(_vehicleGlobalPosition.vel_d) > _params.maxClimbRate * armThresholdFactor;

	// check if we are moving horizontally
	bool horizontalMovement = sqrtf(_vehicleGlobalPosition.vel_n * _vehicleGlobalPosition.vel_n
					+ _vehicleGlobalPosition.vel_e * _vehicleGlobalPosition.vel_e) > _params.maxVelocity
				  && _vehicleStatus.condition_global_position_valid;
#endif/*__LAND_FIX__*/

	// next look if all rotation angles are not moving
	float maxRotationScaled = _params.maxRotation * armThresholdFactor;

	bool rotating = (fabsf(_vehicleAttitude.rollspeed)  > maxRotationScaled) ||
			(fabsf(_vehicleAttitude.pitchspeed) > maxRotationScaled) ||
			(fabsf(_vehicleAttitude.yawspeed) > maxRotationScaled);

	// check if thrust output is minimal (about half of default)
	bool minimalThrust = _actuators.control[3] <= _params.maxThrottle;

	if (verticalMovement || rotating || !minimalThrust || horizontalMovement) {
		// sensed movement, so reset the land detector
		_landTimer = now;
		return false;
	}

	return now - _landTimer > LAND_DETECTOR_TRIGGER_TIME;
}

void MulticopterLandDetector::updateParameterCache(const bool force)
{
	bool updated;
	parameter_update_s paramUpdate;

	orb_check(_parameterSub, &updated);

	if (updated) {
		orb_copy(ORB_ID(parameter_update), _parameterSub, &paramUpdate);
	}

	if (updated || force) {
		param_get(_paramHandle.maxClimbRate, &_params.maxClimbRate);
		param_get(_paramHandle.maxVelocity, &_params.maxVelocity);
		param_get(_paramHandle.maxRotation, &_params.maxRotation);
		_params.maxRotation = math::radians(_params.maxRotation);
		param_get(_paramHandle.maxThrottle, &_params.maxThrottle);
	}
}
