/**
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * pid.c - implementation of the PID regulator
 */
#include <math.h>
#include "pid.h"
#include <iostream>

using namespace std;

void pidInit(PidObject* pid, const float desired, const float kp,
             const float ki, const float kd, const float dt) {
  pid->error     = 0;
  pid->prevError = 0;
  pid->integ     = 0;
  pid->deriv     = 0;
  pid->desired = desired;
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->iLimit    = DEFAULT_PID_INTEGRATION_LIMIT;
  pid->iLimitLow = -DEFAULT_PID_INTEGRATION_LIMIT;
  pid->dt        = dt;
}

float pidUpdate(PidObject* pid, const float measured, const bool updateError) {
    float output;
    if (updateError)
    {
        pid->error = pid->desired - measured;
    }
    //cout << " error: " << pid->error << endl;
    //cout << " dt: " << pid->dt << endl;
    
    pid->integ += pid->error * pid->dt;
    if (pid->integ > pid->iLimit)
    {
        pid->integ = pid->iLimit;
    }
    else if (pid->integ < pid->iLimitLow)
    {
        pid->integ = pid->iLimitLow;
    }
    //cout << " integral: " << pid->integ << endl;

    pid->deriv = (pid->error - pid->prevError) / pid->dt;
    //cout << " derivate: " << pid->deriv << endl;

    pid->outP = pid->kp * pid->error;
    pid->outI = pid->ki * pid->integ;
    pid->outD = pid->kd * pid->deriv;

    output = pid->outP + pid->outI + pid->outD;

    pid->prevError = pid->error;
    //cout << "pid: " << output << endl;
    return output;
}

void pidSetIntegralLimit(PidObject* pid, const float limit) {
    pid->iLimit = limit;
}


void pidSetIntegralLimitLow(PidObject* pid, const float limitLow) {
    pid->iLimitLow = limitLow;
}

void pidReset(PidObject* pid) {
  pid->error     = 0;
  pid->prevError = 0;
  pid->integ     = 0;
  pid->deriv     = 0;
}

void pidSetError(PidObject* pid, const float error) {
  pid->error = error;
}

void pidSetDesired(PidObject* pid, const float desired) {
  pid->desired = desired;
}

float pidGetDesired(PidObject* pid) {
  return pid->desired;
}

bool pidIsActive(PidObject* pid) {
  bool isActive = true;

  if (pid->kp < 0.0001 && pid->ki < 0.0001 && pid->kd < 0.0001) {
    isActive = false;
  }

  return isActive;
}

void pidSetKp(PidObject* pid, const float kp) {
  pid->kp = kp;
}

void pidSetKi(PidObject* pid, const float ki) {
  pid->ki = ki;
}

void pidSetKd(PidObject* pid, const float kd) {
  pid->kd = kd;
}

void pidSetDt(PidObject* pid, const float dt) {
    pid->dt = dt;
}

void controllerCorrectAttitudePID(
      PidObject* pidRoll, PidObject* pidPitch, PidObject* pidYaw,
      float eulerRollActual, float eulerPitchActual, float eulerYawActual,
      float eulerRollDesired, float eulerPitchDesired, float eulerYawDesired,
      float* rollRateDesired, float* pitchRateDesired, float* yawRateDesired)
{
  pidSetDesired(pidRoll, eulerRollDesired);
  *rollRateDesired = pidUpdate(pidRoll, eulerRollActual, true);

  // Update PID for pitch axis
  pidSetDesired(pidPitch, eulerPitchDesired);
  *pitchRateDesired = pidUpdate(pidPitch, eulerPitchActual, true);

  // Update PID for yaw axis
  float yawError;
  yawError = eulerYawDesired - eulerYawActual;
  if (yawError > 180.0)
    yawError -= 360.0;
  else if (yawError < -180.0)
    yawError += 360.0;
  pidSetError(pidYaw, yawError);
  *yawRateDesired = pidUpdate(pidYaw, eulerYawActual, false);
}


void controllerCorrectRatePID(PidObject* pidRollRate, PidObject* pidPitchRate, PidObject* pidYawRate,
                              float rollRateActual, float pitchRateActual, float yawRateActual,
                              float rollRateDesired, float pitchRateDesired, float yawRateDesired,
                              int16_t* rollOutput, int16_t* pitchOutput, int16_t* yawOutput)
{
  //cout << "ctrl roll ";
  //cout << "ctrl desired: " << rollRateDesired << " ctrl actual: " << rollRateActual << endl;
  pidSetDesired(pidRollRate, rollRateDesired);
  *rollOutput = saturateSignedInt16(pidUpdate(pidRollRate, rollRateActual, true));

  //cout << "ctrl pitch ";
  //cout << "ctrl desired: " << pitchRateDesired << " ctrl actual: " << pitchRateActual << endl;
  pidSetDesired(pidPitchRate, pitchRateDesired);
  *pitchOutput = saturateSignedInt16(pidUpdate(pidPitchRate, pitchRateActual, true));

  //cout << "ctrl yaw ";
  //cout << "ctrl desired: " << yawRateDesired << " ctrl actual: " << yawRateActual << endl;
  pidSetDesired(pidYawRate, yawRateDesired);
  *yawOutput = saturateSignedInt16(pidUpdate(pidYawRate, yawRateActual, true));
}

int16_t saturateSignedInt16(float in) {
  // don't use INT16_MIN, because later we may negate it, which won't work for that value.
  if      (in > INT16_MAX)  return INT16_MAX;
  else if (in < -INT16_MAX) return -INT16_MAX;
  else                      return (int16_t)in;
}