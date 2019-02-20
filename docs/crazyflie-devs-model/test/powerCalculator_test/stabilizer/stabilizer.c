/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
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
 */

#include <math.h>
#include "stabilizer.h"

static uint16_t limitThrust(int32_t value)
{
  if(value > UINT16_MAX)
  {
    value = UINT16_MAX;
  }
  else if(value < 0)
  {
    value = 0;
  }

  return (uint16_t)value;
}

void distributePower_formationX(const uint16_t thrust, const int16_t roll,
                                const int16_t pitch, const int16_t yaw,
                                uint32_t* motorPowerM1, uint32_t* motorPowerM2, 
                                uint32_t* motorPowerM3, uint32_t* motorPowerM4) {

  int16_t r = roll >> 1;
  int16_t p = pitch >> 1;
  *motorPowerM1 = limitThrust(thrust - r + p + yaw);
  *motorPowerM2 = limitThrust(thrust - r - p - yaw);
  *motorPowerM3 =  limitThrust(thrust + r - p + yaw);
  *motorPowerM4 =  limitThrust(thrust + r + p - yaw);
}

void distributePower_normal(const uint16_t thrust, const int16_t roll,
                            const int16_t pitch, const int16_t yaw,
                            uint32_t* motorPowerM1, uint32_t* motorPowerM2, 
                            uint32_t* motorPowerM3, uint32_t* motorPowerM4) {

  int16_t r = roll >> 1;
  int16_t p = pitch >> 1;
  *motorPowerM1 = limitThrust(thrust + pitch + yaw);
  *motorPowerM2 = limitThrust(thrust - roll - yaw);
  *motorPowerM3 =  limitThrust(thrust - pitch + yaw);
  *motorPowerM4 =  limitThrust(thrust + roll - yaw);
}