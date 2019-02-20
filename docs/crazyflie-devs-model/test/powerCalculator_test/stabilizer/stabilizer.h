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
 */
#ifndef STABALIZER_H_
#define STABALIZER_H_

#include <stdbool.h>
#include <cstdint> // This is to use int16_t, uint32_t, etc

void distributePower_formationX(const uint16_t thrust, const int16_t roll,
                            	const int16_t pitch, const int16_t yaw,
                            	uint32_t* motorPowerM1, uint32_t* motorPowerM2, 
                            	uint32_t* motorPowerM3, uint32_t* motorPowerM4);

void distributePower_normal(const uint16_t thrust, const int16_t roll,
                            const int16_t pitch, const int16_t yaw,
                            uint32_t* motorPowerM1, uint32_t* motorPowerM2, 
                            uint32_t* motorPowerM3, uint32_t* motorPowerM4);


#endif /* STABALIZER_H_ */
