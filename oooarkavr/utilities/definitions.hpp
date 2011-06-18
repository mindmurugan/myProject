/*
 * definitions.hpp
 * Copyright (C) James Goppert 2010 <jgoppert@users.sourceforge.net>
 *
 * definitions.hpp is free software: you can redistribute it and/or modify it
 * under the terms of thppe GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * definitions.hpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef definitions_hpp
#define definitions_hpp

#include "WProgram.h"
#include "oooarkavr/math/Vector.hpp"

namespace oooarkavr
{
//constants
const float pi = 3.14159265358979323846264338327950288;
const float deg2Rad = 1.745329251994329577e-2;
const float r0 = 6378137; //average radius of earth (MSL)
const static float omega = 7.2921159e-5; // [rad/s] angular velocity of earth
const static float g0 = 9.812865328; // [m/s^2] standard gravitational acceleration of earth
const static float rad2Deg = 57.2957795; // [deg/rad]
// enums
enum Endian {big,little};
enum ImuMessages {setStateMessage,selfCalibrateMessage,alignMessage,
                  setGyroBiasMessage, externalCalibrateMessage
                 };
// unions
union float_uint8
{
    float asFloat;
    uint8_t asUint8[4];
};
union int32_uint8
{
    int32_t asInt32;
    uint8_t asUint8[4];
};
union int16_uint8
{
    int16_t asInt16;
    uint8_t asUint8[2];
};
union uint32_uint8
{
    uint32_t asUint32;
    uint8_t asUint8[4];
};
union floatUnion
{
    float asFloat;
    uint8_t asUint8[4];
};

} // oooarkavr

#endif

// vim:ts=4:sw=4
