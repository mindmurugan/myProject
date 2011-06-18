/*
 * basic.hpp
 * Copyright (C) James Goppert 2010 <jgoppert@users.sourceforge.net>
 *
 * basic.hpp is free software: you can redistribute it and/or modify it
 * under the terms of thppe GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * basic.hpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef basic_hpp
#define basic_hpp

#include "WProgram.h"

namespace oooarkavr
{

void printArray(const char * msg, const float * array, size_t n, HardwareSerial & serial=Serial);
void printScalar(const char * msg, const float & scalar, HardwareSerial & serial=Serial);
void dot(const float *v, const float *w, float & s, const size_t & n);
void norm(const float *v, float & norm, const size_t & n);
void cross(const float *u, const float *v, float *w);
void unit(const float * v, float * unit, const size_t & n);
void quatProd(const float * q, const float * p, float * w);
void quatToDcm(const float * q, float * dcm);
void matrixProd(const float * x, const float * y, float * z, const size_t & m, const size_t & n, const size_t & p);
void matrixVectorProd(const float * x, const float * y, float * z, const size_t & m, const size_t & n);
void quatToEuler(const float * q, float & roll, float & pitch, float & yaw);
void quatRotate(const float * q, const float * v, float * w);
void quatConj(const float * q, float * v);
void axisAngle2Quat(const float * axis, const float & angle, float * q);
void quatVectorAlign(const float * v, const float * w, float * q);


} // oooarkavr

#endif

// vim:ts=4:sw=4
