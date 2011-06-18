/*
 * Actuator.hpp
 * Copyright (C) Brandon Wampler 2010 <bwampler@purdue.edu>
 *
 * Actuator.hpp is free software: you can redistribute it and/or modify it
 * under the terms of thppe GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Actuator.hpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef Actuator_hpp
#define Actuator_hpp

#include "oooarkavr/math/Vector.hpp"
#include "oooarkavr/math/SmoothInt.hpp"
#include "Servo.h"

namespace oooarkavr
{

class Actuator
{
public:
    Actuator();
    Actuator(int pinIn, int pinOut, Vector<float> positionMap, Vector<int> pwmMap, Servo * servo, float smoothing = 0,  int smoothRange = 0, int initialSmoothValue = 1500);
    Actuator(const Actuator & act);
    virtual ~Actuator();
    Actuator & operator=(const Actuator & act);
    int positionToPwm(float position);
    void setPosition(float pos);
    void setPwm(int pwmIn, bool smooth = 1);
    int pinIn, pinOut;
    Vector<float> positionMap;
    Vector<int> pwmMap;
    Servo * servo;
    SmoothInt<int> pwm;

private:

};

} // oooarkavr

#endif

// vim:ts=4:sw=4
