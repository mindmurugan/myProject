/*
 * ServoManager.hpp
 * Copyright (C) Murugan Palaniappan 2010 <mpalania@purdue.edu>
 * Copyright (C) Jeremy Moon <jamoon@purdue.edu>
 *
 * ServoManager.hpp is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ServoManager.hpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef ServoManager_hpp
#define ServoManager_hpp

#include "Servo.h"
#include "oooarkavr/math/Vector.hpp"
#include "oooarkavr/math/SmoothScalar.hpp"
#include "oooarkavr/systems/System.hpp"
#include "oooarkavr/control/Controller.hpp"
#include "oooarkavr/control/Actuator.hpp"

namespace oooarkavr
{

class ServoManager : public System
{
public:
    enum controlMode {AUTO, MANUAL};
    ServoManager(const int & freq, const int & controlModeInPin, Controller * controller,  Vector<Actuator*> * actuators);
    virtual ~ServoManager();
    void setupActuators();
    virtual void run();
private:
    // constants
    int controlModePinIn;
    // variables
    controlMode mode;
    Vector<float> control;
    // controller
    Controller * controller;

    // servos
    Vector<Actuator*> * actuators;
};


} // oooarkavr

#endif

// vim:ts=4:sw=4
