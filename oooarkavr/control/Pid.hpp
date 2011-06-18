/*
 * Pid.hpp
 * Copyright (C) Gihun Bae 2010 <gbae@purdue.edu>
 *
 * Pid.hpp is free software: you can redistribute it and/or modify it
 * under the terms of thppe GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Pid.hpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef Pid_hpp
#define Pid_hpp

#include "oooarkavr/control/Controller.hpp"

namespace oooarkavr
{

class Pid : public Controller
{
public:
    Pid(int freq, float maxWindup);
    virtual ~Pid();
    virtual void run();
    void setGains(float kP, float kI, float kD);
    float kP, kI, kD; // prop./ integ./ deriv. gains
    float maxWindup; // max value of integrator
    float ePrev; // previous error
    float e; // error
    float eI; // integral of error
    float eD; // derivative of error
};

} // oooarkavr

#endif

// vim:ts=4:sw=4
