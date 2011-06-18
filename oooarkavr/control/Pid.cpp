/*
 * Pid.cpp
 * Copyright (C) Gihun Bae 2010 <gbae@purdue.edu>
 *
 * Pid.cpp is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Pid.cpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "Pid.hpp"

namespace oooarkavr
{
Pid::Pid(int freq, float maxWindup) : Controller(freq),
        kP(0), kI(0), kD(0), maxWindup(maxWindup), ePrev(0), e(0), eI(0), eD(0)
{
    control.setSize(1);
    command.setSize(1);
    feedback.setSize(1);
}

Pid::~Pid()
{
}

void Pid::run()
{
    float dt = getDt();
    e = command(0) - feedback(0);
    if (dt<=0 || dt>2./getFreq())
    {
        ePrev = e;
        return;
    }
    eI = eI + e*dt;
    if (eI>maxWindup) eI = maxWindup;
    eD = (e - ePrev)/dt;
    ePrev = e;
    control(0) = kP*e + kI*eI + kD*eD;
    Serial.print("dt: "), Serial.print(dt);
    Serial.print(" e: "), Serial.print(e);
    Serial.print(" eI: "), Serial.print(eI);
    Serial.print(" eD: "), Serial.print(eD);
    Serial.print(" ePrev: "), Serial.print(ePrev);
    Serial.print(" command: "), Serial.print(command(0));
    Serial.print(" feedback: "), Serial.print(feedback(0));
    Serial.print(" control: "), Serial.print(control(0));
    Serial.println();
}

void Pid::setGains(float kP, float kI, float kD)
{
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
}

} // oooarkavr


// vim:ts=4:sw=4
