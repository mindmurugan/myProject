/*
 * ServoManager.cpp
 * Copyright (C) Murugan Palaniappan 2010 <mpalania@purdue.edu>
 * Copyright (C) Jeremy Moon 2010 <jamoon@purdue.edu>
 *
 * ServoManager.cpp is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ServoManager.cpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "ServoManager.hpp"

namespace oooarkavr
{

ServoManager::ServoManager(const int & freq,const int & controlModePinIn,Controller * controller, Vector<Actuator*> * actuators) :
        System(freq), controlModePinIn(controlModePinIn), mode(MANUAL), control(), controller(controller), actuators(actuators)
{
}

void ServoManager::setupActuators()
{
    for (size_t i = 0; i < actuators->getSize(); i++)
    {
        pinMode((*actuators)(i)->pinIn, INPUT);
        (*actuators)(i)->servo->attach((*actuators)(i)->pinOut, (*actuators)(i)->pwmMap(0), \
                                       (*actuators)(i)->pwmMap((*actuators)(i)->pwmMap.getSize()));
    }
    pinMode(controlModePinIn,INPUT);

}

ServoManager::~ServoManager()
{
}

void ServoManager::run()
{
    int pulseWidthControlMode = pulseIn(controlModePinIn, HIGH,50000);

    if (pulseWidthControlMode > 1800)
    {
        mode = AUTO;
        //Serial.println("Autopilot has been activated, Gear 0");

        Vector<float> control = controller->getControl();
        //Serial.print("control size: ");
        //Serial.println(control.getSize());
        //Serial.print("actuators size: ");
        //Serial.println(actuators->getSize());

        if (actuators->getSize() != control.getSize()) Serial.println("Error:actuators size and and control size not equal");
        for (size_t i = 0; i< actuators->getSize(); i++)
        {

            //Serial.print("Actuator ");
            //Serial.println(i);
            //Serial.println((*actuators)(i)->positionToPwm(control(i)));
            (*actuators)(i)->setPosition(control(i));
        }
    }
    else
    {
        mode = MANUAL;
        int pwm = 0;
        //Serial.println("Manual Control, Gear 1 ");
        for (size_t i = 0; i< actuators->getSize(); i++)
        {
            pwm = pulseIn((*actuators)(i)->pinIn, HIGH,50000);
            if (pwm == 0) (*actuators)(i)->setPwm(1500);
            else (*actuators)(i)->setPwm(pwm);
            //Serial.print(pwm);
            //Serial.print("    ");
        }
        //Serial.println();
    }
}



} // oooarkavr


// vim:ts=4:sw=4
