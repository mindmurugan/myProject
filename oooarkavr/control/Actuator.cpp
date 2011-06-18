/*
 * Actuator.cpp
 * Copyright (C) Brandon Wampler 2010 <bwampler@purdue.edu>
 *
 * Actuator.cpp is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Actuator.cpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "Actuator.hpp"


namespace oooarkavr
{

Actuator::Actuator() : pinIn(), pinOut(), positionMap(), pwmMap(), servo(), pwm(0, 0, 1500)
{
}

Actuator::Actuator(int pinIn, int pinOut, Vector<float> positionMap, Vector<int> pwmMap, Servo * servo, float smoothing, int smoothRange, int initialSmoothValue) :  pinIn(pinIn), pinOut(pinOut), positionMap(positionMap), pwmMap(pwmMap), servo(servo), pwm(smoothing, smoothRange, initialSmoothValue)
{

}

Actuator::Actuator(const Actuator & act) : pinIn(act.pinIn), pinOut(act.pinOut), positionMap(act.positionMap), pwmMap(act.pwmMap), servo(act.servo), pwm(act.pwm)

{
}

Actuator & Actuator::operator=(const Actuator & act)
{
    (*this).pinIn = act.pinIn;
    (*this).pinOut = act.pinOut;
    (*this).positionMap = act.positionMap;
    (*this).pwmMap = act.pwmMap;
    (*this).servo = act.servo;

    return *this;
}

Actuator::~Actuator()
{
}

int Actuator::positionToPwm(float position)
{
    //Serial.print("Position Map: ");
    //positionMap.print();
    //Serial.print("PWM Map: ");
    //Serial.print(pwmMap(0));
    //Serial.println(pwmMap(1));

    if (position > positionMap(positionMap.getSize()-1))
    {
        Serial.print("Position Commanded: ");
        Serial.println(position);

        Serial.print("Position > max, pwm set to max: ");
        Serial.println(pwmMap(positionMap.getSize()-1));
        Serial.println();
        return pwmMap(positionMap.getSize()-1);
    }
    else if (position < positionMap(0))
    {
        Serial.print("Position Commanded: ");
        Serial.println(position);
        Serial.print("Position < min, pwm set to min: ");
        Serial.println(pwmMap(0));
        Serial.println();
        return pwmMap(0);
    }
    else
    {
        int j;
        for (size_t i = 0; i < positionMap.getSize(); i++)
        {
            if (positionMap(i) > position)
            {
                j = i;
                break;
            }
        }
        //y=mx +b
        float m = (pwmMap(j)-pwmMap(j-1)) / (positionMap(j)-positionMap(j-1));
        int b = pwmMap(j-1);
        float x = position - positionMap(j-1);
        //Serial.print("Position Commanded: ");
        //Serial.println(position);
        //Serial.print("PWM Sent ");
        //Serial.println((int) m*x + b);
        //Serial.println();
        return (int) m*x + b;
    }
}

void Actuator::setPosition(float pos)
{
    servo->writeMicroseconds(positionToPwm(pos));
}

void Actuator::setPwm(int pwmIn, bool smooth)
{
    pwm = pwmIn;
    if (smooth == 1)
    {
        servo->writeMicroseconds(pwm.get());
    }
    else
    {
        if (pwmIn < pwmMap(0)) pwmIn = pwmMap(0);
        if (pwmIn > pwmMap(pwmMap.getSize()-1)) pwmIn = pwmMap(pwmMap.getSize()-1);
        servo->writeMicroseconds(pwmIn);
    }

}

} // oooarkavr


// vim:ts=4:sw=4
