/*
 * System.hpp
 * Copyright (C) James Goppert 2010 <jgoppert@users.sourceforge.net>
 *
 * System.hpp is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * System.hpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef System_hpp
#define System_hpp

#include "oooarkavr/math/Vector.hpp"
#include "oooarkavr/math/SmoothScalar.hpp"
#include "oooarkavr/utilities/definitions.hpp"

namespace oooarkavr
{

// system interface
class ISystem
{
public:
    virtual ~ISystem() {};
    virtual void update() = 0;
    virtual void run() = 0;
    virtual void printFreqs() const = 0;
    virtual uint16_t getFreq() const = 0;
};

// system class
class System : public virtual ISystem
{
public:
    System(int freq, HardwareSerial & serial=Serial);
    virtual ~System();
    void addSubSystem(ISystem * subSystem);
    virtual void update();
    virtual void run() = 0;
    void printFreqs() const;
    virtual float getDt() const
    {
        return (millis()-timeLast)/1000.;
    }
    HardwareSerial & getSerial() const
    {
        return serial;
    }
    virtual uint16_t getFreq() const;
private:
    HardwareSerial & serial;
    uint16_t freq;
    SmoothScalar<float> freqMeas;
    long int timeLast;
    Vector<ISystem *> subSystems;
};

} // oooarkavr

#endif

// vim:ts=4:sw=4
