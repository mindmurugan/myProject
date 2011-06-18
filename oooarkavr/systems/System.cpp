/*
 * System.cpp
 * Copyright (C) James Goppert 2010 <jgoppert@users.sourceforge.net>
 *
 * System.cpp is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * System.cpp is distributed in the hope that it will be useful, but
 * WITHgetSerial() ANY WARRANTY; withgetSerial() even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "System.hpp"

namespace oooarkavr
{

System::System(int freq, HardwareSerial & serial) : serial(serial), freq(freq), \
        freqMeas(), timeLast(-100000), subSystems()
{
    float smoothing = 1-0.5/freq; // 2 seconds to settle, start at 0
    if (smoothing < 0) smoothing = 0;
    freqMeas = SmoothScalar<float>(0,smoothing);
}

System::~System()
{
}

void System::addSubSystem(ISystem * subSystem)
{
    subSystems.push_back(subSystem);
}

void System::update()
{
    // update subsystems
    for (int i=0;i<subSystems.getSize();i++) subSystems(i)->update();

    // calculate time elapsed
    float dt = (millis()-timeLast)/1000.;

    // run system if enough time elapsed
    if (dt >= 1./freq)
    {
        //  calculate smoothed frequency
        freqMeas = 1/dt; // smooth frequency and store

        // run system
        run();

        // update time last
        timeLast = millis();
    }
}

void System::printFreqs() const
{
    getSerial().print(freq,4);
    getSerial().print(" (");
    getSerial().print(freqMeas.get(),4);
    getSerial().print(") ");
    for (int i=0;i<subSystems.getSize();i++) subSystems(i)->printFreqs();
    if (subSystems.getSize() > 0) getSerial().println();
}

uint16_t System::getFreq() const
{
    return freq;
}


} // oooarkavr


// vim:ts=4:sw=4
