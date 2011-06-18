/*
 * Timer.hpp
 * Copyright (C) James Goppert 2010 <jgoppert@users.sourceforge.net>
 *
 * Timer.hpp is free software: you can redistribute it and/or modify it
 * under the terms of thppe GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Timer.hpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef Timer_hpp
#define Timer_hpp

#include "WProgram.h"

namespace oooarkavr
{

// class to manage scheduling
class Timer
{
public:
    Timer(const uint32_t & freq);
    virtual ~Timer() {};
    bool isReady();
    uint16_t getFreq();
    uint32_t getPeriod();
private:
    const static uint32_t clockBuffer = 100; // how early to permit run
    // this prevents skipping cycles
    uint32_t period;
    uint32_t periodCurrent;
    uint32_t periodSmooth;
    uint32_t timeLast;
};

} // oooarkavr

#endif

// vim:ts=4:sw=4
