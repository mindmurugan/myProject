/*
 * Timer.cpp
 * Copyright (C) James Goppert 2010 <jgoppert@users.sourceforge.net>
 *
 * Timer.cpp is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Timer.cpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "Timer.hpp"

namespace oooarkavr
{

Timer::Timer(const uint32_t & freq) :
        period(1000000/freq), periodCurrent(period),
        periodSmooth(period), timeLast(0)
{
}

bool Timer::isReady()
{
    uint32_t time = micros();
    if (time<timeLast)
    {
        return false;
    }
    periodCurrent=time-timeLast;
    if (periodCurrent + clockBuffer >= period)
    {
        timeLast=time;
        periodSmooth=.9*periodSmooth + .1*periodCurrent;
        return true;
    }
    return false;
}
uint16_t Timer::getFreq()
{
    return round(1000000./periodSmooth);
}
uint32_t Timer::getPeriod()
{
    return period;
}

} // oooarkavr


// vim:ts=4:sw=4
