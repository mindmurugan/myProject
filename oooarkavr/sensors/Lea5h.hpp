/*
 * Lea5h.hpp
 * Copyright (C) James Goppert 2010 <jgoppert@users.sourceforge.net>
 * Copyright (C) Brandon Wampler 2010 <bwampler@users.sourceforge.net>
 * Copyright (C) Tony D'Mello 2010 <apdmello@users.sourceforge.net>
 *
 * Lea5h.hpp is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Lea5h.hpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef Lea5h_hpp
#define Lea5h_hpp

#include "oooarkavr/communication/Ubx.hpp"
#include "oooarkavr/systems/System.hpp"
#include "oooarkavr/utilities/definitions.hpp"
#include "oooarkavr/guidance/Guide.hpp"
#include "oooarkavr/sensors/Sensor.hpp"
#include "oooarkavr/sensors/Gps.hpp"

namespace oooarkavr
{

class Lea5h : public Gps
{
public:
    Lea5h(int freq, HardwareSerial & serial);
    virtual ~Lea5h();
    void run();
    void initialize();
private:
    Ubx ubx;
};

} // oooarkavr

#endif

// vim:ts=4:sw=4
