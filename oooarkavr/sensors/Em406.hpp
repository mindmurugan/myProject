/*
 * Em406.hpp
 * Copyright (C) James Goppert 2010 <jgoppert@users.sourceforge.net>
 * Copyright (C) Brandon Wampler 2010 <bwampler@users.sourceforge.net>
 * Copyright (C) Tony D'Mello 2010 <apdmello@users.sourceforge.net>
 *
 * Em406.hpp is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Em406.hpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef Em406_hpp
#define Em406_hpp

#include "oooarkavr/communication/Sirf.hpp"
#include "oooarkavr/systems/System.hpp"
#include "oooarkavr/utilities/definitions.hpp"
#include "oooarkavr/guidance/Guide.hpp"
#include "oooarkavr/sensors/Sensor.hpp"
#include "oooarkavr/sensors/Gps.hpp"


namespace oooarkavr
{

class Em406 : public Gps
{
public:
    Em406(int freq, HardwareSerial & serial);
    virtual ~Em406();
    void run();
    void initialize();
private:
    const static uint8_t turnOffAll[];
    const static uint8_t turnOnGeodetic[];
    const static uint8_t nmea2sirf115200[];
    const static uint8_t sirf2nmea[];
    Sirf sirf;
};

} // oooarkavr

#endif

// vim:ts=4:sw=4
