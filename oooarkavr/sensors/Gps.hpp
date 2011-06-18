/*
 * Gps.hpp
 * Copyright (C) James Goppert 2010 <jgoppert@users.sourceforge.net>
 * Copyright (C) Brandon Wampler 2010 <bwampler@users.sourceforge.net>
 * Copyright (C) Tony D'Mello 2010 <apdmello@users.sourceforge.net>
 *
 * Gps.hpp is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Gps.hpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef Gps_hpp
#define Gps_hpp

#include "oooarkavr/communication/CommProtocol.hpp"
#include "oooarkavr/systems/System.hpp"
#include "oooarkavr/utilities/definitions.hpp"
#include "oooarkavr/guidance/Guide.hpp"
#include "oooarkavr/sensors/Sensor.hpp"


namespace oooarkavr
{

class Gps : public virtual System
{
public:
    Gps(int freq, CommProtocol * commProtocol);
    virtual ~Gps();
    virtual void run() = 0;
    virtual void initialize() = 0;
    void print(HardwareSerial & out);
    const Vector<int32_t> getGeodetic() const
    {
        return Vector<int32_t> (latitude,longitude,altitudeEllipse);
    }
    Vector<float> getEuler();
    Vector<float> getWN();
    GeodeticCoord getGeodetic();
    Vector<float> getVelNED();
    CommProtocol * getCommProtocol()
    {
        return commProtocol;
    }
protected:
    CommProtocol * commProtocol;
    // variables
    int32_t latitude, longitude, altitudeEllipse, altitudeMSL;
    //Vector<char> time;
    int16_t climbRate, headingRate;
    //uint32_uint8 EHPE, EVPE, distance;
    int16_t SOG, COG, EHPE, EVPE, EHVE, HE;
};

} // oooarkavr

#endif

// vim:ts=4:sw=4
