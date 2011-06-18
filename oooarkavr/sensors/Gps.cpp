/*
 * Gps.cpp
 * Copyright (C) James Goppert 2010 <jgoppert@users.sourceforge.net>
 *
 * Gps.cpp is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Gps.cpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "Gps.hpp"

namespace oooarkavr
{

Gps::Gps(int freq, CommProtocol * commProtocol) : System(freq), commProtocol(commProtocol), latitude(), longitude(), altitudeEllipse(), altitudeMSL(), climbRate(), headingRate(), SOG(), COG(), EHPE(), EVPE(), EHVE(), HE()
{
}

Gps::~Gps()
{
}

Vector<float> Gps::getEuler()
{
    Vector<float> eul(3);
    eul(0) = 0;
    eul(1) = 0;
    eul(2) = deg2Rad*COG/1e2;
    return eul;
}

Vector<float> Gps::getWN()
{
    static int16_t lastHeading = deg2Rad*COG/1e2;
    Vector<float> wn(3);
    wn(0) = 0;
    wn(1) = 0;
    wn(2) = deg2Rad*COG/1e2 - deg2Rad*lastHeading/1e2;
    return wn;
}

Vector<float> Gps::getVelNED()
{
    Vector<float> vel(3);
    vel(0) = cos(COG/1e2*deg2Rad)*SOG/1e2;
    vel(1) = sin(COG/1e2*deg2Rad)*SOG/1e2;
    vel(2) = -climbRate/1e2;
    return vel;
}

GeodeticCoord Gps::getGeodetic()
{
    GeodeticCoord p;
    p.set(latitude, longitude, altitudeMSL);
    return p;
}

void Gps::print(HardwareSerial & out)
{
    out.println();
    out.println("Gps Data");
    out.print("Latitude(deg 1e7): "), out.println(latitude);
    out.print("Longitude(deg 1e7): "), out.println(longitude);
    out.print("Altitude MSL(m): "), out.println(altitudeMSL/1e2);
    out.print("Speed over ground(m/s): "), out.println(SOG/1e2);
    out.print("ClimbRate(m/s): "), out.println(climbRate/1e2);
    out.print("Course over ground(deg): "), out.println(COG/1e2);
    out.print("Pos Error(m): "), out.println(EHPE/1e2);
    out.print("Alt Error(m): "), out.println(EVPE/1e2);
    out.print("Vel Error(m/s): "), out.println(EHVE/1e2);
    out.print("Heading Error(deg): "), out.println(HE/1e2);
    out.println();
}

} // oooarkavr


// vim:ts=4:sw=4
