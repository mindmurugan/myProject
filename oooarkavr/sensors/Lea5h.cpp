/*
 * Lea5h.cpp
 * Copyright (C) Brandon Wampler 2010 <bwampler@users.sourceforge.net>
 *
 * Lea5h.cpp is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Lea5h.cpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "Lea5h.hpp"

namespace oooarkavr
{

Lea5h::Lea5h(int freq, HardwareSerial & serial) :  ubx(serial), Gps(freq, &ubx), System(freq)
{
}

Lea5h::~Lea5h()
{
}

void Lea5h::initialize()
{
}

void Lea5h::run()
{
    if (commProtocol->receive())
    {
        uint8_t classID, messageID;
        Serial.println("GPS RECEIVED");
        //commProtocol->printReceiveBuffer(Serial);
        commProtocol->unpack(&classID, 1, 0);
        commProtocol->unpack(&messageID, 1, 1);
        if (classID == 0x01 && messageID == 0x2)
        {
            uint32_t time;
            commProtocol->unpackLsb(time, 4);
            Serial.println(time, DEC);
            commProtocol->unpackLsb(latitude,8); //deg (+North)*10^7
            commProtocol->unpackLsb(longitude,12); //deg (+East)*10^7
            commProtocol->unpackLsb(altitudeEllipse,16); //(31-34. Ellipsoid) meters *10^3
            commProtocol->unpackLsb(altitudeMSL,20); //MSL meters*10^3a
            altitudeMSL = altitudeMSL/10;
            altitudeEllipse /= 10;
        }
        //39. Map Datum
        //commProtocol->unpackLsb(SOG,40); //(Speed Over Ground) m/s *10^2
        //commProtocol->unpackLsb(COG,42); // (Course Over Ground) deg clockwise from true north *10^2
        //44-45. Magnetic Variation
        //commProtocol->unpackLsb(climbRate,46); //m/s *10^2
        //commProtocol->unpackLsb(headingRate,48); //deg/s *10^2
        //commProtocol->unpackLsb(EHPE,50); //Estimated Horrizontal Position Error in m *10^2
        //commProtocol->unpackLsb(EVPE,54); //Estimated Vertical Position Error in m*10^2
        //for (int i=0; i<4; i++) time(i+14) = payload(i+58); //ETE sec *10^2
        //commProtocol->unpackLsb(EHVE,62); //Estimated Horrizontal Velocity Error m/s *10^2
        //for (int i=0; i<16; i++) time(i+18) = payload(i+64); //Clock Bias, error, Clock Drift, error m *10^2 and m/s *10^2
        //for (int i=0; i<4; i++) distance.asUint8[3-i] = payload(i+80); //m
        //for (int i=0; i<2; i++) DE.asUint8[1-i] = payload(i+84); //Distance Error in m
        //commProtocol->unpackLsb(HE,86); //Heading Error in deg*10^2
        //88. Number of SVs in Fix
        //89. Horizontal Dilution of Precision x5 (.2 resolution)
        //90. Additional Mode Info
        //commProtocol->printReceiveBuffer(Serial);
        //print(Serial);
    }
}

} // oooarkavr


// vim:ts=4:sw=4
