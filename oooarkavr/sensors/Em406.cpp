/*
 * Em406.cpp
 * Copyright (C) James Goppert 2010 <jgoppert@users.sourceforge.net>
 *
 * Em406.cpp is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Em406.cpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "Em406.hpp"

namespace oooarkavr
{

const uint8_t Em406::turnOffAll[] = {0xA6,0x02,0x00,0x00,0x00,0x00,0x00,0x00};
const uint8_t Em406::turnOnGeodetic[] = {0xA6,0x00,0x29,0x01,0x00,0x00,0x00,0x00};
const uint8_t Em406::nmea2sirf115200[]="$PSRF100,0,115200,8,1,0*04\r\n";
const uint8_t Em406::sirf2nmea[]={0x81,0x02,0x01,0x01,0x00,0x01,0x01,0x01,0x05,0x01,0x01,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x25,0x80};

Em406::Em406(int freq, HardwareSerial & serial) : sirf(serial), Gps(freq, &sirf), System(freq)
{
}

Em406::~Em406()
{
}

void Em406::initialize()
{
    // set gps to sirf protocol

    // try 4800 baud
    commProtocol->serial.begin(4800);
    commProtocol->serial.print((char *)nmea2sirf115200);
    delay(300);

    // try 9600 baud
    commProtocol->serial.begin(9600);
    commProtocol->serial.print((char *)nmea2sirf115200);
    delay(300);

    // connect to gps
    commProtocol->serial.begin(115200);
    delay(300);

    // send messages
    commProtocol->pack(turnOffAll,sizeof(turnOffAll));
    commProtocol->send();
    commProtocol->pack(turnOnGeodetic,sizeof(turnOnGeodetic));
    commProtocol->send();
}

void Em406::run()
{
    if (commProtocol->receive())
    {
        Serial.println("GPS RECEIVED");
        //commProtocol->printReceiveBuffer(Serial);
        //we can ignore the first few bytes
        //0. Message Id (should be 0x29)
        //1-2. Valid navigation (should be 0 if everything worked out)
        //3-4. Type of navigation
        //for (int i=0; i<14; i++) time(i) = payload(i+5);
        //end for
        //19-22. Satellite ID
        commProtocol->unpackMsb(latitude,23); //deg (+North)*10^7
        commProtocol->unpackMsb(longitude,27); //deg (+East)*10^7
        commProtocol->unpackMsb(altitudeEllipse,31); //(31-34. Ellipsoid) meters *10^2
        commProtocol->unpackMsb(altitudeMSL,35); //MSL meters*10^2
        //39. Map Datum
        commProtocol->unpackMsb(SOG,40); //(Speed Over Ground) m/s *10^2
        commProtocol->unpackMsb(COG,42); // (Course Over Ground) deg clockwise from true north *10^2
        //44-45. Magnetic Variation
        commProtocol->unpackMsb(climbRate,46); //m/s *10^2
        commProtocol->unpackMsb(headingRate,48); //deg/s *10^2
        commProtocol->unpackMsb(EHPE,50); //Estimated Horrizontal Position Error in m *10^2
        commProtocol->unpackMsb(EVPE,54); //Estimated Vertical Position Error in m*10^2
        //for (int i=0; i<4; i++) time(i+14) = payload(i+58); //ETE sec *10^2
        commProtocol->unpackMsb(EHVE,62); //Estimated Horrizontal Velocity Error m/s *10^2
        //for (int i=0; i<16; i++) time(i+18) = payload(i+64); //Clock Bias, error, Clock Drift, error m *10^2 and m/s *10^2
        //for (int i=0; i<4; i++) distance.asUint8[3-i] = payload(i+80); //m
        //for (int i=0; i<2; i++) DE.asUint8[1-i] = payload(i+84); //Distance Error in m
        commProtocol->unpackMsb(HE,86); //Heading Error in deg*10^2
        //88. Number of SVs in Fix
        //89. Horizontal Dilution of Precision x5 (.2 resolution)
        //90. Additional Mode Info
        //commProtocol->printReceiveBuffer();
        //print(Serial);
    }
}

} // oooarkavr


// vim:ts=4:sw=4
