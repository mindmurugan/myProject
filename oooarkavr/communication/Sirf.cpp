/*
 * Sirf.cpp
 * Copyright (C) James Goppert 2010 <jgoppert@users.sourceforge.net>
 *
 * Sirf.cpp is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Sirf.cpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "Sirf.hpp"

namespace oooarkavr
{



Sirf::Sirf(HardwareSerial & serial) : CommProtocol(serial)
{
    header[0] = 0xA0;
    header[1] = 0xA2;
    footer[0] = 0xB0;
    footer[1] = 0xB3;
}

Sirf::~Sirf()
{
}
bool Sirf::send()
{
    // compute checksum and get size
    int16_uint8 transmitSum, transmitSize;
    transmitSum.asInt16 = transmitBuffer.sum();
    transmitSize.asInt16 = transmitBuffer.getSize();
    //Serial.print("Sum: ");
    //Serial.println(transmitBuffer.sum());
    //Serial.print("Size: ");
    //Serial.println(transmitBuffer.getSize());

    // send message
///*
    serial.print(header[0],BYTE);
    serial.print(header[1],BYTE);
    serial.print(transmitSize.asUint8[1],BYTE);
    serial.print(transmitSize.asUint8[0],BYTE);
    for (size_t i=0;i<transmitSize.asInt16;i++) serial.print(transmitBuffer(i),BYTE);
    serial.print(transmitSum.asUint8[1],BYTE);
    serial.print(transmitSum.asUint8[0],BYTE);
    serial.print(footer[0],BYTE);
    serial.print(footer[1],BYTE);

    //Serial.println("send function");
    //serial.print(header[0],HEX);
    //serial.print(header[1],HEX);
    //serial.print(transmitSize.asUint8[1],HEX);
    //serial.print(transmitSize.asUint8[0],HEX);
    //for (size_t i=0;i<transmitSize.asInt16;i++) serial.print(transmitBuffer(i),HEX);
    //serial.print(transmitSum.asUint8[1],HEX);
    //serial.print(transmitSum.asUint8[0],HEX);
    //serial.print(footer[0],HEX);
    //serial.print(footer[1],HEX);

    transmitBuffer.setSize(0);
    return true;
}

bool Sirf::receive()
{
    //Serial.println("At least the function's being called :/");
    //Serial.println(serial.read(),HEX);

    // get data
    while (serial.available()>0)
    {
        // read
        dataPrev = data;
        data = serial.read();

        // debug data
        //Serial.println(" available: ");
        //Serial.print(serial.available(),DEC);
        //Serial.print(" step: ");
        //Serial.print(step);
        //Serial.print(", data: ");
        //Serial.print(data,HEX);
        //Serial.print(" ");
        //Serial.print(", payload bytes read: ");
        //Serial.print(receiveBuffer.getSize(),DEC);
        //Serial.println();

        // reset step
        if (step <= 0 || step >8)
        {
            //Serial.println("\rStep 0 - reset - Complete\n");
            receiveBuffer.setSize(0);
            receiveSize.asInt16 = 0;
            receiveSum.asInt16 = 0;
            step = 1;
        }

        // process packet
        if (data == header[1] && dataPrev == header[0])
        {
            //Serial.println("\rStep 1 - header - Complete\n");
            step = 2;
        }
        else if (step==2)
        {
            receiveSize.asUint8[1] = data;
            step=3;
            //Serial.println("\rStep 2 - size high byte - Complete\n");
        }
        else if (step==3)
        {
            receiveSize.asUint8[0] = data;
            //Serial.println("\rStep 3 - size low byte - Complete\n");
            //Serial.print(" Size: ");
            //Serial.println(receiveSize.asInt16);
            if (receiveSize.asInt16+8>128)
            {
                Serial.println("Message Size Greater Than Serial Buffer");
                step =0;
            }
            else
                step=4;
        }
        else if (step==4)
        {
            if (receiveBuffer.getSize()+1<=receiveSize.asInt16)
            {
                receiveBuffer.push_back(data);
                if (receiveBuffer.getSize() == receiveSize.asInt16)
                {
                    //Serial.println("\rStep 4 - read payload for given size - Complete\n");
                    step=5;
                }
            }
            else
                step = 0;
        }
        else if (step==5)
        {
            receiveSum.asUint8[1] = data;
            step=6;
            //Serial.println("\rStep 5 - checksum high byte - Complete\n");
        }
        else if (step==6)
        {
            receiveSum.asUint8[0] = data;
            step=7;
            //Serial.println("\rStep 6 - checksum low byte - Complete\n");
        }
        else if (step==7)
        {
            if (data == footer[0])
            {
                step=8;
                //Serial.println("\rStep 7 - footer high byte - Complete\n");
            }
            else
                step = 0;
        }
        else if (step==8)
        {
            if (data == footer[1])
            {
                if (receiveBuffer.sum() == receiveSum.asInt16)
                {
                    //Serial.println("\rStep 8 - footer low byte - Complete\n");
                    step = 0;
                    return true;
                }
                else
                {
                    /*Serial.print("Checksum Received: ");
                     Serial.println(receiveSum.asInt16);
                     Serial.print(receiveSum.asUint8[0],HEX);
                     Serial.print(" ");
                     Serial.println(receiveSum.asUint8[1],HEX);
                     Serial.print("Checksum Calculated: ");
                     Serial.println(receiveBuffer.sum());
                     */
                    step = 0;
                }
            }
            else
                step = 0;
        }
    }
    return false;
}
} // oooarkavr


// vim:ts=4:sw=4
