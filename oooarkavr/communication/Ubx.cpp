/*
 * Ubx.cpp
 * Copyright (C) Brandon Wampler 2010 <bwampler@users.sourceforge.net>
 *
 * Ubx.cpp is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Ubx.cpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "Ubx.hpp"

namespace oooarkavr
{



Ubx::Ubx(HardwareSerial & serial) : CommProtocol(serial), CK_A_REC(0), CK_B_REC(0), CK_A(0), CK_B(0), CK_A_Send(0), CK_B_Send(0)

{
    header[0] = 0xB5;
    header[1] = 0x62;
    //footer[0] = 0xB0;
    //footer[1] = 0xB3;
}

Ubx::~Ubx()
{
}

bool Ubx::send(char classID, char messageID)
{
    CK_A_Send = CK_A_Send = 0;
    // compute checksum and get size
    int16_uint8 transmitSize;
    transmitSize.asInt16 = transmitBuffer.getSize();
    Buffer temp;
    temp.push_back(classID);
    temp.push_back(messageID);
    temp.push_back(transmitSize.asUint8[0]);
    temp.push_back(transmitSize.asUint8[1]);
    temp.sumFletcher(CK_A_Send, CK_B_Send);

    transmitBuffer.sumFletcher(CK_A_Send, CK_B_Send);

    //Serial.print("Sum: ");
    //Serial.println(transmitBuffer.sum());
    //Serial.print("Size: ");
    //Serial.println(transmitBuffer.getSize());

    // send message
//[>
    serial.print(header[0],BYTE);
    serial.print(header[1],BYTE);
    serial.print(classID,BYTE);
    serial.print(messageID, BYTE);
    serial.print(transmitSize.asUint8[0],BYTE);
    serial.print(transmitSize.asUint8[1],BYTE);
    for (size_t i=0;i<transmitSize.asInt16;i++) serial.print(transmitBuffer(i),BYTE);
    serial.print(CK_A_Send,BYTE);
    serial.print(CK_B_Send,BYTE);

    transmitBuffer.setSize(0);
    return true;
}

bool Ubx::send()
{
    char classID = 0x06;
    char messageID = 0x00;
    CK_A_Send = CK_B_Send = 0;
    // compute checksum and get size
    int16_uint8 transmitSize;
    transmitSize.asInt16 = transmitBuffer.getSize();
    Buffer temp;
    temp.push_back(classID);
    temp.push_back(messageID);
    temp.push_back(transmitSize.asUint8[0]);
    temp.push_back(transmitSize.asUint8[1]);
    temp.sumFletcher(CK_A_Send, CK_B_Send);

    transmitBuffer.sumFletcher(CK_A_Send, CK_B_Send);

    //Serial.print("Sum: ");
    //Serial.println(transmitBuffer.sum());
    //Serial.print("Size: ");
    //Serial.println(transmitBuffer.getSize());

    // send message
//[>
    serial.print(header[0],BYTE);
    serial.print(header[1],BYTE);
    serial.print(classID,BYTE);
    serial.print(messageID, BYTE);
    serial.print(transmitSize.asUint8[0],BYTE);
    serial.print(transmitSize.asUint8[1],BYTE);
    for (size_t i=0;i<transmitSize.asInt16;i++) serial.print(transmitBuffer(i),BYTE);
    serial.print(CK_A_Send,BYTE);
    serial.print(CK_B_Send,BYTE);

    transmitBuffer.setSize(0);
    return true;
}

bool Ubx::receive()
{
    // get data
    while (serial.available()>0)
    {

        // read
        dataPrev = data;
        data = serial.read();

        // debug data
        //Serial.print(" available: ");
        //Serial.println(serial.available(),DEC);
        //Serial.print(" step: ");
        //Serial.print(step);
        //Serial.print(", data: ");
        //Serial.print(data,HEX);
        //Serial.print(", payload bytes read: ");
        //Serial.print(receiveBuffer.getSize(),DEC);
        //Serial.println();

        // reset step
        if (step <= 0 || step > 9)
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
            classID = data;
            receiveBuffer.push_back(data);
            step = 3;
        }
        else if (step==3)
        {
            messageID = data;
            receiveBuffer.push_back(data);
            step = 4;
        }
        else if (step==4)
        {
            receiveSize.asUint8[0] = data;
            receiveBuffer.push_back(data);
            step=5;
            //Serial.println("\rStep 4 - size low byte - Complete\n");
        }
        else if (step==5)
        {
            receiveSize.asUint8[1] = data;
            receiveBuffer.push_back(data);
            //Serial.println("\rStep 5 - size high byte - Complete\n");
            //Serial.print(" Size: ");
            //Serial.println(receiveSize.asInt16);
            if ((receiveSize.asInt16+8)>127)
            {
                Serial.println("Message Size Greater Than Serial Buffer");
                step =0;
            }
            else
                step=6;
        }
        else if (step==6)
        {
            //+ 4 for classID, messageID, and payload length, +1 since pushback happens after first if() check
            if (receiveBuffer.getSize()+1<=receiveSize.asInt16 + 4)
            {
                receiveBuffer.push_back(data);
                if (receiveBuffer.getSize() == receiveSize.asInt16 + 4)
                {
                    //Serial.print("Payload Size: "), Serial.println(receiveBuffer.getSize(), DEC);
                    //Serial.println("\rStep 6 - read payload for given size - Complete\n");
                    step=7;
                }
            }
            else
                step = 0;
        }
        else if (step==7)
        {
            CK_A = CK_B = 0;
            receiveBuffer.sumFletcher(CK_A, CK_B);
            CK_A_REC = data;
            step=8;
            //Serial.println("\rStep 7 - checksum CK_A_REC - Complete\n");
        }
        else if (step==8)
        {
            CK_B_REC = data;
            step=9;
            //Serial.println("\rStep 8 - checksum CK_B_REC - Complete\n");
        }
        else if (step==9)
        {
            if ((CK_A_REC == CK_A) && (CK_B_REC == CK_B))
            {
                //Serial.println("\rStep 9 - compare checksums - Complete\n");
                step = 0;

                return true;
            }
            else
            {
                Serial.print("CK_A_Rec: ");
                Serial.println(CK_A_REC, HEX);
                Serial.print("CK_B_Rec: ");
                Serial.println(CK_B_REC, HEX);
                Serial.print("CK_A: ");
                Serial.println(CK_A, HEX);
                Serial.print("CK_B: ");
                Serial.println(CK_B, HEX);

                step = 0;
            }
        }
    }
    return false;
}
} // oooarkavr


// vim:ts=4:sw=4
