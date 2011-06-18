/*
 * CommProtocol.cpp
 * Copyright (C) Brandon Wampler 2010 <bwampler@users.sourceforge.net>
 *
 * CommProtocol.cpp is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * CommProtocol.cpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "CommProtocol.hpp"

namespace oooarkavr
{

CommProtocol::CommProtocol(HardwareSerial & serial) :
        serial(serial), receiveBuffer(size_t(0),200), transmitBuffer(),
        step(0), receiveSize(), receiveSum(), data(0), dataPrev(0)
{
}

CommProtocol::~CommProtocol()
{
}

// pack/unpack a raw buffer
void CommProtocol::pack(const uint8_t * buffer, const size_t & length)
{
    for (size_t i=0;i<length;i++)
        transmitBuffer.push_back(buffer[i]);
}


void CommProtocol::packZeros(const size_t & length)
{//adds zeros to send buffer.
    const uint8_t buffer = 0;
    for (size_t i=0;i<length;i++)
        pack(&buffer,1);
}//end packZeros

void CommProtocol::unpack(uint8_t * buffer, const size_t & length, const size_t & pos)
{
    for (size_t i=0;i<length;i++)
        buffer[i] = receiveBuffer(pos+i);
}

// pack least significant bit first, little endian
void CommProtocol::packLsb(const float & val)
{
    for (size_t i=0;i<4;i++)
        transmitBuffer.push_back(((uint8_t *)(&val))[i]);
}

void CommProtocol::packLsb(const int16_t & val)
{
    for (size_t i=0;i<2;i++)
        transmitBuffer.push_back(((uint8_t *)(&val))[i]);
}

void CommProtocol::packLsb(const int32_t & val)
{
    for (size_t i=0;i<4;i++)
        transmitBuffer.push_back(((uint8_t *)(&val))[i]);
}


// pack most significant bit first, big endian
void CommProtocol::packMsb(const float & val)
{
    for (size_t i=0;i<4;i++)
        transmitBuffer.push_back(((uint8_t *)(&val))[3-i]);
}

void CommProtocol::packMsb(const int16_t & val)
{
    for (size_t i=0;i<2;i++)
        transmitBuffer.push_back(((uint8_t *)(&val))[1-i]);
}
void CommProtocol::packMsb(const  uint16_t & val)
{
    for (size_t i=0;i<2;i++)
        transmitBuffer.push_back(((uint8_t *)(&val))[1-i]);
}


void CommProtocol::packMsb(const int32_t & val)
{
    for (size_t i=0;i<4;i++)
        transmitBuffer.push_back(((uint8_t *)(&val))[3-i]);
}

void CommProtocol::packMsb(const uint32_t & val)
{
    for (size_t i=0;i<4;i++)
        transmitBuffer.push_back(((uint8_t *)(&val))[3-i]);
}

// unpack least significant bit first, little endian
void CommProtocol::unpackLsb(float & val, const size_t & pos)
{
    float_uint8 data;
    for (size_t i=0;i<4;i++)
        data.asUint8[i] = receiveBuffer(i+pos);
    val = data.asFloat;
}

void CommProtocol::unpackLsb(int16_t & val, const size_t & pos)
{
    int16_uint8 data;
    for (size_t i=0;i<2;i++)
        data.asUint8[i] = receiveBuffer(i+pos);
    val = data.asInt16;
}

void CommProtocol::unpackLsb(int32_t & val, const size_t & pos)
{
    int32_uint8 data;
    for (size_t i=0;i<4;i++)
        data.asUint8[i] = receiveBuffer(i+pos);
    val = data.asInt32;
}

void CommProtocol::unpackLsb(uint32_t & val, const size_t & pos)
{
    uint32_uint8 data;
    for (size_t i=0;i<4;i++)
        data.asUint8[i] = receiveBuffer(i+pos);
    val = data.asUint32;
}

// unpack most significant bit first, big endian
void CommProtocol::unpackMsb(int16_t & val, const size_t & pos)
{
    int16_uint8 data;
    for (size_t i=0;i<2;i++)
        data.asUint8[i] = receiveBuffer(1-i+pos);
    val = data.asInt16;
}

void CommProtocol::unpackMsb(int32_t & val, const size_t & pos)
{
    int32_uint8 data;
    for (size_t i=0;i<4;i++)
        data.asUint8[i] = receiveBuffer(3-i+pos);
    val = data.asInt32;
}

void CommProtocol::unpackMsb(uint32_t & val, const size_t & pos)
{
    uint32_uint8 data;
    for (size_t i=0;i<4;i++)
        data.asUint8[i] = receiveBuffer(3-i+pos);
    val = data.asUint32;
}

void CommProtocol::unpackMsb(float & val, const size_t & pos)
{
    float_uint8 data;
    for (size_t i=0;i<4;i++)
        data.asUint8[i] = receiveBuffer(3-i+pos);
    val = data.asFloat;
}

void CommProtocol::printReceiveBuffer(HardwareSerial & serial)
{
    for (size_t i = 0; i < receiveSize.asInt16; i++)
    {
        serial.print(receiveBuffer(i), HEX);
        serial.print(" ");
    }
    Serial.println();
}

void CommProtocol::printTransmitBuffer(HardwareSerial & serial)
{
    for (size_t i = 0; i < receiveSize.asInt16; i++)
    {
        serial.print(transmitBuffer(i), HEX);
        serial.print(" ");
    }
}

int16_t CommProtocol::getReceiveSize()
{
    return receiveSize.asInt16;
}
} // oooarkavr


// vim:ts=4:sw=4
