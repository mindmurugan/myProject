/*
 * CommProtocol.hpp
 * Copyright (C) Brandon Wampler 2010 <bwampler@users.sourceforge.net>
 *
 * CommProtocol.hpp is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * CommProtocol.hpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef CommProtocol_hpp
#define CommProtocol_hpp

#include "oooarkavr/math/Vector.hpp"
#include "oooarkavr/utilities/definitions.hpp"

namespace oooarkavr
{

class CommProtocol
{
public:

    // buffer definition
    typedef Vector<uint8_t,uint16_t> Buffer;

    // constructors
    CommProtocol(HardwareSerial & comm);

    // destructors
    virtual ~CommProtocol();

    // send a message
    virtual bool send() = 0;

    // receive a message
    virtual bool receive() = 0;
    void printReceiveBuffer(HardwareSerial & serial);
    void printTransmitBuffer(HardwareSerial & serial);
    int16_t getReceiveSize();

    // pack/unpack raw buffer
    void pack(const uint8_t * buffer, const size_t & length);
    void packZeros(const size_t & length);
    void unpack(uint8_t * buffer, const size_t & length, const size_t & pos);

    // pack data least significant bit first
    void packLsb(const float & val);
    void packLsb(const int16_t & val);
    void packLsb(const int32_t & val);

    // pack data most significant bit first
    void packMsb(const float & val);
    void packMsb(const int16_t & val);
    void packMsb(const uint16_t & val);
    void packMsb(const int32_t & val);
    void packMsb(const uint32_t & val);

    // unpack data least significant bit first
    void unpackLsb(float & val, const size_t & pos);
    void unpackLsb(int16_t & val, const size_t & pos);
    void unpackLsb(int32_t & val, const size_t & pos);
    void unpackLsb(uint32_t & val, const size_t & pos);

    // unpack data most significant bit first
    void unpackMsb(float & val, const size_t & pos);
    void unpackMsb(int16_t & val, const size_t & pos);
    void unpackMsb(int32_t & val, const size_t & pos);
    void unpackMsb(uint32_t & val, const size_t & pos);

    HardwareSerial & serial;

protected:
    Buffer receiveBuffer, transmitBuffer;
    int step;
    int16_uint8 receiveSize, receiveSum;
    uint8_t data, dataPrev;
    uint8_t header[2];
    uint8_t footer[2];
};

} // oooarkavr

#endif

// vim:ts=4:sw=4
