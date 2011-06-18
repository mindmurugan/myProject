/*
 * Ubx.hpp
 * Copyright (C) Brandon Wampler 2010 <bwampler@users.sourceforge.net>
 *
 * Ubx.hpp is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Ubx.hpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef Ubx_hpp
#define Ubx_hpp

#include "oooarkavr/math/Vector.hpp"
#include "oooarkavr/utilities/definitions.hpp"
#include "oooarkavr/communication/CommProtocol.hpp"

namespace oooarkavr
{

class Ubx : public CommProtocol
{
public:
    // constructors
    Ubx(HardwareSerial & comm);

    // destructors
    virtual ~Ubx();

    // send a message
    bool send(char classID, char messageID);
    bool send(); //default send message (Configure message classID: 0x06 messageID: 0x00);

    // receive a message
    bool receive();
    char classID, messageID;
    uint8_t CK_A_REC, CK_B_REC, CK_A, CK_B, CK_A_Send, CK_B_Send;
};

} // oooarkavr

#endif

// vim:ts=4:sw=4
