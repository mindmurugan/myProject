/*
 * Sirf.hpp
 * Copyright (C) James Goppert 2010 <jgoppert@users.sourceforge.net>
 *
 * Sirf.hpp is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Sirf.hpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef Sirf_hpp
#define Sirf_hpp

#include "oooarkavr/math/Vector.hpp"
#include "oooarkavr/utilities/definitions.hpp"
#include "oooarkavr/communication/CommProtocol.hpp"

namespace oooarkavr
{

class Sirf : public CommProtocol
{
public:
    // constructors
    Sirf(HardwareSerial & comm);

    // destructors
    virtual ~Sirf();

    // send a message
    bool send();

    // receive a message
    bool receive();
};

} // oooarkavr

#endif

// vim:ts=4:sw=4
