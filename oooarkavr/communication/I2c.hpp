/*
 * I2c.hpp
 * Copyright (C) James Goppert 2010 <jgoppert@users.sourceforge.net>
 *
 * I2c.hpp is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * I2c.hpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef I2c_hpp
#define I2c_hpp

#include "Wire.h"
#include "oooarkavr/utilities/definitions.hpp"

namespace oooarkavr
{

// class to communicate on i2c bus
class I2c
{
public:
    I2c(const uint8_t & wireId, const Endian & endian);
    virtual ~I2c() {};
    int read(const uint8_t & address, const size_t & requestSize, const size_t samples, float * data);
    void write(const uint8_t & address, const uint8_t & data);
    /*void I2C_Init();
    void Compass_Init();
    */

private:
    uint8_t wireId;
    Endian endian;
};

} // oooarkavr

#endif

// vim:ts=4:sw=4
