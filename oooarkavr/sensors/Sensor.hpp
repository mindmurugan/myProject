/*
 * Sensor.hpp
 * Copyright (C) James Goppert 2010 <jgoppert@users.sourceforge.net>
 *
 * Sensor.hpp is free software: you can redistribute it and/or modify it
 * under the terms of thppe GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Sensor.hpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef Sensor_hpp
#define Sensor_hpp

#include "oooarkavr/utilities/definitions.hpp"
#include "oooarkavr/math/Vector.hpp"
#include "oooarkavr/systems/System.hpp"

namespace oooarkavr
{

// sensor interface
class ISensor
{
public:
    virtual ~ISensor() {};
    virtual void run() = 0;
    virtual Vector<uint16_t> getData();
    virtual Vector<uint8_t> read() = 0;
};

// A class to abstract sensor reading
class Sensor : public virtual System, public virtual ISensor
{
public:
    Sensor(uint16_t freq, uint8_t size, Vector<uint16_t> cutFreq, uint8_t samples, uint8_t delay, uint8_t precision, Endian endian);
    virtual Vector<uint8_t> read() = 0;
    virtual void run();
    virtual ~Sensor();
    Vector<uint16_t> getData();
private:
    // attributes
    Vector<uint16_t> data; // data collected
    Vector<uint16_t> cutFreq; // cut off frequencies for low pass filters
    uint8_t samples;  // number of sample to average per read
    uint8_t delay; // delay period in microseconds
    uint8_t width; // ratio of data type to read type
    uint8_t precisionAdjustment; // number to multiply by to maximize precision of oversampling
    uint8_t rawBits; // number of bits per raw measurement
    Endian endian; // specifies big/little byte order for read

    // methods
    Vector<uint16_t> processRaw(Vector<uint8_t> raw);
    Vector<uint16_t> processRaw(Vector<uint16_t> raw);
};

// sensor interfaces

// global positioning system
class IGps
{
public:
    virtual ~IGps() {};
    virtual const Vector<int32_t> getGeodetic() const = 0; // 1E-7 deg
    virtual const Vector<float> getVelNED() const = 0;
};

// inertial measurement unit
class IImu
{
public:
    virtual ~IImu() {};
};

} // oooarkavr

#endif

// vim:ts=4:sw=4
