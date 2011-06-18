/*
 * Sensor.cpp
 * Copyright (C) James Goppert 2010 <jgoppert@users.sourceforge.net>
 *
 * Sensor.cpp is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Sensor.cpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "Sensor.hpp"

namespace oooarkavr
{

Sensor::Sensor(uint16_t freq, uint8_t size, Vector<uint16_t> cutFreq, uint8_t samples,
               uint8_t delay, uint8_t precision, Endian endian) :
        System(freq),
        data(size),
        cutFreq(cutFreq),
        samples(samples),
        delay(delay),
        width(sizeof(uint16_t)/sizeof(uint8_t)),
        precisionAdjustment(1<<(8*sizeof(uint16_t)-precision)),
        rawBits(8*sizeof(uint8_t)),
        endian(endian)
{
}

void Sensor::run()
{
    Serial.println("running.");
    Vector<uint16_t> tmp(data.getSize()); // initialized to zero by const.
    for (size_t i=0;i<samples;i++)
    {
        tmp += processRaw(read());
        delayMicroseconds(delay);
    }
    tmp/=samples;
    data = tmp + (data-tmp)*cutFreq/(cutFreq+getFreq());
}

Sensor::~Sensor()
{
}

Vector<uint16_t> Sensor::getData()
{
    return data;
}

Vector<uint16_t> Sensor::processRaw(Vector<uint8_t> raw)
{
    for (size_t i=0;i<data.getSize();i++) for (size_t j=0;j<width;j++)
        {
            if (endian == little) data(i) |= raw(width*i+j) << rawBits*width*j;
            else if (endian == big) data(i) |= raw(width*i+j) << rawBits*(width-1-j);
        }
    return data*=precisionAdjustment;
}

Vector<uint16_t> Sensor::processRaw(Vector<uint16_t> raw)
{
    return data*=precisionAdjustment;
}

} // oooarkavr


// vim:ts=4:sw=4
