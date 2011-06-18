/*
 * I2c.cpp
 * Copyright (C) James Goppert 2010 <jgoppert@users.sourceforge.net>
 *
 * I2c.cpp is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * I2c.cpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "I2c.hpp"

namespace oooarkavr
{

I2c::I2c(const uint8_t & wireId, const Endian & endian) :
        wireId(wireId), endian(endian)
{
}

int I2c::read(const uint8_t & address, const size_t & requestSize,
              const size_t samples, float * data)
{

    uint8_t bytes[2];
    int32_t * sum = new int32_t[requestSize];
    for (size_t i=0;i<requestSize;i++) sum[i] = 0;
    for (size_t i=0;i<samples;i++)
    {
        Wire.beginTransmission(wireId);
        Wire.send(address);
        Wire.requestFrom(wireId,2*requestSize);
        for (size_t j=0;j<requestSize;j++)
        {
            if (Wire.available()) bytes[0]=Wire.receive();
            else
            {
                Wire.endTransmission();
                delete [] sum;
                return -1;
            }
            if (Wire.available()) bytes[1]=Wire.receive();
            else
            {
                Wire.endTransmission();
                delete [] sum;
                return -2;
            }
            if (endian==little) sum[j] += bytes[0] | bytes[1] << 8;
            else if (endian==big) sum[j] += bytes[0] << 8 | bytes[1];
        }
        Wire.endTransmission();
    }
    for (size_t i=0;i<requestSize;i++)
        data[i] = float(sum[i])/samples;
    delete [] sum;
    return 0;
}

void I2c::write(const uint8_t & address, const uint8_t & data)
{
    //Serial.println("sending to magnetometer");
    //Serial.flush();
    Wire.beginTransmission(wireId);
    //Serial.println("Communication Open");
    //Serial.flush();
    Wire.send(address);
    //Serial.println("Address Sent");
    //Serial.flush();
    Wire.send(data);
    //Serial.println("Data sent");
    //Serial.flush();
    Wire.endTransmission();
    //Serial.println("message sent");
    //Serial.flush();
}

/*void I2C_Init()
{
  Wire.begin();
}


void Compass_Init()
{
	int CompassAddress = 0x1E;
  Wire.beginTransmission(CompassAddress);
  Wire.send(0x02);
  Wire.send(0x00);   // Set continouos mode (default to 10Hz)
  Wire.endTransmission(); //end transmission

}
*/


} // oooarkavr


// vim:ts=4:sw=4
