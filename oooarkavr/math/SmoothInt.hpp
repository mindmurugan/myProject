/*
 * SmoothScalar.hpp
 * Copyright (C) James Goppert 2010 <jgoppert@users.sourceforge.net>
 *
 * SmoothScalar.hpp is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * SmoothScalar.hpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef SmoothInt_hpp
#define SmoothInt_hpp

namespace oooarkavr
{

template <class scalar>
class SmoothInt
{
public:
    SmoothInt(const float & smoothing=0, const scalar & smoothRange = 0, const scalar & initialValue=0) : smoothing(smoothing), smoothRange(smoothRange),  value(initialValue)

    {
    }
    void reset(const scalar & val)
    {
        value = val;
    }
    const SmoothInt & operator=(const scalar & val)
    {
        if (abs(val-value)<smoothRange || smoothRange == 0)
        {
            value =  ((float)1-smoothing)*(float)val + smoothing*(float)value;
        }
        else
        {
            reset((val+value)/2);
        }
        return (*this);
    }
    const scalar get() const
    {
        return value;
    }
    void setSmoothing(const float & smooth)
    {
        smoothing = smooth;
    }
    void setSmoothRange(const scalar & sr)
    {
        smoothRange = sr;
    }
    void print(HardwareSerial & serial=Serial, const char * msg="", const int & format=4) const
    {
        serial.print(msg);
        serial.println(value,format);
    }
private:
    float smoothing;
    scalar value;
    scalar smoothRange;
};

} // oooarkavr

#endif

// vim:ts=4:sw=4
