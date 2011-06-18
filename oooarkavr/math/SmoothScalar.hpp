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

#ifndef SmoothScalar_hpp
#define SmoothScalar_hpp

namespace oooarkavr
{

template <class scalar>
class SmoothScalar
{
public:
    SmoothScalar(const scalar & smoothing=0, const scalar & initialValue=0) : smoothing(smoothing), value(initialValue)
    {
    }
    void reset(const scalar & val)
    {
        value = val;
    }
    const SmoothScalar & operator=(const scalar & val)
    {
        value = (1-smoothing)*val + smoothing*value;
        return (*this);
    }
    const scalar get() const
    {
        return value;
    }
    void setSmoothing(const scalar & smooth)
    {
        smoothing = smooth;
    }
    void print(HardwareSerial & serial=Serial, const char * msg="", const int & format=4) const
    {
        serial.print(msg);
        serial.println(value,format);
    }
private:
    scalar smoothing;
    scalar value;
};

} // oooarkavr

#endif

// vim:ts=4:sw=4
