/*
 * Guide2.hpp
 * Copyright (C) Brandon Wampler 2010 <bwampler@users.sourceforge.net>
 *
 * Guide2.hpp is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Guide2.hpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef Guide2_hpp
#define Guide2_hpp

#include "HardwareSerial.h"
#include "oooarkavr/math/Vector.hpp"
#include "oooarkavr/math/Quaternion.hpp"
#include "oooarkavr/systems/System.hpp"

namespace oooarkavr
{
const float pi = 3.14159265358979323846264338327950288;
const float deg2rad = 1.745329251994329577e-2;
const float r0 = 6378137; //average radius of earth

class GeodeticCoord
{
public:
    int32_t lat, lon, alt;
    GeodeticCoord(): lat(0), lon(0), alt(0) {}
    GeodeticCoord(int32_t lat, int32_t lon, int32_t alt): lat(lat), lon(lon), alt(alt) {}
    void set(int32_t la, int32_t lo, int32_t al)
    {
        lat =la;
        lon= lo;
        alt = al;
    }

};

class Guide : public System
{
public:
    Guide(int freq, Vector<GeodeticCoord> * fp, GeodeticCoord * pos, Vector<float> * vel_NED, Quaternion<float> * quat, Vector<float> * w_N);
    // Default deconstructor
    virtual ~Guide();

    //Update Class members
    void setup(int initIndex);
    virtual void run();
    void updateCrosstrackStates();
    void updateHeadingStates();
    void updateTrackDistToNextWp(); //needs crosstrack updated first
    Vector<float> getErrorState();

    //Home sweet home
    void setHomeWp(const int32_t & lat, const int32_t & lon, const int32_t & alt);
    void setHomeWp(const GeodeticCoord & home);
    void goHome();

    //Utility functions
    void setNextWpIndex(int i);
    float distRadians(const GeodeticCoord & c1, const GeodeticCoord & c2);
    float trueHeading(const GeodeticCoord & from, const GeodeticCoord & to);

    //Class Members
    GeodeticCoord prevWp, nextWp, homeWp;
    float distanceToWp, desiredHeading, xtRad;
    int prevIndex, nextIndex;
    Vector<float> errorState; // (0)crossTrack (1)crossTrack rate (2)headingError (3)headingErrorRate
    Vector<GeodeticCoord> * flightPlan;
    GeodeticCoord * pos;
    Vector<float> * w_N;
    Quaternion<float> * quat;
    Vector<float> * vel_NED;

private:

};

} // oooarkavr

#endif

// vim:ts=4:sw=4
