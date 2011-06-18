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
#include "oooarkavr/systems/System.hpp"
#include "oooarkavr/utilities/definitions.hpp"

namespace oooarkavr
{

class GeodeticCoord
{
public:
    Vector<int32_t> vec;
    float mPLat, mPLon;
    GeodeticCoord(): vec(3), mPLat(0), mPLon(0) {}
    GeodeticCoord(int32_t lat, int32_t lon, int32_t alt): vec(3), mPLat(0), mPLon(0)
    {
        vec(0) = lon;
        vec(1) = lat;
        vec(2) = alt;
        mPLat = (r0+alt/1e2)*pi/180;
        mPLon = (r0+alt/1e2)*cos(deg2Rad*(lat/1e7))*pi/180;
    }
    void set(int32_t la, int32_t lo, int32_t al)
    {
        vec(0) = lo;
        vec(1) = la;
        vec(2) = al;
        mPLat = (r0+alt()/1e2)*pi/180;
        mPLon = (r0+alt()/1e2)*cos(deg2Rad*(lat()/1e7))*pi/180;
    }
    int32_t lat()
    {
        return vec(1);
    }
    int32_t lon()
    {
        return vec(0);
    }
    int32_t alt()
    {
        return vec(2);
    }
    float mPerLat()
    {
        return mPLat;
    }
    float mPerLon()
    {
        return mPLon;
    }
    GeodeticCoord	operator-(GeodeticCoord& g)
    {
        GeodeticCoord result;
        result.vec.setSize(3);
        result.vec = this->vec - g.vec;
        result.mPLat = (this->mPerLat() + g.mPerLat())/2;
        result.mPLon = (this->mPerLon() + g.mPerLon())/2;
        return result;
    }
    Vector<float> rect()
    {
        Vector<float> result;
        result.setSize(3);
        result(0) = vec(0)/1e7*mPerLon();
        result(1) = vec(1)/1e7*mPerLat();
        result(2) = vec(2)/1e2;
        return result;
    }


};

class Guide : public System
{
public:
    Guide(int freq, Vector<GeodeticCoord*> * fp, Vector<float> * errorState, GeodeticCoord * pos, Vector<float> * vel_NED, Vector<float> * euler, Vector<float> * w_N, float wpCrit = 10);
    // Default deconstructor
    virtual ~Guide();

    //Update Class members
    void setup(int initIndex);
    virtual void run();
    Vector<float> getErrorState();

    //Home sweet home
    void setHomeWp(const int32_t & lat, const int32_t & lon, const int32_t & alt);
    void setHomeWp(const GeodeticCoord & home);
    void goHome();

    //Utility functions
    void setNextWpIndex(int i);
    float heading(GeodeticCoord & c1, GeodeticCoord & c2);


    //Class Members
    GeodeticCoord prevWp, nextWp, homeWp;
    float distanceToWp, desiredHeading, wpCriteria;
    int prevIndex, nextIndex;
    Vector<GeodeticCoord*> * flightPlan;
    GeodeticCoord * pos;
    Vector<float> * w_N;
    Vector<float> * euler, * vel_NED, * errorState; // (0)crossTrack (1)crossTrack rate (2)headingError (3)headingErrorRate


private:

};

} // oooarkavr

#endif

// vim:ts=4:sw=4
