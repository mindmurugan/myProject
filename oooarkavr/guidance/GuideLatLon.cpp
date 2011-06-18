/*
 * Guide.cpp
 * Copyright (C) Brandon Wampler 2010 <bwampler@users.sourceforge.net>
 *
 * Guide.cpp is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Guide.cpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "Guide.hpp"

namespace oooarkavr
{

/*Calculations assume that west longitude and south lattitude coordinates are negetive, positive
 crosstrack is the the right. Heading increases as you rotate clockwise with north being 0 rad and south being pi.
 Westerly headings are represented by [0,-pi) */

Guide::Guide(int freq, Vector<GeodeticCoord> * fp, GeodeticCoord * pos, Vector<float> * vel_NED, Quaternion<float> * quat, Vector<float> * w_N) : System(freq), prevWp(0,0,0), nextWp(0,0,0), homeWp(0,0,0), distanceToWp(0), desiredHeading(0), xtRad(0), prevIndex(0), nextIndex(1), errorState(4), flightPlan(fp), pos(pos), w_N(w_N), quat(quat), vel_NED(vel_NED)
{
}

// Default deconstructor
Guide::~Guide()
{
}

void Guide::setup(int initIndex)
{
    setNextWpIndex(initIndex);
}


void Guide::run()
{
    Serial.println("run called");
    //Update errorState and other useful parameters
    updateHeadingStates();
    updateCrosstrackStates();

    updateTrackDistToNextWp();
    if (distanceToWp < 5) setNextWpIndex(nextIndex+1);
}

void Guide::updateHeadingStates()
{
    desiredHeading=trueHeading(prevWp,nextWp);
    Serial.print("Desired Heading: ");
    Serial.println(desiredHeading,8);
    Vector<float> euler = quat->toEuler();
    if 		((euler(2) - desiredHeading) < -pi) errorState(2) =  2*pi + (euler(2) - desiredHeading);
    else if ((euler(2) - desiredHeading) >  pi) errorState(2) = -2*pi - (euler(2) - desiredHeading);
    else	errorState(2) = euler(2) - desiredHeading;
    Serial.print("heading: ");
    Serial.println(euler(2),6);
    errorState(3) = (*w_N)(2);
}

//Calculate Crosstrack
void Guide::updateCrosstrackStates()
{
    //Update Crosstrack error
    xtRad =  asin(sin(distRadians(prevWp,*pos)) * sin(trueHeading(prevWp,*pos)-trueHeading(prevWp,nextWp)));

    //If at poles
    if (cos(prevWp.lat) < .00000001)     // EPS a small number ~ machine precision
    {
        if (prevWp.lat > 0)
        {
            xtRad = asin(sin(distRadians(prevWp,*pos)) * sin(deg2rad*((nextWp.lon - pos->lon)/1e7))); //starting from N pole
        }
        else
        {
            xtRad = asin(sin(distRadians(prevWp,*pos)) * sin(deg2rad*((pos->lon - nextWp.lon)/1e7))); //starting from S pole
        }
    }
    Serial.print("xtRad: ");
    Serial.println(xtRad,10);
    errorState(0) = (r0+(prevWp.alt+pos->alt)/2.0e2)*xtRad;

    float vTheta = atan2((*vel_NED)(1),(*vel_NED)(0));
    errorState(1) = -vel_NED->norm()*sin(desiredHeading-vTheta);
}

Vector<float> Guide::getErrorState()
{
    return errorState;
}

void Guide::updateTrackDistToNextWp()
{
    distanceToWp = (r0+(nextWp.alt+pos->alt)/2.0e2)*acos(cos(distRadians(*pos,nextWp))/cos(xtRad));
}

void Guide::setHomeWp(const int32_t & lat, const int32_t & lon, const int32_t & alt)
{
    homeWp.lat = lat;
    homeWp.lon = lon;
    homeWp.alt = alt;
}

void Guide::setHomeWp(const GeodeticCoord & home)
{
    homeWp = home;
}

void Guide::goHome()
{
    nextWp = homeWp;
    prevWp = *pos;
    desiredHeading = trueHeading(prevWp, nextWp);
}

void Guide::setNextWpIndex(int i)
{
    Serial.print("set next index: ");
    Serial.println(i);
    if ((size_t) i > (flightPlan->getSize()-1))
    {
        Serial.println("Flight Plan Index out of range");
    }
    else
    {
        if (i==0)
        {
            prevIndex = flightPlan->getSize()-1;
        }
        else
        {
            prevIndex = i - 1;
        }
        nextIndex = i;
        nextWp = (*flightPlan)(nextIndex);
        prevWp = (*flightPlan)(prevIndex);
    }
    desiredHeading = trueHeading(prevWp, nextWp);
}

//Calculate distance Between two waypoints in radians
float Guide::distRadians(const GeodeticCoord & c1, const GeodeticCoord & c2)
{
    float distRad = acos( cos(deg2rad*c1.lat/1e7)*cos(deg2rad*c2.lat/1e7)*cos(deg2rad*((c2.lon - c1.lon)/1e7)) + sin(deg2rad*c1.lat/1e7)*sin(deg2rad*c2.lat/1e7) );
    return distRad;
}

//Calculate true course
float Guide::trueHeading(const GeodeticCoord & c1, const GeodeticCoord & c2)
{
    float th = 0;

    //If at poles
    if (cos(deg2rad*c1.lat/1e7) < .00000001)     // EPS a small number ~ machine precision
    {
        if (c1.lat > 0)
        {
            th= pi;        //  starting from N pole
        }
        else th= 0.0;         //  starting from S pole
    }

    //All other coordinates
    float d = distRadians(c1,c2);
    if (sin(deg2rad*((c2.lon-c1.lon)/1e7))>=0)
    {
        th= acos((sin(deg2rad*c2.lat/1e7)-sin(deg2rad*c1.lat/1e7)*cos(d))/(sin(d)*cos(deg2rad*c1.lat/1e7)));
    }
    else
    {
        th=-acos((sin(deg2rad*c2.lat/1e7)-sin(deg2rad*c1.lat/1e7)*cos(d))/(sin(d)*cos(deg2rad*c1.lat/1e7)));
    }

    return th;
}

} // oooarkavr


// vim:ts=4:sw=4
