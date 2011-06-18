/*
 * Guide.cpp
 * Copyright (C) Brandon Wampler 2010 <bwampler@users.sourceforge.net>
 *
 * Guide.cpp is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any lat()er version.
 *
 * Guide.cpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License alon()g
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "Guide.hpp"

namespace oooarkavr
{

/*Calculat()ions assume that west lon()gitude and south lat()titude coordinates are negetive, positive
 crosstrack is the the right. Heading increases as you rotate clockwise with north being 0 rad and south being pi.
 Westerly headings are represented by [0,-pi) */

Guide::Guide(int freq, Vector<GeodeticCoord*> * fp, Vector<float> * errorState, GeodeticCoord * pos, Vector<float> * vel_NED, Vector<float> * euler, Vector<float> * w_N, float wpCrit) : System(freq), prevWp(0,0,0), nextWp(0,0,0), homeWp(0,0,0), distanceToWp(0), desiredHeading(0), wpCriteria(wpCrit), prevIndex(0), nextIndex(1), flightPlan(fp), pos(pos), w_N(w_N), euler(euler), vel_NED(vel_NED), errorState(errorState)
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
    GeodeticCoord prevNext = nextWp-prevWp;
    GeodeticCoord currNext = nextWp-(*pos);
    desiredHeading = heading(prevWp, nextWp);
    if (prevNext.rect().cross(currNext.rect())(2) > 0)
    {
        (*errorState)(0) = (prevNext.rect().cross(currNext.rect())).norm()/(prevNext.rect().norm());
    }
    else
    {
        (*errorState)(0) = -(prevNext.rect().cross(currNext.rect())).norm()/(prevNext.rect().norm());
    }

    if (prevNext.rect().cross(currNext.rect())(2) > 0)
    {
        (*errorState)(1) = prevNext.rect().cross((*vel_NED)).norm()/prevNext.rect().norm();
    }
    else
    {
        (*errorState)(1) = -prevNext.rect().cross((*vel_NED)).norm()/prevNext.rect().norm();
    }


    distanceToWp = currNext.rect().norm();


    if 		(((*euler)(2) - desiredHeading) < -pi) (*errorState)(2) =  2*pi + ((*euler)(2) - desiredHeading);
    else if (((*euler)(2) - desiredHeading) >  pi) (*errorState)(2) = -2*pi - ((*euler)(2) - desiredHeading);
    else	(*errorState)(2) = (*euler)(2) - desiredHeading;
    (*errorState)(3) = (*w_N)(2);




    if (distanceToWp < wpCriteria) setNextWpIndex(nextIndex + 1);

    //Serial.print("Crosstrack: "), Serial.println((*errorState)(0),4);
    //Serial.print("Crosstrack Rate: "), Serial.println((*errorState)(1),4);
    //Serial.print("heading error: "), Serial.println((*errorState)(2),4);
    //Serial.print("heading rate error: "), Serial.println((*errorState)(3),4);

    //Serial.print("desired heading: "), Serial.println(desiredHeading,4);
    //Serial.print("dist to wp: "), Serial.println(distanceToWp,4);
    //Serial.println();
    //Serial.println();
}

float Guide::heading( GeodeticCoord & c1, GeodeticCoord & c2)
{
    GeodeticCoord c1c2;
    c1c2 = c2-c1;
    float head = atan2(c1c2.lon()*c1c2.mPerLon(),c1c2.lat()*c1c2.mPerLat());
    return head;
}

Vector<float> Guide::getErrorState()
{
    return *errorState;
}

void Guide::setHomeWp(const int32_t & lat, const int32_t & lon, const int32_t & alt)
{
    homeWp.vec(0) = lon;
    homeWp.vec(1) = lat;
    homeWp.vec(2) = alt;
}

void Guide::setHomeWp(const GeodeticCoord & home)
{
    homeWp = home;
}

void Guide::goHome()
{
    nextWp = homeWp;
    prevWp = (*pos);
    desiredHeading = heading(prevWp, nextWp);
}

void Guide::setNextWpIndex(int i)
{
    Serial.print("set next index: ");
    Serial.println(i);
    nextIndex = i;
    if ((size_t) nextIndex > (flightPlan->getSize()-1))
    {
        Serial.println("Flight Plan Index out of range");
        nextIndex = 0;
        prevIndex = flightPlan->getSize()-1;
    }
    else
    {
        prevIndex = i - 1;
    }

    nextWp = (*(*flightPlan)(nextIndex));
    prevWp = (*(*flightPlan)(prevIndex));
    desiredHeading = heading(prevWp, nextWp);
}



} // oooarkavr


// vim:ts=4:sw=4
