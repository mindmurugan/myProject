/*
 * Controller.hpp
 * Copyright (C) James Goppert 2010 <jgoppert@users.sourceforge.net>
 *
 * Controller.hpp is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Controller.hpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef Controller_hpp
#define Controller_hpp

#include "HardwareSerial.h"
#include "oooarkavr/systems/System.hpp"

namespace oooarkavr
{

class Controller : public System
{
public:
    // Default constructor
    Controller(int freq);
    // Default deconstructor
    virtual ~Controller();
    Vector<float> getControl()
    {
        return control;
    }
    void run() = 0;
    Vector<float> control;
    Vector<float> command;
    Vector<float> feedback;
};

} // oooarkavr

#endif

// vim:ts=4:sw=4
