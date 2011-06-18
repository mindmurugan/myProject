/*
    cPlusPlus.hpp This is a matrix class for the avr microcontroller.

    Copyright (C) 2010  James Goppert jgoppert@users.sourceforge.net

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see .
*/

// Matrix/ Vector class derived from this source:
// http://devhood.com/tutorials/tutorial_details.aspx?tutorial_id=502

#ifndef cPlusPlus_hpp
#define cPlusPlus_hpp

#include "WProgram.h"



void assert(bool test, const char * file, const int line, HardwareSerial & serial=Serial);
void * operator new(size_t size);
void operator delete(void * ptr);
void * operator new[](size_t size);
void operator delete[](void * ptr);
__extension__ typedef int __guard __attribute__((mode (__DI__)));
extern "C" int __cxa_guard_acquire(__guard *);
extern "C" void __cxa_guard_release (__guard *);
extern "C" void __cxa_guard_abort (__guard *);
extern "C" void __cxa_pure_virtual(void);
extern "C" int freeMemory();
void displayMemory();

#endif
