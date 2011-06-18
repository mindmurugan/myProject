/*
    cPlusPlus.cpp This is a matrix class for the avr microcontroller.

    Copyright (C) 2010  James Goppert jgoppert@users.sourceforge.net

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see .
*/

#include "cPlusPlus.hpp"

#ifdef OOOARK_ASSERT
void assert(bool test, const char * file, const int line, HardwareSerial & serial)
{
    if (!test)
    {
        serial.print("Assert: ");
        serial.print(file);
        serial.print(" ");
        serial.println(line);
    }
}
#endif

void * operator new(size_t size)
{
    displayMemory();
    return malloc(size);
}
void operator delete(void * ptr)
{
    displayMemory();
    if (ptr) free(ptr);
}
void * operator new[](size_t size)
{
    displayMemory();
    return malloc(size);
}
void operator delete[](void * ptr)
{
    displayMemory();
    if (ptr) free(ptr);
}
int __cxa_guard_acquire(__guard *g)
{
    return !*(char *)(g);
}
void __cxa_guard_release (__guard *g)
{
    *(char *)g = 1;
}
void __cxa_guard_abort (__guard *) {}
void __cxa_pure_virtual(void) {}

// free memory
extern unsigned int __data_start;
extern unsigned int __data_end;
extern unsigned int __bss_start;
extern unsigned int __bss_end;
extern unsigned int __heap_start;
extern void *__brkval;


void displayMemory()
{
    static int minMemFree=0;
    if (minMemFree<=0 || freeMemory()<minMemFree)
    {
        minMemFree = freeMemory();
        Serial.print("min bytes free: ");
        Serial.println(minMemFree);
    }
}

int freeMemory()
{
    int free_memory;

    if ((int)__brkval == 0)
        free_memory = ((int)&free_memory) - ((int)&__bss_end);
    else
        free_memory = ((int)&free_memory) - ((int)__brkval);

    return free_memory;
}
