/*
 * basic.cpp
 * Copyright (C) James Goppert 2010 <jgoppert@users.sourceforge.net>
 *
 * basic.cpp is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * basic.cpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "basic.hpp"

namespace oooarkavr
{

void printArray(const char * msg, const float * array, size_t n, HardwareSerial & serial)
{
    serial.print(msg);
    for (size_t i=0;i<n;i++) serial.print(array[i],8), serial.print(" ");
    serial.println();
}

void printScalar(const char * msg, const float & scalar, HardwareSerial & serial)
{
    serial.print(msg);
    serial.print(scalar,8);
    serial.println();
}

void dot(const float *v, const float *w, float & s, const size_t & n)
{
    s = 0;
    for (size_t i=0;i<n;i++) s += v[i]*w[i];
}

void norm(const float *v, float & norm, const size_t & n)
{
    dot(v,v,norm,n);
    norm = sqrt(norm);
}

void cross(const float *u, const float *v, float *w)
{
    w[0] =  u[1]*v[2]-v[1]*u[2];
    w[1] = -u[0]*v[2]+v[0]*u[2];
    w[2] = -u[0]*v[1]+v[0]*u[1];
}

void unit(const float * v, float * unit, const size_t & n)
{
    float vNorm;
    norm(v,vNorm,n);
    for (size_t i=0;i<n;i++) unit[i] = v[i]/vNorm;
}

void quatProd(const float * q, const float * p, float * w)
{
    const float &a=q[0], &b=q[1], &c=q[2], &d=q[3];
    const float &e=p[0], &f=p[1], &g=p[2], &h=p[3];
    w[0] = e*a-b*f-c*g-d*h;
    w[1] = a*f+b*e+c*h-d*g;
    w[2] = a*g+c*e-b*h+d*f;
    w[3] = a*h+d*e+b*g-c*f;
    unit(w,w,4);
}

void quatToDcm(const float * q, float * dcm)
{
    const float &a=q[0], &b=q[1], &c=q[2], &d=q[3];
    float aa=a*a,ab=a*b,ac=a*c,ad=a*d,bb=b*b,
                                         bc=b*c,bd=b*d,cc=c*c,cd=c*d,dd=d*d;
    dcm[0] = aa+bb-cc-dd;
    dcm[1] = 2*(bc-ad);
    dcm[2] = 2*(bd+ac);
    dcm[3] = 2*(bc+ad);
    dcm[4] = a*a-b*b+c*c-d*d;
    dcm[5] = 2*(cd-ab);
    dcm[6] = 2*(bd-ac);
    dcm[7] = 2*(cd+ab);
    dcm[8] = a*a-b*b-c*c+d*d;
}

void matrixProd(const float * x, const float * y, float * z, const size_t & m, const size_t & n, const size_t & p)
{
    for (size_t i=0;i<m;i++) for (size_t j=0;j<p;j++)
        {
            z[i*p+j]=0;
            for (size_t k=0;k<n;k++) z[i*p+j] += x[i*n+k]*y[k*p+j];
        }
}

void matrixVectorProd(const float * x, const float * y, float * z, const size_t & m, const size_t & n)
{
    for (size_t i=0;i<m;i++)
    {
        z[i]=0;
        for (size_t k=0;k<n;k++) z[i] += x[i*n+k]*y[k];
    }
}

void quatToEuler(const float * q, float & roll, float & pitch, float & yaw)
{
    const float &a=q[0], &b=q[1], &c=q[2], &d=q[3];
    float aa=a*a,ab=a*b,ac=a*c,ad=a*d,bb=b*b,
                                         bc=b*c,bd=b*d,cc=c*c,cd=c*d,dd=d*d;
    roll = atan2(2*(cd+ab),aa-bb-cc+dd);
    pitch = asin(-2*(bd-ac));
    yaw = atan2(2*(bc+ad),aa+bb-cc-dd);
}

void quatRotate(const float * q, const float * v, float * w)
{
    float dcm[9];
    quatToDcm(q,dcm);
    matrixVectorProd(dcm,v,w,3,3);
}

void quatConj(const float * q, float * v)
{
    v[0] =  q[0];
    v[1] = -q[1];
    v[2] = -q[2];
    v[3] = -q[3];
}

void axisAngle2Quat(const float * axis, const float & angle, float * q)
{
    float cosTh, sinTh, axisNorm;
    cosTh = cos(angle/2);
    sinTh = sin(angle/2);
    norm(axis,axisNorm,3);
    q[0] = cosTh;
    q[1] = sinTh*axis[0]/axisNorm;
    q[2] = sinTh*axis[1]/axisNorm;
    q[3] = sinTh*axis[2]/axisNorm;
}

void quatVectorAlign(const float * v, const float * w, float * q)
{
    float axis[3], angle, dp, normV, normW, axisNorm;
    dot(v,w,dp,3);
    norm(v,normV,3);
    norm(w,normW,3);
    angle = acos(dp/(normV*normW));
    cross(v,w,axis);
    norm(axis,axisNorm,3);
    axisAngle2Quat(axis,angle,q);
}

} // oooarkavr


// vim:ts=4:sw=4
