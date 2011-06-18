/* Quaternion.hpp
 * Copyright (C) James Goppert 2010 <jgoppert@users.sourceforge.net>
 *
 * Quaternion.hpp is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Quaternion.hpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef Quaternion_hpp
#define Quaternion_hpp

#include "Vector.hpp"
#include "Matrix.hpp"

namespace oooarkavr
{

#ifdef OOOARK_ASSERT
const static char quaternionSource[] ="Quaternion.hpp";
#endif

// quaternion
template <class dataType>
class Quaternion : public Vector<dataType>
{
public:
    // default constructor
    Quaternion() : Vector<dataType>(4)
    {
        (*this)(0) = 1; // to initialize to unit magnitude
    }
    // axis angle constructor
    Quaternion(const Vector<dataType> & axis, const dataType & angle) : Vector<dataType>(4)
    {
        Vector<float> axisHat = axis.unit();
        float theta = angle/2.;
        (*this)(0) = cos(theta);
        (*this)(1) = sin(theta)*axisHat(0);
        (*this)(2) = sin(theta)*axisHat(1);
        (*this)(3) = sin(theta)*axisHat(2);
    }
    // vector alignment constructor
    Quaternion(const Vector<dataType> & v, const Vector<dataType> & w)
    {
        Vector<float> axis = v.cross(w);
        float angle = v.dot(w)/(v.norm()*w.norm());
        angle = acos(angle);
        (*this) = Quaternion(axis,angle);
    }
    // copy construct
    Quaternion(const Quaternion &q) : Vector<dataType>(q)
    {
    }
    // copy construct from vector
    Quaternion(const Vector<dataType> &v) : Vector<dataType>(v)
    {
#ifdef OOOARK_ASSERT
        assert(v.getSize()==4,quaternionSource,__LINE__);
#endif
    }
    // deconstructor
    virtual ~Quaternion()
    {
    }
    // product with scalar, vector inheritance won't work directly
    // since * is also being defined for quaternion product
    const Quaternion operator*(const dataType & s) const
    {
        Quaternion q;
        q(0) = s*(*this)(0);
        q(1) = s*(*this)(1);
        q(2) = s*(*this)(2);
        q(3) = s*(*this)(3);
        return q;
    }
    // quaternion product
    const Quaternion operator*(const Quaternion & p) const
    {
        Quaternion result;
        result(0) = p(0)*(*this)(0)-p(1)*(*this)(1)-p(2)*(*this)(2)-p(3)*(*this)(3);
        result(1) = p(1)*(*this)(0)+p(0)*(*this)(1)+p(3)*(*this)(2)-p(2)*(*this)(3);
        result(2) = p(2)*(*this)(0)+p(0)*(*this)(2)-p(3)*(*this)(1)+p(1)*(*this)(3);
        result(3) = p(3)*(*this)(0)+p(0)*(*this)(3)+p(2)*(*this)(1)-p(1)*(*this)(2);
        return result;
    }
    // quaternion conj
    const Quaternion conj() const
    {
        Quaternion p;
        p(0) = (*this)(0);
        p(1) = -(*this)(1);
        p(2) = -(*this)(2);
        p(3) = -(*this)(3);
        return p;
    }
    // quaternion integration
    const Quaternion integrate(const Vector<dataType> & w, const dataType & dt) const
    {
#ifdef OOOARK_ASSERT
        assert(w.getSize()==3,quaternionSource,__LINE__);
#endif
        Quaternion wQ;
        wQ(0) = 0;
        wQ(1) = w(0);
        wQ(2) = w(1);
        wQ(3) = w(2);
        return ((*this)+(*this)*wQ*(.5*dt)).unit();
    }
    // quaternion rotation
    const Vector<dataType> rotate(const Vector<dataType> & v) const
    {
#ifdef OOOARK_ASSERT
        assert(v.getSize()==3,quaternionSource,__LINE__);
#endif
        Quaternion wQ, vQ;
        Vector<dataType> w(3);
        vQ(0) = 0;
        vQ(1) = v(0);
        vQ(2) = v(1);
        vQ(3) = v(2);
        wQ = (*this)*vQ*(*this).conj();
        w(0) = wQ(1);
        w(1) = wQ(2);
        w(2) = wQ(3);
        return w;
    }
    // quaternion to dcm
    const Matrix<dataType> toDcm() const
    {
        Matrix<dataType> dcm(3,3);
        dataType aa,ab,ac,ad,bb,bc,bd,cc,cd,dd;
        aa = (*this)(0)*(*this)(0);
        ab = (*this)(0)*(*this)(1);
        ac = (*this)(0)*(*this)(2);
        ad = (*this)(0)*(*this)(3);
        bb = (*this)(1)*(*this)(1);
        bc = (*this)(1)*(*this)(2);
        bd = (*this)(1)*(*this)(3);
        cc = (*this)(2)*(*this)(2);
        cd = (*this)(2)*(*this)(3);
        dd = (*this)(3)*(*this)(3);
        dcm(0,0) = aa+bb-cc-dd;
        dcm(0,1) = 2*(bc-ad);
        dcm(0,2) = 2*(bd+ac);
        dcm(1,0) = 2*(bc+ad);
        dcm(1,1) = aa-bb+cc-dd;
        dcm(1,2) = 2*(cd-ab);
        dcm(2,0) = 2*(bd-ac);
        dcm(2,1) = 2*(cd+ab);
        dcm(2,2) = aa-bb-cc+dd;
        return dcm;
    }
    // quaternion to euler angle conversion, roll, pitch, yaw
    const Vector<dataType> toEuler() const
    {
        Quaternion q=*this;
        Vector<dataType> euler(3);
        dataType aa,ab,ac,ad,bb,bc,bd,cc,cd,dd;
        aa = (*this)(0)*(*this)(0);
        ab = (*this)(0)*(*this)(1);
        ac = (*this)(0)*(*this)(2);
        ad = (*this)(0)*(*this)(3);
        bb = (*this)(1)*(*this)(1);
        bc = (*this)(1)*(*this)(2);
        bd = (*this)(1)*(*this)(3);
        cc = (*this)(2)*(*this)(2);
        cd = (*this)(2)*(*this)(3);
        dd = (*this)(3)*(*this)(3);
        euler(0) = atan2(2*(cd+ab),aa-bb-cc+dd);
        euler(1) = asin(-2*(bd-ac));
        euler(2) = atan2(-2*(bc+ad),aa+bb-cc-dd);
        return euler;
    }
    // self test
    static bool selfTest()
    {
        // Quaternion product test
        //TODO: finish implemented self testings
        Serial.println("Quaternion product test.");
        Quaternion p,q,l;
        q(0) = 1;
        q(1) = 1;
        l=p*q;
        p.print(Serial,"p: ");
        return true;
    }
};

} // oooarkavr

#endif

// vim:ts=4:sw=4
