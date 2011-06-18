/*
 * Matrix.h
 * Copyright (C) James Goppert 2010 <jgoppert@users.sourceforge.net>
 *
 * Matrix.h is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Matrix.h is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef Matrix_hpp
#define Matrix_hpp

#include "oooarkavr/math/Vector.hpp"

namespace oooarkavr
{

#ifdef OOOARK_ASSERT
const static char matrixSource[] ="Matrix.hpp";
#endif

// matrix
template <class dataType>
class Matrix
{
public:
    // default constructor
    Matrix(const int & rows=0, const int & cols=0) : numCols(0), numRows(0), data(NULL)
    {
        setSize(rows,cols);
    }
    // copy constructor
    Matrix(const Matrix& m) : numCols(0), numRows(0), data(NULL)
    {
        setSize(m.getRows(),m.getCols());
        for (int r=0;r<getRows();r++) for (int c=0;c<getCols();c++)
                (*this)(r,c)=m(r,c);
    }
    // destructor
    virtual ~Matrix()
    {
        setSize(0,0);
    }
    // set the size of the matrix
    void setSize(const int & rows, const int & cols)
    {
        if (cols > 0 && rows >0)
        {
            data = new dataType[rows*cols];
            memset(data,0,sizeof(dataType)*rows*cols/sizeof(char));
            numCols=cols;
            numRows=rows;
        }
        else
        {
            delete [] data;
            data=NULL;
            numCols = 0;
            numRows = 0;
        }
    }
    // return number of columns
    const int getCols() const
    {
        return numCols;
    }
    // return number of rows
    const int getRows() const
    {
        return numRows;
    }
    // const return element
    const dataType operator()(const int & row, const int & col) const
    {
#ifdef OOOARK_ASSERT
        assert(row<getRows() && col<getCols(),matrixSource,__LINE__);
#endif
        return data[row*getCols()+col];
    }
    // return element
    dataType& operator()(const int & row, const int & col)
    {
#ifdef OOOARK_ASSERT
        assert(row<getRows() && col<getCols(),matrixSource,__LINE__);
#endif
        return data[row*getCols()+col];
    }
    // assigment operator
    Matrix& operator=(const Matrix& m)
    {
        setSize(m.getRows(),m.getCols());
        for (int r=0;r<getRows();r++) for (int c=0;c<getCols();c++)
                (*this)(r,c) = m(r,c);
        return *this;
    }
    // equal
    const bool operator==(const Matrix& m) const
    {
#ifdef OOOARK_ASSERT
        assert(getRows()==m.getRows() && getCols()==m.getCols(),matrixSource,__LINE__);
#endif
        for (int r=0;r<getRows();r++) for (int c=0;c<getCols();c++)
            {
                if ((*this)(r,c)!=m(r,c)) return false;
            }
        return true;
    }
    // not equal
    const bool operator!=(const Matrix& m) const
    {
        return !((*this)==m);
    }
    // addition
    const Matrix operator+( const Matrix& m) const
    {
#ifdef OOOARK_ASSERT
        assert(getCols()==m.getCols() && getRows()==m.getRows(),matrixSource,__LINE__);
#endif
        Matrix result(getRows(),getCols());
        for (int r=0;r<getRows();r++)
            for (int c=0;c<getCols();c++)
                result(r,c)=(*this)(r,c)+m(r,c);
        return result;
    }
    // subtraction
    const Matrix operator-( const Matrix& m) const
    {
#ifdef OOOARK_ASSERT
        assert(getCols()==m.getCols() && getRows()==m.getRows(),matrixSource,__LINE__);
#endif
        Matrix result(getRows(),getCols());
        for (int r=0;r<getRows();r++)
            for (int c=0;c<getCols();c++)
                result(r,c)=(*this)(r,c)-m(r,c);
        return result;
    }
    // +=
    Matrix& operator+=(const Matrix& m)
    {
#ifdef OOOARK_ASSERT
        assert(getCols()==m.getCols() && getRows()==m.getRows(),matrixSource,__LINE__);
#endif
        for (int r=0;r<getRows();r++)
            for (int c=0;c<getCols();c++)
                (*this)(r,c)+=m(r,c);
        return *this;
    }
    // -=
    Matrix& operator-=( const Matrix& m)
    {
#ifdef OOOARK_ASSERT
        assert(getCols()==m.getCols() && getRows()==m.getRows(),matrixSource,__LINE__);
#endif
        for (int r=0;r<getRows();r++)
            for (int c=0;c<getCols();c++)
                (*this)(r,c)-=m(r,c);
        return *this;
    }
    // mult. by a scalar
    const Matrix operator*(const dataType & s) const
    {
        Matrix result(getRows(),getCols());
        for (int r=0;r<getRows();r++)
            for (int c=0;c<getCols();c++)
                result(r,c)=(*this)(r,c)*s;
        return result;
    }
    // matrix mult.
    const Matrix operator*(Matrix& m) const
    {
#ifdef OOOARK_ASSERT
        assert(getCols()==m.getRows(),matrixSource,__LINE__);
#endif
        Matrix result(getRows(),m.getCols());
        for (int r=0;r<getRows();r++)
        {
            for (int c=0;c<m.getCols();c++)
            {
                for (int i=0;i<getCols();i++)
                {
                    result(r,c)+=(*this)(r,i)*m(i,c);
                }
            }
        }
        return result;
    }
    // vector mult.
    const Vector<dataType> operator*(const Vector<dataType> & v) const
    {
#ifdef OOOARK_ASSERT
        assert(getCols()==v.getSize(),matrixSource,__LINE__);
#endif
        Vector<dataType> result(getRows());
        for (int r=0;r<getRows();r++)
        {
            for (int c=0;c<getCols();c++)
            {
                result(r) += (*this)(r,c)*v(c);
            }
        }
        return result;
    }

    // printing
    void print(HardwareSerial & serial=Serial, const char * msg="") const
    {
        serial.println(msg);
        for (int i=0;i<getRows();i++)
        {
            for (int j=0;j<getCols();j++)
            {
                serial.print((*this)(i,j));
                serial.print(" ");
            }
            serial.println();
        }
    }
    // self test
    static bool selfTest()
    {
        // Matrix assignment/print test
        Serial.println("Matrix assignment/print test.");

        Matrix m(3,3);

        m(0,0) = 1;
        m(0,1) = 2;
        m(0,2) = 3;
        m(1,0) = 4;
        m(1,1) = 5;
        m(1,2) = 6;
        m(2,0) = 7;
        m(2,1) = 8;
        m(2,2) = 9;

        Serial.print("m size: ");
        Serial.print(m.getRows());
        Serial.print("x");
        Serial.println(m.getCols());
        m.print(Serial,"m: ");
        Serial.println("Should be 1-9 row major order.");
        Serial.println("Please verify.");
        Serial.println();

        // Matrix product test
        Serial.println("Matrix product test.");

        Matrix n(3,2);

        n(0,0) = 1;
        n(0,1) = 2;
        n(1,0) = 3;
        n(1,1) = 4;
        n(2,0) = 5;
        n(2,1) = 6;

        Matrix mn(3,2);

        mn(0,0) = 22;
        mn(0,1) = 28;
        mn(1,0) = 49;
        mn(1,1) = 64;
        mn(2,0) = 76;
        mn(2,1) = 100;

        n.print(Serial,"n: ");
        (m*n).print(Serial,"m*n: ");
        if (mn==(m*n)) Serial.println("PASSED");
        else
        {
            Serial.println("FAILED");
            return false;
        }
        Serial.println();
        return true;
    }
private:
    int numCols, numRows;
    dataType* data;
};

} // oooarkavr

#endif

// vim:ts=4:sw=4
