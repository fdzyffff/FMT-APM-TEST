/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// Copyright 2010 Michael Smith, all rights reserved.

// Derived closely from:
/****************************************
* 2D Vector Classes
* By Bill Perone (billperone@yahoo.com)
* Original: 9-16-2002
* Revised: 19-11-2003
*          18-12-2003
*          06-06-2004
*
* � 2003, This code is provided "as is" and you can use it freely as long as
* credit is given to Bill Perone in the application it is used in
****************************************/
#pragma once

#include <cmath>
#include "float.h"

template <typename T>
struct Vector2
{
    T x, y;

    // trivial ctor
    constexpr Vector2<T>()
        : x(0)
        , y(0) {}

    // setting ctor
    constexpr Vector2<T>(const T x0, const T y0)
        : x(x0)
        , y(y0) {}

    // function call operator
    void operator ()(const T x0, const T y0)
    {
        x= x0; y= y0;
    }

    // test for equality
    bool operator ==(const Vector2<T> &v) const;

    // test for inequality
    bool operator !=(const Vector2<T> &v) const;

    // negation
    Vector2<T> operator -(void) const;

    // addition
    Vector2<T> operator +(const Vector2<T> &v) const;

    // subtraction
    Vector2<T> operator -(const Vector2<T> &v) const;

    // uniform scaling
    Vector2<T> operator *(const T num) const;

    // uniform scaling
    Vector2<T> operator  /(const T num) const;

    // addition
    Vector2<T> &operator +=(const Vector2<T> &v);

    // subtraction
    Vector2<T> &operator -=(const Vector2<T> &v);

    // uniform scaling
    Vector2<T> &operator *=(const T num);

    // uniform scaling
    Vector2<T> &operator /=(const T num);

    // dot product
    T operator *(const Vector2<T> &v) const;

    // cross product
    T operator %(const Vector2<T> &v) const;

    // computes the angle between this vector and another vector
    // returns 0 if the vectors are parallel, and M_PI if they are antiparallel
    float angle(const Vector2<T> &v2) const;

    // computes the angle in radians between the origin and this vector
    T angle(void) const;

    // check if any elements are NAN
    bool is_nan(void) const;

    // check if any elements are infinity
    bool is_inf(void) const;

    // check if all elements are zero
    bool is_zero(void) const { return (fabsf(x) < FLT_EPSILON) && (fabsf(y) < FLT_EPSILON); }

    const T & operator[](uint8_t i) const {
        const T *_v = &x;
#if MATH_CHECK_INDEXES
        assert(i >= 0 && i < 2);
#endif
        return _v[i];
    }

    // zero the vector
    void zero()
    {
        x = y = 0;
    }

    // gets the length of this vector squared
    T   length_squared() const
    {
        return (T)(*this * *this);
    }

    // gets the length of this vector
    float           length(void) const;

    // normalizes this vector
    void    normalize()
    {
        *this/=length();
    }

    // returns the normalized vector
    Vector2<T>  normalized() const
    {
        return *this/length();
    }

    // reflects this vector about n
    void    reflect(const Vector2<T> &n)
    {
        Vector2<T>        orig(*this);
        project(n);
        *this= *this*2 - orig;
    }

    // projects this vector onto v
    void    project(const Vector2<T> &v)
    {
        *this= v * (*this * v)/(v*v);
    }

    // returns this vector projected onto v
    Vector2<T>  projected(const Vector2<T> &v)
    {
        return v * (*this * v)/(v*v);
    }

    // given a position p1 and a velocity v1 produce a vector
    // perpendicular to v1 maximising distance from p1
    static Vector2<T> perpendicular(const Vector2<T> &pos_delta, const Vector2<T> &v1)
    {
        Vector2<T> perpendicular1 = Vector2<T>(-v1[1], v1[0]);
        Vector2<T> perpendicular2 = Vector2<T>(v1[1], -v1[0]);
        T d1 = perpendicular1 * pos_delta;
        T d2 = perpendicular2 * pos_delta;
        if (d1 > d2) {
            return perpendicular1;
        }
        return perpendicular2;
    }

    /*
     * Returns the point closest to p on the line segment (v,w).
     *
     * Comments and implementation taken from
     * http://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
     */
    static Vector2<T> closest_point(const Vector2<T> &p, const Vector2<T> &v, const Vector2<T> &w)
    {
        // length squared of line segment
        const float l2 = (v - w).length_squared();
        if (l2 < FLT_EPSILON) {
            // v == w case
            return v;
        }
        // Consider the line extending the segment, parameterized as v + t (w - v).
        // We find projection of point p onto the line.
        // It falls where t = [(p-v) . (w-v)] / |w-v|^2
        // We clamp t from [0,1] to handle points outside the segment vw.
        const float t = ((p - v) * (w - v)) / l2;
        if (t <= 0) {
            return v;
        } else if (t >= 1) {
            return w;
        } else {
            return v + (w - v)*t;
        }
    }

    // w defines a line segment from the origin
    // p is a point
    // returns the closest distance between the radial and the point
    static float closest_distance_between_radial_and_point(const Vector2<T> &w,
                                                           const Vector2<T> &p)
    {
        const Vector2<T> closest = closest_point(p, Vector2<T>(0,0), w);
        const Vector2<T> delta = closest - p;
        return delta.length();
    }

};

typedef Vector2<int16_t>        Vector2i;
typedef Vector2<uint16_t>       Vector2ui;
typedef Vector2<int32_t>        Vector2l;
typedef Vector2<uint32_t>       Vector2ul;
typedef Vector2<float>          Vector2f;
