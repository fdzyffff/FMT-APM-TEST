/// @file	AC_P.cpp
/// @brief	Generic P algorithm

#include "AP_Math.h"
#include "AC_P.h"

float AC_P::get_p(float error) const
{
    return (float)error * _kp;
}

