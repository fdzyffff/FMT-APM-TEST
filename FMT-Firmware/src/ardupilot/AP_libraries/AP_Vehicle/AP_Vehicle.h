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
#pragma once

/*
  this header holds a parameter structure for each vehicle type for
  parameters needed by multiple libraries
 */

#include "AP_Param.h"

class AP_Vehicle {

public:
    /*
      common parameters for fixed wing aircraft
     */
    struct FixedWing {
    };

    /*
      common parameters for multicopters
     */
    struct MultiCopter {
        int16_t angle_max;
    };
};


#include "AP_Vehicle_Type.h"
