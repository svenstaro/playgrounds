/* GlobalInclude.h -- Various includes and definitions needed by many files in this project.  

  Copyright (C) 2010 Allen Jordan

  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.

  Allen Jordan allen.jordan@gmail.com
*/

#pragma once

#include <stdexcept>
#include <iostream>
#include <fstream>
#include <vector>
#include <stdlib.h>

#include <SFML/Graphics.hpp>
#include <Box2D/Box2D.h>
#include <boost/foreach.hpp>

using namespace std;

//definitions for physics world <-> window projection coordinate transformation
#define PIXELS_PER_METER	64.0f
#define METERS_PER_PIXEL	(1.0f/PIXELS_PER_METER)
//shorthand version
#define PPM PIXELS_PER_METER
#define MPP METERS_PER_PIXEL
