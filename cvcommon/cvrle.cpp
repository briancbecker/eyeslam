/*==================================================================================

    Filename:  cvrle.cpp

    Copyright 2007 Daniel Barber
                   Robotics Laboratory
                   University of Central Florida
                   http://robotics.ucf.edu

    Program Contents:
    -------------------------------------------------------------------------------
    Structure for storing Run Length Encoded data from an image for
    segmentation.
    -------------------------------------------------------------------------------

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA

==================================================================================*/
#include "cvrle.h"


//////////////////////////////////////////////////////////////////////////////
///
///   \brief Constructor.
///
//////////////////////////////////////////////////////////////////////////////
CvRLE::CvRLE() : y(0), x(0), width(0), color(0), parent(0) {}

//////////////////////////////////////////////////////////////////////////////
///
///   \brief Copy constructor.
///
//////////////////////////////////////////////////////////////////////////////
CvRLE::CvRLE(const CvRLE &another)
{
    y = another.y;
    x = another.x;
    width = another.width;
    color = another.color;
    parent = another.parent;
}



/* End of File */
