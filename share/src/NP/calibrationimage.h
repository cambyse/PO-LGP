/*  Copyright 2010 Nils Plath
    email: nilsp@cs.tu-berlin.de

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/> */

/*! \file calibrationimage.h
    \brief Data structure for camera calibration. */

#ifndef NP_CALIBRATIONIMAGE_H
#define NP_CALIBRATIONIMAGE_H

#include <MT/array.h>
#include "cvutils.h"
namespace np {

class CalibrationImage
{
public:
  CalibrationImage(byteA& img, uint sizex, uint sizey, double edge);
 ~CalibrationImage();
  bool                    found_pattern();
  void                    draw(byteA& canvas);
  doubleA                 get_corners();

//protected:
  byteA                   image_;
  uint                    size_[2];
  double                  edge_;
  doubleA                 corners_;
  std::vector<CvPoint2D32f> corners2_;
};

}; // namespace np

#endif
