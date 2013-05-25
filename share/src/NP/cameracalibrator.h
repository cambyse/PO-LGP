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

/** \file cameracalibrator.h
    \brief Lens calibration for mono/stereo cameras */

#ifndef NP_CAMERACALIBRATOR_H
#define NP_CAMERACALIBRATOR_H

#include <MT/array.h>
#include <MT/vision.h>
#include <opencv/cv.h>
#include <opencv/cxtypes.h>

namespace np 
{
class CalibrationImage;
class Camera;
class CvWindow;


class CameraCalibrator:public CalibrationParameters
{
public:
  CameraCalibrator(Camera *c, uint x=9, uint y=6, double l=1., double scale=.6);
 ~CameraCalibrator();
  void                            run();

protected:
  void                            reset();
  int                             detect();
  int                             display();
  double                          calibrate();
  int                             rectify();
  int                             display_result();
  double                          scale_;

  Camera                         *camera_;
  CvWindow                       *cvwindow_;
  uint                            size_[2];
  double                          edge_;
  MT::Array<CalibrationImage*>    calibration_images_;
  //doubleA                         KL_, KR_, dL_, dR_, R_, T_, E_, F_;
  //doubleA                         RL_, RR_, PL_, PR_, Q_;

  void                            command_handler(const char *cmd);
};

} // namespace np

#endif
