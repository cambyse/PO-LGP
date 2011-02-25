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

#include "calibrationimage.h"
#include "nputils.h"
#include "cvutils.h"
#include "opencv_helper.h"

np::CalibrationImage::CalibrationImage(byteA& img, uint sizex, uint sizey, double edge)
 : edge_(edge)
{
  if (img.nd!=2)
    msg_error(HERE, "input image has to be a BW/grayscale image");

  size_[0] = sizex;
  size_[1] = sizey;

  // detect chessboard pattern
//  std::vector<CvPoint2D32f> corners(size_[0]*size_[1]);
  int num_corners=0, found=0;
  corners2_.resize(size_[0]*size_[1]);
  found=cvFindChessboardCorners(img, cvSize(size_[0], size_[1]),
          &corners2_[0], &num_corners,
          CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);

  // return if nothing has been found
  if (found!=1)
    return;

  // determine corners with subpixel accuracy
  cvFindCornerSubPix(img, &corners2_[0], num_corners, cvSize(5, 5),
    cvSize(-1,-1), cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 30, 0.01));

  // store corner points
  array2array(image_,img);
//  corners2_ = corners;
};

np::CalibrationImage::~CalibrationImage()
{corners2_.clear(); image_.clear();};

bool np::CalibrationImage::found_pattern()
{
  //return (corners_.d0>0 && corners_.d0==(size_[0]*size_[1]));
  return ((corners2_.size()>0 && corners2_.size()==(size_[0]*size_[1])));
};

void np::CalibrationImage::draw(byteA& canvas)
{
  cvDrawChessboardCorners(canvas, cvSize(size_[0], size_[1]), &corners2_[0], (size_[0]*size_[1]), 1);
};

doubleA np::CalibrationImage::get_corners()
{return corners_;};
