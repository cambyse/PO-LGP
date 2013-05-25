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

/** \file camera.h
    \brief Access to webcams (USB/UVC) and to PTG Bumblebee2 (libdc1394) */

#ifndef NP_CAMERA_H
#define NP_CAMERA_H

#include <MT/array.h>
#include <system/module.h>
#include <MT/robot_variables.h>


//===========================================================================
//
// CAMERA CALIBRATION
//

struct CalibrationParameters{
  //output of cvStereoCalibrate
  arr cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2;
  arr R,T,E,F;
  //output of cvStereoRectify
  arr R1,R2,P1,P2,Q;
  //output of cvInitUndistortRectifyMap
  floatA map1L, map2L, map1R, map2R;

  void rectifyImages(byteA &imgL, byteA& imgR);
  void stereo2world(floatA& world,const floatA& stereo);
  void write(std::ostream& os) const;
  void read(std::istream& is);
};

struct sCameraModule;

struct CameraModule:public Process{
  CameraImages *output;

  sCameraModule *s;
  CalibrationParameters calib;

  CameraModule();
  void open();
  void step();
  void close();
};


namespace np {
// struct DistFov;
// typedef MT::Array<DistFov*> DistFovList;
struct DistPoly
{
  doubleA           KL, KR, dL, dR, R, T, E, F;
  doubleA           RL, RR, PL, PR, Q;
  floatA            mapxL, mapyL;
  floatA            mapxR, mapyR;
};
// typedef MT::Array<DistPoly*> DistPolyList;

enum CamId
{
  RIGHT=0,
  LEFT=1
};

class Camera
{
public:
  Camera(){};
  ~Camera(){};
  virtual void      init() = 0;
  virtual void      deinit() = 0;
  virtual void      start_capturing() = 0;
  virtual void      stop_capturing() = 0;

  virtual void      rectify(bool r=false) = 0;
  virtual void      grab(byteA& rgb) = 0;
  virtual void      grab(byteA& rgbL, byteA& rgbR) = 0;

  virtual void      load_parameters(const char *filename) = 0;

  virtual void      distort(doubleA& xd, const doubleA& x, CamId c=RIGHT) = 0;
  virtual void      undistort(doubleA& xu, const doubleA& x, CamId c=RIGHT) = 0;
  virtual bool      is_stereo() = 0;

// protected:
  uint              width_, height_;
  bool              rectify_;
  DistPoly          distpoly_;
//   DistFovList       distfov_;           // one (mono) or more (stereo/...)
//                                         // FOV distortion models
};

struct Bumblebee2WS;

class Bumblebee2 : public Camera {
public:
  Bumblebee2();
  ~Bumblebee2();

  void              init();
  void              init(uint64_t identifier);
  void              init(const char* identifier);
  void              deinit();
  void              start_capturing();
  void              stop_capturing();
  void              rectify(bool r);
  void              grab(byteA& rgb);
  void              grab(byteA& rgbL, byteA& rgbR);
  void              load_parameters(const char *filename);
  void              distort(doubleA& xd, const doubleA& x, CamId c=RIGHT);
  void              undistort(doubleA& xu, const doubleA& x, CamId c=RIGHT);
  bool              is_stereo();
protected:
  Bumblebee2WS     *ws_;
};

}; // namespace np

#endif
