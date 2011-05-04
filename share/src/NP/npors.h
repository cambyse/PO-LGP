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

/*! \file npors.h
    \brief Routines for and around Marc's ORS and ORS-OpenGL code */

#ifndef _NP_CVORS_H
#define _NP_CVORS_H

#include<MT/array.h>
#include<MT/opengl.h>
#include<MT/ors.h>

namespace np {

struct OrsStereoCamera
{
  doubleA pos;      // position, focus, and up-vector of center (!) of camera
  doubleA foc;
  doubleA up;
  double baseline;
  double angle;
  uint w,h;
  double whRatio;
};

void set_pos(ors::Body* b, const doubleA& p);

void init_OrsStereoCamera
(
 OrsStereoCamera& cam, 
 double x         = 0,
 double y         = -1,
 double z         = 0,
 double fx        = 0,
 double fy        = 0,
 double fz        = 0,
 double ux        = 0,
 double uy        = 1,
 double uz        = 0,
 double baseline  = 0.1,
 double angle     = 36,
 double whRatio   = 1.0,
 uint w           = 640,
 uint h           = 480
);

void gl_grab_stereo_image
(
 byteA& left,
 byteA& right,
 OpenGL& opengl,
 const OrsStereoCamera& cam
);

ors::Body* get_chessboard(floatA& world_coords, uint nx, uint ny, double l);
void get_chessboard(ors::Body*& chessboard, floatA& world_coords, uint nx, uint ny, double l);
void get_chessboard_corners_3d(floatA& world_coords, const ors::Body* chessboard, uint nx, uint ny);
} // namespace np

std::ostream& operator<<(std::ostream& os,const np::OrsStereoCamera& osc);

#endif
