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

#include "npors.h"
#include "nputils.h"
#include <string.h>

const double rad2deg = 180/cpi;
const double deg2rad = cpi/180;

void rotation_matrix(doubleA& R, double a, uint x = 0, uint y = 0, uint z = 0)
{
  if (R.N != 9)
  {
    R.resize(3,3);
    R = 0;
    R.setDiag(1);
  }

  if (x)
  {
    R(1,1) =  cos(a);
    R(1,2) =  sin(a);
    R(2,1) = -sin(a);
    R(2,2) =  cos(a);
  }

  if (y)
  {
    R(0,0) =  cos(a);
    R(0,2) = -sin(a);
    R(2,0) =  sin(a);
    R(2,2) =  cos(a);
  }

  if (z)
  {
    R(0,0) =  cos(a);
    R(0,1) =  sin(a);
    R(1,0) = -sin(a);
    R(1,1) =  cos(a);
  }
};

inline void get_angles(doubleA& angles, const doubleA& x1, const doubleA& x2)
{
  double x = x1(0)-x2(0);
  double y = x1(1)-x2(1);
  double z = x1(2)-x2(2);
  double ax = (y != 0. ? atan(z/y)*rad2deg : 0);
  double ay = (x != 0. ? atan(z/x)*rad2deg : 0.);
  double az = (y != 0. ? atan(x/y)*rad2deg : 0.);

  if (angles.N != 3)
    angles.resize(3);

  angles(0) = ax;
  angles(1) = ay;
  angles(2) = az;
};

inline void rotate(doubleA& xnew, const doubleA& R, const doubleA& x, const doubleA& rc)
{
  if (xnew.N != 3)
    xnew.resize(3);

  doubleA xtemp = x-rc, xrot(3);

  xrot = R*xtemp;
  xnew(0) = xrot(0) + rc(0);
  xnew(1) = xrot(1) + rc(1);
  xnew(2) = xrot(2) + rc(2);
};

inline void get_pos(doubleA& p, const ors::Body* b)
{
  if(p.N != 3)
    p.resize(3);

  p(0) = b->X.p.v[0];
  p(1) = b->X.p.v[1];
  p(2) = b->X.p.v[2];
//  p(3) = 1; 
};

void np::set_pos(ors::Body* b, const doubleA& p)
{
  b->X.p.v[0] = p(0);
  b->X.p.v[1] = p(1);
  b->X.p.v[2] = p(2);
};


void np::init_OrsStereoCamera
(
 OrsStereoCamera& cam, 
 double x,
 double y,
 double z,
 double fx,
 double fy,
 double fz,
 double ux,
 double uy,
 double uz,
 double baseline,
 double angle,
 double whRatio,
 uint w,
 uint h
)
{
  cam.pos.resize(3); cam.foc.resize(3); cam.up.resize(3);
  cam.pos(0) = x; cam.pos(1) = y; cam.pos(2) = z;
  cam.foc(0) = fx; cam.foc(1) = fy; cam.foc(2) = fz;
  cam.up(0) = ux; cam.up(1) = uy; cam.up(2) = uz;

  cam.baseline = baseline;
  cam.angle = angle;
  cam.w = w;
  cam.h = h;
  cam.whRatio = whRatio;
};


void np::gl_grab_stereo_image
(
 byteA& left,
 byteA& right,
 OpenGL& gl,
 const OrsStereoCamera& cam
)
{
  doubleA pos_orig(3), foc_orig(3);//, ha_orig;
  doubleA pos_old, pos_new, foc_new;
//   doubleA R, Rx, Ry, Rz;
  doubleA R, Rx, Rz;
  doubleA angles;
  doubleA cam2focus;
  float ha_orig, whRatio_orig;
  int w_orig, h_orig;
  
  if (left.N != cam.h*cam.w*3)
    left.resize(cam.h, cam.w, 3);
  if (right.N != cam.h*cam.w*3)
    right.resize(cam.h, cam.w, 3);

  // remember original camera settings
  ha_orig = gl.camera.heightAngle;
  w_orig = gl.width();
  h_orig = gl.height();
  whRatio_orig = gl.camera.whRatio;
  for (uint i = 0; i < 3; i++)
  {
    pos_orig(i) = gl.camera.f->p.v[i];
    foc_orig(i) = gl.camera.foc->v[i];
  }

  // compute rotation matrix
  get_angles(angles, cam.foc, cam.pos);
  rotation_matrix(Rx,-angles(0)*deg2rad,1,0,0);
//   rotation_matrix(Ry, angles(1)*deg2rad,0,1,0);
  rotation_matrix(Rz, angles(2)*deg2rad,0,0,1);
  R = Rz * Rx;
//   std::cout << "R = " << std::endl << R << std::endl;

  // change camera settings
  gl.resize(cam.w,cam.h);
  gl.camera.heightAngle = cam.angle;
  gl.update();


  // vector from center of camera to focus point
  cam2focus = cam.foc - cam.pos;

  // determine left focus
  pos_old = cam.pos;
  pos_old(0) -= cam.baseline/2;
  rotate(pos_new, R, pos_old, cam.pos);
  gl.camera.setPosition(pos_new(0), pos_new(1), pos_new(2));
  foc_new = pos_new + cam2focus;
  gl.camera.focus(foc_new(0), foc_new(1), foc_new(2));
  gl.update();

  gl.update();                                                // grab left frame
  glGrabImage(left);
  flip_image(left);

  // determine right focus
  pos_old = cam.pos;
  pos_old(0) += cam.baseline/2;
  rotate(pos_new, R, pos_old, cam.pos);
  gl.camera.setPosition(pos_new(0), pos_new(1), pos_new(2));
  foc_new = pos_new + cam2focus;
  gl.camera.focus(foc_new(0), foc_new(1), foc_new(2));
  gl.update();

  gl.update();                                                // grab left frame
  glGrabImage(right);
  flip_image(right);

  // restore original camera settings
  gl.camera.heightAngle = ha_orig;
  gl.resize(w_orig, h_orig);
  gl.camera.whRatio = whRatio_orig;
  for (uint i = 0; i < 3; i++)
  {
    gl.camera.f->p.v[i] = pos_orig(i);
    gl.camera.foc->v[i] = foc_orig(i);
  }
  gl.update();

//      floatA centers, r;
//      np::track_red_ball(centers, r, left, true);
//      np::display("window", left, 10);

//      np::remap(left_dist, left, mapx, mapy);
//      getHsvCenter(left_center, left, 2, TUPLE<float>(.0,1.,1.),TUPLE<float>(.2,.5,.5));
////      std::cout << "left_center = " << left_center << std::endl;
//      np::save_image(left_dist, "z.left.png");
//      rgb2rgba(test, left_dist, 255);
//      revel.addFrame(test.p);
}

ors::Body* np::get_chessboard(floatA& world_coords, uint nx, uint ny, double l)
{
  double cx = ((double)nx)*l/2;
  double cy = ((double)ny)*l/2;
  ors::Body *chessboard = new ors::Body();
  chessboard->name = "chessboard";
  chessboard->X.setText("<t(0 0 0)>");

  double offsetx = -cx+l/2, offsety = -cy+l/2;
  for (uint y=0; y < ny; y++)
  {
    for (uint x=0; x < nx; x++)
    {
      ors::Shape *s = new ors::Shape;
      chessboard->shapes.append(s);
      s->body=chessboard;
      s->type = boxST;
      s->size[0]=l; s->size[1]=l; s->size[2]=.01; s->size[3]=.1;
      if ((y+x)%2 == 0) {s->color[0]=0.; s->color[1]=0.; s->color[2]=0.;}
      else {s->color[0]=1.; s->color[1]=1.; s->color[2]=1.;}

      s->rel.p(0)=offsetx; s->rel.p(1)=offsety; s->rel.p(2)=0.;
      offsetx+=l;
    }
    offsety +=  l;
    offsetx  = -cx+l/2;
  }

  // set up 3D coordinates of chessboard corners
  // NOTE These coordinates do NOT correspond to the actual world coordinates,
  //      but instead they capture the geometric structure of the calibration
  //      object. This corresponds to assuming that the chessboards remains at
  //      a fixed position and the camera moves, opposed to moving chessboard
  //      and fixed camera.
  uint counter = 0;
  world_coords.resize((nx-1)*(ny-1),3);               // world coordinates of each
                                                    // inner corner used during
                                                    // calibration
  for (uint y = 0; y < ny-1; y++)
    for (uint x = 0; x < nx-1; x++)
    {
      world_coords(counter,0) = ((float)x)*l;
      world_coords(counter,1) = ((float)y)*l;
      world_coords(counter,2) = 0.;
      counter++;
    }

  return chessboard;
};

void np::get_chessboard(ors::Body*& chessboard, floatA& world_coords, uint nx, uint ny, double l)
{
  double cx = ((double)nx)*l/2;
  double cy = ((double)ny)*l/2;
//  ors::Body *chessboard = new ors::Body();
//  chessboard->name = "chessboard";
//  chessboard->X.setText("<t(0 0 0)>");

  std::ostringstream oss;
  double offsetx = -cx+l/2, offsety = -cy+l/2;
  for (uint y=0; y < ny; y++)
  {
    for (uint x=0; x < nx; x++)
    {
      ors::Shape *s = new ors::Shape;
      chessboard->shapes.append(s);
      s->body=chessboard;
      s->type = boxST;
      s->size[0]=l; s->size[1]=l; s->size[2]=.01; s->size[3]=.1;
      if ((y+x)%2 == 0) {s->color[0]=0.; s->color[1]=0.; s->color[2]=0.;}
      else {s->color[0]=1.; s->color[1]=1.; s->color[2]=1.;}
      s->rel.p(0)=offsetx; s->rel.p(1)=offsety; s->rel.p(2)=.0;

      // add a inner-corner shape to query 3D world coordinates during sim
      if (y < (ny-1) && x < (nx-1))
      {
        ors::Shape *c = new ors::Shape;
        chessboard->shapes.append(c);
        oss.str("");
        oss << (y)*(nx-1)+x;
        std::cout << oss.str() << std::endl;
        c->body=chessboard;
        c->name=oss.str().c_str();
        c->type = sphereST;
#ifndef NP_DEBUG_CHESSBOARD
        c->size[0]=.001; c->size[1]=.001; c->size[2]=.001; c->size[3]=.001;
#else
        c->size[0]=.05; c->size[1]=.05; c->size[2]=.05; c->size[3]=.05;
#endif
        c->color[0]=1.; c->color[1]=.0; c->color[2]=.0;
        c->rel.p(0)=offsetx+(l/2); c->rel.p(1)=offsety+(l/2); c->rel.p(2)=.0;
      }

      offsetx+=l;
    }
    offsety +=  l;
    offsetx  = -cx+l/2;
  }

  // set up 3D coordinates of chessboard corners
  // NOTE These coordinates do NOT correspond to the actual world coordinates,
  //      but instead they capture the geometric structure of the calibration
  //      object. This corresponds to assuming that the chessboards remains at
  //      a fixed position and the camera moves, opposed to moving chessboard
  //      and fixed camera.
  uint counter = 0;
  world_coords.resize((nx-1)*(ny-1),3);               // world coordinates of each
                                                    // inner corner used during
                                                    // calibration
  for (uint y = 0; y < ny-1; y++)
    for (uint x = 0; x < nx-1; x++)
    {
      world_coords(counter,0) = ((float)x)*l;
      world_coords(counter,1) = ((float)y)*l;
      world_coords(counter,2) = 0.;
      counter++;
    }
};

void np::get_chessboard_corners_3d(floatA& world_coords, const ors::Body* chessboard, uint nx, uint ny)
{
  uint inx = nx-1, iny = ny-1;
  std::ostringstream oss;
  floatA coords(3);
  world_coords.clear();
  world_coords.reshape(0,3);
  for (uint y=0; y < iny; y++)
  {
    for (uint x=0; x < inx; x++)
    {
        oss.str("");
        oss << y*inx+x;
        for (uint i = 0; i < chessboard->shapes.N; i++)
        {
          if(strcmp(oss.str().c_str(), chessboard->shapes(i)->name) == 0)
          {
            for (uint j = 0; j < 3; j++)
              coords(j) = chessboard->shapes(i)->X.p.v[j];
            world_coords.append(coords);
          }
        }
      }
    }
}

std::ostream& operator<<(std::ostream& os,const np::OrsStereoCamera& osc)
{
  os << "pos:      " << osc.pos << std::endl
     << "foc:      " << osc.foc << std::endl
     << "up:       " << osc.up  << std::endl
     << "baseline: " << osc.baseline << std::endl
     << "angle:    " << osc.angle << std::endl
     << "width:    " << osc.w << std::endl
     << "height:   " << osc.h << std::endl;

  return os;
}


