/*  ---------------------------------------------------------------------
    Copyright 2012 Marc Toussaint
    email: mtoussai@cs.tu-berlin.de

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */

/**
 * @file
 * @ingroup group_geo
 */
/**
 * @ingroup group_geo
 * @{
 */


#include "geo.h"
#include "opengl.h"

//global options
bool orsDrawJoints=true, orsDrawShapes=true, orsDrawBodies=true, orsDrawProxies=true;
bool orsDrawMeshes=true, orsDrawWires=false, orsDrawZlines=false;
double orsDrawAlpha=1.00;
uint orsDrawLimit=0;

#ifdef MT_GL
#  include <GL/gl.h>
#  include <GL/glu.h>

extern void glDrawRect(float, float, float, float, float, float,
                       float, float, float, float, float, float);

extern void glDrawText(const char* txt, float x, float y, float z);

//! static GL routine to draw a ors::Mesh
void ors::glDrawMesh(void *classP) {
  ((ors::Mesh*)classP)->glDraw();
}

//! GL routine to draw a ors::Mesh
void ors::Mesh::glDraw() {
  if(V.d0!=Vn.d0 || T.d0!=Tn.d0) {
    computeNormals();
  }
  if(orsDrawWires) {
#if 0
    uint t;
    for(t=0; t<T.d0; t++) {
      glBegin(GL_LINE_LOOP);
      glVertex3dv(&V(T(t, 0), 0));
      glVertex3dv(&V(T(t, 1), 0));
      glVertex3dv(&V(T(t, 2), 0));
      glEnd();
    }
#else
    glEnableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
    if(C.N) glEnableClientState(GL_COLOR_ARRAY); else glDisableClientState(GL_COLOR_ARRAY);
    glVertexPointer(3, GL_DOUBLE, 0, V.p);
    if(C.N) glColorPointer(3, GL_DOUBLE, 0, C.p);
    glDrawElements(GL_LINE_STRIP, T.N, GL_UNSIGNED_INT, T.p);
    //glDrawArrays(GL_LINE_STRIP, 0, V.d0);
#endif
    return;
  }
#if 1
  if(!GF.N) { //no group frames  ->  use OpenGL's Arrays for fast drawing...
    GLboolean turnOnLight=false;
    if(C.N){ glGetBooleanv(GL_LIGHTING, &turnOnLight); glDisable(GL_LIGHTING); }

    glShadeModel(GL_FLAT);
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);
    if(C.N) glEnableClientState(GL_COLOR_ARRAY); else glDisableClientState(GL_COLOR_ARRAY);
    glVertexPointer(3, GL_DOUBLE, 0, V.p);
    if(C.N) glColorPointer(3, GL_DOUBLE, 0, C.p);
    glNormalPointer(GL_DOUBLE, 0, Vn.p);

    glDrawElements(GL_TRIANGLES, T.N, GL_UNSIGNED_INT, T.p);

    if(turnOnLight){ glEnable(GL_LIGHTING); }
  } else {
    int g;
    uint v, t, i, j;
    double GLmatrix[16];
    Vector w;
    if(!GT.N) {
      for(t=0; t<T.d0; t++) {
        glPushName(t <<4);
        glBegin(GL_TRIANGLES);
        for(j=0; j<3; j++) {
          v=T(t, j);
          if(G.N) g=G(v); else g=-1;
          w.set(&Vn(v, 0));
          if(g!=-1) w=GF(g)->rot*w;
          glNormal3dv(w.p());
          if(C.N) glColor3dv(&C(v, 0));
          w.set(&V(v, 0));
          if(g!=-1) w=GF(g)->pos+GF(g)->rot*w;
          glVertex3dv(w.p());
        }
        glEnd();
        glPopName();
      }
    } else {
      //faces that belong to one group only
      for(g=0; g<(int)GT.N-1; g++) {
        glPushMatrix();
        GF(g)->getAffineMatrixGL(GLmatrix);
        glLoadMatrixd(GLmatrix);
        glBegin(GL_TRIANGLES);
        for(i=0; i<GT(g).N; i++) {
          t=GT(g)(i);
          for(j=0; j<3; j++) {
            v=T(t, j);
            glNormal3dv(&Vn(v, 0));
            if(C.N) glColor3dv(&C(v, 0));
            glVertex3dv(&V(v, 0));
          }
        }
        glEnd();
        glPopMatrix();
      }
      //faces with vertices from different groups (transform each vertex individually)
      glBegin(GL_TRIANGLES);
      for(i=0; i<GT(GT.N-1).N; i++) {
        t=GT(GT.N-1)(i);
        for(j=0; j<3; j++) {
          v=T(t, j);
          g=G(v);
          w.set(&Vn(v, 0));  if(g!=-1) w=GF(g)->rot*w;  glNormal3dv(w.p());
          if(C.N) glColor3dv(&C(v, 0));
          w.set(&V(v, 0));  if(g!=-1) w=GF(g)->pos+GF(g)->rot*w;  glVertex3dv(w.p());
        }
      }
      glEnd();
    }
    /*for(j=0;j<strips.N;j++){
      glBegin(GL_TRIANGLE_STRIP);
      for(i=0;i<strips(j).N;i++){
      glNormal3dv(&N(strips(j)(i), 0));
      if(C.N) glColor3fv(C(strips(j)(i)));
      glVertex3dv(&V(strips(j)(i), 0));
    }
      glEnd();
    }*/
  }
#elif 0 //simple with vertex normals
  uint i, v;
  glShadeModel(GL_SMOOTH);
  glBegin(GL_TRIANGLES);
  for(i=0; i<T.d0; i++) {
    if(C.d0==T.d0)  glColor(C(t, 0), C(t, 1), C(t, 2),1.);
    v=T(i, 0);  glNormal3dv(&Vn(v, 0));  if(C.d0==V.d0) glColor3dv(&C(v, 0));  glVertex3dv(&V(v, 0));
    v=T(i, 1);  glNormal3dv(&Vn(v, 0));  if(C.d0==V.d0) glColor3dv(&C(v, 0));  glVertex3dv(&V(v, 0));
    v=T(i, 2);  glNormal3dv(&Vn(v, 0));  if(C.d0==V.d0) glColor3dv(&C(v, 0));  glVertex3dv(&V(v, 0));
  }
  glEnd();
#else //simple with triangle normals
  uint t, v;
  computeNormals();
  glBegin(GL_TRIANGLES);
  for(t=0; t<T.d0; t++) {
    glNormal3dv(&Tn(t, 0));
    if(C.d0==T.d0)  glColor(C(t, 0), C(t, 1), C(t, 2),1.);
    v=T(t, 0);  if(C.d0==V.d0) glColor3dv(&C(v, 0));  glVertex3dv(&V(v, 0));
    v=T(t, 1);  if(C.d0==V.d0) glColor3dv(&C(v, 0));  glVertex3dv(&V(v, 0));
    v=T(t, 2);  if(C.d0==V.d0) glColor3dv(&C(v, 0));  glVertex3dv(&V(v, 0));
  }
  glEnd();
#if 0 //draw normals
  glColor(.5, 1., .0);
  Vector a, b, c, x;
  for(i=0; i<T.d0; i++) {
    glBegin(GL_LINES);
    a.set(&V(T(i, 0), 0)); b.set(&V(T(i, 1), 0)); c.set(&V(T(i, 2), 0));
    x.setZero(); x+=a; x+=b; x+=c; x/=3;
    glVertex3dv(x.v);
    a.set(&Tn(i, 0));
    x+=.05*a;
    glVertex3dv(x.v);
    glEnd();
  }
#endif
#endif
}

#else //!MT_GL
void ors::Mesh::glDraw() { NICO }
void ors::glDrawMesh(void*) { NICO }
#endif
/** @} */
