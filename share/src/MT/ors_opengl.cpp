/*  Copyright 2009 Marc Toussaint
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
    along with this program. If not, see <http://www.gnu.org/licenses/> */

#include "ors.h"
#include "opengl.h"

#ifdef MT_GL
#  include <GL/gl.h>
#  include <GL/glu.h>

//global options
bool orsDrawJoints=true, orsDrawShapes=true, orsDrawBodies=true, orsDrawProxies=true;
bool orsDrawMeshes=true, orsDrawWires=false;
double orsDrawAlpha=1.00;
uint orsDrawLimit=0;

extern void glDrawRect(float, float, float, float, float, float,
                       float, float, float, float, float, float);

extern void glDrawText(const char* txt, float x, float y, float z);

//void glColor(float *rgb);//{ glColor(rgb[0], rgb[1], rgb[2], 1.); }

//! static GL routine to draw a ors::Mesh
void ors::glDrawMesh(void *classP){
  glDraw(*((ors::Mesh*)classP));
}

//! GL routine to draw a ors::Mesh
void ors::glDraw(ors::Mesh& mesh){
  if(mesh.V.d0!=mesh.Vn.d0 || mesh.T.d0!=mesh.Tn.d0){
    mesh.computeNormals();
  }
  if(orsDrawWires){
#if 0
    uint t;
    for(t=0; t<mesh.T.d0; t++){
      glBegin(GL_LINE_LOOP);
      glVertex3dv(&mesh.V(mesh.T(t, 0), 0));
      glVertex3dv(&mesh.V(mesh.T(t, 1), 0));
      glVertex3dv(&mesh.V(mesh.T(t, 2), 0));
      glEnd();
    }
#else
    glEnableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
    if(mesh.C.N) glEnableClientState(GL_COLOR_ARRAY); else glDisableClientState(GL_COLOR_ARRAY);
    glVertexPointer(3, GL_DOUBLE, 0, mesh.V.p);
    if(mesh.C.N) glColorPointer(3, GL_DOUBLE, 0, mesh.C.p);
    glDrawElements(GL_LINE_STRIP, mesh.T.N, GL_UNSIGNED_INT, mesh.T.p);
    //glDrawArrays(GL_LINE_STRIP, 0, mesh.V.d0);
#endif
    return;
  }
#if 1
  if(!mesh.GF.N){ //no group frames  ->  use OpenGL's Arrays for fast drawing...
    glShadeModel(GL_SMOOTH);
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);
    if(mesh.C.N) glEnableClientState(GL_COLOR_ARRAY); else glDisableClientState(GL_COLOR_ARRAY);
    glVertexPointer(3, GL_DOUBLE, 0, mesh.V.p);
    if(mesh.C.N) glColorPointer(3, GL_DOUBLE, 0, mesh.C.p);
    glNormalPointer(GL_DOUBLE, 0, mesh.Vn.p);
    
    glDrawElements(GL_TRIANGLES, mesh.T.N, GL_UNSIGNED_INT, mesh.T.p);
  }else{
    int g;
    uint v, t, i, j;
    double GLmatrix[16];
    Vector w;
    if(!mesh.GT.N){
      for(t=0; t<mesh.T.d0; t++){
        glPushName(t <<4);
        glBegin(GL_TRIANGLES);
        for(j=0; j<3; j++){
          v=mesh.T(t, j);
          if(mesh.G.N) g=mesh.G(v); else g=-1;
          w.set(&mesh.Vn(v, 0));
          if(g!=-1) w=mesh.GF(g)->rot*w;
          glNormal3dv(w.p);
          if(mesh.C.N) glColor3dv(&mesh.C(v, 0));
          w.set(&mesh.V(v, 0));
          if(g!=-1) w=mesh.GF(g)->pos+mesh.GF(g)->rot*w;
          glVertex3dv(w.p);
        }
        glEnd();
        glPopName();
      }
    }else{
      //faces that belong to one group only
      for(g=0; g<(int)mesh.GT.N-1; g++){
        glPushMatrix();
        mesh.GF(g)->getAffineMatrixGL(GLmatrix);
        glLoadMatrixd(GLmatrix);
        glBegin(GL_TRIANGLES);
        for(i=0; i<mesh.GT(g).N; i++){
          t=mesh.GT(g)(i);
          for(j=0; j<3; j++){
            v=mesh.T(t, j);
            glNormal3dv(&mesh.Vn(v, 0));
            if(mesh.C.N) glColor3dv(&mesh.C(v, 0));
            glVertex3dv(&mesh.V(v, 0));
          }
        }
        glEnd();
        glPopMatrix();
      }
      //faces with vertices from different groups (transform each vertex individually)
      glBegin(GL_TRIANGLES);
      for(i=0; i<mesh.GT(mesh.GT.N-1).N; i++){
        t=mesh.GT(mesh.GT.N-1)(i);
        for(j=0; j<3; j++){
          v=mesh.T(t, j);
          g=mesh.G(v);
          w.set(&mesh.Vn(v, 0));  if(g!=-1) w=mesh.GF(g)->rot*w;  glNormal3dv(w.p);
          if(mesh.C.N) glColor3dv(&mesh.C(v, 0));
          w.set(&mesh.V(v, 0));  if(g!=-1) w=mesh.GF(g)->pos+mesh.GF(g)->rot*w;  glVertex3dv(w.p);
        }
      }
      glEnd();
    }
    /*for(j=0;j<strips.N;j++){
      glBegin(GL_TRIANGLE_STRIP);
      for(i=0;i<strips(j).N;i++){
      glNormal3dv(&mesh.N(strips(j)(i), 0));
      if(mesh.C.N) glColor3fv(mesh.C(strips(j)(i)));
      glVertex3dv(&mesh.V(strips(j)(i), 0));
    }
      glEnd();
    }*/
  }
#elif 0 //simple with vertex normals
  uint i, v;
  glShadeModel(GL_SMOOTH);
  glBegin(GL_TRIANGLES);
  for(i=0; i<mesh.T.d0; i++){
    v=mesh.T(i, 0);  glNormal3dv(&mesh.Vn(v, 0));  if(mesh.C.N) glColor3dv(&mesh.C(v, 0));  glVertex3dv(&mesh.V(v, 0));
    v=mesh.T(i, 1);  glNormal3dv(&mesh.Vn(v, 0));  if(mesh.C.N) glColor3dv(&mesh.C(v, 0));  glVertex3dv(&mesh.V(v, 0));
    v=mesh.T(i, 2);  glNormal3dv(&mesh.Vn(v, 0));  if(mesh.C.N) glColor3dv(&mesh.C(v, 0));  glVertex3dv(&mesh.V(v, 0));
  }
  glEnd();
#else //simple with triangle normals
  uint i, v;
  mesh.computeNormals();
  glBegin(GL_TRIANGLES);
  for(i=0; i<mesh.T.d0; i++){
    glNormal3dv(&mesh.Tn(i, 0));
    v=mesh.T(i, 0);  if(mesh.C.N) glColor3dv(&mesh.C(v, 0));  glVertex3dv(&mesh.V(v, 0));
    v=mesh.T(i, 1);  if(mesh.C.N) glColor3dv(&mesh.C(v, 0));  glVertex3dv(&mesh.V(v, 0));
    v=mesh.T(i, 2);  if(mesh.C.N) glColor3dv(&mesh.C(v, 0));  glVertex3dv(&mesh.V(v, 0));
  }
  glEnd();
#if 0 //draw normals
  glColor(.5, 1., .0);
  Vector a, b, c, x;
  for(i=0; i<mesh.T.d0; i++){
    glBegin(GL_LINES);
    a.set(&mesh.V(mesh.T(i, 0), 0)); b.set(&mesh.V(mesh.T(i, 1), 0)); c.set(&mesh.V(mesh.T(i, 2), 0));
    x.setZero(); x+=a; x+=b; x+=c; x/=3;
    glVertex3dv(x.v);
    a.set(&mesh.Tn(i, 0));
    x+=.05*a;
    glVertex3dv(x.v);
    glEnd();
  }
#endif
#endif
}

#ifndef MT_ORS_ONLY_BASICS
//! static GL routine to draw a ors::Graph
void ors::glDrawGraph(void *classP){
  ors::glDraw(*((ors::Graph*)classP));
}

void glDrawShape(ors::Shape *s, const ors::Transformation& X){
  //set name (for OpenGL selection)
  glPushName((s->index <<2) | 1);
  glColor(s->color[0], s->color[1], s->color[2], orsDrawAlpha);
  
  double scale=.33*(s->size[0]+s->size[1]+s->size[2] + 2.*s->size[3]); //some scale
  if(!scale) scale=1.;
  scale*=.3;
  
  double GLmatrix[16];
  ors::Transformation f;
  f = X;
  f.appendTransformation(s->rel);
  f.getAffineMatrixGL(GLmatrix);
  glLoadMatrixd(GLmatrix);
  
  if(!orsDrawShapes){
    glDrawAxes(scale);
    glColor(0, 0, .5);
    glDrawSphere(.1*scale);
  }
  if(orsDrawShapes) switch(s->type){
      case ors::noneST: break;
      case ors::boxST:
        if(orsDrawMeshes && s->mesh.V.N) ors::glDraw(s->mesh);
        else glDrawBox(s->size[0], s->size[1], s->size[2]);
        break;
      case ors::sphereST:
        if(orsDrawMeshes && s->mesh.V.N) ors::glDraw(s->mesh);
        else glDrawSphere(s->size[3]);
        break;
      case ors::cylinderST:
        if(orsDrawMeshes && s->mesh.V.N) ors::glDraw(s->mesh);
        else glDrawCylinder(s->size[3], s->size[2]);
        break;
      case ors::cappedCylinderST:
        if(orsDrawMeshes && s->mesh.V.N) ors::glDraw(s->mesh);
        else glDrawCappedCylinder(s->size[3], s->size[2]);
        break;
      case ors::markerST:
        if(orsDrawMeshes && s->mesh.V.N) ors::glDraw(s->mesh);
        else glDrawAxes(s->size[0]);
        break;
      case ors::meshST:
        CHECK(s->mesh.V.N, "mesh needs to be loaded to draw mesh object");
        ors::glDraw(s->mesh);
        break;
      case ors::pointCloudST:
        CHECK(s->mesh.V.N, "mesh needs to be loaded to draw point cloud object");
        glDrawDots(s->mesh.V);
        break;
      default: HALT("can't draw that geom yet");
    }
  if(!s->contactOrientation.isZero()){
    X.getAffineMatrixGL(GLmatrix);
    glLoadMatrixd(GLmatrix);
    glColor(0, .7, 0);
    glBegin(GL_LINES);
    glVertex3d(0., 0., 0.);
    glVertex3d(.1*s->contactOrientation(0), .1*s->contactOrientation(1), .1*s->contactOrientation(2));
    glEnd();
  }
  glPopName();
}

//! GL routine to draw a ors::Graph
void ors::glDraw(Graph& C){
  ors::Body *n;
  ors::Joint *e;
  ors::Shape *s;
  ors::Proxy *proxy;
  uint i=0, j, k;
  ors::Transformation f;
  double GLmatrix[16];
  
  glPushMatrix();
  
  //bodies
  if(orsDrawBodies) for_list(j, n, C.bodies){
    for_list(k, s, n->shapes) glDrawShape(s, n->X);
    i++;
    if(orsDrawLimit && i>=orsDrawLimit) break;
  }
  
  //joints
  if(orsDrawJoints) for_list(j, e, C.joints){
    //set name (for OpenGL selection)
    glPushName((e->index <<2) | 2);
    
    double s=e->A.pos.length()+e->B.pos.length(); //some scale
    s*=.25;
    
    //from body to joint
    f=e->from->X;
    f.getAffineMatrixGL(GLmatrix);
    glLoadMatrixd(GLmatrix);
    glColor(1, 1, 0);
    //glDrawSphere(.1*s);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(e->A.pos(0), e->A.pos(1), e->A.pos(2));
    glEnd();
    
    //joint frame A
    f.appendTransformation(e->A);
    f.getAffineMatrixGL(GLmatrix);
    glLoadMatrixd(GLmatrix);
    glDrawAxes(s);
    glColor(1, 0, 0);
    glRotatef(90, 0, 1, 0);  glDrawCylinder(.05*s, .3*s);  glRotatef(-90, 0, 1, 0);
    
    //joint frame B
    f.appendTransformation(e->Q);
    f.getAffineMatrixGL(GLmatrix);
    glLoadMatrixd(GLmatrix);
    glDrawAxes(s);
    
    //from joint to body
    glColor(1, 0, 1);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(e->B.pos(0), e->B.pos(1), e->B.pos(2));
    glEnd();
    glTranslatef(e->B.pos(0), e->B.pos(1), e->B.pos(2));
    //glDrawSphere(.1*s);
    
    glPopName();
    i++;
    if(orsDrawLimit && i>=orsDrawLimit) break;
  }
  
  //proxies
  if(orsDrawProxies) for(i=0; i<C.proxies.N; i++) if(!C.proxies(i)->age){
        proxy = C.proxies(i);
        glLoadIdentity();
        glColor(1, 0, 0);
        glBegin(GL_LINES);
        glVertex3dv(proxy->posA.p);
        glVertex3dv(proxy->posB.p);
        glEnd();
        ors::Transformation f;
        f.pos=proxy->posA;
        f.rot.setDiff(ors::Vector(0, 0, 1), proxy->posA-proxy->posB);
        f.getAffineMatrixGL(GLmatrix);
        glLoadMatrixd(GLmatrix);
        glDisable(GL_CULL_FACE);
        glDrawDisk(.02);
        glEnable(GL_CULL_FACE);
        
        f.pos=proxy->posB;
        f.getAffineMatrixGL(GLmatrix);
        glLoadMatrixd(GLmatrix);
        glDrawDisk(.02);
      }
      
  glPopMatrix();
}

/* please don't remove yet: code for displaying edges might be useful...

void glDrawOdeWorld(void *classP){
  _glDrawOdeWorld((dWorldID)classP);
}

void _glDrawOdeWorld(dWorldID world)
{
  glStandardLight();
  glColor(3);
  glDrawFloor(4);
  uint i;
  Color c;
  dVector3 vec, vec2;
  dBodyID b;
  dGeomID g, gg;
  dJointID j;
  dReal a, al, ah, r, len;
  glPushName(0);
  int t;

  //bodies
  for(i=0, b=world->firstbody;b;b=(dxBody*)b->next){
    i++;
    glPushName(i);

    //if(b->userdata){ glDrawBody(b->userdata); }
    c.setIndex(i); glColor(c.r, c.g, c.b);
    glShadeModel(GL_FLAT);

    //bodies
    for(g=b->geom;g;g=dGeomGetBodyNext(g)){
      if(dGeomGetClass(g)==dGeomTransformClass){
  ((dxGeomTransform*)g)->computeFinalTx();
        glTransform(((dxGeomTransform*)g)->final_pos, ((dxGeomTransform*)g)->final_R);
  gg=dGeomTransformGetGeom(g);
      }else{
  glTransform(g->pos, g->R);
  gg=g;
      }
      b = dGeomGetBody(gg);
      // set the color of the body, 4. Mar 06 (hh)
      c.r = ((Body*)b->userdata)->cr;
      c.g = ((Body*)b->userdata)->cg;
      c.b = ((Body*)b->userdata)->cb;
      glColor(c.r, c.g, c.b);

      switch(dGeomGetClass(gg))
  {
  case dSphereClass:
    glDrawSphere(dGeomSphereGetRadius(gg));
    break;
  case dBoxClass:
    dGeomBoxGetLengths(gg, vec);
    glDrawBox(vec[0], vec[1], vec[2]);
    break;
  case dCCylinderClass: // 6. Mar 06 (hh)
    dGeomCCylinderGetParams(gg, &r, &len);
    glDrawCappedCylinder(r, len);
    break;
  default: HALT("can't draw that geom yet");
  }
      glPopMatrix();
    }

    // removed shadows,  4. Mar 06 (hh)

    // joints

      dxJointNode *n;
      for(n=b->firstjoint;n;n=n->next){
      j=n->joint;
      t=dJointGetType(j);
      if(t==dJointTypeHinge){
      dJointGetHingeAnchor(j, vec);
      a=dJointGetHingeAngle(j);
      al=dJointGetHingeParam(j, dParamLoStop);
      ah=dJointGetHingeParam(j, dParamHiStop);
      glPushMatrix();
      glTranslatef(vec[0], vec[1], vec[2]);
      dJointGetHingeAxis(j, vec);
      glBegin(GL_LINES);
      glColor3f(1, 0, 0);
      glVertex3f(0, 0, 0);
      glVertex3f(LEN*vec[0], LEN*vec[1], LEN*vec[2]);
      glEnd();
      //glDrawText(STRING(al <<'<' <<a <<'<' <<ah), LEN*vec[0], LEN*vec[1], LEN*vec[2]);
      glPopMatrix();
      }
      if(t==dJointTypeAMotor){
  glPushMatrix();
  glTranslatef(b->pos[0], b->pos[1], b->pos[2]);
  dJointGetAMotorAxis(j, 0, vec);
  glBegin(GL_LINES);
  glColor3f(1, 1, 0);
  glVertex3f(0, 0, 0);
  glVertex3f(LEN*vec[0], LEN*vec[1], LEN*vec[2]);
  glEnd();
  glPopMatrix();
      }
      if(t==dJointTypeBall){
  dJointGetBallAnchor(j, vec);
  dJointGetBallAnchor2(j, vec2);
  glPushMatrix();
  glTranslatef(vec[0], vec[1], vec[2]);
  glBegin(GL_LINES);
  glColor3f(1, 0, 0);
  glVertex3f(-.05, 0, 0);
  glVertex3f(.05, 0, 0);
  glVertex3f(0, -.05, 0);
  glVertex3f(0, .05, 0);
  glVertex3f(0, 0, -.05);
  glVertex3f(0, 0, .05);
  glEnd();
  glPopMatrix();
  glPushMatrix();
  glTranslatef(vec2[0], vec2[1], vec2[2]);
  glBegin(GL_LINES);
  glColor3f(1, 0, 0);
  glVertex3f(-.05, 0, 0);
  glVertex3f(.05, 0, 0);
  glVertex3f(0, -.05, 0);
  glVertex3f(0, .05, 0);
  glVertex3f(0, 0, -.05);
  glVertex3f(0, 0, .05);
  glEnd();
  glPopMatrix();
      }
    }
      glPopName();
  }
  glPopName();
}
*/

void animateConfiguration(ors::Graph& C, OpenGL& gl){
  arr x, x0, v0;
  uint t, i;
  C.calcBodyFramesFromJoints();
  x0.resize(C.getJointStateDimension());
  v0.resizeAs(x0);
  C.getJointState(x0, v0);
  for(i=x0.N; i--;){
    //for(i=20;i<x0.N;i++){
    x=x0;
    for(t=0; t<20; t++){
      x(i)=x0(i) + .5*sin(MT_2PI*t/20);
      C.setJointState(x, v0);
      C.calcBodyFramesFromJoints();
      MT::wait(0.01);
      if(!gl.update()){ return; }
    }
  }
  C.setJointState(x0, v0);
  C.calcBodyFramesFromJoints();
}

bool infoHoverCall(void *p, OpenGL *gl){
  ors::Graph *C = (ors::Graph*)p;
  ors::Joint *j=NULL;
  ors::Shape *s=NULL;
  OpenGL::GLSelect *top=gl->topSelection;
  if(!top) return false;
  uint i=top->name;
  //cout  <<"HOVER call: id = 0x"  <<std::hex  <<gl->topSelection->name  <<endl;
  if((i&3)==1) s=C->shapes(i>>2);
  if((i&3)==2) j=C->joints(i>>2);
  if(s){
    gl->text.clr()
     <<"shape selection: body="  <<s->body->name  <<" X="  <<s->body->X  <<" ats="  <<endl;
    listWrite(s->ats, gl->text, "\n");
  }
  if(j){
    gl->text.clr()
     <<"edge selection: "  <<j->from->name  <<' '  <<j->to->name
     <<"\nA="  <<j->A  <<"\nQ="  <<j->Q  <<"\nB="  <<j->B  <<endl;
    listWrite(j->ats, gl->text, "\n");
  }
  if(!j && !s) gl->text.clr();
  return true;
}

void editConfiguration(const char* filename, ors::Graph& C, OpenGL& gl){
  gl.exitkeys="1234567890hjklias, ";
  gl.selectOnHover=true;
  gl.addHoverCall(infoHoverCall, &C);
  for(;;){
    cout  <<"reloading `"  <<filename  <<"' ... "  <<std::endl;
    try {
      MT::lineCount=1;
      MT::load(C, filename);
    } catch (const char* msg){
      cout  <<"line "  <<MT::lineCount  <<": "  <<msg  <<" -- please check the file and press ENTER"  <<endl;
      gl.watch();
      continue;
    }
    animateConfiguration(C, gl);
    gl.watch();
    while(MT::contains(gl.exitkeys, gl.pressedkey)){
      switch(gl.pressedkey){
        case '1':  orsDrawBodies^=1;  break;
        case '2':  orsDrawShapes^=1;  break;
        case '3':  orsDrawJoints^=1;  break;
        case '4':  orsDrawProxies^=1;  break;
        case '5':  gl.reportSelects^=1;  break;
        case '6':  gl.reportEvents^=1;  break;
        case '7':  gl.selectOnHover^=1;  break;
        case 'j':  gl.camera.X->pos += gl.camera.X->rot*ors::Vector(0, 0, .1);  break;
        case 'k':  gl.camera.X->pos -= gl.camera.X->rot*ors::Vector(0, 0, .1);  break;
        case 'i':  gl.camera.X->pos += gl.camera.X->rot*ors::Vector(0, .1, 0);  break;
        case ',':  gl.camera.X->pos -= gl.camera.X->rot*ors::Vector(0, .1, 0);  break;
        case 'l':  gl.camera.X->pos += gl.camera.X->rot*ors::Vector(.1, .0, 0);  break;
        case 'h':  gl.camera.X->pos -= gl.camera.X->rot*ors::Vector(.1, 0, 0);  break;
        case 'a':  gl.camera.focus( //TODO
            (gl.camera.X->rot*(*gl.camera.foc - gl.camera.X->pos)
             ^ gl.camera.X->rot*ors::Vector(1, 0, 0)) * .001
            + *gl.camera.foc);
          break;
        case 's':  gl.camera.X->pos += //TODO
            (
              gl.camera.X->rot*(*gl.camera.foc - gl.camera.X->pos)
              ^(gl.camera.X->rot * ors::Vector(1., 0, 0))
            ) * .01;
          break;
      }
      gl.watch();
    }
  }
}

#if 0 //MT_ODE
void testSim(const char* filename, ors::Graph *C, Ode *ode, OpenGL *gl){
  gl->watch();
  uint t, T=200;
  arr x, v;
  createOde(*C, *ode);
  C->getJointState(x, v);
  for(t=0; t<T; t++){
    ode->step();
    
    importStateFromOde(*C, *ode);
    C->setJointState(x, v);
    C->calcBodyFramesFromJoints();
    exportStateToOde(*C, *ode);
    
    gl->text.clr()  <<"time "  <<t;
    gl->timedupdate(10);
  }
}
#endif
#endif

#else //!MT_GL
void ors::glDraw(ors::Mesh& mesh){ NICO; }
void ors::glDrawMesh(void *classP){ NICO; }
#ifndef MT_ORS_ONLY_BASICS
void ors::glDraw(ors::Graph& graph){ NICO; }
void ors::glDrawGraph(void *classP){ NICO; }
void editConfiguration(const char* filename, ors::Graph *C, OpenGL *gl){ NICO; }
void animateConfiguration(ors::Graph *C, OpenGL *gl){}
#endif
#endif
