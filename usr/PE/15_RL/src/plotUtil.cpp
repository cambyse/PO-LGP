#include "plotUtil.h"

void drawTraj(uint color, arr& p, uint lineStyle) {
  glColor(color);
  glPointSize(6.0f);
  glLineWidth(2);
  if (lineStyle == 1) {
    glBegin(GL_POINTS);
  } else {
    glBegin(GL_LINES);
  }
  glVertex3f(p(0,0),p(0,1),p(0,2));
  uint i;
  for (i = 1; i<p.d0-1; i++) {
    glVertex3f(p(i,0),p(i,1),p(i,2));
    glVertex3f(p(i,0),p(i,1),p(i,2));
  }
  glVertex3f(p(i,0),p(i,1),p(i,2));
  glEnd();
  glLineWidth(1);
}

void drawRedLine(void* classP){  arr *p = (arr*)classP;  drawTraj(2,*p,2); }
void drawRedPoints(void* classP){  arr *p = (arr*)classP;  drawTraj(2,*p,1); }
void drawGreenLine(void* classP){  arr *p = (arr*)classP;  drawTraj(5,*p,2); }
void drawGreenPoints(void* classP){  arr *p = (arr*)classP;  drawTraj(5,*p,1); }
void drawBlueLine(void* classP){  arr *p = (arr*)classP;  drawTraj(0,*p,2); }
void drawBluePoints(void* classP){  arr *p = (arr*)classP;  drawTraj(0,*p,1); }
void drawYellowLine(void* classP){  arr *p = (arr*)classP;  drawTraj(1,*p,2); }
void drawBlackPoints(void* classP){  arr *p = (arr*)classP;  drawTraj(1,*p,1); }

void drawLine(ors::KinematicWorld &world, arr &x, uint color) {
  CHECK(x.d1 == 3,"x.d1 != 3");
  switch (color){
    case 0:
      world.gl().add(drawRedLine,&(x));
      break;
    case 1:
      world.gl().add(drawBlueLine,&(x));
      break;
    case 2:
      world.gl().add(drawGreenLine,&(x));
      break;
  }
}

void drawPoints(ors::KinematicWorld &world, arr &x, uint color) {
  CHECK(x.d1 == 3,"x.d1 != 3");
  switch (color){
    case 0:
      world.gl().add(drawRedPoints,&(x));
      break;
    case 1:
      world.gl().add(drawBluePoints,&(x));
      break;
    case 2:
      world.gl().add(drawGreenPoints,&(x));
      break;
  }
}

void drawLine(ors::KinematicWorld &world, arr &q,arr &x, const char *name ,uint color,uint lower=0,uint upper=0) {
  x.clear();
  if (upper==0) upper = q.d0;
  for (uint i=lower;i<upper;i++) {
    world.setJointState(q[i]);
    x.append(~ARRAY(world.getShapeByName(name)->X.pos) );
  }
  drawLine(world,x,color);
}

void drawPoints(ors::KinematicWorld &world, arr q,arr &x, const char *name ,uint color) {
  x.clear();
  for (uint i=0;i<q.d0;i++) {
    world.setJointState(q[i]);
    x.append(~ARRAY(world.getShapeByName(name)->X.pos) );
  }
  drawPoints(world,x,color);
}
