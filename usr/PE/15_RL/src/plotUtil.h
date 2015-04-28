#include <GL/glu.h>
#include <Gui/opengl.h>

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

