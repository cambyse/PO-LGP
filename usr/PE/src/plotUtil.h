#ifndef PLOTUTIL_H
#define PLOTUTIL_H

#include <GL/glu.h>
#include <Gui/opengl.h>
#include <Ors/ors.h>

void drawTraj(uint color, arr& p, uint lineStyle);

void drawRedLine(void* classP);
void drawRedPoints(void* classP);
void drawGreenLine(void* classP);
void drawGreenPoints(void* classP);
void drawBlueLine(void* classP);
void drawBluePoints(void* classP);
void drawYellowLine(void* classP);
void drawBlackPoints(void* classP);

void drawLine(ors::KinematicWorld &world, arr &x, uint color);
void drawPoints(ors::KinematicWorld &world, arr &x, uint color);
void drawLine(ors::KinematicWorld &world, arr &q,arr &x, const char *name ,uint color,uint lower,uint upper);
void drawPoints(ors::KinematicWorld &world, arr q,arr &x, const char *name ,uint color);

#endif
