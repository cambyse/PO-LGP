#include <Core/util.h>
#include <Motion/motion.h>
#include <Gui/opengl.h>
#include "spline.h"
#include <vector>
#include <GL/glu.h>
#include <stdlib.h>

const double PI = 3.1415926535897932384626;

int main(int argc,char **argv){
  arr l = linspace(0.,PI*.5,60);

  arr points = sin(~l);
  points.append(-cos(~l));
  points = ~points;

  uint order = 2;
  arr knots = linspace(0.,1.,points.d0-order+1);
  cout << "#knots: " << knots.d0 << endl;
  cout << "#points: " << points.d0 << endl;
  Spline s = Spline(knots,points,order);
  s.plotSpline();

  arr dstate = ARRAY(0.0,0.0);
  arr dgoal = ARRAY(0.8,0.1);
  double cs = 0.0;
  s.transform(dgoal,dstate,cs);
  os.plotSpline();
  MT::wait();
  return 0;
}
