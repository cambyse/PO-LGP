#ifndef SPLINE_H
#define SPLINE_H

#include <Core/util.h>
#include <Core/array.h>
#include <Ors/ors.h>


struct Spline {
  uint order; // Spline-Ordnung
  arr points; // n Kontrollpunkte
  arr knots;  // (n-order+2) Stuetzstellen
  arr sRef;

  Spline(arr &_knots, arr &_points, uint _order);
  ~Spline();

  arr eval(double _s);
  arr deval(double _s);
  void transform(arr &_dgoal, arr &_dstate, double &_cs);
  void plotSpline();

};

#endif // SPLINE_H
