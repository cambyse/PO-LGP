#include "spline.h"

Spline::Spline(arr &_knots, arr &_points, uint _order) {
  knots = _knots;
  points = _points;
  order = _order;

  sRef = _knots;
  knots = cat(repmat(ARRAY(knots(0,0)),order-1,1),knots,repmat(ARRAY(knots.last()),order-1,1));
}

Spline::~Spline() {

}

bool leq(const double& a,const double& b){ return a<=b; }

arr Spline::eval(double _s) {
  arr val;
  uint idx = knots.rankInSorted(_s+1e-9,leq);

  if (idx > points.d0) {
    return points[points.d0-1];
  }

  for (uint u=0; u<points.d1; u++) {
    arr X = zeros(order,order);
    X[0] = points.sub(idx-order,idx-1,u,u);
    X = ~X;
    for(uint i = 2; i<=order; i++) {
      for(uint j = i; j<=order; j++) {
        double num = _s - knots(idx-order+j-1,0);
        double weight;
        if (num == 0) {
          weight = 0.;
        } else {
          weight = num/(knots(idx+j-i,0)-knots(idx-order+j-1,0));
        }
        X(j-1,i-1) = (1-weight)*X(j-2,i-2) + weight*X(j-1,i-2);
      }
    }
    val.append(X(order-1,order-1));
  }
  return val;
}

arr Spline::deval(double _s) {
  // compute derivative with finite differences
  double ds = sRef(1,0)-sRef(0,0);
  arr val;

  if ((_s+ds) > sRef.last()) {
    val = (eval(_s)-eval(_s-ds))/ds;
  } else if ((_s-ds)< sRef(0,0)) {
    val = (eval(_s+ds)-eval(_s))/ds;
  } else {
    val = (eval(_s+ds)-eval(_s-ds))/(2*ds);
  }

  return val;
}


void Spline::transform(arr &_dgoal, arr &_dstate, double &_cs){
  int idx = sRef.rankInSorted(_cs,leq)-1;
  idx = max(ARRAY(double(idx),0.));

  for (uint i = idx; i<points.d0; i++) {
    points[i] = points[i] + _dstate;
  }

  for (uint i = idx; i<points.d0; i++) {
    points[i] = points[i] + (_dgoal-_dstate)*( (sRef(i,0)-1)/(1-_cs) +1);
  }
}


void Spline::plotSpline() {
  arr interp;
  double f = knots(0,0);
  while (f <= knots.last() ) {
    interp.append(~eval(f));
     f += 1./30.;//0.0001;
  }

  write(LIST<arr>(points),"out/points.output");
  write(LIST<arr>(interp),"out/interp.output");

  gnuplot("set term wxt 1 title 'dimension 1'");
  gnuplot("plot 'out/points.output'using 1:2  with points pointtype 7 pointsize 1, 'out/interp.output'using 1:2 with points pointtype 7 pointsize 0.5");


// MT::wait(1);


}
