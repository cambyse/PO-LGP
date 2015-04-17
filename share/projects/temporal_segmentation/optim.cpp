#include <Core/array.h>
#include "optim.h"

double LineSearch::search(const arr &x, const arr &d) {
  double alpha = 10;

  arr J = objf.jacobian(x);
  double J_T_d = scalarProduct(J, d);
  for(;;) {
    if(objf.f(x + alpha * d) <= objf.f(x) + rho_ls * alpha * J_T_d)
      break;
    alpha *= rho_alpha_minus;
  }
  return alpha;
}

void BFGS::loopUntil(arr &x, double end_condition) {
  arr I, newJ, d, delta, newx, y, tmpH, tmpHT;
  double step, delta_T_y;

  uint n = x.N;

  I = eye(n);
  invH = I;

  LineSearch ls;
  ls.objf = objf;

  arr J = objf.jacobian(x);
  for(uint t = 0;; t++) {
    // cout << "LOOP " << t << endl;
    // cout << "x: " << x << endl;
    // cout << "f(x): " << objf.f(x) << endl;
    // cout << "J: " << J << endl;
    d = - invH * J;
    // d /= length(d);

    // cout << "invH: " << invH << endl;
    // cout << "d: " << d << endl;

    step = ls.search(x, d);
    delta = step * d;
    // cout << "step: " << step << endl;
    // cout << "delta: " << delta << endl;
    newx = x + delta;
    newJ = objf.jacobian(newx);

    y = newJ - J;

    x = newx;
    J = newJ;

    delta_T_y = scalarProduct(delta, y);
    tmpH = (I - (y ^ delta)) / delta_T_y;
    transpose(tmpHT, tmpH);
    invH = tmpHT * invH * tmpH + (delta ^ delta) / delta_T_y;

    if(absMax(delta) < end_condition)
      break;
  }
}

void Newton::loopUntil(arr &x, double end_condition) {
  arr J, H, E;
  arr dx;

  E = diag(1e-9, x.N);
  for(;;) {
    J = objf.jacobian(x);
    H = objf.hessian(x);
    dx = inverse(H + E) * J;
    x -= dx;
    if(length(dx) < end_condition)
      break;
  }
}

