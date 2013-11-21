#include "dmp.h"

DMP::DMP(arr &y_ref_, uint nBase_, double dt_) {
  y_ref = y_ref_;
  dt = dt_;
  nBase = nBase_;

  T = y_ref.d0*dt;
  tau = 0.5/T;

  dimY = y_ref.d1;

  alphax = 25./3.;
  alphay = 25.;
  betay = alphay/4.;

  X = 1.;
  Xd = 0.;

  Y = y_ref[0];
  Yd = (y_ref[1]-y_ref[0])/dt;
  Ydd = Y*0.;

  y_bk.append(~Y);
  yd_bk.append(~Yd);

  y0 = Y;
  goal = y_ref[y_ref.d0-1];
  amp = goal-y0;

  // init forcing function weights
  C = arr(nBase);
  H = arr(nBase);

  uint i;
  for (i=0;i<nBase;i++) {
    C(i) = exp(-alphax*i*0.5/(nBase-1));
  }

  for (i=0;i<nBase-1;i++) {
    H(i) = 0.5 / (0.65*(C(i+1) - C(i))*(C(i+1) - C(i)));
  }
  H(nBase-1) = H(nBase-2);
}

DMP::~DMP() {

}

void DMP::trainDMP(double T_, const arr &goal_) {

  if (T_>0.) {
    cout << "T changed" << endl;
    T = T_;
    tau = 0.5/T;
  }
  if(&goal_) {
    cout << "goal changed" << endl;
    goal = goal_;
    amp = goal-y0;
  }

  uint i,j;
  arr PHI = zeros(y_ref.d0,nBase);
  double x_=1.;
  for(i = 0; i<y_ref.d0; i++) {

    for(j = 0; j<nBase; j++) {
      PHI(i,j) = exp(-H(j)*(x_-C(j))*(x_-C(j)));
    }

    double cs = sum(PHI.row(i));

    for(j = 0; j<nBase; j++) {
      PHI(i,j) = PHI(i,j)*(x_/cs);
    }

    x_ = x_ - alphax*x_*tau*dt;
  }

  arr FT;
  arr trajd = y_ref*0.;
  arr trajdd = y_ref*0.;

  for (i=0; i<trajdd.d0; i++) {
    for (j=0; j<trajdd.d1; j++) {
      if (i == 0) {
        trajd(i,j) = (y_ref(i+1,j) - y_ref(i,j))/(dt);
      } else if(i == trajdd.d0-1) {
        trajd(i,j) = (y_ref(i,j) - y_ref(i-1,j))/(dt);
      } else {
        trajd(i,j) = (y_ref(i+1,j) - y_ref(i-1,j))/(2*dt);
      }
    }
  }

  for (i=0; i<trajdd.d0; i++) {
    for (j=0; j<trajdd.d1; j++) {
      if (i == 0) {
        trajdd(i,j) = (trajd(i+1,j) - trajd(i,j))/(dt);
      } else if(i == trajdd.d0-1) {
        trajdd(i,j) = (trajd(i,j) - trajd(i-1,j))/(dt);
      } else {
        trajdd(i,j) = (trajd(i+1,j) - trajd(i-1,j))/(2*dt);
      }
    }
  }


  for(i = 0; i<dimY; i++) {
    FT = (trajdd.col(i)/(tau*tau) - alphay*(betay*(goal(i)-y_ref.col(i)) -trajd.col(i)/tau))/amp(i);
    weights.append(~(inverse(~PHI*PHI + eye(PHI.d1)*1e-7)*(~PHI)*FT));
  }
  weights=~weights;

}

void DMP::iterate() {

  uint i;
  arr psi;
  double f;
  for(i =0; i< dimY; i++) {
    psi = exp(-H%(X-C)%(X-C));

    f = sum(~weights.col(i)*(psi)*X)/sum(psi);
    cout << "goal(i)-Y(i)" << goal(i)-Y(i) << endl;
    cout << "Yd(i)/tau" << Yd(i)/tau << endl;
    cout << "(amp(i)*f" << (amp(i)*f) << endl;
    cout << "betay*(goal(i)-Y(i))-(Yd(i)/tau)" << betay*(goal(i)-Y(i))-(Yd(i)/tau) << endl;

    Ydd(i) = (alphay*(betay*(goal(i)-Y(i))-(Yd(i)/tau)) + (amp(i)*f))*tau*tau;
    cout <<"Ydd(i)"<< Ydd(i) << endl;
    Yd(i) = Yd(i) + Ydd(i)*dt;
    Y(i) = Y(i) + Yd(i)*dt;
  }

  y_bk.append(~Y);
  yd_bk.append(~Yd);

  Xd = -alphax*X*tau;
  X = X + Xd*dt;
}

void DMP::reset() {
  X = 1.;
  Xd = 0.;
  Y = y0;
  Yd = Yd*0.;
  Ydd = Ydd*0.;

  yd_bk.clear();
  y_bk.clear();
}

void DMP::plotDMP() {
  write(LIST<arr>(y_ref),"out/y_ref.output");
  write(LIST<arr>(y_bk),"out/y_bk.output");

  gnuplot("set term wxt 1 title 'dimension 1'");
  gnuplot("plot 'out/y_ref.output'using 1  with points pointtype 7 pointsize 1, 'out/y_bk.output'using 1 with points pointtype 7 pointsize 0.5, 'out/y_ref.output'using 2  with points pointtype 7 pointsize 1, 'out/y_bk.output'using 2 with points pointtype 7 pointsize 0.5");

  MT::wait(15);
}

void DMP::printDMP() {
  std::cout <<"tau : " << tau << std::endl;
  std::cout <<"T : " << T << std::endl;
  std::cout <<"dt : " << dt << std::endl;
  std::cout <<"nBase : " << nBase << std::endl;
  std::cout <<"dimY : " << dimY << std::endl;
  std::cout <<"amp : " << amp << std::endl;
  std::cout <<"y0 : " << y0 << std::endl;
  std::cout <<"goal : " << goal << std::endl;
  std::cout <<"alphax : " << alphax << std::endl;
  std::cout <<"alphay : " << alphay << std::endl;
  std::cout <<"betay : " << betay << std::endl;
  std::cout <<"Y : " << Y << std::endl;
//    std::cout <<"C : " << C << std::endl;
//    std::cout <<"H : " << H << std::endl;
//  std::cout <<"weights : " << weights << std::endl;
}
