#include <Core/array.h>

double sf_ucb1(const arr& y){
  uint n=y.N;
//  if(n==1) return y(0);
  double c = (y.max() - y.min());
  return sum(y)/n + c*sqrt(2.*::log(2.*n)/n); //assuming 1 other child!
}

double sf_meanPlusSdv(const arr& y, double b=1.5){
  if(y.N==1) return y(0);
  uint n=y.N;
  double mean = sum(y)/n;
  double var = 1./(n-1)*sumOfSqr(y-mean);
  return mean + b * sqrt(var/n);
}

double sf_maxBootstrapMean(const arr& y){
  double ma = y.max();
  double maxMean = y.min();
  for(uint k=0;k<20;k++){
    arr z = bootstrap(y);
//    z.append(2.*ma);
    double m = sum(z)/z.N;
    if(m>maxMean) maxMean=m;
  }
  return maxMean;
}

void test(){
  ofstream fil("z");
  fil <<"n ucb1 meanPlusSdv maxBootstrapMean" <<endl;
  uint K=100;
  for(uint n=1;n<100;n++){
    arr z = zeros(3);
    for(uint k=0;k<K;k++){
      arr y(n);
      rndGauss(y,1.,false);
      y += 100.;
      z(0) += sf_ucb1(y);
      z(1) += sf_meanPlusSdv(y);
      z(2) += sf_maxBootstrapMean(y);
    }
    z /= (double)K;
    fil <<n <<' ';
    z.write(fil, " ", " ", "  ");
    fil <<endl;
  }
}

int main(int argc,char **argv){
  test();
}
