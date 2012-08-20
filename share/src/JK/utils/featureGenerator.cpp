#include "featureGenerator.h"
#include <MT/array.h>

void DistanceFeatureGenerator::makeFeatures(arr& Z, const arr& X){
  Z.append(X.sub(0,-1,4,6) - X.sub(0,-1,0,2));
  Z.reshape(1,3);
}
void TrayFeatureGenerator::makeFeatures(arr& Z, const arr& X){
  Z.append(X.sub(0,-1,4,6) - X.sub(0,-1,0,2));
  Z.append(X.sub(0,-1,3,3));
  Z.reshape(1,4);
}

void CubicFeatureGenerator::makeFeatures(arr&Z, const arr& X) {
  uint n=X.d0, d=X.d1;
  Z.resize(n, 9);// 1 + d + d*(d+1)/2 + d*(d+1)*(d+2)/6);
  uint i, j, k, l, m;
  for(i=0; i<n; i++){
    arr x=X[i];
    arr z=Z[i];
    l=3;
    z(0) = x(0) - x(4);
    z(1) = x(1) - x(5);
    z(2) = x(2) - x(6);


    //z(l++)=1.;
    //for(j=0; j<d; j++) z(l++) = x(j);
    for(j=0; j<3; j++) for(k=0; k<=j; k++) z(l++) = (x(j)-x(j+4))*(x(k)-x(k+4));
    //for(j=0; j<d; j++) for(k=0; k<=j; k++) for(m=0; m<=k; m++) z(l++) = x(j)*x(k)*x(m);
  }
}
