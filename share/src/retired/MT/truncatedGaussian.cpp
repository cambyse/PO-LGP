/*  ---------------------------------------------------------------------
    Copyright 2012 Marc Toussaint
    email: mtoussai@cs.tu-berlin.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */



#include <Core/array.h>
#include <Core/util.h>

/** @brief input z: determines the heavyside function [[x>z]], output: mean
   and variances of the remaining probability mass when everything
   left of z is cut off */
void TruncatedStandardGaussian(double& mean, double& var, double z){
  double norm = ::sqrt(MT_PI/2.) * (1.-::erf(z/::sqrt(2.)));
  //cout <<"truncating with z=" <<z <<" (norm=" <<norm <<")" <<endl;
  if(norm<1e-2) cout <<"likelihood of that truncation is very low:" <<norm <<endl;
  mean = ::exp(-z*z/2.)/norm;
  var  = 1. + z*mean - mean*mean;
}

void TruncateGaussian(arr& a, arr& A, const arr& c, double d){

  //cout <<"A=" <<A <<" a=" <<a <<" c=" <<c <<" d=" <<d <<endl;
  
  //-- linear transform to make (a, A) to a standard Gaussian
  arr M;
  lapack_cholesky(M, A);
  //cout <<"M=" <<M <<endl <<~M*M <<endl <<"A=" <<A <<endl;
  
  //-- transform and rescale the constraint
  double z=scalarProduct(c, a)-d;
  //cout <<"a=" <<a <<"\nc*a=" <<scalarProduct(c, a) <<"\nd=" <<d <<endl;
  arr v;
  v=M*c;
  double norm_v=length(v);
  CHECK(norm_v>1e-10, "no gradient for Gaussian trunctation!");
  z/=norm_v;
  v/=norm_v;
  //cout <<"c=" <<c <<" v=" <<v <<endl;
  
  //-- build rotation matrix for constraint to be along the x-axis
  uint n=a.N;
  arr e_1(n);
  e_1.setZero(); e_1(0)=1.;
  arr R;
  rotationFromAtoB(R, e_1, v);
  //cout <<"R=" <<R <<~R <<inverse(R) <<v <<endl <<R*e_1 <<endl;
  
  //-- get mean and variance along x-axis
  double mean, var;
  TruncatedStandardGaussian(mean, var, -z);
  arr B(n, n), b(n);
  b.setZero();  b(0)=mean;
  B.setId(n);   B(0, 0)=var;
  
  //-- transform back
  A = ~M*R*B*~R*M;
  a = ~M*R*b + a;
  
  checkNan(a);
  checkNan(A);
}

void TruncateGaussianBySampling(arr& a, arr& A, const arr& c, double d, uint N, arr *data){
  uint i, j, n=a.N;
  //-- generate samples from the Gaussian
  arr M, x(n), X;
  lapack_cholesky(M, A);
  for(i=0; i<N; i++){
    for(j=0; j<n; j++) x(j)=rnd.gauss();
    x = ~M*x;
    x += a;
    if(scalarProduct(c, x)-d>=0.) X.append(x);
  }
  if(!X.N){
    cout <<"TruncateGaussianBySampling: no samples survived!" <<endl;
    if(data)(*data).clear();
    return;
  }
  X.reshape(X.N/n, n);
  gaussFromData(a, A, X);
  if(data) *data = X;
}
