#include<MT/array.h>
#include<MT/util.h>
#include<MT/plot.h>

using namespace std;

#include <MT/MLcourse.h>
#include <MT/ann.h>
#include <MT/util.h>
#include <MT/plot.h>


void generateSpiralData(){
  uint n=MT::getParameter<uint>("n",100);
  double Phi=MT::getParameter<double>("Phi",1.);
  arr X,y;
  X.resize(2*n,2);
  y.resize(2*n);
  for(uint i=0;i<n;i++){
    double phi= Phi*MT_2PI*i/n;
    double r = 1.+1.*i/n;
    X[0+i] = ARR(r*cos(phi),r*sin(phi));       y(0+i)=0;
    X[n+i] = ARR(-r*cos(phi),-r*sin(phi));     y(n+i)=1;
  }
  rndGauss(X,MT::getParameter<double>("sigma",.004),true);
  write(LIST(X,y),"spiral.dat");
  gnuplot("plot [-2:2][-2:2] 'spiral.dat' us 1:2:3 with points pt 2 lc variable",NULL,true);
  //MT::wait();
}

void laplacianEmbedding(arr& Z,const arr& W){
  uint n=W.d0;
  arr degree = sum(W,1);
  arr L = diag(degree) - W;

  //SVD
  arr U,d,V,D(n,n);
  svd(U,d,V,L);
  D.setDiag(d);
  
  cout <<"Eigenvalues = " <<d <<endl;
  cout <<"3 smallest Eigenvectors =\n " <<~V.sub(0,-1,-3,-1) <<endl;

  //encoding
  Z = V.sub(0,-1,-3,-2); //pick the 3rd and 2nd last columns
}

void kernelPCAEmbedding(arr& Z,const arr& W){
  uint n=W.d0;
  arr K=W;

  //centering
  arr Ones(n,n),Id(n,n);
  Id.setDiag(1.);
  Ones = 1./(n*n);
  K = (Id-Ones)*K*(Id-Ones);

  //SVD
  arr U,d,V,D(n,n);
  svd(U,d,V,K);
  D.setDiag(d);
  
  
  cout <<"Eigenvalues = " <<d <<endl;
  cout <<"2 largest Eigenvectors =\n " <<~V.sub(0,-1,0,1) <<endl;

  /*for(uint i=0;i<n;i++){
    arr v = V.sub(0,-1,i,i);
    v.reshape(v.N);
    cout <<d(i)*v <<endl <<L*v <<endl;
  }*/
  
  //encoding
  V = V.sub(0,-1,0,1); //pick only first 2 columns
  Z.resize(n,2);
  arr kappa;
  for(uint i=0;i<n;i++){
    kappa = K[i];
    Z[i]() = ~V*kappa;
  }
  Z=V;
}

void testEmbedding(){
  //load data
  ifstream fil("spiral.txt");
  //ifstream fil("spiral_debug.txt");
  arr X,y;
  X.read(fil);
  y=(~X)[2];
  X.delColumns(2);
  uint n=X.d0;

  //generate kNN graph
  uint k=MT::getParameter<uint>("k",7);
  double c=MT::getParameter<double>("c",.05);
  arr W(n,n);
  W = 0.;
  ANN ann;
  ann.setX(X);
  intA idx;
  for(uint i=0;i<n;i++){
    ann.getNN(idx, X[i], k+1);
    for(uint j=0;j<idx.N;j++){
      uint jj=idx(j);
      double d = sumOfSqr(X[i]-X[jj]);
      //W(i,jj)=W(jj,i)=1.;
      W(i,jj)=W(jj,i)=exp(-d/c);
    }
  }
  
  arr Z;
  laplacianEmbedding(Z,W);
  //kernelPCAEmbedding(Z,W);

  write(LIST(Z,y),"embedded.dat"); //(D*U).sub(0,-1,0,1),
  gnuplot("plot 'embedded.dat' us 1:2:3 with points pt 2 lc variable",NULL,true);
}


int main(int argc, char *argv[]){
  MT::initCmdLine(argc,argv);

  //generateSpiralData();
  testEmbedding();
  
  return 0;
}

