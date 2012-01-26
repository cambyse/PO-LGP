#define MT_IMPLEMENT_TEMPLATES

#include <MT/util.h>
#include <MT/gauss.h>
#include <MT/plot.h>

static double TAU=.1;

void fun(doubleA& x){
  x(0)+=TAU*x(1);
  //x(1)-=TAU*x(0);
}

uint N=2;
void testUT(){
  Gaussian a;
  a.c.setUni(0.,N);  //1./::sqrt(N);
  a.C.setDiag(1.,N); //A(1,0)=.5;
  //rndUni(A,1,2.); makeSymmetric(A);
  std::cout <<a;

  for(uint i=0;i<100;i++){
    if(!(i%10)) plotCovariance(a.c,a.C);
    unscentedTransform(a,a,fun);
    MT::IOraw=false;
    std::cout <<a;
  }
  plot();
}


void testMult(){
  Gaussian d,e,c,b;
  c.c <<"[0 0]"; c.C <<"[1. .9; .9 1.]";
  b.c <<"[.5]";  b.C.setDiag(.1,1);
  d=c;
  multiplyToJoint(d,b);

  arr X,y(1);
  double p;
  sample(X,10000,c);
  //plotPoints(X);
  for(uint i=0;i<X.d0;i++){
    y = X(i,1);
    p = NNNN(y,b.c,b.C);
    //cout <<p <<' ' <<y(0) <<endl;
    if(rnd.uni()>p){ X.delRows(i); i--; }
  }
  estimate(e,X);
  plotClear();
  //plotPoints(X);
  plotCovariance(c.c,c.C);
  plotCovariance(b.c,b.C);
  plotCovariance(d.c,d.C);
  plotCovariance(e.c,e.C);
  plot();

}

void testGradient() {
  arr cov = eye(2);
  arr mu = ARR(0., 0.);

  arr grad;

  Gaussian g;
  g.setC(mu, cov);

  g.gradient(grad, ARR(1.,1.));

  cout << "Gradient: " << grad << endl;
}

void testSdv(){
  arr X;
  Gaussian a;
  a.c <<"[0 0]"; a.C <<"[1. 0; 0. 1.]";
  sample(X,1000,a);
  double d=0.;
  for(uint i=0;i<X.d0;i++){
    d+=sumOfSqr(X[i]);
  }
  d/=X.d0;
  cout <<d <<' ' <<sqrt(d) <<endl;
}

void testPullback(){
  Gaussian a,b;
  a <<"[0] [1.;]";
  cout <<"Gaussian a = " <<a;
  plotClear();
  plotCovariance(a.c,a.C);

  //a redundant linear function
  arr f,F,Q;
  f <<"[0.]";
  F <<"[1 .5;]";
  Q <<"[0.]";

  //bwd
  b.c <<"[0. 0.]";
  backward(b,a,f,F,Q);
  cout <<"Gaussian b = " <<b;
  plotCovariance(b.c,b.C);

  //fwd again
  forward(a,b,f,F,Q);
  a.c +=.1;
  cout <<"Gaussian a = " <<a;
  plotCovariance(a.c,a.C);
  plot();


}

void testConditional(){
  Gaussian g1,g2,g1m,g2m,y;
  uint t,k=2,l=1,T=10;
  MT_DEBUG(double eps=1e-3;)

  for(t=0;t<100;t++){
    useC=false;
    g1.setRandom(7);
    g2=g1;
    getMarginal(g1m,g1,3);
    blowupMarginal(y,g1m,4);
    g1m=y;
    makeConditional(g2,3);
    product(y,g2,g1m);
    CHECK(sameGaussian(g1,y,eps),":-(");
  }

  arr f(l),F(l,k),Q(l,l),ff,FF,QQ;
  double v1,v2;
  for(t=0;t<T;t++){
    rndUniform(f,-1.,1.,false);
    rndUniform(F,-1.,1.,false);
    rndUniform(Q, .1,1.,false); Q = ~Q*Q;
    cout <<"f F Q = " <<f <<F <<Q;
    
    useC=true;
    g1.setConditional(f,F,Q);
    cout <<"joint made with C:" <<g1;
    useC=false;
    g2.setConditional(f,F,Q);
    cout <<"joint made with U:" <<g2;
    CHECK(sameGaussian(g1,g2,1e-3),":-(");
    
    useC=true;
    getConditional(g1,2,ff,FF,QQ);
    cout <<"cond made with C:" <<ff <<FF <<QQ;
    CHECK(maxDiff(f,ff,0)<eps,"wrong f!=ff " <<f <<ff);
    CHECK(maxDiff(F,FF,0)<eps,"wrong F!=FF " <<F <<FF);
    CHECK(maxDiff(Q,QQ,0)<eps,"wrong Q!=QQ " <<Q <<QQ);

    useC=false;
    getConditional(g1,2,ff,FF,QQ);
    cout <<"cond made with U:" <<ff <<FF <<QQ;
    CHECK(maxDiff(f,ff,0)<eps,"wrong f!=ff " <<f <<ff);
    CHECK(maxDiff(F,FF,0)<eps,"wrong F!=FF " <<F <<FF);
    CHECK(maxDiff(Q,QQ,0)<eps,"wrong Q!=QQ " <<Q <<QQ);

    v1=rnd.uni(1.,10.);
    v2=rnd.uni(1.,10.);//e-3,1e-2);
    useC=true;
    g1.setConditional(2,v1,v2);
    cout <<"joint made with C:" <<g1;
    useC=false;
    g2.setConditional(2,v1,v2);
    cout <<"joint made with U:" <<g2;
    CHECK(sameGaussian(g1,g2,1e-3),":-(");
  }

  useC=true;
  getMarginal(g1m,g1,k);
  cout <<"marginal made with C:" <<g1m;
  useC=false;
  getMarginal(g2m,g2,k);
  cout <<"marginal made with U:" <<g2m;
}

void testEvaluate() {
	Gaussian g;
  g.setDiagonal(1, 1);
	double p = 0;

	p = g.evaluate(ARR(0.));
	std::cout << "P(0) = " << p << std::endl;

	p = g.evaluate(ARR(10.));
	std::cout << "P(10) = " << p << std::endl;
	
	p = g.evaluate(ARR(1.));
	std::cout << "P(1) = " << p << std::endl;

}

double px(const arr& x){
  if(x(1)>.2) return 1.;
  return 0.;
}

void testWeightedSampling(){
  Gaussian g;
  g.setDiagonal(3,1.);
  arr M(g.c.N,g.c.N); rndUniform(M,0.,1.,false); g.C=M*g.C*~M;
  plotCovariance(g.c,g.C);
  cout <<g;

  arr X,W;
  systematicWeightedSamples(X,W,g);
  plotPoints(X);

  estimateWeighted(g,X,W);
  plotCovariance(g.c,g.C);
  cout <<g;
  plot();

  g.setDiagonal(g.c.N,1.);
  rndUniform(M,0.,1.,false); g.C=M*g.C*~M;
  plotClear();
  plotCovariance(g.c,g.C);

  resampleAndEstimate(g,px,100);
  plotCovariance(g.c,g.C);
  plot();
}

void testMarginal(){
  Gaussian g,m1,m2;
  g.setRandom(10);

  uintA L; L <<"[1 8 3 2]";

  getMarginal(m1,g,L);
  g.makeU(); g.okC=false;
  getMarginal(m2,g,L);

  cout
    <<"g=" <<g
    <<" m1=" <<m1
    <<" m2=" <<m2
    <<endl;
}

void testKLD(){
  Gaussian A,B;
  A.setRandom(3);
  B.setRandom(3);
  //B=A;

  cout <<A <<B <<endl;
  double d=KLDsym(A,B);
  cout <<d <<endl;
}

void testProduct(){
  uint k;
  double l1,l2;
  Gaussian a,b,c1,c2,b1,b2;
  for(uint t=0;t<100;t++){
    k=rnd.num(3,7);
    a.setRandom(k);
    b.setRandom(k);
    useC=true;   product(c1,a,b,&l1);
    useC=false;  product(c2,a,b,&l2);

    cout <<"product with C: likelihood=" <<l1 <<endl <<c1;
    cout <<"product with U: likelihood=" <<l2 <<endl <<c2;
    CHECK(fabs(l1-l2)<1e-5,":-(");
    CHECK(sameGaussian(c1,c2,1e-5),":-(");

    useC=true;   division(b1,c1,a,&l1);
    useC=false;  division(b2,c2,a,&l2);

    cout <<"division with C: likelihood=" <<l1 <<endl <<b1;
    cout <<"division with U: likelihood=" <<l2 <<endl <<b2;
    CHECK(fabs(l1-l2)<1e-5,":-(");
    CHECK(sameGaussian(b1,b2,1e-5),":-(");
    CHECK(sameGaussian(b1,b,1e-5),":-(");
    CHECK(sameGaussian(b2,b,1e-5),":-(");
  }
}

void testReduction(){
  uint n=100,m=10;
  GaussianA f(n),g;
  for(uint i=0;i<n;i++) f(i).setRandom(5);
  arr P(n);
  rndUniform(P,.9,1.);

  reduceIterated(g,m,f,P,50);
}

int main(int argn,char** argv){

  //testProduct();
  //testUT();
  //testMult();
  //testSdv();
  //testPullback();
  //testConditional();
  //testWeightedSampling();
  //testMarginal();
  //testKLD();

  testReduction();
  //testReduction();
	testEvaluate();
  testGradient();

  return 0;
}
