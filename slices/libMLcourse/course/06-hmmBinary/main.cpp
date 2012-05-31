#include <MT/array.h>
#include <MT/util.h>


bool plot=false;

//===========================================================================
//
// Baum Welch methods
//

void computeEvidences(arr& rho, const arr& y, const arr& mu, double sigma){
  //-- evidences from observations
  uint T=y.d0,K=mu.N,t,k;
  rho.resize(T,K);
  for(t=0;t<T;t++){
    for(k=0;k<K;k++) rho(t,k)=::exp(-.5 * MT::sqr(y(t)-mu(k)) / (sigma*sigma) );
  }
  
  if(plot){
    MT::save(rho,"z.rho");
    gnuplot("plot 'z.y' w p title 'data', 'z.rho' us 0:2 w l title 'evidences'","z.pdf",true);
    MT::wait();
  }
}

void Estep(arr& a, arr& b, const arr& P0, const arr& P, const arr& rho){
  uint T=rho.d0,K=P.d0,t;
  a.resize(T,K);
  b.resize(T,K);
  a[0]()=P0;   //initialization of alpha
  b[T-1]()=1.; //initialization of beta
  //--- fwd and bwd iterations:
  for(t=1;t<T;t++){
    a[t]() =  P * (rho[t-1]%a[t-1]); // %=element-wise multiplication, *=inner product
    normalizeDist(a[t]()); //for numerical stability
  }
  for(t=T-1;t--;){
    b[t]() = ~P * (rho[t+1]%b[t+1]);
    normalizeDist(b[t]());
  }

  if(plot){
    MT::save(a,"z.alpha");
    MT::save(b,"z.beta");
    gnuplot("plot 'z.y' w p title 'data', 'z.alpha' us 0:2 w l title 'filtering', 'z.beta' us 0:2 w l title 'betas', 'z.x' w l title 'true state' lw 1","z.pdf",true);
    MT::wait();
  }
}

void EstepQ(arr& q, arr& q_pair, const arr& P0, const arr& P, const arr& rho){
  uint T=rho.d0,K=P.d0,t,k,j;
  arr a,b;
  Estep(a,b,P0,P,rho);
  q.resize(T,K);
  q_pair.resize(T-1,K,K);
  for(t=0;t<T;t++)  q[t]() = a[t]() % rho[t]() % b[t](); // %=element-wise multiplication
  for(t=0;t<T-1;t++){
    for(k=0;k<K;k++) for(j=0;j<K;j++){
      q_pair(t,k,j) = a(t,k)*rho(t,k)*P(j,k)*rho(t+1,j)*b(t+1,j);
    }
  }

  for(t=0;t<T;t++)   normalizeDist(q[t]());
  for(t=0;t<T-1;t++) normalizeDist(q_pair[t]());

  if(plot){
    MT::save(q,"z.q");
    gnuplot("plot 'z.y' w p title 'data', 'z.q' us 0:2 w l title 'posterior q' lw 4, 'z.x' w l title 'true state' lw 1","z.pdf",true);
    MT::wait();
  }
}

void Mstep(arr& P0, arr& P, arr& mu, const arr& q, const arr& q_pair, const arr& y){
  uint T=y.d0,K=P.d0,t,k,j;
  P0 = q[0];

  P.setZero();
  arr p(K); p.setZero();
  for(t=0;t<T-1;t++){ p += q[t];  P += q_pair[t]; }
  for(k=0;k<K;k++) for(j=0;j<K;j++)  P(j,k) = P(j,k) / p(k);

  arr w(K,T);
  arr qsum = sum(q,0);
  for(t=0;t<T;t++) for(k=0;k<K;k++)  w(k,t) = q(t,k)/qsum(k);

  mu = w*y;
}

//===========================================================================
//
// generate data
//

void generateData(arr& y, uintA& x,  uint T, const arr& P0, const arr& P, const arr& mu, double sigma){
  uint K=P0.N,t;
  rnd.clockSeed();
  arr p(K);
  x.resize(T);
  y.resize(T);
  p=P0;
  for(t=0;t<T;t++){
    x(t) = SUS(p); //state
    y(t) = mu(x(t)) + sigma*rnd.gauss(); //SUS(B[x(t)]); //observation
    p = P[x(t)];
  }
  x.reshape(T,1);  MT::save(x,"z.x");  x.reshape(T);
  y.reshape(T,1);  MT::save(y,"z.y");  y.reshape(T);
  if(plot){
    gnuplot("plot 'z.y' w p title 'data'","z.pdf",true);
    MT::wait();
    gnuplot("plot 'z.y' w p title 'data', 'z.x' w l title 'true state' lw 4","z.pdf",true);
    MT::wait();
  }
}


//===========================================================================
//
// main
//

int main(int argc,char** argv){
  MT::initCmdLine(argc,argv);
  MT::arrayBrackets="  ";

  //-- build transition and observation matrices
  uint T=1000,K=2;
  double eps=MT::getParameter<double>("eps",.01);
  double sigma=MT::getParameter<double>("sigma",1.);
  arr P0(K),P(K,K),mu(K);
  P0=1./K;
  P=eps;  for(uint i=0;i<K;i++) P(i,i) = 1.-(K-1)*eps;
  for(uint k=0;k<K;k++) mu(k) = (double)k;
  
  uintA x;
  arr y,rho,a,b,q,q_pair;

  generateData(y,x, T, P0,P,mu,sigma);
  
#if 1 //reinit parameters to test learning
  P = ARR(.9,.1,.1,.9); P.reshape(2,2);
  mu = ARR(.4,.6);
#endif

  for(uint iter=0;iter<10;iter++){
    cout <<"EM iteration " <<iter
      <<"\n  P0=" <<P0
      <<"\n  P=" <<P
      <<"\n  mu=" <<mu
      <<endl;
    computeEvidences(rho, y,mu,sigma);
    EstepQ(q, q_pair, P0,P,rho);
    Mstep(P0,P,mu, q, q_pair, y);
  }
  
  MT::save(q,"z.q");
  gnuplot("plot 'z.y' w p title 'data', 'z.q' us 0:2 w l title 'posterior q' lw 4, 'z.x' w l title 'true state' lw 1","z.pdf",true);
}
