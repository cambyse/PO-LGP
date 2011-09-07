#include<MT/util.h>
#include<MT/ann.h>

void testANN(){
  uint N=1000,dim=2;

  ANN ann;
  doubleA x(dim),X(N,dim),Y;
  intA idx;
  
  rndUniform(X,0.,1.,false);

  MT::IOraw=true;

  MT::timerStart();
  ann.setX(X);
  rndUniform(x,0.,1.,false); //x=.5;
  ann.getNN(Y,x,10,.0, true);
  std::cout <<"build time (#" <<ann.X.N <<") = " <<MT::timerRead() <<"sec" <<std::endl;
  MT::save(X,"z.data");
  MT::save(x,"z.query");
  MT::save(Y,"z.neighbors");
  gnuplot("set size square; set data style points; plot 'z.data','z.query','z.neighbors'");
  MT::wait();
}

void testANNIncremental(){
  uint N=1000,dim=2;

  ANN ann;
  ann.buffer=20;
  doubleA x(dim),q(dim),Q;
  intA idx;
  
  rndUniform(q,0.,1.,false); //constant query point
  for(uint i=0;i<N;i++){
    rndUniform(x,0.,1.,false);
    ann.append(x);
    if(i>10){
      ann.getNN(Q,q,10,.0, true);
      MT::wait();
    }
  }
}

/*void testANNregression(){
  doubleA X,Y,Z;
  uint i,j;
  X.setGrid(1,-3.,3.,100);
  Y.resize(X.d0,1);
  for(i=0;i<X.N;i++) Y(i,0)=::sin(X(i,0));
  write(X,Y,"z.Y");
  gnuplot("plot 'z.Y'");

  ANN ann;
  doubleA x,y;

  for(i=0;i<500;i++){
    j=rnd(X.N);
    x=X[j]; rndGauss(x,.01,true);
    y=Y[j]; rndGauss(y,.1,true);
    ann.learn(x,y);
    //ann.map(X,Z);
    if(!(i%20)){
      ann.map(X,Z);
      write(X,Z,"z.Z");
      MT::save(ann.X,"z.X");
      gnuplot("plot 'z.X' with points,'z.Y','z.Z'");
    }
  }

}*/

int main(int argn,char** argv){

  //testANN();
  testANNIncremental();
  //testANNregression();

  return 0;
}