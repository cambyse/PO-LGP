#include <Core/util.h>
#include <math.h>

int main(int argc,char **argv){
  MT::initCmdLine(argc,argv);

  ofstream fil("z.endeffDist");

  for(uint i=0;i<10000;i++){
    double q1=rnd.uni()*MT_2PI;
    double q2=rnd.uni()*MT_2PI;
    double x=cos(q1) + cos(q2);
    double y=sin(q1) + sin(q2);
    fil <<x <<' ' <<y <<endl;
  }
  
  gnuplot("set size ratio 1; plot 'z.endeffDist' w p ps .5",false, true, "z.endeffDist.pdf");

  return 0;
}


