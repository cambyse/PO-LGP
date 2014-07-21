#include <Gui/mesh.h>
#include <Gui/opengl.h>
#include <Algo/algos.h>
#include <Gui/plot.h>
#include <qhull/qhull.h>
#include <Optim/optimization.h>

//===========================================================================
//
// 1. test: convex hull distance and its gradient
//

namespace Qtest{
  arr origin;
  double f(arr *grad,const arr& X,void*){
    double d=distanceToConvexHull(X,origin,0,0,true);
    if(grad) distanceToConvexHullGradient(*grad,X,origin,true);
    return d;
  }
}

void TEST(ConvexHull) {
  uint N=20,D=2;
  arr X(N,D);
  rndUniform(X,-1.,1.,false);
  cout <<"X=" <<X <<endl;

  arr origin(D),p;
  origin.setZero();
  uintA V;
  double d;

  QHULL_DEBUG_LEVEL=2;

  for(uint i=0;i<100;i++){
    d = distanceToConvexHull(X,origin,&p,&V,true);
    
    cout
      <<"\ndistance = " <<d
      <<"\nprojected point = " <<p
      <<"\nface vertices = " <<V
      <<endl;

    origin(0) += .02;
  }

  arr grad;
  d = distanceToConvexHullGradient(grad,X,origin,true);

  cout <<"gradient = " <<grad <<endl;

  QHULL_DEBUG_LEVEL=0;

  N=20;
  D=6;
  X.resize(N,D);
  Qtest::origin.resize(D);
  for(uint k=0;k<20;k++){
    rndUniform(Qtest::origin,-1.2,1.2,false);
    rndUniform(X,-1.,1.,false);
    checkGradient(Convert(Qtest::f, NULL), X, 1e-4);
  }
}

//===========================================================================
//
// 2. test: force closure measure and its gradient
//

namespace FCtest{
  ors::Vector center;
  arr Xn;
  double f(arr *grad,const arr& X,void*){
    return forceClosure(X,Xn,center,.5,10.,grad);
  }
}

void TEST(ForceClosure) {
  uint N=4,i,k;
  arr X(N,3),Xn(N,3),c(3);
  c.setZero();
  rndUniform(X,-1.,1.,false);

  Xn=X;  for(i=0;i<N;i++) Xn[i]() /= length(Xn[i]);

  plotOpengl();
  plotClear(); plotPoints(c); plotPoints(X); plotVectorField(X,Xn);  plot(false);

  arr dFdX;

  double d;
  ors::Vector center;
  center.set(c.p);

  //gradient descent on force closure
  for(k=0;k<1000;k++){
    d=forceClosure(X,Xn,center,.5,10.,&dFdX);
    cout <<"d=" <<d <<endl;
    plotClear(); plotPoints(c); plotPoints(X); plotVectorField(X,Xn);  plot(false);
    X -= .005*dFdX;
    Xn=X;  for(i=0;i<N;i++) Xn[i]() /= -length(Xn[i]);
  }

  //gradient check
  center.setZero();
  for(k=0;k<100;k++){
    rndUniform(X,-1.,1.,false);
    Xn=X;  for(i=0;i<N;i++) Xn[i]() /= -length(Xn[i]);

    d=forceClosure(X,Xn,center,.5,10.,0);
    cout <<"FC= " <<d <<endl;

    FCtest::center=center;
    FCtest::Xn=Xn;
    checkGradient(Convert(FCtest::f, NULL), X, 1e-4);
  }
}

//===========================================================================
//
// 3. test: force closure measure applied on ors scenario
//

void drawInit(void*){
  //glStandardLight();
  //glColor(1.,.5,0.);
  //glDrawAxes(1.);
}

/*
void testFCinOrs(){
  ors::KinematicWorld C;
  C <<FILE("../../configurations/forceClosureTest.ors");

  OpenGL gl;
  gl.add(drawInit,0);
  gl.add(ors::glDrawGraph,&C);
  gl.watch();

  SwiftInterface swift;
  swift.cutoff=1000.;
  swift.init(C);

  uint t;
  arr x,v,grad;
  double fc;
  C.getJointState(x,v);
  for(t=0;t<100;t++){
    C.setJointState(x);
    swift.computeProxies(C,false);
    C.sortProxies(true);

    fc=forceClosureFromProxies(C,0);
    cout <<"forceclosure=" <<fc <<endl;
    // C.reportProxies();
    // c=C.getContactGradient(grad,.1); //generate a gradient pushing away to 10cm distance
    // gl->text.clear() <<"t=" <<t <<"  movement along negative contact gradient (using SWIFT to get contacts)";
    gl.watch();
    //x += inverse(grad)*(-.1*c);
    //x -= .01*grad; //.1 * (invJ * grad);
  }
}*/

int MAIN(int argc, char *argv[]){
  cout <<"QHull version = " <<qhullVersion() <<endl;

  testConvexHull();
  //testForceClosure();
  //testFCinOrs();

  return 0;
}

