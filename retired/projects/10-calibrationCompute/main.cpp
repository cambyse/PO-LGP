#define MT_IMPLEMENTATION
#include <MT/ors.h>
#include <MT/optimization.h>
#include <MT/calibration.h>


void loadCalibData(arr& viewData, arr& qData, uint n, const char *filename){
  arr v(4),q(n);
  ifstream fil;
  MT::open(fil,filename);
  for(;;){
    fil >>"LEDcenters= ";
    if(!fil.good()) break;
    v.readRaw(fil);
    fil >>"q=";
    q.readRaw(fil);
    if(!fil.good()) break;
    viewData.append(v);
    qData.append(q);
  }
  viewData.reshape(viewData.N/4,4);
  qData.reshape(qData.N/n,n);
}

struct MyOptimizationProblem:public OptimizationProblem{
  arr x,x0;
  ors::Transformation rel;
  uint t_offset;
  arr q_offset;

  ors::Graph C;
  OpenGL gl;
  int bodyIndex;
  
  arr viewData,qData,PL,PR,K,R,t;
  arr xL_data,xR_data,X_data; //data in standard homogeneous coordinates
  
  void open(){
    //-- parameters
    x.resize(11); x.setZero();
    t_offset=27;
    x(0)=0.; //time lag
    x(1)=-.04;  x(2)=-.025;  x(3)=-0.18; //marker position
    x0 = x;

    //-- ors
    C.init("../../configurations/schunk.ors");
    bodyIndex=C.getBodyByName("m9")->index;
    rel.setText("<t(-.04 -.025 -0.18)>");
    gl.add(glStandardScene,&C);
    gl.add(ors::glDrawGraph,&C);
    gl.watch();
    q_offset = arr(14);
    q_offset = 0;

    //rest: q offset
    loadCalibData(viewData,qData,C.getJointStateDimension(),"../10-calibrationCaptureData/calib.data");

    processData();
  }
  
  void processData(){
    //-- copy to standard homogeneous coordinates
    uint N=qData.d0-t_offset-3;
    X_data.resize(N,4);
    xL_data.resize(N,3);
    xR_data.resize(N,3);
    arr y(3);
    for(uint i=0;i<N;i++){
      C.setJointState(qData[i]+q_offset);
      C.calcBodyFramesFromJoints();
      C.kinematicsPos(y, bodyIndex, &rel);
      X_data [i] = ARR(y(0), y(1), y(2), 1.);
      xL_data[i] = ARR(viewData(i+t_offset,0), viewData(i+t_offset,1), 1.);
      xR_data[i] = ARR(viewData(i+t_offset+3,2), viewData(i+t_offset+3,3), 1.);
    }
  }
  
  double f(arr *grad,const arr& x,int i=-1){
    CHECK(!grad,"can't compute grad!");
    //memmove(rel.pos.p ,&x(1),3*sizeof(double));
    //memmove(q_offset.p,&x(4),7*sizeof(double)); 
    estimateCameraProjectionMatrix(PR, xR_data, X_data);
    estimateCameraProjectionMatrix(PL, xL_data, X_data);
    decomposeCameraProjectionMatrix(K, R, t, PL, true);
    return projectionError(PL, xL_data, X_data);
  }
  
  void replay(){
    uint i;
    orsDrawJoints=false;
    arr P;
    P=~R; P.append(-R*t); P=~P;
    P /= P(2,3);
    gl.camera.setCameraProjectionMatrix(P); //M.gl.P);
    /*ors::Quaternion rot,r2;
    rot.setMatrix((~R).p);
    r2.setDeg(180,0,1,0);
    rot = rot*r2;
    gl.camera.X.pos.set(t.p);
    gl.camera.X.rot=rot;*/
    ors::Shape *s=C.getShapeByName("cameraMarker");
    s->rel.pos.set(t.p);
    s->rel.rot.setMatrix((~R).p);
    for(i=0;i<qData.d0;i++){
      C.setJointState(qData[i]);
      C.calcBodyFramesFromJoints();
      gl.update();
    }
  }
  
  void saveP(){
    ofstream fil("calib_P");
    PL.writeTagged(fil,"PL");
    fil <<endl <<endl;
    PR.writeTagged(fil,"PR");
    fil.close();
  }
  
  void savePredictions(){
    uint N=X_data.d0;
    arr xL,xR,X;
    //-- left projection
    xL = X_data*~PL;
    for(uint i=0;i<N;i++) xL[i]() /= xL(i,2);
    //-- right projection
    xR = X_data*~PR;
    for(uint i=0;i<N;i++) xR[i]() /= xR(i,2);
    //-- triangulation
    X.resizeAs(X_data);
    for(uint i=0;i<N;i++)
      stereoTriangulation(X[i](), xL_data[i], xR_data[i], PL, PR);

    ofstream fil("data.pred");
    for(uint i=0;i<N;i++){
      xL[i].write(fil, " ", "\n", "  ");
      xL_data[i].write(fil, " ", "\n", "  ");
      xR[i].write(fil, " ", "\n", "  ");
      xR_data[i].write(fil, " ", "\n", "  ");
      X[i].write(fil, " ", "\n", "  ");
      X_data[i].write(fil, " ", "\n", "  ");
      fil <<endl;
    }
    fil.close();
  }
};


int main(int argc, char **argv){
  MyOptimizationProblem OP;
  OP.open();
  OP.f(NULL,OP.x0);
  OP.saveP();
  OP.savePredictions();
  //OP.replay();

  return 0;
}


