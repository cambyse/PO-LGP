#include <Ors/ors.h>
#include <Gui/opengl.h>

void circle(){
  ors::Graph G;
  OpenGL gl;
  init(G, gl, "pr2_model/pr2_left.ors");
  makeConvexHulls(G.shapes);

  arr q,W;
  uint n = G.getJointStateDimension();
  cout <<"n=" <<n <<endl;
  G.getJointState(q);
  W.setDiag(10.,n);
  W(0,0) = 1e3;

  gl.watch();

  uint endeff = G.getBodyByName("l_wrist_roll_link")->index;

  arr y_target,y,J;
  for(uint i=0;i<1000;i++){
    G.kinematicsPos(y, endeff);
    G.jacobianPos  (J, endeff);
#if 1
    y_target = ARR(0.5, .2, 1.1); 
    y_target += .2 * ARR(0, cos((double)i/20), sin((double)i/20)); 
#else
    y_target = y;
    y_target(2) += 0.1;
#endif
    cout <<i <<" current eff pos = " <<y <<"  current error = " <<length(y_target-y) <<endl;;
    double prec=1e-0;
    q += inverse(~J*J + W/prec)*~J*(y_target - y); 

    G.setJointState(q);
    G.calcBodyFramesFromJoints();
    gl.update();
  }
  gl.watch();
}

int main(int argc,char **argv){
  MT::initCmdLine(argc,argv);

  circle();

  return 0;
}
