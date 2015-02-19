#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Motion/taskMaps.h>
#include <Gui/opengl.h>
#include <Optim/optimization.h>
#include <Perception/videoEncoder.h>
#include <iomanip>

void TEST(Matrices) {
  ors::KinematicWorld G("chain.ors");
  MotionProblem MP(G);

  arr y_fL, J_fL;
  MP.world.kinematicsPos(y_fL, J_fL, MP.world.getShapeByName("endeff")->body);
  MP.world.getBodyByName("target")->X.rot.setRandom();
  arr dir = ARRAY(2.,1.,0.);
  ors::Quaternion quat;
  quat.setDiff(ors::Vector(1.,0.,0.),dir);
  arr R = ~quat.getArr();
  cout << "R: " << R << endl;
  arr J_fL0 = R*J_fL;
  J_fL0[0]=0.;

  arr JJ = ~J_fL*inverse(J_fL * ~J_fL)*J_fL0 ;
  arr dq;
  G.getJointState(dq);
  dq = 0.*dq+0.1;
  arr dy = ~R*J_fL*JJ*dq;
  cout << JJ*dq << endl;
  cout << "dy: " << dy << endl;

  cout << ~dy*R[0] << endl;
  cout << ~dy*R[1] << endl;
  cout << ~dy*R[2] << endl;
}

void TEST(GradCheck) {
  ors::KinematicWorld G("chain.ors");
  MotionProblem MP(G);

  arr y_fL, J_fL;
  MP.world.kinematicsPos(y_fL, J_fL, MP.world.getShapeByName("endeff")->body);
  MP.world.getBodyByName("target")->X.rot.setRandom();
  arr dir = ARRAY(2.,1.,0.);
  ors::Quaternion quat;
  quat.setDiff(ors::Vector(1.,0.,0.),dir);
  arr R = ~quat.getArr();
  cout << "R: " << R << endl;
  arr J_fL0 = R*J_fL;
  J_fL0[0]=0.;

  arr JJ = ~J_fL*inverse(J_fL * ~J_fL)*J_fL0 ;
  arr dq;
  G.getJointState(dq);
  dq = 0.*dq+0.1;
  arr dy = ~R*J_fL*JJ*dq;
  cout << JJ*dq << endl;
  cout << "dy: " << dy << endl;

  cout << ~dy*R[0] << endl;
  cout << ~dy*R[1] << endl;
  cout << ~dy*R[2] << endl;
}

int main(int argc,char** argv){
  MT::initCmdLine(argc,argv);
//  testMatrices();
  testGradCheck();
  return 0;
}
