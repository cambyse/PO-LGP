#include <Ors/roboticsCourse.h>
#include <Core/array.h>

void TEST(Dynamics){
  Simulator S("littleRobot.ors");
  
  arr q,qdot,qddot;
  arr M,Minv,F,u;
  S.getJointAngles(q);
  qdot.resizeAs(q);
  qdot.setZero();
  u=qdot;
  double tau = .01; //duration of one time step = .01sec

  arr Kp,Kd;
  Kp = eye(3);
  Kp(0,0) = 100.0;
  Kp(1,1) = 2.0;
  Kp(2,2) = 100.0;
  Kd = eye(3)*80.;

  //arr qRef = ARR(-3.14/2,0.0,0.0,0.0,0.0,0.0,0.0,0.0);
  arr qRef = ARR(-3.14/2,0.0);

  cout <<"initial posture (hit ENTER in the OpenGL window to continue!!)" <<endl;
    S.watch();        //pause and watch initial posture


  arr J,KpTilde;

  for(uint i=0;i<10000;i++){
    mlr::wait(0.01);
    //ors::Body* pin = S.getOrsGraph().getBodyByName("pin");
    arr force = zeros(3,1);
    force(1,0) = 100.0;
    //force(0,0) = 4.0;
    //force(2,0) = 3.0;
    //S.getOrsGraph().addForce(force, pin);

    S.jacobianPos(J,"arm2");
    S.getDynamics(M,F);
    KpTilde = ~J*Kp*J;
    //cout << KpTilde << endl;
    u = M*(KpTilde*(qRef-q) - ~J*Kd*J*qdot) + F;
    //u = -10.0 * qdot; //a little friction
    if(i > 500 && i < 700) {
      cout << u << endl;
      u += ~J*force;
      cout << u << endl;
      cout << "force" << endl;
    }
    S.stepDynamics(u, tau);
    S.watch(false);
    S.getJointAnglesAndVels(q, qdot);
  }
  
  S.watch();
}

int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);
  testDynamics();
  return 0;
}
