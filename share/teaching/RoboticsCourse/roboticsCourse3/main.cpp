#include <Ors/roboticsCourse.h>


void TEST(DynamicSimulation){
  Simulator S("arm3.ors");
  arr q,qdot,qddot;
  arr M,Minv,F,tau;
  S.getJointAngles(q);
  qdot.resizeAs(q);
  qdot = 0.;
  tau=qdot;

  cout <<"initial posture (hit ENTER in the OpenGL window to continue!!)" <<endl;
  S.watch();        //pause and watch initial posture

  double Delta = .01; //duration of one time step = .01sec
  bool control = MT::getParameter<bool>("control",true);

  S.setDynamicGravity(MT::getParameter<bool>("gravity",true));

  for(uint i=0;i<1000;i++){
    
    //** CONTROLLER part

    //get system equation
    S.getDynamics(M, F);

    if(!control){
      //pure friction
      tau = -.1 * qdot; //pure friction
    }else{
      //separate PDs in each joint separately:
      double lambda = 1./MT_2PI; //corresponds to 1 second period
      double xi = 1.;
      double Kp,Kd;
      Kp = 1./(lambda*lambda);
      Kd = 2.*xi/lambda;
      tau(0) = Kp * (0. - q(0)) + Kd * (0. - qdot(0));
      tau(1) = Kp * (0. - q(1)) + Kd * (0. - qdot(1));
      tau(2) = Kp * (0. - q(2)) + Kd * (0. - qdot(2));

      //compute coordinated tau so that each joint behaves like a PD:
      //(the tau given above is now the desired qddot; see slide 22)
      tau = M*tau + F;
    }
    

    //** SIMULATOR part (you could replace this with the real robot)
    //integrate the system equation (that's all you need as a dynamics engine!!)
    inverse(Minv,M);
    qddot = Minv * (tau - F);
    //Euler integration (Runge-Kutte4 would be much more precise...)
    q    += Delta * qdot;
    qdot += Delta * qddot;
    S.setJointAnglesAndVels(q, qdot);

    //output
    cout <<" t = " <<.01*i
	 <<" E = " <<S.getEnergy()
	 <<" q = " <<q <<endl;
    
  }
}


int main(int argc,char **argv){
  MT::initCmdLine(argc,argv);

  testDynamicSimulation();

  return 0;
}
