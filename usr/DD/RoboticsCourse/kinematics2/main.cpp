#include <Ors/roboticsCourse.h>

void multiTask(){
  Simulator S("man.ors");
  arr q,y_target,y,J,W,Phi,PhiJ;
  uint n = S.getJointDimension();

  S.getJointAngles(q);
  W.setDiag(1.,n);

  arr qHome = q;
  double precTask2 = 1.0;

  double precTask3 = 100.0;

  for(uint i=0;i<1000;i++){
    mlr::wait(0.01);
    Phi.clear();
    PhiJ.clear();

    //1st task: track with right hand
    y_target = ARR(-0.2, -0.4, 1.1);
    y_target += .2 * ARR(cos((double)i/20), 0, sin((double)i/20));
    S.kinematicsPos(y,"handR");
    S.jacobianPos  (J,"handR");
    Phi.append((y - y_target)/1e-2);
    PhiJ.append(J / 1e-2);

    //2nd task: joint should stay close to zero
    S.getJointAngles(q);
    Phi.append(precTask2*(q-qHome));
    PhiJ.append(precTask2*eye(n));

    //3rd task: point upward with left hand
    S.kinematicsVec(y, "handL");
    S.jacobianVec(J, "handL");
    Phi.append(precTask3 * (y-ARR(0.0,0.0,1.0)));
    PhiJ.append(precTask3 * J);

    //compute joint updates
    q -= inverse(~PhiJ*PhiJ + W)*~PhiJ* Phi;
    S.setJointAngles(q);
  }
}


int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);
  multiTask();
  return 0;
}
