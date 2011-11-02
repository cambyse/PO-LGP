#include <MT/roboticsCourse.h>

void pin_in_a_hole(){
  Simulator S("pin_in_a_hole.ors");
  S.setContactMargin(.02); //this is 2 cm (all units are in meter)
  arr q,y_target,y,J,W,Phi,PhiJ;
  uint n = S.getJointDimension();

  S.getJointAngles(q);
  W.setDiag(1.,n);

  arr y_hole, y_pin0;
  S.kinematicsPos(y_hole,"hole");
  S.kinematicsPos(y_pin0,"pin");
  double y_deviation;

  arr q0,qT;
  q0 = q;
  qT = q;

  for(uint i=0;i<1000;i++){

    Phi.clear();
    PhiJ.clear();

    //1st task: pin positioned within hole
    S.kinematicsPos(y,"pin");
    S.jacobianPos  (J,"pin");
    y_target = y + (y_hole-y); //exponentially approach the hole
    y_deviation = 1e-2; //(deviation is 1/precision^2)

    Phi .append((y - y_target) / y_deviation);
    PhiJ.append(J / y_deviation);

    //2nd task: collisions
    S.kinematicsVec(y, "pin");
    S.jacobianVec(J, "pin");
    y_target = y + (ARR(0.,0.,-1.) - y);
    y_deviation = 1e-2;

    Phi .append((y - y_target) / y_deviation);
    PhiJ.append(J / y_deviation);

    //compute joint updates
    qT = q - .01*inverse(~PhiJ*PhiJ + W)*(W*(q-q0) + ~PhiJ* Phi);
    q = qT;
    S.setJointAngles(q);
    
    cout <<norm(Phi) <<endl;
    if(norm(Phi)<1e-2) break;
  }

  for(uint i=0;i<1000;i++){
    S.kinematicsPos(y_hole,"hole");

    Phi.clear();
    PhiJ.clear();

    //1st task: pin positioned within hole
    S.kinematicsPos(y, "pin");
    S.jacobianPos  (J, "pin");
    y_target = y + .01*(y_pin0-y);
    y_deviation = 1e-1;

    Phi .append((y - y_target) / y_deviation);
    PhiJ.append(J / y_deviation);

    //2nd task: collisions
    S.kinematicsContacts(y);
    S.jacobianContacts(J);
    y_target = ARR(0.); //target is zero collision costs
    y_deviation = 1e-2;

    Phi .append((y - y_target) / y_deviation);
    PhiJ.append(J / y_deviation);

    //compute joint updates
    q -= inverse(~PhiJ*PhiJ + W)*~PhiJ* Phi;
    S.setJointAngles(q);
  }
}

int main(int argc,char **argv){
  MT::initCmdLine(argc,argv);

  pin_in_a_hole();

  return 0;
}
