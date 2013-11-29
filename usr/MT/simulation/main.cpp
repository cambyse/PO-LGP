#include <Ors/roboticsCourse.h>
#include <Motion/feedbackControl.h>
#include <Motion/taskMap_default.h>


void openingSimulator(){
  Simulator S("man.ors");
  cout <<"joint dimensions = " <<S.getJointDimension() <<endl;

  cout <<"initial posture (hit ENTER in the OpenGL window to continue!!)" <<endl;
  S.watch();        //pause and watch initial posture

  arr q;
  S.getJointAngles(q);

  q(0) += 0.1;                 //change the first entry of q-vector
  S.setJointAngles(q);
  S.watch();
  
  q = 0.;                      //set q-vector equal zero
  S.setJointAngles(q);
  S.watch();
}

void reach(){
  Simulator S("man.ors");
  arr q,qdot;
  S.getJointAngles(q);
  qdot.resizeAs(q).setZero();

  cout <<"initial posture (hit ENTER in the OpenGL window to continue!!)" <<endl;
  S.watch();        //pause and watch initial posture

  FeedbackMotionProblem MP(&S.getOrsGraph());
  MP.loadTransitionParameters();
  PDtask *task;
  task = MP.addTask("endeff", new DefaultTaskMap(posTMT, S.getOrsGraph(), "handR", NoVector, "rightTarget"));
  task->setTarget(ARR(0,0,0));
  task->setGainsAsNatural(2.,.95);

  double tau=0.01;
  for(uint i=0;i<10;i++){
    MP.setState(q,qdot);
    arr a = MP.operationalSpaceControl();

    q += tau*qdot;
    qdot += tau*a;

    //optional: pause and watch OpenGL
    S.watch();
  }
}


int main(int argc,char **argv){
  MT::initCmdLine(argc,argv);

  reach();
  return 0;
}
