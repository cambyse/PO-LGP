#include <MT/robot.h>
#include <MT/perceptionModule.h>
#include <MT/motionPlannerModule.h>
//#include <MT/robot_marcTask.h>
#include <MT/robotActionInterface.h>
#include <TL/decisionMakingModule.h>
#include <signal.h>


bool breakCondition(RobotProcessGroup & robotProcesses){
  return robotProcesses.signalStop || robotProcesses.joy.state(0)==16 || robotProcesses.joy.state(0)==32;
}

void resetPlanner(ReceedingHorizonProcess & planner){
  //planner.threadStop();
  //planner.threadWait();
  planner.threadClose();

  planner.planVar->writeAccess(NULL);
  planner.planVar->converged=false;
  planner.planVar->executed=false;
  planner.planVar->ctrlTime=0.;
  planner.planVar->deAccess(NULL);

  planner.threadLoop();
}

int main(int argc,char** argv) {
  MT::initCmdLine(argc,argv);
  signal(SIGINT,RobotProcessGroup::signalStopCallback);

  RobotActionInterface R;
  R.open();

  // -- start additional processes and variables
  
  // perception
  PerceptionModule perc;
  perc.input=&R.getProcessGroup()->evis.output;
  R.getProcessGroup()->gui.perceptionOutputVar=&perc.output;

  // motion
  FutureMotionPlan planVar;
  FutureMotionGoal goalVar;
  ReceedingHorizonProcess planner;
  planner.planVar = &planVar;
  planner.goalVar = &goalVar;
  planner.sys_parent = &R.getProcessGroup()->ctrl.sys;

  R.getProcessGroup()->gui.planVar  = &planVar;
  R.getProcessGroup()->ctrl.planVar  = &planVar;
  R.getProcessGroup()->ctrl.joyVar  = &R.getProcessGroup()->joy;

  if(R.getProcessGroup()->openBumble) perc.threadLoop();
  planner.threadLoop();

  //brain
  //brain.ors = robotProcesses.ctrl.sys.ors;
  //brain.threadOpen();

  //-- loop pick and place
  
  for(uint k=0;k<10;k++){
    if(R.getProcessGroup()->openBumble) R.perceiveObjects(perc);
    R.pickObject(planner,"cyl1");
    resetPlanner(planner);
    R.placeObject(planner, "cyl1", "table", "cyl2");
    resetPlanner(planner);
    R.plannedHoming(planner, "cyl1", "cyl2");
    resetPlanner(planner);
    cout <<"DDOONNEE!" <<endl;
  }
  R.wait();

}
