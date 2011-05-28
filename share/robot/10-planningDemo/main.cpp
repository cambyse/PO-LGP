#include <MT/robot.h>
#include <MT/perceptionModule.h>
#include <MT/motionPlannerModule.h>
#include <MT/robot_marcTask.h>
#include <MT/robotActionInterface.h>
#include <TL/decisionMakingModule.h>
#include <signal.h>


bool breakCondition(RobotModuleGroup & master){
  return master.signalStop || master.joy.state(0)==16 || master.joy.state(0)==32;
}

void resetPlanner(ReceedingHorizonProcess & planner){
  planner.planVar->deAccess(NULL);
  planner.goalVar->deAccess(NULL);
  planner.threadClose();
  planner.threadOpen();
  planner.planVar->readAccess(NULL);
  planner.goalVar->writeAccess(NULL);
}

int main(int argn,char** argv) {
  MT::initCmdLine(argn,argv);
  signal(SIGINT,RobotModuleGroup::signalStopCallback);

  RobotActionInterface R;
  R.open();

  //********** launch all modules
  RobotModuleGroup &master = *R.getProcessGroup(); //bugged ????
  PerceptionModule perc;  perc.input=&master.evis.output;
  //MotionPlannerModuleGroup motion;
  //DecisionMakingModule brain;
  master.gui.perceptionOutputVar=&perc.output;

  TaskAbstraction& task = *R.getTask();
  //master.ctrl.task=&task;

  //robot group
  //master.open();

  //perc
  perc.threadOpen();

  //motion
  FutureMotionPlan planVar;
  FutureMotionGoal goalVar;
  ReceedingHorizonProcess planner;
  planner.planVar = &planVar;
  planner.goalVar = &goalVar;
  planner.sys_parent = &master.ctrl.sys;

  master.gui.planVar  = &planVar;
  task.planVar = &planVar;
  task.joyVar  = &master.joy;

  planner.threadOpen();

  //brain
  //brain.ors = master.ctrl.sys.ors;
  //brain.threadOpen();

  task.controlMode=stopCM;
  int STATE=-1;
  if(!master.openBumble) STATE=0;



#if 0
  planner.threadLoop();
  perc.threadLoop();
  while(!breakCondition(master)){
    while(!R.perceiveObjects(perc) && !breakCondition(master))
    {
      master.step();
    }
    while(!R.reachGrasp(planner,"cyl1") && !breakCondition(master)){
      master.step();
    }
    while(!R.closeHandAndAttach() && !breakCondition(master)){
      master.step();
    }

    while(!R.wait4PlannerAndReset(planner)  && !breakCondition(master)){
      //master.step();
    }

    while(!R.place(planner, "cyl1", "table", "cyl2")  && !breakCondition(master)){
      master.step();
    }
    while(!R.stopMotion()  && !breakCondition(master)){
      master.step();
    }

    while(!R.openHandReattach("cyl1","cyl2")  && !breakCondition(master)){
      master.step();
    }

    while(!R.wait4PlannerAndReset(planner)  && !breakCondition(master)){
      //	master.step();
    }

    while(!R.homing(planner, "cyl1", "cyl2")  && !breakCondition(master)){
      master.step();
    }


    while(!R.wait4PlannerAndReset(planner,50)  && !breakCondition(master)){
      master.step();
    }
    for(uint i=0;i<perc.output.objects.N;i++) perc.output.objects(i).found=0;

    task.controlMode=stopCM;
    planVar.deAccess(NULL);
    goalVar.deAccess(NULL);
    planner.threadClose();
    planner.threadOpen();
    planVar.readAccess(NULL);
    goalVar.writeAccess(NULL);

  }

#else
  //********** main loop
  for (;!master.signalStop && master.joy.state(0)!=16 && master.joy.state(0)!=32;) {
    cout << "  STATE " << STATE << " ";
    //brain -> motion
    /*if(brain.actionIsReady && brain.action!=DecisionMakingModule::SYMBOLIC_ACTION__NO_ACTION_FOUND){
      motion.graspTargetBodyId = brain.actionArgument;
    }*/
    goalVar.writeAccess(NULL);
    planVar.readAccess(NULL);

    switch(STATE){
    case -1:{
      if(R.perceiveObjects(perc)){
	STATE++;
	cout <<'\7';
      }
    } break;
    case 0:{ //grasp
      if(R.reachGrasp(planner,"cyl1"))
        STATE++;
    } break;
    case 1:{ //stop and reattach object
      if(R.reattach("cyl1"))
        STATE++;
    } break;
    case 2:{ //close hand
      if(R.closeHandAndAttach())
        STATE++;
    } break;
    case 3:{ //wait until planner is ready with current step, then reset the planner
      if(R.wait4PlannerAndReset(planner)){
        resetPlanner(planner);
        /* planVar.deAccess(NULL);
        goalVar.deAccess(NULL);
        planner.threadClose();
        planner.threadOpen();
        planVar.readAccess(NULL);
        goalVar.writeAccess(NULL);*/
        STATE++;
      }
    } break;
    case 4:{  //place plan and execute
      if(R.place(planner, "cyl1", "table", "cyl2"))
        STATE++;
    } break;
    case 5:{
      if(R.stopMotion())
        STATE++;
    } break;
    case 6:{
      if(R.openHandReattach("cyl1","cyl2"))
        STATE++;
    } break;
    case 7:{
      if(R.wait4PlannerAndReset(planner)){
        resetPlanner(planner);
        STATE++;
      }
    } break;
    case 8:{
     /* if(planVar.converged && planVar.cost > 40){
        task.controlMode=stopCM;
        resetPlanner(planner);
        planVar.converged=false;
        break;
      }*/
      if(R.homing(planner, "cyl1", "cyl2"))
        STATE++;
    } break;
    case 9:{
      cout << endl << " planner homed " << master.ctrl.q_reference << endl << endl;
      if(R.wait4PlannerAndReset(planner,50)){
        for(uint i=0;i<perc.output.objects.N;i++) perc.output.objects(i).found=0;
        STATE=-1;
        if(!master.openBumble) STATE=0;
        STATE=10;
      }
    }break;
    case 10:{
      static int count=0;  count++;
      if(count>300){
	count = 0;
        STATE=20;
      }
    }break;
    case 20:{
      task.controlMode=stopCM;
      resetPlanner(planner);
      STATE = -1;
      if(!master.openBumble) STATE=0;
    } break;
    case 21:{
      task.controlMode=homingCM;
    } break;
    default: HALT("");
    }

    cout <<'\r';
    planVar.write(cout);
    cout <<" controlMode= " <<task.controlMode <<flush;

    goalVar.deAccess(NULL);
    planVar.deAccess(NULL);

    //------------STEPPING
    master.step();
    planner.threadStepOrSkip(0);
    perc.threadStepOrSkip(0);
    //brain.threadStepOrSkip(0);
    //reportGlobalProcessGraph();
  }
#endif

  //brain.threadClose();
  planner.threadClose();
  perc.close();
  master.close();
}
