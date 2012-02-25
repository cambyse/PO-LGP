#include <motion/motion.h>

struct MyTask:public FeedbackControlTaskAbstraction{
  TaskVariable *TV_eff;
  MyTask(){ requiresInit=true; }
  virtual void initTaskVariables(const ors::Graph &ors){
    TV_eff  = new DefaultTaskVariable("endeffector", ors, posTVT, "m9", "<t(0 0 -.24)>", NULL, NULL, NoArr);
    TVs = LIST<TaskVariable>(*TV_eff);
    requiresInit = false;
  }
  virtual void updateTaskVariableGoals(const ors::Graph& ors){
    TV_eff->active=true;
    TV_eff->targetType=directTT; //specifies the feedback type target
    if(false){ //position control
      TV_eff->y_prec  =1e3;
      TV_eff->y_target = TV_eff->y_target + ARR(0.,0.,.002); //move upward
    }else{ //velocity control
      TV_eff->v_prec =1e3;
      TV_eff->v_target = ARR(0.,0.,.05); //move upward (recall tau=0.01 -> results in same speed)
    }
  }
};

int main(int argn, char** argv){
  MT::initCmdLine(argn, argv);
  //ThreadInfoWin win;
  //win.threadLoopWithBeat(.1);

  // variables
  GeometricState geometricState;
  Action action;
  MotionPlan motionPlan;
  MotionKeyframe frame0,frame1;
  ControllerTask controllerTask;
  HardwareReference hardwareReference;
  SkinPressure skinPressure;
  JoystickState joystickState;

  // processes
  myController controller;
  MotionPlanner motionPlanner;
  MotionPrimitive motionPrimitive(action, frame0, frame1, motionPlan, geometricState);

  // viewers
  PoseViewer<MotionPlan>        view1(motionPlan, geometricState);
  PoseViewer<HardwareReference> view2(hardwareReference, geometricState);
  PoseViewer<MotionKeyframe>    view3(frame1, geometricState);
  
  ProcessL P=LIST<Process>(controller, motionPlanner, motionPrimitive);
  P.append(LIST<Process>(view1, view2, view3));
  
  cout <<"** setting grasp action" <<endl;
  action.writeAccess(NULL);
  action.action = Action::grasp;
  action.objectRef1 = (char*)"target1";
  action.executed = false;
  action.deAccess(NULL);

  cout <<"** setting controller to follow" <<endl;
  controllerTask.writeAccess(NULL);
  controllerTask.mode = ControllerTask::followTrajectory;
  controllerTask.deAccess(NULL);

  uint mode=2;
  switch(mode){
  case 1:{ //serial mode
    motionPrimitive.open();
    motionPrimitive.step();
    motionPlanner.open();
    motionPlanner.step();
  } break;
  case 2:{ //threaded mode
    loopWithBeat(P,.01);
    MT::wait(20.);
  } break;
  case 3:{ //feedback controller
    controller.open();
    view2.open();
    MyTask myTask;
    action.action = Action::noAction;
    controllerTask.writeAccess(NULL);
    controllerTask.mode = ControllerTask::feedback;
    controllerTask.feedbackControlTask = &myTask;
    controllerTask.forceColLimTVs = false;
    controllerTask.deAccess(NULL);
    for(;;){
      controller.step();
      view2.step();
      MT::wait();
    }
  };
  case 4:{ //feedback controller
    MyTask myTask;
    action.action = Action::grasp;
    controllerTask.writeAccess(NULL);
    controllerTask.mode = ControllerTask::feedback;
    controllerTask.feedbackControlTask = &myTask;
    controllerTask.forceColLimTVs = false;
    controllerTask.deAccess(NULL);
    loopWithBeat(P,.01);
    MT::wait(30.);
  };
  }

  close(P);
  birosInfo.dump();
  
  cout <<"bye bye" <<endl;
};



