#include <motion/motion.h>
#include <hardware/hardware.h>

struct MyTask:FeedbackControlTaskAbstraction{
  TaskVariable *TV_eff;
  virtual void initTaskVariables(const ors::Graph &ors){
    listDelete(TVs);
    TV_eff  = new DefaultTaskVariable("endeffector", ors, posTVT, "m9", "<t(0 0 -.24)>", NULL, NULL, NoArr);
    TVs = LIST<TaskVariable>(*TV_eff);
    requiresInit = false;
    TV_eff->active=true;
  }
  virtual void updateTaskVariableGoals(const ors::Graph& ors){
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

  // variables
  GeometricState geometricState;
  ControllerTask controllerTask;
  HardwareReference hardwareReference;

  // processes
  Controller controller;

  // viewers
  PoseViewer<HardwareReference> view(hardwareReference, geometricState);
  
  ProcessL P=LIST<Process>(controller, view);

  MyTask myTask;
  controllerTask.writeAccess(NULL);
  controllerTask.mode = ControllerTask::feedback;
  controllerTask.feedbackControlTask = &myTask;
  controllerTask.forceColLimTVs = false;
  controllerTask.deAccess(NULL);

  uint mode=MT::getParameter<uint>("mode", 1);
  if(mode==0){
    controller.open();
    view.open();
    for(;;){
      controller.step();
      view.step();
      MT::wait();
    }
  }
  if(mode==1){
    loopWithBeat(P,.01);
    MT::wait(30.);
  }

  close(P);
  birosInfo.dump();
  
  cout <<"bye bye" <<endl;
};



