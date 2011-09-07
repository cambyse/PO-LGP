#define MT_IMPLEMENTATION
//#define MT_NO_THREADS

#include <MT/robot.h>
#include <signal.h>
#include <MT/specialTaskVariables.h>



#define MYTIME 400




int main(int argn,char** argv){
  MT::initCmdLine(argn,argv);
  signal(SIGINT,RobotModuleGroup::signalStopCallback);

  RobotModuleGroup master;
  
  BwdMsgTask task;
  master.ctrl.task=&task;

  master.open();

  ReceedingHorizonModule recho;
  recho.init(master.ctrl.sys);
  recho.threadOpen();
  
  setGraspGoals(*recho.sys,MYTIME,"S1");
  //soc::straightTaskTrajectory(*recho.sys,recho.planner.q,0);
  recho.step();
  

  uint counter=0;
  
  for(;!master.signalStop;){ //catches the ^C key
    //communication:
    // controller -> recho
    if(counter>20){
      recho.time_shift = counter;
      //recho.q0=master.ctrl.q_reference;
      //recho.v0=master.ctrl.v_reference;
      counter -= counter;
    }

    // recho -> controller
    master.ctrl.bwdMsg_v   =recho.bwdMsg_v   [counter];
    master.ctrl.bwdMsg_Vinv=recho.bwdMsg_Vinv[counter];
    if(recho.planner.cost < 1.){
      if(counter<MYTIME-1) counter++;
    }
    cout <<"\rbwdMsg# " <<counter <<flush;

    // recho -> gui
    if( master.gui.q_trajectory.N == 0 ){
      master.gui.q_trajectory = recho.planner.q;
    }
    
    master.step();
    recho.threadStepOrSkip(200);
    
    if(master.joy.state(0)==16 || master.joy.state(0)==32) break;
  }
  
  recho.threadClose();
  master.close();
  
  return 0;
}


