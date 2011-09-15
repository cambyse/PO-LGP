#define MT_IMPLEMENTATION
//#define MT_NO_THREADS

#include <MT/robot.h>
#include <signal.h>
#include <MT/specialTaskVariables.h>



#define MYTIME 400




int main(int argn,char** argv){
  MT::initCmdLine(argn,argv);
  signal(SIGINT,RobotProcessGroup::signalStopCallback);

  RobotProcessGroup robotProcesses;
  
  BwdMsgTask task;
  robotProcesses.ctrl.task=&task;

  robotProcesses.open();

  ReceedingHorizonModule recho;
  recho.init(robotProcesses.ctrl.sys);
  recho.threadOpen();
  
  setGraspGoals(*recho.sys,MYTIME,"S1");
  //soc::straightTaskTrajectory(*recho.sys,recho.planner.q,0);
  recho.step();
  

  uint counter=0;
  
  for(;!robotProcesses.signalStop;){ //catches the ^C key
    //communication:
    // controller -> recho
    if(counter>20){
      recho.time_shift = counter;
      //recho.q0=robotProcesses.ctrl.q_reference;
      //recho.v0=robotProcesses.ctrl.v_reference;
      counter -= counter;
    }

    // recho -> controller
    robotProcesses.ctrl.bwdMsg_v   =recho.bwdMsg_v   [counter];
    robotProcesses.ctrl.bwdMsg_Vinv=recho.bwdMsg_Vinv[counter];
    if(recho.planner.cost < 1.){
      if(counter<MYTIME-1) counter++;
    }
    cout <<"\rbwdMsg# " <<counter <<flush;

    // recho -> gui
    if( robotProcesses.gui.q_trajectory.N == 0 ){
      robotProcesses.gui.q_trajectory = recho.planner.q;
    }
    
    robotProcesses.step();
    recho.threadStepOrSkip(200);
    
    if(robotProcesses.joy.state(0)==16 || robotProcesses.joy.state(0)==32) break;
  }
  
  recho.threadClose();
  robotProcesses.close();
  
  return 0;
}


