#define MT_IMPLEMENTATION

#include <MT/robot.h>
#include <signal.h>
#include<SD/ISF_GP.h>
#include<SD/graspISF.h>
#include<SD/utils.h>
#include<SD/surface_helpers.h>
#include<SD/percept_ISF_process.h>

#include <MT/earlyVisionModule.h>
#include <MT/perceptionModule.h>

#include <MT/revelModule.h>


struct DummyTask:public TaskAbstraction{//do nothing
  DummyTask(){};
  //virtual void initTaskVariables(ControllerProcess*){};
  virtual void updateTaskVariables(ControllerProcess*){
    activateAll(TVall,false);
    TV_col->active=true;
    TV_lim->active=true;
  };
};

int
get_joy_state(RobotModuleGroup& robot){
    int r = 0;
    if (robot.openJoystick){
      robot.joy.readAccess(NULL);
      r = robot.joy.state(0);
      robot.joy.deAccess(NULL);
    }
    return r;
}

void
get_skin_state(GraspISFTask& task, RobotModuleGroup& robot ){

    SkinPressureVar *skin = &robot.skinPressureVar;

    if(task.open_skin){
      skin->readAccess(NULL);
      task.skin_state =
        ARR( skin->y_real(0), skin->y_real(2),skin->y_real(4));
      skin->deAccess(NULL);
      /* trimm to target value to avoid oscilation */
      for(uint i=0;i<task.TV_skin->y.N;i++)
        if(task.TV_skin->y(i)>task.tv_skin_trgt) task.TV_skin->y(i)=task.tv_skin_trgt;
    }else{ /* simulate */
      task.TV_skin->y=ARR(task.tv_skin_fake,task.tv_skin_fake,task.tv_skin_fake);
    }
}

void
plot_belief_slice(GraspObject_GP &obj, double x1){

  arr X, Y1, Y2;
  double lo, hi;
  uint i;

  obj.getEnclCube(lo,hi);
  X.setGrid(2, lo, hi, 50);
  Y1.resize(X.d0);
  Y2.resize(X.d0);

  FOR1D(X,i){
    Y1(i) = obj.phi(NULL, &Y2(i), ARR(x1, X[i](0), X[i](1)) );
  }

  plotClear();
  //plotFunctionP(X, Y1);
  plotFunction(X, Y1+Y2);

  //plot(true);
  plot(false);
}
 
void
plot_belief_slices(GraspObject_GP &obj){
  double x1;
  uint res=30;
  arr lo,hi;

  plotGnuplot();

  obj.getEnclRect(lo,hi);
  lo -= (hi-lo)*.2; hi += (hi-lo)*.17;
  
  for(x1=lo(0); x1<hi(0); x1+=(hi(0)-lo(0))/res)
    plot_belief_slice(obj,x1);

  plotOpengl();
}

int
main(int argn,char** argv){
  MT::initCmdLine(argn,argv);

  signal(SIGINT,RobotModuleGroup::signalStopCallback);


  GraspObjectVar graspobj;

  //plot_belief_slices(*(GraspObject_GP*)graspobj);

  Percept_ISF_process perceive;

  PerceptionModule perc; 
  GraspISFTask task;
  MT::wait(2);//DBG: why is MT.cfg locked. give time to constructors to read all that stuff

  RobotModuleGroup robot;
  RevelInterface revel;

  perceive.graspobj=&graspobj;
  perc.input=&robot.evis.output;
  robot.gui.perceptionOutputVar=&perc.output;
  perceive.perc_out = &perc.output;

  DummyTask dummy;
  robot.ctrl.task=&dummy;


  perceive.threadLoop();

  // TODO wait until perceive got obj.
  MT::wait(2);

  robot.open();
  perc.threadOpen();

  robot.ctrl.task=&task;
  task.graspobj = graspobj.o;

  // skin -> graspISFtask
  task.open_skin = robot.openSkin;
  get_skin_state(task,robot);

  graspobj.readAccess(NULL);
  SD_INF("Building mesh, patience...");
  graspobj.o->buildMesh();
  robot.gui.gl->addView(0,glDrawMeshObject,graspobj.o);
  robot.gui.gl->addView(1,glDrawMeshObject,graspobj.o);
  if(graspobj.prior){
    graspobj.prior->buildMesh();
    robot.gui.gl->addView(2,glDrawMeshObject,graspobj.prior);
    robot.gui.gl->addView(1,glDrawMeshObject,graspobj.prior);
  }
  graspobj.deAccess(NULL);
  /* simulation view */
  robot.gui.gl->views(0).camera.setPosition(-1.2,-4.,2.);
  robot.gui.gl->views(0).camera.focus(.0,-.2,.6);
  /* real view */
  robot.gui.gl->views(2).camera.setPosition(7.,-.4,2.);
  //robot.gui.gl->views(2).camera.focus(.0,-.2,.6);
  /* lower view  */
  robot.gui.gl->views(1).camera.setPosition(-1.2,-4.,2.);
  robot.gui.gl->views(1).camera.focus(.0,-.2,.6);

  revel.open(100,100);

  for(;!robot.signalStop;){ //catches the ^C key

    // skin -> graspISFtask
    get_skin_state(task,robot);

    robot.step();
    if(get_joy_state(robot)==16 || get_joy_state(robot)==32) break;

    //MT::wait(.02);

    /*
       robot.gui.readAccess(NULL);
       robot.gui.gl->update();
       revel.addFrameFromOpengl();
       robot.gui.deAccess(NULL);
     */

    // for reactive: try get_perc_obj() here but switch off buildMesh for performance
  }
  revel.close();
  robot.close();
  perc.threadClose();

  plotClear();
  task.plot_all();

  return 0;
}
