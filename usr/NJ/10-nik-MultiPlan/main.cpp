#define MT_IMPLEMENTATION

#include <signal.h>
#include <MT/ors.h>
#include <MT/opengl.h>
#include <MT/robot.h>
#include <MT/soc.h>

#include "ThreadPlanners.h"
#include <NJ/VisionTrackRoutines.h>
ofstream pos_file("onlinePos.dat");  
OpenGL *globalGL=NULL;

struct MultiPlan:public TaskAbstraction {
  uint visStep;
  TaskVariable * TV_fNew;
  ors::Body * obst,*obstV,*obstV2,*future,*target;
  ors::Vector lastTarget;
  AverageTrack Kal1, Kal2;
  bool started_track, bGoTarget, bFirstSense;
  double goodCost, colPrec;
  double lastDistance;
  arr Pl,Pr;
  RobotProcessGroup *robotProcesses;
  PerceptionModule * perc;
  void findObstacle();
  void findTarget();
  void Change();
  virtual void updateTaskVariables(ControllerModule *ctrl); //overloading the virtual
  virtual void initTaskVariables(ControllerModule *ctrl);
  void init(RobotProcessGroup *_master);
  ThreadPlanner recho;
  uint nMod;//dynamic target or obstacle..
};

void MultiPlan::init(RobotProcessGroup *_master){
  robotProcesses=_master;
  cout << "init TV_q = "<<TV_q->y << endl;
  cout << "init TV_x->x="<<TV_eff->y << endl;
  MT::IOraw = true;
  started_track = false;
  arr p2;
  MT::load(p2, "../../src/NJ/regparams");
  Pl = p2.sub(0,2,0,3);
  Pr = p2.sub(3,5,0,3);

  robotProcesses->gui.gl->camera.setPosition(-0.5,-7,1);
  robotProcesses->gui.gl->camera.focus(0., -0.5, 1.);
  //find camera location
  ors::Body * b = new ors::Body(robotProcesses->gui.ors->bodies);
  ors::Shape * s=new ors::Shape(robotProcesses->gui.ors->shapes,b);
  s->type=1;
  s->size[0]=.0; s->size[1]=.0; s->size[2]=0.; s->size[3]=.02;
  s->color[0]=.5; s->color[1]=.2; s->color[2]=.8;
  b->X.p = CameraLocation(Pl);

  ors::Body * br = new ors::Body(robotProcesses->gui.ors->bodies);
  ors::Shape * sr=new ors::Shape(robotProcesses->gui.ors->shapes,br);
  sr->type=1;
  sr->size[0]=.0; sr->size[1]=.0; sr->size[2]=0.; sr->size[3]=.02;
  sr->color[0]=.5; sr->color[1]=.7; sr->color[2]=.8;
  br->X.p = CameraLocation(Pr);cout << "camera " << b->X.p << " " << br->X.p << endl;

  robotProcesses->ctrl.ors.getBodyByName("camera")->X.p = (b->X.p+br->X.p)/2;
  robotProcesses->gui.ors->getBodyByName("camera")->X.p = (b->X.p+br->X.p)/2;
  robotProcesses->gui.ors2->getBodyByName("camera")->X.p = (b->X.p+br->X.p)/2;

  Kal1.Init();
  Kal2.Init();

  MT::getParameter(nMod,"mod");
  if(nMod == 1)
    bFirstSense =false;
  else
    bFirstSense =true;;//to see when sensing started

   MT::getParameter(colPrec,"TV_col_yprec");
}

void MultiPlan::initTaskVariables(ControllerModule *ctrl){
  ors::Graph &ors=ctrl->ors;
  TV_fNew   = new TaskVariable("posNew",ors,posTVT,"m9","<t( .02   .022 -.366)>",0,0,0);
  TV_fNew->targetType=directTT;
  TVall.append(TV_fNew);//keep in mind, this is in place 0, others come after it
  TaskAbstraction::initTaskVariables(ctrl);
  double margin;
  MT::getParameter(margin,"swiftMargin");
  MT::getParameter(goodCost,"goodCost");
  TV_col->params=ARR(margin); //change the margin for the collision variable
}

void MultiPlan::findObstacle(){
  double time = MT::realTime();
  if(visStep != robotProcesses->evis.timer.steps ){//should manuallz remove 0 observations vision.min = 0
    visStep = robotProcesses->evis.timer.steps;
    arr vision1(4),vision2(4);
    robotProcesses->evis.hsvCenters = robotProcesses->evis.hsvCenters*(float)pow(2.0,robotProcesses->evis.downScale);
    for(uint i = 0; i < 4; i++){
      vision1(i) = 	robotProcesses->evis.hsvCenters(i);
      vision2(i) = 	robotProcesses->evis.hsvCenters(i+4);
    }

    ors::Vector val1,val2;
    Kal1.addSample(vision1,Pl,Pr,time);
    Kal2.addSample(vision2,Pl,Pr,time);
    val1 = Kal1.p;val2 = Kal2.p;
    pos_file << val1 << " " << val2 << " " << Kal1.v << " " << Kal2.v << " " << vision1 << " " << vision2 << endl;
    ors::Vector oriVal = val2-val1;oriVal = oriVal/oriVal.length();
    ors::Vector poscenter = val2 + oriVal*0.2;//green marker 0.2 from center
    obst->X.p =  poscenter;
    obst->X.v = (Kal1.v+Kal2.v)/2.0;//average speed of 2 tracked markers
    ors::Frame f;
    f.r.setDiff(ors::Vector(0,0,1),oriVal);
    obst->X.r = f.r;

    obstV2->X.r =  obst->X.r;//just ext state changes yhis
    obstV->X =  obst->X;//just vision, is there a better way = 2 ors stucts for vision and collision...
    future->X = obst->X;
    future->X.p += 0.3*(obst->X.v);//predict 0.3 second ahead

    recho.UpdateExtState(obst);
  }
  else
    cout << "no vision" << endl << endl;
}

void MultiPlan::findTarget(){

  double time = MT::realTime();
  if(visStep != robotProcesses->evis.timer.steps &&perc->objects.N > 0 && perc->objects(0)->visionCenter.N == 4){//should manuallz remove 0 observations vision.min = 0
    visStep = robotProcesses->evis.timer.steps;
    arr vision1 = perc->objects(0)->visionCenter;
    //for(uint i = 0; i < 4; i++)if(i < 2)
    //  vision1(i) = perc->objects(0)->shapePointsL(0,i);
    //else
    //  vision1(i) = perc->objects(0)->shapePointsR(0,i-2);
    vision1  = vision1*pow(2.0,robotProcesses->evis.downScale);
    Kal1.addSample(vision1,Pl,Pr,time);
    ors::Vector val = Kal1.p;
    target->X.p = val;
    pos_file << Kal1.v.length() << " " << lastTarget << endl;    // pos_file << val << " " << Kal1.v << " " << vision1 << endl;
    pos_file.flush();
    robotProcesses->gui.ors->getBodyByName("target")->X.p = val;
    robotProcesses->gui.ors2->getBodyByName("target")->X.p = val;
   if(recho.bwdMsg_count == recho.T || recho.bwdMsg_count == 0) robotProcesses->gui.gl->text.clr() << " sense " << val << endl << lastTarget;
   // recho.UpdateExtState(target);//no influence without setGoals !!, can be ignored for now...
  }
  else if(perc->objects.N == 0)
    cout << "no vision" << endl << endl;
}

void MultiPlan::Change(){
 // if( bFirstSense)recho.helpers(0)->nInit = 2;//hack to test how it works with just 1 process
  recho.UpdateExtState(target);
  recho.UnsetInit();
  recho.SetQV0(robotProcesses->ctrl.q_reference,robotProcesses->ctrl.v_reference*.0);//ctrl->q_reference equals TV_q->y ??
  TV_q->y_target =  TV_q->y;
  robotProcesses->gui.dispSteps = 2;
  robotProcesses->gui.q_trajectory = NULL;//to reset shown trajec
  robotProcesses->ctrl.v_reference =   robotProcesses->ctrl.v_reference*0.0;
  robotProcesses->ctrl.bwdMsg_v = NULL;
  robotProcesses->ctrl.bwdMsg_Vinv = NULL;
  cout << endl << " on target "<<(char)7<<" ##########" << robotProcesses->ctrl.q_reference << endl << endl << endl << endl << endl << endl;
}

void MultiPlan::updateTaskVariables(ControllerModule *ctrl){
  activateAll(TVall,false); //deactivate all variables
  ctrl->useBwdMsg = false;
  arr qhome = ctrl->q_home;
  if(nMod == 3 || nMod == 1){
    qhome(4) = 0.4;qhome(1) = 0.4;}
  TV_fNew->updateState();
         arr x = arr(lastTarget.v,3) - TV_fNew->y;
         lastDistance = norm(x);
  TV_lim->active=true;
  TV_col->active=true;TV_col->y_prec = colPrec;
  TV_q->active = true;
  TV_q->v_prec = 0.;
  TV_q->y_prec = 0.;
  static int counter = 0;
  static double tmpco = 0;
  if (!started_track && maxDiff(qhome, TV_q->y)<0.02) {//ctrl->q_home
    counter = 0;
    cout<<"Starting real demo."<<endl;

    started_track = true;
    if(nMod != 1)
      recho.SetQV0(TV_q->y,ctrl->v_reference*1.0);
  }
  //if(nMod == 1 && bFirstSense)
  //  recho.SetQV0(TV_q->y,ctrl->v_reference*1.0);

  if (started_track) {
    if(nMod ==2)
      findObstacle();
    if(nMod ==1){
      findTarget();
      if(Kal1.v.length() < 200000.02 //&& (lastTarget - target->X.p).length() > 0.1
         && target->X.p(0) > -0.5 && target->X.p(0) < 0.5
         && target->X.p(1) > -1.05 && target->X.p(1) < 0.
         && target->X.p(2) > 0.8 && target->X.p(2) < 1.5
          &&(recho.bwdMsg_count == recho.T || robotProcesses->joy.state(0) == 4)
          //&& robotProcesses->joy.state(0) == 4
          ){//if button 3 pressed
        Change();
        tmpco = 0;//for the fractional bwd msg counter
        bFirstSense =true;
        lastTarget = target->X.p;
      }
    }
    if(nMod >= 3){//random jumps of target
      ors::Vector val = target->X.p ;
      TV_fNew->updateState();
      ors::Vector diff = val - ors::Vector(TV_fNew->y(0),TV_fNew->y(1),TV_fNew->y(2));
      bool OnTarget = diff.length() < 0.05;//is it on target
      if(OnTarget) counter ++;
      if(counter%30 == 0 &&OnTarget){
        if( val(0) > 0.0 && val(1) == -0.64)//counter%1000 == 0
          val = ors::Vector(-0.2,-0.64,1);
        else  if( val(0) < 0.0 && val(1) == -0.64)
          val = ors::Vector(-0.2,-0.84,1);
        else  if( val(0) < 0.0 && val(1) == -0.84){
          val = ors::Vector(0.28,-0.84,1);          //   recho.helpers(0)->nInit = 2;
        }
        else  if( val(0) > 0.0 && val(1) == -0.84)
          val = ors::Vector(0.28,-0.64,1);
        Change();
        counter = 0;
      }
      target->X.p = val;
      robotProcesses->gui.ors->getBodyByName("target")->X.p = val;
      robotProcesses->gui.ors2->getBodyByName("target")->X.p = val;
      recho.UpdateExtState(target);
    }
    if(recho.bwdMsg_v.d1==2*ctrl->q_reference.N && recho.bwdMsg_count < recho.T
        &&recho.lastCost < goodCost
        //  && ( ctrl->bwdMsg_v.N == 0 || norm(ctrl->q_reference - ctrl->bwdMsg_v.sub(0,6) ) < 0.7)
        //&& (TV_q->y_target.N == 0 || norm(TV_q->y - TV_q->y_target) < 0.5
    ){
       arr tmp = recho.bwdMsg_v[recho.bwdMsg_count];
      if(recho.bwdMsg_count != 0) cout << " deviate " <<  norm(ctrl->q_reference - ctrl->bwdMsg_v.sub(0,6) ) << "; ";
       ctrl->useBwdMsg=true;
      ctrl->bwdMsg_v   .referToSubDim(recho.bwdMsg_v,   (uint)recho.bwdMsg_count);
      ctrl->bwdMsg_Vinv.referToSubDim(recho.bwdMsg_Vinv,(uint)recho.bwdMsg_count);
       cout <<"bwdMsg#"<<recho.bwdMsg_count << endl;// << " new target " << tmp.sub(0,6) << endl;

      //if(tmpco >= 399.0) tmpco = 399;       tmpco += 1.0;//allow to slow trajectory

       robotProcesses->gui.gl->text.clr() << "msg# " <<recho.bwdMsg_count << endl << " dist " << lastDistance;
      recho.bwdMsg_count ++ ;
     }
   else{ //stop
      cout << " stopping " << recho.bwdMsg_count << " " << recho.lastCost  << " " << recho.bwdMsg_v.d1 << endl;
      TV_q-> v_prec = 100;
      TV_q->v_target = TV_q->y*0.0;
      TV_q->y_prec = 1e1;
      TV_q->y_target = TV_q->y;
    }
  }
  else {
    cout<<"Starting home to "<< qhome << " diff " << maxDiff(qhome, TV_q->y)<< endl;
    TV_col->y_prec = 0;
    TV_q->v_prec = 0.1;
    TV_q->v_target = qhome - TV_q->y;
    double vmax = .15, v=norm(TV_q->v_target);
    if (v>vmax) TV_q->v_target *= vmax/v;
  }
}


int main(int argn,char** argv){
  MT::IOraw = true;
  MT::initCmdLine(argn,argv);
  signal(SIGINT,RobotProcessGroup::signalStopCallback);
  RobotProcessGroup robotProcesses;
  MultiPlan demo;

  PerceptionModule perc;
  demo.perc = &perc;

  robotProcesses.ctrl.task=&demo;//before robotProcesses open() !!
  robotProcesses.evis.downScale = 1;//2 times smaller resolution
  robotProcesses.open();
  perc.threadOpen();
  globalGL = robotProcesses.gui.gl;
  demo.init(&robotProcesses);

  demo.obst = robotProcesses.ctrl.ors.getBodyByName("obstacle");
  if(demo.nMod >= 3 || demo.nMod == 1){//dummy obstacle first, ease up
    ors::Shape * s = demo.obst->shapes(0);
    arr size = ARR(0,0,0.53,0.08);if(demo.nMod == 1){size(2) = 0.72; size(3) = 0.06;demo.obst->X.p(2) += 0.1;demo.obst->X.p(1) -= 0.16;}
    memmove(s->size, size.p, 4*sizeof(double));    //demo.obst->X.p = ors::Vector(0.0,-0.7,0.98);
    copyBodyInfos(*robotProcesses.gui.ors,robotProcesses.ctrl.ors);
    copyBodyInfos(*robotProcesses.gui.ors2,robotProcesses.ctrl.ors);
  }
  demo.obstV = robotProcesses.gui.ors->getBodyByName("obstacle");// cout << " size " << demo.obstV->shapes(0)->size[2] << endl;
  demo.obstV->shapes(0)->mesh.clear();//call always... just size is not enough
  demo.obstV2 = robotProcesses.gui.ors2->getBodyByName("obstacle");
  demo.obstV2->shapes(0)->mesh.clear();
  demo.future = robotProcesses.gui.ors->getBodyByName("obstacleF");
  arr atarget; MT::getParameter(atarget,"target");
  ors::Vector itarget =  ors::Vector(atarget(0),atarget(1),atarget(2));
  if(demo.nMod == 4)
    itarget = ors::Vector(0.28,-0.84,1);
  if(demo.nMod == 5 )
    itarget = ors::Vector(0.28,-0.64,1);
  robotProcesses.ctrl.ors.getBodyByName("target")->X.p = itarget;//planner thread uses this actually, not GL body !!!
  robotProcesses.ctrl.ors.calcNodeFramesFromEdges();///stupid bugg, otherwise target has other values.... yess now fixed
  robotProcesses.gui.ors->getBodyByName("target")->X.p = robotProcesses.ctrl.ors.getBodyByName("target")->X.p;
  robotProcesses.gui.ors2->getBodyByName("target")->X.p = robotProcesses.ctrl.ors.getBodyByName("target")->X.p;//even worse design - 2 guis with their ors graphs
  demo.target = robotProcesses.ctrl.ors.getBodyByName("target");

  demo.recho.init(robotProcesses.ctrl.sys,&demo,"ST1",1,MT::getParameter<int>("Tplan"));//after objects on correct place !!
  demo.recho.threadOpen();

  for(;!robotProcesses.signalStop;){ //catches the ^C key
    robotProcesses.evis.lock.readLock();  perc.lock.writeLock();
    perc.hsvChannelsL = robotProcesses.evis.hsvThetaL;
    perc.hsvChannelsR = robotProcesses.evis.hsvThetaR;
    robotProcesses.evis.lock.unlock();    perc.lock.unlock();
    robotProcesses.step();
    perc.threadStepOrSkip(0);
    if(demo.bFirstSense && demo.started_track )//
      demo.recho.threadStepOrSkip(100);//why, what is the meaning....max skips, ignore
    if(robotProcesses.joy.state(0)==16 || robotProcesses.joy.state(0)==32) break;

    if( robotProcesses.gui.q_trajectory.N == 0 ){
      if(demo.recho. bwdMsg_v.N > 0 )
      {
     //   robotProcesses.gui.q_trajectory = demo.recho.helpers(0)->Clusters;
        robotProcesses.gui.dispSteps = -1;
        robotProcesses.gui.q_trajectory = demo.recho.bwdMsg_v.sub(0,demo.recho.bwdMsg_v.d0-1,0,6);
        cout << " new GUISTATE " << robotProcesses.gui.q_trajectory.sub(0,3,0,6) << endl << endl;
      }
      else{
        cout << " no guiview " << demo.recho.bwdMsg_v.N << " " << demo.recho.lastCost << endl;
      }
    }
  }
  robotProcesses.close();
  return 0;
}


