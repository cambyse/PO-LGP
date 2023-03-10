#define MLR_IMPLEMENTATION
//#define NIKOLAY

#include <signal.h>
#include <MT/kin.h>
#include <MT/opengl.h>
#include <MT/robot.h>
#include <MT/soc.h>
#include <NP/camera.h>
#include <MT/earlyVisionModule.h>
#include <MT/perceptionModule.h>
#include <MT/guiModule.h>

int dispcounter;

#include "ThreadPlanners.h"
#include <NJ/VisionTrackRoutines.h>
ofstream pos_file("onlinePos.dat");  
ofstream cam_file("camPos.dat");
ofstream q_file("qPos.dat");
OpenGL *globalGL=NULL;


struct MultiPlan:public TaskAbstraction {
  double TVariance;//value for good sensing
  uint visStep;
  TaskVariable * TV_fNew;
  mlr::Body * obst,*obstV,*obstV2,*future,*target;
  mlr::Vector lastTargetMa, lastTargetMi;
  AverageTrackPos Kal;
  bool started_track, bGoTarget, bFirstSense;
  double goodCost, colPrec;
  double lastDistance;
  arr Pl,Pr;
  RobotProcessGroup *robotProcesses;
  PerceptionModule * perc;
  bool findTarget();
  void Change();
  virtual void updateTaskVariables(ControllerModule *ctrl); //overloading the virtual
  virtual void initTaskVariables(ControllerModule *ctrl);
  void init(RobotProcessGroup *_master);
  ThreadPlanner recho;
  uint nMod;//dynamic target or obstacle..
  int ChangeMagnitude();
  void CustomJump();
  int counter;
  double FracPlan;
  double PrecPlan, vPrecPlan;
};

void MultiPlan::init(RobotProcessGroup *_master){
  robotProcesses=_master;
  cout << "init TV_q = "<<TV_q->y << endl;
  cout << "init TV_x->x="<<TV_eff->y << endl;
  mlr::IOraw = true;
  started_track = false;
  bFirstSense = false;
  Pl = perc->Pl/ perc->Pl(2,3);Pr = perc->Pr/perc->Pr(2,3);
  if(robotProcesses->openGui){
    robotProcesses->gui.gl->views(0).camera.setPosition(-0.1,-1.2,7.);//-0.5,-7,1);
    robotProcesses->gui.gl->views(0).camera.focus(-0.1, -0.6, 0);
    //find camera location
    if(false){
    mlr::Body * b = new mlr::Body(robotProcesses->gui.ors->bodies);
    mlr::Shape * s=new mlr::Shape(robotProcesses->gui.ors->shapes,b);
    s->type=1;
    s->size[0]=.0; s->size[1]=.0; s->size[2]=0.; s->size[3]=.02;
    s->color[0]=.5; s->color[1]=.2; s->color[2]=.8;
    b->X.pos = CameraLocation(Pl);

    mlr::Body * br = new mlr::Body(robotProcesses->gui.ors->bodies);
    mlr::Shape * sr=new mlr::Shape(robotProcesses->gui.ors->shapes,br);
    sr->type=1;
    sr->size[0]=.0; sr->size[1]=.0; sr->size[2]=0.; sr->size[3]=.02;
    sr->color[0]=.5; sr->color[1]=.7; sr->color[2]=.8;
    br->X.pos = CameraLocation(Pr);cout << "camera " << b->X.pos << " " << br->X.pos << endl;
    }
  }
  Kal.Init();

  mlr::getParameter(nMod,"mod");
  mlr::getParameter(colPrec,"TV_col_yprec");
  mlr::getParameter(TVariance,"TVariance");
  mlr::getParameter(FracPlan,"FracPlan");
  mlr::getParameter(PrecPlan,"PrecPlan");
  mlr::getParameter(vPrecPlan,"vPrecPlan");
}

void MultiPlan::initTaskVariables(ControllerModule *ctrl){
  mlr::KinematicWorld &ors=ctrl->ors;
  TV_fNew   = new TaskVariable("posNew",ors,posTVT,"m9","<t( -.0000031   .000002 -.357)>",0,0,0);//.031   .02 -.357 for opposite finger, better chose another
  TV_fNew->targetType=directTT;
  TVall.append(TV_fNew);//keep in mind, this is in place 0, others come after it
  TaskAbstraction::initTaskVariables(ctrl);
  double margin;
  mlr::getParameter(margin,"swiftMargin");
  mlr::getParameter(goodCost,"goodCost");
  TV_col->params=ARR(margin); //change the margin for the collision variable
}

bool MultiPlan::findTarget(){
  double time = mlr::realTime();
  bool bGoodVision = false;//&& perc->output.objects(0).found
  if(perc->output.objects.N > 0 && perc->output.objects(0).center3d.N>0){//should manuallz remove 0 observations vision.min = 0
    arr cent = perc->output.objects(0).center3d;
    //for(uint i = 0; i < 4; i++)if(i < 2)
    //  vision1(i) = perc->objects(0)->shapePointsL(0,i);
    //else
    //  vision1(i) = perc->objects(0)->shapePointsR(0,i-2);
    Kal.addSample(cent,time);
    cam_file << Kal.p << " " << Kal.pdif.length() << endl; cam_file.flush();
    if(Kal.pdif.length() < TVariance//Kal1.Variance < TVariance*TVariance
        && Kal.p(0) > -0.5 && Kal.p(0) < 0.5
        && Kal.p(1) > -1.5 && Kal.p(1) < -0.5
      && Kal.p(2) > 0.5 && Kal.p(2) < 1.25)    {
     // if(true){
      mlr::Vector val = Kal.p;
      target->X.pos = val; recho.UpdateExtState(target);
      robotProcesses->gui.ors->getBodyByName("target")->X.pos = val;
      robotProcesses->gui.ors2->getBodyByName("target")->X.pos = val;
      recho.UpdateExtState(target);//no influence without setGoals !!, can be ignored for now...
      bGoodVision = true;
      robotProcesses->gui.ors2->getBodyByName("VisMarker")->X.pos(2) = -1.8; robotProcesses->gui.ors2->getBodyByName("VisMarker")->X.pos(1) = -10.8;
    }
  }
  else
    cam_file << " -1 -1 -1 -1" << endl;
  if(! bGoodVision){
    cout << "no vision" << endl << endl;
    robotProcesses->gui.ors2->getBodyByName("VisMarker")->X.pos(2) = 0.8; robotProcesses->gui.ors2->getBodyByName("VisMarker")->X.pos(1) = -.6;
  }
  return bGoodVision;
}

void MultiPlan::Change(){
  //recho.UpdateExtState(target);
  //recho.SetQV0(robotProcesses->ctrl.q_reference,robotProcesses->ctrl.v_reference*1.0);//ctrl->q_reference equals TV_q->y ??
  recho.bUnsetInit = true;
  //TV_q->active = false;//y_target =  TV_q->y;
  robotProcesses->gui.dispSteps = 2;
  robotProcesses->gui.q_trajectory = NULL;//to reset shown trajec
}

int MultiPlan::ChangeMagnitude(){
  mlr::Vector val = target->X.pos ;
  mlr::Vector o = obst->X.pos;
  double COdif = o(1) - val(1);
  double LOdif = o(1)  - lastTargetMa(1);
  double CLdif = val(1)  - lastTargetMa(1);
  if (COdif*LOdif < 0 || (fabs(COdif) < 0.09 && fabs(CLdif) > 0.04) ){pos_file << " init reason1 " << val(1) << " " << o(1) << " " << lastTargetMa(1);
    return 3;}
  if( (val - lastTargetMa).length() > 0.06){pos_file << " init reason2 ";
    return 3;  }
  //return 0;
  if( (val - lastTargetMi).length() > 0.01 && recho.bwdMsg_count > recho.T*0.75)//will give it more time if deviation accumulates
    return 3;//2 for another routine
  return 0;
}

void MultiPlan::CustomJump(){
  mlr::Vector val = target->X.pos ;
  TV_fNew->updateState();
  mlr::Vector diff = val - mlr::Vector(TV_fNew->y(0),TV_fNew->y(1),TV_fNew->y(2));
  bool OnTarget = diff.length() < 0.025;//is it on target
  if(OnTarget) counter ++;
  if(bDATAMODE ||false){
    if(rnd.uni() < -0.00004 || (rnd.uni() < -0.001 && recho.bwdMsg_count == 0)|| (counter%30 == 0 &&OnTarget)){//either on target, or failed to catch orevious atrget
      val = NextRandom(obst->X.pos,lastTargetMa);
      /*int rI = rnd.uni()*4;
		//if( val(0) > 0.0 && fabs(val(1)-0.64)< fabs(val(1)-0.84) )
		if(rI == 0)	val = mlr::Vector(-0.2,-0.64,1);
		//else  if( val(0) < 0.0 && fabs(val(1)-0.64)< fabs(val(1)-0.84))
		if(rI == 1)	val = mlr::Vector(-0.2,-0.84,1);
		//	else  if( val(0) < 0.0 && fabs(val(1)-0.64)> fabs(val(1)-0.84))
		if(rI == 2)	val = mlr::Vector(0.28,-0.84,1);
		//else  if( val(0) > 0.0 && fabs(val(1)-0.64)> fabs(val(1)-0.84))
		if(rI == 3)	val = mlr::Vector(0.28,-0.64,1);
		counter = 0;*/
    }
    else if(rnd.uni() < 0.02)//small perturbations
      val = mlr::Vector(val(0)+ (rnd.uni()-0.5)*0.03,val(1)+ (rnd.uni()-0.5)*0.03,val(2)+ (rnd.uni()-0.5)*0.03);
  }
  else{
    if(OnTarget) {val =  mlr::Vector(val(0)- 0.08*rnd.uni() - 0.04,-0.9,0.95);lastTargetMa(0) = 6666;}//to force init change
  }
  target->X.pos = val;recho.UpdateExtState(target);
  if(robotProcesses->openGui){
    robotProcesses->gui.ors->getBodyByName("target")->X.pos = val;
    robotProcesses->gui.ors2->getBodyByName("target")->X.pos = val;
  }
  int nBigChange = ChangeMagnitude();
  if(!bFirstSense)
    nBigChange = 3;
  bFirstSense =true;
  if(nBigChange == 3){
    Change();
    lastTargetMa = target->X.pos;
    lastTargetMi = lastTargetMa;
    pos_file << " init " << endl;
  }
  else if(nBigChange == 2){
    //recho.SetQV0(robotProcesses->ctrl.q_reference,robotProcesses->ctrl.v_reference*1.0);//1 or 0 ?
    recho.bShiftAll = true;
    pos_file << " shift " << endl;
    lastTargetMi = target->X.pos;
  }
  else
    lastTargetMi = target->X.pos;
}

void MultiPlan::updateTaskVariables(ControllerModule *ctrl){
  recho.SetQV0(ctrl->q_reference,ctrl->v_reference*1.0); //when to set ??
  activateAll(TVall,false); //deactivate all variables
  ctrl->useBwdMsg = false;
  arr qhome = ctrl->q_home;
  if(nMod == 3 || nMod == 1){
    qhome(4) = 0.45;qhome(1) = 0.45;
    if(!bDATAMODE && false)
      qhome =  ARR(-1.32216, -0.731968 ,-1.725, -1.63269, -1.24208, 0.592419, -1.26379);

  }
 /* if(started_track && false){
    TV_fNew->active = true;
    TV_fNew->y_prec = 1e0;
    TV_fNew->y_target = arr(lastTargetMi.p,3);
  }*/
  TV_fNew->updateState();
  arr x = arr(lastTargetMi.p,3) - TV_fNew->y;
  lastDistance = norm(x);
  TV_lim->active=true;
  TV_col->active=true;TV_col->y_prec = colPrec;
  TV_q->active = true;
  TV_q->v_prec = 0.;
  TV_q->y_prec = 0.;
  if (!started_track && maxDiff(qhome, TV_q->y)<0.01) {//ctrl->q_home
    counter = 0;
    cout<<"Starting real demo."<<endl;
    started_track = true;
  }

  if (started_track) {
    if(nMod ==1){
      bool tmpsense= findTarget();
      if(tmpsense)
        bFirstSense = true;
      int nBigChange = ChangeMagnitude();
      pos_file << " change value " << nBigChange << " " << tmpsense << endl;
      if(tmpsense){
        if(nBigChange == 3){
          Change();
          lastTargetMa = target->X.pos;
          lastTargetMi = lastTargetMa;
        }
        else if(nBigChange == 2){
          recho.bShiftAll=true;
          lastTargetMi = target->X.pos;
        }
        else
           lastTargetMi = target->X.pos;
      }
    }
    if(nMod >= 3){//random jumps of target
      CustomJump();
    }
    if(recho.bwdMsg_v.d1==1*ctrl->q_reference.N && recho.bwdMsg_count < recho.T-1
        &&recho.lastCost < goodCost && !recho.bUnsetInit && !recho.bShiftAll
    ){
      if(recho.bwdMsg_count == 0) {FracPlan = FindSpeed(recho.bwdMsg_v);pos_file << " FRAC SPEED " << FracPlan << endl << endl;}
      TV_q->active = true;
      double interp =  recho.bwdMsg_count - floor( recho.bwdMsg_count) ;if (FracPlan < 1.000001) interp = 1.0;
      arr yGoal = interp*recho.bwdMsg_v[recho.bwdMsg_count+1] + (1 -interp)*recho.bwdMsg_v[recho.bwdMsg_count];
      arr vGoal = (recho.bwdMsg_v[recho.bwdMsg_count+1] - recho.bwdMsg_v[recho.bwdMsg_count])/FracPlan;
      pos_file << " y " << yGoal << " " << interp << " " <<  recho.bwdMsg_count+1 << " best " << recho.nBest << endl;
      ctrl->q_reference.writeRaw(q_file);q_file << " ";yGoal.writeRaw(q_file); q_file <<  recho.bwdMsg_count+1 << endl;
      if(recho.bwdMsg_count != 0) cout << " deviate " <<  norm(ctrl->q_reference - yGoal ) << " y " << yGoal(0)
																				    << " cost " << recho.lastCost<< " ; LD: " << lastDistance;
      if(norm(ctrl->q_reference - yGoal ) > 0.05)
        pos_file << " bad deviate " << endl << recho.bwdMsg_v.sub(recho.bwdMsg_count,recho.bwdMsg_count+1,0,6) << endl;
      TV_q->y_target = yGoal;//it i dim 7 already, just q
      TV_q->y_prec = PrecPlan;
      TV_q->v_target = vGoal;
      TV_q->v_prec = vPrecPlan;
      cout <<" best " << recho.nBest << " b# "<<recho.bwdMsg_count +1 << endl;
      recho.bwdMsg_count += 1/FracPlan ;
    }
    else if(recho.bwdMsg_count >= recho.T-1){
      recho.bUnsetInit = true;//recho.bShiftAll = true;
      pos_file <<" semaphors " << recho.bUnsetInit << recho.bShiftAll << endl;
    }
    else{ //stop
      cout << " stopping " << recho.bwdMsg_count << " " << recho.lastCost  << " " << recho.bwdMsg_v.d1 <<
          " semaphors " << recho.bUnsetInit << recho.bShiftAll << endl;      //robotProcesses->gui.gl->text.clear() << "msg# " <<recho.bwdMsg_count << endl;
      TV_q-> v_prec = 1e2;
      TV_q->v_target = TV_q->y*0.0;
        TV_q->y_prec = 1e1;TV_q->y_target = TV_q->y;
      if(recho.bwdMsg_count > 9 && (recho.bwdMsg_count < recho.T-30) && !recho.bUnsetInit)
        pos_file << " costs worsened " << endl;
    }
  }
  else {
    cout<<"Starting home to "<< qhome << " diff " << maxDiff(qhome, TV_q->y)<< endl;
    TV_col->y_prec = 0;
    TV_q->v_prec = 1;
    TV_q->v_target = qhome - TV_q->y;
    double vmax = .15, v=norm(TV_q->v_target);
    if (v>vmax) TV_q->v_target *= vmax/v;
  }
}

int main(int argc,char** argv){
  mlr::IOraw = true;
  mlr::initCmdLine(argc,argv);
  signal(SIGINT,RobotProcessGroup::signalStopCallback);
  RobotProcessGroup robotProcesses;
  MultiPlan demo;

  PerceptionModule perc;
  // perc.input=&robotProcesses.evis.output;
  MyCamera cam;
  EarlyVisionModule evis; evis.input=&cam;
 perc.input=&evis.output;

  //robotProcesses.gui.perceptionOutputVar=&perc.output;
  robotProcesses.gui.cameraVar=&cam;

  demo.perc = &perc;

  robotProcesses.ctrl.task=&demo;//before robotProcesses open() !!
 // robotProcesses.evis.downScale = 1;//2 times smaller resolution
  robotProcesses.open();

  cam .threadLoop();
  evis.threadLoop();
  perc.threadLoop();
//  while(true)
 //   mlr::wait(0.1);

  //perc.threadOpen();
  globalGL = robotProcesses.gui.gl;
  demo.init(&robotProcesses);

  demo.obst = robotProcesses.ctrl.ors.getBodyByName("obstacle");
  if(demo.nMod >= 3 || demo.nMod == 1){//dummy obstacle first, ease up
    mlr::Shape * s = demo.obst->shapes(0);
    arr size = ARR(0,0,0.53,0.08);//if(demo.nMod == 1){size(2) = 0.72; size(3) = 0.06;demo.obst->X.pos(2) += 0.1;demo.obst->X.pos(1) -= 0.16;}
    memmove(s->size, size.p, 4*sizeof(double));    //demo.obst->X.pos = mlr::Vector(0.0,-0.7,0.98);
    if(robotProcesses.openGui){
      copyBodyInfos(*robotProcesses.gui.ors,robotProcesses.ctrl.ors);
      copyBodyInfos(*robotProcesses.gui.ors2,robotProcesses.ctrl.ors);
    }
  }
  arr atarget; mlr::getParameter(atarget,"target");
  mlr::Vector itarget =  mlr::Vector(atarget(0),atarget(1),atarget(2));
  robotProcesses.ctrl.ors.getBodyByName("target")->X.pos = itarget;//planner thread uses this actually, not GL body !!!
  robotProcesses.ctrl.ors.calcBodyFramesFromJoints();///stupid bugg, otherwise target has other values.... yess now fixed
  if(robotProcesses.openGui){
    robotProcesses.gui.ors->getBodyByName("target")->X.pos = robotProcesses.ctrl.ors.getBodyByName("target")->X.pos;
    robotProcesses.gui.ors2->getBodyByName("target")->X.pos = robotProcesses.ctrl.ors.getBodyByName("target")->X.pos;//even worse design - 2 guis with their ors graphs
  }
  demo.target = robotProcesses.ctrl.ors.getBodyByName("target");
  demo.recho.init(robotProcesses.ctrl.sys,&demo,"ST1",1,mlr::getParameter<int>("Tplan"));//after objects on correct place !!
  demo.recho.threadOpen();
  demo.recho.UpdateExtState(demo.target);//critical, otherwise inited badly !!!
  dispcounter = 0;
  for(;!robotProcesses.signalStop;){
    robotProcesses.step();
    //perc.threadStepOrSkip(0);
    if(demo.bFirstSense && demo.started_track &&demo.recho.bReady){//
      demo.recho.threadStepOrSkip(200);//why, what is the meaning....max skips, ignore			//   demo.recho.threadWait();
    }
    if(robotProcesses.gamepad.state(0)==16 || robotProcesses.gamepad.state(0)==32) break;
    if( robotProcesses.gui.q_trajectory.N == 0 && demo.recho.bwdMsg_v.N > 0 && !demo.recho.bUnsetInit &&!demo.recho.bShiftAll && demo.recho.bReady && demo.recho.lastCost < 1)
    {
      robotProcesses.gui.dispSteps = -1;
      //robotProcesses.gui.q_trajectory = demo.recho.helpers(dispcounter)->QDisplayGL2;//aico.q.sub(0,demo.recho.helpers(dispcounter)->aico.q.d0-1,0,6);
      robotProcesses.gui.q_trajectory = demo.recho.bwdMsg_v;
      if(demo.nMod == 3 && false){
      robotProcesses.gui.linesToDisplay.clear();
      for(uint i = 0; i < demo.recho.helpers.N; i++)
        robotProcesses.gui.linesToDisplay.append( 1.0*demo.recho.helpers(i)->XDisplayGL2 );
      }
     // dispcounter = (dispcounter+1)%demo.recho.nPlan;
    }
  }
  robotProcesses.close();
  return 0;
}


