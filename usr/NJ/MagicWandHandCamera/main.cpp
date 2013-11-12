#define MT_IMPLEMENTATION

#include <signal.h>
#include <MT/ors.h>
#include <MT/opengl.h>
#include <MT/robot.h>
#include <MT/soc.h>
#include <NJ/VisionTrackRoutines.h>

ofstream pos_file("onlinePos.dat");  

struct MyDemo:public TaskAbstraction {
  TaskVariable * TV_fNew, * qBiasRotate;
  ors::Body * target;
  bool started_track;
  arr Pl,Pr;
  RobotProcessGroup *robotProcesses;
  PerceptionModule * perc;
  AverageTrack Kal1;
  uint visStep;
  void findTarget();
  virtual void updateTaskVariables(ControllerModule *ctrl); //overloading the virtual
  virtual void initTaskVariables(ControllerModule *ctrl);
  void init(RobotProcessGroup *_master);
};


void MyDemo::init(RobotProcessGroup *_master){
  robotProcesses=_master;
  cout << "init TV_q = "<<TV_q->y << endl;
  cout << "init TV_x->x="<<TV_eff->y << endl;

  MT::IOraw = true;
  started_track = false;

  arr p2;
  MT::load(p2, "../../src/NJ/regparams");
  Pl = p2.sub(0,2,0,3);
  Pr = p2.sub(3,5,0,3);
  Kal1.Init();

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

      arr Rot;
      MT::load(Rot, "../../src/NJ/RotationMatrix");arr r2; transpose(r2,Rot);Rot = r2;
      Rot = Rot*-1.0;//hack to get positive trace and determinant 1
      ors::Quaternion q;
      q.setMatrix(Rot.p);
      cout <<"Rot = " <<Rot <<"Q=" <<q <<endl;
      ors::Vector z; ors::Quaternion q2;
      q.getZ(z);q2.setRad(PI,z);
      q = q2*q;
      robotProcesses->gui.gl->camera.X->r = q;
      robotProcesses->gui.gl->camera.X->p = b->X.p;
      robotProcesses->gui.gl->camera.setHeightAngle(50);
      robotProcesses->gui.gl->camera.setZRange(.05,10.);
      //robotProcesses->gui.gl->camera.fixedProjectionMatrix=Pr;robotProcesses->gui.gl->camera.fixedProjectionMatrix.append(ARR(0,0,0,1.));
      robotProcesses->gui.ors->getBodyByName("camera")->X.p = (b->X.p+br->X.p)/2;
      robotProcesses->gui.ors->getBodyByName("camera")->X.r = q;
}

void MyDemo::initTaskVariables(ControllerModule *ctrl){
  ors::Graph &ors=ctrl->ors;

  TV_fNew   = new TaskVariable("posNew",ors,posTVT,"m9","<t( .02   .022 -.366)>",0,0,0);
  TV_fNew->targetType=directTT;

  arr par(16); par = 0.0; par(0) = 1;
  qBiasRotate   = new TaskVariable("joint bias",ors,qSingleTVT,0,0,0,0,par);
  qBiasRotate->targetType=directTT;

  TVall.append(TUPLE(TV_fNew,qBiasRotate));
  TaskAbstraction::initTaskVariables(ctrl);//        cout << "variables " << TVall.N << endl;
}

void MyDemo::findTarget(){
  double time = MT::realTime();
  if(visStep != robotProcesses->evis.timer.steps &&perc->objects.N > 0 && perc->objects(0)->visionCenter.N == 4){//should manuallz remove 0 observations vision.min = 0
    visStep = robotProcesses->evis.timer.steps;
    arr vision1 =  perc->objects(0)->visionCenter;
   // robotProcesses->evis.hsvCenters = robotProcesses->evis.hsvCenters*(float)pow(2.0,robotProcesses->evis.downScale);
   // for(uint i = 0; i < 4; i++)if(i < 2)
   //   vision1(i) = perc->objects(0)->shapePointsL(0,i);
   // else
   //   vision1(i) = perc->objects(0)->shapePointsR(0,i-2);

    vision1  = vision1*pow(2.0,robotProcesses->evis.downScale);

    Kal1.addSample(vision1,Pl,Pr,time);
    //arr val1a =   Find3dPoint(Pl,Pr,vision);ors::Vector val(val1a(0),val1a(1),val1a(2));
    ors::Vector val = Kal1.p;
    target->X.p = val;

   // pos_file << Kal1.v.length() << " " << lastTarget << endl;
   pos_file << val << " " << Kal1.v << " " << vision1 << endl;
    pos_file.flush();
    robotProcesses->gui.ors->getBodyByName("target")->X.p = val;
  }
  else if(perc->objects.N == 0 || perc->objects(0)->visionCenter.N != 4)
    cout << "no vision" << endl << endl;
}

void MyDemo::updateTaskVariables(ControllerModule *ctrl){
  activateAll(TVall,false); //deactivate all variables

  TV_lim->active=true;
 // TV_lim->y_prec = 1e1;
  TV_col->active=true;
 // TV_col->y_prec=1e-1;

  TV_fNew->active = true;//strange, needs to be active here!!!
  TV_fNew->y_prec = 0.;
  TV_fNew->v_prec = 0;

  qBiasRotate->active = true;
  qBiasRotate->y_prec = 0.;
  qBiasRotate->v_prec = 0;
  qBiasRotate->v_target = -0.01;

  TV_q->active = true;
  TV_q->y_prec = 0.;

  if (!started_track && maxDiff(ctrl->q_home, TV_q->y)<0.001) {
    cout<<"Starting track."<<endl;
    started_track = true;
  }

  if (started_track) {
    findTarget();//call later if it is too slow, or in some intervals
    arr x = arr(target->X.p.v,3) - TV_fNew->y;
    robotProcesses->gui.gl->text.clear() << "dist to target " << norm(x) << " pos " << TV_fNew->y << " targ" << target->X.p;
    double speedM = 0.04;
    if(norm(x) > speedM) x = x*speedM/norm(x);

    TV_fNew->v_target = x;
    TV_fNew->v_prec =1e2;

    qBiasRotate->v_prec = 1e-2;

    TV_q->v_prec = 1e-1;
    TV_q->v_target = ctrl->q_home - TV_q->y;
    double vmax = .2, v=norm(TV_q->v_target);
    if (v>vmax) TV_q->v_target *= vmax/v;
  }
  else {
    cout<<"Starting home "<< maxDiff(ctrl->q_home, TV_q->y)<< endl;

    TV_q->v_prec = 1.;
    TV_q->v_target = ctrl->q_home - TV_q->y;
    double vmax = .3, v=norm(TV_q->v_target);
    if (v>vmax) TV_q->v_target *= vmax/v;
  }
}




int main(int argc,char** argv){
  //sudo chmod a+rw /dev/raw1394
  //sudo chmod a+rw /dev/video1394/0
  MT::IOraw = true;
  MT::initCmdLine(argc,argv);
  signal(SIGINT,RobotProcessGroup::signalStopCallback);
  RobotProcessGroup robotProcesses;
  MyDemo demo;
  robotProcesses.ctrl.task=&demo;
  PerceptionModule perc;
  demo.perc = &perc;

  robotProcesses.open();
  perc.threadOpen();
  robotProcesses.gui.ors->getBodyByName("obstacle")->X.p(2) = 100;
  demo.target = new ors::Body(robotProcesses.gui.ors->bodies);   ///robotProcesses.ors only for control, gui.ors for visualization
  ors::Shape * s=new ors::Shape(robotProcesses.gui.ors->shapes,demo.target);
  s->type=1;
  s->size[0]=.0; s->size[1]=.0; s->size[2]=0; s->size[3]=.02;
  s->color[0]=.5; s->color[1]=.2; s->color[2]=.2;

  demo.target->X.p = ors::Vector(0,0,0);
  demo.init(&robotProcesses);
  //cout << "tennis " << robotProcesses.gui.ors->getShapeByName("tennisBall")->X.p << endl;

  for(;!robotProcesses.signalStop;){ //catches the ^C key
    robotProcesses.evis.lock.readLock();  perc.lock.writeLock();
      perc.hsvChannelsL = robotProcesses.evis.hsvThetaL;
      perc.hsvChannelsR = robotProcesses.evis.hsvThetaR;
      robotProcesses.evis.lock.unlock();    perc.lock.unlock();
      robotProcesses.step();
      perc.threadStepOrSkip(0);
    //  robotProcesses.gui.gl->watch();
    if(robotProcesses.joy.state(0)==16 || robotProcesses.joy.state(0)==32) break;
    //cout <<task.TV_eff->y <<endl;
  }
  robotProcesses.close();
  return 0;
}


