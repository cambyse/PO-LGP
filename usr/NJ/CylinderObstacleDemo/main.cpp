#define MLR_IMPLEMENTATION

#include <signal.h>
#include <MT/kin.h>
#include <MT/opengl.h>
#include <MT/robot.h>
#include <MT/soc.h>
//#include <NJ/VisionTrackRoutines.h> connected in perception, connect only once!!

ofstream pos_file("onlinePos.dat");  

struct MyDemo:public TaskAbstraction {
  uint visStep;
  TaskVariable * TV_fNew;
  mlr::Body * obst,*obstV,*future,*target;
  AverageTrack Kal1, Kal2;
  bool started_track, bGoTarget;
  arr Pl,Pr;
  RobotProcessGroup * robotProcesses;
  PerceptionModule * perc;
  void findObstacle();
  virtual void updateTaskVariables(ControllerModule *ctrl); //overloading the virtual
  virtual void initTaskVariables(ControllerModule *ctrl);
  void init(RobotProcessGroup *_master);
  mlr::Vector lastOri;
};


void MyDemo::init(RobotProcessGroup *_master){
  robotProcesses = _master;
  cout << "init TV_q = "<<TV_q->y << endl;
  cout << "init TV_x->x="<<TV_eff->y << endl;
  mlr::IOraw = true;
  started_track = false;

  arr p2;
  p2 <<FILE("../../src/NJ/regparams");
  Pl = p2.sub(0,2,0,3);
  Pr = p2.sub(3,5,0,3);

  robotProcesses->gui.gl->camera.setPosition(-0.5,-7,1);
  robotProcesses->gui.gl->camera.focus(0., -0.5, 1.);

  //find camera location
  mlr::Body * b = new mlr::Body(robotProcesses->gui.ors->bodies);
  mlr::Shape * s=new mlr::Shape(robotProcesses->gui.ors->shapes,b);
  s->type=1;
  s->size[0]=.0; s->size[1]=.0; s->size[2]=0.; s->size[3]=.02;
  s->color[0]=.5; s->color[1]=.2; s->color[2]=.8;
  b->X.p = CameraLocation(Pl);

  mlr::Body * br = new mlr::Body(robotProcesses->gui.ors->bodies);
  mlr::Shape * sr=new mlr::Shape(robotProcesses->gui.ors->shapes,br);
  sr->type=1;
  sr->size[0]=.0; sr->size[1]=.0; sr->size[2]=0.; sr->size[3]=.02;
  sr->color[0]=.5; sr->color[1]=.7; sr->color[2]=.8;
  br->X.p = CameraLocation(Pr);cout << "camera " << b->X.p << " " << br->X.p << endl;

  arr Rot;
  Rot <<FILE("../../src/NJ/RotationMatrix");arr r2; transpose(r2,Rot);Rot = r2;
  Rot = Rot*-1.0;//hack to get positive trace and determinant 1
  mlr::Quaternion q;
  q.setMatrix(Rot.p);
  cout <<"Rot = " <<Rot <<"Q=" <<q <<endl;
  mlr::Vector z; mlr::Quaternion q2;
  q.getZ(z);q2.setRad(PI,z);
  q = q2*q;
  robotProcesses->gui.gl->camera.X->r = q;
  robotProcesses->gui.gl->camera.X->p = b->X.p;
  robotProcesses->gui.gl->camera.setHeightAngle(50);
  robotProcesses->gui.gl->camera.setZRange(.05,10.);
  //robotProcesses->gui.gl->camera.fixedProjectionMatrix=Pr;robotProcesses->gui.gl->camera.fixedProjectionMatrix.append(ARR(0,0,0,1.));
  robotProcesses->gui.ors->getBodyByName("camera")->X.p = (b->X.p+br->X.p)/2;
  robotProcesses->gui.ors->getBodyByName("camera")->X.r = q;

  Kal1.Init();
  Kal2.Init();
  mlr::getParameter(bGoTarget,"goTarget");
}

void MyDemo::initTaskVariables(ControllerModule *ctrl){
  mlr::KinematicWorld &ors=ctrl->ors;
  TV_fNew   = new TaskVariable("posNew",ors,posTVT,"m9","<t( .02   .022 -.366)>",0,0,0);
  TV_fNew->targetType=directTT;
  TVall.append(TV_fNew);
  TaskAbstraction::initTaskVariables(ctrl);//        cout << "variables " << TVall.N << endl;
  TV_col->params=ARR(.15); //change the margin for the collision variable
}


void MyDemo::findObstacle(){
  double time = mlr::realTime();
  if(visStep != robotProcesses->evis.timer.steps && perc->objects.N == 2){//should manuallz remove 0 observations vision.min = 0
    visStep = robotProcesses->evis.timer.steps;
    arr vision1(4),vision2(4);
    //perc->hsvCenters = perc->hsvCenters*(float)pow(2.0,robotProcesses->evis.downScale);
    for(uint i = 0; i < 4; i++)if(i < 2){
      vision1(i) = perc->objects(0)->shapePointsL(0,i);
      vision2(i) = perc->objects(1)->shapePointsL(0,i);
      //  vision1(i) =  robotProcesses->perc.hsvCenters(i);
      //  vision2(i) =  robotProcesses->perc.hsvCenters(i+4);
    }else{
      vision1(i) = perc->objects(0)->shapePointsR(0,i-2);
      vision2(i) = perc->objects(1)->shapePointsR(0,i-2);
    }
    vision1  = vision1*pow(2.0,robotProcesses->evis.downScale);
    vision2  = vision2*pow(2.0,robotProcesses->evis.downScale);

    mlr::Vector val1,val2;
    Kal1.addSample(vision1,Pl,Pr,time);
    Kal2.addSample(vision2,Pl,Pr,time);
    val1 = Kal1.p;val2 = Kal2.p;
    pos_file << val1 << " " << val2 << " " << Kal1.v << " " << Kal2.v << " " << vision1 << " " << vision2 << endl;
    mlr::Vector oriVal = val2-val1;oriVal = oriVal/oriVal.length();
    mlr::Vector poscenter = val2 + oriVal*0.2;//green marker 0.2 from center
    obst->X.p =  poscenter;
    obst->X.v = (Kal1.v+Kal2.v)/2.0;//average speed of 2 tracked markers
    mlr::Frame f;
    f.r.setDiff(mlr::Vector(0,0,1),oriVal);
    obst->X.r = f.r;
    obstV->X =  obst->X;//just vision, is there a better way = 2 ors stucts for vision and collision...
    future->X = obst->X;
    future->X.p += 0.3*(obst->X.v);//predict 0.3 second ahead
    cout << " vision " << endl;
  }
  else if(visStep != robotProcesses->evis.timer.steps)//so reason was absence of evidence
    cout << "no vision" << endl << endl;
}

void MyDemo::updateTaskVariables(ControllerModule *ctrl){
  activateAll(TVall,false); //deactivate all variables

  TV_lim->active=true;
  TV_lim->y_prec = 1e1*0.5;
  TV_col->active=true;
  TV_col->y_prec=1e1;//how to make it more reactive to collisions??
  TV_col->y_target = 0;

  TV_fNew->active = true;//strange, needs to be active here!!!
  TV_fNew->y_prec = 0.;
  TV_fNew->v_prec = 0;

  TV_q->active = true;
  TV_q->y_prec = 0.;

  if (!started_track && maxDiff(ctrl->q_home, TV_q->y)<0.001) {
    cout<<"Starting track."<<endl;
    started_track = true;
  }

  if (started_track) {
    findObstacle();
    if(bGoTarget){	//going to target or not
      arr x = arr(target->X.p.v,3) - TV_fNew->y;
      //ctrl->gui->gl->text.clear() << "dist to target " << norm(x) << " pos " << TV_fNew->y << " targ" << target->X.p;
      double speedM = 0.04;
      if(norm(x) > speedM) x = x*speedM/norm(x);

      TV_fNew->v_target = x;
      TV_fNew->v_prec =1e2;
      TV_q->v_prec = 1e-1;
      TV_q->v_target = ctrl->q_home - TV_q->y;
      double vmax = .2, v=norm(TV_q->v_target);
      if (v>vmax) TV_q->v_target *= vmax/v;
    }
  }
  else {
    cout<<"Starting home "<< maxDiff(ctrl->q_home, TV_q->y)<< endl;

    TV_q->v_prec = 1.;
    TV_q->v_target = ctrl->q_home - TV_q->y;
    double vmax = .3, v=norm(TV_q->v_target);
    if (v>vmax) TV_q->v_target *= vmax/v;
  }
}

inline void rgb2hsv(byte *hsv,byte *rgb){
  float r,g,b,m,v;
  r=rgb[0];    g=rgb[1];    b=rgb[2];

  v=r>g?r:g;  v=v>b?v:b; //max of all = value
  m=r<g?r:g;  m=m<b?m:b; //min of all

  hsv[2]=v;
  if(!v>0) hsv[1]=0; else hsv[1]=(255.f*(v-m))/v;
  if(v==m) hsv[0]=0;
  else if(v==r) hsv[0] = (255.f*(0.f+(g-b)/(v-m)))/6.f;
  else if(v==g) hsv[0] = (255.f*(2.f+(b-r)/(v-m)))/6.f;
  else if(v==b) hsv[0] = (255.f*(4.f+(r-g)/(v-m)))/6.f;
}


int main(int argc,char** argv){
  //sudo chmod a+rw /dev/raw1394
  //sudo chmod a+rw /dev/video1394/0
  mlr::IOraw = true;
  mlr::initCmdLine(argc,argv);
  signal(SIGINT,RobotProcessGroup::signalStopCallback);
  RobotProcessGroup robotProcesses;
  MyDemo demo;
  PerceptionModule perc;
  demo.perc = &perc;

  robotProcesses.ctrl.task=&demo;
  mlr::getParameter(robotProcesses.evis.downScale,"downscale");//1 = 2 times smaller resolution
  robotProcesses.open();//	robot.gui.ors->getBodyByName("OBJECTS")->X.p(0) = 100;
  perc.threadOpen();
  demo.obst = robotProcesses.ctrl.ors.getBodyByName("obstacle");
  demo.obstV = robotProcesses.gui.ors->getBodyByName("obstacle");
  demo.future = robotProcesses.gui.ors->getBodyByName("obstacleF");
  demo.init(&robotProcesses);

  arr atarget; mlr::getParameter(atarget,"target");
  demo.target = robotProcesses.gui.ors->getBodyByName("target");
  demo.target->X.p =  mlr::Vector(atarget(0),atarget(1),atarget(2));

  for(;!robotProcesses.signalStop;){ //catches the ^C key
    robotProcesses.evis.lock.readLock();  perc.lock.writeLock();
    perc.hsvChannelsL = robotProcesses.evis.hsvThetaL;
    perc.hsvChannelsR = robotProcesses.evis.hsvThetaR;
    robotProcesses.evis.lock.unlock();    perc.lock.unlock();

    //evis -> gui
    if(robotProcesses.evis.hsvThetaL.nd==3){
      static uint selectHsv=0;
      robotProcesses.gui.img[3] = evi2rgb(robotProcesses.evis.hsvThetaL[(selectHsv++)%robotProcesses.evis.hsvThetaL.d0]);
      intA boxL;
    }

    //perc->gui
    robotProcesses.gui.img[2] = perc.disp;

    //save the hsv image
    byteA hsvL; hsvL.resizeAs(robotProcesses.evis.cameraL);
    for(uint i=0;i<robotProcesses.evis.cameraL.N/3;i++){
      rgb2hsv(hsvL.p+3*i , robotProcesses.evis.cameraL.p+3*i);
    }
    intA hsvInt;
    copy(hsvInt,hsvL);
    ofstream hsvfile("img.hsv");
    hsvInt.writeRaw(hsvfile);
    hsvfile.close();


    robotProcesses.step();
    perc.threadStepOrSkip(0);
    //  robot.gui.gl->watch();
    if(robotProcesses.gamepad.state(0)==16 || robotProcesses.gamepad.state(0)==32) break;
    //cout <<task.TV_eff->y <<endl;
  }
  robotProcesses.close();
  return 0;
}


