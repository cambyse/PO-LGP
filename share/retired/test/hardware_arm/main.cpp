#include <Hardware/schunk/schunk.h>
//#include <MT/socSystem_ors.h>
//#include <MT/soc_inverseKinematics.h>
#include <Gui/opengl.h>
#include <Ors/ors.h>

//this is usually not included -- only here to acess directly
#define NTCAN_CLEAN_NAMESPACE
#define WITH_ESD_CAN 1
#define OSNAME_LINUX 1
#define __LINUX__
#include <lwa/Device/Device.h>
#include <sdh/sdh.h>


#include <signal.h>

#include <stdio.h>
#include <termios.h>
#include <unistd.h>

using namespace SDH;

cSDH *global_sdh=NULL;

void STOP(int){
  if(global_sdh) global_sdh->EmergencyStop();
  global_sdh->Close();
  cerr <<"caught signal -- STOPPED SDH HAND" <<endl;
}



void drawBase(void*){
  glStandardLight(NULL);
  glDrawFloor(10,.8,.8,.8);
  glColor(1.,.5,0.);
}

void loadOrsFile(ors::Graph& C, OpenGL& gl,const char *file="../../configurations/schunk.ors"){
  char *path,*name,cwd[200];
  MT::decomposeFilename(path,name,file);
  getcwd(cwd,200);
  chdir(path);
  
  gl.add(drawBase,0);
  gl.add(ors::glDrawGraph,&C);
  //gl->setClearColors(1.,1.,1.,1.);
  gl.camera.setPosition(7.,-0.,2.);
  gl.camera.focus(0,0,.8);

  MT::load(C,name);
  C.calcBodyFramesFromJoints();

  chdir(cwd);
}


void testSchunk(){
  SchunkArm schunk;
  schunk.open();
  schunk.reportParameters(cout);
  testCube( schunk, 9 );
  printf( "\nClosing device returned %d\n", schunk.pDev->exit() );
  printf( "That's all folks!\n" );
}

// void testControl(){
//   OpenGL gl;
  
//   OrsSystem sys;
//   sys.initBasics(NULL,NULL,&gl,100,4.,false,NULL);

//   uint t;
//   arr q,dq;
//   sys.getq0(q);
//   sys.setq(q);
//   gl.watch("configurations home position");
  
//   bool openArm = MT::getParameter<bool>("openArm");
//   //q_currentReferenceVar schunkVar;
//   SchunkArm schunk; //(&schunkVar);
//   if(openArm) schunk.open();

//   uintA motorIndex(7);
//   motorIndex(0) = sys.ors->getBodyByName("m3")->inLinks(0)->index;
//   motorIndex(1) = sys.ors->getBodyByName("m4")->inLinks(0)->index;
//   motorIndex(2) = sys.ors->getBodyByName("m5")->inLinks(0)->index;
//   motorIndex(3) = sys.ors->getBodyByName("m6")->inLinks(0)->index;
//   motorIndex(4) = sys.ors->getBodyByName("m7")->inLinks(0)->index;
//   motorIndex(5) = sys.ors->getBodyByName("m8")->inLinks(0)->index;
//   motorIndex(6) = sys.ors->getBodyByName("m9")->inLinks(0)->index;

//   floatA qm(7); qm.setZero();
//   if(openArm) schunk.getPos(qm);
//   for(uint m=0;m<7;m++) q(motorIndex(m)) = qm(m);
//   cout <<t <<" schunk position = " <<qm <<endl;

//   sys.setq(q);
//   gl.watch("configuration read out from schunk arm");

//   DefaultTaskVariable TV_eff("endeffector", *sys.ors, posTVT,"m9","<t(0 0 -.24)>",0,0,0);
//   TV_eff.setGainsAsAttractor(20,.2);
//   TV_eff.y_prec=1000.;
//   sys.setTaskVariables(ARRAY((TaskVariable*)&TV_eff));
//   double mean=TV_eff.y(2);

  
//   ofstream log("LOG");
//   double time0,time1,dt;
//   for(t=0;t<5000 && !schunkShutdown;t++){
//     time0=MT::realTime();
    
//     //-- compute new motion
//     TV_eff.y_target(2) = mean + .1*sin(double(t)/200.);
//     cout <<t <<" target=" <<TV_eff.y_target <<flush;
//     dq = q;
//     bayesianIKControl2(sys,q,dq,0);
//     sys.setq(q);

//     //-- send to schunk
//     time1=MT::realTime();
//     unsigned long uiState;
//     unsigned char uiDio;
//     if(openArm) for(uint m=0;m<7;m++){
//       if(!schunk.isOpen) break;
//       //if(openArm) schunk.pDev->setMaxVel(m+3, .1);
//       //if(openArm) schunk.pDev->setMaxAcc(m+3, .1);
//       //schunk.pDev->moveRamp(m+3, q(motorIndex(m)));
//       schunk.pDev->moveStep(m+3, q(motorIndex(m)), 50);
//       //schunk.pDev->moveStepExtended(m+3, q(motorIndex(m)), 300,
//       //                              &uiState, &uiDio, &qm(motorIndex(m)) );
//       //if(m>=4) schunk.pDev->moveVel(m+3, 100.*dq(motorIndex(m))); //EXPERIMENTAL!!!
//     }
//     log <<MT::realTime()-time1;
    
//     //if(openArm) schunk.getPos(qm);
//     cout <<" dt=" <<std::setprecision(5) <<dt <<endl;
//     cout <<" q=" <<q <<flush;
//     cout <<" real_q=" <<qm <<flush;

//     gl.text.clear() <<"time " <<t <<endl;
//     //if(!(t%10)) gl.update();
    
//     //-- total loop time
//     dt=MT::realTime()-time0;
//     log <<' ' <<dt;
//     MT::wait(.01-dt);
//     log <<' ' <<MT::realTime()-time0 <<endl;
//   }
//   if(openArm && schunk.isOpen) schunk.close();
//   log.close();
// }

int main(int argc,char** argv){
  MT::initCmdLine(argc,argv);

  signal(SIGINT,schunkEmergencyShutdown);

  //setRRscheduling(MT::getParameter<uint>("priority"));

  int mode=MT::getParameter<int>("mode");
  switch(mode){
    case 1:  testSchunk();   break;
      //case 2:  testControl();  break;
    default: HALT("");
  }
  
  return 0;
}

