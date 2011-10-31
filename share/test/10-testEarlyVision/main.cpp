#include <signal.h>

#include <MT/process_internal.h>
#include <NP/camera.h>
#include <NP/uvccamera.h>

#include <MT/vision.h>
#include <MT/earlyVisionModule.h>


static bool STOP=false;
void shutdown(int){
  cout <<"shutting down!" <<endl;
  STOP=true;
}

int main(int argn,char** argv){
  MT::initCmdLine(argn,argv);
  signal(SIGINT,shutdown);

  //Variables
  CameraImages currentCameraImages;

  //Processes
  EarlyVisionModule evis;
  CameraModule cam;
  cam.output = &currentCameraImages;
  evis.input = &currentCameraImages;
#if 0
  currentCameraImages.loadDummyImages();
#endif
  
  cam .threadOpen();
  evis.threadOpen();

  cam .threadLoop();
  evis.threadLoop();
  
  for(uint i=0;!STOP && i<10000;i++){
    MT::wait(.05);

    if(evis.hsvCenters.N){
      cout <<"hsv centers = " <<evis.hsvCenters <<endl;
      floatA hsvStereo(3);
      hsvStereo(0) = evis.hsvCenters(0);
      hsvStereo(1) = evis.hsvCenters(1);
      hsvStereo(2) = sqrt(MT::sqr(evis.hsvCenters(0)-evis.hsvCenters(2))+
			  MT::sqr(evis.hsvCenters(1)-evis.hsvCenters(3)));
      cout <<"hsv stereo  = " <<hsvStereo <<endl;
      floatA hsvWorld(3);
      //evis.calib.stereo2world(hsvWorld,hsvStereo);
      //cout <<"hsv world   = " <<hsvWorld <<endl;
    }
    
    evis.output.readAccess(NULL);
    if(evis.output.hsvThetaL.N){
      write_ppm(evi2rgb(evis.output.hsvThetaL[0]),"hsvTheta.ppm");
    }
    evis.output.deAccess(NULL);
  }

  evis.threadClose();
  cam.threadClose();
  
  return 0;
}
