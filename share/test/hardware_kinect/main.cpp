#include <System/engine.h>
#include <Gui/opengl.h>
#include <signal.h>

void lib_hardware_kinect();
void lib_Perception();

bool stopnow=false;
void STOP(int){
  MT_MSG("SIGINT callback");
  stopnow=true;
}

int main(int argc, char **argv) {
  lib_hardware_kinect();
  lib_Perception();

  signal(SIGINT,STOP);

  struct MySystem:System{
    ACCESS(byteA, kinect_rgb);
    ACCESS(floatA, kinect_depth);
    ACCESS(arr, kinect_points);
    MySystem(){
      addModule("KinectPoller", NULL, ModuleThread::loopWithBeat, .1);
      addModule("ImageViewer",NULL,STRINGS("kinect_rgb"), ModuleThread::listenFirst, 0);
      addModule("Kinect2PointCloud", NULL, ModuleThread::loopWithBeat, 1.);
      addModule("PointCloudViewer",NULL,STRINGS("kinect_points", "kinect_pointColors"), ModuleThread::listenFirst, 0);
      complete();
    }
  } S;

  cout <<S <<endl;

  OpenGL gl;
  arr pts;

  engine().open(S);

  MT::timerStart();
  for(uint i=0;i<1000;i++){
    if(stopnow) break;
    S.kinect_depth.var->waitForNextWriteAccess();
    copy(gl.background, S.kinect_depth.get()/10.f);
    gl.update();
  }
  cout <<"fps = " <<1000./MT::timerRead() <<endl;

  engine().close(S);

  cout <<"bye bye" <<endl;
}

