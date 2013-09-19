#include <iostream>
#include "ueyecamera.h"

using namespace std;

int main(int argc, char *argv[]) {
  int width = 1280;
  int height = 1024;
  int fps = 60;

  UEyeCamera *cam;
  
  cam = new UEyeCamera(0, width, height, fps);
  cam->camInit();
  cam->open();

  cam->InitImageQueue_wr();
  int nframes = 300;
  for(int i = 0; i < nframes; i++)
    cam->grab();
  cam->ExitImageQueue_wr();

  cam->close();
  cam->camExit();

  cout << "============= RESULTS ============" << endl;
  cout << "First Frame: " << cam->sFirst << endl;
  cout << "Last Frame:  " << cam->sLast << endl;
  cout << "(for " << nframes << " frames, at " << fps << "fps, this should be about " << ((double)nframes/fps)<< " seconds)" << endl;
}

