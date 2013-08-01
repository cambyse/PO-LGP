#include<iostream>
#include<unistd.h>
#include<cstring>
#include<string>
#include<sstream>
#include<wchar.h>
#include<cstdlib>
//TODO remove useless includes

#include<ueye.h>

#include<Core/array.h>
#include<Gui/opengl.h>
#include<GL/glut.h>

extern "C" {
#include<libavcodec/avcodec.h>
#include<libavformat/avformat.h>
#include<libswscale/swscale.h>
}

using namespace std;

struct UeyeRecorder : OpenGL::GLKeyCall {
  static const int MAX_CAMS_PER_ROW = 2;

  // for visualization
  OpenGL gl;
  byteA **img;

  unsigned int numCams;
  bool quit, play, rec;

  Recorder(): quit(false), play(true), rec(false) {
    gl.addKeyCall(this);

    numCams = UeyeCamera::getNumCameras();

    cameras = new UEyeCamera*[numCams];
    cameraThreads = new CameraThread*[numCams];

    for(int c = 0; c < numCams; c++)
      cameras[i] = new UeyeCamera(c, 1280, 1024, 60, "video.avi");

    for(int c = 0; c < numCams; c++)
      cameraThreads = new CameraThread(cameras[i]);

    /*
    if(kinect)
      kinectThread = new KinectThread("kinect.avi");
    */

    // Init GL
    img = new byteA*[numCams];
    for(int c = 0; c < numCams; c++)
      img[c] = new byteA(h, w, 3);

    int nr = (numCams-1) / MAX_CAMS_PER_ROW + 1;
    int nc = MAX_CAMS_PER_ROW;
    float r, c;
    for(int cam = 0; cam < numCams; cam++) {
      gl.addView(cam, &nothing, 0);
      r = cam / nc;
      c = cam % nc;
      gl.setViewPort(cam, c/nc, (c+1)/nc, r/nr, (r+1)/nr);
      gl.views(cam).img = img[cam];
    }
  }

  ~Recorder() {
    /*
    if(kinect)
      delete kinectThread;
    */

    for(int c = 0; c < numCams; c++) {
      delete cameraThreads[c];
      delete cameras[c];
      delete img[c];
    }

    delete[] cameraThreads;
    delete[] cameras;
    delete[] img;
  }

  void record() {
    initAll();
    openAll();
    captureAll();
    closeAll();
  }

  static void nothing(void*) {}

  bool keyCallback(OpenGL &) {
    switch(gl.pressedkey) {
      case 'p':
        play = !play;
        break;
      case 'q':
        quit = true;
        break;
      case 'r':
        rec = !rec;
        break;
      default:
        cout << "Unknown key pressed: " << gl.pressedkey << endl;
        break;
    }
    return true;
  }
};

int main(int argc, char *argv[]) {
  Recorder r;
  r.record();
}

