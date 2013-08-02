using namespace std;

#include "recorder.h"
#include "ueyecamera.h"

Recorder::Recorder(int w, int h, int f, bool k): width(w),
                                                  height(h),
                                                  fps(f),
                                                  kinect(k) {
  quit = false;
  play = true;
  rec = false;

  gl.addKeyCall(this);

  numCams = UEyeCamera::getNumCameras();

  cameras = new UEyeCamera*[numCams];
  cameraThreads = new CameraThread*[numCams];

  for(int c = 0; c < numCams; c++)
    cameras[c] = new UEyeCamera(c, 1280, 1024, 60);

  for(int c = 0; c < numCams; c++)
    cameraThreads[c] = new CameraThread(cameras[c], true, "./rec/");

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

  connect(&timer, SIGNAL(timeout()), this, SLOT(updateDisplay()));
}

Recorder::~Recorder() {
  for(int c = 0; c < numCams; c++) {
    delete cameraThreads[c];
    delete cameras[c];
    delete img[c];
  }

  delete[] cameraThreads;
  delete[] cameras;
  delete[] img;

  /*
  if(kinect)
    delete kinectThread;
  */
}

void Recorder::record() { }

bool Recorder::keyCallback(OpenGL &) {
  switch(gl.pressedkey) {
    case 'p':
      pressPlay();
      break;
    case 'q':
      pressQuit();
      break;
    case 'r':
      pressRecord();
      break;
    default:
      cout << "Unknown key pressed: " << gl.pressedkey << endl;
      break;
  }
  return true;
}

void Recorder::pressPlay() {
  play = !play;
  if(play) {
    for(int c = 0; c < numCams; c++)
      cameraThreads[c]->start();

    /*
    if(kinect)
      kinectThread->start();
    */

    //display frequency (does not affect recording)
    timer.start(1000/30);
  }
}

void Recorder::pressRecord() {
  rec = !rec;
}

void Recorder::pressQuit() {
  quit = true;

  for(int c = 0; c < numCams; c++)
    cameraThreads[c]->stop();

  /*
  if(kinect)
    kinectThread->stop();
  */

  timer.stop();
}

void Recorder::updateDisplay() {
  gl.update();
}

