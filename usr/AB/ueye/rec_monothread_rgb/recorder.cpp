#include <QCoreApplication>
#include <iostream>
#include <QThread>
#include <unistd.h>
#include <ctime>
#include <sys/types.h>
#include <sys/stat.h>
#include <Core/util.h>
#include <Core/thread.h>

#include "ueyecamera.h"
#include "recorder.h"

using namespace std;

struct RecorderKeys: public OpenGL::GLKeyCall {
  Recorder *rec;

  RecorderKeys(Recorder *r): rec(r) {};
  virtual ~RecorderKeys() {};

  bool keyCallback(OpenGL &);
};

bool RecorderKeys::keyCallback(OpenGL &) {
  switch(rec->getGL()->pressedkey) {
    case 'q':
      rec->quit();
      break;
    case 'r':
      rec->rec();
      break;
    default:
      cout << "Unknown key pressed: " << rec->getGL()->pressedkey << endl;
      break;
  }
  return true;
}

Recorder::Recorder(int w, int h, int f): width(w), height(h), fps(f) {
  throut::throutRegHeading(this, "Recorder: ");
}

Recorder::~Recorder() {
  throut::throutUnregHeading(this);

  delete gl;
  delete keys;

  for(int c = 0; c < numCams; c++)
    delete image[c];
  delete[] image;
}

OpenGL* Recorder::getGL() {
  return gl;
}

void Recorder::setup() {
  numCams = UEyeCamera::getNumCameras();
  throut::throut(this, STRING("numCams = " << numCams));
  if(numCams <= 0) {
    throut::throut(this, "!! No cams connected !!");
    exit(1);
  }

  play_flag = false;
  rec_flag = false;

  initImages();
  initCameras();
  initGui();

  /*
  bool err_flag = false;
  for(int c = 0; c < numCams; c++) {
    if(camera[c]->getErrFlag()) {
      cout << "!! Errors encountered while opening camera " << c+1 << " !!" << endl;
      err_flag = true;
    }
  }
  */
  /*
  if(err_flag)
    exit(1);
    */
}

void Recorder::initImages() {
  image = new byteA*[numCams];
  for(int c = 0; c < numCams; c++)
    image[c] = new byteA(height, width, 3);
}

void Recorder::initCameras() {
  camera = new UEyeCamera(width, height, fps);
  thread = new QThread();

  switch(numCams) {
    case 1:
      camera->setup(0);
      break;
    case 2:
      camera->setup(0, 0);
      break;
    case 3:
      camera->setup(0, 0, 0);
      break;
    case 4:
      camera->setup(0, 0, 0, 0);
      break;
  }

  camera->init();
  if(camera->queryError()) {
    throut::throut(this, "!! Error found initializing cameras !!");
    exit(1);
  }

  camera->open();
  if(camera->queryError()) {
    throut::throut(this, "!! Error found opening cameras !!");
    exit(1);
  }

  connect(thread, SIGNAL(started()), camera, SLOT(camProcess()));
  connect(camera, SIGNAL(started()), this, SLOT(cameraStarted()));
  connect(camera, SIGNAL(finished()), thread, SLOT(quit()));
  connect(camera, SIGNAL(finished()), this, SLOT(closeAndExitCamera()));
  connect(camera, SIGNAL(exited()), this, SLOT(quitQCore()));

  connect(thread, SIGNAL(finished()), thread, SLOT(deleteLater()));
  connect(camera, SIGNAL(exited()), camera, SLOT(deleteLater()));

  camera->moveToThread(thread);
  thread->start();
}

void Recorder::initGui() {
  gl = new OpenGL();

  int rows = (numCams-1) / CAMS_PER_ROW + 1;
  int cols = CAMS_PER_ROW;
  float row, col;
  for(int c = 0; c < numCams; c++) {
    gl->addView(c, &nothing, 0);
    row = c / cols;
    col = c % cols;
    gl->setViewPort(c, col/cols, (col+1)/cols, row/rows, (row+1)/rows);
    gl->views(c).img = image[c];
  }

  keys = new RecorderKeys(this);

  timer.setInterval(1000/30);
  connect(&timer, SIGNAL(timeout()), this, SLOT(updateDisplay()));
}

void Recorder::play() {
  play_flag = !play_flag;
  if(play_flag)
    timer.start();
  else
    timer.stop();
}

void Recorder::rec() {
  rec_flag = !rec_flag;
  if(rec_flag)
    camera->startRec();
  else
    camera->stopRec();
}

void Recorder::quit() {
  timer.stop();
  camera->quit();
}

void Recorder::cameraStarted() {
  play();
  gl->addKeyCall(keys);
}

void Recorder::closeAndExitCamera() {
  camera->close();
  camera->exit();
}

void Recorder::quitQCore() {
  QCoreApplication::quit();
}

void Recorder::nothing(void*) {}

void Recorder::updateDisplay() {
  for(int c = 0; c < numCams; c++)
    camera->queryImage(c, (char*)image[c]->p);
  gl->update();
}

#include "recorder_moc.cpp"

