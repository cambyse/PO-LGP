#include <iostream>
#include <QCoreApplication>
#include <QThread>
#include <unistd.h>
#include <ctime>
#include <sys/types.h>
#include <sys/stat.h>
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
    case 'p':
      rec->play();
      break;
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

Recorder::Recorder() {
  width = 1280;
  height = 1024;
  fps = 60;
  kinect = false;
}

Recorder::~Recorder() {
  delete gl;
  delete keys;

  for(int c = 0; c < numCams; c++) {
    delete image[c];
    delete thread[c];
  }
  delete[] image;
  delete[] thread;
  delete map;
}

void Recorder::setSize(int w, int h) {
  width = w;
  height = h;
}

void Recorder::setFPS(int f) {
  fps = f;
}

void Recorder::setKinect(bool k) {
  kinect = k;
}

OpenGL* Recorder::getGL() {
  return gl;
}

void Recorder::setup() {
  numCams = UEyeCamera::getNumCameras();
  cout << "Recorder::numCams = " << numCams << endl;
  cout << endl;

  if(numCams == 0) {
    cout << "!! No cams connected !!" << endl;
    exit(1);
  }

  initImages();
  initThreads();
  initGui();

  play_flag = false;
  rec_flag = false;
}

void Recorder::initImages() {
  // init frame containers
  image = new byteA*[numCams];
  for(int c = 0; c < numCams; c++)
    image[c] = new byteA(height, width, 3);
}

void Recorder::initThreads() {
  camera = new UEyeCamera*[numCams];
  thread = new QThread*[numCams];
  map = new QSignalMapper(this);

  for(int c = 0; c < numCams; c++) {
    camera[c] = new UEyeCamera(c+1, width, height, fps);
    thread[c] = new QThread();

    connect(thread[c], SIGNAL(started()), camera[c], SLOT(process()));
    connect(camera[c], SIGNAL(started()), this, SLOT(startedCam()));
    connect(camera[c], SIGNAL(finished()), thread[c], SLOT(quit()));
    connect(thread[c], SIGNAL(finished()), map, SLOT(map()));
    map->setMapping(thread[c], c);

    connect(camera[c], SIGNAL(finished()), camera[c], SLOT(deleteLater()));

    camera[c]->moveToThread(thread[c]);
  }
  connect(map, SIGNAL(mapped(int)), this, SLOT(collectCam(int)));
  openCams = 0;

  // start cameras
  for(int c = 0; c < numCams; c++) {
    camera[c]->init();
    cout << endl;
  }
  cout << endl;
  for(int c = 0; c < numCams; c++) {
    camera[c]->open();
    cout << endl;
  }

  // start threads
  cout << "Recorder:: starting threads." << endl;
  for(int c = 0; c < numCams; c++)
    thread[c]->start();
}

void Recorder::initGui() {
  gl = new OpenGL();

  int rows = (numCams-1) / MAX_CAMS_PER_ROW + 1;
  int cols = MAX_CAMS_PER_ROW;
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
  if(play_flag) {
    timer.start();
  }
  else {
    timer.stop();
  }
}

void Recorder::rec() {
  rec_flag = !rec_flag;
  if(rec_flag)
    for(int c = 0; c < numCams; c++)
      camera[c]->startRec(foldername);
  else
    for(int c = 0; c < numCams; c++)
      camera[c]->stopRec();
}

void Recorder::quit() {
  timer.stop();
  for(int c = 0; c < numCams; c++)
    camera[c]->quit();
}

void Recorder::startedCam() {
  openCams++;
  cout << "Recorder::startedCam(), remaining: " << (numCams-openCams) << endl;

  if(openCams == numCams) {
    play();
    gl->addKeyCall(keys);
  }
}

void Recorder::collectCam(int c) {
  // for some reason this doens't work
  // camera[c]->close();
  
  thread[c]->wait();
  openCams--;
  cout << "Recorder::collectCam(), remaining: " << openCams << endl;

  if(openCams == 0)
    QCoreApplication::quit();
}

void Recorder::nothing(void*) {}

void Recorder::updateDisplay() {
  for(int c = 0; c < numCams; c++)
    camera[c]->getImage((char*)image[c]->p);
  gl->update();
}

void Recorder::newSession() {
  time_t t = time(0);
  struct tm *now = localtime(&t);

  for(int fnum = 0; ; fnum++) {
    foldername.clear() << "./rec/session_"
                      << (now->tm_year-100) << "."
                      << (now->tm_mon + 1) << "."
                      << (now->tm_mday) << "_"
                      << (now->tm_hour) << ":"
                      << (now->tm_min) << "_"
                      << fnum << "/";
    if(mkdir(foldername, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == 0)
      break;
  }
}

#include "recorder_moc.cpp"

