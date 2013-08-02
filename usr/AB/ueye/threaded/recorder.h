#ifndef _RECORDER_H_
#define _RECORDER_H_

#include <Core/array.h>
#include <Gui/opengl.h>
#include <GL/glut.h>
#include <QTimer>

#include "camerathread.h"
#include "ueyecamera.h"

class Recorder: OpenGL::GLKeyCall, QObject {
  Q_OBJECT

  // for visualization
  static const int MAX_CAMS_PER_ROW = 2;
  OpenGL gl;
  byteA **img;

  int width, height, fps;

  int numCams;
  UEyeCamera **cameras;
  CameraThread **cameraThreads;

  bool quit, play, rec;
  bool kinect;

  QTimer timer;


  private slots:
    void updateDisplay();

  public:
    Recorder(int w, int h, int f, bool k);
    ~Recorder();

    void record();

    static void nothing(void*) {}

    bool keyCallback(OpenGL &);
    void pressPlay();
    void pressRecord();
    void pressQuit();
};

#endif // _RECORDER_H_

