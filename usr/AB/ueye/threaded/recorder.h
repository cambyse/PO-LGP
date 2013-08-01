#ifndef _RECORDER_H_
#define _RECORDER_H_

#include<Core/array.h>
#include<Gui/opengl.h>
#include<GL/glut.h>

#include<QTimer>

struct Recorder : OpenGL::GLKeyCall {
  static const int MAX_CAMS_PER_ROW = 2;

  // for visualization
  OpenGL gl;
  byteA **img;

  unsigned int numCams;
  bool quit, play, rec;

  QTimer timer;

  private slots:
    void updateDisplay();

  public:
    Recorder();
    ~Recorder();

    void record();

    static void nothing(void*) {}

    bool keyCallback(OpenGL &);
    void pressPlay();
    void pressRecord();
    void pressQuit();
};

#endif // _RECORDER_H_

