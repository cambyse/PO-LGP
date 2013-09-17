#pragma once

#include <Core/array.h>
#include <Gui/opengl.h>
#include <GL/glut.h>

#include <QSignalMapper>
#include <QThread>
#include <QTimer>

#include "ueyecamera.h"

struct RecorderKeys;

class Recorder: public QObject {
  Q_OBJECT
  public:
    static const int MAX_CAMS_PER_ROW = 2;

  private:
    // camera stuff
    OpenGL *gl;
    byteA **image;
    int numCams, openCams;

    // camera parameters
    int width, height, fps;
    bool kinect;

    // keys
    RecorderKeys *keys;

    // camera threads
    UEyeCamera **camera;
    QThread **thread;
    QSignalMapper *map;

    QTimer timer;

    // control flags
    bool play_flag, rec_flag;

  public:
    Recorder();
    ~Recorder();

    void setSize(int w, int h);
    void setFPS(int f);
    void setKinect(bool k);
    OpenGL* getGL();

    void setup();
    void initImages();
    void initThreads();
    void initGui();

    void play();
    void rec();
    void quit();

  private:
    static void nothing(void*);

    void collectThreads();

  private slots:
    void startedCam();
    void collectCam(int c);
    void updateDisplay();
};

