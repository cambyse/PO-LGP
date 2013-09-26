#pragma once

#include <Core/array.h>
#include <Gui/opengl.h>
#include <GL/glut.h>

#include <QThread>
#include <QTimer>

#include "ueyecamera.h"

struct RecorderKeys;

class Recorder: public QObject {
  Q_OBJECT
  public:
    static const int CAMS_PER_ROW = 2;

  private:
    // camera parameters
    int width, height, fps;

    // camera stuff
    OpenGL *gl;
    byteA **image;
    int numCams;

    // keys
    RecorderKeys *keys;

    // camera threads
    UEyeCamera *camera;
    QThread *thread;

    QTimer timer;

    // control flags
    bool play_flag, rec_flag;

  public:
    Recorder(int w, int h, int f);
    ~Recorder();

    OpenGL* getGL();

    void setup();
    void initImages();
    void initCameras();
    void initGui();

    void play();
    void rec();
    void quit();

  private:
    static void nothing(void*);

  private slots:
    void cameraStarted();
    void closeAndExitCamera();
    void quitQCore();
    void updateDisplay();
};

