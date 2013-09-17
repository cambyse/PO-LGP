#pragma once

#include <QObject>
#include <QMutex>
#include <QThread>
#include <Core/array.h>

#include "videowriter.h"

class RecWorker: public QObject {
  Q_OBJECT

  MT::Array<char *> frames;
  int nframes, pframes;

  QMutex frameMutex, quitMutex;

  VideoWriter_x264 *vw;

  bool quit_flag;

  public:
    RecWorker(char *fname, int w, int h, int f);
    ~RecWorker();

    void bufferFrame(char *p);
    void processBuffer();

    void quit();

  private slots:
    void process();
    int processFrame();


  signals:
    void finished();
};

