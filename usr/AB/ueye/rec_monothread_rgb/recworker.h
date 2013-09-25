#pragma once

#include <fstream>
#include <QObject>
#include <QMutex>
#include <QThread>
#include <Core/array.h>

#include <Hardware/VideoWriter/videowriter.h>

class RecWorker: public QObject {
  Q_OBJECT

  private:
    MT::Array<char *> frames, timestamps;
    int nframes, pframes;

    QMutex frameMutex, quitMutex;

    VideoWriter_x264 *vw;
    FILE *tsfile;

    bool quit_flag;

  public:
    RecWorker(char *vname, char *sname, int w, int h, int f);
    ~RecWorker();

    void bufferFrame(char *p, char *s);
    void processBuffer();

    void quit();

  private slots:
    void process();
    int processFrame();

  signals:
    void finished();
};

