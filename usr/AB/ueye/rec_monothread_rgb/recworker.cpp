#include <Core/util.h>
#include <Core/thread.h>
#include "recworker.h"

RecWorker::RecWorker(char *vname, char *sname, int w, int h, int f): nframes(0), pframes(0) {
  throut::throutRegHeading(this, STRING("RecWorker(" << vname << "): "));
  vw = new VideoWriter_x264(vname, w, h, f, 20, "superfast");
  tsfile = fopen(sname, "w");
}

RecWorker::~RecWorker() {
  throut::throutUnregHeading(this);
  delete vw;
  fclose(tsfile);
}

void RecWorker::process() {
  quitMutex.lock();
  quit_flag = false;
  quitMutex.unlock();

  for(bool quit = false; !quit; ) {
    /*
    if(processFrame() == 0)
      usleep(16667);
    */
    processFrame();

    quitMutex.lock();
    quit = quit_flag;
    quitMutex.unlock();
  }

  emit finished();
}

void RecWorker::quit() {
  quitMutex.lock();
  quit_flag = true;
  quitMutex.unlock();
}

void RecWorker::bufferFrame(char *p, char *s) {
  frameMutex.lock();
  frames.append(p);
  timestamps.append(s);
  nframes++;
  frameMutex.unlock();
}

int RecWorker::processFrame() {
  frameMutex.lock();
  bool hasFrame = nframes > pframes;
  frameMutex.unlock();

  if(hasFrame) {
    frameMutex.lock();
    //throut::throut(this, STRING("Processing frame " << pframes << "/" << nframes));

    char *p = frames.popFirst();
    vw->addFrame((uint8_t*)p);
    delete p;

    char *s = timestamps.popFirst();
    fprintf(tsfile, "%4i %s\n", pframes, s);
    fflush(tsfile);
    delete s;

    pframes++;
    frameMutex.unlock();
  }
  
  frameMutex.lock();
  // number of unprocessed frames
  int rf = nframes - pframes;
  frameMutex.unlock();

  return rf; 
}

void RecWorker::processBuffer() {
  while(processFrame() > 0) {}
}

#include "recworker_moc.cpp"

