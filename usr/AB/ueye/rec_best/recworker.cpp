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
  throut::throut(this, "Recording Started");

  quitMutex.lock();
  quit_flag = false;
  quitMutex.unlock();

  for(bool quit = false; !quit; ) {
    /*
    if(processFrame() == 0)
      usleep(16667);
      */
    processBuffer();
    usleep(16667);

    quitMutex.lock();
    quit = quit_flag;
    quitMutex.unlock();
  }

  processBuffer();
  throut::throut(this, STRING("Recording Stopped: " << pframes << " processed"));

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
    char *p = frames.popFirst();
    char *s = timestamps.popFirst();
    //throut::throut(this, STRING("Processing frame " << pframes << "/" << nframes << ", with timestamp " << s));
    frameMutex.unlock();

    vw->addFrame((uint8_t*)p);
    delete p;

    fprintf(tsfile, "%4i %s\n", pframes, s);
    fflush(tsfile);
    delete s;

    pframes++;
  }
  
  frameMutex.lock();
  int rf = nframes - pframes; // number of unprocessed frames as of now
  frameMutex.unlock();

  return rf; 
}

void RecWorker::processBuffer() {
  while(processFrame() > 0) {}
}

#include "recworker_moc.cpp"

