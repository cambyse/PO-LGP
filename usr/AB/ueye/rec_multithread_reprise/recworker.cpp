#include "recworker.h"

RecWorker::RecWorker(char *fname, int w, int h, int f): nframes(0), pframes(0) {
  vw = new VideoWriter_x264((const char *)fname, w, h, f, 20, "superfast");
}

RecWorker::~RecWorker() {
  delete vw;
}

void RecWorker::process() {
  quitMutex.lock();
  quit_flag = false;
  quitMutex.unlock();

  for(bool quit = false; !quit; ) {
    // TODO some form of wait.. maybe only use events to process single frames?
    if(pframes < nframes)
      processFrame();

    quitMutex.lock();
    quit = quit_flag;
    quitMutex.unlock();
  }

  //emit finished();
}

void RecWorker::quit() {
  quitMutex.lock();
  quit_flag = true;
  quitMutex.unlock();
}

void RecWorker::bufferFrame(char *p) {
  frameMutex.lock();
  frames.append(p);
  nframes++;
  frameMutex.unlock();
}

int RecWorker::processFrame() {
  frameMutex.lock();
  //cout << "Processing frame " << pframes << "/" << nframes << endl;
  if(nframes > pframes) {
    char *p = frames.popFirst();
    vw->addFrame((uint8_t*)p);
    free(p);
    pframes++;
  }
  int rf = nframes - pframes;
  frameMutex.unlock();

  return rf; // returns remaining number of frames
}

void RecWorker::processBuffer() {
  while(processFrame() > 0) {}
}

#include "recworker_moc.cpp"

