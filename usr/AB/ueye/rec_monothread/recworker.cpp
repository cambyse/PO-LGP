#include "recworker.h"

RecWorker::RecWorker(char *fname, int w, int h, int f): nframes(0) {
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
    if(nframes)
      processFrame();

    quitMutex.lock();
    quit = quit_flag;
    quitMutex.unlock();
  }

  processBuffer();

  emit finished();
}

void RecWorker::quit() {
  /*
  quitMutex.lock();
  quit_flag = true;
  quitMutex.unlock();
  */
}

int RecWorker::bufferFrame(char *p) {
  //frameMutex.lock();
  frames.append(p);
  int nf = ++nframes;
  //frameMutex.unlock();

  cout << "BUFFERING FRAME" << nframes << endl;
  return nf;
}

int RecWorker::processFrame() {
  cout << "PROCESSING FRAME" << nframes << endl;
  return 0;

  int nf = 0;
  //frameMutex.lock();
  if(nframes > 0) {
    char *p = frames.popFirst();
    nf = --nframes;
    vw->addFrame((uint8_t*)p);
    free(p);
  }
  //frameMutex.unlock();

  return nf;
}

void RecWorker::processBuffer() {
  for(bool done = false; !done; )
    if(processFrame() == 0)
      done = true;
}

#include "recworker_moc.cpp"

