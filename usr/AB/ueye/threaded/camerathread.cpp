#include "camerathread.h"
#include "VideoWriter_x264.h"
//#include <sys/time.h>
#include<time.h>

CameraThread::CameraThread(AbstractCamera *_camera,
                            bool _record,
                            const char *_path):
                                                  camera(_camera),
                                                  record(_record),
                                                  path(_path),
                                                  stopped(false) {
  // create output buffer (updated at each frame by grabber thread)
  grabberBuff = (uchar*)malloc(3*camera->getWidth()*camera->getHeight());

  // create image buffer (only updated when new image is requested by other
  // external threads)
  imageBuff = (uchar*)malloc(3*camera->getWidth()*camera->getHeight());
}

CameraThread::~CameraThread() {
  free(grabberBuff);
  free(imageBuff);
}

byteA CameraThread::getImage() {
  if(buffMutex.tryLock()) {
    memcpy(imageBuff, grabberBuff, 3*camera->getWidth()*camera->getHeight());
    buffMutex.unlock();
  }
  byteA image(camera->getHeight(), camera->getWidth(), 3);
  byteA.p = imageBuff;

  return image;
}

void CameraThread::stop() {
  stopped = true;
}

void CameraThread::run() {
  std::cout << "RUN_START()" << std::endl;
  FILE *timestamp_file;
  VideoWriter_x264 *videoWriter;

  if(record) {
    // create timestamp file
    char fileName[200];
    sprintf(fileName,
            "%s/%s_timestamps.txt", path,
            camera->getName().toStdString().c_str());
    timestamp_file = fopen(fileName,"w");

    // open video file
    sprintf(fileName,
            "%s/%s_video.mp4",
            path,
            camera->getName().toStdString().c_str());
    videoWriter = new VideoWriter_x264(fileName,
                                        camera->getWidth(),
                                        camera->getHeight(),
                                        camera->getFPS(),
                                        20,
                                        "superfast");
  }

  double frameTime;

  int frameCount = 0;

  camera->open();

  bool success_time, success;
  Mat frameMat;
  while(!stopped) {
    camera->grab();
    //frameTime = getTime();
    bool success_time = getTime(&frameTime);
    bool success_frame = camera->retrieve(frameMat);

    if(success_frame && success_time) {
      if(record) {
        // save timestamp
        fprintf(timestamp_file,"%07d %2.16f\n", frameCount, frameTime);
        // encode
        videoWriter->addFrame(frameMat.data);
      }

      buffMutex.lock();
      memcpy(grabberBuff,
              frameMat.data,
              3*camera->getWidth()*camera->getHeight());
      buffMutex.unlock();

      frameCount++;
    }
//    msleep(5);
  }

  camera->close();

  if(record) {
    delete videoWriter;
    fclose(timestamp_file);
  }

  // prepare for next run
  stopped = false;

  std::cout << "RUN_FINISHED()." << std::endl;
}

bool CameraThread::getTime(double *time) {
  /*
  timeval timeOfDay;
  gettimeofday(&timeOfDay, NULL);
  return(timeOfDay.tv_sec+(timeOfDay.tv_usec/1000000.0));
  */
 timespec timeOfDay;
 int res = clock_gettime(CLOCK_MONOTONIC_RAW, &timeOfDay);
 *time = timeOfDay.tv_sec+(timeOfDay.tv_nsec/1000000000.0);

 return res == 0;
}
