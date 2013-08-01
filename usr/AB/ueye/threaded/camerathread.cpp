#include "camerathread.h"
#include "VideoWriter_x264.h"
//#include <sys/time.h>
#include<time.h>

CameraThread::CameraThread(AbstractCamera *camera, bool recordData, QString outputPath) {
  _stopped = false;

  _camera = camera;
  _recordData = recordData;
  _outputPath = outputPath;

  // create output buffer (updated at each frame by grabber thread)
  _grabberOutputBuffer = (uchar *)
                          malloc(3*_camera->getWidth()*_camera->getHeight());

  // create image buffer (only updated when new image is requested by other
  // external threads)
  _imageOutputBuffer = (uchar *)
                          malloc(3*_camera->getWidth()*_camera->getHeight());
}

CameraThread::~CameraThread() {
  free(_imageOutputBuffer);
  free(_grabberOutputBuffer);
}

QImage CameraThread::getImage() {
  if (_bufferMutex.tryLock()) {
    memcpy(_imageOutputBuffer,
            _grabberOutputBuffer,
            3*_camera->getWidth()*_camera->getHeight());
    _bufferMutex.unlock();
  }
  QImage img(_imageOutputBuffer,
              _camera->getWidth(),
              _camera->getHeight(),
              QImage::Format_RGB888);
  return img;
}

void CameraThread::stop() {
  _stopped = true;
}

void CameraThread::run() {
  std::cout << "RUN_START()" << std::endl;
  FILE * timestamp_file;
  VideoWriter_x264 *videoWriter;

  if(_recordData) {
    // create timestamp file
    char fileName[200];
    sprintf(fileName,
            "%s/%s_timestamps.txt",
            _outputPath.toStdString().c_str(),
            _camera->getName().toStdString().c_str());
    timestamp_file = fopen(fileName,"w");

    // open video file
    sprintf(fileName,
            "%s/%s_video.mp4",
            _outputPath.toStdString().c_str(),
            _camera->getName().toStdString().c_str());
    videoWriter = new VideoWriter_x264(fileName,
                                        _camera->getWidth(),
                                        _camera->getHeight(),
                                        _camera->getFPS(),
                                        20,
                                        "superfast");
  }

  double frameTime;

  int frameCount = 0;

  _camera->open();

  bool success_time, success;
  Mat frameMat;
  while(!_stopped) {
    _camera->grab();
    //frameTime = getTime();
    bool success_time = getTime(&frameTime);
    bool success_frame = _camera->retrieve(frameMat);

    if(success_frame && success_time) {
      if(_recordData) {
        // save timestamp
        fprintf(timestamp_file,"%07d %2.16f\n", frameCount, frameTime);
        // encode
        videoWriter->addFrame(frameMat.data);
      }

      _bufferMutex.lock();
      memcpy(_grabberOutputBuffer, frameMat.data, 3*_camera->getWidth()*_camera->getHeight());
      _bufferMutex.unlock();

      frameCount++;
    }
//    msleep(5);
  }

  _camera->close();

  if (_recordData) {
    delete videoWriter;
    fclose (timestamp_file);
  }

  // prepare for next run
  _stopped = false;

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
