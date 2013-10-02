#include <iostream>
#include <QCoreApplication>
#include <cstdlib>
#include "ueyecamera.h"

using namespace std;

UEyeCamera::UEyeCamera(int cid, int w, int h, int f):
                camID(HIDS(cid)), width(w), height(h), fps(f) {
  // TODO remove. Gets any available camera.
  camID = 0;

  vw = NULL;

  image = NULL;

  nrecframes = 0;
}

int UEyeCamera::getWidth() {
  return width;
}

int UEyeCamera::getHeight() {
  return height;
}

int UEyeCamera::getFPS() {
  return fps;
}

MT::String UEyeCamera::getName() {
  return name;
}

void UEyeCamera::camInit() {
  MT::String m;
  msg("InitCamera()");
  camStatus = is_InitCamera(&camID, NULL);
  query_status(camID, "InitCamera", &camStatus);
  name.clear() << "video_" << camID;
  m.clear() << "-- camID " << camID;
  msg(m);
  m.clear() << "-- name " << name;
  msg(m);
  /*
  msg("-- camID ", camID);
  msg("-- name ", name);
  */

  camStatus = is_SetColorMode(camID, IS_CM_BGR8_PACKED);
  query_status(camID, "SetColorMode", &camStatus);

  camStatus = is_SetColorConverter(camID,
                                    IS_CM_BGR8_PACKED,
                                    IS_CONV_MODE_SOFTWARE_5X5);
  query_status(camID, "SetColorConverter", &camStatus);

  // should be default anyway.. plus it gives me an error
  //camStatus = is_SetDisplayMode(camID, IS_SET_DM_DIB);
  //query_status(camID, "SetDisplayMode", &camStatus);

  camStatus = is_SetExternalTrigger(camID, IS_SET_TRIGGER_OFF);
  query_status(camID, "SetExternalTrigger", &camStatus);

  //camStatus = is_GetSensorInfo(camID, &camInfo);
  //query_status(camID, "GetSensorInfo", &camStatus);
  //cout << " - sensor ID = " << camInfo.SensorID << endl;
  //cout << " - camera model = " << camInfo.strSensorName << endl;
  //cout << " - max width = " << camInfo.nMaxWidth << endl;
  //cout << " - max height = " << camInfo.nMaxHeight << endl;
  //cout << " - pixel size = " << (float)camInfo.wPixelSize/100 << " µm" << endl;

  bpp = 24;

  numBuff = 10;
  camBuff = (char**)malloc(numBuff*sizeof(char*));
  camBuffID = (INT*)malloc(numBuff*sizeof(INT));
  for(int i = 0; i < numBuff; i++) {
    camStatus = is_AllocImageMem(camID,
                                  width,
                                  height,
                                  bpp,
                                  &camBuff[i],
                                  &camBuffID[i]);
    query_status(camID, "AllocImageMem", &camStatus);
  }
  image_copy = (char*)malloc(3*width*height*sizeof(char));

  camStatus = is_ClearSequence(camID);
  query_status(camID, "ClearSequence", &camStatus);
  for(int i = 0; i < numBuff; i++) {
    camStatus = is_AddToSequence(camID, camBuff[i], camBuffID[i]);
    query_status(camID, "AddToSequence", &camStatus);
  }

  // SEPARATION

  UINT pr[3];
  memset(pr, 0, 3*sizeof(UINT));
  camStatus = is_PixelClock(camID, IS_PIXELCLOCK_CMD_GET_RANGE,
                            (void*)pr, sizeof(pr));
  query_status(camID, "PixelClock", &camStatus);
  m.clear() << " - pixelclock range = " << pr[0] << ":" << pr[2] << ":" << pr[1];
  msg(m);

  pixelclock = pr[1];
  camStatus = is_PixelClock(camID, IS_PIXELCLOCK_CMD_SET,
                            (void*)&pixelclock, sizeof(pixelclock));
  query_status(camID, "PixelClock", &camStatus);
  m.clear() << " - set pixelclock = " << pixelclock;
  msg(m);

  camStatus = is_PixelClock(camID, IS_PIXELCLOCK_CMD_GET,
                            (void*)&pixelclock, sizeof(pixelclock));
  query_status(camID, "PixelClock", &camStatus);
  m.clear() << " - real pixelclock = " << pixelclock;
  msg(m);

  camStatus = is_SetFrameRate(camID, fps, &real_fps);
  query_status(camID, "SetFrameRate", &camStatus);
  m.clear() << " - set fps = " << fps;
  msg(m);
  m.clear() << " - real fps = " << real_fps;
  msg(m);

  double er[3];
  camStatus = is_Exposure(camID, IS_EXPOSURE_CMD_GET_EXPOSURE_RANGE,
                          (void*)er, sizeof(er));
  query_status(camID, "Exposure", &camStatus);
  m.clear() << " - exposure range = " << er[0] << ":" << er[2] << ":" << er[1];

  exposure = er[1];
  camStatus = is_Exposure(camID, IS_EXPOSURE_CMD_SET_EXPOSURE,
                          (void*)&exposure, sizeof(exposure));
  query_status(camID, "Exposure", &camStatus);
  m.clear() << " - set exposure = " << exposure;

  camStatus = is_Exposure(camID, IS_EXPOSURE_CMD_GET_EXPOSURE,
                          (void*)&exposure, sizeof(exposure));
  query_status(camID, "Exposure", &camStatus);
  m.clear() << " - real exposure = " << exposure;
}

void UEyeCamera::open() {
  msg("CaptureVideo()");
  camStatus = is_CaptureVideo(camID, IS_DONT_WAIT);
  query_status(camID, "CaptureVideo", &camStatus);

  camStatus = is_EnableEvent(camID, IS_SET_EVENT_FRAME);
  query_status(camID, "EnableEvent", &camStatus);
}

void UEyeCamera::close() {
  if(vw != NULL)
    stopRec();

  camStatus = is_DisableEvent(camID, IS_SET_EVENT_FRAME);
  query_status(camID, "DisableEvent", &camStatus);

  camStatus = is_StopLiveVideo(camID, IS_WAIT);
        //FORCE_VIDEO_STOP);
  query_status(camID, "StopLiveVideo", &camStatus);
  
  camStatus = is_ClearSequence(camID);
  query_status(camID, "ClearSequence", &camStatus);

  for(int i = 0; i < numBuff; i++) {
    camStatus = is_FreeImageMem(camID, camBuff[i], camBuffID[i]);
    query_status(camID, "FreeImageMem", &camStatus);
  }
  free(camBuff);
  free(camBuffID);
  free(image_copy);

  // should be executed in the main thread... but this is the only way to make
  // it work..
  camExit();
}

void UEyeCamera::camExit() {
  ExitCamera_wrapper();
}

void UEyeCamera::waitUntilExit() {
  while(CaptureVideo_wrapper(IS_GET_LIVE) == TRUE) {
    msg("Still on");
    usleep(1000000);
  }
  msg("Finally Exited");
}

void UEyeCamera::startRec(MT::String fname) {
  recMutex.lock();
  if(vw == NULL) {
    fname << name << ".avi";
    vw = new VideoWriter_x264((const char*)fname, width, height, fps, 20, "superfast");
  }
  nrecframes = 0;
  recMutex.unlock();
}

void UEyeCamera::stopRec() {
  recMutex.lock();
  delete vw;
  vw = NULL;
  MT::String m;
  m << "num recorded frames: " << nrecframes;
  msg(m);
  recMutex.unlock();
}

void UEyeCamera::grab() {
  imageBuffNum = 0; image = NULL;
  camStatus = is_GetActSeqBuf(camID, NULL, NULL, &image);
  query_status(camID, "GetActSeqBuf", &camStatus);

  //camStatus = is_LockSeqBuf(camID, imageBuffNum, image);
  camStatus = is_LockSeqBuf(camID, IS_IGNORE_PARAMETER, image);
  query_status(camID, "LockSeqBuf", &camStatus);

  imgMutex.lock();
  memcpy(image_copy, image, 3*width*height);
  imgMutex.unlock();
  
  MT::String m;
  m.clear() << "imageID = " << getImageID(image);
  msg(m);

  recMutex.lock();
  if(vw != NULL)
    vw->addFrame((uint8_t*)image_copy);
  nrecframes++;
  recMutex.unlock();
  
  camStatus = is_UnlockSeqBuf(camID, IS_IGNORE_PARAMETER, image);
  query_status(camID, "UnlockSeqBuf", &camStatus);

  /*
  double fps;
  camStatus = is_GetFramesPerSecond(camID, &fps);
  query_status(camID, "GetFramesPerSecond", &camStatus);
  MT::String m;
  m.clear() << "fps = " << fps;
  msg(m);
  */
}

void UEyeCamera::getImage(char *p) {
  imgMutex.lock();
  memcpy(p, image_copy, 3*width*height);
  imgMutex.unlock();
}

void UEyeCamera::process() {
  emit started();
  
  bool quit;

  quitMutex.lock();
  quit_flag = false;
  quitMutex.unlock();
  for(;;) {
    camStatus = is_WaitEvent(camID, IS_SET_EVENT_FRAME, INFINITE);
    query_status(camID, "WaitEvent", &camStatus);

    grab();

    quitMutex.lock();
    quit = quit_flag;
    quitMutex.unlock();

    if(quit)
      break;
  }
  close();

  emit finished();
}

void UEyeCamera::quit() {
  msg("quit()");
  quitMutex.lock();
  quit_flag = true;
  quitMutex.unlock();
}

bool UEyeCamera::query_status(HIDS camID, const char *method, INT *status) {
  if(*status != IS_SUCCESS) {
    IS_CHAR* mess;
    INT ret = is_GetError(camID, status, &mess);
    if(ret != IS_SUCCESS)
      // OH THE IRONY indicates is_GetError itself has incurred into an error // not really...
      mess = (IS_CHAR*)"OH THE IRONY..";
    cout << method << " failed with status " << *status << " (" << mess << ")";
    QCoreApplication::quit();
    return true;
  }
  return false;
}

INT UEyeCamera::CaptureVideo_wrapper(INT wait) {
  camStatus = is_CaptureVideo(camID, wait);
  if(camStatus == TRUE)
    return TRUE;
  if(camStatus != IS_SUCCESS) {
    msg("CaptureVideo() failed");
    if(camStatus == IS_NO_SUCCESS)
      err();
    else
      msg(NULL);
    QCoreApplication::quit();
  }
  return IS_SUCCESS;
}

void UEyeCamera::ExitCamera_wrapper() {
  camStatus = is_ExitCamera(camID);
  if(camStatus != IS_SUCCESS) {
    msg("ExitCamera() failed");
    if(camStatus == IS_NO_SUCCESS)
      err();
    else if(camStatus == IS_CANT_OPEN_REGISTRY)
      msg("IS_CANT_OPEN_REGISTRY");
    else if(camStatus == IS_CANT_READ_REGISTRY)
      msg("IS_CANT_READ_REGISTRY");
    else if(camStatus == IS_ERROR_CPU_IDLE_STATES_CONFIGURATION)
      msg("IS_ERROR_CPU_IDLE_STATES_CONFIGURATION");
    else if(camStatus == IS_INVALID_CAMERA_HANDLE)
      msg("IS_INVALID_CAMERA_HANDLE");
    else if(camStatus == IS_INVALID_PARAMETER)
      msg("IS_INVALID_PARAMETER");
    else if(camStatus == IS_IO_REQUEST_FAILED)
      msg("IS_IO_REQUEST_FAILED");
    else if(camStatus == IS_NO_IMAGE_MEM_ALLOCATED)
      msg("IS_NO_IMAGE_MEM_ALLOCATED");
    else
      msg(NULL);
    QCoreApplication::quit();
  }
}

void UEyeCamera::err() {
  IS_CHAR *mess;
  camStatus = is_GetError(camID, &camStatus, &mess);
  if(camStatus == IS_SUCCESS)
    msg((const char *)mess);
  else {
    msg("GetError() failed");
    if(camStatus == IS_NO_SUCCESS)
      err();
    else if(camStatus == IS_INVALID_CAMERA_HANDLE)
      msg("IS_INVALID_CAMERA_HANDLE");
    else if(camStatus == IS_INVALID_PARAMETER)
      msg("IS_INVALID_PARAMETER");
  }
}

QMutex UEyeCamera::msgMutex;

void UEyeCamera::msg(const char *m) {
  if(m == NULL)
    msg("no message");
  else {
    UEyeCamera::msgMutex.lock();
    cout << "UEyeCamera(" << camID << ") - " << m << endl;
    UEyeCamera::msgMutex.unlock();
  }
}

void UEyeCamera::msg(const MT::String &m) {
  if(m.N == 0)
    msg("no message");
  else {
    UEyeCamera::msgMutex.lock();
    cout << "UEyeCamera(" << camID << ") - " << m << endl;
    UEyeCamera::msgMutex.unlock();
  }
}

INT UEyeCamera::getImageID(char *buff) {
  for(int i = 0; i < numBuff; i++)
    if(camBuff[i] == buff)
      return camBuffID[i];
  return -1;
}

int UEyeCamera::getNumCameras() {
  INT numCams = -1;
  INT status = is_GetNumberOfCameras(&numCams);
  if(status != IS_SUCCESS) {
    cout << "UEyeCamera::getNumCameras() An error of some sort" << endl;
    numCams = 0;
  }
  return (int)numCams;
}

#include "ueyecamera_moc.cpp"
