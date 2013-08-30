#include <iostream>
#include <cstdlib>
#include "ueyecamera.h"

using namespace std;

UEyeCamera::UEyeCamera(int cid, int w, int h, int f):
                camID(cid), width(w), height(h), fps(f) {

  // TODO remove. Gets any available camera.
  camID = 0;

  name = QString("video_");
  image = NULL;
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

QString UEyeCamera::getName() {
  return name;
}

void UEyeCamera::init() {
// TODO check out how to initialize a specific camera
  cout << "UEyeCamera()::InitCamera()" << endl;
  camStatus = is_InitCamera(&camID, NULL);
  query_status(camID, "InitCamera", &camStatus);
  cout << " - camID = " << camID << endl;

  camStatus = is_SetColorMode(camID, IS_CM_BGR8_PACKED);
  query_status(camID, "SetColorMode", &camStatus);

  camStatus = is_SetColorConverter(camID,
                                    IS_CM_BGR8_PACKED,
                                    IS_CONV_MODE_SOFTWARE_5X5);
  query_status(camID, "SetColorConverter", &camStatus);

  // should be default anyway.. plus it gives me an error
  //camStatus = is_SetDisplayMode(camID, IS_SET_DM_DIB);
  //query_status(camID, "SetDisplayMode", &camStatus);

  // probably not necessary, but better make sure..
  camStatus = is_SetExternalTrigger(camID, IS_SET_TRIGGER_OFF);
  //camStatus = is_SetExternalTrigger(camID, IS_SET_TRIGGER_SOFTWARE);
  query_status(camID, "SetExternalTrigger", &camStatus);

  //camStatus = is_GetSensorInfo(camID, &camInfo);
  //query_status(camID, "GetSensorInfo", &camStatus);
  //cout << " - sensor ID = " << camInfo.SensorID << endl;
  //cout << " - camera model = " << camInfo.strSensorName << endl;
  //cout << " - max width = " << camInfo.nMaxWidth << endl;
  //cout << " - max height = " << camInfo.nMaxHeight << endl;
  //cout << " - pixel size = " << (float)camInfo.wPixelSize/100 << " µm" << endl;

  bpp = 24;
  //cout << "width = " << width << endl;
  //cout << "height = " << height << endl;
  //cout << "bpp = " << bpp << endl;

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

  camStatus = is_PixelClock(camID, IS_PIXELCLOCK_CMD_GET,
                            (void*)&old_pixelclock, sizeof(old_pixelclock));
  query_status(camID, "PixelClock", &camStatus);
  cout << " - old_pixelclock = " << old_pixelclock << endl;

  UINT nRange[3];
  memset(nRange, 0, 3*sizeof(UINT));
  camStatus = is_PixelClock(camID, IS_PIXELCLOCK_CMD_GET_RANGE,
                            (void*)nRange, sizeof(nRange));
  query_status(camID, "PixelClock", &camStatus);
  cout << " - min_pixelclock = " << nRange[0] << endl;
  cout << " - max_pixelclock = " << nRange[1] << endl;
  cout << " - step_pixelclock = " << nRange[2] << endl;

  // TODO how to determine this from the fps?
  pixelclock = 86; // nRange[1];
  camStatus = is_PixelClock(camID, IS_PIXELCLOCK_CMD_SET,
                            (void*)&pixelclock, sizeof(pixelclock));
  query_status(camID, "PixelClock", &camStatus);
  cout << " - pixelclock = " << pixelclock << endl;

  // TODO check out how to fix fps issue. I want 60fps.
  camStatus = is_SetFrameRate(camID, fps, &real_fps);
  query_status(camID, "SetFrameRate", &camStatus);
  cout << " - real_fps = " << real_fps << endl;

  camStatus = is_Exposure(camID, IS_EXPOSURE_CMD_SET_EXPOSURE,
                          (void*)&exposure, sizeof(exposure));
  query_status(camID, "Exposure", &camStatus);
  cout << " - exposure = " << exposure << endl;
}

void UEyeCamera::open() {
  cout << "UEyeCamera(" << camID << ")::CaptureVideo" << endl;
  //camStatus = is_CaptureVideo(camID, IS_GET_LIVE);
  //camStatus = is_CaptureVideo(camID, IS_DONT_WAIT);
  camStatus = is_CaptureVideo(camID, IS_WAIT);
  query_status(camID, "CaptureVideo", &camStatus);

  camStatus = is_EnableEvent(camID, IS_SET_EVENT_FRAME);
  query_status(camID, "EnableEvent", &camStatus);
}

void UEyeCamera::close() {
  camStatus = is_DisableEvent(camID, IS_SET_EVENT_FRAME);
  query_status(camID, "DisableEvent", &camStatus);

  camStatus = is_StopLiveVideo(camID, IS_FORCE_VIDEO_STOP);
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

  camStatus = is_ExitCamera(camID);
  query_status(camID, "ExitCamera", &camStatus);
}

void UEyeCamera::grab() {
  imageBuffNum = 0; image = NULL;
  camStatus = is_GetActSeqBuf(camID, NULL, NULL, &image);
  query_status(camID, "GetActSeqBuf", &camStatus);

  //camStatus = is_LockSeqBuf(camID, imageBuffNum, image);
  camStatus = is_LockSeqBuf(camID, IS_IGNORE_PARAMETER, image);
  query_status(camID, "LockSeqBuf", &camStatus);

  mutex.lock();
  memcpy(image_copy, image, 3*width*height);
  mutex.unlock();
  
  //camStatus = is_UnlockSeqBuf(camID, imageBuffNum, image);
  camStatus = is_UnlockSeqBuf(camID, IS_IGNORE_PARAMETER, image);
  query_status(camID, "UnlockSeqBuf", &camStatus);

  /*
  //camStatus = is_GetFramesPerSecond(camID, &fps);
  //query_status(camID, "GetFramesPerSecond", &camStatus);
  */
}

void UEyeCamera::getImage(char *p) {
  mutex.lock();
  memcpy(p, image_copy, 3*width*height);
  mutex.unlock();
}

void UEyeCamera::process() {
  cout << "UEyeCamera(" << camID << ")::started() emitted" << endl;
  emit started();
  
  quit_flag = false;
  while(!quit_flag) {
    camStatus = is_WaitEvent(camID, IS_SET_EVENT_FRAME, INFINITE);
    query_status(camID, "WaitEvent", &camStatus);

    grab();
  }
  close();

  cout << "UEyeCamera(" << camID << ")::finished() emitted" << endl;
  emit finished();
}

void UEyeCamera::quit() {
  cout << "UEyeCamera(" << camID << ")::quit()" << endl;
  quit_flag = true;
}

bool UEyeCamera::query_status(HIDS camID, const char *method, INT *status) {
  if(*status != IS_SUCCESS) {
    IS_CHAR* msg;
    INT ret = is_GetError(camID, status, &msg);
    if(ret != IS_SUCCESS)
      // OH THE IRONY indicates is_GetError itself has incurred into an error
      msg = (IS_CHAR*)"OH THE IRONY..";
    cout << method << " failed with status " << *status
          << " (" << msg << ")" << endl;
    exit(0);
    return true;
  }
  return false;
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
  query_status(-1, "GetNumberOfCameras", &status);
  return (int)numCams;
}

#include "ueyecamera_moc.cpp"

