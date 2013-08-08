#include <iostream>
#include <cstdlib>
#include "ueyecamera.h"

using namespace std;

UEyeCamera::UEyeCamera(int cid, int w, int h, int f):
                camID(cid+1), width(w), height(h), fps(f) {

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
  cout << "InitCamera" << endl;
  camStatus = is_InitCamera(&camID, NULL);
  query_status(camID, "InitCamera", &camStatus);

  camStatus = is_SetColorMode(camID, IS_CM_BGR8_PACKED);
  query_status(camID, "SetColorMode", &camStatus);

  camStatus = is_SetColorConverter(camID,
                                    IS_CM_BGR8_PACKED,
                                    IS_CONV_MODE_SOFTWARE_5X5);
  query_status(camID, "SetColorConverter", &camStatus);

  // should be default anyway.. plus it gives me an error
  /*
  camStatus = is_SetDisplayMode(camID, IS_SET_DM_DIB);
  query_status(camID, "SetDisplayMode", &camStatus);
  */

  // probably not necessary, but better make sure..
  /*
  camStatus = is_SetExternalTrigger(camID, IS_SET_TRIGGER_OFF);
  query_status(camID, "SetExternalTrigger", &camStatus);
  */

  camStatus = is_GetSensorInfo(camID, &camInfo);
  query_status(camID, "GetSensorInfo", &camStatus);
  cout << " - sensor ID = " << camInfo.SensorID << endl;
  cout << " - camera model = " << camInfo.strSensorName << endl;
  cout << " - max width = " << camInfo.nMaxWidth << endl;
  cout << " - max height = " << camInfo.nMaxHeight << endl;
  cout << " - pixel size = " << (float)camInfo.wPixelSize/100 << " µm" << endl;

  bpp = 24;
  cout << "width = " << width << endl;
  cout << "height = " << height << endl;
  cout << "bpp = " << bpp << endl;

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

  camStatus = is_ClearSequence(camID);
  query_status(camID, "ClearSequence", &camStatus);
  for(int i = 0; i < numBuff; i++) {
    camStatus = is_AddToSequence(camID, camBuff[i], camBuffID[i]);
    query_status(camID, "AddToSequence", &camStatus);
  }
  // SEPARATION
  cout << "Setting Parameters" << endl;

  camStatus = is_PixelClock(camID, IS_PIXELCLOCK_CMD_GET,
                            (void*)&old_pixelclock, sizeof(old_pixelclock));
  query_status(camID, "PixelClock", &camStatus);
  cout << " - old_pixelclock = " << old_pixelclock << endl;

  // TODO how to determine this from the fps?
  pixelclock = 35;
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
  cout << "CaptureVideo" << endl;
  //camStatus = is_CaptureVideo(camID, IS_GET_LIVE);
  camStatus = is_CaptureVideo(camID, IS_DONT_WAIT);
  query_status(camID, "CaptureVideo", &camStatus);
}

void UEyeCamera::close() {
  cout << "StopLiveVideo" << endl;
  camStatus = is_StopLiveVideo(camID, IS_FORCE_VIDEO_STOP);
  query_status(camID, "StopLiveVideo", &camStatus);

  cout << "ClearSequence" << endl;
  camStatus = is_ClearSequence(camID);
  query_status(camID, "ClearSequence", &camStatus);

  for(int i = 0; i < numBuff; i++) {
    camStatus = is_FreeImageMem(camID, camBuff[i], camBuffID[i]);
    query_status(camID, "FreeImageMem", &camStatus);
  }
  free(camBuff);
  free(camBuffID);

  camStatus = is_ClearSequence(camID);
  query_status(camID, "ClearSequence", &camStatus);

  cout << "ExitCamera" << endl;
  camStatus = is_ExitCamera(camID);
  query_status(camID, "ExitCamera", &camStatus);
}

void UEyeCamera::grab() {
  mutex.lock();
  imageBuffNum = 0;
  image = NULL;
  camStatus = is_GetActSeqBuf(camID, &imageBuffNum, NULL, &image);
  query_status(camID, "GetActSeqBuf", &camStatus);
  mutex.unlock();

  /*
  //camStatus = is_LockSeqBuf(camID, imageBuffNum, image);
  camStatus = is_LockSeqBuf(camID, IS_IGNORE_PARAMETER, image);
  query_status(camID, "LockSeqBuf", &camStatus);

  //camStatus = is_GetFramesPerSecond(camID, &fps);
  //query_status(camID, "GetFramesPerSecond", &camStatus);

  camStatus = is_UnlockSeqBuf(camID, imageBuffNum, image);
  query_status(camID, "UnlockSeqBuf", &camStatus);
  */
}

void UEyeCamera::getImage(char *p) {
  mutex.lock();
  if(image != NULL)
    memcpy(p, image, 3*width*height);
  mutex.unlock();
}

void UEyeCamera::process() {
  init();
  open();
  
  quit_flag = false;
  while(!quit_flag) {
    grab();
  }
  close();

  cout << "UEyeCamera::finished() being sent" << endl;
  emit finished();
  cout << "UEyeCamera::finished() sent" << endl;
}

void UEyeCamera::quit() {
  cout << "UEyeCamera::quit()" << endl;
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
  INT numCams, status;
  status = is_GetNumberOfCameras(&numCams);
  query_status(-1, "GetNumberOfCameras", &status);
  return (int)numCams;
}

#include "ueyecamera_moc.cpp"

