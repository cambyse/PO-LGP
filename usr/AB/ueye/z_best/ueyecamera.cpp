#include <iostream>
#include <cstdlib>
#include "ueyecamera.h"

using namespace std;

UEyeCamera::UEyeCamera(int cid, int w, int h, int f):
                camID(cid), width(w), height(h), fps(f) {

  // TODO remove. Gets any available camera.
  camID = 0;

  vw = NULL;

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

MT::String UEyeCamera::getName() {
  return name;
}

void UEyeCamera::init() {
// TODO check out how to initialize a specific camera
  cout << "UEyeCamera()::InitCamera()" << endl;
  camStatus = is_InitCamera(&camID, NULL);
  query_status(camID, "InitCamera", &camStatus);
  name.clear() << "video_" << camID;
  cout << " - camID = " << camID << endl;
  cout << " - name = " << name << endl;

  /*
  camStatus = is_EnableAutoExit(camID, IS_ENABLE_AUTO_EXIT);
  query_status(camID, "EnableAutoExit", &camStatus);
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
  cout << " - pixelclock range = " << pr[0] << ":" << pr[2] << ":" << pr[1] << endl;

  pixelclock = pr[1];
  camStatus = is_PixelClock(camID, IS_PIXELCLOCK_CMD_SET,
                            (void*)&pixelclock, sizeof(pixelclock));
  query_status(camID, "PixelClock", &camStatus);
  cout << " - set/real pixelclock = " << pixelclock << flush;

  camStatus = is_PixelClock(camID, IS_PIXELCLOCK_CMD_GET,
                            (void*)&pixelclock, sizeof(pixelclock));
  query_status(camID, "PixelClock", &camStatus);
  cout << "/" << pixelclock << endl;

  camStatus = is_SetFrameRate(camID, fps, &real_fps);
  query_status(camID, "SetFrameRate", &camStatus);
  cout << " - set/real fps = " << fps << "/" << real_fps << endl;

  double er[3];
  camStatus = is_Exposure(camID, IS_EXPOSURE_CMD_GET_EXPOSURE_RANGE,
                          (void*)er, sizeof(er));
  query_status(camID, "Exposure", &camStatus);
  cout << " - exposure range = " << er[0] << ":" << er[2] << ":" << er[1] << endl;

  exposure = er[1];
  camStatus = is_Exposure(camID, IS_EXPOSURE_CMD_SET_EXPOSURE,
                          (void*)&exposure, sizeof(exposure));
  query_status(camID, "Exposure", &camStatus);
  cout << " - set/real exposure = " << exposure << flush;

  camStatus = is_Exposure(camID, IS_EXPOSURE_CMD_GET_EXPOSURE,
                          (void*)&exposure, sizeof(exposure));
  query_status(camID, "Exposure", &camStatus);
  cout << "/" << exposure << endl;
}

void UEyeCamera::open() {
  cout << "UEyeCamera(" << camID << ")::CaptureVideo" << endl;
  camStatus = is_CaptureVideo(camID, IS_DONT_WAIT);
  query_status(camID, "CaptureVideo", &camStatus);

  //camStatus = is_CaptureVideo(camID, IS_DONT_WAIT);
  //camStatus = is_CaptureVideo(camID, IS_WAIT);
  //camStatus = is_CaptureVideo(camID, IS_GET_LIVE);
  //camStatus = is_CaptureVideo(camID, 300);

  camStatus = is_EnableEvent(camID, IS_SET_EVENT_FRAME);
  query_status(camID, "EnableEvent", &camStatus);
}

void UEyeCamera::close() {
  if(vw != NULL)
    stopRec();

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

void UEyeCamera::startRec(MT::String fname) {
  rec_mutex.lock();
  if(vw == NULL) {
    fname << name << ".avi";
    vw = new VideoWriter_x264((const char*)fname, width, height, fps, 20, "superfast");
  }
  rec_mutex.unlock();
}

void UEyeCamera::stopRec() {
  rec_mutex.lock();
  delete vw;
  vw = NULL;
  rec_mutex.unlock();
  cout << "cam(" << camID << ")::stopRec()" << endl;
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

  rec_mutex.lock();
  if(vw != NULL)
    vw->addFrame((uint8_t*)image_copy);
  rec_mutex.unlock();
  
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
  for(;;) {
    camStatus = is_WaitEvent(camID, IS_SET_EVENT_FRAME, INFINITE);
    query_status(camID, "WaitEvent", &camStatus);

    grab();

    //analyse_status();

    if(quit_flag)
      break;
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

void UEyeCamera::analyse_status() {
  camStatus = is_CaptureStatus(camID, IS_CAPTURE_STATUS_INFO_CMD_GET,
                        (void*)&ueyeStatus, sizeof(ueyeStatus));
  query_status(camID, "CaptureStatus", &camStatus);

  if(ueyeStatus.dwCapStatusCnt_Total > 0) {
    cout << ueyeStatus.dwCapStatusCnt_Total << " errors:" << endl;
    if(ueyeStatus.adwCapStatusCnt_Detail[IS_CAP_STATUS_API_NO_DEST_MEM] > 0)
      cout << "IS_CAP_STATUS_API_NO_DEST_MEM: " << ueyeStatus.adwCapStatusCnt_Detail[IS_CAP_STATUS_API_NO_DEST_MEM] << endl;
    if(ueyeStatus.adwCapStatusCnt_Detail[IS_CAP_STATUS_API_CONVERSION_FAILED] > 0)
      cout << "IS_CAP_STATUS_API_CONVERSION_FAILED: " << ueyeStatus.adwCapStatusCnt_Detail[IS_CAP_STATUS_API_CONVERSION_FAILED] << endl;
    if(ueyeStatus.adwCapStatusCnt_Detail[IS_CAP_STATUS_API_IMAGE_LOCKED] > 0)
      cout << "IS_CAP_STATUS_API_IMAGE_LOCKED: " << ueyeStatus.adwCapStatusCnt_Detail[IS_CAP_STATUS_API_IMAGE_LOCKED] << endl;
    if(ueyeStatus.adwCapStatusCnt_Detail[IS_CAP_STATUS_DRV_OUT_OF_BUFFERS] > 0)
      cout << "IS_CAP_STATUS_DRV_OUT_OF_BUFFERS: " << ueyeStatus.adwCapStatusCnt_Detail[IS_CAP_STATUS_DRV_OUT_OF_BUFFERS] << endl;
    if(ueyeStatus.adwCapStatusCnt_Detail[IS_CAP_STATUS_DRV_DEVICE_NOT_READY] > 0)
      cout << "IS_CAP_STATUS_DRV_DEVICE_NOT_READY: " << ueyeStatus.adwCapStatusCnt_Detail[IS_CAP_STATUS_DRV_DEVICE_NOT_READY] << endl;
    if(ueyeStatus.adwCapStatusCnt_Detail[IS_CAP_STATUS_USB_TRANSFER_FAILED] > 0)
      cout << "IS_CAP_STATUS_USB_TRANSFER_FAILED: " << ueyeStatus.adwCapStatusCnt_Detail[IS_CAP_STATUS_USB_TRANSFER_FAILED] << endl;
    if(ueyeStatus.adwCapStatusCnt_Detail[IS_CAP_STATUS_DEV_TIMEOUT] > 0)
      cout << "IS_CAP_STATUS_DEV_TIMEOUT: " << ueyeStatus.adwCapStatusCnt_Detail[IS_CAP_STATUS_DEV_TIMEOUT] << endl;
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
  query_status(-1, "GetNumberOfCameras", &status);
  return (int)numCams;
}

#include "ueyecamera_moc.cpp"
