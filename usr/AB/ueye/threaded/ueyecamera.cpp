#include<iostream>
#include"ueyecamera.h"

using namespace std;

UEyeCamera::UEyeCamera(int _camID, int _w, int _h, int _fps):
                        camID(_camID),
                        width(_w), height(_h), fps(_fps) {
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

  camBuff = (char**)malloc(numBuff*sizeof(char*));
  camBuffID = (int*)malloc(numBuff*sizeof(int));
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

  setParams();
}

UEyeCamera::~UEyeCamera() { }

void UEyeCamera::setParams() {
  cout << "Setting Parameters" << endl;
  

  camStatus = is_PixelClock(camID, IS_PIXELCLOCK_CMD_GET,
                            (void*)&old_pixelclock, sizeof(old_pixelclock));
  query_status(camID, "PixelClock", &camStatus);
  cout << " - old_pixelclock = " << old_pixelclock << endl;

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
  camStatus = is_GetActSeqBuf(camID, &imageBuffNum, NULL, &image);
  query_status(camID, "GetActSeqBuf", &camStatus);

  camStatus = is_LockSeqBuf(camID, imageBuffNum, image);
  query_status(camID, "LockSeqBuf", &camStatus);

  /*
  camStatus = is_GetFramesPerSecond(camID, &fps);
  query_status(camID, "GetFramesPerSecond", &camStatus);
  */
}

void UEyeCamera::retrieve(byte *img) {
  img = (byte*)image;
// TODO unlock sequence buffer!!!
// probably I first have to really copy the array, not the pointer only..
// maybe already use byteA here?
 
  /*
  camStatus = is_UnlockSeqBuf(camID, imageBuffNum, image);
  query_status(camID, "UnlockSeqBuf", &camStatus);
  */
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
