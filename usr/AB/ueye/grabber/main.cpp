#include<iostream>
#include<unistd.h>
#include<cstring>
#include<string>
#include<sstream>
#include<wchar.h>
#include<cstdlib>

#include<opencv2/opencv.hpp>
#include<ueye.h>

//#include<MT/vision.h>
//#include<Core/util.h>

using namespace std;

void init();
void pre();
void open();
void params();
void capture(int n, bool screen = false);
void close();
void query_status(HIDS camID, const char *method, INT *status);
INT getImageID(char *buff);

HIDS camID;
INT camStatus;
SENSORINFO camInfo;
char *image;

INT numCams;
int numBuff;
char **camBuff;
INT *camBuffID;

INT width;
INT height;
INT bitspixel;

UINT old_pixelclock;
UINT pixelclock;
double fps;
double real_fps;
double exposure;

IMAGE_FILE_PARAMS fParams;
wchar_t fname[255];

int main(int argc, char *argv[]) {
  init();
  pre();
  open();
  params();
  //capture(10, true);
  capture(10, false);
  close();
}

void init() {
  camID = 1;
  numBuff = 5;

  width = 1280;
  height = 1024;
  bitspixel = 24;

  pixelclock = 35;
  fps = 60;
  exposure = 40;

  fParams.nFileType = IS_IMG_BMP;
  fParams.pwchFileName = NULL;
  fParams.ppcImageMem = NULL;
  fParams.pnImageID = NULL;
  fParams.nQuality = 0;
}

void pre() {
  camStatus = is_GetNumberOfCameras(&numCams);
  query_status(-1, "GetNumberOfCameras", &camStatus);
  cout << " - num cameras = " << numCams << endl;

  // TODO is_GetCameraList
}

void open() {
  cout << "InitCamera" << endl;
  camStatus = is_InitCamera(&camID, NULL);
  query_status(camID, "InitCamera", &camStatus);

  camStatus = is_SetColorMode(camID, IS_CM_BGR8_PACKED);
  query_status(camID, "SetColorMode", &camStatus);

  camStatus = is_SetColorConverter(camID, IS_CM_BGR8_PACKED, IS_CONV_MODE_SOFTWARE_5X5);
  query_status(camID, "SetColorConverter", &camStatus);

  //should be default anyway.. plus it gives me an error
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
  cout << " - pixel size = " << (float)camInfo.wPixelSize/100 << " micro meters" << endl;

  camBuff = (char**)malloc(numBuff*sizeof(char*));
  camBuffID = (int*)malloc(numBuff*sizeof(int));
  for(int i = 0; i < numBuff; i++) {
    camStatus = is_AllocImageMem(camID,
                                  width,
                                  height,
                                  bitspixel,
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

  /*
  camStatus = is_SetImageMem(camID, camBuff[0], camBuffID[0]);
  query_status(camID, "SetImageMem", &camStatus);
  */

  cout << "EnableEvent" << endl;
  camStatus = is_EnableEvent(camID, IS_SET_EVENT_FRAME);
  query_status(camID, "EnableEvent", &camStatus);
}

void close() {
  for(int i = 0; i < numBuff; i++) {
    camStatus = is_FreeImageMem(camID, camBuff[i], camBuffID[i]);
    query_status(camID, "FreeImageMem", &camStatus);
  }
  free(camBuff);
  free(camBuffID);

  /*
  camStatus = is_FreeImageMem(camID, camBuff, camBuffID);
  query_status(camID, "FreeImageMem", &camStatus);
  */

  cout << "DisableEvent" << endl;
  camStatus = is_DisableEvent(camID, IS_SET_EVENT_FRAME);
  query_status(camID, "DisableEvent", &camStatus);

  cout << "ExitCamera" << endl;
  camStatus = is_ExitCamera(camID);
  query_status(camID, "ExitCamera", &camStatus);
}

void params() {
  cout << "Setting Parameters" << endl;
  

  camStatus = is_PixelClock(camID, IS_PIXELCLOCK_CMD_GET, (void*)&old_pixelclock, sizeof(old_pixelclock));
  query_status(camID, "PixelClock", &camStatus);
  cout << " - old_pixelclock = " << old_pixelclock << endl;

  camStatus = is_PixelClock(camID, IS_PIXELCLOCK_CMD_SET, (void*)&pixelclock, sizeof(pixelclock));
  query_status(camID, "PixelClock", &camStatus);
  cout << " - pixelclock = " << pixelclock << endl;

  // TODO check out how to fix fps issue. I want 60fps.
  camStatus = is_SetFrameRate(camID, fps, &real_fps);
  query_status(camID, "SetFrameRate", &camStatus);
  cout << " - real_fps = " << real_fps << endl;

  camStatus = is_Exposure(camID, IS_EXPOSURE_CMD_SET_EXPOSURE, (void*)&exposure, sizeof(exposure));
  query_status(camID, "Exposure", &camStatus);
  cout << " - exposure = " << exposure << endl;
}

void capture(int n, bool screen) {
  if(screen) {
    wstringstream wss;
    for(int i = 0; i < n; i++) {
      wss.str(wstring());
      wss << "file_" << i << flush;
      //memset(fname, 0, sizeof(fname));
      wcscpy(fname, wss.str().c_str());
      fParams.pwchFileName = fname;

      wcout << "FreezeVideo (" << wss.str() << ")" << endl;
      camStatus = is_FreezeVideo(camID, IS_WAIT);
      query_status(camID, "FreezeVideo", &camStatus);

      cout << "WaitEvent" << endl;
      camStatus = is_WaitEvent(camID, IS_SET_EVENT_FRAME, 500);//INFINITE);
      query_status(camID, "WaitEvent", &camStatus);

      cout << "GetImageMem" << endl;
      camStatus = is_GetImageMem(camID, (VOID**)&image);
      query_status(camID, "GetImageMem", &camStatus);
      cout << " - buff = " << &image << endl;

      cv::Mat img(width, height, CV_8SC3, image);
      cv::namedWindow("Display Image", CV_WINDOW_AUTOSIZE);
      cv::imshow("Display Image", img);

      cv::Size t = img.size();
      cout << "img.channels() = " << img.channels() << endl;
      cout << "img.size() = " << t.width << "x" << t.height << endl;

      cv::waitKey(0);
      /*
      wcout << "ImageFile (" << wss.str() << ")" << endl;
      camStatus = is_ImageFile(camID,
                                IS_IMAGE_FILE_CMD_SAVE,
                                (void*)&fParams,
                                sizeof(fParams));
      query_status(camID, "ImageFile", &camStatus);
      */
    }
  }
  else {
      cout << "CaptureVideo" << endl;
      camStatus = is_CaptureVideo(camID, IS_DONT_WAIT);
      query_status(camID, "CaptureVideo", &camStatus);

      wstringstream wss;
      for(int i = 0; i < n; i++) {
          /*
          cout << "WaitEvent" << endl;
          camStatus = is_WaitEvent(camID, IS_SET_EVENT_FRAME, 500);//INFINITE);
          query_status(camID, "WaitEvent", &camStatus);
          */
          /*
          cout << "GetImageMem" << endl;
          camStatus = is_GetImageMem(camID, (VOID**)&image);
          query_status(camID, "GetImageMem", &camStatus);
          cout << " - buff = " << &image << endl;
          */

          cout << "GetActSeqBuf" << endl;
          camStatus = is_GetActSeqBuf(camID, NULL, NULL, &image);
          query_status(camID, "GetActSeqBuf", &camStatus);

          cout << " - buffID = " << getImageID(image) << endl;

          camStatus = is_GetFramesPerSecond(camID, &fps);
          query_status(camID, "GetFramesPerSecond", &camStatus);
          cout << " - fps = " << fps << endl;

          cv::Mat img(width, height, CV_8SC3, image);
          cv::namedWindow("Display Image", CV_WINDOW_AUTOSIZE);
          cv::imshow("Display Image", img);

          cv::waitKey(0);

          wss.str(wstring());
          wss << "file_" << i << flush;
          //memset(fname, 0, sizeof(fname));
          wcscpy(fname, wss.str().c_str());

          fParams.pwchFileName = fname;
          wcout << "ImageFile (" << wss.str() << ")" << endl;
          camStatus = is_ImageFile(camID,
                                    IS_IMAGE_FILE_CMD_SAVE,
                                    (void*)&fParams,
                                    sizeof(fParams));
          query_status(camID, "ImageFile", &camStatus);
      }

      cout << "StopLiveVideo" << endl;
      camStatus = is_StopLiveVideo(camID, IS_FORCE_VIDEO_STOP);
      query_status(camID, "StopLiveVideo", &camStatus);

      cout << "ClearSequence" << endl;
      camStatus = is_ClearSequence(camID);
      query_status(camID, "ClearSequence", &camStatus);
  }
}

void query_status(HIDS camID, const char *method, INT *status) {
  if(*status != IS_SUCCESS) {
    IS_CHAR* msg;
    INT ret = is_GetError(camID, status, &msg);
    if(ret != IS_SUCCESS)
      // OH THE IRONY indicates is_GetError itself has incurred into an error
      msg = (IS_CHAR*)"OH THE IRONY..";
    cout << method << " failed with status " << *status << " (" << msg << ")" << endl;
  }
}

INT getImageID(char *buff) {
  for(int i = 0; i < numBuff; i++)
    if(camBuff[i] == buff)
      return camBuffID[i];
  return -1;
}
