#include<iostream>
#include<unistd.h>
#include<cstring>
#include<string>
#include<sstream>
#include<wchar.h>
#include<cstdlib>

#include<ueye.h>

#include<Core/array.h>
#include<Gui/opengl.h>
#include<GL/glut.h>

#include"VideoWriter_x264.h"

extern "C" {
#include<libavcodec/avcodec.h>
#include<libavformat/avformat.h>
#include<libswscale/swscale.h>
}

using namespace std;

struct Recorder: public OpenGL::GLKeyCall {
  static const int MAX_CAMS_PER_ROW = 2;

  // for visualization
  OpenGL gl;
  byteA **img;

  HIDS camID;
  INT camStatus;
  SENSORINFO camInfo;
  char *image;

  INT numCams;
  int numBuff;
  char **camBuff;
  INT *camBuffID;

  INT w;    // width
  INT h;    // height
  INT bpp;  // bits per pixel

  UINT old_pixelclock;
  UINT pixelclock;
  double fps;
  double real_fps;
  double exposure;

  IMAGE_FILE_PARAMS fParams;
  wchar_t fname[255];

  VideoWriter_x264 *vw;

  bool quit, play, rec;

  Recorder() {
    gl.addKeyCall(this);
  }

  ~Recorder() {
    delete vw;
  }

  void record() {
    init();
    open();
    params();
    capture();
    close();
  }

  void init() {
    camID = 1;
    numBuff = 5;

    w = 1280;
    h = 1024;
    bpp = 24;

    pixelclock = 35;
    fps = 60;
    exposure = 40;

    fParams.nFileType = IS_IMG_BMP;
    fParams.pwchFileName = NULL;
    fParams.ppcImageMem = NULL;
    fParams.pnImageID = NULL;
    fParams.nQuality = 0;

    camStatus = is_GetNumberOfCameras(&numCams);
    query_status(-1, "GetNumberOfCameras", &camStatus);
    cout << " - num cameras = " << numCams << endl;

    // TODO is_GetCameraList
    
    quit = false;
    play = true;
    rec = false;

    initGL();

    vw = new VideoWriter_x264("video.avi", w, h, fps, 20, "superfast");
  }

  void initGL() {
    img = new byteA*[numCams];
    for(int c = 0; c < numCams; c++)
      img[c] = new byteA(h, w, 3);

    // TODO only for test purposes
    numCams++;

    int nr = (numCams-1) / MAX_CAMS_PER_ROW + 1;
    int nc = MAX_CAMS_PER_ROW;
    float r, c;
    for(int cam = 0; cam < numCams; cam++) {
      gl.addView(cam, &nothing, 0);
      r = cam / nc;
      c = cam % nc;
      gl.setViewPort(cam, c/nc, (c+1)/nc, r/nr, (r+1)/nr);
      // TODO only for test purposes
      //gl.views(cam).img = img[0];
      gl.views(cam).img = img[cam];
    }

    // TODO only for test purposes
    numCams--;
  }

  void open() {
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
                                    w,
                                    h,
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
  }

  void close() {
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

  void params() {
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

  void capture() {
    cout << "CaptureVideo" << endl;
    camStatus = is_CaptureVideo(camID, IS_DONT_WAIT);
    query_status(camID, "CaptureVideo", &camStatus);

    wstringstream wss;
    while(!quit) {
      camStatus = is_GetActSeqBuf(camID, NULL, NULL, &image);
      query_status(camID, "GetActSeqBuf", &camStatus);

      if(rec)
        vw->addFrame((uint8_t*)image);

      if(play) {
        camStatus = is_GetFramesPerSecond(camID, &fps);
        query_status(camID, "GetFramesPerSecond", &camStatus);

        for(int cam = 0; cam < numCams; cam++) {
          img[cam]->p = (byte*)image;
          gl.views(cam).text = STRING("frame: 1" << "\nfps: " << fps);
        }
        gl.update(); // all time spent here.. problem: usb2
      }
    }

    cout << "StopLiveVideo" << endl;
    camStatus = is_StopLiveVideo(camID, IS_FORCE_VIDEO_STOP);
    query_status(camID, "StopLiveVideo", &camStatus);

    cout << "ClearSequence" << endl;
    camStatus = is_ClearSequence(camID);
    query_status(camID, "ClearSequence", &camStatus);
  }

  void query_status(HIDS camID, const char *method, INT *status) {
    if(*status != IS_SUCCESS) {
      IS_CHAR* msg;
      INT ret = is_GetError(camID, status, &msg);
      if(ret != IS_SUCCESS)
        // OH THE IRONY indicates is_GetError itself has incurred into an error
        msg = (IS_CHAR*)"OH THE IRONY..";
      cout << method << " failed with status " << *status
            << " (" << msg << ")" << endl;
    }
  }

  INT getImageID(char *buff) {
    for(int i = 0; i < numBuff; i++)
      if(camBuff[i] == buff)
        return camBuffID[i];
    return -1;
  }

  static void nothing(void*) {
  }

  bool keyCallback(OpenGL &) {
    switch(gl.pressedkey) {
      case 'p':
        play = !play;
        break;
      case 'q':
        quit = true;
        break;
      case 'r':
        rec = !rec;
        break;
      default:
        cout << "Unknown key pressed: " << gl.pressedkey << endl;
        break;
    }
    return true;
  }
};

int main(int argc, char *argv[]) {
  Recorder r;
  r.record();
}

