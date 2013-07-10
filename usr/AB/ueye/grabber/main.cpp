#include<iostream>
#include<unistd.h>
#include<cstring>
#include<string>
#include<sstream>
#include<wchar.h>

#include<ueye.h>

using namespace std;

void init();
void open();
void capture(int n);
void close();
void query_status(HIDS camID, INT *status);

HIDS camID;
char *camBuff;
INT camBuffID;
INT camStatus;
//camExposure = 5000;

INT width;
INT height;
INT bitspixel;

IMAGE_FILE_PARAMS fParams;
wchar_t fname[255];

int main(int argc, char *argv[]) {
  init();
  open();
  capture(10);
  close();
}

void init() {
  camID = 1;

  width = 1280;
  height = 1024;
  bitspixel = 24;

  fParams.nFileType = IS_IMG_BMP;
  fParams.pwchFileName = NULL;
  fParams.ppcImageMem = NULL;
  fParams.pnImageID = NULL;
  fParams.nQuality = 0;
}

void open() {
  cout << "InitCamera" << endl;
  camStatus = is_InitCamera(&camID, NULL);
  query_status(camID, &camStatus);

  cout << "AllocImageMem" << endl;
  camStatus = is_AllocImageMem(camID,
                                width,
                                height,
                                bitspixel,
                                &camBuff,
                                &camBuffID);
  query_status(camID, &camStatus);

  cout << "SetImageMem" << endl;
  camStatus = is_SetImageMem(camID, camBuff, camBuffID);
  query_status(camID, &camStatus);
}

void close() {
  cout << "FreeImageMem" << endl;
  camStatus = is_FreeImageMem(camID, camBuff, camBuffID);
  query_status(camID, &camStatus);

  cout << "ExitCamera" << endl;
  camStatus = is_ExitCamera(camID);
  query_status(camID, &camStatus);
}

void capture(int n) {
  if(n>0) {
    wstringstream wss;
    for(int i = 0; i < n; i++) {
      wss.str(wstring());
      wss << "file_" << i << flush;
      //memset(fname, 0, sizeof(fname));
      wcscpy(fname, wss.str().c_str());
      fParams.pwchFileName = fname;

      wcout << "FreezeVideo (" << wss.str() << ")" << endl;
      camStatus = is_FreezeVideo(camID, IS_WAIT);
      query_status(camID, &camStatus);

      wcout << "ImageFile (" << wss.str() << ")" << endl;
      camStatus = is_ImageFile(camID,
                                IS_IMAGE_FILE_CMD_SAVE,
                                (void*)&fParams,
                                sizeof(fParams));
      query_status(camID, &camStatus);
    }
  }
  else {
      wcout << "CaptureVideo" << endl;
      camStatus = is_CaptureVideo(camID, IS_WAIT);
      query_status(camID, &camStatus);
  }
}

void query_status(HIDS camID, INT *status) {
  if(*status != IS_SUCCESS) {
    IS_CHAR* msg;
    INT ret = is_GetError(camID, status, &msg);
    if(ret != IS_SUCCESS)
      // OH NOES indicates is_GetError has incurred into an error itself
      msg = (IS_CHAR*)"OH NOES";
    cout << " -> return value = " << *status << " (" << msg << ")" << endl;
  }
}

