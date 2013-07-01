#include<iostream>
#include<xiApi.h>
#include<xiExt.h>

using namespace std;

void run_once();
void init();
void open_start();
void get_images(int n);
void stop_close();
void print_status(XI_RETURN &s);

bool debug = false;

unsigned int xiNumDevices;
int xiExposure = 5000;
HANDLE xiHandle = NULL;
XI_RETURN xiStatus;
XI_IMG xiImage;

int main(int argc, char *argv[]) {
  run_once();
}

void run_once() {
  cout << "START" << endl;

  init();
  open_start();
  get_images(10);
  stop_close();

  cout << "DONE" << endl;
}

void init() {
  xiImage.size = sizeof(XI_IMG);
  xiImage.bp = NULL;
  xiImage.bp_size = 0;

  if(debug) {
    cout << "xiSetParamInt(XI_PRM_DEBUG_LEVEL): " << endl;
    xiStatus = xiSetParamInt(0, XI_PRM_DEBUG_LEVEL, XI_DL_TRACE);
    print_status(xiStatus);
  }

  cout << "xiGetNumberDevices: " << endl;
  xiStatus = xiGetNumberDevices(&xiNumDevices);
  print_status(xiStatus);
  cout << "xiNumDevices: " << xiNumDevices << endl;

  if(xiNumDevices == 0) {
    cout << "No Devices :(" << endl;
    exit(-1);
  }
}

void open_start() {
  cout << "xiOpenDevice: " << endl;
  xiStatus = xiOpenDevice(0, &xiHandle);
  print_status(xiStatus);

  cout << "xiSetParam(Exposure): " << xiExposure << endl;
  xiStatus = xiSetParamInt(xiHandle, XI_PRM_EXPOSURE, xiExposure);
  print_status(xiStatus);

  cout << "xiStartAcquisition(): " << endl;
  xiStatus = xiStartAcquisition(xiHandle);
  print_status(xiStatus);
}

void get_images(int n) {
  for(int i = 0; i < n; i++) {
    cout << "xiGetImage() [" << i << "]" << endl;
    xiStatus = xiGetImage(xiHandle, xiExposure, &xiImage);
    print_status(xiStatus);
  }
}

void stop_close() {
  cout << "xiStopAcquisition(): " << endl;
  xiStatus = xiStopAcquisition(xiHandle);
  print_status(xiStatus);

  cout << "xiCloseDevice()" << endl;
  xiStatus = xiCloseDevice(xiHandle);
  print_status(xiStatus);
  xiHandle = NULL;
}

const char* error_codes[] = {
  "Function call succeeded",
  "Invalid handle",
  "Register read error",
  "Register write error",
  "Freeing resiurces error",
  "Freeing channel error",
  "Freeing bandwith error",
  "Read block error",
  "Write block error",
  "No image",
  "Timeout",
  "Invalid arguments supplied",
  "Not supported",
  "Attach buffers error",
  "Overlapped result",
  "Memory allocation error",
  "DLL context is NULL",
  "DLL context is non zero",
  "DLL context exists",
  "Too many devices connected",
  "Camera context error",
  "Unknown hardware",
  "Invalid TM file",
  "Invalid TM tag",
  "Incomplete TM",
  "Bus reset error",
  "Not implemented",
  "Shading too bright",
  "Shading too dark",
  "Gain is too low",
  "Invalid bad pixel list",
  "Bad pixel list realloc error",
  "Invalid pixel list",
  "Invalid Flash File System",
  "Invalid profile",
  "Invalid calibration",
  "Invalid buffer",
  "BEWARE: UNKNOWN ERROR CODE",
  "Invalid data",
  "Timing generator is busy",
  "Wrong operation open/write/read/close",
  "Acquisition already started",
  "Old version of device driver installed to the system.",
  "To get error code please call GetLastError function.",
  "Data can't be processed",
  "Acquisition has been stopped. It should be started before GetImage.",
  "Acquisition has been stoped with error.",
  "Input ICC profile missed or corrupted",
  "Output ICC profile missed or corrupted",
  "Device not ready to operate",
  "Shading too contrast",
  "Modile already initialized",
  "Application doesn't enough privileges(one or more app",
  "Installed driver not compatible with current software",
  "TM file was not loaded successfully from resources",
  "Device has been reseted, abnormal initial state",
  "No Devices Found",
  "Resource(device) or function locked by mutex",
  "BEWARE: UNKNOWN ERROR CODE",
  "BEWARE: UNKNOWN ERROR CODE",
  "BEWARE: UNKNOWN ERROR CODE",
  "BEWARE: UNKNOWN ERROR CODE",
  "BEWARE: UNKNOWN ERROR CODE",
  "BEWARE: UNKNOWN ERROR CODE",
  "BEWARE: UNKNOWN ERROR CODE",
  "BEWARE: UNKNOWN ERROR CODE",
  "BEWARE: UNKNOWN ERROR CODE",
  "BEWARE: UNKNOWN ERROR CODE",
  "BEWARE: UNKNOWN ERROR CODE",
  "BEWARE: UNKNOWN ERROR CODE",
  "BEWARE: UNKNOWN ERROR CODE",
  "BEWARE: UNKNOWN ERROR CODE",
  "BEWARE: UNKNOWN ERROR CODE",
  "BEWARE: UNKNOWN ERROR CODE",
  "BEWARE: UNKNOWN ERROR CODE",
  "BEWARE: UNKNOWN ERROR CODE",
  "BEWARE: UNKNOWN ERROR CODE",
  "BEWARE: UNKNOWN ERROR CODE",
  "BEWARE: UNKNOWN ERROR CODE",
  "BEWARE: UNKNOWN ERROR CODE",
  "BEWARE: UNKNOWN ERROR CODE",
  "BEWARE: UNKNOWN ERROR CODE",
  "BEWARE: UNKNOWN ERROR CODE",
  "BEWARE: UNKNOWN ERROR CODE",
  "BEWARE: UNKNOWN ERROR CODE",
  "BEWARE: UNKNOWN ERROR CODE",
  "BEWARE: UNKNOWN ERROR CODE",
  "BEWARE: UNKNOWN ERROR CODE",
  "BEWARE: UNKNOWN ERROR CODE",
  "BEWARE: UNKNOWN ERROR CODE",
  "BEWARE: UNKNOWN ERROR CODE",
  "BEWARE: UNKNOWN ERROR CODE",
  "BEWARE: UNKNOWN ERROR CODE",
  "BEWARE: UNKNOWN ERROR CODE",
  "BEWARE: UNKNOWN ERROR CODE",
  "BEWARE: UNKNOWN ERROR CODE",
  "BEWARE: UNKNOWN ERROR CODE",
  "BEWARE: UNKNOWN ERROR CODE",
  "BEWARE: UNKNOWN ERROR CODE",
  "BEWARE: UNKNOWN ERROR CODE",
  "Unknown parameter",
  "Wrong parameter value",
  "BEWARE: UNKNOWN ERROR CODE",
  "Wrong parameter type",
  "Wrong parameter size",
  "Input buffer too small",
  "Parameter info not supported",
  "Parameter info not supported",
  "Data format not supported",
  "Read only parameter",
  "BEWARE: UNKNOWN ERROR CODE",
  "This camera does not support currently available bandwidth"
};

void print_status(XI_RETURN &s) {
  cout << " - return value = " << s << flush;
  cout << " (" << error_codes[s] << ")" << endl;
}

