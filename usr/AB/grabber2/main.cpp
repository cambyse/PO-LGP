#include<iostream>
#include<xiApi.h>
#include<xiExt.h>

#define HandleResult(res,place) if (res!=XI_OK) {printf("Error after %s (%d)",place,res);goto finish;}

using namespace std;

void print_status(XI_RETURN &s);

int main(int argc, char *argv[]) {
// Sample for XIMEA Software Package V2.57

  HANDLE xiH = NULL;
  XI_RETURN stat = XI_OK;

  // image buffer
  XI_IMG image;
  image.size = sizeof(XI_IMG);
  image.bp = NULL;
  image.bp_size = 0;

  // Retrieving a handle to the camera device 
  stat = xiOpenDevice(0, &xiH);
  print_status(stat);

  // Setting "exposure" parameter (10ms)
  int time_us = 10000;
  stat = xiSetParam(xiH, XI_PRM_EXPOSURE, &time_us, sizeof(time_us), xiTypeInteger);
  print_status(stat);

  // Start acquisition
  stat = xiStartAcquisition(xiH);
  print_status(stat);

  // getting image from camera
  stat = xiGetImage(xiH, 1000, &image);
  print_status(stat);

  cout << "OK - Got one image (" << image.width << "x" << image.height << ") from camera" << endl;

  finish:
  // Close device
  if (xiH) xiCloseDevice(xiH);
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

