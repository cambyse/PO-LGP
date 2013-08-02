/*
#include<iostream>
#include<unistd.h>
#include<cstring>
#include<string>
#include<sstream>
#include<wchar.h>
#include<cstdlib>

#include<ueye.h>


//TODO remove useless includes
extern "C" {
#include<libavcodec/avcodec.h>
#include<libavformat/avformat.h>
#include<libswscale/swscale.h>
}
*/

#include "recorder.h"

int main(int argc, char *argv[]) {
  Recorder r(1280, 1024, 60, false);
  r.record();
}

