#include <cstring>
#include <iostream>
#include <fstream>
#include "videowriter.h"

using namespace std;

int main(int argc, char *argv[]) {
  ifstream *fin;
  VideoWriter_x264 *vw;
  char ifname[101], ofname[100];

  int w, h, fps;
  char *p;
  w = 1280;
  h = 1024;
  fps = 30;
  for(int i = 1; i < argc; i++) {
    strcpy(ifname, argv[i]);
    strcpy(ofname, argv[i]);
    p = strstr(ofname, "_raw.avi");
    if(p == NULL)
      break;
    strcpy(p, ".avi");
    cout << "Encoding file: " << ifname << endl;
    cout << "    into file: " << ofname << endl;

    // TODO READ FIRST X BYTES WITH INFO
    // like width, height, numframes

    fin = new ifstream((char*)ifname);
    vw = new VideoWriter_x264((const char *)ofname, w, h, fps, 20, "superfast");
    p = new char[3*w*h];

    for(int fnum = 1; !fin->eof(); fnum++) {
      fin->read(p, 3*w*h);
      vw->addFrame((uint8_t*)p);
      cout << "\r encoding frame " << fnum << flush;
    }
    cout << endl;

    delete vw;
    fin->close();
    delete fin;
    delete p;

    cout << endl;
  }

  cout << "Done!" << endl;
}

