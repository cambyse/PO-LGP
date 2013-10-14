#include <iostream>
#include <unistd.h>
#include <QCoreApplication>
#include "manager.h"
#include "ueyecamera.h"

using namespace std;

int main(int argc, char *argv[]) {
  QCoreApplication app(argc, argv);
  Manager m;

  int nCams = 2;
  if(UEyeCamera::getNumCameras() < nCams) {
    cout << "No cameras connected" << endl;
    return 1;
  }
  
  m.setup(nCams);

  return app.exec();
}

