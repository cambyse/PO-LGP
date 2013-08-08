#include <QCoreApplication>
#include "recorder.h"

int main(int argc, char *argv[]) {
  QCoreApplication *app = new QCoreApplication(argc, argv);

  Recorder *rec = new Recorder();
  // to change some parameters
  // rec->setSize(w, h);
  // rec->setKinect(true);
  rec->init();
  rec->play();

  return app->exec();
}

