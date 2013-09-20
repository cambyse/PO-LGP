#include <QCoreApplication>
#include "recorder.h"

int main(int argc, char *argv[]) {
  QCoreApplication *app = new QCoreApplication(argc, argv);

  Recorder *rec = new Recorder();
  // to change some parameters
  // rec->setSize(w, h);
  // rec->setFPS(f);
  rec->setup();

  return app->exec();
}

