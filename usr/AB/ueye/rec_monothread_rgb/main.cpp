#include <QCoreApplication>
#include "recorder.h"

int main(int argc, char *argv[]) {
  QCoreApplication *app = new QCoreApplication(argc, argv);

  Recorder rec(1280, 1024, 60);
  rec.setup();

  return app->exec();
}

