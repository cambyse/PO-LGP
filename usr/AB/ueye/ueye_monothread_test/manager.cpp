#include <iostream>
#include <QCoreApplication>
#include <QThread>
#include <QTimer>

#include "manager.h"

using namespace std;

void Manager::setup(int nCams) {
  QThread *thread = new QThread();
  ueye = new UEyeCamera(1280, 1024, 60);

  switch(nCams) {
    case 1:
      ueye->setup(0);
      break;
    case 2:
      ueye->setup(0, 0);
      break;
    case 3:
      ueye->setup(0, 0, 0);
      break;
    case 4:
      ueye->setup(0, 0, 0, 0);
      break;
    default:
      cout << "UNACCEPTABLE nCams VALUE" << endl;
      exit(1);
  }
  ueye->init();
  ueye->open();

  QTimer *timer = new QTimer();
  timer->setSingleShot(true);
  timer->setInterval(15000);

  connect(thread, SIGNAL(started()), ueye, SLOT(camProcess()));
  connect(timer, SIGNAL(timeout()), this, SLOT(quitUEye()));
  connect(ueye, SIGNAL(finished()), thread, SLOT(quit()));
  connect(ueye, SIGNAL(finished()), this, SLOT(exitUEye()));
  connect(ueye, SIGNAL(exited()), this, SLOT(quitQCore()));

  connect(thread, SIGNAL(finished()), timer, SLOT(deleteLater()));
  connect(thread, SIGNAL(finished()), thread, SLOT(deleteLater()));

  ueye->moveToThread(thread);
  cout << "Starting thread" << endl;
  thread->start();

  timer->start();
};

void Manager::quitUEye() {
  cout << "Quitting ueye" << endl;
  ueye->quit();
}

void Manager::exitUEye() {
  cout << "Exiting ueye" << endl;
  ueye->close();
  ueye->exit();
}

void Manager::quitQCore() {
  delete ueye;

  cout << "Quitting QCoreApplication" << endl;
  QCoreApplication::quit();
}

#include "manager_moc.cpp"

