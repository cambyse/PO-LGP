#include <iostream>
#include <unistd.h>

#include <QApplication>
#include <QMainWindow>
#include <QPushButton>
#include <pthread.h>
#include <QThread>

#include "win.h"

void *StartQAppThread(void *) {
  int argc=1;
  char **argv = new char*[1];
  argv[0] = (char*)"x.exe";
  QApplication app(argc, argv);
  QMainWindow w;
  w.show();
  w.setCentralWidget(new QPushButton("NewButton"));
  app.exec();
  pthread_exit(NULL);
}

struct QtThread:QThread{
  virtual void run(){
    int argc=1;
    char **argv = new char*[1];
    argv[0] = (char*)"x.exe";
    QApplication app(argc, argv);
    QMainWindow w;
    w.show();
    w.setCentralWidget(new QPushButton("NewButton"));
    exec();
  }
};


int main(int argc, char *argv[]) {
#if 1
  pthread_t thread1;
  pthread_create(&thread1, NULL, StartQAppThread, NULL);
#else
  QtThread bla;
  bla.start();
#endif

  sleep(2);

  Gui gui;
  //gui.moveToThread(&bla);
  gui.show();

  sleep(10);
  return 0;
}
