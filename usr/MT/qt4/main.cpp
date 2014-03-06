#include <iostream>
#include <unistd.h>

#include <QApplication>
#include <QMainWindow>
#include <QPushButton>
#include <pthread.h>
#include <QThread>

#include "win.h"

void* StartQAppThread(void*) {
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
#elif 0
  pid_t pid = fork();
  if(pid<0){ perror("fork failed"); return -1; }
  if(pid){ //parent process
    StartQAppThread(NULL);
    return 1;
  }
  //child process
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
