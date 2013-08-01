#pragma once

#include <QMainWindow>
#include <QTimer>
#include "camerathread.h"
#include "kinectthread.h"
#include "abstractcamera.h"

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow {

  Q_OBJECT

public:
  MainWindow(QWidget *parent = 0);
  ~MainWindow();

protected:
  void changeEvent(QEvent *e);

private slots:
  void on_startButton_clicked();
  void on_stopButton_clicked();
  void on_openButton_clicked();
  void on_closeButton_clicked();
  void updateDisplay();

private:
  Ui::MainWindow *ui;
  QTimer _displayTimer;

  AbstractCamera** cameras;
  CameraThread** cameraThreads;

  KinectThread* kinectThread;

  int fps;
  int n_cameras;
  bool useKinect;

};
