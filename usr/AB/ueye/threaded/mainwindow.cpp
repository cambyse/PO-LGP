#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "opencv2/opencv.hpp"
//#include "vaiocamera.h"
//#include "logitechcamera.h"
//#include "philipscamera.h"
#include "ximeacamera.h"
#include<unistd.h> // only for sleep

using namespace cv;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow) {

  ui->setupUi(this);
  connect(&_displayTimer, SIGNAL(timeout()), this, SLOT(updateDisplay()));

  bool recordData = false;
  useKinect = false;
  fps = 30;

  XimeaCamera::init();
  n_cameras = XimeaCamera::getNumDevices();
  
  // setup camera(s)
  cameras = new AbstractCamera*[n_cameras];
  cameraThreads = new CameraThread*[n_cameras];

  QString outputPath = "/home/bais/tmp";

  for(int i = 0; i < n_cameras; i++)
    cameras[i] = new XimeaCamera(i, 1280, 1024, fps);

  for(int c=0;c<n_cameras;c++)
    cameraThreads[c] = new CameraThread(cameras[c],recordData,outputPath);

  // setup kinect
  if (useKinect)
    kinectThread = new KinectThread(recordData,outputPath);

}

MainWindow::~MainWindow() {
  if (useKinect)
    delete kinectThread;

  for(int c=0;c<n_cameras;c++) {
    delete cameraThreads[c];
    delete cameras[c];
  }

  delete[] cameraThreads;
  delete[] cameras;

  delete ui;
}

void MainWindow::changeEvent(QEvent *e) {
    QMainWindow::changeEvent(e);
    switch (e->type()) {
    case QEvent::LanguageChange:
        ui->retranslateUi(this);
        break;
    default:
        break;
    }
}

void MainWindow::on_startButton_clicked() {
}

void MainWindow::on_stopButton_clicked() {
}

void MainWindow::on_openButton_clicked() {
  std::cout << "This doesn't do anything yet." << std::endl;
}

void MainWindow::on_closeButton_clicked() {
  std::cout << "This doesn't do anything yet." << std::endl;
}

void MainWindow::updateDisplay()
{
  // cameras

  QLabel* camLabels[2];
  camLabels[0] = ui->cam0Label;
  camLabels[1] = ui->cam1Label;

  for(int c=0;c<n_cameras;c++) {

    QImage img = cameraThreads[c]->getImage();

    // rescale to label and display
    camLabels[c]->setPixmap(QPixmap::fromImage(img.rgbSwapped().scaled(camLabels[c]->width(),camLabels[c]->height(), Qt::KeepAspectRatio)));

  }

  // kinect

  if (useKinect) {

    QImage imgRGB, imgDepth;
    kinectThread->getImages(imgRGB,imgDepth);

    // rescale to label and display
    ui->kinectRGBLabel->setPixmap(QPixmap::fromImage(imgRGB.rgbSwapped().scaled(ui->kinectRGBLabel->width(),ui->kinectRGBLabel->height(), Qt::KeepAspectRatio)));
    ui->kinectDepthLabel->setPixmap(QPixmap::fromImage(imgDepth.rgbSwapped().scaled(ui->kinectDepthLabel->width(),ui->kinectDepthLabel->height(), Qt::KeepAspectRatio)));

  }

}
