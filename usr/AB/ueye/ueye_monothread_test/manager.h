#pragma once

#include <QObject>
#include "ueyecamera.h"

class Manager: public QObject {
  Q_OBJECT

  UEyeCamera *ueye;

  public:
    Manager() {};
    ~Manager() {};

    void setup(int nCams);

  public slots:
    void quitUEye();
    void exitUEye();
    void quitQCore();
};

