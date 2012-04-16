#ifndef MT_gui_h
#define MT_gui_h

#include "win_ui.h"
#include <QtGui/QFileDialog>
#include <QtGui/QMessageBox>
#include <iostream>

void addVariablesToTree(QTreeWidget *tree);

class Gui:public QDialog{
Q_OBJECT
public:
  Ui_win ui;

  Gui(){
    ui.setupUi(this);
    show();
  }
  
  void add(){
    addVariablesToTree(ui.VariableTree);
  }

public slots:
};

#endif
