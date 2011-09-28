

#include <MT/util.h>

#include "win.h"

using namespace std;

//void glDraw(void*){ glStandardLight(); glMaterialColor(1.,0.,0.); glDrawBox(1.,1.,1.); }

int main(int argc,char **argv){

  QApplication app(argc, argv);
  
  Gui gui;
  gui.show();

  gui.exec();

  return 0;
}
