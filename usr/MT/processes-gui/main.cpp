#include <MT/util.h>

#include "win.h"

using namespace std;


int main(int argc,char **argv){

  QApplication app(argc, argv);
  
  Gui gui;
  gui.show();
  

  gui.exec();

  return 0;
}
