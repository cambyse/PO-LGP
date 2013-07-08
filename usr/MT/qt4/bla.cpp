#include <MT/util.h>

#include "win.h"
#include <MT/qt.h>

using namespace std;

// void glDraw(void*){
//   glStandardLight(NULL);
//   glColor(1.,0.,0.);
//   glFrontFace(GL_CW);
//   glutSolidTeapot(1.);
//   glFrontFace(GL_CCW);
// }

int main(int argc,char **argv){

  qtCheckInitialized();
  
  Gui gui;
  gui.show();
  
  //gui.ui.glview->gl->add(glDraw,NULL);

  gui.exec();
  //MT::wait(10.);

  return 0;
}
