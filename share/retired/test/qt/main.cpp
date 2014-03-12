#include <Core/util.h>

#include "win.h"

using namespace std;

#if 0
void glDraw(void*){
#ifdef MT_GL
  glStandardLight(NULL);
  glColor(1.,0.,0.);
  glFrontFace(GL_CW);
  glutSolidTeapot(1.);
  glFrontFace(GL_CCW);
#endif
}
#endif

int main(int argc,char **argv){

  QApplication app(argc, argv);
  
  Gui gui;
  gui.show();
  
//  gui.ui.glview->gl->add(glDraw,NULL);

  gui.exec();

  return 0;
}
