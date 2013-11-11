#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <Core/registry.h>

const char *USAGE=
"\n\
Usage:  ors_edit <ors-filename>\n\
\n\
Iterate between editing the file (with an external editor) and\n\
viewing the model in the OpenGL window (after pressing ENTER).\n\
\n\
Use the number keys 1 2 3 4 5 to toggle display options.\n\
";

void drawBase(void*){
  glStandardLight(NULL);
  glDrawFloor(10,.8,.8,.8);
  glColor(1.,.5,0.);
}

void TEST(OrsEditor) {
  cout <<USAGE <<endl;

  MT::String file=MT::getParameter<MT::String>("file",STRING("test.ors"));
  if(argc==2) file=argv[1];
  cout <<"opening file `" <<file <<"'" <<endl;

  ors::Graph G;
  OpenGL gl;
  init(G, gl, file);

  //some optional manipulations
  G.meldFixedJoints();
  G.removeUselessBodies();
    G.calcBodyFramesFromJoints();
//  G.topSort();
//  G.makeLinkTree();
  MT::save(G,"z.ors");

  editConfiguration(file, G, gl);
}

int MAIN(int argc,char **argv){
  MT::initCmdLine(argc, argv);

  testOrsEditor();

  return 0;
}
