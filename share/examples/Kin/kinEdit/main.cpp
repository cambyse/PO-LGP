#include <Kin/kin.h>
#include <Gui/opengl.h>
#include <Core/graph.h>

const char *USAGE=
"\n\
Usage:  ors_edit <ors-filename>\n\
\n\
Iterate between editing the file (with an external editor) and\n\
viewing the model in the OpenGL window (after pressing ENTER).\n\
\n\
Use the number keys 1 2 3 4 5 to toggle display options.\n\
";

void TEST(OrsEditor) {
  cout <<USAGE <<endl;

  mlr::String file=mlr::getParameter<mlr::String>("file",STRING("test.ors"));
  if(mlr::argc==2 && mlr::argv[1][0]!='-') file=mlr::argv[1];
  cout <<"opening file `" <<file <<"'" <<endl;

  mlr::KinematicWorld G(file);

  //G.setAgent(99);

  G.checkConsistency();
  G >>FILE("z.ors");
  //some optional manipulations
  G.checkConsistency();
  G.meldFixedJoints();
  G.checkConsistency();
  G >>FILE("z.ors");
  G.removeUselessBodies();
  G.checkConsistency();
  G >>FILE("z.ors");
//  makeConvexHulls(G.shapes);
//  computeOptimalSSBoxes(G.shapes);
//  G >>FILE("z.ors");
//    G.watch(true);
//    return;
  G.topSort();
  G.checkConsistency();
  G.makeLinkTree();
  G.checkConsistency();
  G.calc_q_from_Q();
  G.calc_fwdPropagateFrames();
  G >>FILE("z.ors");

  if(mlr::checkParameter<bool>("cleanOnly")) return;

  editConfiguration(file, G);
}

int MAIN(int argc,char **argv){
  mlr::initCmdLine(argc, argv);

  orsDrawAlpha = 1.;
  testOrsEditor();

  return 0;
}
