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


int MAIN(int argc,char **argv){
  mlr::initCmdLine(argc, argv);

    cout <<USAGE <<endl;

    mlr::String file=mlr::getParameter<mlr::String>("file",STRING("test.ors"));
    if(mlr::argc==2 && mlr::argv[1][0]!='-') file=mlr::argv[1];
    cout <<"opening file `" <<file <<"'" <<endl;

    mlr::KinematicWorld G(file);


    G.checkConsistency();
    G >>FILE("z.ors");
    //some optional manipulations
    G.optimizeTree();
    G.checkConsistency();
    G >>FILE("z.ors");
  //  makeConvexHulls(G.shapes);
  //  computeOptimalSSBoxes(G.shapes);
  //  G >>FILE("z.ors");
  //  G.watch(true);
  //  return;

    if(mlr::checkParameter<bool>("cleanOnly")) return 0;

    editConfiguration(file, G);

  return 0;
}
