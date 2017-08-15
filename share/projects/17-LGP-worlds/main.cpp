#include <Kin/kin.h>
#include <Gui/opengl.h>
#include <Core/graph.h>

int MAIN(int argc,char **argv){
  mlr::initCmdLine(argc, argv);

  OpenGL gl;

  uint N=5;
  mlr::Array<mlr::KinematicWorld> K(N);
  for(uint i=0;i<N;i++){
    K(i).init(STRING("problem-0"<<i+1<<".g"));
    gl.addView(i, glStandardScene, NULL);
    gl.addSubView(i, K(i));
    gl.views(i).camera.setDefault();
    gl.views(i).camera.focus(1., 0., .7);

  }
  gl.setSubViewTiles(3,2);

  gl.watch();

  return 0;
}
