#include <stdlib.h>
#include <GL/gl.h>

#include <Geo/mesh.h>
#include <Gui/opengl.h>

#include <Optim/optimization.h>


//===========================================================================

void TEST(SSBoxFit){
  ors::Mesh org, box;

  struct Transparent:GLDrawer{
    void glDraw(OpenGL &){ glColor(1,.5,.0,.5); }
  } trans;

  OpenGL gl;
  gl.add(glStandardScene);
  gl.add(org);
  gl.add(trans);
  gl.add(box);

  for(uint k=0;k<30;k++){
    org.V.resize(100,3);
    rndUniform(org.V, 2., 3.);
    org.setRandom();
    org.translate(0,0,1);
    org.makeConvexHull();
    box.makeSSBox(NoArr, NoTransformation, org.V, 10, 3);

    gl.watch();
  }


}

//===========================================================================

int MAIN(int argc, char** argv){
  testSSBoxFit();
  return 0;
}
