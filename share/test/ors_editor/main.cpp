#include <MT/ors.h>
#include <MT/opengl.h>

const char *USAGE=
"\n\
Usage:  ors_edit <ors-filename>\n\
\n\
Iterate between editing the file (with an external editor) and\n\
viewing the model in the OpenGL window (after pressing ENTER).\n\
\n\
Use the number keys 1 2 3 4 5 to toggle display options.\n\
\n\
enum ShapeType { noneST=-1, boxST=0, sphereST, cappedCylinderST, meshST, cylinderST, markerST, pointCloudST }; \n\
enum JointType { JT_hinge=0, JT_transX, JT_universal, JT_fixed, ballJT, JT_glue };\n\
enum BodyType  { noneBT=-1, dynamicBT=0, kinematicBT, staticBT };\n\
";

void drawBase(void*){
  glStandardLight(NULL);
  glDrawFloor(10,.8,.8,.8);
  glColor(1.,.5,0.);
}

int main(int argn,char **argv){
  cout <<USAGE <<endl;

  MT::String file=MT::getParameter<MT::String>("file",STRING("test.ors"));
  if(argn==2) file=argv[1];
  cout <<"opening file `" <<file <<"'" <<endl;

  ors::Graph C;
  OpenGL gl;
  init(C, gl,file );
  gl.add(drawBase,0);
  gl.add(ors::glDrawGraph,&C);
  //gl.reportEvents=true;
  //gl.reportSelects=true;

  editConfiguration(file,C,gl);

  return 0;
}
