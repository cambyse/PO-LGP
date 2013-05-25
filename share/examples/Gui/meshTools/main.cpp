//

#include <stdlib.h>

#include <Gui/geo.h>
#include <Gui/opengl.h>

#include "swift_decomposer.cpp"
#include <Gui/color.h>

const char *USAGE=
"\n\
Usage:  ors_meshTools file.[tri|obj|off|ply|stl] <tags...>\n\
\n\
Tags can be -view, -box, -fuse, -clean, -center, -scale, -swift, -save, -qhull, -flip, -decomp\n";


void drawInit(void*){
  glStandardLight(NULL);
  glDrawAxes(1.);
  glColor(1.,.5,0.);
}

int main(int argn, char** argv){

  if(argn<2){
    cout <<USAGE <<endl;
    return 1;
  }

  MT::initCmdLine(argn,argv);
  MT::String file(argv[1]);

  OpenGL *gl=NULL;

  ors::Mesh mesh;
  mesh.readFile(file);

  cout <<"#vertices = " <<mesh.V.d0 <<" #triangles=" <<mesh.T.d0 <<endl;

  file(file.N-4)=0; //replace . by 0

  //modify
  if(MT::checkCmdLineTag("view")){
    cout <<"viewing..." <<endl;
    if(!gl) gl=new OpenGL;
    gl->clear();
    gl->add(drawInit);
    gl->add(ors::glDrawMesh,&mesh);
    gl->watch();
  }
  if(MT::checkCmdLineTag("box")){
    cout <<"box" <<endl;
    mesh.box();
  }
  if(MT::checkCmdLineTag("scale")){
    double s;
    MT::getParameter(s,"scale");
    cout <<"scale " <<s <<endl;
    mesh.scale(s);
  }
  if(MT::checkCmdLineTag("qhull")){
    cout <<"qhull..." <<endl;
    mesh.deleteUnusedVertices();
#ifdef MT_QHULL
    getTriangulatedHull(mesh.T,mesh.V);
#else
    MT_MSG("can'd use qhull - compiled without MT_QHULL flag");
#endif
  }
  if(MT::checkCmdLineTag("fuse")){
    double f;
    MT::getParameter(f,"fuse");
    cout <<"fuse " <<f <<endl;
    mesh.fuseNearVertices(f);
  }
  if(MT::checkCmdLineTag("clean")){
    cout <<"clean" <<endl;
    mesh.clean();
  }
  if(MT::checkCmdLineTag("flip")){
    cout <<"clean" <<endl;
    //mesh.fuseNearVertices(1e-2);
    mesh.flipFaces();
  }
  if(MT::checkCmdLineTag("center")){
    cout <<"center" <<endl;
    mesh.center();
  }
  if(MT::checkCmdLineTag("swift")){
    mesh.writeTriFile(STRING(file<<"_x.tri"));
    MT::String cmd;
    cmd <<"decomposer_c-vs6d.exe -df " <<file <<"_x.dcp -hf " <<file <<"_x.chr " <<file <<"_x.tri";
    cout <<"swift: " <<cmd <<endl;
    if(system(cmd)) MT_MSG("system call failed");
  }
  if(MT::checkCmdLineTag("decomp")){
    cout <<"decomposing..." <<endl;
    intA triangleAssignments;
    MT::Array<MT::Array<uint> > shapes;
    decompose(mesh, STRING(file<<"_x.dcp"), triangleAssignments, shapes);
    mesh.C.resize(mesh.T.d0,3);
    for(uint t=0;t<mesh.T.d0;t++){
      MT::Color col;
      col.setIndex(triangleAssignments(t));
      mesh.C(t,0) = col.r;  mesh.C(t,1) = col.g;  mesh.C(t,2) = col.b;
    }
  }
  if(MT::checkCmdLineTag("view")){
    cout <<"viewing..." <<endl;
    if(!gl) gl=new OpenGL;
    gl->clear();
    gl->add(drawInit);
    gl->add(ors::glDrawMesh,&mesh);
    gl->watch();
  }
  if(MT::checkCmdLineTag("save")){
    cout <<"saving..." <<endl;
    cout << "\tto " << file <<"_x.tri" << endl;
    mesh.writeTriFile(STRING(file<<"_x.tri"));
    cout << "\tto " << file <<"_x.off" << endl;
    mesh.writeOffFile(STRING(file<<"_x.off"));
    cout << "\tto " << file <<"_x.ply" << endl;
    mesh.writePLY(STRING(file<<"_x.ply"), true);
  }

  cout <<"#vertices = " <<mesh.V.d0 <<" #triangles=" <<mesh.T.d0 <<endl;

  return 1;
}
