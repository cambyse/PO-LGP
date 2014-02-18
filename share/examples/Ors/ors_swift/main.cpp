#include <Ors/ors.h>
#include <Ors/ors_swift.h>
#include <Gui/opengl.h>

void drawInit(void*){
  glStandardLight(NULL);
  glDrawAxes(1.);
  glColor(1.,.5,0.);
}

void TEST(Swift) {
  ors::KinematicWorld G("swift_test.ors");

  G.swift().setCutoff(2.);
  G.computeProxies();

  uint t;
  for(t=0;t<50;t++){
    G.bodies(0)->X.addRelativeTranslation(0,0,-.01);
    G.bodies(0)->X.addRelativeRotationDeg(10,1,0,0);
    G.calc_fwdPropagateFrames();

    G.computeProxies();

    G.reportProxies();

    G.watch(true);
  }
}


void TEST(Sphere) {
  OpenGL gl;
  ors::Mesh mesh;

  //MeshSetTetrahedron(mesh);
  //MeshSetOctahedron(mesh);
  //MeshSetDodecahedron(mesh);
  //MeshSetBox(mesh);
  mesh.setSphere();
  //MeshSetHalfSphere(mesh);
  //MeshSetCylinder(mesh,.2,1.);
  //MeshSetCappedCylinder(mesh,.2,1.);
  gl.add(drawInit,0);
  gl.add(ors::glDrawMesh,&mesh);
  gl.watch();
  
  getTriangulatedHull(mesh.T,mesh.V);

  gl.watch();
}

void TEST(Meshes) {
  OpenGL gl;
  ors::Mesh mesh;
  mesh.readStlFile("~/share/data/3dmodel/schunk-hand.stl");
  mesh.scale(.001);
  mesh.writeTriFile("~/share/data/3dmodel/schunk-hand-full.tri");
  mesh.fuseNearVertices(1e-4);
  mesh.writeTriFile("~/share/data/3dmodel/schunk-hand-e4.tri");
  mesh.fuseNearVertices(1e-3);
  mesh.writeTriFile("~/share/data/3dmodel/schunk-hand-e3.tri");
  mesh.fuseNearVertices(1e-2);
  mesh.writeTriFile("~/share/data/3dmodel/schunk-hand-e2.tri");
  gl.add(drawInit,0);
  gl.add(ors::glDrawMesh,&mesh);
  gl.watch();
}

void TEST(Meshes2) {
  ors::Mesh mesh1,mesh2;
  OpenGL gl;
  gl.add(drawInit,0);
  gl.add(ors::glDrawMesh,&mesh1);
  mesh1.readTriFile("~/share/data/3dmodel/schunk-arm-e3.tri");
  mesh2.readTriFile("~/share/data/3dmodel/schunk-hand-e3.tri");
  uint i,m=0; double my=mesh1.V(m,1);
  for(i=0;i<mesh1.V.d0;i++) if(mesh1.V(i,1)>mesh1.V(m,1)){ m=i; my=mesh1.V(m,1); }
  mesh2.translate(0,my,0);
  mesh1.addMesh(mesh2);
  //mesh1.writeTriFile("~/share/data/3dmodel/schunk-e3.tri");
  //mesh1.writeOffFile("~/share/data/3dmodel/schunk-e3.off");
  gl.watch();
}

void TEST(Meshes3) {
  ors::Mesh mesh;
  OpenGL gl;
  gl.add(drawInit,0);
  gl.add(ors::glDrawMesh,&mesh);
  //MeshSetSphere(mesh,0);
  mesh.readTriFile("~/share/data/3dmodel/schunk-hand-e4.tri");
  gl.reportSelects=true;
  gl.watch();
  cout <<gl.topSelection->name <<endl;
  uint i=gl.topSelection->name >> 4;
  mesh.skin(i);
  gl.watch();
}

int MAIN(int argc, char** argv){

  testSwift();
  testSphere();
  testMeshes3();

  return 0;
}

