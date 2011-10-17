#include <MT/plot.h>
#include <MT/opengl.h>
#include <MT/ors.h>

using namespace std;


/************ first test ************/

void draw1(void*){
  glStandardLight(NULL);
  glColor(1,0,0);
  glFrontFace(GL_CW);
  glutSolidTeapot(1.);
  glFrontFace(GL_CCW);
}

void testTeapot(){
  OpenGL gl;
  gl.add(draw1,0);
  gl.watch();
  cout <<"returned from watch - watch again" <<endl;
  gl.watch();
  cout <<"returned from 2nd watch - quit" <<endl;
}

/************ multiple views ************/

void testMultipleViews(){
  byteA img;
  read_ppm(img,"box.ppm",false);
  OpenGL gl;
  gl.reportEvents=true;
  gl.reportSelects=true;
  gl.add(draw1,0);
  gl.addView(0,draw1,0);
  gl.addView(1,draw1,0);
  gl.setViewPort(1,.1,.4,.1,.4);
  gl.setViewPort(0,.6,.9,.6,.9);
  gl.views(0).img=&img;
  gl.watch();
}

/************ depth test ************/


void testDepth(){
  OpenGL gl("title",300,300);
  gl.add(draw1,0);
  cout <<"normal view " <<endl;
  gl.watch();

  gl.camera.setPosition(0,0,10);
  gl.camera.setZRange(9,10);
  gl.camera.setHeightAbs(2);
  cout <<"orthogonal top view" <<endl;
  gl.watch();

  //grap the depth image from current view:
  byteA depthImage(300,300);
  glGrabDepth(depthImage);
  cout <<"max " <<(int)depthImage.max() <<" min " <<(int)depthImage.min() <<endl;
  gl.watchImage(depthImage,true,1);
}


/************ second test ************/

static void draw2(void*){
  glStandardLight(NULL);
  glDrawAxes(1.);
  glColor(1.,1.,1.);
}

void testMesh(){
  uint i,j,N=10;
  ors::Vector v;
  ors::Mesh mesh;

  //add points to the mesh
  mesh.V.resize(N*N,3);
  for(i=0;i<N;i++) for(j=0;j<N;j++){
    v(0)=(double)j-N/2; v(1)=(double)i-N/2;
    v(2)=-.15*(v(0)*v(0)+v(1)*v(1)); //paraboloid
    //v(2)=((i+j)&1); //up-down surface
    //v(2)=2.*rnd.uni(); //random surface
    mesh.V(i*N+j,0)=v(0); mesh.V(i*N+j,1)=v(1); mesh.V(i*N+j,2)=v(2); //insert v in the list mesh.V
  }

  //connect them to a grid (i.e., define the triangle between the points)
  mesh.setGrid(N,N);
  //mesh.trinormals=true; //leads to uniform triangle colors
  //mesh.gridToStrips(grid); //alternative to triangle list -- but deletion is disabled

  //calculate normals
  mesh.computeNormals();

  //test deletion of some random triangles
  for(i=0;i<mesh.T.d0;i++) if(rnd.uni()<.1){ mesh.T(i,0)=mesh.T(i,1)=mesh.T(i,2)=0; }
  mesh.deleteUnusedVertices();

  //draw
  OpenGL gl;
  gl.text="testing Mesh";
  gl.add(draw2,0);
  gl.add(ors::glDrawMesh,&mesh);
  gl.watch();
}


/************ third test ************/

void testObj(){
  ors::Mesh mesh,mesh2;

  //mesh.readObjFile("../../external/3dmodel/obj/gipshand2-273k.obj");
  mesh.readObjFile("base-male-nude.obj");
  //mesh.readObjFile("../../../3dmodel/obj/gipshand2-273k.obj");
  //mesh.readObjFile("../../../3dmodel/obj/base-male-nude.obj");
  mesh.scale(.1,.1,.1);
  mesh.computeNormals();
  mesh2.readObjFile("magnolia.obj");
  mesh2.scale(.01,.01,.01);
  mesh2.computeNormals();
  OpenGL gl;
  gl.text="testing Mesh";
  gl.add(draw2,0);
  gl.add(ors::glDrawMesh,&mesh);
  gl.watch();
  gl.clear();
  gl.add(draw2,0);
  gl.add(ors::glDrawMesh,&mesh2);
  gl.watch();
}



/************ fourth test ************/

/* menu test */

void menuCallback1(int i){
  cout <<"menu 1 callback: " <<i <<endl;
}
void menuCallback2(int i){
  cout <<"menu 2 callback: " <<i <<endl;
}
void menuCallback3(int i){
  cout <<"menu 3 callback: " <<i <<endl;
}

void testMenu(){
  OpenGL gl;
  gl.text.clr() <<"press the right moust";
  gl.add(draw1,0);

  int submenu1, submenu2;

  submenu1 = glutCreateMenu(menuCallback1);
  glutAddMenuEntry((char*)"abc", 1);
  glutAddMenuEntry((char*)"ABC", 2);
  submenu2 = glutCreateMenu(menuCallback2);
  glutAddMenuEntry((char*)"Green", 1);
  glutAddMenuEntry((char*)"Red", 2);
  glutAddMenuEntry((char*)"White", 3);
  glutCreateMenu(menuCallback3);
  glutAddMenuEntry((char*)"9 by 15", 0);
  glutAddMenuEntry((char*)"Times Roman 10", 1);
  glutAddMenuEntry((char*)"Times Roman 24", 2);
  glutAddSubMenu((char*)"Messages", submenu1);
  glutAddSubMenu((char*)"Color", submenu2);
  glutAttachMenu(GLUT_RIGHT_BUTTON);

  gl.watch();
}


/************ 5th test ************/

byteA texImg;
static GLuint texName;
void init5(void){
  read_ppm(texImg,"box.ppm");
  texName=glImageTexture(texImg);
}
void draw5(void*){
  glStandardLight(NULL);

  glDrawTexQuad(texName, -2.0, -1.0, 0.0,
                         -2.0, 1.0, 0.0,
			 0.0, 1.0, 0.0,
			 0.0, -1.0, 0.0);

  glEnable(GL_TEXTURE_2D);
  glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
  glBindTexture(GL_TEXTURE_2D, texName);
  glBegin(GL_QUADS);
  glTexCoord2f(0.0, 0.0); glVertex3f(1.0, -1.0, 0.0);
  glTexCoord2f(0.0, 1.0); glVertex3f(1.0, 1.0, 0.0);
  glTexCoord2f(1.0, 1.0); glVertex3f(2.41421, 1.0, -1.41421);
  glTexCoord2f(1.0, 0.0); glVertex3f(2.41421, -1.0, -1.41421);
  glEnd();
  glFlush();
  glDisable(GL_TEXTURE_2D);

  glColor(1,0,0);
  glutSolidTeapot(1.);

}

void testTexture(){
  OpenGL gl;
  init5();
  gl.add(draw5,0);
  gl.watch();
}

/************ test clicking on and identifying objects in the scene ************/

void draw3(void*){
  glPushName(0xa0);
  glStandardLight(NULL);
  glColor(1,0,0);
  glutSolidTeapot(1.);
  glPopName();
}
void testSelect(){
  OpenGL gl;
  gl.add(draw3,0);
  gl.text <<"hover over objects and read cout...";
  gl.selectOnHover=true;
  gl.reportSelects=true;
  gl.reportEvents=false;
  gl.watch();
}

/************ test clicking on and identifying objects in the scene ************/

void testUI(){
  OpenGL gl;
  glUI ui;
  gl.reportEvents=true;
  gl.add(draw1,0);
  gl.add(glDrawUI,&ui);
  gl.addHoverCall(glHoverUI,&ui);
  gl.addClickCall(glClickUI,&ui);
  ui.addButton(100,100,"OK, this is it!");
  gl.watch();
}

void testImage(){
  OpenGL gl;
  byteA img;
  read_ppm(img,"box.ppm",false);
  gl.watchImage(img,true,2);
}

int main(int argc,char **argv){
  MT::initCmdLine(argc,argv);

#ifdef MT_QT
  QApplication myapp(argc,argv);
#endif
  
  testMultipleViews();
  //return 0;
  testDepth();
  testTeapot();
  testObj();
  testMesh();
  testUI();
  testTexture();
  testSelect();
  testMenu();
  testImage();

  return 0;
}



