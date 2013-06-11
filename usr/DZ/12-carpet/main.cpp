#include <MT/soc.h> 
#include <MT/util.h>
#include <MT/opengl.h>
#include <MT/ors.h>     
#include <MT/plot.h>    
#include <Core/array.h> 

//===========================================================================
 const double PI = 3.1415926535897932384626; 
// const double PI_OVER_2 = 0.5* PI;



void gridToTriangles(ors::Mesh& mesh,const doubleA &grid){
  uint i, j, k=mesh.T.d0;
  mesh.T.resizeCopy(mesh.T.d0+2*(grid.d0-1)*(grid.d1-1), 3);
  for(i=0;i<grid.d0-1;i++) for(j=0;j<grid.d1-1;j++){
    if((i+j)&1){ 
      mesh.T(k, 0)=grid(i+1, j  );
      mesh.T(k, 1)=grid(i  , j  );
      mesh.T(k, 2)=grid(i  , j+1);
      k++;
      mesh.T(k, 0)=grid(i+1, j  );
      mesh.T(k, 1)=grid(i  , j+1);
      mesh.T(k, 2)=grid(i+1, j+1); 
      k++;
    }else{
      mesh.T(k, 0)=grid(i+1, j  );
      mesh.T(k, 1)=grid(i  , j  );
      mesh.T(k, 2)=grid(i+1, j+1);
      k++;
      mesh.T(k, 0)=grid(i+1, j+1);
      mesh.T(k, 1)=grid(i  , j  );
      mesh.T(k, 2)=grid(i  , j+1);
      k++;
    } 
  }
}

void flipFace(ors::Mesh& M,uint i){
  uint  a;
    a=M.T(i, 0);
    M.T(i, 0)=M.T(i, 1);
   M.T(i, 1)=a;
  
}

void create_surface(ors::Mesh& M,uint width,double step){
  
  uint N = (width/step);
  arr verts; verts.resize(N*N,3);
  cout<<width*(1.0-1.0/N);
  doubleA grid; grid.setGrid(2,0,width*(1.0-1.0/N),N-1);
  verts.setZero();
  verts.setMatrixBlock(grid,0,0);

  M.V =verts;
  M.T.clear();
  arr order; order.resize(N*N) ;for (int i=0;i<N*N;i++) order(i)=i;
  order.reshape(N,N);
  gridToTriangles(M,order); 

} 
int factorial(int x) {
  return (x <= 1 ? 1 : x * factorial(x - 1));
}

double BernsteinPolynomial(uint i,uint n,double x) {
  
  
double coeff = 1.0*factorial(n) / (factorial(n-i) * factorial(i));
return coeff * pow(x,int(i)) * pow(1.0-x, int(n-i));
}

void BezierSurface(const arr control,arr& result){
  CHECK(result.nd==3,"WRONG DIMENSIONS");
  arr bern_n = zeros(result.d0,control.d0);
  arr bern_m = zeros(result.d1,control.d1);
  for (uint i=0;i<result.d0;i++)  
  for (uint j=0;j<control.d0;j++)  
  {
    bern_n(i,j) = BernsteinPolynomial(j,control.d0-1, 1.0*i/(result.d0  -1));  //j is sum index
  }
  for (uint i=0;i<result.d1;i++)  
  for (uint j=0;j<control.d1;j++)  
  {
    bern_m(i,j) = BernsteinPolynomial(j,control.d1-1, 1.0*i/(result.d1 -1 ));
  }
  cout <<bern_n;
 for (uint x=0;x<result.d0;x++)  
  for (uint y=0;y<result.d1;y++)  
   for (uint i=0;i<control.d0;i++)  
    for (uint j=0;j<control.d1;j++)  
   for (uint w=0;w<3;w++)
 //result[x,y]() += bern_n(x,i)*bern_m(y,j)*control[i,j]();
   {   result(x,y,w) += bern_n(x,i)*bern_m(y,j)* control(j,i,w);
    }
}


void problem1(){   
  cout <<"\n= Sample surface=\n" <<endl;
   ors::Graph ors;
   ors.init("clear.ors");
   OpenGL gl; gl.add(glStandardScene); gl.add(ors::glDrawGraph,&ors); 
   gl.camera.setPosition(0,-5,6);  gl.camera.focus(0,0,2);

  ors::Mesh newM;
  uint CARPET_WIDTH = 1;
  double CARPET_RES = 0.1;
//  orsDrawAlpha=.50;
  create_surface(newM,CARPET_WIDTH ,CARPET_RES); 
  ors::Body *b = new ors::Body(ors);
  b->X.pos.setRandom();
  b->X.pos(2) += 2.5;
  b->name <<"carpet";
  ors::Shape *s = new ors::Shape(ors, b);
  s->type=ors::meshST;
  s->mesh = newM;
  ors.calcShapeFramesFromBodies();
  gl.watch();
    for (uint i=0; i<CARPET_WIDTH/CARPET_RES; i++){
     for (uint j=0; j<CARPET_WIDTH/CARPET_RES; j++){
      //s->mesh.V(100+i,2) = sin( i/(*CARPET_WIDTH/CARPET_RES)*4*PI);
      //s->mesh.V((CARPET_WIDTH/CARPET_RES)*i+j,2) = (i+j)/(2*CARPET_WIDTH/CARPET_RES);
      s->mesh.V((CARPET_WIDTH/CARPET_RES)*i+j,2) = sin( (i+j)/(2*CARPET_WIDTH/CARPET_RES)*PI);
      
      gl.update();   
    }}
    
gl.watch();
  s->mesh.flipFaces();
  ors::Shape *s2 = new ors::Shape(ors,b,s);
  s->mesh.flipFaces();
      //!rotate surface
   for (uint t=0; t<1000;t++){  
    ors::Quaternion &rot(b->X.rot);
    ors::Quaternion a,b,c;
      c.setDeg(3.,0,0,1);
      switch((t/100)%3){
	case 0: a.setDeg(2.,0,0,1); break;
	case 1: a.setDeg(2.,0,1,0); break;
	case 2: a.setDeg(2.,1,0,0); break;
     }
    a = a*rot*c; b.setRandom();
    rot.setInterpolate(0.0, a, b);
    ors.calcBodyFramesFromJoints(); 
    gl.update();
   }
gl.watch();
}


void problem2(){   
  cout <<"\n= Bezier surface=\n" <<endl;
   ors::Graph ors;
   ors.init("clear.ors");
   OpenGL gl; gl.add(glStandardScene); gl.add(ors::glDrawGraph,&ors); 
   gl.camera.setPosition(0,-5,6);  gl.camera.focus(0,0,2);

  ors::Mesh newM;
  uint CARPET_WIDTH = 1;
  double CARPET_RES = 0.1;
//  orsDrawAlpha=.50;
  create_surface(newM,CARPET_WIDTH ,CARPET_RES); 
  ors::Body *b = new ors::Body(ors);
  b->X.pos(2) += 2.5;
  b->name <<"carpet";
  //! Bezier Control Points from file
  uint N = 9;
  arr bcp = zeros(N,3);
  ifstream in("bcp.txt");  bcp.readRaw(in); in.close(); 
  bcp*=5.0;// scale
  for (uint i=0; i<N; i++)
  {
  ors::Shape *s = new ors::Shape(ors, b);
  s->type=ors::sphereST;
  s->size[0]=s->size[1]=s->size[2]=s->size[3]=0.05;
  s->color[0]=s->color[1]=s->color[2]=0.5;
  s->rel.pos = bcp[i]();
  }
  //! end of Bezier Control Points  part
  ors::Shape *s = new ors::Shape(ors, b);
  s->type=ors::meshST;
  s->mesh = newM;
  ors.calcShapeFramesFromBodies();
  //! HERE we compute Bezier Surface from Bezier Control Points 
    arr bezier =zeros(CARPET_WIDTH/CARPET_RES,CARPET_WIDTH/CARPET_RES*3);
    bcp.reshape(3,3,3);
    bezier.reshape(CARPET_WIDTH/CARPET_RES,CARPET_WIDTH/CARPET_RES,3);
    BezierSurface(bcp,bezier); 
    //!  
    gl.watch();
    for (uint i=0; i<CARPET_WIDTH/CARPET_RES; i++)
     for (uint j=0; j<CARPET_WIDTH/CARPET_RES; j++){
         for (uint w=0;w<3;w++)
         s->mesh.V((CARPET_WIDTH/CARPET_RES)*i+j,w) = bezier(i,j,w);
      gl.update();   
    }
    
  gl.watch();
  s->mesh.flipFaces();
  ors::Shape *s2 = new ors::Shape(ors,b,s);
  s->mesh.flipFaces();
gl.watch();
}

void problem3(){   
  cout <<"\n= Bezier surface for human hand=\n" <<endl;
   ors::Graph ors;
   ors.init("bottle_grasp.ors");
   OpenGL gl; gl.add(glStandardScene); gl.add(ors::glDrawGraph,&ors); 
   gl.camera.setPosition(0,-3,3);  gl.camera.focus(0,0,1);

  ors::Mesh newM;
  uint CARPET_WIDTH = 1;
  double CARPET_RES = 0.2;
//  orsDrawAlpha=.50;
  create_surface(newM,CARPET_WIDTH ,CARPET_RES); 
  ors::Body *b = new ors::Body(ors);
  ors.calcShapeFramesFromBodies();
  gl.update();
  b->name <<"carpet";
  //  b->X.pos(2) += 0.5;
  //! Bezier Control Points from file
  uint N = 16;
  arr bcp = zeros(N,3);
  MT::Array<char*> names;
  
  names.append("palm");  names.append("palm");   names.append("palm");   names.append("palm");
  
  names.append("thumb0");  names.append("index1");   names.append("mid1");   names.append("pinky1");
  names.append("thumb1");  names.append("index2"); names.append("mid2");   names.append("pinky2");
  names.append("thumb3");  names.append("index3"); names.append("mid3");   names.append("pinky3");
  
  for (uint i=0; i<N; i++)
  {
  bcp[i]() = arr(ors.getBodyByName(names(i))->X.pos.p,3);
//  if (i<4) {bcp(i,2) += (2-i)*0.02; }
  ors::Shape *s = new ors::Shape(ors, b);
  s->type=ors::sphereST;
  s->size[0]=s->size[1]=s->size[2]=s->size[3]=0.001;
  s->color[0]=s->color[1]=s->color[2]=0.5;
  s->rel.pos = bcp[i]();
  }
  //! end of Bezier Control Points  part
  ors::Shape *s = new ors::Shape(ors, b);
  s->type=ors::meshST;
  s->mesh = newM;
  s->color[0]=1.0;
  ors.calcShapeFramesFromBodies();
  //! HERE we compute Bezier Surface from Bezier Control Points 
    arr bezier =zeros(CARPET_WIDTH/CARPET_RES,CARPET_WIDTH/CARPET_RES*3);
    bcp.reshape(sqrt(N),sqrt(N),3);
    bezier.reshape(CARPET_WIDTH/CARPET_RES,CARPET_WIDTH/CARPET_RES,3);
    BezierSurface(bcp,bezier); 
    //!  
    gl.watch();
    for (uint i=0; i<CARPET_WIDTH/CARPET_RES; i++)
     for (uint j=0; j<CARPET_WIDTH/CARPET_RES; j++){
         for (uint w=0;w<3;w++)
         s->mesh.V((CARPET_WIDTH/CARPET_RES)*i+j,w) = bezier(i,j,w);
      gl.update();   
    }
  gl.watch();
  s->mesh.flipFaces();
  ors::Shape *s2 = new ors::Shape(ors,b,s);
  s->mesh.flipFaces();
  gl.watch();
}



int main(int argn,char **argv){
  MT::initCmdLine(argn,argv);

  int mode=MT::getParameter<int>("mode");
  switch(mode){
  case 1:  problem1();  break;
  case 2:  problem2();  break;
  case 3:  problem3();  break;
  default: NIY;
  }
  return 0; 
}
