#include <MT/soc.h> 
#include <MT/util.h>
#include <MT/opengl.h>
#include <MT/ors.h>     
#include <MT/plot.h>    
#include <MT/array.h> 
#include "TriMesh.h"
//#include "TriMesh_algo.h"
//#include "strutil.h"
//#include "KDtree.h"
//#include "lineqn.h"
//#include <algorithm>

//===========================================================================
 const double PI = 3.1415926535897932384626; 
// const double PI_OVER_2 = 0.5* PI; 
 
 

 Color intcurv_test(TriMesh *mesh, int v, int nedges, int typ)
{
       
	int nv = mesh->vertices.size();
        mesh->flags.clear(); // neighbors distance
	mesh->flags.resize(nv);
	for (int i = 0; i < nv; i++)
		mesh->flags[i] = (i == v) ? 0 : nedges;
	for (int iter = 1; iter < nedges; iter++) {
		for (int i = 0; i < nv; i++) {
			for (int j = 0; j < mesh->neighbors[i].size(); j++) {
				int n = mesh->neighbors[i][j];
				if (mesh->flags[n] + 1 < mesh->flags[i])
					mesh->flags[i] = mesh->flags[n] + 1;
			}
		}
	}
       Color my_color;
       int counter =0;
       float test_curv =0;

	for (int i = 0; i < nv; i++) {
		if (mesh->flags[i] < nedges){
		//test_curv += 0.5f * (mesh->curv1[i] + mesh->curv2[i]);// MEAN CURV
                test_curv +=mesh->curv1[i]* mesh->curv2[i]; // GAUSSIAN CURV
		my_color=my_color+mesh->colors[i];// = Color(1.0f, c, c);
		counter++;
		}
               // mesh->colors[i] = Color(1.0f, 1.0f, 1.0f);
	}
        my_color/=counter;
    //    test_curv/=counter;
       return Color(test_curv, 0.0f, 0.0f);//my_color;
}
void intcurv_mesh(TriMesh *mesh, const char *v_, int nedges_)
{
      int nv = mesh->vertices.size();
      TriMesh *new_mesh = TriMesh::read(v_); // mesh for computing Gaussian curvature integrals
      new_mesh->colors.resize(new_mesh->vertices.size());
      new_mesh->need_neighbors();
      new_mesh->need_curvatures();
       Color tmp;
 
       float maxn=0;float minn=0;
         for (int i = 0; i< nv;i++){
           tmp= intcurv_test(new_mesh,i,nedges_,0); // radius = nedges
           mesh->colors[i]=tmp;  
           if (mesh->colors[i][0]>maxn) maxn = mesh->colors[i][0];
           if (mesh->colors[i][0]<minn) minn = mesh->colors[i][0];
           }
 float counter=0.0f;
 for (int i = 0; i< nv;i++){
    mesh->colors[i][0]=(mesh->colors[i][0]-minn)/ (maxn-minn);
   if (mesh->colors[i][0]>=0.8) {mesh->colors[i][1]=1.0f; counter++;} //! if 80 %  threshold of maximum add greeb component
   
}
cout <<maxn<<endl;
cout <<minn<<endl;
cout <<maxn<<endl;
}

//! BETTER VERSION BELOW
double integral_curvature(TriMesh *mesh, int point, double radius, int type, bool test)
{
        double result=0;
	arr nodes; arr nnodes;
            
	
	int level=1; nodes.append(point);
	while ((level<radius)&&(nodes.N>0))
	{
	  
	  for (uint i=0; i<nodes.N;i++) 
	    for (int j = 0; j < mesh->neighbors[nodes(i)].size(); j++) {
	       int n = mesh->neighbors[nodes(i)][j];
	       if (!nnodes.contains(n)) nnodes.append( n); 
	      if (test) mesh->colors[n] = Color(1.0f, 1.0f, 1.0f);
	       if (type==0)    result+= mesh->curv1[n]* mesh->curv2[n];
	       else result+= 0.5f*(mesh->curv1[n]+ mesh->curv2[n]);
	    }
	  level++;
	  nodes.clear(); copy(nodes,nnodes);
	}
       return result;
}


// Color whole mesh by curvature integral
void integral_curvature_mesh(TriMesh *mesh, const char * filename, double radius, int type)
{
      int nv = mesh->vertices.size();
      TriMesh *new_mesh = TriMesh::read(filename); // mesh for computing Gaussian curvature integrals
      new_mesh->colors.resize(new_mesh->vertices.size());
      new_mesh->need_neighbors();
      new_mesh->need_curvatures();
      double tmp;
      arr curvatures; curvatures.resize(nv); curvatures.reshape(curvatures.N);
       float maxn=0;float minn=0;
         for (int i = 0; i< nv;i++){
           tmp= integral_curvature(new_mesh,i,radius,type,false); // radius = nedges
           //mesh->colors[i]=tmp;  
	   curvatures(i) = tmp;
           if (tmp>maxn) maxn = tmp;
           if (tmp<minn) minn = tmp;
           }
 float counter=0.0f;
 for (int i = 0; i< nv;i++){
    mesh->colors[i][0]=(curvatures(i)-minn)/ (maxn-minn);
   if (mesh->colors[i][0]>=0.6) {mesh->colors[i][1]=1.0f; counter++;} //! if 80 %  threshold of maximum add green component
   
}

cout <<minn<<endl;
cout <<maxn<<endl;
}

void problem1(){   
  cout <<"\n= Sample trimesh functions test=\n" <<endl;
   ors::Graph ors; ors.init("clear.ors");
   OpenGL gl; gl.add(glStandardScene); gl.add(ors::glDrawGraph,&ors); 
   gl.camera.setPosition(0,-5,6);  gl.camera.focus(0,0,2);

  ors::Mesh newM;
  ors::Body *b = new ors::Body(ors);
  b->X.pos(2) += 2.5;
  b->name <<"main";
  ors::Shape *s = new ors::Shape(ors, b);
  s->type=ors::meshST;
  const char * filename = "input1.obj";
  const char * outfilename = "output1.ply";  //! PLY works with colors, but ORS can not currently read it properly
  int scale = 0.1;
  s->mesh.readObjFile(filename);
 
  //! import of trimesh functions here
  TriMesh *themesh = TriMesh::read(filename);
  themesh->colors.resize(themesh->vertices.size());
  intcurv_mesh(themesh, filename,4);
  themesh->write(outfilename);
  //! end of import
  ors.calcShapeFramesFromBodies();
  gl.watch();
}


void problem2(){   
  cout <<"\n= Sample trimesh functions test=\n" <<endl;
   ors::Graph ors; ors.init("clear.ors");
   OpenGL gl; gl.add(glStandardScene); gl.add(ors::glDrawGraph,&ors); 
   gl.camera.setPosition(0,-5,6);  gl.camera.focus(0,0,2);

  ors::Mesh newM;
  ors::Body *b = new ors::Body(ors);
  b->X.pos(2) += 2.5;
  b->name <<"main";
  ors::Shape *s = new ors::Shape(ors, b);
  s->type=ors::meshST;
  const char * filename = "input1.obj";
 // const char * filename = "body_clean.obj";
  const char * outfilename = "output1.ply";  //! PLY works with colors, but ORS can not currently read it properly
  int scale = 0.1;
  s->mesh.readObjFile(filename);
 
  //! import of trimesh functions here
  TriMesh *themesh = TriMesh::read(filename);
  themesh->colors.resize(themesh->vertices.size());
  themesh->need_neighbors();
  themesh->need_curvatures();
  //integral_curvature(themesh, 2300,32,0,true);
  integral_curvature_mesh(themesh,filename,5,0);
   //integral_curvature_mesh(themesh,filename,3,0);
  
  themesh->write(outfilename);
  //! end of import
  ors.calcShapeFramesFromBodies();
  gl.watch();
}

int main(int argn,char **argv){
  MT::initCmdLine(argn,argv);

  int mode=MT::getParameter<int>("mode");
  switch(mode){
  case 1:  problem1();  break;
  case 2:  problem2();  break;
  default: NIY;
  }
  return 0; 
}
