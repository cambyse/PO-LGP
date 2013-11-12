
#ifndef MT_graspObjects_h
#define MT_graspObjects_h

#include <Ors/ors.h>
#include "ISF_GP.h"

//MT: move this to cpp file
double staticPhi(double x, double y, double z, void *p);
double staticPhi(const arr&, const void *);
double staticPhi(arr*, const arr&, void *);
double staticPhi(arr*,arr*, const arr&, void *);

//===========================================================================
//
// generic grasp object that defines an implicit shape potential
//

//a generic 3D potential, e.g., to represent an implicit surface
struct MeshObject {
  ors::Mesh m;
  MeshObject(char* meshfile, const arr& center, const double scale);
  MeshObject(){};
  void getEnclCube(double &lo, double &hi);
  void getEnclRect(arr &maxs, arr &mins);
  /* gl and meshes */
  virtual void glDraw(){ if(m.V.N) m.glDraw(); };
};

struct PotentialField : public MeshObject
{
  arr X,dX; //vector fields for plotting

  virtual double psi(arr* grad,arr *hess,const arr& x)=0;
  void getNormGrad(arr& grad,const arr& x){
    psi(&grad,NULL,x);
    double d=length(grad);
    if(d>1e-200) grad/=d; else MT_MSG("gradient too small!");
  }
  void buildMesh();
  void saveMesh(const char *filename);
  void loadMesh(const char *filename);
  virtual arr center(){return ARR(0,0,0);}//FIX: ugly. maybe 'out' parameter by reference rather than 'return'.

};

struct GraspObject : public PotentialField
{
  bool distanceMode;
  GraspObject(){ distanceMode = false; }
  
  virtual double distanceToSurface(arr *grad, arr *hess, const arr& x) { NIY; }
  virtual double psi(arr* grad, arr *hess, const arr& x); //MT: what is the difference between psi and phi?
  virtual double phi(arr *grad, arr *hess, double *var,const arr& x);
  void getNormGrad(arr& grad,const arr& x) ;
  virtual double max_var(){return 0;}

};

void glDrawMeshObject(void*p);

struct GraspObject_InfCylinder:public GraspObject {
  arr c;    //center
  arr z;    //z orientation
  double r; //radius
  double s; //kernel parameter
  
  double distanceToSurface(arr *grad,arr *hess,const arr& x);
  GraspObject_InfCylinder();
  GraspObject_InfCylinder(arr c1,arr z1, double r1, double s1);
  arr center(){return c;};
};

struct GraspObject_Cylinder1:public GraspObject { // poor man's cylinder
  arr c;    //center
  arr z;    //z orientation
  double r; //radius
  double h; //height = 2 * (center to plane)
  double s; //kernel parameter
  
  double distanceToSurface(arr *grad,arr *hess,const arr& x);
  GraspObject_Cylinder1();
  GraspObject_Cylinder1(arr, arr, double, double, double);
  GraspObject_Cylinder1(const ors::Shape* s);
  arr center(){return c;};
};

struct GraspObject_Box:public GraspObject {
  arr c;    //center
  arr dim,rot;
  double s; //kernel parameter
  
  double distanceToSurface(arr *grad,arr *hess,const arr& x);
  GraspObject_Box();
  GraspObject_Box(const arr& center, double dx_, double  dy_, double dz_); //assumes box is axis aligned
  GraspObject_Box(const ors::Shape* s);
  arr center(){ return c; };
};


struct GraspObject_Sphere:public GraspObject {
  arr c;    //center
  double r; //radius
  double s; //kernel parameter
  
  double distanceToSurface(arr *grad,arr *hess,const arr& x);
  GraspObject_Sphere();
  GraspObject_Sphere(arr&, double, double);
  arr center(){return c;};
};

struct GraspObject_GP:public GraspObject {

  arr c;
  double d;
  isf_gp_t isf_gp;
  
  double phi(arr *grad,arr *hess,double *var,const arr& x);
  
  GraspObject_GP();
  GraspObject_GP(const arr&, const double);
  double max_var();
  arr center(){return c;};
};

struct GraspObject_GPblob:public GraspObject_GP { // blub demo

  GraspObject_GPblob();
};


struct GraspObject_GP_analytical_prior:public GraspObject_GP {

  GraspObject *prior;
  
  GraspObject_GP_analytical_prior(GraspObject *prior);
};

struct offset_param_t {
  arr off;
  void *p;
  offset_param_t(arr _off, void *_p){p=_p; off=_off;};
};

#ifdef  MT_IMPLEMENTATION
#  include "graspObjects.cpp"
#endif

#endif

