#define MT_IMPLEMENT_TEMPLATES
#include "graspObjects.h"
#include <MT/array.h>
#include <MT/util.h>

double
staticPhi(arr *grad, arr *hess, const arr &x, void *p){
  return ((PotentialField*)p)->psi(grad,hess,x);
}
double
staticPhi(arr *grad, const arr &x, void *p){
  return ((PotentialField*)p)->psi(grad,NULL,x);
}
double
staticPhi(const arr &x,const void *p){
  return ((PotentialField*)p)->psi(NULL,NULL,x);
}
double
staticPhi(double x,double y,double z,void *p){
  return staticPhi(ARR(x,y,z), p);
}
/* phi translated to some offset*/
double
offset_f(double x,double y,double z,void *_p){
  offset_param_t *p = (offset_param_t *)_p;
  return staticPhi(p->off + ARR(x,y,z), p->p);
}


double
static_mu(const arr &x, const void *p){
  return ((GraspObject*)p)->phi(NULL,NULL,NULL,x);
}

MeshObject::MeshObject(char *meshfile, const arr& c, const double sc){
  CHECK(sc,"cant scale by 0");
  uint i;

  m.readFile(meshfile);
  m.scale(sc);
  //mesh->fuseNearVertices(1e-1);
 // m.V = m.V + ~c;
  FOR1D(m.V, i) m.V[i]() += c;
}

void
MeshObject::getEnclRect(arr &mins, arr &maxs){
  arr vertx_col;
  uint i;

  if (!m.V.N) {mins.setZero();maxs.setZero();return;} // no mesh
  mins.resize(3);maxs.resize(3);
  vertx_col = ~m.V;
  for(i = 0; i<3; ++i){
    mins(i) = vertx_col[i].min();
    maxs(i) = vertx_col[i].max();
  }
}

void
MeshObject::getEnclCube(double &lo, double &hi){
  arr mins, maxs;
  getEnclRect(mins,maxs);
  if (mins == maxs){lo=0;hi=0;return;}
  lo=mins.min();hi=maxs.max();
  lo -= .2*(hi-lo); hi += .17*(hi-lo);//?here?
}

void glDrawMeshObject(void*p){ ((MeshObject*)p)->glDraw(); }

uint plot_field_dens() {
  static uint value;
  static bool read=true;
  if(read){
    read=false;
    value = MT::getParameter<uint>("plotPotentialFieldDensity",50);
  }
  return value;
}
double plot_field() {
  static double value;
  static bool read=true;
  if(read){
    read=false;
    value = MT::getParameter<double>("plotPotentialField");
  }
  return value;
}

void PotentialField::saveMesh(const char *filename){
  m.writeTriFile(filename);
}

void PotentialField::loadMesh(const char *filename){
  m.readFile(filename);
}

void
PotentialField::buildMesh(){
  uint i;
  arr mins, maxs;
  arr ce = center();
  double lo, hi;
  const uint resol = 100;
  const double range = 1.5;
  const double step = 2*range/resol;

  // get center, mins and maxs of mesh
  if (!m.V.N) m.setImplicitSurface(staticPhi,this,-range,range,resol);
  getEnclRect(mins,maxs);

  if (mins == maxs) {MT_MSG("no surface found in "<<-range<<","<<range<<" at resolution"<<resol); return;}
  mins-=ARR(step,step,step);// FIX: constants
  maxs+=ARR(step,step,step);

  arr off = .5 * (maxs + mins);
  //hi = (off-mins+.01*(maxs-mins)).max(); lo = -hi;// FIX: 100
  hi = (maxs-mins).max(); hi *= 1.5; lo = -hi;
  MT_MSG("offs=" <<off<<"mins=" <<mins<<" ,maxs=" <<maxs<<", hi="<<hi<<", lo="<<lo);


  offset_param_t po(off,this);
  m.setImplicitSurface(offset_f,&po,lo,hi,50);
  m.translate(off(0),off(1),off(2));


  if(0!=plot_field()){
    getEnclCube(lo,hi);
    MT_MSG("new: hi="<<hi<<", lo="<<lo);
    X.setGrid(3,lo,hi,plot_field_dens());
    dX.resizeAs(X);
    for(i=0;i<X.d0;i++){
      X[i]()+=ce;
      psi(&dX[i](),NULL,X[i]);
    }
    dX *= plot_field();
    plotVectorField(X,dX); // plot gradient
  }
}

/** Value and gradient and hessian and variance of the potential field. 
 *
 * Variance doesn't make much sense when not a Gaussian Process or another
 * probabilistic method: here variance is zero indicating to the invoker that
 * vaiance doesn't play a role.
 *
 * ACHTUNG: nonNULL hess implies nonNULL grad
 * look at mlr/stanio/concepts/note_analytic_impl_shapes_hessian.tex in the
 * repository for more details
 */
double
GraspObject::phi(arr *grad,arr *hess,double *var,const arr& x) { //generic phi, based only on (scaled) distance to surface
  // see gnuplot:  plot[-1:4] 1.-exp(-.5*(x+1)*(x+1))/exp(-.5)
  double d = distanceToSurface(grad,hess,x);
  double e = exp(-.5*(d+1.)*(d+1.))*exp(.5);
  if(d<-1.) d=-1.;
  double phi;
  if(!distanceMode){
    phi=1.-e;
    if(grad){
      *grad = e * (d+1.) * (*grad);
    }
    if(hess){
      CHECK(grad!=NULL,"need to store gradient.");
      /* ed(d+2)\nabla\nabla^T + e*(d+1)*H  */
      *hess = - e*d*(d+2.) * (*grad) * (~*grad)
      + e*(d+1.) * (*hess);
    }
  }else{
    //take directly the distance as task variable
    //hessian and gradient are correctly returned from distanceToSurface
    phi=d;
  }
  if(var) *var = 0; //default variance is 0 (analytic shapes)
  return phi;
} 

double
GraspObject::psi(arr* grad,arr* hess,const arr& x)  {
  return phi(grad,hess,NULL,x);
};

void
GraspObject::getNormGrad(arr& grad,const arr& x) {
  phi(&grad,NULL,NULL,x);
  double d=norm(grad);
  if(d>1e-200) grad/=d; else MT_MSG("gradient too small!");
}

/* =============== Inf cyllinder ================ */

/** distance to surface, distance gradient, and hessian for this shape
 *
 * Details in inf cylinder section of 
 * mlr/stanio/concepts/note-analytic-impl-shapes-hessian
 */
double
GraspObject_InfCylinder::distanceToSurface(arr *grad,arr *hess,const arr& x){
  z = z / norm(z);
  arr a = (x-c) - scalarProduct((x-c), z) * z;
  arr I(x.d0,x.d0);
  uint i;
  double na = norm(a);

  if(grad) *grad = s*a/na;
  if(hess){
    I.setZero();
    for(i=0;i<x.d0;++i) I(i,i)=1;
    *hess = s/na * (I - z*(~z) - 1/(na*na) * a*(~a));
  }
  return s*(na-r);
}

/* construct from center, ori, rad, sigma */
GraspObject_InfCylinder::GraspObject_InfCylinder(arr c1,arr z1, double r1, double s1){
  c = c1;
  z = z1;
  r = r1;
  s = s1;
}

/* construct from cmd line params */
GraspObject_InfCylinder::GraspObject_InfCylinder(){
  c = MT::getParameter<arr>("center");
  z = MT::getParameter<arr>("orientation");
  r = MT::getParameter<double>("radius");
  s = MT::getParameter<double>("sigma");
}

/* =============== Cut cyllinder ================ */

double
GraspObject_Cylinder1::distanceToSurface(arr *grad,arr *hess,const arr& x){
  z = z / norm(z);
  arr b = scalarProduct((x-c), z) * z;
  arr a = (x-c) - b;
  arr I(x.d0,x.d0);
  uint i;
  double na = norm(a);
  double nb = norm(b);
  arr aaTovasq = 1/(na*na) * a*(~a);
  arr zzT = z*(~z);

  if ( nb < h/2. ){ // x projection on z is inside cyl
    if(na<r && (h/2.-nb)<(r-na)){ // x is INSIDE the cyl and closer to the lid than the wall
      if(grad) *grad = s*z;
      if(hess) { I.setZero(); *hess=I; }
      return s*(nb-h/2.);
    }else{
      if(grad) *grad = s*a/na;
      if(hess){
        I.setZero();
        for(i=0;i<x.d0;++i) I(i,i)=1;
        *hess = s/na * (I - zzT - aaTovasq);
      }
      return s*(na-r);
    }
  }else{// x projection on z is outside cylinder
    if ( na < r ){// inside the infinite cylinder
      if(grad) *grad = s*z;
      if(hess) { I.setZero(); *hess=I; }
      return s*(nb-h/2.);
    }else{ // outside the infinite cyl
      arr v =  b/nb * (nb-h/2.)  + a/na * (na-r); //MT: good! (note: b/nb is the same as z)
      double nv=norm(v);
      if(grad) *grad = s* v/nv; 
      if(hess){
      I.setZero();
      for(i=0;i<x.d0;++i) I(i,i)=1;
      arr dvdx = (na-r)/na*( I - zzT - aaTovasq ) 
                 + aaTovasq + zzT;
      *hess = s/nv* (dvdx - 1/nv/nv * v * (~v) * (~dvdx) );
      }
      return s* nv;
    }
  }
  HALT("You shouldn't be here!");
}

/* construct from config */
GraspObject_Cylinder1::GraspObject_Cylinder1(){
  c = MT::getParameter<arr>("center");
  z = MT::getParameter<arr>("orientation");
  r = MT::getParameter<double>("radius");
  s = MT::getParameter<double>("sigma");
  h = MT::getParameter<double>("height");
}

/* construct from center, ori, radius,sigma, height */
GraspObject_Cylinder1::GraspObject_Cylinder1(arr c1,arr z1, double r1, double s1, double h1){
  c = c1;
  z = z1;
  r = r1;
  s = s1;
  h = h1;
}

/* =============== Box ================ */

double GraspObject_Box::distanceToSurface(arr *grad,arr *hess,const arr& x){
  arr a = x-c;
  //box dimensions:

  arr a_rel = axes*a;

  double d;
  arr closest(3);
  arr del = fabs(a_rel)-dim;
  if(del.max()<0.){ //inside
    closest.setZero();
    uint side=del.maxIndex(); //which side are we closest to?
    if(a_rel(side)>0) closest(side) = dim(side);  else  closest(side)=-dim(side); //in positive or neg direction?
    d = -norm(a_rel - closest);
  }else{ //outside
    closest = a_rel;
    closest = MT::MAX(-dim,closest);
    closest = MT::MIN(dim,closest);
    d = norm(a_rel - closest);
  }
  
  if(grad) *grad = (~axes)*(a_rel - closest)/d; //transpose(R) rotates the gradient back to world coordinates
  if(hess) NIY;
  return d;
}

GraspObject_Box::GraspObject_Box(){ NIY }

GraspObject_Box::GraspObject_Box(const arr& center, double dx_, double  dy_, double dz_){
  //assumes box is axis aligned
  c=center;
  dim = ARR(dx_,dy_,dz_);
  axes = ARR(MT_SQRT2/2.,MT_SQRT2/2.,0,
             -MT_SQRT2/2.,MT_SQRT2/2.,0,
             0,0,1);
  axes = ARR(1,0,0,
             0,1,0,
             0,0,1);
  axes.reshape(3,3);
  s=1.;
}

/* =============== Sphere ================ */

double
GraspObject_Sphere::distanceToSurface(arr *grad,arr *hess,const arr& x){
  arr d = x-c;
  double nd = norm(d);
  arr I(x.d0,x.d0);
  uint i;

  if(grad) *grad = s*d/nd;
  if(hess){
    I.setZero();
    for(i=0;i<x.d0;++i) I(i,i)=1;
    *hess = s/nd * (I - 1/(nd*nd) * d*(~d));
  }
  return s*(nd-r);
}

GraspObject_Sphere::GraspObject_Sphere(){
  c = MT::getParameter<arr>("center");
  r = MT::getParameter<double>("radius");
  s = MT::getParameter<double>("sigma");
}

GraspObject_Sphere::GraspObject_Sphere(arr &c1, double r1, double s1){
  c = c1;
  r = r1;
  s = s1;
}

/* =============== ISF GP ================ */

double
GraspObject_GP::phi(arr *grad, arr *hess, double *var, const arr& x){
  double y, sig;
  //arr x = xx - c;

  isf_gp.gp.evaluate(x,y,sig); 

  if (grad) {isf_gp.gp.gradient(*grad, x); *grad /= norm(*grad);}

  if (hess) isf_gp.gp.hessian(*hess, x);

  if (var) *var = sig ;

  //SD_DBG("x="<<x<<"; y="<<y<<" sig="<<sig<<" gradient="<<((grad)?*grad:0));

  return y;
}

GraspObject_GP::GraspObject_GP(const arr &cc,const double dd){
  c = cc;
  d = dd;

  isf_gp.set_size(d);
}

GraspObject_GP::GraspObject_GP(){
  c = MT::getParameter<arr>("center");
  d = MT::getParameter<double>("objsize");

  isf_gp.set_size(d);
}

double
GraspObject_GP::max_var(){
  return isf_gp.gp.max_var();
}

/* =============== ISF GP random  ================ */

GraspObject_GPblob::GraspObject_GPblob():GraspObject_GP(){
  // generate object
  rnd.seed(MT::getParameter<uint>("seed", 1));
  randomGP_on_random_points(isf_gp.gp, c);
  isf_gp.gp.recompute();
}

/* =============== ISF GP + function prior  ================ */

GraspObject_GP_analytical_prior::GraspObject_GP_analytical_prior(GraspObject *_prior):GraspObject_GP(){
  // the default constructors for GraspObject_GP have set the parameters at
  // this point

  //the def constr for the prior has set the params
  prior = _prior;

  // generate object around prior
  isf_gp.set_shape_prior(static_mu, prior);
  rnd.seed(MT::getParameter<uint>("seed", 1));
  randomGP_on_random_points(isf_gp.gp, c,
      MT::Parameter<double>("objsize"),
      2); // NOTE: c == prior.c
  isf_gp.gp.recompute();
}

