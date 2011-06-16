#include "graspObjects.h"

double
staticPhi(arr *grad, const arr &x, void *p){
  return ((PotentialField*)p)->psi(grad,x);
}
double
staticPhi(const arr &x,const void *p){
  return ((PotentialField*)p)->psi(NULL,x);
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
  return ((GraspObject*)p)->phi(NULL,NULL,x);
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


  if(MT::getParameter<uint>("plotPotentialField", 0)){
    getEnclCube(lo,hi);
    MT_MSG("new: hi="<<hi<<", lo="<<lo);
    X.setGrid(3,lo,hi,50);
    dX.resizeAs(X);
    for(i=0;i<X.d0;i++){
      X[i]()+=ce;
      psi(&dX[i](),X[i]);
    }
    dX *= .005;
    plotVectorField(X,dX); // plot gradient
  }
}


double
GraspObject::phi(arr *grad,double *var,const arr& x) { //generic phi, based only on (scaled) distance to surface
  // see gnuplot:  plot[-1:4] 1.-exp(-.5*(x+1)*(x+1))/exp(-.5)
  double d=distanceToSurface(grad,x);
  if(d<-1.) d=-1.; 
#if 0
  double phi=1.-exp(-.5*(d+1.)*(d+1.))/exp(-.5);
  if(grad){
    (*grad) = (-exp(-.5*(d+1.)*(d+1.))/exp(-.5)) * (-(d+1.)) * (*grad);
  }
#else
  double phi=d;
#endif
  if(var) *var = 0; //default variance is 0 (analytic shapes)
  return phi;
} 

double
GraspObject::psi(arr* grad,const arr& x)  {
  return phi(grad,NULL,x);
};

void
GraspObject::getNormGrad(arr& grad,const arr& x) {
  phi(&grad,NULL,x);
  double d=norm(grad);
  if(d>1e-200) grad/=d; else MT_MSG("gradient too small!");
}

/* =============== Inf cyllinder ================ */

double
GraspObject_InfCylinder::distanceToSurface(arr *grad,const arr& x){
  z = z / norm(z);
  arr d_vec = (x-c) - scalarProduct((x-c), z) * z;
  double d = norm(d_vec);
  if(grad) *grad = s*d_vec/d;
  return s*(d-r);
}

GraspObject_InfCylinder::GraspObject_InfCylinder(arr c1,arr z1, double r1, double s1){
  c = c1;
  z = z1;
  r = r1;
  s = s1;
}
GraspObject_InfCylinder::GraspObject_InfCylinder(){
  c = MT::getParameter<arr>("center");
  z = MT::getParameter<arr>("orientation");
  r = MT::getParameter<double>("radius");
  s = MT::getParameter<double>("sigma");
}

/* =============== Cut cyllinder ================ */

double
GraspObject_Cylinder1::distanceToSurface(arr *grad,const arr& x){
  z = z / norm(z);
  arr hx_vec = scalarProduct((x-c), z) * z;
  arr d_vec = (x-c) - hx_vec;
  double d = norm(d_vec);
  double hx = norm(hx_vec);
  if ( hx < h/2. ){ // x projection on z is inside cyl
    if(grad) *grad = s*d_vec/d;
    return s*(d-r);
  }else{// x projection on z is outside cylinder
    if ( d < r ){// inside the infinite cylinder
      if(grad) *grad = s*hx_vec/hx;
      return s*(hx-h/2.);
    }else{ // outside the infinite cyl
      arr vec =  hx_vec * hx / (hx+h/2.) + d_vec * d / (d+r);
      if(grad) *grad = s* vec / norm(vec); 
      return s* norm(vec);
    }
  }
}

GraspObject_Cylinder1::GraspObject_Cylinder1(){
  c = MT::getParameter<arr>("center");
  z = MT::getParameter<arr>("orientation");
  r = MT::getParameter<double>("radius");
  s = MT::getParameter<double>("sigma");
  h = MT::getParameter<double>("height");
}
GraspObject_Cylinder1::GraspObject_Cylinder1(arr c1,arr z1, double r1, double s1, double h1){
  c = c1;
  z = z1;
  r = r1;
  s = s1;
  h = h1;
}

/* =============== Sphere ================ */

double
GraspObject_Sphere::distanceToSurface(arr *grad,const arr& x){
  double d = norm(x-c);
  if(grad) *grad = s*(x-c)/d;
  return s*(d-r);
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
GraspObject_GP::phi(arr *grad, double *var, const arr& x){
  double y, sig;
  //arr x = xx - c;

  isf_gp.gp.evaluate(x,y,sig); 

  if (grad) isf_gp.gp.gradient(*grad, x);

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

