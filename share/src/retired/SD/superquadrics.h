#ifndef SD_superquadrics_h
#define SD_superquadrics_h

#include <Core/array.h>

/* parameters for superquadrics: kx*|x|^px + ky*|y|^py +kz*|z|^pz = b */
struct sq_param{
  double b; //bias
  double px; double py; double pz; // powers
  double kx; double ky; double kz; // coeficients
  void set(){b=1;kx=ky=kz=px=py=pz=1;}; // default no bias and linear
  void set(double _b, double _px, double _py, double _pz){ set(_px,_py,_pz); b=_b; }
  void set(double _px, double _py, double _pz){
    b=1;
    kx=ky=kz=1;
    px=_px; py=_py; pz=_pz;
  };
  void set(double _b,  double _px, double _py, double _pz, double _kx, double _ky, double _kz){
    b=_b;
    px=_px; py=_py; pz=_pz;
    kx=_kx; ky=_ky; kz=_kz;
  };
  void set(arr& a){
    switch (a.N){
      case 0: set(); break;
      case 1: set();b=a(0); break;
      case 3: set(a(0),a(1), a(2)); break;
      case 4: set(a(0),a(1), a(2),a(3)); break;
      case 7: set(a(0),a(1),a(2),a(3),a(4),a(5),a(6)); break;
      default: HALT("bad array number of els");
    };
  }

};

void
superquadricsgradient(arr& dx, arr& x, void *par){

  sq_param *p = (sq_param*)par; 
  // d(k*|x|^a)/dx = (a-1)*k*|x|^(a-1)*x/|x|
  dx = ARR(
     (x(0)>0?1:(-1)) * p->kx * p->px * pow(fabs(x(0)), p->px - 1),
     (x(1)>0?1:(-1)) * p->ky * p->py * pow(fabs(x(1)), p->py - 1),
     (x(2)>0?1:(-1)) * p->kz * p->pz * pow(fabs(x(2)), p->pz - 1) 
     );
}
double
superquadrics(double x,double y,double z, void *par){

  sq_param *p = (sq_param*)par; 
  double v
    = p->kx * pow(fabs(x),p->px) 
    + p->ky * pow(fabs(y),p->py)
    + p->kz * pow(fabs(z),p->pz)
    - p->b;
  return v;
}


#endif// header ifdef
