
#ifndef SD_rbf_h
#define SD_rbf_h

#include <Core/array.h>

/*RBF params*/
struct RBF_param_t {
	arr c;
	double s;
	double r;
};



/**  exp(- s * dist ) */
double
RBF_gauss( const double s, const double dist){
  return  ::exp( - s * dist*dist ) ;
}

/** RBFupside down and translated to center c, pushed up so that as ISF defines
 * a sphere with radius r around c. */
double
ISF_RBF_gauss(const RBF_param_t  *p, const arr &x){
  return  - RBF_gauss(p->s, length(x - p->c)) + RBF_gauss(p->s, p->r); 
}

void
ISF_RBF_gauss_gradient(arr& grad, const arr& x, const RBF_param_t *p){
    grad = (-2*p->s*(p->c - x)) * ISF_RBF_gauss(p,x);
}

#endif// header ifdef
