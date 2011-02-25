#ifndef SD_isf_gp_h
#define SD_isf_gp_h

#include <MT/array.h>
#include <MT/gaussianProcess.h>

/** type of saling of gaussian process */
enum SD_scale_t {scX=1, scY=2, scXY=3}; //scXY is short for (scX|scY)

/** standard detail size */
const real std_size = .5;
const GaussKernelParams std_gkp(.1, 1.,.2,.5);

struct isf_gp_t{

  GaussianProcess gp;
  real d; // unique detail size; selects the right parametrisation of GP
  GaussKernelParams p;
  
  isf_gp_t();
  isf_gp_t(const real dsize, const real );
  
  void set_size(const real);
  void scale_gp_params(const real, const SD_scale_t );
  void scale_gp_input(const real, const SD_scale_t );
  void translate_gp_input(const arr&);
  void set_shape_prior( double (*_mu)(const arr&, const void*), void *priorP);

  void write(std::ostream& os) const;
  
};


// ================= helpers ================
/** scale GP by the given factor (scale parameters only, not input) */
void scale_gp_params(GaussianProcess &, const real, const SD_scale_t);
/** scale GP inputs by the given factor */
void scale_gp_input(GaussianProcess &, const real, const SD_scale_t);
/** compute "optimal" GaussKernelParams for detail size */
void gp4d(GaussianProcess &, const real dsize);


// ================ utils ==================
/** generate random observations around given center */
void randomGP_on_random_points(GaussianProcess &gp, const arr &c=ARR(0,0,0), double size=1., uint np=5);



#ifdef MT_IMPLEMENTATION
#include "ISF_GP.cpp"
#endif

#endif// header ifdef
