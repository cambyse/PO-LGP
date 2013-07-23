#ifndef SD_isf_gp_h
#define SD_isf_gp_h

#include <Core/array.h>
#include <Algo/gaussianProcess.h>

/** type of saling of gaussian process */
enum SD_scale_t {scX=1, scY=2, scXY=3}; //scXY is short for (scX|scY)

/** standard detail size */
const double std_size = .5;
const GaussKernelParams std_gkp(.1, 1.,.2,.5);

struct isf_gp_t{

  GaussianProcess gp;
  double d; // unique detail size; selects the right parametrisation of GP
  GaussKernelParams p;
  
  isf_gp_t();
  isf_gp_t(const double dsize, const double );
  
  void set_size(const double);
  void scale_gp_params(const double, const SD_scale_t );
  void scale_gp_input(const double, const SD_scale_t );
  void translate_gp_input(const arr&);
  void set_shape_prior( double (*_mu)(const arr&, const void*), void *priorP);

  void write(std::ostream& os) const;
  void read(std::istream& is);
  
};


// ================= helpers ================
/** scale GP by the given factor (scale parameters only, not input) */
void scale_gp_params(GaussianProcess &, const double, const SD_scale_t);
/** scale GP inputs by the given factor */
void scale_gp_input(GaussianProcess &, const double, const SD_scale_t);
/** compute "optimal" GaussKernelParams for detail size */
void gp4d(GaussianProcess &, const double dsize);


// ================ utils ==================
/** generate random observations around given center */
void randomGP_on_random_points(GaussianProcess &gp, const arr &c=ARR(0,0,0), double size=1., uint np=5);



#ifdef MT_IMPLEMENTATION
#include "ISF_GP.cpp"
#endif

#endif// header ifdef
