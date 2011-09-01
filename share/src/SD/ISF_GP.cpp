
#include "ISF_GP.h"
#include <SD/utils.h>
#include <ostream>


/** scale parameters by iven factor in X, Y or both */
void
scale_gp_params(GaussianProcess &gp, const double sc, const SD_scale_t sct){

  CHECK(sc > 0 , "Really want to have non-positive scaling "<<sc<<" ?!");
  SD_DBG3("Scaling x by "  << (sct & scX? sc: 1.)<<"; "
      <<"Scaling y by "  << (sct & scY? sc: 1.));

  GaussKernelParams *p = (GaussKernelParams*)gp.kernelP;
  p->obsVar    *= (sct & scY? sc*sc: 1.);
  p->priorVar  *= (sct & scY? sc*sc: 1.);
  p->widthVar  *= (sct & scX ? sc*sc: 1.); // either x or y
  p->derivVar  *= (sct & scY? sc*sc: 1.) / (sct & scX? sc*sc: 1.);
  gp.mu = (sct & scY? sc: 1.);

}

/** scale input by given factor in X, Y or both*/
void
scale_gp_input(GaussianProcess &gp, const double sc, const SD_scale_t sct){
  double sf_x = (sct&scX? sc: 1);
  double sf_y = (sct&scY? sc: 1);
  double sf_dydx = sf_y / sf_x;

  gp.X  *= sf_x;
  gp.dX *= sf_x;
  gp.Y  *= sf_y;
  gp.dY *= sf_dydx;
}


/** come up with the "right" parameters for given detail size */
void
gp4d(GaussianProcess &gp, const double dsize){
  CHECK(dsize > 0, "It is a bad idea to have non-positive detail size.");

  GaussKernelParams *p = (GaussKernelParams*)gp.kernelP;

  // kinda standard for d=.5=std_size
  // TODO  revise values below, incl. std_size
  p->obsVar    = std_gkp.obsVar;
  p->priorVar  = std_gkp.priorVar;
  p->widthVar  = std_gkp.widthVar;
  p->derivVar  = std_gkp.derivVar;

  gp.mu = 1.;
  
  scale_gp_params(gp, dsize/std_size, scXY);
}

/** create standard-sized gp (standard is randomly picked, more or less)
 * You should fit it to your needs by scaling or set_size() */
isf_gp_t::isf_gp_t(){
  // set GP's parameter pointer to this->gkp
  gp.setGaussKernelGP(&p, 1); // TODO come up with mu-policy dependent on dsize
  set_size(std_size);
}

/** create a GP parametrized to the detail size and using the given bias */
isf_gp_t::isf_gp_t(const double dsize, const double bias){
  gp.setGaussKernelGP(&p, bias); // TODO come up with mu-policy dependent on dsize
  set_size(dsize);
}

/** scale the GP sc times according to the scale type given*/
void
isf_gp_t::scale_gp_params(const double sc, const SD_scale_t sct){
  ::scale_gp_params(gp, sc, sct);
}

/** scale the input sc times according to the scale type given*/
void
isf_gp_t::scale_gp_input(const double sc, const SD_scale_t sct){
  ::scale_gp_input(gp, sc, sct);
}

/** homogeneous transform of input */
void
isf_gp_t::translate_gp_input(const arr &t){
  uint i;

  FOR1D(gp.X,i){
    gp.X[i] = gp.X[i] + t;
  }
  FOR1D(gp.dX, i){
    gp.dX[i] = gp.dX[i] + t;
  }
}


/** select proper parameters for the detail size given*/
void
isf_gp_t::set_size(const double dsize){
  d = dsize;
  ::gp4d(gp,d);
}

void
isf_gp_t::set_shape_prior( double (*_mu)(const arr&, const void*), void *priorP){
  // stay with the same GP params, and set prior
  gp.setGaussKernelGP(
      gp.kernelP,
      _mu,
      priorP);
}


inline std::istream&
operator>>(std::istream& is,isf_gp_t& x){ x.read(is); return is; }
void
isf_gp_t::read(std::istream& is) {
  char c=' ';
  GaussKernelParams *gkp=(GaussKernelParams*)gp.kernelP;
  is >>
    gkp->obsVar>>c>>
    gkp->priorVar>>c>>
    gkp->widthVar>>c>>
    gkp->derivVar>>c>>
    gp.X>>c>>
    gp.Y>>c>>
    gp.dX>>c>>
    gp.dY>>c>>
    gp.dI>>c>>
    gp.mu>>c
    ;
}
inline std::ostream&
operator<<(std::ostream& os,const isf_gp_t& x){ x.write(os); return os; }
void
isf_gp_t::write(std::ostream& os) const{
  char c=' ';
  GaussKernelParams *gkp=(GaussKernelParams*)gp.kernelP;
  os <<
    gkp->obsVar<<c<<
    gkp->priorVar<<c<<
    gkp->widthVar<<c<<
    gkp->derivVar<<c<<
    gp.X<<c<<
    gp.Y<<c<<
    gp.dX<<c<<
    gp.dY<<c<<
    gp.dI<<c<<
    gp.mu<<c
    ;
}


/** Train GP with observations drawn from itself at points uniformly distributed around c
 * optional center c defaults to ARR(0,0,0)
 * optional size defaults to 1. (was previously  sqrt(((GaussKernelParams*)gp.kernelP)->widthVar)
 * optional number of points to sample defaults to 5 (decreasing it saves a bit of runtime)
 */
void
randomGP_on_random_points(GaussianProcess &gp, const arr &c, double size, uint np){

  // generate some data points
  arr pointsA(np,3), point;
  for(uint iDP=0; iDP<pointsA.d0; ++iDP){
    for(uint iDim=0; iDim<pointsA.d1; ++iDim){
      pointsA(iDP,iDim) = rnd.uni(-size/2, size/2) + c(iDim);
    }
  }
  randomFunction(gp,pointsA,false);
}
