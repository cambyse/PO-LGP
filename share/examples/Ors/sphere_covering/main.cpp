#include <Optim/optimization.h>
#include <Optim/benchmarks.h>
#include <Ors/ors.h>
#include <Gui/opengl.h>

struct CoveringSpheresProblem:ConstrainedProblem {
  const arr& x;
  double p, alpha;
  uint s;
  CoveringSpheresProblem(const arr& x, uint s):x(x), p(3.), alpha(-10.), s(s) {}
  arr initialization(const arr& x){
    arr c(s, 3);
    for(uint j=0;j<s;j++) c[j] = x[rnd(x.d0)];

    arr r=zeros(s);
    for(uint i=0;i<x.d0;i++) for(uint j=0;j<s;j++){
      double d=length(x[i] - c[j]);
      if(2.*d>r(j)) r(j) = 2.*d;
    }

    return cat(c,r).reshape(dim_x());
  }

  virtual double fc(arr& df, arr& Hf, arr& g, arr& Jg, const arr& cr) {
    uint s=cr.N/4;
    arr c,r;
    c.referToSubRange(cr,0,3*s-1); c.reshape(s,3);
    r.referToSubRange(cr,3*s,-1); r.reshape(s);

    //f
    double fac = 1.;
    double fx = 0.;
    for(uint i=0;i<s;i++) fx += pow(fabs(r(i)), p);
    fx *= fac;
    if(&df){
      df=zeros(4*s);
      for(uint i=0;i<s;i++) df(3*s + i) = p * pow(fabs(r(i)), p-1.);
      df *= fac;
    }
    if(&Hf){
      Hf=zeros(4*s, 4*s);
      for(uint i=0;i<s;i++) Hf(3*s + i, 3*s + i) = p * (p-1.) * pow(fabs(r(i)), p-2.);
      Hf *= fac;
    }

    //g
    arr d_ij(x.d0, s), ed_ij(x.d0, s), sed_i(x.d0);
    if(&g || &Jg){
      for(uint i=0;i<x.d0;i++) for(uint j=0;j<s;j++)
        d_ij(i,j) = length(x[i] - c[j]) - r(j);
      ed_ij = ::exp(alpha*d_ij);
      sed_i = sum(ed_ij,1);
    }
    if(&g){
      g = sum((d_ij%ed_ij),1) / sed_i;
    }
    if(&Jg){
      arr Jg_dij = (ed_ij%(1.+alpha*(d_ij - repmat(g,1,s))));
      for(uint i=0;i<x.d0;i++) Jg_dij[i]() /= sed_i(i);

      arr Jg_cj(x.d0, s, 3);
      for(uint i=0;i<x.d0;i++) for(uint j=0;j<s;j++){
        Jg_cj[i][j] = Jg_dij(i,j) * (c[j]-x[i])/length(x[i]-c[j]);
      }

      arr Jg_rj = -Jg_dij;
      Jg_cj.reshape(x.d0,3*s);
      Jg.setBlockMatrix(Jg_cj, Jg_rj);
      Jg.reshape(x.d0,cr.d0);
    }
    return fx;
  }
  virtual uint dim_x(){ return 4*s;  }
  virtual uint dim_g(){ return x.d0; }
};

//==============================================================================

void TEST(CoveringSphere){
  uint n=1000, s=6;
  arr x(n,3); rndGauss(x);
  for(uint i=0;i<x.d0;i++) x(i,0) *=3.;
  for(uint i=0;i<x.d0;i++) x(i,2) +=.5;
//  x *= .1;

  CoveringSpheresProblem F(x,s);

  //-- initial x
  arr cr = F.initialization(x);
  rndGauss(cr, .01, true);

  cout <<"point = " <<x <<endl;
  cout <<"cr_init=" <<cr <<endl;
  checkAllGradients(F, cr, 1e-4);
  optConstrained(cr, NoArr, F);
  cout <<"cr_opt=" <<cr <<endl;


  ors::KinematicWorld G;
  arr c,r;
  c.referToSubRange(cr,0,3*s-1); c.reshape(s,3);
  r.referToSubRange(cr,3*s,-1); r.reshape(s);
  for(uint j=0;j<s;j++) if(r(j)>1e-3){
    ors::Shape *s = new ors::Shape(G, NoBody);
    s->type = ors::sphereST;
    s->X.pos.set(c[j].p);
    s->size[3] = r(j);
  }
  for(uint i=0;i<x.d0;i++){
    ors::Shape *s = new ors::Shape(G, NoBody);
    s->type = ors::sphereST;
    s->X.pos.set(x[i].p);
    s->size[3] = .1;
    s->color[0]=0.;
  }

//  orsDrawAlpha=.5;
  G.gl().watch();
}

//==============================================================================

int main(int argc,char** argv){
  MT::initCmdLine(argc,argv);


  testCoveringSphere();

  return 0;
}
