#include <Core/array.h>
#include <Geo/geo.h>

struct Plane{
  arr mean, normal;
  arr inlierPoints;
  arr borderPoints;
};

struct CostFct_PlanePoints{
  const arr& n;
  const arr& m;
  const arr& X;
  const arr& transform;
  arr R, y;
  ors::Quaternion quat;
  CostFct_PlanePoints(const arr& n, const arr& m, const arr& X, const arr& transform);
  double f(){ return sumOfSqr(y); }
  arr df_transform();

  ScalarFunction f_transform(){
    return [this](arr& g, arr& H, const arr& x) -> double {
      CostFct_PlanePoints fx(n,m,X,x);
      if(&g) g=fx.df_transform();
      return fx.f();
    };
  }

};
