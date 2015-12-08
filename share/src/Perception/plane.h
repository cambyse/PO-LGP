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
  arr y;
  ors::Quaternion r;

  CostFct_PlanePoints(const arr& n, const arr& m, const arr& X, const arr& transform);

  double f();
  arr df_transform();

  ScalarFunction f_transform();
};
