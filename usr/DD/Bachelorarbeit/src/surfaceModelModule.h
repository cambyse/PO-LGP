#ifndef SURFACEMODELMODULE_H
#define SURFACEMODELMODULE_H

#include <Core/thread.h>
#include <Roopi/roopi.h>
#include <Algo/gpOp.h>
#include <Roopi/loggingModule.h>

struct SurfaceModelObject {
  GaussianProcessOptimized gp;
  ors::Mesh mesh;
  arr varianceOnSurface; //same ordering as mesh.V!!!
  arr varianceGradientsOnSurface; //same ordering as mesh.V!!
  arr gradientsOnSurface; //same ordering as mesh.V!!
  arr gaussianCurvatureOnSurface; //same ordering as mesh.V!!

  SurfaceModelObject();

  void calculateSurface(uint resolution = 20);
  void calculateVariance();
  void calculateGradientsOnSurface();
  void calculateVarianceGradientOnSurface();
  double calculateVarianceMeasure() const;
  double calculateMeshDistance(const ors::Mesh& otherMesh) const;
  void plotVarianceOnSurface();

  double calculateGaussianCurvature(const arr& pos);
  void calculateGaussianCurvatureOnSurface();

  /// (approximate) shortest paths between two points unsing dijkstra
  arr computeGeodesicEuklideanPathOnSurface(const arr& startPos, const arr& targetPos, arr& startOnSurface = NoArr, arr& targetOnSurface = NoArr);

  //non member functions!!! Used for performance issues in multithread environments
  static arr computeGeodesicPathOnSurface(const arr& startPos, const arr& targetPos, ors::Mesh mesh, std::function<double(const arr&, const arr&, uint, uint)> distanceFunction, arr& startOnSurface = NoArr, arr& targetOnSurface = NoArr);
  static arr computeGeodesicEuklideanPathOnSurface(const arr& startPos, const arr& targetPos, ors::Mesh mesh, arr& startOnSurface = NoArr, arr& targetOnSurface = NoArr);
  static arr computeGeodesicVariancePathOnSurface(const arr& startPos, const arr& targetPos, ors::Mesh mesh, const arr& varianceSurface, arr& startOnSurface = NoArr, arr& targetOnSurface = NoArr);

  arr smoothGeodesicPathWithKOMO(const arr& dijkstraPath);
};


struct SurfaceModelModule : Thread {
  Roopi& roopi;

  Access_typed<GaussianProcessOptimized> gpSurface;

  arr X, Y;

  SetOfDataFiles log;

  TaskMap_Default* posMap;

  SurfaceModelModule(Roopi& roopi);
  ~SurfaceModelModule();

  void open();
  void step();
  void close();
};

#endif // SURFACEMODELMODULE_H
