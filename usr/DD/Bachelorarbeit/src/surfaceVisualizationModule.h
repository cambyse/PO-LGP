#ifndef SURFACEVISUALISATIONMODULE_H
#define SURFACEVISUALISATIONMODULE_H

#include <Core/array.h>
#include <Core/thread.h>
#include <Kin/kinViewer.h>
#include <Algo/gaussianProcess.h>
#include <Algo/gpOp.h>
#include <Geo/mesh.h>
#include <Roopi/roopi.h>

#include "surfaceModelModule.h"

struct SurfaceGeodesicPathModule : Thread {
  Access<SurfaceModelObject> surfaceModelObject;

  Access<arr> pathStart;
  Access<arr> pathTarget;
  Access<arr> shortestPath;
  Access<arr> realTargetOnSurface;

  SurfaceGeodesicPathModule();

  arr getRealTargetOnSurface();
  arr getPath();
  void setStart(arr start);
  void setTarget(arr target);

  void open() {}
  void step();
  void close() {}
};

struct SurfaceSimilarityModule : Thread {
  Access<SurfaceModelObject> surfaceModelObject;
  Access<double> surfaceSimilarityMeasure;
  Access<mlr::Mesh> trueSurfaceMesh;

  SurfaceSimilarityModule();

  void open() {}
  void step();
  void close() {}
};

struct SurfaceVisualizationModule : Thread {
  Roopi& roopi;
  Access<GaussianProcessOptimized> gpSurface;
  Access<mlr::Mesh> trueSurfaceMesh;
  Access<SurfaceModelObject> surfaceModelObject;
  Access<double> surfaceSimilarityMeasure;

  SurfaceSimilarityModule surfaceSimilarityModule;

  mlr::Mesh me;

  OpenGL gl;

  uint numberOfIts;

  SurfaceVisualizationModule(Roopi& roopi);

  void open();
  void step();
  void close();
};

struct PathAndGradientVisualizationModule : Thread {
  Roopi& roopi;

  Access<GaussianProcessOptimized> gpSurface;
  GaussianProcessOptimized gpCopy;

  Access<SurfaceModelObject> surfaceModelObject;
  Access<arr> gradReference;

  Access<arr> shortestPath;

  OrsPoseViewer* viewer;

  PathAndGradientVisualizationModule(Roopi& roopi);

  void open();
  void step();
  void close() {}
};

#endif // SURFACEVISUALISATIONMODULE_H
