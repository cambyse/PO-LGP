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
  Access_typed<SurfaceModelObject> surfaceModelObject;

  Access_typed<arr> pathStart;
  Access_typed<arr> pathTarget;
  Access_typed<arr> shortestPath;
  Access_typed<arr> realTargetOnSurface;

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
  Access_typed<SurfaceModelObject> surfaceModelObject;
  Access_typed<double> surfaceSimilarityMeasure;
  Access_typed<mlr::Mesh> trueSurfaceMesh;

  SurfaceSimilarityModule();

  void open() {}
  void step();
  void close() {}
};

struct SurfaceVisualizationModule : Thread {
  Roopi& roopi;
  Access_typed<GaussianProcessOptimized> gpSurface;
  Access_typed<mlr::Mesh> trueSurfaceMesh;
  Access_typed<SurfaceModelObject> surfaceModelObject;
  Access_typed<double> surfaceSimilarityMeasure;

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

  Access_typed<GaussianProcessOptimized> gpSurface;
  GaussianProcessOptimized gpCopy;

  Access_typed<SurfaceModelObject> surfaceModelObject;
  Access_typed<arr> gradReference;

  Access_typed<arr> shortestPath;

  OrsPoseViewer* viewer;

  PathAndGradientVisualizationModule(Roopi& roopi);

  void open();
  void step();
  void close() {}
};

#endif // SURFACEVISUALISATIONMODULE_H
