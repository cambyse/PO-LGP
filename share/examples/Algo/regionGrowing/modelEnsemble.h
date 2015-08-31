#pragma once

#include "dataNeighbored.h"
#include "minEigModel.h"
#include <Gui/opengl.h>

struct ModelEnsemble :OpenGL::GLDrawer{
  MT::Array<MinEigModel*> models;

  arr vert;

  ModelEnsemble();

  bool addNewRegionGrowingModel(DataNeighbored& D);
  void reestimateVert();
  void reoptimizeModels(DataNeighbored& D);

  void glDraw(OpenGL &);

  void report(ostream& os=cout);
};
