#include "opengl.h"
#include "ors.h"

struct OrsSceneGui {
  struct sOrsSceneGui *s;
  
  OrsSceneGui(ors::Graph& ors, OpenGL *gl=NULL);
  
  void edit();
  
};

