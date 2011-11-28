#include "opengl.h"
#include "ors.h"

struct sOrsSceneGui;

struct OrsSceneGui{
  sOrsSceneGui *s;
  
  OrsSceneGui(ors::Graph& ors, OpenGL *gl=NULL);
  
  void edit();
  
};

