#include "registry.h"

//===========================================================================
//
// global singleton TypeRegistrationSpace
//

Singleton<MapGraph> single_registry;

MapGraph& registry(){
  return single_registry.obj();
}
