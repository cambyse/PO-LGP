#include "registry.h"

//===========================================================================
//
// global singleton TypeRegistrationSpace
//

Singleton<KeyValueGraph> single_registry;

KeyValueGraph& registry(){
  return single_registry.obj();
}
