/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#include "registry.h"

//===========================================================================
//
// global singleton TypeRegistrationSpace
//

Singleton<Graph> registry;

struct RegistryInitializer{
  Mutex lock;
  RegistryInitializer(){
    int n;
    for(n=1; n<mlr::argc; n++){
      if(mlr::argv[n][0]=='-'){
        mlr::String key(mlr::argv[n]+1);
        if(n+1<mlr::argc && mlr::argv[n+1][0]!='-'){
          mlr::String value;
          value <<'=' <<mlr::argv[n+1];
          registry().readNode(value, false, false, key);
          n++;
        }else{
          registry().newNode<bool>({key}, {}, true);
        }
      }else{
        MLR_MSG("non-parsed cmd line argument:" <<mlr::argv[n]);
      }
    }

    mlr::String cfgFileName="MT.cfg";
    if(registry()["cfg"]) cfgFileName = registry().get<mlr::String>("cfg");
    LOG(3) <<"opening config file '" <<cfgFileName <<"'";
    FILE(cfgFileName) >>registry();
  }
  ~RegistryInitializer(){
  }
};

Singleton<RegistryInitializer> registryInitializer;

void initRegistry(){
  registryInitializer();
}

bool getParameterFromGraph(const std::type_info& type, void* data, const char* key){
  registryInitializer();
  Node *n = registry().findNodeOfType(type, {key});
  if(n){
    n->copyValueInto(data);
    return true;
  }else{
    n = registry().findNode({key});
    if(n && n->isOfType<double>()){
      if(type==typeid(int)){ *((int*)data) = (int)n->get<double>(); return true; }
      if(type==typeid(uint)){ *((uint*)data) = (uint)n->get<double>(); return true; }
      if(type==typeid(bool)){ *((bool*)data) = (bool)n->get<double>(); return true; }
    }
  }
  return false;
}
