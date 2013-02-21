#include "biros_module.h"

void Module_Process::open(){
#if 0
  mod = new OpencvCamera;
  GenericVariable *v = biros().getOrCreateVariable<GenericVariable>("cameraOutputRgb", this);
  v->data = mod->cameraOutputRgb_access.createOwnData();
#else
  Item *modReg = registry().getItem("moduledecl", STRING(strlen(name)<<name)); //OpencvCamera::staticRegistrator.regItem;
  if(!modReg) HALT("could not find moduledcl" <<name)
      mod = (Module*)modReg->value<KeyValueGraph>()->getItem("type")->value<TypeInfo>()->newInstance();

  //create the variables
  for_list_(VariableAccess, var, mod->accesses){
    GenericVariable *v = biros().getOrCreateVariable<GenericVariable>(var->name, this);
    var->process = this;
    var->guard = v;
    if(!v->data) v->data = var->createOwnData();
    else var->setData(v->data);
  }
  //mod->cameraOutputRgb_access.var = &var->x;
#endif
}
