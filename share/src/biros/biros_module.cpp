#include "biros_module.h"

void Module_Process::open(){
#if 0
  mod = new OpencvCamera;
  GenericVariable *v = biros().getOrCreateVariable<Variable>("cameraOutputRgb", this);
  v->data = mod->cameraOutputRgb_access.createOwnData();
#else
  Item *modReg = registry().getItem("Decl_Module", STRING(strlen(name)<<name)); //OpencvCamera::staticRegistrator.reg;
  if(!modReg) HALT("could not find Decl_Module" <<name)
      mod = (Module*)modReg->value<TypeInfo>()->newInstance();

  //create the variables
  for_list_(Access, var, mod->accesses){
    Variable *v = biros().getOrCreateVariable<Variable>(var->name, this);
    var->process = this;
    var->guard = v;
    if(!v->data) v->data = var->createOwnData();
    else var->setData(v->data);
  }
  //mod->cameraOutputRgb_access.var = &var->x;
#endif
}
