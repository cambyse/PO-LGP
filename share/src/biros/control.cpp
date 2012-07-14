#include "control.h"

void b::dump(){
  birosInfo.dump();
}

ViewInfoL b::getViews(ViewInfo::ViewType viewType, const char* appliesOn_sysType){
  uint i;
  ViewInfo *vi;
  ViewInfoL vis;
  for_list(i,vi,birosInfo.views){
    if(vi->type==viewType && (!strcmp(vi->appliesOn_sysType,"ALL") || !strcmp(vi->appliesOn_sysType, appliesOn_sysType)))
      vis.append(vi);
  }
  return vis;
}

ViewInfo* b::getView(const char *name){
  return listFindByName(birosInfo.views, name);
}


View* b::newView(Process& proc, ViewInfo *vi){
  if(!vi){
    ViewInfoL vis=getViews(ViewInfo::processVT, typeid(proc).name());
    if(!vis.N){
      MT_MSG("No View for Variable sysType '" <<typeid(proc).name() <<"' found");
      return NULL;
    }
    vi = vis(0);
  }
  cout
    <<"Creating new view '" <<vi->name <<"' for process '" <<proc.name <<"'" <<endl;
  View *v = vi->newInstance();
  v->proc = &proc;
  return v;
}

View* b::newView(Parameter& param, ViewInfo *vi){
  if(!vi){
    ViewInfoL vis=getViews(ViewInfo::parameterVT, param.typeName());
    if(!vis.N){
      MT_MSG("No View for Variable sysType '" <<param.typeName() <<"' found");
      return NULL;
    }
    vi = vis(0);
  }
  cout
    <<"Creating new view '" <<vi->name <<"' for parameter '" <<param.name <<"'" <<endl;
  View *v = vi->newInstance();
  v->param = &param;
  return v;
}

View* b::newView(Variable& var, ViewInfo *vi){
  if(!vi){
    ViewInfoL vis=getViews(ViewInfo::variableVT, typeid(var).name());
    if(!vis.N){
      MT_MSG("No View for Variable sysType '" <<typeid(var).name() <<"' found");
      return NULL;
    }
    vi = vis(0);
  }
  cout
    <<"Creating new view '" <<vi->name <<"' for variable '" <<var.name <<"'" <<endl;
  View *v = vi->newInstance();
  v->var = &var;
  return v;
}

View* b::newView(FieldInfo& field, ViewInfo *vi){
  if(!vi){
    ViewInfoL vis=getViews(ViewInfo::fieldVT, field.sysType);
    if(!vis.N){
      MT_MSG("No View for field sysType '" <<field.sysType <<"' found");
      return NULL;
    }
    vi = vis(0);
  }
  cout
    <<"Creating new view '" <<vi->name <<"' for field '" <<field.name
    <<"' (type '" <<field.sysType <<"') of Variable '" <<field.var->name <<"'" <<endl;
  View *v = vi->newInstance();
  v->field = &field;
  return v;
}



