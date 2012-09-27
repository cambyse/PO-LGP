#include "control.h"

void b::dump(){
  birosInfo().dump();
}

//call getViews for a list of possible sysTypes
ViewInfoL b::getViews(const CharAL appliesOn_sysTypeL){
  uint i, j;
  const char* appliesOn_sysType;
  ViewInfo *vi;
  ViewInfoL vis;
  for_list_rev(i,vi,birosInfo().views){
    if(!strcmp(vi->appliesOn_sysType,"ALL"))
      vis.append(vi);
    else
    	for_list(j, appliesOn_sysType, appliesOn_sysTypeL)
				if(!strcmp(vi->appliesOn_sysType, appliesOn_sysType))
					vis.append(vi);
  }
  return vis;
}

ViewInfoL b::getViews(const char* appliesOn_sysType){
  return b::getViews(ARRAY(appliesOn_sysType));
}

ViewInfoL b::getViews(const char* appliesOn_sysType0, const char* appliesOn_sysType1){
	return b::getViews(ARRAY(appliesOn_sysType0, appliesOn_sysType1));
}

ViewInfo* b::getView(const char *name){
  return listFindByName(birosInfo().views, name);
}

View* b::newView(Process& proc, ViewInfo *vi, GtkWidget *container){
  if(!vi){
  	ViewInfoL vis=getViews(typeid(proc).name(), typeid(Process).name());
    if(!vis.N){
      MT_MSG("No View for process sysType '" <<typeid(proc).name() <<"' found");
      return NULL;
    }
    vi = vis(0);
  }
  cout
    <<"Creating new view '" <<vi->name <<"' for process '" <<proc.name <<"'" <<endl;
  View *v = vi->newInstance();
  v->object = &proc;
  v->gtkNew(container);
  return v;
}

View* b::newView(Parameter& param, ViewInfo *vi, GtkWidget *container){
  if(!vi){
  	ViewInfoL vis=getViews(param.typeName(), typeid(Parameter).name());
    if(!vis.N){
      MT_MSG("No View for paramater sysType '" <<param.typeName() <<"' found");
      return NULL;
    }
    vi = vis(0);
  }
  cout
    <<"Creating new view '" <<vi->name <<"' for parameter '" <<param.name <<"'" <<endl;
  View *v = vi->newInstance();
  v->object = &param;
  v->gtkNew(container);
  return v;
}

View* b::newView(Variable& var, ViewInfo *vi, GtkWidget *container){
  if(!vi){
  	ViewInfoL vis=getViews(typeid(var).name(), typeid(Variable).name());
    if(!vis.N){
      MT_MSG("No View for variable sysType '" <<typeid(var).name() <<"' found");
      return NULL;
    }
    vi = vis(0);
  }
  cout
    <<"Creating new view '" <<vi->name <<"' for variable '" <<var.name <<"'" <<endl;
  View *v = vi->newInstance();
  v->object = &var;
  v->gtkNew(container);
  return v;
}

View* b::newView(FieldInfo& field, ViewInfo *vi, GtkWidget *container){
  if(!vi){
  	ViewInfoL vis=getViews(field.sysType, typeid(FieldInfo).name());
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
  v->object = &field;
  v->gtkNew(container);
  return v;
}
