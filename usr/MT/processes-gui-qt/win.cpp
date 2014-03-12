#define MT_IMPLEMENT_TEMPLATES
#include <biros/biros_internal.h>
#include <QtGui/QTreeWidget>

void addVariablesToTree(QTreeWidget *tree){
  uint i,j;
  Variable *v;
  _Variable_field_info_base *vi;
  global.readAccess(NULL);
  for_list(Type,  v,  global.variables){
    QStringList list;
    list.append(v->name);
    list.append(STRING("id=" <<v->id <<" type=" <<typeid(*v).name() <<" state=" <<v->lockState()));
    QTreeWidgetItem *var=new QTreeWidgetItem(list);
    tree->addTopLevelItem( var );
    for_list(Type,  vi,  v->fields){

      MT::String value;
      vi->write_value(value);
      QStringList list;
      list.append(vi->name);
      //list.append(vi->p);
      list.append(vi->type().p);  //value.p;
      var->addChild( new QTreeWidgetItem(list) );
    }
  }
  global.deAccess(NULL);
}
