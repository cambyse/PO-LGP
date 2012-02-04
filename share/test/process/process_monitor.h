#include <MT/process_internal.h>

#define BIR_VARIABLE \
static int bir_typeId; \
static MT::Array<_Variable_field_info_base*> bir_fields;

#define BIR_FIELD(type, name) \
  type name; \
  inline void set_##name(const type& _x, Process *p){ \
    writeAccess(p);  name=_x;  deAccess(p); } \
  inline void get_##name(type& _x, Process *p){ \
    readAccess(p);   _x=name;  deAccess(p); } \
  inline type get_##name(Process *p){ \
    type _x; readAccess(p); _x=name; deAccess(p);  return _x;  } \
  inline void reg_##name(){ \
    if(bir_typeId==-1) fields.append(new _Variable_field_info<type>(&name,#name)); }


#define TEXTTIME(dt) dt<<'|'<<dt##Mean <<'|' <<dt##Max

void dumpInfo(){
  cout <<" +++ VARIABLES +++" <<endl;
  uint i, j;
  Variable *v;
  Process *p;
  _Variable_field_info_base *vi;
  global.readAccess(NULL);
  for_list(i, v, global.variables){
    cout <<"Variable " <<v->id <<'_' <<v->name <<" lock-state=" <<v->lockState();
    if(v->fields.N){
      cout <<'{' <<endl;
      for_list(j, vi, v->fields){
        cout <<"   field " <<j <<' ' <<vi->name <<' ' <<vi->p <<" value=";
        vi->write_value(cout);
        cout <<endl;
      }
      cout <<"\n}";
    }
    //<<" {" <<endl;
    cout <<endl;
  }
  cout <<endl;
  cout <<" +++ PROCESSES +++" <<endl;
  for_list(i, p, global.processes){
    cout <<"Process " <<p->name <<" (";
    for_list(j, v, p->V){
      if(j) cout <<',';
      cout <<v->id <<'_' <<v->name;
    }
    cout <<") {" <<endl;
    cout
      <<" tid=" <<p->s->tid
      <<" priority=" <<p->s->threadPriority
      <<" steps=" <<p->s->timer.steps
      <<" cycleDt=" <<TEXTTIME(p->s->timer.cyclDt)
      <<" busyDt=" <<TEXTTIME(p->s->timer.busyDt)
      <<" state=";
    int state=p->s->threadCondition.state;
    if(state>0) cout <<state; else switch(state){
        case tsOPEN:    cout <<"open";   break;
        case tsCLOSE:   cout <<"close";  break;
        case tsLOOPING: cout <<"loop";   break;
        case tsBEATING: cout <<"beat";   break;
        case tsSYNCLOOPING: cout <<"sync";   break;
        case tsIDLE:    cout <<"idle";   break;
        default: cout <<"undefined:";
      }
    cout <<"\n}" <<endl;
  }
  global.deAccess(NULL);
}
