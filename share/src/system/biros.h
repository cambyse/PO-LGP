#ifndef MT_biros_h
#define MT_biros_h

#include <Core/array.h>
#include <MT/util.h>
#include "module.h"
#include "engine_internal.h"

/**
 * @file
 * @ingroup group_biros
 */
/**
 * @addtogroup group_biros
 * @{
 */
/* NOTES:

  -- derive all biros objects from bObject -> specific lists become generic lists
  -- insideOut -> one view with all bObjects

  -- flow chart: x-axis=time, rows=processes&variables, transactions=vertical arrows, on each row: bars indicate duration of step of process / duration of revision of variable; arrow onto variable -> change of revision/color/bar of that variable; each process bar is labeled by step number and process name; each variable bar by revision and variable name.
  */

//===========================================================================
//
// forward declarations
//

struct Variable;
struct Process;
struct Parameter;
typedef MT::Array<Variable*> VariableL;
typedef MT::Array<Process*> ProcessL;
typedef MT::Array<Parameter*> ParameterL;





//===========================================================================
/**
 * Helper to register FIELDs in a Variable.
 */
struct FieldRegistration {
  const char* name;
  const char* userType;
  const char* sysType;
  void *p;           /// pointer to object
  Variable *var;     /// pointer to the containing variable
  virtual void writeValue(ostream& os) const = 0;
  virtual void readValue(istream& os) const { NIY; }
};

/**
 * Helper to register FIELDs in a Variable.
 */
template<class T>
struct FieldRegistration_typed:FieldRegistration {
  FieldRegistration_typed(T *_p, Variable *_var, const char* _name, const char* _userType) {
    name = _name;
    userType = _userType;
    sysType = typeid(T).name();
    p = _p;
    var = _var;
  }
  void writeValue(ostream& os) const { os <<*((T*)p); }
  //void readValue(istream& is) const { is >>*((T*)p); }
};

void registerField(Variable *v, FieldRegistration* f);

/**
 * \def FIELD(type, name)
 * A macro to create a member of a Variable.
 *
 * Creates setters/getters and a register function.
 */
#ifndef FIELD
#define FIELD(type, name) \
  type name; \
  inline int set_##name(const type& _x, Module *p){ \
    writeAccess(p);  name=(type&)_x;  return deAccess(p); } \
  inline int get_##name(type& _x, Module *p){ \
    readAccess(p);   _x=name;  return deAccess(p); } \
  inline type get_##name(Process *p){ \
    type _x; readAccess(p); _x=name; deAccess(p);  return _x;  } \
  inline void reg_##name(){ \
    registerField(this, new FieldRegistration_typed<type>(&name,this,#name,#type)); }
#endif

//===========================================================================
/**
 * Parameters similar to MT:getParameter.
 *
 * You can access parameters with `biros().getParameter()' functions.
 *
 * Parameterhs are usually not created directly by the user,
 * they are created automatically by a call of `getParameter')
 */
struct Parameter {
  void *pvalue;
  const char* name;
  ModuleL dependers;
  Parameter();
  virtual void writeValue(ostream& os) const = 0;
  virtual const char* typeName() const = 0;
};


/**
 * @brief Like Parameter but with type T.
 *
 * @tparam T the type of the parameter.
 */
template<class T>
struct Parameter_typed:Parameter {
  T value;
  Parameter_typed(const char* _name, const T& _default):Parameter() {
    name = _name;
    pvalue = &value;
    if(&_default) MT::getParameter<T>(value, name, _default);
    else          MT::getParameter<T>(value, name);
  }
  void writeValue(ostream& os) const { os <<value; }
  const char* typeName() const { return typeid(T).name(); }
};


//===========================================================================
/**
 * Macro to create a Processs.
 *
 * TODO This is actually only used in motion.h. Do we really need it?
 */
#define PROCESS(name)   \
  struct name:Process { \
    struct s##name *s;  \
    name();             \
    virtual ~name();    \
    void open();        \
    void step();        \
    void close();       \
  };


//===========================================================================
/**
 * Macro to easily acess a Variable.
 */
#define VAR(Type) \
  Type *_##Type;  _##Type = biros().getVariable<Type>(#Type, NULL);



//===========================================================================
/**
 * The class Biros allows basic access/step control and access to global system info.
 *
 * Biros reprents the graph of variables and processes.
 */
struct Biros:Variable {
  VariableL variables;
  ProcessL processes;
  ParameterL parameters;

  /// @name c'tor/d'tor
  Biros();
  ~Biros();

  /// @name access existing processes, variables and parameters.
  Process *getProcessFromPID();
  template<class T> T* getVariable(const char* name, Module *p, bool required = false);
  template<class T> T* getOrCreateVariable(const char* name, Module *p);
  template<class T> void getVariable(T*& v, const char* name, Module *p, bool required = false);
  template<class T> T* getProcess (const char* name, Module *p, bool required = false);
  template<class T> T getParameter(const char *name, Module *p=NULL);
  template<class T> T getParameter(const char *name, const T& _default, Module *p=NULL);
  template<class T> void setParameter(const char *name, T value);

  /// @name dump ALL available information
  void dump();

};

Biros& biros();


//===========================================================================
//
// WorkingCopy TODO: remove this!
//

template<class T>
struct WorkingCopy {
  T *var;             ///< pointer to the Variable (T must be derived from Variable)
  T copy;
  Process *p;         ///< pointer to the Process that might want to access the Variable
  int last_revision; ///< last revision of a read/write access

  WorkingCopy() { p=NULL; var=NULL; last_revision = 0;  }
  T& operator()() { return copy; }

  void init(T *_v, Process *_p) {
    p=_p;
    var=_v;
    var->readAccess(p);
    copy = *var;
    last_revision = var->revision.getValue();
    var->deAccess(p);
    copy.name <<"_WorkingCopy_" <<(p?p->module->name:STRING("GLOBAL"));
  }
  void init(const char* var_name, Process *_p) {
    T *_v = biros().getVariable<T>(var_name, _p);
    init(_v, _p);
  }
  bool needsUpdate() {
    return last_revision != var->get_revision();
  }
  void push() {
    if (var->get_revision()>last_revision) MT_MSG("Warning: push overwrites revision");
    var->writeAccess(p);
    NIY //never do this: you can't overwrite the members name, field, s, id, revision, etc!!
    *var = copy;
    last_revision = var->revision; //(was incremented already on writeAccess)
    var->deAccess(p);
  }
  void pull() {
    if (last_revision == var->revision.getValue()) return;
    var->readAccess(p);
    copy = *var;
    last_revision = var->revision.getValue();
    var->deAccess(p);
  }
};


//===========================================================================
/**
 * @name  Helpers to print out information
 * @{
 */
void writeInfo(ostream& os, Process& p, bool brief, char nl='\n');
void writeInfo(ostream& os, Variable& v, bool brief, char nl='\n');
void writeInfo(ostream& os, FieldRegistration& f, bool brief, char nl='\n');
void writeInfo(ostream& os, Parameter& pa, bool brief, char nl='\n');
/** @} */


#include "biros_t.cxx"

#ifdef  MT_IMPLEMENTATION
#  include "biros.cpp"
#endif

/** @} */
#endif
