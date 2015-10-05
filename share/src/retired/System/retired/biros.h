/*  ---------------------------------------------------------------------
    Copyright 2013 Marc Toussaint
    email: mtoussai@cs.tu-berlin.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */

#ifndef MT_biros_h
#define MT_biros_h

#include <Core/array.h>
#include <Core/util.h>
#include <Core/module.h>
#include "engine.h"
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
struct Parameter;
typedef MT::Array<Variable*> VariableL;
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
  Module_ThreadL dependers;
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
  Module_ThreadL processes;
  ParameterL parameters;

  /// @name c'tor/d'tor
  Biros();
  ~Biros();

  /// @name access existing processes, variables and parameters.
  Module_Thread *getProcessFromPID();
  template<class T> T* getVariable(const char* name, Module_Thread *p, bool required = false);
  template<class T> T* getOrCreateVariable(const char* name, Module_Thread *p);
  template<class T> void getVariable(T*& v, const char* name, Module_Thread *p, bool required = false);
  template<class T> T* getProcess (const char* name, Module_Thread *p, bool required = false);
  template<class T> T getParameter(const char *name, Module_Thread *p=NULL);
  template<class T> T getParameter(const char *name, const T& _default, Module_Thread *p=NULL);
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
  Module_Thread *p;         ///< pointer to the Process that might want to access the Variable
  int last_revision; ///< last revision of a read/write access

  WorkingCopy() { p=NULL; var=NULL; last_revision = 0;  }
  T& operator()() { return copy; }

  void init(T *_v, Module_Thread *_p) {
    p=_p;
    var=_v;
    var->readAccess(p);
    copy = *var;
    last_revision = var->revision.getValue();
    var->deAccess(p);
    copy.name <<"_WorkingCopy_" <<(p?p->name:STRING("GLOBAL"));
  }
  void init(const char* var_name, Module_Thread *_p) {
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
void writeInfo(ostream& os, Module_Thread& p, bool brief, char nl='\n');
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
