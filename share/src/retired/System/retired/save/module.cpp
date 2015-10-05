#include <Core/module.h>
#include "engine.h"

//===========================================================================
//
// Variable
//

Variable::Variable(const char *_name):s(NULL), data(NULL), name(_name), revision(0), reg(NULL) {
  s = new sVariable();
  s->listeners.memMove=true;
  //MT logValues = false;
  //MT dbDrivenReplay = false;
  //MT pthread_mutex_init(&replay_mutex, NULL);
//  if(&(biros()) != NULL) { //-> birosInfo itself will not be registered!
//    biros().writeAccess(NULL);
//    biros().variables.memMove = true;
//    biros().variables.append(this);
//    biros().deAccess(NULL);
//  }
}

Variable::~Variable() {
//  if(this != global_birosInfo) { //-> birosInfo itself will not be de-registered!
//    biros().writeAccess(NULL);
//    biros().variables.removeValue(this);
//    biros().deAccess(NULL);
//  }
  //for (uint i=0; i<s->fields.N; i++) delete s->fields(i);

  //MT pthread_mutex_destroy(&replay_mutex);

  delete s;
}

int Variable::readAccess(Module *p) {
  engine().acc->queryReadAccess(this, p);
  rwlock.readLock();
  engine().acc->logReadAccess(this, p);
  return revision.getValue();
}

int Variable::writeAccess(Module *p) {
  engine().acc->queryWriteAccess(this, p);
  rwlock.writeLock();
  int r = revision.incrementValue();
  engine().acc->logWriteAccess(this, p);
  uint i; Module *l;
  for_list(Type,  l,  s->listeners) if(l!=p) engine().step(*l);
  return r;
}

int Variable::deAccess(Module *p) {
  if(rwlock.state == -1) { //log a revision after write access
    //MT logService.logRevision(this);
    //MT logService.setValueIfDbDriven(this); //this should be done within queryREADAccess, no?!
    engine().acc->logWriteDeAccess(this,p);
  } else {
    engine().acc->logReadDeAccess(this,p);
  }
  int rev=revision.getValue();
  rwlock.unlock();
  return rev;
}

void Variable::waitForNextRevision(){
  revision.waitForSignal();
}

int Variable::waitForRevisionGreaterThan(int rev) {
  revision.lock();
  revision.waitForValueGreaterThan(rev, true);
  rev = revision.value;
  revision.unlock();
  return rev;
}

FieldRegistration& Variable::get_field(uint i) const{
  return *s->fields(i);
}

void sVariable::serializeToString(MT::String &string) const {
#if 0
  string.clear();
  MT::String field_string;
  field_string.clear();

  // go through fields
  for (uint i=0; i < fields.N; i++) {

    fields(i)->writeValue(field_string);

    // replace every occurence of "\" by "\\"
    for (uint j=0; j < field_string.N; j++) {
      char c = field_string(j);
      if('\\' == c) string << '\\';
      string << c;
    }

    // add seperator after field
    string << "\\,";
  }
#endif
  NIY
}

void sVariable::deSerializeFromString(const MT::String &string) {
#if 0
  MT::String string_copy(string), field_string;
  field_string.clear();
  uint j = 0;
  for (uint i=0; i< fields.N; i++) {
    // get field strings from string (seperated by "\\,")
    bool escaped = false; // true if previous char was '\\'
    while (j < string_copy.N) {
      char c = string_copy(j++);
      if('\\' == c) {
        escaped = true;
      } else {
        if(escaped) {
          if(',' == c) {
            break;
          }
        }
        escaped = false;
        field_string << c;
      }
    }
    fields(i)->readValue(field_string);
  }
#endif
  NIY
}
