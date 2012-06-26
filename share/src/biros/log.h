#include "biros.h"

struct AccessTicket;
typedef MT::Array<AccessTicket*> AccessTicketL;

struct AccessTicket{
  Variable *var;
  Process *proc;
  enum AccessType{ read, write } type;
  uint revision;
  uint procStep;
};


