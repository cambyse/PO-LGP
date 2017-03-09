#include "act.h"

typedef std::function<int()> Script;

struct Act_Script : Act{
  struct sAct_Script *s;
  Act_Script(Roopi *r, const Script& S);
  ~Act_Script();
};
