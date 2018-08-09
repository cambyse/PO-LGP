#ifndef BASICTASK_H
#define BASICTASK_H

#include "HRI_task.h"
#include "HRI_state.h"
#include <vector>
#include <string>

class BasicTask : public HRITask
{
public:
  BasicTask(int m);
  int mode;
  int started;
protected:
  void performAction();
  void performActionCommands();
  bool placeObj(HRIObject* obj, int location);
  bool isReachable(HRIObject* obj);
  bool isPlausible(OnTableState& s);
  void log(std::string s);
  std::vector<HRIObject*> nextActions(OnTableState&);
  std::string getTargetID(std::string tpos);
};

#endif // BASICTASK_H
