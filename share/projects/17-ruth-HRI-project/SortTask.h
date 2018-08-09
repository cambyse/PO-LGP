#ifndef SORTTASK_H
#define SORTTASK_H

#include "HRI_task.h"
#include "HRI_state.h"
#include <vector>
#include <string>

class SortTask : public HRITask
{
public:
  SortTask(int m);
  int mode;
  int currentState;
  int started;
//  int phase;
//  int current_position;
//  int pick;
//  double tableSize[3];
//  double objectSize;
protected:
  void performAction();
  void performActionProactive();
  void performActionAutonomous();
  void performActionAutonomousPlus();
  void performActionRequest();
  void performActionCommands();
  void performActionRobotCommands();
  bool placeObj(HRIObject* obj);
  void humanPlaceObject(OnTableState& s, HRIObject::Color hobjC, HRIObject::Size hobjS);
  bool isReachable(HRIObject* obj);
  bool isPlausible(OnTableState& s);
  void log(std::string s);
  std::vector<HRIObject*> nextActions(OnTableState&);
  std::string getTargetID(std::string tpos);
};

#endif // SORTTASK_H
