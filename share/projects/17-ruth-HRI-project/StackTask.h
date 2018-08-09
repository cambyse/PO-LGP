#ifndef STACKTASK_H
#define STACKTASK_H

#include "HRI_task.h"
#include "HRI_state.h"
#include <vector>
#include <string>


class StackTask : public HRITask
{
public:
  StackTask(int m =0);
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
  int getStackState(OnTableState& s);
  void log(std::string s);
  std::vector<HRIObject*> nextActions(int , OnTableState&);
  std::vector<HRIObject*> futureActions(int , OnTableState&);
  std::string getTargetID(std::string tpos);
};

#endif // STACKTASK_H
