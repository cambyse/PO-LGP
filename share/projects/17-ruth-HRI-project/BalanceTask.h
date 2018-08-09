#ifndef BALANCETASK_H
#define BALANCETASK_H

#include "HRI_task.h"
#include "HRI_state.h"
#include <vector>
#include <string>

class BalanceTask : public HRITask
{
public:
  BalanceTask(int m);
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
  bool performActionAutonomous();
  bool performActionAutonomousPlus();
  void performActionRequest();
  void performActionCommands();
  void performActionRobotCommands();
  void waitForObjectPlaced(HRIObject::Color c1, HRIObject::Color c2, HRIObject::Size s1, HRIObject::Size s2);
  void waitForObjectPlaced(HRIObject::Color c1, HRIObject::Size s1, const char* targetName);
  void requestObject(HRIObject* obj);
  void humanPlaceObject(OnTableState& s, HRIObject::Color hobjC, HRIObject::Size hobjS);
  bool placeObj(HRIObject* obj);
  bool placeObjs(HRIObject* obj, HRIObject* obj2);
  bool placeObjWait(HRIObject* obj);
  bool isReachable(HRIObject* obj);
  bool isPlausible(OnTableState& s);
  int getStackState(OnTableState& s);
  void log(std::string s);
  std::vector<HRIObject*> nextActions(int , OnTableState&);
  std::string getTargetID(std::string tpos);
};

#endif // BALANCETASK_H
