#ifndef BRIDGETASK_H
#define BRIDGETASK_H

#include "HRI_task.h"
#include "HRI_state.h"
#include <vector>
#include <string>

class BridgeTask : public HRITask {
public:
  BridgeTask(int m = 0);
  int mode;
  int currentState;
  int started;
protected:
  void performAction();
  void performActionProactive();
  void performActionAutonomous();
  void performActionAutonomousPlus();
  void performActionRequest();
  void performActionCommands();
  void performActionReactive();
  void performActionRobotCommands();
  void humanPlaceObject(OnTableState& s, HRIObject::Color hobjC, HRIObject::Size hobjS);
  bool placeObj(HRIObject* obj);
  int getBridgeState(OnTableState& s);
  bool isReachable(HRIObject* obj);
  bool isPlausible(OnTableState& s);
  void log(std::string s);

  std::vector<HRIObject*> nextActions(int , OnTableState&);
  std::vector<HRIObject*> futureActions(int bridgestate, OnTableState& s, HRIObject::Color objC, HRIObject::Size objS);
//  std::vector<HRIObject*> futureActions(int , OnTableState&);
  std::time_t p_time;
  int p_state;
};

#endif
