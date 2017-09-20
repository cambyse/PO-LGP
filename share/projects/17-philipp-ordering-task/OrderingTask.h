#ifndef ORDERINGTASK_H
#define ORDERINGTASK_H

#include "../17-philipp-HRI-task/HRI_task.h"
#include "../17-philipp-HRI-task/HRI_state.h"
#include <vector>
#include <string>

class OrderingTask : public HRITask {
public:
  OrderingTask(int m = 0);
  int mode;
protected:
  void performAction();
  void performActionProactive();
  void performActionAutonomous();
  void performActionRequest();
  void performActionCommands();
  bool placeObj(HRIObject* obj);
  bool isReachable(HRIObject* obj);
  bool isPlausible(OnTableState& s);
  void log(std::string s);
  std::vector<HRIObject*> nextActions(OnTableState&);
  std::string getTargetID(std::string tpos);
};

#endif
