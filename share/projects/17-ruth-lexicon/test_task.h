#ifndef TEST_TASK_H
#define TEST_TASK_H

#include "HRI_task.h"
#include "lexicon_TTSR.h"

// test phase 1 (point to objects): #include "test_task.h" and test_task task;

class test_task : public HRITask {

protected:
  //flag for task type
  int learning_type;
  //count for phrases
  int current_phrase;
  lexicon TTSRlexiconExemplar;
  lexicon TTSRlexiconGrid;

  void performAction();
  bool isReachable(HRIObject* obj);
  std::vector<HRIObject*> reachableObjects(OnTableState& s);
  std::vector<HRIObject*> allObjects(OnTableState& s);
  LeftOrRight sideOfTable(HRIObject* obj);



public:
  test_task();
};

#endif // TEST_TASK_H
