#ifndef TEST_PLACE_TASK_H
#define TEST_PLACE_TASK_H

#include "HRI_task.h"
#include "lexicon_TTSR.h"

// test phase 2 (place object): #include "test_place_task.h" and test_place_task task;

class test_place_task : public HRITask
{
protected:
  //flag for task type
  int learning_type;
  //count for phrases
  int current_phrase;
  lexicon TTSRlexiconExemplar;
  lexicon TTSRlexiconGrid;

  void performAction();
  bool isReachable(HRIObject* obj);
  LeftOrRight sideOfTable(HRIObject* obj);

public:
  test_place_task();
};

#endif // TEST_PLACE_TASK_H
