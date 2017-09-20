#ifndef LEARN_TASK_H
#define LEARN_TASK_H

#include "HRI_task.h"
#include "lexicon_TTSR.h"
//#include <vector>


class learn_task : public HRITask {

protected:
  //flag for learning type
  int learning_type;
  //count for phrases learnt in exemplar learning
  int current_phrase;
  //count for positions in grid for grid learning
  int current_position;
  double dataExemplar[5][2];
  bool dataGrid [25][5];
  lexicon TTSRlexiconExemplar;
  lexicon TTSRlexiconGrid;

  void performAction();
  bool isReachable(HRIObject* obj);

public:
  learn_task();
};

#endif // LEARN_TASK_H
