#include "test_task.h"

// test phase 1 (point to objects): #include "test_task.h" and test_task task;
// note: should include 9 objects set out in a grid on the table in model.g

bool test_task::isReachable(HRIObject* obj) {
  return obj->pos.x<.8; //TODO: check in real world what is reachable
}

LeftOrRight test_task::sideOfTable(HRIObject* obj){
  LeftOrRight hand = LR_left;
  if (obj->pos.y<0){
    hand = LR_right;
  }
  return hand;
}

void test_task::performAction() {
  //  point to object referred to by phrase
  //  loop for all phrases
  if (learning_type==0){
    percept_nofilter();

    std::vector<HRIObject*> objs = test_task::allObjects(state);
    for (current_phrase=0; current_phrase<5; current_phrase++){
      int objIndex = TTSRlexiconExemplar.choose_object(R,current_phrase,objs);
      if (objIndex>=0){
        LeftOrRight hand = test_task::sideOfTable(objs[objIndex]);
        auto pointObj = R.pointBox(objs[objIndex]->id.c_str(), hand);
        R.wait(+pointObj);
        auto an = R.armsNeutral();
        R.wait({-an});
      }
      cout<<objIndex<<endl;
    }
      current_phrase=0;
      learning_type=1;
  } else if (learning_type==1){
    percept_nofilter();
    std::vector<HRIObject*> objs = test_task::reachableObjects(state);
    for (current_phrase=0; current_phrase<5; current_phrase++){
      int objIndex = TTSRlexiconGrid.choose_object(R,current_phrase,objs);
      if (objIndex>=0){
        LeftOrRight hand = test_task::sideOfTable(objs[objIndex]);
        auto pointObj = R.pointBox(objs[objIndex]->id.c_str(), hand);
        R.wait(+pointObj);
        auto an = R.armsNeutral();
        R.wait({-an});
      }
      cout<<objIndex<<endl;
    }
      current_phrase=0;
      learning_type=2;
  } else {
    cout << state << endl;
    R.wait(); // no specific task -> just wait for user input
  }
}

std::vector<HRIObject*> test_task::reachableObjects(OnTableState& s) {
  std::vector<HRIObject*> result;
  HRIObject* s1 = s.getObj("S1");
  HRIObject* s2 = s.getObj("S2");
  HRIObject* s3 = s.getObj("S3");
  HRIObject* s4 = s.getObj("S4");
  HRIObject* s5 = s.getObj("S5");
  HRIObject* s6 = s.getObj("S6");
  HRIObject* s7 = s.getObj("S7");
  HRIObject* s8 = s.getObj("S8");
  HRIObject* s9 = s.getObj("S9");
  if ((s1!=NULL)&&(isReachable(s1))){
    result.push_back(s1);
  }
  if ((s2!=NULL)&&(isReachable(s2))){
    result.push_back(s2);
  }
  if ((s3!=NULL)&&(isReachable(s3))){
    result.push_back(s3);
  }
  if ((s4!=NULL)&&(isReachable(s4))){
    result.push_back(s4);
  }
  if ((s5!=NULL)&&(isReachable(s5))){
    result.push_back(s5);
  }
  if ((s6!=NULL)&&(isReachable(s6))){
    result.push_back(s6);
  }
  if ((s7!=NULL)&&(isReachable(s7))){
    result.push_back(s7);
  }
  if ((s8!=NULL)&&(isReachable(s8))){
    result.push_back(s8);
  }
  if ((s9!=NULL)&&(isReachable(s9))){
    result.push_back(s9);
  }
  return result;
}

std::vector<HRIObject*> test_task::allObjects(OnTableState& s) {
  std::vector<HRIObject*> result;
  HRIObject* s1 = s.getObj("S1");
  HRIObject* s2 = s.getObj("S2");
  HRIObject* s3 = s.getObj("S3");
  HRIObject* s4 = s.getObj("S4");
  HRIObject* s5 = s.getObj("S5");
  HRIObject* s6 = s.getObj("S6");
  HRIObject* s7 = s.getObj("S7");
  HRIObject* s8 = s.getObj("S8");
  HRIObject* s9 = s.getObj("S9");
  if (s1!=NULL){
    result.push_back(s1);
  }
  if (s2!=NULL){
    result.push_back(s2);
  }
  if (s3!=NULL){
    result.push_back(s3);
  }
  if (s4!=NULL){
    result.push_back(s4);
  }
  if (s5!=NULL){
    result.push_back(s5);
  }
  if (s6!=NULL){
    result.push_back(s6);
  }
  if (s7!=NULL){
    result.push_back(s7);
  }
  if (s8!=NULL){
    result.push_back(s8);
  }
  if (s9!=NULL){
    result.push_back(s9);
  }
  return result;
}


test_task::test_task()
{
  //load lexicon from a file - need settings for which lexicon (exemplar vs grid, which person)
  //create model of objects in the world
  learning_type=0;
  current_phrase=0;
  TTSRlexiconExemplar.init_lexicon();
  TTSRlexiconExemplar.load_lexicon("exemplar_lexicon.dat");
  TTSRlexiconGrid.init_lexicon();
  TTSRlexiconGrid.load_lexicon("grid_lexicon.dat");
}
