#include "test_place_task.h"

// test phase 2 (place object): #include "test_place_task.h" and test_place_task task;

bool test_place_task::isReachable(HRIObject* obj) {
  return obj->pos.x<.8; //TODO: check in real world what is reachable
}

LeftOrRight test_place_task::sideOfTable(HRIObject* obj){
  LeftOrRight hand = LR_left;
  if (obj->pos.y<0){
    hand = LR_right;
  }
  return hand;
}

void test_place_task::performAction() {
  if (learning_type==-1){
    HRIObject* bs = state.getObj(HRIObject::blue, HRIObject::small);
    if (isReachable(bs)) {
      cout<<"reaching";
      LeftOrRight hand = LR_right;
      auto graspObj = R.graspBox(bs->id.c_str(), hand);
      R.wait(+graspObj);
      auto placeObj = R.placeDistDir(bs->id.c_str(),"table",0,0,0.,0);
      R.wait(+placeObj);
      auto an = R.armsNeutral();
      R.wait({-an});
      hand = LR_left;
      auto graspObjL = R.graspBox(bs->id.c_str(), hand);
      R.wait(+graspObjL);
      auto placeObjL = R.placeDistDir(bs->id.c_str(),"table",0,0,0.,0);
      R.wait(+placeObjL);
      auto an2 = R.armsNeutral();
      R.wait({-an2});
      state.updateFromModelworld(R);
      percept_nofilter();
    }
    learning_type = 0;
  } else if (learning_type==0){
    percept_nofilter();
    mlr::Vector pos = TTSRlexiconExemplar.choose_position(R,current_phrase);
    cout<<"learning 0, word:"<<current_phrase<<"pos:"<<pos.x<<","<<pos.y<<endl;
    HRIObject* bs = state.getObj(HRIObject::blue, HRIObject::small);
    if (isReachable(bs)) {
      cout<<"reaching";
      LeftOrRight hand = LR_right;
      auto graspObj = R.graspBox(bs->id.c_str(), hand);
      R.wait(+graspObj);
      auto placeObj = R.placeDistDir(bs->id.c_str(),"table",pos.x,pos.y,0.,0);
      R.wait(+placeObj);
      auto an = R.armsNeutral();
      R.wait({-an});
      state.updateFromModelworld(R);
      percept_nofilter();
    }
    current_phrase=current_phrase+1;
    if (current_phrase>=5){
      current_phrase=0;
      learning_type=1;
    }
  } else if (learning_type==1){
    percept_nofilter();
    mlr::Vector pos = TTSRlexiconGrid.choose_position(R,current_phrase);
    HRIObject* bs = state.getObj(HRIObject::blue, HRIObject::small);
    if (isReachable(bs)) {
      LeftOrRight hand = LR_right;
      auto graspObj = R.graspBox(bs->id.c_str(), hand);
      R.wait(+graspObj);
      auto placeObj = R.placeDistDir(bs->id.c_str(),"table",pos.x,pos.y,0.,0);
      R.wait(+placeObj);
      auto an = R.armsNeutral();
      R.wait({-an});
      state.updateFromModelworld(R);
      percept_nofilter();
    }
    current_phrase=current_phrase+1;
    if (current_phrase>=5){
      current_phrase=0;
      learning_type=2;
    }
  } else {
    cout << state << endl;
    R.wait(); // no specific task -> just wait for user input
  }
}

test_place_task::test_place_task()
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
