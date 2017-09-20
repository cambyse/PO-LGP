#include "learn_task.h"
#include "lexicon_TTSR.h"

// learning phase of teaching a robot words for placing objects on a table
// two learning stages: exemplar and grid
// exemplar expects person to place obect at locations for "left", "right", "front",
// "back", "centre"
// grid holds object at locations in a grid and expects responses for which words are relevant
// "0: left, 1:right, 2:front, 3:back, 4:centre"
// [grid requires keyboard input, program should be started from terminal]

bool learn_task::isReachable(HRIObject* obj) {
  return obj->pos.x<.8; //TODO: check in real world what is reachable
}

void learn_task::performAction() {
  //if exemplar learning:
  //  "Place the object at phrase then click ok"
  //  store information about where the object is and what the phrase is
  //  update lexicon
  //  loop for all phrases
  //if grid learning
  //  pick up object and place at position
  //  "where is the object? select all relevant phrases then click ok"
  //  store information about where the object is and which phrases are relevant
  //  update lexicon
  //  loop for all positions in grid
  if (learning_type==-1){
    for (int i=0;i<10; i++){
      auto st = R.setTorso(0);
      R.wait({-st});
    }
    learning_type = 0;
  } else  if (learning_type==0){
    // exemplar
    cout<<"exemplar phrase: "<<current_phrase<<endl;
    R.wait();
    percept_nofilter();
    HRIObject* bs = state.getObj(HRIObject::blue, HRIObject::small);
    mlr::Vector pbs=bs->pos;
    cout << pbs.x << endl;
    cout << pbs.y << endl;
    dataExemplar[current_phrase][0]=pbs.x;
    dataExemplar[current_phrase][1]=pbs.y;
    // update lexicon using current phrase and position of small blue object
    mlr::Vector tp = TTSRlexiconExemplar.raw_table_position_to_table_position(TTSRlexiconExemplar.robot_to_raw_table_position(R,pbs));
    TTSRlexiconExemplar.update_lexicon(R,tp,current_phrase);
    current_phrase = current_phrase+1;
    if (current_phrase>=5){
      learning_type = 1;
      current_position=0;
      //save exemplar lexicon
      TTSRlexiconExemplar.save_lexicon("exemplar_lexicon.dat");
      //save data
      ofstream dataExemplarFile("exemplar_data.dat");
      for (int i=0; i<5; i++){
        mlr::Vector rpos;
        rpos.x = dataExemplar[i][0];
        rpos.y = dataExemplar[i][1];
        mlr::Vector rtpos = TTSRlexiconExemplar.robot_to_raw_table_position(R,rpos);
        mlr::Vector tpos = TTSRlexiconExemplar.raw_table_position_to_table_position(rtpos);
        dataExemplarFile<<tpos.x<<" "<<tpos.y<<endl;
      }
    }
 } else if (learning_type==1){
    // grid
    //place object at next position
    HRIObject* bs = state.getObj(HRIObject::blue, HRIObject::small);
    LeftOrRight hand = LR_right;
    if (current_position==0){
      auto graspObj = R.graspBox(bs->id.c_str(), hand);
      R.wait(+graspObj);
      mlr::Vector pos;
      for (int j=0; j<15; j++){
        pos.x = (current_position%5)*(TTSRlexiconGrid.tableSize[0]-2*TTSRlexiconGrid.objectSize)/4 - TTSRlexiconGrid.tableSize[0]/2 + TTSRlexiconGrid.objectSize;
        pos.y = (current_position/5)*(TTSRlexiconGrid.tableSize[1]-2*TTSRlexiconGrid.objectSize)/4 - TTSRlexiconGrid.tableSize[1]/2 + TTSRlexiconGrid.objectSize;
        auto pointObj = R.pointPosition(bs->id.c_str(),"table",hand,pos.x,pos.y,0.);
        R.wait(+pointObj);
        cout<<"grid: "<<current_position<<endl<<"0: left, 1:right, 2:front, 3:back, 4:centre "<<endl;
        mlr::Vector pbs = TTSRlexiconGrid.raw_table_position_to_table_position(pos);
        cout<<"("<<pos.x<<","<<pos.y<<";"<<pbs.x<<","<<pbs.y<<")";
        String option;
        std::cin>>option;
        cout<<option;
        for (int i=0; i<5; i++){
          if (strchr(option,48+i)!=NULL){
            TTSRlexiconGrid.update_lexicon(R,pbs,i);
            dataGrid[current_position][i]=1;
          } else {
            dataGrid[current_position][i]=0;
          }
        }
        current_position=current_position+1;
      }
      pos.x = (current_position%5)*(TTSRlexiconGrid.tableSize[0]-2*TTSRlexiconGrid.objectSize)/4 - TTSRlexiconGrid.tableSize[0]/2 + TTSRlexiconGrid.objectSize;
      pos.y = (current_position/5)*(TTSRlexiconGrid.tableSize[1]-2*TTSRlexiconGrid.objectSize)/4 - TTSRlexiconGrid.tableSize[1]/2 + TTSRlexiconGrid.objectSize;
      auto placeObjL = R.placeDistDir(bs->id.c_str(),"table",pos.x,pos.y,0.,0);
      R.wait(+placeObjL);
      auto an2 = R.armsNeutral();
      R.wait({-an2});
    } else if (current_position==15){
      HRIObject* bs = state.getObj(HRIObject::blue, HRIObject::small);
      hand = LR_left;
      auto graspObj = R.graspBox(bs->id.c_str(), hand);
      R.wait(+graspObj);
      mlr::Vector pos;
      for (int j=0; j<10; j++){
        pos.x = (current_position%5)*(TTSRlexiconGrid.tableSize[0]-2*TTSRlexiconGrid.objectSize)/4 - TTSRlexiconGrid.tableSize[0]/2 + TTSRlexiconGrid.objectSize;
        pos.y = (current_position/5)*(TTSRlexiconGrid.tableSize[1]-2*TTSRlexiconGrid.objectSize)/4 - TTSRlexiconGrid.tableSize[1]/2 + TTSRlexiconGrid.objectSize;
        auto pointObj = R.pointPosition(bs->id.c_str(),"table",hand,pos.x,pos.y,0.);
        R.wait(+pointObj);
        cout<<"grid: "<<current_position<<endl<<"0: left, 1:right, 2:front, 3:back, 4:centre "<<endl;
        mlr::Vector pbs = TTSRlexiconGrid.raw_table_position_to_table_position(pos);
        cout<<"("<<pos.x<<","<<pos.y<<";"<<pbs.x<<","<<pbs.y<<")";
        String option;
        std::cin>>option;
        cout<<option;
        for (int i=0; i<5; i++){
          if (strchr(option,48+i)!=NULL){
            TTSRlexiconGrid.update_lexicon(R,pbs,i);
            dataGrid[current_position][i]=1;
          } else {
            dataGrid[current_position][i]=0;
          }
        }
        current_position=current_position+1;
      }
    }
    if (current_position>=25){
      auto placeObjL = R.placeDistDir(bs->id.c_str(),"table",0,0,0.,0);
      R.wait(+placeObjL);
      auto an2 = R.armsNeutral();
      R.wait({-an2});
      learning_type = 2;
      //save grid lexicon
      TTSRlexiconGrid.save_lexicon("grid_lexicon.dat");
      //save data
      ofstream dataGridFile("grid_data.dat");
      for (int i=0; i<5; i++){
        for (int j=0; j<25; j++){
          if (dataGrid[j][i]){
            dataGridFile<<i<<" "<<j<<endl;
          }
        }
      }
    }
  } else {
    R.wait();
    cout << state << endl;
  }
}


learn_task::learn_task()
{
  //create model of objects in the world, possibly determine table size automatically (currently set by hand)
  learning_type=-1;
  TTSRlexiconExemplar.init_lexicon();
  TTSRlexiconGrid.init_lexicon();
}
