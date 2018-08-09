#include "BasicTask.h"
#include <iostream>
#include <ctime>

using std::cout;
using std::endl;

BasicTask::BasicTask(int m) {
  mode = m;
  started = 0;
  log("start");
}


bool BasicTask::isPlausible(OnTableState& s) {
  // check count of objects of one kind
  // no object should occur more than once for this task
  if (s.countObjs(HRIObject::blue, HRIObject::small) > 1)
    return false;
  if (s.countObjs(HRIObject::blue, HRIObject::large) > 1)
    return false;
  if (s.countObjs(HRIObject::red, HRIObject::small) > 1)
    return false;
  if (s.countObjs(HRIObject::red, HRIObject::medium) > 1)
    return false;
  if (s.countObjs(HRIObject::green, HRIObject::small) > 1)
    return false;
  if (s.countObjs(HRIObject::yellow, HRIObject::small) > 1)
    return false;
  return true;
}

std::vector<HRIObject*> BasicTask::nextActions(OnTableState& s) {
  std::vector<HRIObject*> result;
  int numObjectsOnTable = 0;
  // query objects of stack task
   HRIObject* bs = s.getObj(HRIObject::blue, HRIObject::small);
   HRIObject* bl = s.getObj(HRIObject::blue, HRIObject::large);
   HRIObject* rs = s.getObj(HRIObject::red, HRIObject::small);
   HRIObject* rm = s.getObj(HRIObject::red, HRIObject::medium);
   HRIObject* gs = s.getObj(HRIObject::green, HRIObject::small);
   HRIObject* ys = s.getObj(HRIObject::yellow, HRIObject::small);
   if (bs==NULL||rs==NULL||rm==NULL||gs==NULL||ys==NULL||bl==NULL){
     return result;
   } else {


     cout << "can pick up: ";
     if (bs->below_obj==NULL){
       result.push_back(bs);
       cout << "small blue, ";
       numObjectsOnTable++;
     }
     if (bl->below_obj==NULL){
       result.push_back(bl);
       cout << "large blue, ";
       numObjectsOnTable++;
     }
     if (gs->below_obj==NULL){
       result.push_back(gs);
       cout << "small green, ";
       numObjectsOnTable++;
     }
     if (rs->below_obj==NULL){
       result.push_back(rs);
       cout << "small red, ";
       numObjectsOnTable++;
     }
     if (rm->below_obj==NULL){
       result.push_back(rm);
       cout << "medium red, ";
       numObjectsOnTable++;
     }
     if (ys->below_obj==NULL){
       result.push_back(ys);
       cout << "small yellow, ";
       numObjectsOnTable++;
     }
   }
   cout << numObjectsOnTable << " objects" << endl;
   return result;
}

std::string BasicTask::getTargetID(std::string tpos) {
  std::string targetname = tpos;
  HRIObject* target = NULL;
    for (HRIObject o : state.objects) {
      mlr::Vector pbs=o.pos;
      mlr::Vector tbs=R.getK()().getShapeByName(tpos.c_str())->X.pos;
      if (sqrt((pbs.x-tbs.x)*(pbs.x-tbs.x)+(pbs.y-tbs.y)*(pbs.y-tbs.y)) < 0.03) {
        target=&o;
        break;
      }
    }
    if (target!=NULL) {
      while (target->below_obj != NULL)
        target = target->below_obj;
      targetname = target->id;
    }

    if (state.getObj(targetname)==NULL)
      cout << "target: " << targetname << endl;
    else
      cout << "target: " << *(state.getObj(targetname)) << endl;
  return targetname;
}

bool BasicTask::placeObj(HRIObject* obj, int location) {
  std::stringstream output;
  output<< "robot place object " << *obj;
  log(output.str());
  std::string targetid;
  if ((location>0)&&(location<7)){
    // target is an object
    std::string inID = "S" +  std::to_string(location);
    targetid = inID.c_str();
  } else {
    // target is a predefined position
    if (location==0){
      targetid = getTargetID("target1").c_str();
    } else if (location==7){
      targetid = getTargetID("target2").c_str();
    } else if (location==8){
      targetid = getTargetID("target3").c_str();
    } else if (location==9){
      targetid = getTargetID("target4").c_str();
    }
  }

  auto placeObj = R.placeDistDirTable(obj->id.c_str(),targetid.c_str(),"table",0.,0.,0.,0);
  R.wait(+placeObj);
  return true;
}

bool BasicTask::isReachable(HRIObject* obj) {
  return obj->pos.x<.6;
}

void BasicTask::performAction() {
  if (started==0){
    started=1;
    log("first action");
  }
  cout << "perform action" << endl;
  performActionCommands();
}

void BasicTask::performActionCommands() {
  cout << "State: " << state;
  std::vector<HRIObject*> objs = nextActions(state);
  int inputObject=0;
  int inputAction=0;
  int inputLocation=0;
  if (objs.size()>0){
    cout << "Which object do you want to grasp/place? (1-6), 0=redo perception" << endl;
    std::cin >> inputObject;
    if (inputObject==0) {
      return;
    }
    cout << "Which action do you want to perform? 1=pick up, 2=place, 3=point" << endl;
    std::cin >> inputAction;
    if (inputAction==0) {
      return;
    }
    cout << "Which location do you want to choose? (objects 1-6, predefined locations 7,8,9,0)" <<endl;
    std::cin >> inputLocation;
  } else {
    cout << "no actions available";
//    system("echo \"task completed\" | festival --tts");
//    system("rosrun sound_play say.py \"task completed\"");
    log("task completed");
    std::cin >> inputObject;
  }
  std::string inID = "S" +  std::to_string(inputObject);
  bool availableAction = false;
  for (unsigned int i=0; i<objs.size(); ++i) {
    if (objs[i]->id == inID){
      availableAction = true;
    }
  }
  if (availableAction&&isReachable(state.getObj(inID))) {
    if (inputAction==1){
      //pick up object
      auto graspObj = R.graspBox(inID.c_str(), state.getObj(inID)->pos.y<0 ? LR_right : LR_left);
      R.wait(+graspObj);
    } else if (inputAction==2){
      // place object
      placeObj(state.getObj(inID),inputLocation);
      state.updateFromModelworld(R);
    } else if (inputAction==3){
      // point object
      auto pointObj = R.pointBox(inID.c_str(),state.getObj(inID)->pos.y<0 ? LR_right : LR_left);
      R.wait(+pointObj);
    }
  }
}



void BasicTask::log(std::string s) {
  ofstream myfile;
  std::string filename="logfileSort" + std::to_string(mode) + ".txt";
  myfile.open (filename, std::ios::out | std::ios::app);
  std::time_t result = std::time(0);
  myfile << result << ", ";
  myfile << s << endl;
  myfile.close();
}
