#include "StackTask.h"
#include <iostream>
#include <ctime>

using std::cout;
using std::endl;

StackTask::StackTask(int m) {
  mode = m;
  started = 0;
  currentState = 0;
  log("start");
}


bool StackTask::isPlausible(OnTableState& s) {
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

int StackTask::getStackState(OnTableState& s) {
  int newStackState = 0;
  // query objects of stack task
  HRIObject* bs = s.getObj(HRIObject::blue, HRIObject::small);
  HRIObject* bl = s.getObj(HRIObject::blue, HRIObject::large);
  HRIObject* rs = s.getObj(HRIObject::red, HRIObject::small);
  HRIObject* rm = s.getObj(HRIObject::red, HRIObject::medium);
  HRIObject* gs = s.getObj(HRIObject::green, HRIObject::small);
  HRIObject* ys = s.getObj(HRIObject::yellow, HRIObject::small);
  if (bs==NULL||rs==NULL||rm==NULL||gs==NULL||ys==NULL||bl==NULL){
    newStackState = -1;
  } else {

    // check if blue large is near to goal position
    mlr::Vector pbl=bl->pos;
    mlr::Vector tbl=R.getK()().getShapeByName("target")->X.pos;

    bool bl_placed = (sqrt((pbl.x-tbl.x)*(pbl.x-tbl.x)+(pbl.y-tbl.y)*(pbl.y-tbl.y)) < 0.1);
    cout << bl_placed << " " << pbl << " " << tbl << " " << sqrt((pbl.x-tbl.x)*(pbl.x-tbl.x)+(pbl.y-tbl.y)*(pbl.y-tbl.y)) << endl;

    if (rs->isBelow(ys)){
      newStackState = 6;
    } else if (bs->isBelow(rs)){
      newStackState = 5;
    } else if (gs->isBelow(bs)) {
      newStackState = 4;
    } else if (rm->isBelow(gs)) {
      newStackState = 3;
    } else if (bl->isBelow(rm)) {
      newStackState = 2;
    } else if (bl_placed) {
      newStackState = 1;
    }
  }
  if (newStackState!=currentState){
    std::stringstream output;
    output<< "new state: " << newStackState;
    log(output.str());
    currentState = newStackState;
  }
  return newStackState;
}

std::vector<HRIObject*> StackTask::nextActions(int stackstate, OnTableState& s) {
  std::vector<HRIObject*> result;

  // query objects of stack task
   HRIObject* bs = s.getObj(HRIObject::blue, HRIObject::small);
   HRIObject* bl = s.getObj(HRIObject::blue, HRIObject::large);
   HRIObject* rs = s.getObj(HRIObject::red, HRIObject::small);
   HRIObject* rm = s.getObj(HRIObject::red, HRIObject::medium);
   HRIObject* gs = s.getObj(HRIObject::green, HRIObject::small);
   HRIObject* ys = s.getObj(HRIObject::yellow, HRIObject::small);
   if (bs==NULL||rs==NULL||rm==NULL||gs==NULL||ys==NULL||bl==NULL)
     return result;
   switch (stackstate) {
   case 0:
     result.push_back(bl);
     break;
   case 1:
     result.push_back(rm);
     break;
   case 2:
     result.push_back(gs);
     break;
   case 3:
     result.push_back(bs);
     break;
   case 4:
     result.push_back(rs);
     break;
   case 5:
     result.push_back(ys);
     break;
   default:
     break;
   }
   return result;
}

std::vector<HRIObject*> StackTask::futureActions(int stackstate, OnTableState& s) {
  std::vector<HRIObject*> result;

  // query objects of stack task
   HRIObject* bs = s.getObj(HRIObject::blue, HRIObject::small);
   HRIObject* bl = s.getObj(HRIObject::blue, HRIObject::large);
   HRIObject* rs = s.getObj(HRIObject::red, HRIObject::small);
   HRIObject* rm = s.getObj(HRIObject::red, HRIObject::medium);
   HRIObject* gs = s.getObj(HRIObject::green, HRIObject::small);
   HRIObject* ys = s.getObj(HRIObject::yellow, HRIObject::small);
   if (bs==NULL||rs==NULL||rm==NULL||gs==NULL||ys==NULL||bl==NULL)
     return result;
   switch (stackstate) {
   case 0:
     result.push_back(rm);
     break;
   case 1:
     result.push_back(gs);
     break;
   case 2:
     result.push_back(bs);
     break;
   case 3:
     result.push_back(rs);
     break;
   case 4:
     result.push_back(ys);
     break;
   default:
     break;
   }
   return result;
}

void StackTask::humanPlaceObject(OnTableState& s, HRIObject::Color hobjC, HRIObject::Size hobjS){
  // query objects of stack task
   HRIObject* bs = s.getObj(HRIObject::blue, HRIObject::small);
   HRIObject* bl = s.getObj(HRIObject::blue, HRIObject::large);
   HRIObject* rs = s.getObj(HRIObject::red, HRIObject::small);
   HRIObject* rm = s.getObj(HRIObject::red, HRIObject::medium);
   HRIObject* gs = s.getObj(HRIObject::green, HRIObject::small);
   HRIObject* ys = s.getObj(HRIObject::yellow, HRIObject::small);
   mlr::Vector tbl=R.getK()().getShapeByName("target")->X.pos;

   // large blue, medium red, small green, blue, red, yellow

   mlr::Vector objPosition;
   if (hobjC==HRIObject::green){
     objPosition.x = rm->pos.x;
     objPosition.y = rm->pos.y;
     objPosition.z = rm->pos.z+0.04;
   } else if (hobjC==HRIObject::yellow){
     objPosition.x = rs->pos.x;
     objPosition.y = rs->pos.y;
     objPosition.z = rs->pos.z+0.04;
   } else if (hobjC==HRIObject::red){
     if (hobjS==HRIObject::small){
       objPosition.x = bs->pos.x;
       objPosition.y = bs->pos.y;
       objPosition.z = bs->pos.z+0.04;
     } else {
       objPosition.x = bl->pos.x;
       objPosition.y = bl->pos.y;
       objPosition.z = bl->pos.z+0.04;
     }
   } else if (hobjC==HRIObject::blue){
     if (hobjS==HRIObject::small){
       objPosition.x = gs->pos.x;
       objPosition.y = gs->pos.y;
       objPosition.z = gs->pos.z+0.04;
     } else {
       objPosition.x = tbl.x;
       objPosition.y = tbl.y;
       objPosition.z = bl->pos.z;
     }
   }
   state.setObjectPosition(hobjC,hobjS,objPosition);
   state.updateModelworldFromState();
}


bool StackTask::placeObj(HRIObject* obj) {
  std::stringstream output;
  output<< "robot place object " << *obj;
  log(output.str());
  if (obj->color==HRIObject::blue && obj->size==HRIObject::large) {
    auto placeObj = R.placeDistDirTable(obj->id.c_str(),"target","table",0.,0.,0.,0);
    R.wait(+placeObj);
    return true;
  } else if (obj->color==HRIObject::red && obj->size == HRIObject::medium) {
    auto placeObj = R.placeDistDirTable(obj->id.c_str(),state.getObj(HRIObject::blue, HRIObject::large)->id.c_str(),"table",0.,0.,0.,0);
    R.wait(+placeObj);
    return true;
  } else if (obj->color==HRIObject::green&& obj->size==HRIObject::small) {
    auto placeObj = R.placeDistDirTable(obj->id.c_str(),state.getObj(HRIObject::red, HRIObject::medium)->id.c_str(),"table",0.,0.,0.,0);
    R.wait(+placeObj);
    return true;
  } else if (obj->color==HRIObject::blue&& obj->size == HRIObject::small) {
    auto placeObj = R.placeDistDirTable(obj->id.c_str(),state.getObj(HRIObject::green, HRIObject::small)->id.c_str(),"table",0.,0.,0.,0);
    R.wait(+placeObj);
    return true;
  } else if (obj->color==HRIObject::red && obj->size == HRIObject::small) {
    auto placeObj = R.placeDistDirTable(obj->id.c_str(),state.getObj(HRIObject::blue, HRIObject::small)->id.c_str(),"table",0.,0.,0.,0);
    R.wait(+placeObj);
    return true;
  } else if (obj->color==HRIObject::yellow && obj->size == HRIObject::small) {
    auto placeObj = R.placeDistDirTable(obj->id.c_str(),state.getObj(HRIObject::red, HRIObject::small)->id.c_str(),"table",0.,0.,0.,0);
    R.wait(+placeObj);
    return true;
  }
  return false;
}

bool StackTask::isReachable(HRIObject* obj) {
  return obj->pos.x<.6;
}

void StackTask::performAction() {
  if (started==0){
    started=1;
    log("first action");
//    system("echo \"begin task\" | festival --tts");
//    system("rosrun sound_play say.py \"begin task\"");
    if (mode==0){
//      system("rosrun sound_play say.py \"autonomous strategy \"");
    } else if (mode==1){
//      system("rosrun sound_play say.py \"human commands strategy\"");
    } else if (mode==2){
//      system("rosrun sound_play say.py \"robot commands strategy\"");
    } else if (mode==3){
//      system("rosrun sound_play say.py \"information strategy\"");
    }
//      system("rosrun sound_play say.py \"stack task\"");
  }
  switch(mode) {
  case 0:
    performActionAutonomous();
    break;
  case 1:
    performActionCommands();
    break;
  case 2:
    performActionRobotCommands();
    break;
  case 3:
    performActionAutonomousPlus();
    break;
  default:
    cout << mode << " is no correct interaction mode" << endl;
    break;
  }
}

void StackTask::performActionAutonomous() {
  int stackstate = getStackState(state);
  cout << "State: " << state;
  std::stringstream output;
  std::vector<HRIObject*> objs = nextActions(stackstate, state);
  if (objs.size()==0){
    cout << "no actions available";
//    system("echo \"task completed\" | festival --tts");
//    system("rosrun sound_play say.py \"task completed\"");
    log("task completed");
    int input=0;
    std::cin >> input;
  }
  for (unsigned int i=0; i<objs.size(); ++i) {
    if (isReachable(objs[i])) {
      auto graspObj = R.graspBox(objs[i]->id.c_str(), objs[i]->pos.y<0 ? LR_right : LR_left);
      R.wait(+graspObj);
      placeObj(objs[i]);
      state.updateFromModelworld(R);
      break;
    }
  }
}

void StackTask::performActionAutonomousPlus() {
  int stackstate = getStackState(state);
  cout << "State: " << state;
  std::stringstream output;
  std::vector<HRIObject*> objs = nextActions(stackstate, state);
  std::vector<HRIObject*> objs1 = futureActions(stackstate, state);
//  cout << "State: " << state;
//  std::vector<HRIObject*> objs = nextActions(state);
  if (objs.size()==0){
    cout << "no actions available";
//    system("rosrun sound_play say.py \"task completed\"");
    log("task completed");
    int input=0;
    std::cin >> input;
  }
  int robotObj = -1;
  int humanObj = -1;
  int humanObj1 = -1;
  for (unsigned int i=0; i<objs.size(); ++i) {
    if (isReachable(objs[i])) {
      if(robotObj==-1){
        robotObj=i;
      }
    }
  }
  for (unsigned int i=0; i<objs.size(); ++i) {
    if (!isReachable(objs[i])) {
      if(humanObj==-1){
        humanObj=i;
      }
    }
  }
  if (humanObj!=-1){
    std::stringstream request;
    request << "Can you please place the ";
    if (objs[humanObj]->color==HRIObject::green){
//      system("rosrun sound_play say.py \"please place the small green block\"");
      request << "small green ";
    } else if (objs[humanObj]->color==HRIObject::yellow){
//      system("rosrun sound_play say.py \"please place the small yellow block\"");
      request << "small yellow ";
    } else if (objs[humanObj]->color==HRIObject::red){
      if (objs[humanObj]->size==HRIObject::small){
//        system("rosrun sound_play say.py \"please place the small red block\"");
        request << "small red ";
      } else {
//        system("rosrun sound_play say.py \"please place the medium red block\"");
        request << "medium red ";
      }
    } else if (objs[humanObj]->color==HRIObject::blue){
      if (objs[humanObj]->size==HRIObject::small){
//        system("rosrun sound_play say.py \"please place the small blue block\"");
        request << "small blue ";
      } else {
//        system("rosrun sound_play say.py \"please place the large blue block\"");
        request << "large blue ";
      }
    }
    request << "block";
    cout << request.str();
  }
  if (humanObj==-1){
    // ask to place following the current action
//    std::vector<HRIObject*> objs1 = futureActions(stackstate, state);
    for (unsigned int i=0; i<objs1.size(); ++i) {
      if (!isReachable(objs1[i])) {
        if(humanObj1==-1){
          humanObj1=i;
        }
      }
    }
    if (humanObj1!=-1){
      std::stringstream request;
      request << "After I place this block, can you please place the ";
      if (objs1[humanObj1]->color==HRIObject::green){
//        system("rosrun sound_play say.py \"After I place this block, please place the small green block\"");
        request << "small green ";
      } else if (objs1[humanObj1]->color==HRIObject::yellow){
//        system("rosrun sound_play say.py \"After I place this block, please place the small yellow block\"");
        request << "small yellow ";
      } else if (objs1[humanObj1]->color==HRIObject::red){
        if (objs1[humanObj1]->size==HRIObject::small){
//          system("rosrun sound_play say.py \"After I place this block, please place the small red block\"");
          request << "small red ";
        } else {
//          system("rosrun sound_play say.py \"After I place this block, please place the medium red block\"");
          request << "medium red ";
        }
      } else if (objs1[humanObj1]->color==HRIObject::blue){
        if (objs1[humanObj1]->size==HRIObject::small){
//          system("rosrun sound_play say.py \"After I place this block, please place the small blue block\"");
          request << "small blue ";
        } else {
//          system("rosrun sound_play say.py \"After I place this block, please place the large blue block\"");
          request << "large blue ";
        }
      }
      request << "block";
      cout << request.str();
    }
  }

  if (robotObj!=-1){
    auto graspObj = R.graspBox(objs[robotObj]->id.c_str(), objs[robotObj]->pos.y<0 ? LR_right : LR_left);
    R.wait(+graspObj);
    placeObj(objs[robotObj]);
    state.updateFromModelworld(R);
  } else {
    R.wait(2);
  }
  // now update position of where the human will place the object
  if (humanObj!=-1){
    humanPlaceObject(state, objs[humanObj]->color,objs[humanObj]->size);
  }
  if (humanObj1!=-1){
    humanPlaceObject(state, objs1[humanObj1]->color,objs1[humanObj1]->size);
  }
  state.updateFromModelworld(R);
}


 void StackTask::performActionCommands() {
   int stackstate = getStackState(state);
   cout << "State: " << state;
   std::vector<HRIObject*> objs = nextActions(stackstate, state);
   int input=0;
   if (objs.size()>0){
     cout << "Which object do you want to grasp/place? (1-6), 0=redo perception" << endl;
     std::cin >> input;
     getwc(stdin);
     if (input==0) {
       return;
     }
   } else {
     cout << "no actions available";
//     system("echo \"task completed\" | festival --tts");
//     system("rosrun sound_play say.py \"task completed\"");
     log("task completed");
     std::cin >> input;
   }

   std::string inID = "S" +  std::to_string(input);
   bool availableAction = false;
   for (unsigned int i=0; i<objs.size(); ++i) {
     if (objs[i]->id == inID){
       availableAction = true;
     }
   }
   if (availableAction&&isReachable(state.getObj(inID))) {
     auto graspObj = R.graspBox(inID.c_str(), state.getObj(inID)->pos.y<0 ? LR_right : LR_left);
     R.wait(+graspObj);
     placeObj(state.getObj(inID));
     state.updateFromModelworld(R);
   }
 }

// performActionRobotCommands
// ie the robot requests that the human does something
void StackTask::performActionRobotCommands() {
  //2 stages - first, perform any possible actions autonomously,
  // then request human to perform any other actions
  bool objectPlaced = false;
  int stackstate = getStackState(state);
  cout << "State: " << state;
  std::vector<HRIObject*> objs = nextActions(stackstate, state);
  for (unsigned int i=0; i<objs.size(); ++i) {
    if (isReachable(objs[i])) {
      auto graspObj = R.graspBox(objs[i]->id.c_str(), objs[i]->pos.y<0 ? LR_right : LR_left);
      R.wait(+graspObj);
      placeObj(objs[i]);
      state.updateFromModelworld(R);
      objectPlaced=true;
      break;
    }
  }
  // if it gets here, there were no actions to perform -> ask the human to perform the next action
  if (!objectPlaced) {
    if (objs.size()>0){
      if (objs[0]->color==HRIObject::green){
//        system("echo \"please place the small green block\" | festival --tts");
//        system("rosrun sound_play say.py \"please place the small green block\"");
      } else if (objs[0]->color==HRIObject::yellow){
//        system("echo \"please place the small yellow block\" | festival --tts");
//        system("rosrun sound_play say.py \"please place the small yellow block\"");
      } else if (objs[0]->color==HRIObject::red){
        if (objs[0]->size==HRIObject::small){
//          system("echo \"please place the small red block\" | festival --tts");
//          system("rosrun sound_play say.py \"please place the small red block\"");
        } else {
//          system("echo \"please place the medium red block\" | festival --tts");
//          system("rosrun sound_play say.py \"please place the medium red block\"");
        }
      } else if (objs[0]->color==HRIObject::blue){
        if (objs[0]->size==HRIObject::small){
//          system("echo \"please place the small blue block\" | festival --tts");
//          system("rosrun sound_play say.py \"please place the small blue block\"");
        } else {
//          system("echo \"please place the large blue block\" | festival --tts");
//          system("rosrun sound_play say.py \"please place the large blue block\"");
        }
      }
      std::stringstream request;
      request << "Can you please place the ";
      switch (objs[0]->size) {
      case HRIObject::small:
        request << "small ";
        break;
      case HRIObject::medium:
        request << "medium ";
        break;
      case HRIObject::large:
        request << "large ";
        break;
      default:
        request << "no size ";
      }
      switch (objs[0]->color){
        case HRIObject::red:
          request << "red ";
          break;
        case HRIObject::green:
          request << "green ";
          break;
        case HRIObject::blue:
          request << "blue ";
          break;
        case HRIObject::yellow:
          request << "yellow ";
          break;
        default:
          request << "no colour ";
      }
      request << "object";
      cout << request.str();
      R.wait(5);
//      int input=0;
//      std::cin >> input;
    } else {
      cout << "no actions available";
//      system("echo \"task completed\" | festival --tts");
//      system("rosrun sound_play say.py \"task completed\"");
      log("task completed");
      int input=0;
      std::cin >> input;
    }
  }
}



void StackTask::log(std::string s) {
  ofstream myfile;
  std::string filename="logfileStack" + std::to_string(mode) + ".txt";
  myfile.open (filename, std::ios::out | std::ios::app);
  std::time_t result = std::time(0);
  myfile << result << ", ";
  myfile << s << endl;
  myfile.close();
}
