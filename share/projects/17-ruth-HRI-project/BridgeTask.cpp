#include "BridgeTask.h"
#include <iostream>
#include <ctime>

using std::cout;
using std::endl;

BridgeTask::BridgeTask(int m) {
  mode = m;
  started = 0;
  currentState = 0;
  p_time = std::time(nullptr);
  log("start");
}


bool BridgeTask::isPlausible(OnTableState& s) {
  // no object should occur more than once for this task
  if (s.countObjs(HRIObject::blue, HRIObject::small) > 1)
    return false;
  if (s.countObjs(HRIObject::blue, HRIObject::large) > 1)
    return false;
  if (s.countObjs(HRIObject::red, HRIObject::small) > 1)
    return false;
  if (s.countObjs(HRIObject::green, HRIObject::small) > 1)
    return false;
  if (s.countObjs(HRIObject::yellow, HRIObject::small) > 1)
    return false;
  if (s.countObjs(HRIObject::red, HRIObject::medium) > 1)
    return false;
  return true;
}

int BridgeTask::getBridgeState(OnTableState& s) {
  int newStackState = 0;
  // query objects of bridge task
  HRIObject* bs = s.getObj(HRIObject::blue, HRIObject::small);
  HRIObject* bl = s.getObj(HRIObject::blue, HRIObject::large);
  HRIObject* rs = s.getObj(HRIObject::red, HRIObject::small);
  HRIObject* rm = s.getObj(HRIObject::red, HRIObject::medium);
  HRIObject* gs = s.getObj(HRIObject::green, HRIObject::small);
  HRIObject* ys = s.getObj(HRIObject::yellow, HRIObject::small);
  if (bs==NULL||rs==NULL||gs==NULL||ys==NULL||bl==NULL||rm==NULL){
    newStackState=-1;
  } else {

    // check if blue small and red small are near to goal position
    mlr::Vector pbs=bs->pos;
    mlr::Vector tbs=R.getK()().getShapeByName("targetbs")->X.pos;

    mlr::Vector prs=rs->pos;
    mlr::Vector trs=R.getK()().getShapeByName("targetrs")->X.pos;

    bool bs_placed = (sqrt((pbs.x-tbs.x)*(pbs.x-tbs.x)+(pbs.y-tbs.y)*(pbs.y-tbs.y)) < 0.1);
    bool rs_placed = (sqrt((prs.x-trs.x)*(prs.x-trs.x)+(prs.y-trs.y)*(prs.y-trs.y)) < 0.1);

    if (bl->isBelow(rm)) {
      newStackState=10;
    } else if (bl->pos.z>0.76 && !rstate.containsObj(bl)) {
      newStackState=9;
    } else if (bs->isBelow(gs) && rs->isBelow(ys)) {
      newStackState=8;
    } else if (rs_placed && bs->isBelow(gs)){
      newStackState=7;
    } else if (bs_placed && rs->isBelow(ys)){
      newStackState=6;
    } else if (bs->isBelow(gs)){
      newStackState=5;
    } else if (rs->isBelow(ys)){
      newStackState=3;
    } else if (bs_placed && rs_placed){
      newStackState=4;
    } else if (bs_placed) {
      newStackState=2;
    } else if (rs_placed){
      newStackState=1;
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

std::vector<HRIObject*> BridgeTask::nextActions(int bridgestate, OnTableState& s) {
  std::vector<HRIObject*> result; 
 // query objects of bridge task
  HRIObject* bs = s.getObj(HRIObject::blue, HRIObject::small);
  HRIObject* bl = s.getObj(HRIObject::blue, HRIObject::large);
  HRIObject* rs = s.getObj(HRIObject::red, HRIObject::small);
  HRIObject* rm = s.getObj(HRIObject::red, HRIObject::medium);
  HRIObject* gs = s.getObj(HRIObject::green, HRIObject::small);
  HRIObject* ys = s.getObj(HRIObject::yellow, HRIObject::small);
  if (bs==NULL||rs==NULL||gs==NULL||ys==NULL||bl==NULL||rm==NULL)
    return result;
  switch (bridgestate) {
  case 0:
    result.push_back(rs);
    result.push_back(bs);
    break;
  case 1:
    result.push_back(bs);
    result.push_back(ys);
    break;
  case 2:
    result.push_back(rs);
    result.push_back(gs);
    break;
  case 3:
    result.push_back(bs);
    break;
  case 4:
    result.push_back(ys);
    result.push_back(gs);
    break;
  case 5:
    result.push_back(rs);
    break;
  case 6:
    result.push_back(gs);
    break;
  case 7:
    result.push_back(ys);
    break;
  case 8:
    result.push_back(bl);
    break;
  case 9:
    result.push_back(rm);
    break;
  default:
    break;
  }
  return result;
}

std::vector<HRIObject*> BridgeTask::futureActions(int bridgestate, OnTableState& s, HRIObject::Color objC, HRIObject::Size objS) {
  std::vector<HRIObject*> result;
 // query objects of bridge task
  HRIObject* bs = s.getObj(HRIObject::blue, HRIObject::small);
  HRIObject* bl = s.getObj(HRIObject::blue, HRIObject::large);
  HRIObject* rs = s.getObj(HRIObject::red, HRIObject::small);
  HRIObject* rm = s.getObj(HRIObject::red, HRIObject::medium);
  HRIObject* gs = s.getObj(HRIObject::green, HRIObject::small);
  HRIObject* ys = s.getObj(HRIObject::yellow, HRIObject::small);
  if (bs==NULL||rs==NULL||gs==NULL||ys==NULL||bl==NULL||rm==NULL)
    return result;
  int newbridgestate = -1;
  switch (bridgestate) {
    case 0:
      if ((objC==HRIObject::red)&&(objS==HRIObject::small)){
        newbridgestate = 1;
      } else if ((objC==HRIObject::blue)&&(objS==HRIObject::small)){
        newbridgestate = 2;
      }
      break;
    case 1:
      if ((objC==HRIObject::blue)&&(objS==HRIObject::small)){
        newbridgestate = 4;
      } else if ((objC==HRIObject::yellow)&&(objS==HRIObject::small)){
        newbridgestate = 3;
      }
      break;
    case 2:
      if ((objC==HRIObject::red)&&(objS==HRIObject::small)){
        newbridgestate = 4;
      } else if ((objC==HRIObject::green)&&(objS==HRIObject::small)){
        newbridgestate = 7;
      }
      break;
    case 3:
      if ((objC==HRIObject::blue)&&(objS==HRIObject::small)){
        newbridgestate = 6;
      }
      break;
    case 4:
      if ((objC==HRIObject::yellow)&&(objS==HRIObject::small)){
        newbridgestate = 6;
      } else if ((objC==HRIObject::green)&&(objS==HRIObject::small)){
        newbridgestate = 7;
      }
      break;
    case 5:
      if ((objC==HRIObject::red)&&(objS==HRIObject::small)){
        newbridgestate = 7;
      }
      break;
    case 6:
      if ((objC==HRIObject::green)&&(objS==HRIObject::small)){
        newbridgestate = 8;
      }
      break;
    case 7:
      if ((objC==HRIObject::yellow)&&(objS==HRIObject::small)){
        newbridgestate = 8;
      }
      break;
    case 8:
      if ((objC==HRIObject::blue)&&(objS==HRIObject::large)){
        newbridgestate = 9;
      }
      break;
    default:
      break;
  }
  switch (newbridgestate) {
    case 0:
      result.push_back(rs);
      result.push_back(bs);
      break;
    case 1:
      result.push_back(bs);
      result.push_back(ys);
      break;
    case 2:
      result.push_back(rs);
      result.push_back(gs);
      break;
    case 3:
      result.push_back(bs);
      break;
    case 4:
      result.push_back(ys);
      result.push_back(gs);
      break;
    case 5:
      result.push_back(rs);
      break;
    case 6:
      result.push_back(gs);
      break;
    case 7:
      result.push_back(ys);
      break;
    case 8:
      result.push_back(bl);
      break;
    case 9:
      result.push_back(rm);
      break;
    default:
      break;
  }
  return result;
}

void BridgeTask::humanPlaceObject(OnTableState& s, HRIObject::Color hobjC, HRIObject::Size hobjS){
  // query objects of stack task
   HRIObject* bs = s.getObj(HRIObject::blue, HRIObject::small);
   HRIObject* bl = s.getObj(HRIObject::blue, HRIObject::large);
   HRIObject* rs = s.getObj(HRIObject::red, HRIObject::small);
   HRIObject* rm = s.getObj(HRIObject::red, HRIObject::medium);
   HRIObject* gs = s.getObj(HRIObject::green, HRIObject::small);
   HRIObject* ys = s.getObj(HRIObject::yellow, HRIObject::small);
   mlr::Vector tbs=R.getK()().getShapeByName("targetbs")->X.pos;
   mlr::Vector trs=R.getK()().getShapeByName("targetrs")->X.pos;

   // medium red
   // large blue
   // yellow     green
   // red        blue

   mlr::Vector objPosition;
   if (hobjC==HRIObject::green){
     objPosition.x = bs->pos.x;
     objPosition.y = bs->pos.y;
     objPosition.z = bs->pos.z+0.04;
   } else if (hobjC==HRIObject::yellow){
     objPosition.x = rs->pos.x;
     objPosition.y = rs->pos.y;
     objPosition.z = rs->pos.z+0.04;
   } else if (hobjC==HRIObject::red){
     if (hobjS==HRIObject::small){
       objPosition.x = trs.x;
       objPosition.y = trs.y;
       objPosition.z = rs->pos.z;
     } else {
       objPosition.x = bl->pos.x;
       objPosition.y = bl->pos.y;
       objPosition.z = bl->pos.z+0.04;
     }
   } else if (hobjC==HRIObject::blue){
     if (hobjS==HRIObject::small){
       objPosition.x = tbs.x;
       objPosition.y = tbs.y;
       objPosition.z = bs->pos.z;
     } else {
       objPosition.x = (gs->pos.x+ys->pos.x)/2.0;
       objPosition.y = (gs->pos.y+ys->pos.y)/2.0;
       objPosition.z = gs->pos.z+0.04;
     }
   }
   state.setObjectPosition(hobjC,hobjS,objPosition);
   state.updateModelworldFromState();
}


bool BridgeTask::placeObj(HRIObject* obj) {
  std::stringstream output;
  output<< "robot place object " << *obj;
  log(output.str());
  if (obj->color==HRIObject::blue && obj->size==HRIObject::small) {
    auto placeObj = R.placeDistDirTable(obj->id.c_str(),"targetbs","table",0.,0.,0.,0);
    R.wait(+placeObj);
    return true;
  } else if (obj->color==HRIObject::red && obj->size == HRIObject::small) {
    auto placeObj = R.placeDistDirTable(obj->id.c_str(),"targetrs","table",0.,0.,0.,0);
    R.wait(+placeObj);
    return true;
  } else if (obj->color==HRIObject::yellow && obj->size == HRIObject::small) {
    auto placeObj = R.placeDistDirTable(obj->id.c_str(),state.getObj(HRIObject::red, HRIObject::small)->id.c_str(),"table",0.,0.,0.,0);
    R.wait(+placeObj);
    return true;
  } else if (obj->color==HRIObject::blue && obj->size == HRIObject::large) {
    auto placeObj = R.placeDistDirTable(obj->id.c_str(),"targetbl","table",0.,0.,0.,1);
    R.wait(+placeObj);
    return true;
  } else if (obj->color==HRIObject::green && obj->size == HRIObject::small) {
    auto placeObj = R.placeDistDirTable(obj->id.c_str(),state.getObj(HRIObject::blue, HRIObject::small)->id.c_str(),"table",0.,0.,0.,0);
    R.wait(+placeObj);
    return true;
  } else if (obj->color==HRIObject::red && obj->size == HRIObject::medium) {
    auto placeObj = R.placeDistDirTable(obj->id.c_str(),state.getObj(HRIObject::blue,HRIObject::large)->id.c_str(),"table",0.,0.,0.,0);
    R.wait(+placeObj);
    return true;
  }
  return false;
}

bool BridgeTask::isReachable(HRIObject* obj) {
  return obj->pos.x<.6;
}

void BridgeTask::performAction() {
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
//      system("rosrun sound_play say.py \"build task\"");
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

void BridgeTask::performActionAutonomous() {
  int bridgestate = getBridgeState(state);
  cout << "State of Bridge Task: " << bridgestate << endl;
  cout << "State: " << state;
  std::vector<HRIObject*> objs = nextActions(bridgestate, state);
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

void BridgeTask::performActionAutonomousPlus() {
  int bridgestate = getBridgeState(state);
  cout << "State of Bridge Task: " << bridgestate << endl;
  cout << "State: " << state;
  std::vector<HRIObject*> objs = nextActions(bridgestate, state);
//  std::stringstream output;
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
  std::vector<HRIObject*> objs1;
  if ((robotObj!=-1)&&(humanObj==-1)){
    objs1 = futureActions(bridgestate, state, objs[robotObj]->color, objs[robotObj]->size);
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

void BridgeTask::performActionCommands() {
  int bridgestate = getBridgeState(state);
  std::vector<HRIObject*> objs = nextActions(bridgestate, state);
  int input=0;
  cout << "State: " << state;
  if (objs.size()>0){
    cout << "Which object do you want to grasp/place? (1-6), 0=redo perception" << endl;
    std::cin >> input;
    if (input==0) {
      return;
    }
  } else {
    cout << "no actions available";
//    system("echo \"task completed\" | festival --tts");
//    system("rosrun sound_play say.py \"task completed\"");
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
void BridgeTask::performActionRobotCommands() {
  //2 stages - first, perform any possible actions autonomously,
  // then request human to perform any other actions
  bool objectPlaced = false;
  int bridgestate = getBridgeState(state);
  cout << "State: " << state;
  std::vector<HRIObject*> objs = nextActions(bridgestate, state);
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

void BridgeTask::log(std::string s) {
  ofstream myfile;
  std::string filename="logfileBridge" + std::to_string(mode) + ".txt";
  myfile.open (filename, std::ios::out | std::ios::app);
  std::time_t result = std::time(0);
  myfile << result << ", ";
  myfile << s << endl;
  myfile.close();
}
