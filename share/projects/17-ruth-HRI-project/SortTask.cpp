#include "SortTask.h"
#include <iostream>
#include <ctime>

using std::cout;
using std::endl;

SortTask::SortTask(int m) {
  mode = m;
  started = 0;
  currentState = 0;
  log("start");
}


bool SortTask::isPlausible(OnTableState& s) {
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

std::vector<HRIObject*> SortTask::nextActions(OnTableState& s) {
  int numObjectsPlaced = 0;
  std::vector<HRIObject*> result;

  // query objects of stack task
   HRIObject* bs = s.getObj(HRIObject::blue, HRIObject::small);
   HRIObject* bl = s.getObj(HRIObject::blue, HRIObject::large);
   HRIObject* rs = s.getObj(HRIObject::red, HRIObject::small);
   HRIObject* rm = s.getObj(HRIObject::red, HRIObject::medium);
   HRIObject* gs = s.getObj(HRIObject::green, HRIObject::small);
   HRIObject* ys = s.getObj(HRIObject::yellow, HRIObject::small);
   if (bs==NULL||rs==NULL||rm==NULL||gs==NULL||ys==NULL||bl==NULL){
     numObjectsPlaced = -1;
   } else {


     // check whether near to target position
     mlr::Vector tbs=R.getK()().getShapeByName("targetb")->X.pos;
     mlr::Vector tgs=R.getK()().getShapeByName("targetg")->X.pos;
     mlr::Vector trs=R.getK()().getShapeByName("targetr")->X.pos;
     mlr::Vector tys=R.getK()().getShapeByName("targety")->X.pos;
     bool bs_placed = (sqrt((bs->pos.x-tbs.x)*(bs->pos.x-tbs.x)+(bs->pos.y-tbs.y)*(bs->pos.y-tbs.y)) < 0.1);
     bool bl_placed = (sqrt((bl->pos.x-tbs.x)*(bl->pos.x-tbs.x)+(bl->pos.y-tbs.y)*(bl->pos.y-tbs.y)) < 0.1);
     bool gs_placed = (sqrt((gs->pos.x-tgs.x)*(gs->pos.x-tgs.x)+(gs->pos.y-tgs.y)*(gs->pos.y-tgs.y)) < 0.1);
     bool rs_placed = (sqrt((rs->pos.x-trs.x)*(rs->pos.x-trs.x)+(rs->pos.y-trs.y)*(rs->pos.y-trs.y)) < 0.1);
     bool rm_placed = (sqrt((rm->pos.x-trs.x)*(rm->pos.x-trs.x)+(rm->pos.y-trs.y)*(rm->pos.y-trs.y)) < 0.1);
     bool ys_placed = (sqrt((ys->pos.x-tys.x)*(ys->pos.x-tys.x)+(ys->pos.y-tys.y)*(ys->pos.y-tys.y)) < 0.1);

     if (!bs_placed){
       result.push_back(bs);
     } else {
       numObjectsPlaced++;
     }
     if (!bl_placed){
       result.push_back(bl);
     } else {
       numObjectsPlaced++;
     }
     if (!gs_placed){
       result.push_back(gs);
     } else {
       numObjectsPlaced++;
     }
     if (!rs_placed){
       result.push_back(rs);
     } else {
       numObjectsPlaced++;
     }
     if (!rm_placed){
       result.push_back(rm);
     } else {
       numObjectsPlaced++;
     }
     if (!ys_placed){
       result.push_back(ys);
     } else {
       numObjectsPlaced++;
     }
   }
   if (numObjectsPlaced!=currentState){
     std::stringstream output;
     output<< "new state: " << numObjectsPlaced;
     log(output.str());
     currentState = numObjectsPlaced;
   }
   return result;
}

void SortTask::humanPlaceObject(OnTableState& s, HRIObject::Color hobjC, HRIObject::Size hobjS){
  // query objects of stack task
   HRIObject* bs = s.getObj(HRIObject::blue, HRIObject::small);
   HRIObject* bl = s.getObj(HRIObject::blue, HRIObject::large);
   HRIObject* rs = s.getObj(HRIObject::red, HRIObject::small);
   HRIObject* rm = s.getObj(HRIObject::red, HRIObject::medium);
   HRIObject* gs = s.getObj(HRIObject::green, HRIObject::small);
   HRIObject* ys = s.getObj(HRIObject::yellow, HRIObject::small);

     // check whether near to target position
   mlr::Vector tbs=R.getK()().getShapeByName("targetb")->X.pos;
   mlr::Vector tgs=R.getK()().getShapeByName("targetg")->X.pos;
   mlr::Vector trs=R.getK()().getShapeByName("targetr")->X.pos;
   mlr::Vector tys=R.getK()().getShapeByName("targety")->X.pos;
   bool bs_placed = (sqrt((bs->pos.x-tbs.x)*(bs->pos.x-tbs.x)+(bs->pos.y-tbs.y)*(bs->pos.y-tbs.y)) < 0.1);
   bool bl_placed = (sqrt((bl->pos.x-tbs.x)*(bl->pos.x-tbs.x)+(bl->pos.y-tbs.y)*(bl->pos.y-tbs.y)) < 0.1);
   bool gs_placed = (sqrt((gs->pos.x-tgs.x)*(gs->pos.x-tgs.x)+(gs->pos.y-tgs.y)*(gs->pos.y-tgs.y)) < 0.1);
   bool rs_placed = (sqrt((rs->pos.x-trs.x)*(rs->pos.x-trs.x)+(rs->pos.y-trs.y)*(rs->pos.y-trs.y)) < 0.1);
   bool rm_placed = (sqrt((rm->pos.x-trs.x)*(rm->pos.x-trs.x)+(rm->pos.y-trs.y)*(rm->pos.y-trs.y)) < 0.1);
   bool ys_placed = (sqrt((ys->pos.x-tys.x)*(ys->pos.x-tys.x)+(ys->pos.y-tys.y)*(ys->pos.y-tys.y)) < 0.1);

   mlr::Vector objPosition;
   if (hobjC==HRIObject::green){
     objPosition.x = tgs.x;
     objPosition.y = tgs.y;
     objPosition.z = gs->pos.z;
   } else if (hobjC==HRIObject::yellow){
     objPosition.x = tys.x;
     objPosition.y = tys.y;
     objPosition.z = ys->pos.z;
   } else if (hobjC==HRIObject::red){
     if (hobjS==HRIObject::small){
       objPosition.x = trs.x;
       objPosition.y = trs.y;
       objPosition.z = rs->pos.z;
       if (rm_placed){
         objPosition.z = rs->pos.z+0.04;
       }
     } else {
       objPosition.x = trs.x;
       objPosition.y = trs.y;
       objPosition.z = rm->pos.z;
       if (rs_placed){
         objPosition.z = rm->pos.z+0.04;
       }
     }
   } else if (hobjC==HRIObject::blue){
     if (hobjS==HRIObject::small){
       objPosition.x = tbs.x;
       objPosition.y = tbs.y;
       objPosition.z = bs->pos.z;
       if (bl_placed){
         objPosition.z = bs->pos.z+0.04;
       }
     } else {
       objPosition.x = tbs.x;
       objPosition.y = tbs.y;
       objPosition.z = bl->pos.z;
       if (bs_placed){
         objPosition.z = bl->pos.z+0.04;
       }
     }
   }
   state.setObjectPosition(hobjC,hobjS,objPosition);
   state.updateModelworldFromState();
}

std::string SortTask::getTargetID(std::string tpos) {
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

bool SortTask::placeObj(HRIObject* obj) {
  std::stringstream output;
  output<< "robot place object " << *obj;
  log(output.str());
  std::string targetid;
  int theta = 0;
  if (obj->size!=HRIObject::small){
    theta = 1;
  }
  if (obj->color==HRIObject::blue) {
    targetid = getTargetID("targetb").c_str();
    auto placeObj = R.placeDistDirTable(obj->id.c_str(),targetid.c_str(),"table",0.,0.,0.,theta);
    R.wait(+placeObj);
    return true;
  } else if (obj->color==HRIObject::green) {
    targetid = getTargetID("targetg").c_str();
    auto placeObj = R.placeDistDirTable(obj->id.c_str(),targetid.c_str(),"table",0.,0.,0.,theta);
    R.wait(+placeObj);
    return true;
  } else if (obj->color==HRIObject::red) {
    targetid = getTargetID("targetr").c_str();
    auto placeObj = R.placeDistDirTable(obj->id.c_str(),targetid.c_str(),"table",0.,0.,0.,theta);
    R.wait(+placeObj);
    return true;
  } else if (obj->color==HRIObject::yellow) {
    targetid=getTargetID("targety").c_str();
    auto placeObj = R.placeDistDirTable(obj->id.c_str(),targetid.c_str(),"table",0.,0.,0.,theta);
    R.wait(+placeObj);
    return true;
  }
  return false;
}

bool SortTask::isReachable(HRIObject* obj) {
  return obj->pos.x<.6;
}

void SortTask::performAction() {
  if (started==0){
    started=1;
    log("first action");
//    system("echo \"begin task\" | festival --tts");
//    system("rosrun sound_play say.py \"begin task\"");
      if (mode==0){
//        system("rosrun sound_play say.py \"autonomous strategy \"");
      } else if (mode==1){
//        system("rosrun sound_play say.py \"human commands strategy\"");
      } else if (mode==2){
//        system("rosrun sound_play say.py \"robot commands strategy\"");
      } else if (mode==3){
//        system("rosrun sound_play say.py \"information strategy\"");
      }
//        system("rosrun sound_play say.py \"sort task\"");
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

void SortTask::performActionAutonomous() {
  cout << "State: " << state;
  std::vector<HRIObject*> objs = nextActions(state);
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

void SortTask::performActionAutonomousPlus() {
  cout << "State: " << state;
  std::vector<HRIObject*> objs = nextActions(state);
  if (objs.size()==0){
    cout << "no actions available";
//    system("rosrun sound_play say.py \"task completed\"");
    log("task completed");
    int input=0;
    std::cin >> input;
  }
  int robotObj = -1;
  int humanObj = -1;
  for (unsigned int i=0; i<objs.size(); ++i) {
    if (isReachable(objs[i])) {
      if(robotObj==-1){
        robotObj=i;
      }
    }
  }
  for (unsigned int i=0; i<objs.size(); ++i) {
    if (!isReachable(objs[i])) {
      if (humanObj==-1) {
        if (robotObj==-1){
          humanObj=i;
        } else if (objs[i]->color!=objs[robotObj]->color){
          humanObj=i;
        }
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
  if (robotObj!=-1){
    auto graspObj = R.graspBox(objs[robotObj]->id.c_str(), objs[robotObj]->pos.y<0 ? LR_right : LR_left);
    R.wait(+graspObj);
    placeObj(objs[robotObj]);
    state.updateFromModelworld(R);
  } else {
    //delay for ~2 seconds
    R.wait(2);
  }
  // now update position of where the human will place the object
  if (humanObj!=-1){
    humanPlaceObject(state, objs[humanObj]->color,objs[humanObj]->size);
  }
}

void SortTask::performActionCommands() {
  cout << "State: " << state;
  std::vector<HRIObject*> objs = nextActions(state);
  int input=0;
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
void SortTask::performActionRobotCommands() {
  //2 stages - first, perform any possible actions autonomously,
  // then request human to perform any other actions
  bool objectPlaced = false;
  cout << "State: " << state;
  std::vector<HRIObject*> objs = nextActions(state);
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
//        system("rosrun sound_play say.py \"please place the small green block\"");
      } else if (objs[0]->color==HRIObject::yellow){
//        system("rosrun sound_play say.py \"please place the small yellow block\"");
      } else if (objs[0]->color==HRIObject::red){
        if (objs[0]->size==HRIObject::small){
//          system("rosrun sound_play say.py \"please place the small red block\"");
        } else {
//          system("rosrun sound_play say.py \"please place the medium red block\"");
        }
      } else if (objs[0]->color==HRIObject::blue){
        if (objs[0]->size==HRIObject::small){
//          system("rosrun sound_play say.py \"please place the small blue block\"");
        } else {
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


void SortTask::log(std::string s) {
  ofstream myfile;
  std::string filename="logfileSort" + std::to_string(mode) + ".txt";
  myfile.open (filename, std::ios::out | std::ios::app);
  std::time_t result = std::time(0);
  myfile << result << ", ";
  myfile << s << endl;
  myfile.close();
}
