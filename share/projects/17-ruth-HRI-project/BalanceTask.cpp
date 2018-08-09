#include "BalanceTask.h"
#include <iostream>
#include <ctime>

using std::cout;
using std::endl;

BalanceTask::BalanceTask(int m) {
  mode = m;
  started = 0;
  currentState = 0;
  log("start");
}


bool BalanceTask::isPlausible(OnTableState& s) {
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

int BalanceTask::getStackState(OnTableState& s) {
  int newStackState = 0;
  // query objects of stack task
  HRIObject* bs = s.getObj(HRIObject::blue, HRIObject::small);
  HRIObject* bl = s.getObj(HRIObject::blue, HRIObject::large);
  HRIObject* rs = s.getObj(HRIObject::red, HRIObject::small);
  HRIObject* rm = s.getObj(HRIObject::red, HRIObject::medium);
  HRIObject* gs = s.getObj(HRIObject::green, HRIObject::small);
  HRIObject* ys = s.getObj(HRIObject::yellow, HRIObject::small);
  if (bs==NULL||rs==NULL||rm==NULL||gs==NULL||ys==NULL||bl==NULL){
    newStackState=-1;
  } else {

    // check if red medium is near to goal position
    mlr::Vector prm=rm->pos;
    mlr::Vector trm=R.getK()().getShapeByName("target")->X.pos;
    bool rm_placed = (sqrt((prm.x-trm.x)*(prm.x-trm.x)+(prm.y-trm.y)*(prm.y-trm.y)) < 0.1);

    // check if green small is near to goal position on top of blue large
    mlr::Vector pgs=gs->pos;
    mlr::Vector tgs=R.getK()().getShapeByName("targetg1")->X.pos;
    bool gs_placed = (sqrt((pgs.x-tgs.x)*(pgs.x-tgs.x)+(pgs.y-tgs.y)*(pgs.y-tgs.y)) < 0.1);

    // check if yellow small is near to goal position on top of blue large
    mlr::Vector pys=ys->pos;
    mlr::Vector tys=R.getK()().getShapeByName("targety1")->X.pos;
    bool ys_placed = (sqrt((pys.x-tys.x)*(pys.x-tys.x)+(pys.y-tys.y)*(pys.y-tys.y)) < 0.1);

    if (gs->isBelow(bs)&&ys->isBelow(rs)){
      newStackState=4;
    } else if (gs_placed && ys_placed) {
      newStackState=3;
    } else if (rm->isBelow(bl)) {
      newStackState=2;
    } else if (rm_placed){
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

std::vector<HRIObject*> BalanceTask::nextActions(int stackstate, OnTableState& s) {
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
     result.push_back(bl);
     break;
   case 2:
     result.push_back(gs);
     result.push_back(ys);
     break;
   case 3:
     result.push_back(bs);
     result.push_back(rs);
     break;
   default:
     break;
   }
   return result;
}

void BalanceTask::humanPlaceObject(OnTableState& s, HRIObject::Color hobjC, HRIObject::Size hobjS){
  // query objects of stack task
   HRIObject* bs = s.getObj(HRIObject::blue, HRIObject::small);
   HRIObject* bl = s.getObj(HRIObject::blue, HRIObject::large);
   HRIObject* rs = s.getObj(HRIObject::red, HRIObject::small);
   HRIObject* rm = s.getObj(HRIObject::red, HRIObject::medium);
   HRIObject* gs = s.getObj(HRIObject::green, HRIObject::small);
   HRIObject* ys = s.getObj(HRIObject::yellow, HRIObject::small);
   mlr::Vector trm=R.getK()().getShapeByName("target")->X.pos;
   mlr::Vector tgs=R.getK()().getShapeByName("targetg1")->X.pos;
   mlr::Vector tys=R.getK()().getShapeByName("targety1")->X.pos;

   // red        blue
   // yellow     green
   // large blue
   // medium red

   mlr::Vector objPosition;
   if (hobjC==HRIObject::green){
     objPosition.x = tgs.x;
     objPosition.y = tgs.y;
     objPosition.z = bl->pos.z+0.04;
   } else if (hobjC==HRIObject::yellow){
     objPosition.x = tys.x;
     objPosition.y = tys.y;
     objPosition.z = bl->pos.z+0.04;
   } else if (hobjC==HRIObject::red){
     if (hobjS==HRIObject::small){
       objPosition.x = ys->pos.x;
       objPosition.y = ys->pos.y;
       objPosition.z = ys->pos.z+0.04;
     } else {
       objPosition.x = trm.x;
       objPosition.y = trm.y;
       objPosition.z = rm->pos.z;
     }
   } else if (hobjC==HRIObject::blue){
     if (hobjS==HRIObject::small){
       objPosition.x = gs->pos.x;
       objPosition.y = gs->pos.y;
       objPosition.z = gs->pos.z+0.04;
     } else {
       objPosition.x = rm->pos.x;
       objPosition.y = rm->pos.y;
       objPosition.z = rm->pos.z+0.04;
     }
   }
   state.setObjectPosition(hobjC,hobjS,objPosition);
   state.updateModelworldFromState();
}

bool BalanceTask::placeObj(HRIObject* obj) {
  // in this task, placeObj is only called for placement of single objects ie large blue and medium red
  std::stringstream output;
  output<< "robot place object " << *obj;
  log(output.str());
  if (obj->color==HRIObject::blue && obj->size==HRIObject::large) {
    auto placeObj = R.placeDistDirTable(obj->id.c_str(),state.getObj(HRIObject::red, HRIObject::medium)->id.c_str(),"table",0.,0.,0.,1);
    R.wait(+placeObj);
    return true;
  } else if (obj->color==HRIObject::red && obj->size == HRIObject::medium) {
    auto placeObj = R.placeDistDirTable(obj->id.c_str(),"target","table",0.,0.,0.,1);
    R.wait(+placeObj);
    return true;
  } else if (obj->color==HRIObject::green&& obj->size==HRIObject::small) {
    auto placeObj = R.placeDistDirTable(obj->id.c_str(),state.getObj(HRIObject::blue, HRIObject::large)->id.c_str(),"table",0.,0.10,0.,0);
    R.wait(+placeObj);
    return true;
  } else if (obj->color==HRIObject::blue&& obj->size == HRIObject::small) {
    auto placeObj = R.placeDistDirTable(obj->id.c_str(),state.getObj(HRIObject::green, HRIObject::small)->id.c_str(),"table",0.,0.,0.,0);
    R.wait(+placeObj);
    return true;
  } else if (obj->color==HRIObject::red && obj->size == HRIObject::small) {
    auto placeObj = R.placeDistDirTable(obj->id.c_str(),state.getObj(HRIObject::yellow, HRIObject::small)->id.c_str(),"table",0.,0.,0.,0);
    R.wait(+placeObj);
    return true;
  } else if (obj->color==HRIObject::yellow && obj->size == HRIObject::small) {
    auto placeObj = R.placeDistDirTable(obj->id.c_str(),state.getObj(HRIObject::blue, HRIObject::large)->id.c_str(),"table",0.,-0.10,0.,0);
    R.wait(+placeObj);
    return true;
  }
  return false;
}

bool BalanceTask::placeObjs(HRIObject* obj, HRIObject* obj2) {
  // to place both of a pair of balanced objects (blue and red, or green and yellow)
  std::stringstream output;
  output<< "robot place 2 objects " << *obj << "; " << *obj2;
  log(output.str());
  if (obj->color==HRIObject::red&& obj2->color==HRIObject::blue) {
    auto placeObj = R.placeDistDirTable(obj->id.c_str(),state.getObj(HRIObject::yellow, HRIObject::small)->id.c_str(),"table",0.,0.,0.,0);
    auto placeObj2 = R.placeDistDirTable(obj2->id.c_str(),state.getObj(HRIObject::green, HRIObject::small)->id.c_str(),"table",0.,0.,0.,0);
    R.wait({-placeObj, -placeObj2});
    return true;
  } else if (obj->color==HRIObject::yellow&& obj2->color==HRIObject::green) {
    auto placeObj = R.placeDistDirTable(obj->id.c_str(),state.getObj(HRIObject::blue, HRIObject::large)->id.c_str(),"table",0.,-0.10,0.,0);
    auto placeObj2 = R.placeDistDirTable(obj2->id.c_str(),state.getObj(HRIObject::blue, HRIObject::large)->id.c_str(),"table",0.,0.10,0.,0);
    R.wait({-placeObj, -placeObj2});
    return true;
  }
  return false;
}

bool BalanceTask::placeObjWait(HRIObject* obj) {
  // to place one of a pair of balanced objects, and wait for the human to place the other
  std::stringstream output;
  output<< "robot place balanced object " << *obj;
  log(output.str());
//  if (mode==3){
//    system("rosrun sound_play say.py \"please tell me when to release the block\"");
//    cout << "please tell me when to release the block";
//  }
  if (obj->color==HRIObject::red&& obj->size==HRIObject::small) {
    // hold and wait for the human to place the blue small object
    auto holdObj = R.holdDistDir(obj->id.c_str(),state.getObj(HRIObject::yellow, HRIObject::small)->id.c_str(),0.,0.,0.01,0.0);
    if (mode==3){
      cout << "wait for input:";
      int temp=0;
      std::cin >> temp;
//      R.wait();
    } else {
      R.wait(+holdObj);
      // wait for object to be in the correct place
      if (mode==2){
        //request human to place object
        requestObject(state.getObj(HRIObject::blue,HRIObject::small));
      }
      waitForObjectPlaced(HRIObject::blue,HRIObject::small,"targetg1");
    }
    auto placeObj = R.releaseDistDir(obj->id.c_str(),state.getObj(HRIObject::yellow, HRIObject::small)->id.c_str(),"table",0.,0.,0.,0);
    R.wait(+placeObj);
    return true;
  } else if (obj->color==HRIObject::blue&& obj->size == HRIObject::small) {
    // hold and wait for the human to place the green small object
    auto holdObj = R.holdDistDir(obj->id.c_str(),state.getObj(HRIObject::green, HRIObject::small)->id.c_str(),0.,0.,0.01,0.0);
    if (mode==3){
      cout << "wait for input:";
      int temp=0;
      std::cin >> temp;
//      R.wait();
    } else {
      R.wait(+holdObj);
      // wait for object to be in the correct place
      if (mode==2){
        //request human to place object
        requestObject(state.getObj(HRIObject::red,HRIObject::small));
      }
      waitForObjectPlaced(HRIObject::red,HRIObject::small,"targety1");
    }
    auto placeObj = R.releaseDistDir(obj->id.c_str(),state.getObj(HRIObject::green, HRIObject::small)->id.c_str(),"table",0.,0.,0.,0);
    R.wait(+placeObj);
    return true;
  } else if (obj->color==HRIObject::green && obj->size == HRIObject::small) {
    // hold and wait for the human to place the yellow small object
    auto holdObj = R.holdDistDir(obj->id.c_str(),state.getObj(HRIObject::blue, HRIObject::large)->id.c_str(),0.,0.10,0.01,0.0);
    if (mode==3){
      cout << "wait for input:";
      int temp=0;
      std::cin >> temp;
//      R.wait();
    } else {
      R.wait(+holdObj);
      // wait for object to be in the correct place
      if (mode==2){
        //request human to place object
        requestObject(state.getObj(HRIObject::yellow,HRIObject::small));
      }
      waitForObjectPlaced(HRIObject::yellow,HRIObject::small,"targety1");
    }
    auto placeObj = R.releaseDistDir(obj->id.c_str(),state.getObj(HRIObject::blue, HRIObject::large)->id.c_str(),"table",0.,0.10,0.,0);
    R.wait(+placeObj);
    return true;
  } else if (obj->color==HRIObject::yellow && obj->size == HRIObject::small) {
    // hold and wait for the human to place the green small object
    auto holdObj = R.holdDistDir(obj->id.c_str(),state.getObj(HRIObject::blue, HRIObject::large)->id.c_str(),0.,-0.10,0.01,0.0);
    if (mode==3){
      cout << "wait for input:";
      int temp=0;
      std::cin >> temp;
//      R.wait();
    } else {
      R.wait(+holdObj);
      // wait for object to be in the correct place
      if (mode==2){
        //request human to place object
        requestObject(state.getObj(HRIObject::green,HRIObject::small));
      }
      waitForObjectPlaced(HRIObject::green,HRIObject::small,"targetg1");
    }
    auto placeObj = R.releaseDistDir(obj->id.c_str(),state.getObj(HRIObject::blue, HRIObject::large)->id.c_str(),"table",0.,-0.10,0.,0);
    R.wait(+placeObj);
    return true;
  }
  return false;
}

bool BalanceTask::isReachable(HRIObject* obj) {
  return obj->pos.x<.6;
}

void BalanceTask::performAction() {
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
//      system("rosrun sound_play say.py \"balance task\"");
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



bool BalanceTask::performActionAutonomous() {
  bool objectPlaced = false;
  int stackstate = getStackState(state);
  cout << "State: " << state;
  std::vector<HRIObject*> objs = nextActions(stackstate, state);
  if (objs.size()==0){
    cout << "no actions available";
//    system("echo \"task completed\" | festival --tts");
//    system("rosrun sound_play say.py \"task completed\"");
    log("task completed");
    int input=0;
    std::cin >> input;
  }
  if (objs.size()==1){
    if (isReachable(objs[0])){
      auto graspObj = R.graspBox(objs[0]->id.c_str(), objs[0]->pos.y<0 ? LR_right : LR_left);
      R.wait(+graspObj);
      placeObj(objs[0]);
      state.updateFromModelworld(R);
      objectPlaced = true;
    }
  } else if (objs.size()==2){
    if (isReachable(objs[0])&&isReachable(objs[1])){
      // pick up both objects and place both objects
      if (objs[0]->color==HRIObject::red||objs[0]->color==HRIObject::yellow){
        auto graspObj = R.graspBox(objs[0]->id.c_str(), LR_right);
        R.wait(+graspObj);
        auto AN = R.armsNeutral();
        R.wait(+AN);
        auto graspObj2 = R.graspBox(objs[1]->id.c_str(), LR_left);
        R.wait(+graspObj2);
        auto AN2 = R.armsNeutral();
        R.wait(+AN2);
        placeObjs(objs[0],objs[1]);
      } else {
        auto graspObj = R.graspBox(objs[1]->id.c_str(), LR_right);
        R.wait(+graspObj);
        auto AN = R.armsNeutral();
        R.wait(+AN);
        auto graspObj2 = R.graspBox(objs[0]->id.c_str(), LR_left);
        R.wait(+graspObj2);
        auto AN2 = R.armsNeutral();
        R.wait(+AN2);
        placeObjs(objs[1],objs[0]);
      }
      state.updateFromModelworld(R);
      objectPlaced = true;
    } else if (isReachable(objs[0])){
      // and wait for human to place objs[1]
      LeftOrRight hand = LR_right;
      if (objs[0]->color==HRIObject::blue||objs[0]->color==HRIObject::green){
        hand = LR_left;
      }
      auto graspObj = R.graspBox(objs[0]->id.c_str(), hand);
      R.wait(+graspObj);
      placeObjWait(objs[0]);
//      placeObj(objs[0]);
      state.updateFromModelworld(R);
      objectPlaced = true;
    } else if (isReachable(objs[1])){
      // and wait for human to place objs[0]
      LeftOrRight hand = LR_right;
      if (objs[1]->color==HRIObject::blue||objs[1]->color==HRIObject::green){
        hand = LR_left;
      }
      auto graspObj = R.graspBox(objs[1]->id.c_str(), hand);
      R.wait(+graspObj);
      placeObjWait(objs[1]);
//      placeObj(objs[1]);
      state.updateFromModelworld(R);
      objectPlaced = true;
    }
  }
  return objectPlaced;
}

bool BalanceTask::performActionAutonomousPlus() {
  bool objectPlaced = false;
  int stackstate = getStackState(state);
  cout << "State: " << state;
  std::vector<HRIObject*> objs = nextActions(stackstate, state);
  if (objs.size()==0){
    cout << "no actions available";
//    system("echo \"task completed\" | festival --tts");
//    system("rosrun sound_play say.py \"task completed\"");
    log("task completed");
    int input=0;
    std::cin >> input;
  }
  int robotObj1 = -1;
  int robotObj2 = -1;
  int humanObj1 = -1;
  int humanObj2 = -1;
  int humanObj3 = -1;
  if (objs.size()==1){
    if (isReachable(objs[0])){
      robotObj1 = 0;
    } else {
      humanObj1 = 0;
    }
  } else if (objs.size()==2){
    if (isReachable(objs[0])&&isReachable(objs[1])){
      robotObj1 = 0;
      robotObj2 = 1;
    } else if (isReachable(objs[0])){
      robotObj1 = 0;
      humanObj1 = 1;
    } else if (isReachable(objs[1])){
      robotObj1 = 1;
      humanObj1 = 0;
    } else {
      humanObj1 = 0;
      humanObj2 = 1;
    }
  }
  // make requests of human:
  if ((robotObj1==-1)&&(humanObj1!=-1)){
    // only human should place an object
    std::stringstream request;
    request << "Can you please place the ";
    if (objs[humanObj1]->color==HRIObject::green){
//      system("rosrun sound_play say.py \"please place the small green block\"");
      request << "small green ";
    } else if (objs[humanObj1]->color==HRIObject::yellow){
//      system("rosrun sound_play say.py \"please place the small yellow block\"");
      request << "small yellow ";
    } else if (objs[humanObj1]->color==HRIObject::red){
      if (objs[humanObj1]->size==HRIObject::small){
//        system("rosrun sound_play say.py \"please place the small red block\"");
        request << "small red ";
      } else {
//        system("rosrun sound_play say.py \"please place the medium red block\"");
        request << "medium red ";
      }
    } else if (humanObj1!=-1){
      if (objs[humanObj1]->color==HRIObject::blue){
        if (objs[humanObj1]->size==HRIObject::small){
//          system("rosrun sound_play say.py \"please place the small blue block\"");
          request << "small blue ";
        } else {
//          system("rosrun sound_play say.py \"please place the large blue block\"");
          request << "large blue ";
        }
      }
    }
    request << "block";
    cout << request.str();
  } else if ((robotObj2==-1)&&(humanObj1==-1)){
    // robot should place an object, and human should place the large blue object next
    std::stringstream request;
//          system("rosrun sound_play say.py \"After I place this block, can you please place the large blue block\"");
    request << "After I place this block, can you please place the large blue block";
    cout << request.str();
    humanObj3 = 0;
  } else if ((robotObj1!=-1)&&(humanObj1!=-1)){
    // human and robot should place at the same time
    std::stringstream request;
    request << "Can you please place the ";
    if (objs[humanObj1]->color==HRIObject::green){
//      system("rosrun sound_play say.py \"please place the small green block and tell me when to release the block\"");
      request << "small green ";
    } else if (objs[humanObj1]->color==HRIObject::yellow){
//      system("rosrun sound_play say.py \"please place the small yellow block and tell me when to release the block\"");
      request << "small yellow ";
    } else if (objs[humanObj1]->color==HRIObject::red){
      if (objs[humanObj1]->size==HRIObject::small){
//        system("rosrun sound_play say.py \"please place the small red block and tell me when to release the block\"");
        request << "small red ";
      } else {
//        system("rosrun sound_play say.py \"please place the medium red block\"");
        request << "medium red ";
      }
    } else if (objs[humanObj1]->color==HRIObject::blue){
      if (objs[humanObj1]->size==HRIObject::small){
//        system("rosrun sound_play say.py \"please place the small blue block and tell me when to release the block\"");
        request << "small blue ";
      } else {
//        system("rosrun sound_play say.py \"please place the large blue block\"");
        request << "large blue ";
      }
    }
    request << "block";
    cout << request.str();
  } else if ((robotObj1==-1)&&(humanObj2!=-1)){
    // human should place 2 objects
    std::stringstream request;
    request << "Can you please place the ";
    if (objs[0]->color==HRIObject::yellow||objs[1]->color==HRIObject::yellow){
//      system("rosrun sound_play say.py \"please place the small yellow block and the small green block\"");
      request << "small yellow block and the small green block";
    } else if (objs[0]->color==HRIObject::blue||objs[1]->color==HRIObject::blue){
//      system("rosrun sound_play say.py \"please place the small red block and the small blue block\"");
      request << "small red block and the small blue block";
    }
    cout << request.str();
  }
  // do actions:
  // possible addition: add that human should specify when robot lets go for balance actions
  if ((robotObj1!=-1)&&(robotObj2==-1)&&(humanObj1==-1))  {
    // only the robot should place an object
    auto graspObj = R.graspBox(objs[0]->id.c_str(), objs[0]->pos.y<0 ? LR_right : LR_left);
    R.wait(+graspObj);
    placeObj(objs[0]);
    state.updateFromModelworld(R);
    objectPlaced = true;
  } else if ((robotObj1!=-1)&&(humanObj1!=-1)){
    // human and robot should place at the same time
    LeftOrRight hand = LR_right;
    if (objs[robotObj1]->color==HRIObject::blue||objs[robotObj1]->color==HRIObject::green){
      hand = LR_left;
    }
    auto graspObj = R.graspBox(objs[robotObj1]->id.c_str(), hand);
    R.wait(+graspObj);
    placeObjWait(objs[robotObj1]);
//    placeObj(objs[robotObj1]);
    state.updateFromModelworld(R);
    objectPlaced = true;
  } else if ((robotObj1!=-1)&&(robotObj2!=-1)){
    // robot should place 2 objects
    if (objs[robotObj1]->color==HRIObject::red||objs[robotObj1]->color==HRIObject::yellow){
      auto graspObj = R.graspBox(objs[robotObj1]->id.c_str(), LR_right);
      R.wait(+graspObj);
      auto AN = R.armsNeutral();
      R.wait(+AN);
      auto graspObj2 = R.graspBox(objs[robotObj2]->id.c_str(), LR_left);
      R.wait(+graspObj2);
      auto AN2 = R.armsNeutral();
      R.wait(+AN2);
      placeObjs(objs[robotObj1],objs[robotObj2]);
    } else {
      auto graspObj = R.graspBox(objs[robotObj2]->id.c_str(), LR_right);
      R.wait(+graspObj);
      auto AN = R.armsNeutral();
      R.wait(+AN);
      auto graspObj2 = R.graspBox(objs[robotObj1]->id.c_str(), LR_left);
      R.wait(+graspObj2);
      auto AN2 = R.armsNeutral();
      R.wait(+AN2);
      placeObjs(objs[robotObj2],objs[robotObj1]);
    }
    state.updateFromModelworld(R);
    objectPlaced = true;
  }

  if (robotObj1==-1){
    R.wait(2);
  }

  if (humanObj1!=-1){
    humanPlaceObject(state, objs[humanObj1]->color,objs[humanObj1]->size);
  }
  if (humanObj2!=-1){
    humanPlaceObject(state, objs[humanObj2]->color,objs[humanObj2]->size);
  }
  if (humanObj3!=-1){
    humanPlaceObject(state, HRIObject::blue,HRIObject::large);
  }
  state.updateFromModelworld(R);

  return objectPlaced;
}

void BalanceTask::performActionCommands() {
  int stackstate = getStackState(state);
  cout << "State: " << state;
  std::vector<HRIObject*> objs = nextActions(stackstate, state);
  int input=0;
  if (objs.size()>0){
    cout << "Which object/s do you want me to place? (1-6), 0=redo perception" << endl;
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
  int numObjects = 0;
  int obj1 = -1;
  int obj2 = -1;
  if (input<10){
    obj1=input;
    numObjects = 1;
  } else {
    obj1 = input/10;
    obj2 = input%10;
    numObjects = 2;
  }
  // could check if more than one object has been requested, if so, check both objects for available action
  std::string inID1 = "S" +  std::to_string(obj1);
  std::string inID2 = "S" +  std::to_string(obj2);
  bool availableAction1 = false;
  bool availableAction2 = false;
  for (unsigned int i=0; i<objs.size(); ++i) {
    if (objs[i]->id == inID1){
      availableAction1 = true;
    }
    if (objs[i]->id == inID2){
      availableAction2 = true;
    }
  }
  if ((objs.size()==1) && (numObjects==1) && availableAction1 && isReachable(state.getObj(inID1))) {
    auto graspObj = R.graspBox(inID1.c_str(), state.getObj(inID1)->pos.y<0 ? LR_right : LR_left);
    R.wait(+graspObj);
    placeObj(state.getObj(inID1));
    state.updateFromModelworld(R);
  } else if ((objs.size()==2) && (numObjects==2) && availableAction1 && availableAction2 && isReachable(state.getObj(inID1)) && isReachable(state.getObj(inID2))){
    if (state.getObj(inID1)->color==HRIObject::red||state.getObj(inID1)->color==HRIObject::yellow){
      auto graspObj = R.graspBox(inID1.c_str(), LR_right);
      R.wait(+graspObj);
      auto AN = R.armsNeutral();
      R.wait(+AN);
      auto graspObj2 = R.graspBox(inID2.c_str(), LR_left);
      R.wait(+graspObj2);
      auto AN1 = R.armsNeutral();
      R.wait(+AN1);
      placeObjs(state.getObj(inID1),state.getObj(inID2));
    } else {
      auto graspObj = R.graspBox(inID2.c_str(), LR_right);
      R.wait(+graspObj);
      auto AN = R.armsNeutral();
      R.wait(+AN);
      auto graspObj2 = R.graspBox(inID1.c_str(), LR_left);
      R.wait(+graspObj2);
      auto AN1 = R.armsNeutral();
      R.wait(+AN1);
      placeObjs(state.getObj(inID2),state.getObj(inID1));
    }
    state.updateFromModelworld(R);
  } else if ((objs.size()==2) && (numObjects==1) && availableAction1 && isReachable(state.getObj(inID1))){
    // what if human asks to place one object, but actually 2 objects are in reach? - do nothing, ask for another instruction
    bool reachOtherObj = false;
    if (objs[0]->id == inID1){
      if (isReachable(objs[1])){
        reachOtherObj = true;
      }
    } else {
      if (isReachable(objs[0])){
        reachOtherObj = true;
      }
    }
    if (!reachOtherObj){
      LeftOrRight hand = LR_right;
      if (state.getObj(inID1)->color==HRIObject::blue||state.getObj(inID1)->color==HRIObject::green){
        hand = LR_left;
      }
      auto graspObj = R.graspBox(inID1.c_str(), hand);
      R.wait(+graspObj);
      placeObjWait(state.getObj((inID1.c_str())));
      state.updateFromModelworld(R);
    }
  }
}

void BalanceTask::requestObject(HRIObject* obj){
  if (obj->color==HRIObject::green){
//    system("echo \"please place the small green block\" | festival --tts");
//    system("rosrun sound_play say.py \"please place the small green block\"");
  } else if (obj->color==HRIObject::yellow){
//    system("echo \"please place the small yellow block\" | festival --tts");
//    system("rosrun sound_play say.py \"please place the small yellow block\"");
  } else if (obj->color==HRIObject::red){
    if (obj->size==HRIObject::small){
//      system("echo \"please place the small red block\" | festival --tts");
//      system("rosrun sound_play say.py \"please place the small red block\"");
    } else {
      system("echo \"please place the medium red block\" | festival --tts");
//      system("rosrun sound_play say.py \"please place the medium red block\"");
    }
  } else if (obj->color==HRIObject::blue){
    if (obj->size==HRIObject::small){
//      system("echo \"please place the small blue block\" | festival --tts");
//      system("rosrun sound_play say.py \"please place the small blue block\"");
    } else {
//      system("echo \"please place the large blue block\" | festival --tts");
//      system("rosrun sound_play say.py \"please place the large blue block\"");
    }
  }
  std::stringstream request;
  request << "Can you please place the ";
  switch (obj->size) {
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
  switch (obj->color){
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
}

void BalanceTask::waitForObjectPlaced(HRIObject::Color c1, HRIObject::Size s1, const char* targetName){
  mlr::Vector targetPos=R.getK()().getShapeByName(targetName)->X.pos;
  bool objectPlaced = false;
  OnTableState state2;
  Access<PerceptL> outputs("percepts_input");
  int rev=outputs.getRevision();
  outputs.waitForRevisionGreaterThan(rev+2);
  state2.updateFromModelworld(R);
  if (mode!=0){
    while (!objectPlaced) {
      objectPlaced = state2.findObjectInPercepts(outputs.get(),c1,s1,targetPos);
    }
  }
  cout<<"object has been placed";
  HRIObject* obj = state.getObj(c1, s1);
  HRIObject* obj2 = state.getObj(HRIObject::blue, HRIObject::large);
  mlr::Vector objPosition;
  objPosition.x = targetPos.x;
  objPosition.y = targetPos.y;
  if (c1==HRIObject::green || c1==HRIObject::yellow){
    objPosition.z = obj2->pos.z+0.04;
  } else {
    objPosition.z = obj2->pos.z+0.08;
  }
  state.setObjectPosition(c1,s1,objPosition);
  state.updateModelworldFromState();
}


void BalanceTask::performActionRobotCommands(){
  bool objectPlaced = performActionAutonomous();
  if (!objectPlaced){
    // no action performed by robot, see if an action can be performed by the human
    int stackstate = getStackState(state);
    std::stringstream output;
    std::vector<HRIObject*> objs = nextActions(stackstate, state);
    if (objs.size()==0){
      cout << "no actions available";
//      system("echo \"task completed\" | festival --tts");
//      system("rosrun sound_play say.py \"task completed\"");
      log("task completed");
      int input=0;
      std::cin >> input;
    } else if (objs.size()>0){
      if (objs.size()==1){
        if (objs[0]->color==HRIObject::green){
//          system("echo \"please place the small green block\" | festival --tts");
//          system("rosrun sound_play say.py \"please place the small green block\"");
        } else if (objs[0]->color==HRIObject::yellow){
//          system("echo \"please place the small yellow block\" | festival --tts");
//          system("rosrun sound_play say.py \"please place the small yellow block\"");
        } else if (objs[0]->color==HRIObject::red){
          if (objs[0]->size==HRIObject::small){
//            system("echo \"please place the small red block\" | festival --tts");
//            system("rosrun sound_play say.py \"please place the small red block\"");
          } else {
//            system("echo \"please place the medium red block\" | festival --tts");
//            system("rosrun sound_play say.py \"please place the medium red block\"");
          }
        } else if (objs[0]->color==HRIObject::blue){
          if (objs[0]->size==HRIObject::small){
//            system("echo \"please place the small blue block\" | festival --tts");
//            system("rosrun sound_play say.py \"please place the small blue block\"");
          } else {
//            system("echo \"please place the large blue block\" | festival --tts");
//            system("rosrun sound_play say.py \"please place the large blue block\"");
          }
        }
      } else if (objs.size()==2){
        if (objs[0]->color==HRIObject::yellow||objs[1]->color==HRIObject::yellow){
//          system("echo \"please place the small yellow block and the small green block\" | festival --tts");
//          system("rosrun sound_play say.py \"please place the small yellow block and the small green block\"");
        } else if (objs[0]->color==HRIObject::blue||objs[1]->color==HRIObject::blue){
//          system("echo \"please place the small green block and the small blue block\" | festival --tts");
//          system("rosrun sound_play say.py \"please place the small green block and the small blue block\"");
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
      if (objs.size()==2){
        request << "object, and the ";
        switch (objs[1]->size) {
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
        switch (objs[1]->color){
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
      }
      request << "object";
      cout << request.str();
      R.wait(5);
//      int input=0;
//      std::cin >> input;
    }
  }
}

void BalanceTask::log(std::string s) {
  ofstream myfile;
  std::string filename="logfileBalance" + std::to_string(mode) + ".txt";
  myfile.open (filename, std::ios::out | std::ios::app);
  std::time_t result = std::time(0);
  myfile << result << ", ";
  myfile << s << endl;
  myfile.close();
}
