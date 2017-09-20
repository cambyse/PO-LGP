#include "BridgeTask.h"
#include <iostream>
#include <ctime>

using std::cout;
using std::endl;

BridgeTask::BridgeTask(int m) {
  mode = m;
  p_time = std::time(nullptr);
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
  return true;
}

int BridgeTask::getBridgeState(OnTableState& s) {
  // query objects of bridge task
  HRIObject* bs = s.getObj(HRIObject::blue, HRIObject::small);
  HRIObject* bl = s.getObj(HRIObject::blue, HRIObject::large);
  HRIObject* rs = s.getObj(HRIObject::red, HRIObject::small);
  HRIObject* gs = s.getObj(HRIObject::green, HRIObject::small);
  HRIObject* ys = s.getObj(HRIObject::yellow, HRIObject::small);
  if (bs==NULL||rs==NULL||gs==NULL||ys==NULL||bl==NULL)
    return -1;

  // check if blue small and green small are near to goal position
  mlr::Vector pbs=bs->pos;
  mlr::Vector tbs=R.getK()().getShapeByName("targetbs")->X.pos;
  
  mlr::Vector pgs=gs->pos;
  mlr::Vector tgs=R.getK()().getShapeByName("targetgs")->X.pos;

  bool bs_placed = (sqrt((pbs.x-tbs.x)*(pbs.x-tbs.x)+(pbs.y-tbs.y)*(pbs.y-tbs.y)) < 0.1);
  bool gs_placed = (sqrt((pgs.x-tgs.x)*(pgs.x-tgs.x)+(pgs.y-tgs.y)*(pgs.y-tgs.y)) < 0.1);
  cout << bs_placed << " " << pbs << " " << tbs << " " << sqrt((pbs.x-tbs.x)*(pbs.x-tbs.x)+(pbs.y-tbs.y)*(pbs.y-tbs.y)) << endl;

  if (bl->pos.z>0.76 && !rstate.containsObj(bl))
    return 9;
  if (bs->isBelow(ys) && gs->isBelow(rs))
    return 8;
  if (gs_placed && bs->isBelow(ys))
    return 7;
  if (bs_placed && gs->isBelow(rs))
    return 6;
  if (bs->isBelow(ys))
    return 5;
  if (gs->isBelow(rs))
    return 3;
  if (bs_placed && gs_placed)
    return 4;
  if (bs_placed)
    return 2;
  if (gs_placed)
    return 1;
  return 0;
}

std::vector<HRIObject*> BridgeTask::futureActions(int bridgestate, OnTableState& s) {
static const bool state_trans[10][10]=
  {0, 1, 1, 0, 0, 0, 0, 0, 0, 0,
   0, 0, 0, 1, 1, 0, 0, 0, 0, 0,
   0, 0, 0, 0, 1, 1, 0, 0, 0, 0,
   0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
   0, 0, 0, 0, 0, 0, 1, 1, 0, 0,
   0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
   0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
   0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
   0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 
   0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; 
 std::vector<HRIObject*> result; 
 if (bridgestate==9)
   return result;
 std::vector<HRIObject*> objs = nextActions(bridgestate, state);
 result.insert(result.end(), objs.begin(), objs.end());
 for (int i=0;i<10;++i) {
   if (state_trans[bridgestate][i]==1) {
     std::vector<HRIObject*> fobjs = futureActions(i, state);
     for (unsigned int p=0; p<fobjs.size();++p)
       if(std::find(result.begin(), result.end(), fobjs[p]) == result.end() )
	 result.push_back(fobjs[p]);
   }
 }
 return result;
}
std::vector<HRIObject*> BridgeTask::nextActions(int bridgestate, OnTableState& s) {
  std::vector<HRIObject*> result; 
 // query objects of bridge task
  HRIObject* bs = s.getObj(HRIObject::blue, HRIObject::small);
  HRIObject* bl = s.getObj(HRIObject::blue, HRIObject::large);
  HRIObject* rs = s.getObj(HRIObject::red, HRIObject::small);
  HRIObject* gs = s.getObj(HRIObject::green, HRIObject::small);
  HRIObject* ys = s.getObj(HRIObject::yellow, HRIObject::small);
  if (bs==NULL||rs==NULL||gs==NULL||ys==NULL||bl==NULL)
    return result;
  switch (bridgestate) {
  case 0:
    result.push_back(gs);
    result.push_back(bs);
    break;
  case 1:
    result.push_back(bs);
    result.push_back(rs);
    break;
  case 2:
    result.push_back(gs);
    result.push_back(ys);
    break;
  case 3:
    result.push_back(bs);
    break;
  case 4:
    result.push_back(rs);
    result.push_back(ys);
    break;
  case 5:
    result.push_back(gs);
    break;
  case 6:
    result.push_back(ys);
    break;
  case 7:
    result.push_back(rs);
    break;
  case 8:
    result.push_back(bl);
    break;
  default:
    break;
  }
  return result;
}

bool BridgeTask::placeObj(HRIObject* obj) {
  std::stringstream output;
  output<< "place obj, " << *obj;
  log(output.str());
  if (obj->color==HRIObject::blue && obj->size==HRIObject::small) {
    cout << "place blue" << endl;
    auto placeObj = R.placeDistDir(obj->id.c_str(),"targetbs",0.,0.,0.,0);
    R.wait(+placeObj);
    return true;
  } else if (obj->color==HRIObject::green && obj->size == HRIObject::small) {
    cout << "place green" << endl;
    auto placeObj = R.placeDistDir(obj->id.c_str(),"targetgs",0.,0.,0.,0);
    R.wait(+placeObj);
    return true;
  } else if (obj->color==HRIObject::red && obj->size == HRIObject::small) {
    auto placeObj = R.placeDistDir(obj->id.c_str(),state.getObj(HRIObject::green, HRIObject::small)->id.c_str(),0.,0.,0.,0);
    R.wait(+placeObj);
    return true;
  } else if (obj->color==HRIObject::blue && obj->size == HRIObject::large) {
    //auto placeObj = R.placeDistDir(obj->id.c_str(),state.getObj(HRIObject::yellow, HRIObject::small)->id.c_str(),0.,-.1,0.,0);
    auto placeObj = R.placeDistDir(obj->id.c_str(),"targetbl",0.,0.,0.,1);
    R.wait(+placeObj);
    return true;
  } else if (obj->color==HRIObject::yellow && obj->size == HRIObject::small) {
    auto placeObj = R.placeDistDir(obj->id.c_str(),state.getObj(HRIObject::blue, HRIObject::small)->id.c_str(),0.,0.,0.,0);
    R.wait(+placeObj);
    return true;
  }
  return false;
}

bool BridgeTask::isReachable(HRIObject* obj) {
  return obj->pos.x<.8;
}

void BridgeTask::performAction() {
  log("bridge state, " + std::to_string(getBridgeState(state)));
  switch(mode) {
  case 0:
    performActionProactive();
    break;
  case 1:
    performActionAutonomous();
    break;
  case 2:
    performActionRequest();
    break;
  case 3:
    performActionCommands();
    break;
  case 4:
    performActionReactive();
    break;
  default:
    cout << mode << " is no correct interaction mode" << endl;
    break;
  }
}

void BridgeTask::performActionProactive() {
  int bridgestate = getBridgeState(state);
  cout << "State of Bridge Task: " << bridgestate << endl;
  cout << "State: " << state;
  std::vector<HRIObject*> objs = nextActions(bridgestate, state); 
  for (unsigned int i=0; i<objs.size(); ++i) {
    if (rstate.containsObj(objs[i])) {
      if (placeObj(objs[i])) {
	rstate.removeObj(objs[i]);
	state.updateFromModelworld(R);
	return;
      }
    } else if (isReachable(objs[i])) {
      if (rstate.right==NULL && rstate.left==NULL) {
	auto graspObj = R.graspBox(objs[i]->id.c_str(), objs[i]->pos.y<0 ? LR_right : LR_left);
	R.wait(+graspObj);
	placeObj(objs[i]);
	state.updateFromModelworld(R);
	return;
      } else if (rstate.right==NULL) {
	auto graspObj = R.graspBox(objs[i]->id.c_str(), LR_right);
	R.wait(+graspObj);
	placeObj(objs[i]);
	state.updateFromModelworld(R);
	return;
      } else if (rstate.left==NULL) {
	auto graspObj = R.graspBox(objs[i]->id.c_str(), LR_left);
	R.wait(+graspObj);
	placeObj(objs[i]);
	state.updateFromModelworld(R);
	return;
      }
    }
  }
  // proactively select an object
  cout << "proactive action selection" << endl;
  if (rstate.left==NULL && rstate.right==NULL) {
    std::vector<HRIObject*> fobjs = futureActions(bridgestate, state);
    for (unsigned int i=0; i<fobjs.size(); ++i) {
      cout << *fobjs[i] << endl;
      if (isReachable(fobjs[i])) {
	std::stringstream output;
	output<< "proactive pick, " << *fobjs[i];
	log(output.str());
	if (fobjs[i]->pos.y<0) {
	  auto graspObj = R.graspBox(fobjs[i]->id.c_str(), LR_right);
	  rstate.right=fobjs[i];
	  R.wait(+graspObj);
	} else {
	  auto graspObj = R.graspBox(fobjs[i]->id.c_str(), LR_left);
	  rstate.left=fobjs[i];
	  R.wait(+graspObj);
	}
	return;
      }
    }
  }
}


void BridgeTask::performActionAutonomous() {
  int bridgestate = getBridgeState(state);
  cout << "State of Bridge Task: " << bridgestate << endl;
  cout << "State: " << state;
  std::vector<HRIObject*> objs = nextActions(bridgestate, state);
  for (unsigned int i=0; i<objs.size(); ++i) {
    cout << "next object: " <<  objs[i]->id << endl;
    if (isReachable(objs[i])) {
      auto graspObj = R.graspBox(objs[i]->id.c_str(), objs[i]->pos.y<0 ? LR_right : LR_left);
	R.wait(+graspObj);
	placeObj(objs[i]);
	state.updateFromModelworld(R);
	break;
    }
  }
}

 void BridgeTask::performActionRequest() {
  int bridgestate = getBridgeState(state);
  cout << "State of Bridge Task: " << bridgestate << endl;
  cout << "State: " << state;
  std::vector<HRIObject*> objs = nextActions(bridgestate, state);
  for (unsigned int i=0; i<objs.size(); ++i) {
    cout << "next object: " <<  objs[i]->id << endl;

    if (isReachable(objs[i])) {
      cout << "Waiting for Request to pick and place object" << endl;
      cout << "0=redo perception, 1=perform action" << endl;
      int input=0;
      std::cin >> input;
      if (input==0) {
	return;
      }
      auto graspObj = R.graspBox(objs[i]->id.c_str(), objs[i]->pos.y<0 ? LR_right : LR_left);
      R.wait(+graspObj);
      placeObj(objs[i]);
      state.updateFromModelworld(R);
      break;
    }
  }
 }

 void BridgeTask::performActionCommands() {
   cout << "State: " << state;
   cout << "Which object do you want to grasp/place? (1-5), 0=redo perception" << endl;
   int input=0;
   std::cin >> input;
   if (input==0) {
     return;
   }
   std::string inID = "S" +  std::to_string(input);
   auto graspObj = R.graspBox(inID.c_str(), state.getObj(inID)->pos.y<0 ? LR_right : LR_left);
   R.wait(+graspObj);
   placeObj(state.getObj(inID));
   state.updateFromModelworld(R);
 }

 void BridgeTask::performActionReactive() {
   int bridgestate = getBridgeState(state);
   cout << "State of Bridge Task: " << bridgestate << endl;
   cout << "State: " << state;
   std::time_t time = std::time(nullptr);
   if (bridgestate!=p_state) {
     p_state=bridgestate;
     p_time=time;
   } else if(time-p_time>10) {  //Time window 10s
     std::vector<HRIObject*> objs = nextActions(bridgestate, state);
     for (unsigned int i=0; i<objs.size(); ++i) {
       cout << "next object: " <<  objs[i]->id << endl;
       
       if (isReachable(objs[i])) {
	 auto graspObj = R.graspBox(objs[i]->id.c_str(), objs[i]->pos.y<0 ? LR_right : LR_left);
	 R.wait(+graspObj);
	 placeObj(objs[i]);
	 state.updateFromModelworld(R);
	 break;
       }
     }
   }
 }

void BridgeTask::log(std::string s) {
  ofstream myfile;
  std::string filename="logfile_mode" + std::to_string(mode) + ".txt";
  myfile.open (filename, std::ios::out | std::ios::app);
  std::time_t result = std::time(0);
  myfile << result << ", ";
  myfile << s << endl;
  myfile.close();
}
