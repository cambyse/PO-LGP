#include "OrderingTask.h"
#include <iostream>
#include <ctime>

using std::cout;
using std::endl;

OrderingTask::OrderingTask(int m) {
  mode = m;
}


bool OrderingTask::isPlausible(OnTableState& s) {
  // check count of objects of one kind
  if (s.countObjs(HRIObject::blue, HRIObject::small) > 1)
    return false;
  if (s.countObjs(HRIObject::red, HRIObject::small) > 3)
    return false;
  if (s.countObjs(HRIObject::green, HRIObject::small) > 3)
    return false;
  if (s.countObjs(HRIObject::yellow, HRIObject::small) > 2)
    return false;
  return true;
}

std::vector<HRIObject*> OrderingTask::nextActions(OnTableState& s) {
  std::vector<HRIObject*> result; 

  for (unsigned int i=0;i<s.objects.size();++i) {
    HRIObject* obj = &s.objects[i];
    // check whether near to target position
    mlr::Vector tbs=R.getK()().getShapeByName("targetbs")->X.pos;
    mlr::Vector tgs=R.getK()().getShapeByName("targetgs")->X.pos;
    mlr::Vector trs=R.getK()().getShapeByName("targetrs")->X.pos;
    mlr::Vector tys=R.getK()().getShapeByName("targetys")->X.pos;
    bool bs_placed = (sqrt((obj->pos.x-tbs.x)*(obj->pos.x-tbs.x)+(obj->pos.y-tbs.y)*(obj->pos.y-tbs.y)) < 0.1);
    bool gs_placed = (sqrt((obj->pos.x-tgs.x)*(obj->pos.x-tgs.x)+(obj->pos.y-tgs.y)*(obj->pos.y-tgs.y)) < 0.1);
    bool rs_placed = (sqrt((obj->pos.x-trs.x)*(obj->pos.x-trs.x)+(obj->pos.y-trs.y)*(obj->pos.y-trs.y)) < 0.1);
    bool ys_placed = (sqrt((obj->pos.x-tys.x)*(obj->pos.x-tys.x)+(obj->pos.y-tys.y)*(obj->pos.y-tys.y)) < 0.1);


    if (obj->color==HRIObject::blue && obj->size==HRIObject::small && !bs_placed) {
      result.push_back(obj);
    } else if (obj->color==HRIObject::green && obj->size==HRIObject::small && !gs_placed) {
      result.push_back(obj);
    } else if (obj->color==HRIObject::red && obj->size==HRIObject::small && !rs_placed) {
      result.push_back(obj);
    } else if (obj->color==HRIObject::yellow && obj->size==HRIObject::small && !ys_placed) {
      result.push_back(obj);
    }
  }
  return result;
}

std::string OrderingTask::getTargetID(std::string tpos) {
  std::string targetname = tpos;
  HRIObject* target = NULL;
    for (HRIObject o : state.objects) {
      mlr::Vector pbs=o.pos;
      mlr::Vector tbs=R.getK()().getShapeByName(tpos.c_str())->X.pos;
      if (sqrt((pbs.x-tbs.x)*(pbs.x-tbs.x)+(pbs.y-tbs.y)*(pbs.y-tbs.y)) < 0.1) {
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

bool OrderingTask::placeObj(HRIObject* obj) {
  std::stringstream output;
  output<< "place obj, " << *obj;
  log(output.str());
  cout << state << endl;
  cout << "place obj, " << *obj << endl;
  std::string targetid;
  if (obj->color==HRIObject::blue && obj->size==HRIObject::small) {
    targetid = getTargetID("targetbs").c_str();
    auto placeObj = R.placeDistDir(obj->id.c_str(),targetid.c_str(),0.,0.,0.,0);
    R.wait(+placeObj);
    return true;
  } else if (obj->color==HRIObject::green && obj->size == HRIObject::small) {
    targetid = getTargetID("targetgs").c_str();
    auto placeObj = R.placeDistDir(obj->id.c_str(),targetid.c_str(),0.,0.,0.,0);
    R.wait(+placeObj);
    return true;
  } else if (obj->color==HRIObject::red && obj->size == HRIObject::small) {
    targetid = getTargetID("targetrs").c_str();
    auto placeObj = R.placeDistDir(obj->id.c_str(),targetid.c_str(),0.,0.,0.,0);
    R.wait(+placeObj);
    return true;
  } else if (obj->color==HRIObject::yellow && obj->size == HRIObject::small) {
    targetid=getTargetID("targetys").c_str();
    auto placeObj = R.placeDistDir(obj->id.c_str(),targetid.c_str(),0.,0.,0.,0);
    R.wait(+placeObj);
    return true;
  }
  return false;
}

bool OrderingTask::isReachable(HRIObject* obj) {
  return obj->pos.x<.8;
}

void OrderingTask::performAction() {
  //log("bridge state, " + std::to_string(getBridgeState(state)));
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
  default:
    cout << mode << " is no correct interaction mode" << endl;
    break;
  }
}

void OrderingTask::performActionProactive() {
  cout << "State: " << state;
  std::vector<HRIObject*> objs = nextActions(state); 
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
  /*cout << "proactive action selection" << endl;
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
    }*/
}


void OrderingTask::performActionAutonomous() {
  cout << "State: " << state;
  std::vector<HRIObject*> objs = nextActions(state);
  for (unsigned int i=0; i<objs.size(); ++i) {
    cout << "next object: " <<  objs[i]->id << endl;
    if (isReachable(objs[i])) {
      // use gripper based on position of object
      //LeftOrRight lr = objs[i]->pos.y<0 ? LR_right : LR_left;
      // use gripper based on block color
      LeftOrRight lr = LR_left;
      if (objs[i]->color == HRIObject::green || objs[i]->color == HRIObject::yellow)
	lr = LR_right;
      auto graspObj = R.graspBox(objs[i]->id.c_str(), lr );
	R.wait(+graspObj);
	placeObj(objs[i]);
	state.updateFromModelworld(R);
	break;
    }
  }
}

 void OrderingTask::performActionRequest() {
  cout << "State: " << state;
  std::vector<HRIObject*> objs = nextActions(state);
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

 void OrderingTask::performActionCommands() {
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

void OrderingTask::log(std::string s) {
  ofstream myfile;
  std::string filename="logfile_mode" + std::to_string(mode) + ".txt";
  myfile.open (filename, std::ios::out | std::ios::app);
  std::time_t result = std::time(0);
  myfile << result << ", ";
  myfile << s << endl;
  myfile.close();
}
