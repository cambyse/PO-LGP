#include "HRI_state.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <Control/taskControl.h>

// note that this is an old version of HRI_state.cpp used for the HAI 2017 language paper

using std::cout;
using std::endl;
using std::string;

void HRIObject::setByData(const arr& col, const arr& s, const mlr::Vector p, const mlr::Quaternion r) {
  pos = p;
  rot = r;
  //convert color to lab
  cv::Mat3f rgbColor(cv::Vec3f(col(0), col(1), col(2))), labColor;
  cvtColor(rgbColor, labColor, CV_RGB2Lab);

  //set to nearest color using distances in L*a*b* colorspace
  static const cv::Mat3f labRed(cv::Vec3f(32., 31.5, 3.5));
  static const cv::Mat3f labGreen(cv::Vec3f(47., -27.5, 39.));
  static const cv::Mat3f labBlue(cv::Vec3f(41.,6.5,-37.));
  static const cv::Mat3f labYellow(cv::Vec3f(71, -22, 41.5));
  float distance = 30.0f;
  color = nullcolor;
  float distance_tmp = cv::norm(labColor-labRed);
  if (distance_tmp<distance) {
    distance = distance_tmp;
    color = red;
  }
  distance_tmp = cv::norm(labColor-labGreen);
  if (distance_tmp<distance) {
    distance = distance_tmp;
    color = green;
  }
  distance_tmp = cv::norm(labColor-labBlue);
  if (distance_tmp<distance) {
    distance = distance_tmp;
    color = blue;
  }
  distance_tmp = cv::norm(labColor-labYellow);
  if (distance_tmp<distance) {
    distance = distance_tmp;
    color = yellow;
  }

  // set to nearest size using difference in area
  static const double areaSmall = .065*.065;
  static const double areaMedium = .065*.125;
  static const double areaLarge = .065*.275;
  double difference = .015;
  size = nullsize;
  double difference_tmp = std::abs(s(0)*s(1) - areaSmall);
  if (difference_tmp<difference) {
    difference = difference_tmp;
    size = small;
  }
  difference_tmp = std::abs(s(0)*s(1) - areaMedium);
  if (difference_tmp<difference) {
    difference = difference_tmp;
    size = medium;
  }
  difference_tmp = std::abs(s(0)*s(1) - areaLarge);
  if (difference_tmp<difference) {
    difference = difference_tmp;
    size = large;
  }
}

void HRIObject::setByName(Roopi& R, const char* name) {
  auto K = R.getK();
  id = name;
  arr c = K->getShapeByName(name)->mesh.C;
  if (c.d0>3)
    c = c[2];
  arr s = K->getShapeByName(name)->size;
  mlr::Vector p = K->getShapeByName(name)->X.pos;
  mlr::Quaternion rot = K->getShapeByName(name)->X.rot;
  setByData(c,s,p, rot);
}

bool HRIObject::isBelow(HRIObject* s) {
  if (below_obj==NULL || s==NULL)
    return false;
  return (below_obj->id==s->id);
}


std::ostream& operator<<(std::ostream& os, const HRIObject state) {
  os << state.id << "  Color: ";
  switch (state.color) {
  case HRIObject::red:
    os << "red";
    break;
  case HRIObject::green:
    os << "green";
    break;
  case HRIObject::blue:
    os << "blue";
    break;
  case HRIObject::yellow:
    os << "yellow";
    break;
  default:
    os << "not set";
  }
  os << "  Size: ";
  switch (state.size) {
  case HRIObject::small:
    os << "small";
    break;
  case HRIObject::medium:
    os << "medium";
    break;
  case HRIObject::large:
    os << "large";
    break;
  default:
    os << "not set";
  }
  os << "  Position: " << state.pos;
  if (state.below_obj !=NULL)
    os << " below object " << state.below_obj->id;
  os << endl;
  return os;
}

//-------------------------------------------------------------------------------
//-------------------------------------------------------------------------------

void OnTableState::updateFromPercepts(PerceptL percepts) {
  // create HRIObject list from percepts
  std::vector<HRIObject> percepted_obj;
  for (Percept *p:percepts) {
    if (p->type==Percept::PT_box) {
      HRIObject obj;
      obj.setByData((static_cast<PercBox*> (p))->color, (static_cast<PercBox*> (p))->size, p->transform.pos, p->transform.rot);
      percepted_obj.push_back(obj);
    }
  }
  // update matching objects
  std::vector<int> updatet;
  for (HRIObject p_obj:percepted_obj) {
    for (unsigned int i=0; i<objects.size();++i) {
      if(updatet.empty() || std::find(updatet.begin(), updatet.end(), i) == updatet.end()) {
	if (objects[i].color == p_obj.color && objects[i].size==p_obj.size) {
	  updatet.push_back(i);
	  objects[i].pos = p_obj.pos;
	  objects[i].rot = p_obj.rot;
//	  objects[i].pos(2)=0.74;
	  objects[i].below_obj = NULL;
	}
      }
    }
  }
  //check the non-updatet objects for "below" relations
  for (unsigned int i=0; i<objects.size();++i) {
    if(objects[i].below_obj == NULL && std::find(updatet.begin(), updatet.end(), i) == updatet.end()) {
      double distance = 0.08;
      double tmp_distance;
      for (unsigned int y=0;y<objects.size();++y) {
	if (i!=y && !objects[y].isBelow(&objects[i])) {
	  tmp_distance=sqrt( (objects[i].pos.x-objects[y].pos.x)*(objects[i].pos.x-objects[y].pos.x) +
			     (objects[i].pos.y-objects[y].pos.y)*(objects[i].pos.y-objects[y].pos.y) );
	  if (tmp_distance<distance) {
	    objects[i].below_obj=&objects[y];
	    objects[y].pos(2)=objects[i].pos(2)+0.04;
	    distance=tmp_distance;
	  }
	}
      }
    }
  }
  
  for (unsigned int p=0; p<objects.size();++p) {
    for (unsigned int q=0; q<objects.size();++q) {
      if(objects[q].below_obj!=NULL)
	objects[q].below_obj->pos(2)=objects[q].pos(2)+0.04;
    }
  }
}

void OnTableState::updateFromModelworld(Roopi& R,const char* objname) {

  if (objname == NULL) { 
    objects.clear();
//    for (unsigned int i=1; i<=5; ++i) {
      for (unsigned int i=1; i<=9; ++i) {
      std::string objname("S" + std::to_string(i));
      if(R.getK()->getShapeByName(objname.c_str()))
	{
	  HRIObject obj;
	  obj.setByName(R, objname.c_str());
	  objects.push_back(obj);
	}
    }
  } else {
    HRIObject obj;
    obj.setByName(R, objname);
    bool updated=false;
    for (unsigned int i=0;i<objects.size();++i) {
      if (objects[i].id == objname) {
	objects[i]=obj; //if in list -> update
	updated=true;
	break;
      }
    }
    if (!updated)
      objects.push_back(obj);  //if not in list -> append
  }

  // update below relations
  for (unsigned int p=0; p<objects.size();++p) {
    for (unsigned int q=0; q<objects.size();++q) {
      if (std::abs(objects[p].pos.x-objects[q].pos.x) < 0.01 && std::abs(objects[p].pos.y-objects[q].pos.y) < 0.01 && objects[p].pos.z < objects[q].pos.z) {
	objects[p].below_obj=&objects[q]; // TODO: check for order?
      }
    }
  }
}

HRIObject* OnTableState::getObj(string oid){
  for (unsigned int i=0; i<objects.size();++i)
    if (objects[i].id==oid)
      return &objects[i];
  return NULL;
}

HRIObject* OnTableState::getObj(HRIObject::Color c, HRIObject::Size s){
  for (unsigned int i=0;i<objects.size();++i) {
    if(objects[i].color==c && objects[i].size==s)
      return &objects[i]; //TODO. multiple objects of same type?
  }
  return NULL;
}


void OnTableState::updateModelworldFromState() {
  Access<mlr::KinematicWorld> modelWorld("modelWorld");
      
  double cumerror = 1.; //cumulative error
  for (unsigned int i=0; i<100 && cumerror>.001;++i) {
    cumerror = 0.;
    modelWorld.writeAccess();
    modelWorld->setAgent(1);
    TaskControlMethods taskController(modelWorld());
    arr q=modelWorld().q;
    for (HRIObject obj:objects) {
      if (obj.id != "") {
	mlr::Shape* shape = modelWorld->getShapeByName(obj.id.c_str());
	mlr::Body *b = shape->body;
	cumerror += (shape->X.pos-obj.pos).length();
	CtrlTask *t;
	t = new CtrlTask(STRING("syncPos_" <<b->name), new TaskMap_Default(posTMT, b->shapes.first()->index));
	t->ref = new MotionProfile_Const( obj.pos.getArr() );
	taskController.tasks.append(t);

	t = new CtrlTask(STRING("syncQuat_" <<b->name), new TaskMap_Default(quatTMT, b->shapes.first()->index));
	t->ref = new MotionProfile_Const( obj.rot.getArr4d(), true );
	taskController.tasks.append(t);
      }
    }
    double cost=0.;
    taskController.updateCtrlTasks(0., modelWorld()); //computes their values and Jacobians
    arr dq = taskController.inverseKinematics(NoArr, NoArr, &cost);
    q += dq;
    
    listDelete(taskController.tasks); //cleanup tasks
  
    modelWorld->setAgent(1);
    modelWorld->setJointState(q);
    modelWorld->setAgent(0);
  
    modelWorld.deAccess();
  }

  /*modelWorld.writeAccess();
    for (HRIObject obj:objects) {
    if (obj.below_obj!="") {
    mlr::KinematicSwitch sw1(mlr::KinematicSwitch::deleteJoint, mlr::JT_none, NULL, obj.below_obj.c_str(), modelWorld(), 0);
    sw1.apply(modelWorld());
    mlr::KinematicSwitch sw2(mlr::KinematicSwitch::addJointAtTo, mlr::JT_rigid, obj.id.c_str(), obj.below_obj.c_str(), modelWorld(), 0);
    sw2.apply(modelWorld());
      
    }
    }
    modelWorld.deAccess();*/
}

//mlr::Vector OnTableState::robot_to_raw_table_position(mlr::Vector rpos){
//  //mlr::KinematicWorld kw = R.getK();
//  mlr::KinematicWorld kw("modelWorld");
//  mlr::Body *b1=kw.getBodyByName("base_footprint");
//  mlr::Body *b2=kw.getBodyByName("table");
//  mlr::Vector tpos;
//  arr y,J;
//  kw.kinematicsRelPos(y,J,b1,rpos,b2,NoVector);
////  cout<<"y "<<y<<"; J "<<J<<endl;
//  tpos.x = y.popFirst();
//  tpos.y = y.popFirst();
//  return tpos;
//}

//mlr::Vector OnTableState::raw_table_to_robot_position(mlr::Vector tpos){
//  //mlr::KinematicWorld kw = R.getK();
//  mlr::KinematicWorld kw("modelWorld");
//  mlr::Body *b1=kw.getBodyByName("table");
//  mlr::Body *b2=kw.getBodyByName("base_footprint");
//  mlr::Vector rpos;
//  arr y,J;
//  kw.kinematicsRelPos(y,J,b1,tpos,b2,NoVector);
// // cout<<"y "<<y<<"; J "<<J<<endl;
//  rpos.x = y.popFirst();
//  rpos.y = y.popFirst();
//  return rpos;
//}


std::ostream& operator<<(std::ostream& os, const OnTableState state) {
  os << "#Objects: " << state.objects.size() << endl;
  for (HRIObject obj:state.objects) {
    os << obj << endl;
  }
  return os;
}



//-------------------------------------------------------------------------------
//-------------------------------------------------------------------------------

bool RobotState::containsObj(HRIObject* obj) {
  if (left != NULL)
    if (left->id == obj->id)
      return true;
  if (right != NULL)
    if (right->id == obj->id)
      return true;
  return false;
}

void RobotState::removeObj(HRIObject* obj) {
  if(right != NULL && right->id == obj->id)
    right = NULL;
  if(left != NULL && left->id == obj->id)
    left = NULL;
}
