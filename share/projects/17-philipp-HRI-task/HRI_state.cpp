#include "HRI_state.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <Control/taskControl.h>

using std::cout;
using std::endl;
using std::string;

const cv::Mat3f HRIObject::labRed(cv::Vec3f(40., 35., 0.));
const cv::Mat3f HRIObject::labGreen(cv::Vec3f(60., -30., 37.));
const cv::Mat3f HRIObject::labBlue(cv::Vec3f(55.,8.,-45.));
const cv::Mat3f HRIObject::labYellow(cv::Vec3f(85., -20., 40.));

void HRIObject::setByData(const arr& col, const arr& s, const mlr::Vector p, const mlr::Quaternion r) {
  pos = p;
  rot = r;
  //convert color to lab
  cv::Mat3f rgbColor(cv::Vec3f(col(0), col(1), col(2))), labColor;
  cvtColor(rgbColor, labColor, CV_RGB2Lab);
  //set to nearest color using distances in L*a*b* colorspace
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
  double difference = .005;
  size = nullsize;
  double difference_tmp = std::abs(s(0)*s(1) - areaSmall);
  if (difference_tmp<difference && s(0)>.055 && s(1)>.055) {
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
  return os;
}

//-------------------------------------------------------------------------------
//-------------------------------------------------------------------------------

OnTableState::OnTableState() {
  // get the number of shapes starting with 'S' and the table height from modelworld
  Access<mlr::KinematicWorld> modelWorld("modelWorld");
  modelWorld.readAccess();
  cout << "Output Shapes" << endl;
  objN=0;
  for (mlr::Shape* s: modelWorld->shapes) {
    if (s->name[0]=='S')
      objN++;
    else if (s->name=="table")
      tableheight = s->X.pos(2);
  }
  modelWorld.deAccess();
}

void OnTableState::initFromPercepts(PerceptL percepts) {
  // create HRIObject list from percepts
  cout << "init from percepts" << endl;
  objects.clear();
  objN=0;
  for (Percept *p:percepts) {
    if (p->type==Percept::PT_box) {
      HRIObject obj;
      obj.setByData((static_cast<PercBox*> (p))->color, (static_cast<PercBox*> (p))->size, p->transform.pos, p->transform.rot);
      obj.id=std::string("S") + std::to_string(objN+1);
      objN++;
      obj.pos(2)=tableheight+.04;
      objects.push_back(obj);
    }
  }
}

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
  std::vector<int> matching;
  for (HRIObject p_obj:percepted_obj) {
    matching.clear();
    for (unsigned int i=0; i<objects.size();++i) {
      if(updatet.empty() || std::find(updatet.begin(), updatet.end(), i) == updatet.end()) {
	if (objects[i].color == p_obj.color && objects[i].size==p_obj.size) {
	  matching.push_back(i);
	}
      }
    }
    // more objects of same type possible, calculate a score to find the most likely object to update
    int to_update=-1;
    int score=-100;
    for(unsigned int t=0;t<matching.size();++t) {
      int tmp_score=0;
      if (objects[matching[t]].below_obj==NULL) // is not below an object => increase score
	tmp_score+=10;

      if (tmp_score>score) {
	to_update=matching[t];
	score=tmp_score;
      }
    }
    if(to_update>-1) {
      updatet.push_back(to_update);
      objects[to_update].pos = p_obj.pos;
      objects[to_update].rot = p_obj.rot;
      objects[to_update].pos(2)=tableheight+.04;
      objects[to_update].below_obj = NULL;
    }
  }
  //check the non-updatet objects for "below" relations
  for (unsigned int i=0; i<objects.size();++i) {
    if(objects[i].below_obj == NULL && std::find(updatet.begin(), updatet.end(), i) == updatet.end()) {
      double distance = 0.12;
      double tmp_distance;
      for (unsigned int y=0;y<objects.size();++y) {
	if (i!=y && !objects[y].isBelow(&objects[i])) {
	  tmp_distance=sqrt( (objects[i].pos.x-objects[y].pos.x)*(objects[i].pos.x-objects[y].pos.x) +
			     (objects[i].pos.y-objects[y].pos.y)*(objects[i].pos.y-objects[y].pos.y) );
	  if (tmp_distance<distance) {
	    if(objects[i].color==objects[y].color && objects[i].size==objects[y].size && objects[y].pos.z<objects[i].pos.z) { //for same color objects also check order based on z position
	      objects[y].below_obj=&objects[i];
	      objects[i].pos(2)=objects[y].pos(2)+0.04;
	      distance=tmp_distance;

	    } else {
	      objects[i].below_obj=&objects[y];
	      objects[y].pos(2)=objects[i].pos(2)+0.04;
	      distance=tmp_distance;
	    }
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
    for (unsigned int i=1; i<=objN; ++i) {
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
    double distance = 0.2;
    double tmp_distance;
    double zdiff =100;
    for (unsigned int q=0; q<objects.size();++q) {
      tmp_distance=sqrt( (objects[p].pos.x-objects[q].pos.x)*(objects[p].pos.x-objects[q].pos.x) +
			 (objects[p].pos.y-objects[q].pos.y)*(objects[p].pos.y-objects[q].pos.y) );
      if (objects[q].size==HRIObject::small && tmp_distance < 0.03 && tmp_distance<distance && objects[p].pos.z < objects[q].pos.z && objects[q].pos.z-objects[p].pos.z < zdiff) {
	zdiff = objects[q].pos.z-objects[p].pos.z;
	objects[p].below_obj=&objects[q];
	distance=tmp_distance;
	cout << "UPDATED a small object" << endl;
      }
      else if (objects[q].size==HRIObject::large && tmp_distance < 0.1 && tmp_distance < distance && objects[p].pos.z < objects[q].pos.z && objects[q].pos.z-objects[p].pos.z < zdiff) {
	zdiff = objects[q].pos.z-objects[p].pos.z;
	objects[p].below_obj=&objects[q];
	distance=tmp_distance;
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

HRIObject* OnTableState::getObj(HRIObject::Color c, HRIObject::Size s, int number){
  int n=0;
  for (unsigned int i=0;i<objects.size();++i) {
    if(objects[i].color==c && objects[i].size==s) {
      if (n==number) {
	return &objects[i];
      } else {
	++n;
      }
    }
  }
  return NULL;
}

int OnTableState::countObjs(HRIObject::Color c, HRIObject::Size s){
  int counter=0;
  for (unsigned int i=0;i<objects.size();++i) {
    if(objects[i].color==c && objects[i].size==s)
      counter++;
  }
  return counter;
}


void OnTableState::updateModelworldFromState() {
  Access<mlr::KinematicWorld> modelWorld("modelWorld");
      
  // update color and size
  modelWorld.writeAccess();
  for (HRIObject obj:objects) {
    mlr::Body *body = modelWorld->getBodyByName(obj.id.c_str(), false);
    if (body != NULL) {
      mlr::Shape *s=body->shapes.first();
      if (obj.size==HRIObject::small)
	s->size = ARR(0.06,0.06,0.04);
      else if (obj.size==HRIObject::medium)
	s->size = ARR(0.06,0.12,0.04);
      else if (obj.size==HRIObject::large)
	s->size = ARR(0.06,0.27,0.04);

      s->mesh.setSSBox(s->size(0), s->size(1), s->size(2), 0.0001);
      cv::Mat3f rgbColor;
      if (obj.color==HRIObject::red)
        cvtColor(HRIObject::labRed, rgbColor, CV_Lab2RGB);
      else if (obj.color==HRIObject::green)
        cvtColor(HRIObject::labGreen, rgbColor, CV_Lab2RGB);
      else if (obj.color==HRIObject::blue)
        cvtColor(HRIObject::labBlue, rgbColor, CV_Lab2RGB);
      else if (obj.color==HRIObject::yellow)
        cvtColor(HRIObject::labYellow, rgbColor, CV_Lab2RGB);
      else
	rgbColor = cv::Mat3f(cv::Vec3f(0.,0.,0.));
      s->mesh.C = ARR(rgbColor(0)(0),rgbColor(0)(1),rgbColor(0)(2));

    }
  }
  modelWorld.deAccess();

  // update pose
    double cumerror = 1.; //cumulative error
    for (unsigned int i=0; i<100 && cumerror>.001;++i) {
      cumerror = 0.;
      modelWorld.writeAccess();
      modelWorld->setAgent(1);

      TaskControlMethods taskController(modelWorld());
      arr q=modelWorld().q;
      for (HRIObject obj:objects) {
	if (obj.id != "") {
	  mlr::Body *body = modelWorld->getBodyByName(obj.id.c_str(), false);
	  if (body == NULL) {
	    cout << "Object " << obj.id << " does not exist in modelworld" << endl;
	    // Todo: add object to modelworld?
	    continue;
	  }

	  mlr::Shape *shape = body->shapes(0);
	  
	  cumerror += (shape->X.pos-obj.pos).length();
	  CtrlTask *t;
	  t = new CtrlTask(STRING("syncPos_" <<body->name), new TaskMap_Default(posTMT, body->shapes.first()->index));
	  t->ref = new MotionProfile_Const( obj.pos.getArr() );
	  taskController.tasks.append(t);
	  
	  t = new CtrlTask(STRING("syncQuat_" <<body->name), new TaskMap_Default(quatTMT, body->shapes.first()->index));
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
}


std::ostream& operator<<(std::ostream& os, const OnTableState state) {
  os << "#Objects: " << state.objects.size() << endl;
  for (HRIObject obj:state.objects) {
    os << obj << endl << endl;
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
