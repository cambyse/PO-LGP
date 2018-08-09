#include "HRI_state.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <Control/taskControl.h>

using std::cout;
using std::endl;
using std::string;

//const cv::Mat3f HRIObject::labRed(cv::Vec3f(40., 35., 0.));
//const cv::Mat3f HRIObject::labGreen(cv::Vec3f(60., -30., 37.));
//const cv::Mat3f HRIObject::labBlue(cv::Vec3f(55.,8.,-45.));
//const cv::Mat3f HRIObject::labYellow(cv::Vec3f(85., -20., 40.));
const cv::Mat3f HRIObject::labRed(cv::Vec3f(35., 36., 3.));
const cv::Mat3f HRIObject::labGreen(cv::Vec3f(54., -28., 31.));
const cv::Mat3f HRIObject::labBlue(cv::Vec3f(48.,7.,-38.));
const cv::Mat3f HRIObject::labYellow(cv::Vec3f(75., -23., 43.));
//[31, 35, 4]
//[46, -27, 30]
//[46, 6, -36]
//[73, -20, 42]
//35	36	3
//54	-28	31
//48	7	-38
//75	-23	43


void HRIObject::setByData(const arr& col, const arr& s, const mlr::Vector p, const mlr::Quaternion r) {
  pos = p;
  rot = r;
  //convert color to lab
  cv::Mat3f rgbColor(cv::Vec3f(col(0), col(1), col(2))), labColor;
  cvtColor(rgbColor, labColor, CV_RGB2Lab);
//  cout << "labColor: " << labColor;
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
//  cout << "color: " << color;

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

bool HRIObject::isAbove(HRIObject* s) {
  if (above_obj==NULL || s==NULL)
    return false;
  return (above_obj->id==s->id);
}

bool HRIObject::overlapping(HRIObject* s){
  std::stringstream output;
  output<< "overlapping: ";
  double st=0.06/2.0;
  double mt=0.12/2.0;
  double lt=0.27/2.0;
  bool overlap = false;
  double angle_a = 0.0;
  double angle_b = 0.0;
  double test = rot.x*rot.y + rot.z*rot.w;
  if (test > 0.499) { // singularity at north pole
    angle_a = MLR_PI / 2;
  } else if (test < -0.499) { // singularity at south pole
    angle_a = -MLR_PI / 2;
  } else {
    angle_a = asin(2*rot.x*rot.y+2*rot.z*rot.w);
  }
  if (angle_a<0)
    angle_a = -angle_a;
  test = s->rot.x*s->rot.y + s->rot.z*s->rot.w;
  if (test > 0.499) { // singularity at north pole
    angle_b = MLR_PI / 2;
  } else if (test < -0.499) { // singularity at south pole
    angle_b = -MLR_PI / 2;
  } else {
    angle_b = asin(2*s->rot.x*s->rot.y+2*s->rot.z*s->rot.w);
  }
  if (angle_b<0)
    angle_b = -angle_b;
  bool a_x = false;
  bool b_x = false;
  if (!((angle_a<MLR_PI/4)||((angle_a>(3*MLR_PI/4))&&(angle_a<(5*MLR_PI/4)))||(angle_a>(7*MLR_PI/4)))){
    a_x = true;
  }
  if (!((angle_b<MLR_PI/4)||((angle_b>(3*MLR_PI/4))&&(angle_b<(5*MLR_PI/4)))||(angle_b>(7*MLR_PI/4)))){
    b_x = true;
  }
  output<< " a x: " << a_x << "b x: " << b_x;
  output << "pos " << pos;
  output << "s pos " << s->pos;
  double x_distance = pos.x-s->pos.x;
  if (x_distance<0)
    x_distance = -x_distance;
  double y_distance = pos.y-s->pos.y;
  if (y_distance<0)
    y_distance = -y_distance;
  output << "x " << x_distance << "y " <<y_distance;
  if ((size==HRIObject::small)&&(size==s->size)){
    output<<"small small ";
    // both small, then simply check if close to each other
    if ((x_distance<=st)&&(y_distance<=st))
      overlap = true;
  } else if ((size==HRIObject::medium)&&(s->size==HRIObject::small)){
    output<<"medium small ";
    // check if b is within small distance on small dimension of a, and within medium distance on long dimension of a
    if (a_x){
      // long dimension approx aligned with x axis
      if ((x_distance<=mt)&&(y_distance<=st))
        overlap = true;
    } else {
      // long dimesion approx aligned with y axis
      if ((x_distance<=st)&&(y_distance<=mt))
        overlap = true;
    }
  } else if ((size==HRIObject::small)&&(s->size==HRIObject::medium)){
    output<<"small medium ";
    // check if a is within small distance on small dimension of b, and within medium distance on long dimension of b
    if (b_x){
      // long dimension approx aligned with x axis
      if ((x_distance<=mt)&&(y_distance<=st))
        overlap = true;
    } else {
      // long dimesion approx aligned with y axis
      if ((x_distance<=st)&&(y_distance<=mt))
        overlap = true;
    }
  } else if ((size==HRIObject::large)&&(s->size==HRIObject::small)){
    output<<"large small ";
    // check if b is within small distance on small dimension of a, and within large distance on long dimension of a
    if (a_x){
      // long dimension approx aligned with x axis
      if ((x_distance<=lt)&&(y_distance<=st))
        overlap = true;
    } else {
      // long dimesion approx aligned with y axis
      if ((x_distance<=st)&&(y_distance<=lt))
        overlap = true;
    }
  } else if ((size==HRIObject::small)&&(s->size==HRIObject::large)){
    output<<"small large ";
    // check if a is within small distance on small dimension of b, and within large distance on long dimension of b
    if (b_x){
      // long dimension approx aligned with x axis
      if ((x_distance<=lt)&&(y_distance<=st))
        overlap = true;
    } else {
      // long dimesion approx aligned with y axis
      if ((x_distance<=st)&&(y_distance<=lt))
        overlap = true;
    }
  } else if ((size==HRIObject::medium)&&(s->size==HRIObject::large)){
    output<<"medium large ";
    if (a_x){
      // long dimension of a approx aligned with x axis
      if (b_x){
        // long dimension of a and b approx aligned with x axis
        if ((x_distance<=(lt+mt))&&(y_distance<=(st)))
            overlap = true;
      } else {
        // long dimension of a aligned with x axis, long dimension of b aligned with y axis
        if ((x_distance<=mt)&&(y_distance<=lt))
            overlap=true;
      }
    } else {
      // long dimension of a approx aligned with y axis
      if (b_x){
        // long dimension of b aligned with x axis, long dimension of a aligned with y axis
        if ((x_distance<=lt)&&(y_distance<=mt))
            overlap=true;
      } else {
        // long dimension of a and b approx aligned with y axis
        if ((x_distance<=(st))&&(y_distance<=(lt+mt)))
            overlap = true;
      }
    }
  } else if ((size==HRIObject::large)&&(s->size==HRIObject::medium)){
    output<<"large medium ";
    if (a_x){
      // long dimension of a approx aligned with x axis
      if (b_x){
        // long dimension of a and b approx aligned with x axis
        if ((x_distance<=(lt+mt))&&(y_distance<=(st)))
            overlap = true;
      } else {
        // long dimension of a aligned with x axis, long dimension of b aligned with y axis
        if ((x_distance<=lt)&&(y_distance<=mt))
            overlap=true;
      }
    } else {
      // long dimension of a approx aligned with y axis
      if (b_x){
        // long dimension of b aligned with x axis, long dimension of a aligned with y axis
        if ((x_distance<=mt)&&(y_distance<=lt))
            overlap=true;
      } else {
        // long dimension of a and b approx aligned with y axis
        if ((x_distance<=(st))&&(y_distance<=(mt+lt)))
            overlap = true;
      }
    }
  }
  if (overlap){
    output<<"true";
  }
//  log(output.str());
  return overlap;
}

void HRIObject::log(std::string s) {
  ofstream myfile;
//  std::string filename="logfile_mode" + std::to_string(mode) + ".txt";
  std::string filename="logfileObjects.txt";
  myfile.open (filename, std::ios::out | std::ios::app);
  std::time_t result = std::time(0);
  myfile << result << ", ";
  myfile << s << endl;
  myfile.close();
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
  if (state.above_obj !=NULL)
    os << " above object " << state.above_obj->id;
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
//      cout << "colour:" << (static_cast<PercBox*> (p))->color;
      obj.setByData((static_cast<PercBox*> (p))->color, (static_cast<PercBox*> (p))->size, p->transform.pos, p->transform.rot);
      obj.id=std::string("S") + std::to_string(objN+1);
//      cout << obj.id << " colour: " << (static_cast<PercBox*> (p))->color;
      objN++;
      obj.pos(2)=tableheight+.04;
      objects.push_back(obj);
    }
  }
}

void OnTableState::updateFromPercepts(PerceptL percepts) {
  std::stringstream output;
  output<< "update from percepts: "<<endl;
  // create HRIObject list from percepts
  std::vector<HRIObject> percepted_obj;
  for (unsigned int i=0; i<objects.size();++i){
    objects[i].perceived=false;
  }
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
  HRIObject missing_obj;
  bool missing_obj_found=false;
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
/*    if (to_update==-1){
      //see if there is a partial match
      for (unsigned int i=0; i<objects.size();++i) {
        if(updatet.empty() || std::find(updatet.begin(), updatet.end(), i) == updatet.end()) {
          if (objects[i].color == p_obj.color) {
            matching.push_back(i);
          }
        }
      }
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
    }*/
    if(to_update>-1) {
      updatet.push_back(to_update);
      objects[to_update].moved=sqrt( (objects[to_update].pos.x-p_obj.pos.x)*(objects[to_update].pos.x-p_obj.pos.x) +
        (objects[to_update].pos.y-p_obj.pos.y)*(objects[to_update].pos.y-p_obj.pos.y) );
      double tmp_z = objects[to_update].pos(2);
      objects[to_update].pos = p_obj.pos;
      objects[to_update].pos(2) = tmp_z;
      objects[to_update].rot = p_obj.rot;
//      objects[to_update].pos(2)=tableheight+.04;
//      objects[to_update].below_obj = NULL;
      objects[to_update].perceived = true;
//      output<<"obj: "<<objects[to_update].id;
    }/* else {
      // perceived object has not been matched with any existing object
      missing_obj = p_obj;
      missing_obj_found=true;
//      cout<<"missing-object";
      output<<"missing object found: "<<missing_obj<<endl;
    }*/
  }

/*  if (missing_obj_found){
    // check if this could actually be one of the objects in the list of objects, with a different color or size
    // currently only checking for mix-up between yellow and green
    matching.clear();
    int numMatches = 0;
    for (unsigned int i=0; i<objects.size();++i) {
      if ((!(objects[i].perceived))&&((objects[i].below_obj==NULL))){
//        cout<<"possible match";
        output<<"possible match: "<<objects[i]<<endl;
        // if object has not been perceived this time, but last time was on top of a stack, this might be the object
        if(((objects[i].color==HRIObject::green)&&(missing_obj.color==HRIObject::yellow))||((objects[i].color==HRIObject::yellow)&&(missing_obj.color==HRIObject::green))){
          // green and yellow have been mixed up
          matching.push_back(i);
//          cout<<"("<<objects[i].id<<")";
          numMatches++;
//          cout<<" green-yellow-swap";
        }
      }
    }
//    cout<<numMatches<<".";
    // more objects of same type possible, calculate a score to find the most likely object to update
    int to_update=-1;
    int score=-100;
    for(unsigned int t=0;t<matching.size();++t) {
      int tmp_score=0;
      if (objects[matching[t]].below_obj==NULL) // is not below an object => increase score
        tmp_score+=10;
      if (objects[matching[t]].size!=missing_obj.size)
        tmp_score-=10;

      if (tmp_score>score) {
        to_update=matching[t];
        score=tmp_score;
      }
    }
    if(to_update>-1) {
      updatet.push_back(to_update);
      objects[to_update].moved=sqrt( (objects[to_update].pos.x-missing_obj.pos.x)*(objects[to_update].pos.x-missing_obj.pos.x) +
        (objects[to_update].pos.y-missing_obj.pos.y)*(objects[to_update].pos.y-missing_obj.pos.y) );
      double tmp_z = objects[to_update].pos(2);
      objects[to_update].pos = missing_obj.pos;
      objects[to_update].pos(2) = tmp_z;
      objects[to_update].rot = missing_obj.rot;
//      objects[to_update].pos(2)=tableheight+.04;
//      objects[to_update].below_obj = NULL;
      objects[to_update].perceived = true;
//      output<<"obj: "<<objects[to_update].id;
      cout<<" object updated";
    }
  }*/

  //possible: remove above and below relationships here if objects are no longer overlapping

  for (unsigned int i=0; i<objects.size();++i){
    if (objects[i].perceived && objects[i].moved>0.04 && (objects[i].below_obj==NULL)){
      //possible: don't need to include moved>0.04 here, just check for perceived and no below_obj set
      // if object has been received, and has moved since the last perception, then check whether it has been placed
      // on another object or placed on the table, and update any relevant above and below relationships
      double height = 0;
      int topStack = -1;
      int numObjectsStack = 0;
      for (unsigned int y=0; y<objects.size();++y){
        if ((i!=y)&&objects[i].overlapping(&objects[y])){
          numObjectsStack++;
          if ((objects[y].pos.z)>height){
            topStack=y;
            height=objects[y].pos.z;
          }
        } else if (i!=y){
          // check if any above and below relationships need to be removed, because they are not overlapping
          if (!(objects[y].below_obj==NULL)){
            if (objects[y].below_obj->id==objects[i].id){
              objects[y].below_obj = NULL;
//              output<<"below relationship removed between "<<i<<" and "<<y<<";";
            }
          }
          if (!(objects[y].above_obj==NULL)){
            if (objects[y].above_obj->id==objects[i].id){
              objects[y].above_obj = NULL;
//              output<<"above relationship removed between "<<i<<" and "<<y<<";";
            }
          }
        }
      }
      if (numObjectsStack>0){
        // put on top of stack
        objects[i].pos(2)=height+0.04;
        objects[topStack].below_obj = &objects[i];
        objects[i].above_obj = &objects[topStack];
        objects[i].below_obj = NULL;
//        output<<"object "<<i<< " moved to top of stack above object "<<topStack<<";";
      } else {
        // put on table
        objects[i].pos(2) = tableheight+0.04;
        objects[i].below_obj = NULL;
        objects[i].above_obj = NULL;
//        output<<"object "<<i<< " moved to table;";
      }
    }
  }
  
  /*for (unsigned int p=0; p<objects.size();++p) {
    for (unsigned int q=0; q<objects.size();++q) {
      if(objects[q].below_obj!=NULL)
	objects[q].below_obj->pos(2)=objects[q].pos(2)+0.04;
    }
  }*/
//  log(output.str());
}

void OnTableState::setObjectPosition(HRIObject::Color objColor, HRIObject::Size objSize, mlr::Vector objPosition){
  for (unsigned int i=0; i<objects.size();++i) {
    if (objects[i].color == objColor && objects[i].size==objSize) {
      objects[i].pos.x = objPosition.x;
      objects[i].pos.y = objPosition.y;
      objects[i].pos.z = objPosition.z;
    }
  }
}

bool OnTableState::findObjectInPercepts(PerceptL percepts, HRIObject::Color objColor, HRIObject::Size objSize, mlr::Vector targetPos) {
  bool objectOnTarget = false;
  std::stringstream output;
//  output<< "find object in percepts: "<<endl;
  // create HRIObject list from percepts
  HRIObject matching_obj;
  bool match_found = false;
  std::vector<HRIObject> percepted_obj;
  for (unsigned int i=0; i<objects.size();++i){
    objects[i].perceived=false;
  }
  for (Percept *p:percepts) {
    if (p->type==Percept::PT_box) {
      HRIObject obj;
      obj.setByData((static_cast<PercBox*> (p))->color, (static_cast<PercBox*> (p))->size, p->transform.pos, p->transform.rot);
      percepted_obj.push_back(obj);
    }
  }

  // see if there is a matching objects
  std::vector<int> updatet;
  std::vector<int> matching;
  for (HRIObject p_obj:percepted_obj) {
    if (objColor==p_obj.color && objSize==p_obj.size){
      match_found = true;
      matching_obj = p_obj;
    }
  }
  if (!match_found){
    // see if there is a partial match
    for (HRIObject p_obj:percepted_obj) {
      if (objColor==p_obj.color){
        match_found = true;
        matching_obj = p_obj;
      }
    }
  }
  if (match_found){
    double dist = sqrt((matching_obj.pos.x-targetPos.x)*(matching_obj.pos.x-targetPos.x)+(matching_obj.pos.y-targetPos.y)*(matching_obj.pos.y-targetPos.y));
//    cout<<dist;
    if (dist<0.1){
      objectOnTarget=true;
    }
  }/* else {
    cout<<".";
  }*/
  return objectOnTarget;
}

void OnTableState::updateFromModelworld(Roopi& R,const char* objname) {
  std::stringstream output;
  output<< "update from model world: ";

  if (objname == NULL) {
    objects.clear();
    for (unsigned int i=1; i<=objN; ++i) {
      std::string objname("S" + std::to_string(i));
      if(R.getK()->getShapeByName(objname.c_str()))
        {
          HRIObject obj;
          obj.setByName(R, objname.c_str());
          objects.push_back(obj);
          output<<obj;
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
        output<< "object updated: " << objname;
        output<< objects[i];
//        log(output.str());
        break;
      }
    }
    if (!updated)
      objects.push_back(obj);  //if not in list -> append
  }

  // set above and below relationships to Null if they are no longer overlapping
  for (unsigned int p=0; p<objects.size();++p){
    if (objects[p].below_obj!=NULL){
      if (!objects[p].overlapping(objects[p].below_obj)){
        objects[p].below_obj = NULL;
        output<<"below relationship removed from "<<p<<";";
      }
    }
    if (objects[p].above_obj!=NULL){
      if (!objects[p].overlapping(objects[p].above_obj)){
        objects[p].above_obj = NULL;
        output<<"above relationship removed from "<<p<<";";
      }
    }
  }

  // see if there are any new above and below relationships to be added
  for (unsigned int p=0; p<objects.size();++p){
    if (objects[p].below_obj==NULL){
      double zdiff =100;
      for (unsigned int q=0; q<objects.size();++q) {
        if (p!=q && objects[p].overlapping(&objects[q])) {
          if(objects[p].pos.z < objects[q].pos.z && objects[q].pos.z-objects[p].pos.z < zdiff) {
            zdiff = objects[q].pos.z-objects[p].pos.z;
            objects[p].below_obj=&objects[q];
            output<< "new below: " << p << "," << q << "," << zdiff;
          }
        }
      }
    }
    if (objects[p].above_obj==NULL){
      double zdiffabove =100;
      for (unsigned int q=0; q<objects.size();++q) {
        if (p!=q && objects[p].overlapping(&objects[q])) {
          if(objects[p].pos.z > objects[q].pos.z && objects[p].pos.z-objects[q].pos.z < zdiffabove) {
            zdiffabove = objects[p].pos.z-objects[q].pos.z;
            objects[p].above_obj=&objects[q];
            output<< "new above: " << p << "," << q << "," << zdiffabove;
          }
        }
      }
    }

  }
//  log(output.str());
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
//      body->X.pos.z = obj.pos.z;
      mlr::Shape *s=body->shapes.first();
//      s->X.pos.z = obj.pos.z;
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

void OnTableState::log(std::string s) {
  ofstream myfile;
//  std::string filename="logfile_mode" + std::to_string(mode) + ".txt";
  std::string filename="logfilestate.txt";
  myfile.open (filename, std::ios::out | std::ios::app);
  std::time_t result = std::time(0);
  myfile << result << ", ";
  myfile << s << endl;
  myfile.close();
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
