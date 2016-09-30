#include <Motion/komo.h>
#include <string>
#include <map>

using namespace std;

struct HandPositionMap:TaskMap{
  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G, int t=-1){
    ors::Body *arm = G.getBodyByName("left_wrist");
    arr posArm, Jarm;
    G.kinematicsPos(posArm, Jarm, arm);

    posArm -= ARR(.5,.5,1.3);

    y = posArm;
    J = Jarm;
  }

  virtual uint dim_phi(const ors::KinematicWorld& G){
    return 3;
  }

  virtual mlr::String shortTag(const ors::KinematicWorld& G){ return mlr::String("RebaMap"); }

};

//===========================================================================

struct RebaMap:TaskMap{
	map<const string, arr> coeffs_map;
	const int nb_joints = 18;
	const string joint_names[18] = {"neck_0", "neck_1", "neck_2",
	                                "spine_0", "spine_1", "spine_2",
	                                "right_knee", "left_knee",
	                                "right_shoulder_0", "right_shoulder_1",
	                                "left_shoulder_0", "left_shoulder_1",
	                                "right_elbow_0", "left_elbow_0",
	                                "right_wrist_0", "right_wrist_1",
	                                "left_wrist_0", "left_wrist_1"};

	RebaMap(){
		initCoefficientMap(coeffs_map);
	}

	void initCoefficientMap( map<const string, arr>& theMap ){
		theMap["neck_0"] = ARR(0.45594512, -0.07958066,  1.00000056);
		theMap["neck_1"] = ARR(0.45594512, -0.07958066,  1.00000056);
		theMap["neck_2"] = ARR(0.45594512, -0.07958066,  1.00000056);
		theMap["spine_0"] = ARR(1.82377278,  0.        ,  1.        );          
		theMap["spine_1"] = ARR(1.82377278,  0.        ,  1.        );   
		theMap["spine_2"] = ARR(1.82377278,  0.        ,  1.        );
		theMap["right_knee"] = ARR(1.82377278,  0.        ,  1.        );       
		theMap["left_knee"] = ARR(1.82377278,  0.        ,  1.        ); 
		theMap["right_shoulder_0"] = ARR(1.21587174, -3.81974618,  4.        ); 
		theMap["right_shoulder_1"] = ARR(1.21584852,  0.        ,  1.        );
		theMap["left_shoulder_0"] = ARR(1.21587174, -3.81974618,  4.        );  
		theMap["left_shoulder_1"] = ARR(1.21584852,  0.        ,  1.        );
		theMap["right_elbow_0"] = ARR(1.36783611, -2.3873254 ,  2.        );    
		theMap["left_elbow_0"] = ARR(1.36783611,  2.3873254 ,  2.        ); 
		theMap["right_wrist_0"] = ARR(1.62113894,  0.        ,  1.        );    
		theMap["right_wrist_1"] = ARR(1.62113894,  0.        ,  1.        ); 
		theMap["left_wrist_0"] = ARR(1.62113894,  0.        ,  1.        );     
		theMap["left_wrist_1"] = ARR(1.62113894,  0.        ,  1.        );
	}

  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G, int t=-1){
    y = ARR();
    if(&J){
      J = zeros(nb_joints, G.q.N);
    }

    for (int i=0; i<nb_joints; ++i){
    	const string name = joint_names[i];
    	arr coeffs = coeffs_map[name];
    	ors::Joint *joint = G.getJointByName(name.c_str());
    	float joint_value = G.q(joint->qIndex);
    	y.append(coeffs(0) * joint_value * joint_value + coeffs(1) * joint_value + coeffs(2));
    	J(i, joint->qIndex) = 2 * coeffs(0) * joint_value + coeffs(1);
    }
  }


  virtual uint dim_phi(const ors::KinematicWorld& G){
    return nb_joints;
  }

  virtual mlr::String shortTag(const ors::KinematicWorld& G){ return mlr::String("RebaMap"); }

};

//===========================================================================

void moveReba(){
  
  KOMO komo;
  komo.setConfigFromFile();

//  komo.setHoming(-1., -1., 1e-1);
//  komo.setSquaredQVelocities();
  komo.setSquaredQAccelerations();
#if 0
  komo.setPosition(1., 1., "endeffL", "target", sumOfSqrTT, NoArr, 1e2);
#else
  komo.setTask(0., 1., new RebaMap(), sumOfSqrTT, NoArr, 1e2);
#endif
  komo.setSlowAround(1., .1, 1e3);

  komo.reset();
  komo.run();
  komo.checkGradients();

  Graph result = komo.getReport(true);

  for(;;) komo.displayTrajectory(.1, true);
}

//===========================================================================

int main(int argc,char** argv){
  mlr::initCmdLine(argc,argv);

  moveReba();

  return 0;
}