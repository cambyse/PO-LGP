#include <Motion/feedbackControl.h>
#include <Actions/taskCtrlActivities.h>
#include <Actions/swig.h>
#include <string>
#include <vector>
#include <algorithm>
#include <iterator>
#include <iostream>
#include <cassert>
#include <sstream>


int main(int argc, char** argv) {

  ActionSwigInterface S;

  std::vector<float> v;

  std::stringstream ss;

  S.createNewSymbol("wheels"); //-> wird automatisiert
  for (int i=0; i < 5; i++){
    ss.str("");
    ss.clear();
    v.clear();
    ss << (S.getJointByName("worldTranslationRotation")["q"]);
    std::copy(std::istream_iterator<float>(ss), std::istream_iterator<float>(), std::back_inserter(v));
    ss.str("");
    ss.clear();
    
    ss << "(FollowReferenceActivity wheels){ type=wheels, target=[0, 0, " << (v[2] - 1.6) <<"] PD=[1,1,1,10]}";
    

    cout << "#################" << endl<< "worldtranslationrotation Rotation ist:" << v[2] << endl << "worldtranslationrotation Rotation soll:" << (v[2] - 1.6) << endl << "#################" << endl;
    cout << endl << "#################" << endl << "Wird Uebergeben: " << ss.str() << endl << "##################" << endl;
    
    S.setFact(ss.str().c_str());
    
    S.waitForCondition("(conv FollowReferenceActivity wheels)");
    S.stopFact("(FollowReferenceActivity wheels)");
    S.stopFact("(conv FollowReferenceActivity wheels)");

  }

  return 0;
}


