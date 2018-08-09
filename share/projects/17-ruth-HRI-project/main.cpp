#include "BridgeTask.h"
#include "StackTask.h"
#include "SortTask.h"
#include "BalanceTask.h"
#include <Core/util.h>
#include <iostream>
//#include <sound_play/sound_play.h>

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);
//  ros::init(argc, argv, "");
//  ros::NodeHandle nh;
//  while (!nh.ok()){
//    cout<<".";
//  }
//  system("rosrun sound_play say.py \"hello world\"");
//  sound_play::SoundClient sc;
//  sc.say("sound play test","voice_kal_diphone");
//  system("echo \"please place the large blue block\" | festival --tts");
  cout << "What interaction mode do you want to use?" << endl;
  cout << "0 = Autonomous, 1 = Human Commands, 2 = Robot Commands, 3 = Autonomous Plus" << endl;
  int mode=0;
  std::cin >> mode;
  cout << "Which task do you want to perform?" << endl;
  cout << "0 = Sort, 1 = Stack, 2 = Build, 3 = Balance" << endl;
  int taskType = 0;
  std::cin >> taskType;
/*  if (mode==0){
    system("rosrun sound_play say.py \"autonomous strategy \"");
  } else if (mode==1){
    system("rosrun sound_play say.py \"human commands strategy\"");
  } else if (mode==2){
    system("rosrun sound_play say.py \"robot commands strategy\"");
  }*/
  if (taskType==0){
//    system("rosrun sound_play say.py \"sort task\"");
    SortTask task(mode);
    task.start();
  } else if (taskType==1){
//    system("rosrun sound_play say.py \"stack task\"");
    StackTask task(mode);
    task.start();
  } else if (taskType==2){
//    system("rosrun sound_play say.py \"build task\"");
    BridgeTask task(mode);
    task.start();
  } else if (taskType==3){
//    system("rosrun sound_play say.py \"balance task\"");
    BalanceTask task(mode);
    task.start();
  }
  return 0;
}
