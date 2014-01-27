#include <Core/util.h>

#include <Hardware/racer/MD25.h>


int main() {
  MD25* motor = new MD25();

  
  ofstream fil("nogit-data/motor.dat");
  fil <<"time vel acc enc0 enc1" <<endl;
  
  int32_t encoder1 = 0;
  int32_t encoder2 = 0;
  int vel,acc;

  if (motor->open()) std::cout << "Open connection to MD25 successfully " << std::endl;

  for(;;){
    acc=1;
    motor->setMotorSpeedAndAcceleration("", vel, vel, acc);
    motor->readEncoder1(encoder1);
    motor->readEncoder2(encoder2);

    double time=MT::realTime();
    fil <<time <<' ' <<vel <<' ' <<acc <<' ' <<encoder1 <<' ' <<encoder2 <<endl;

    if(time>10.) break;
  }
  
  motor->close();
  fil.close();
  return 0;
}


