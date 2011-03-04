
//#define MT_IMPLEMENTATION
#include <MT/util.h>
#include <MT/array.h>
#include <signal.h>

#include<MT/schunk.h>
#include <sdh/sdh.h>

void testMove(){
  signal(SIGINT,schunkEmergencyShutdown);
  SchunkHandModule sdh;
  sdh.open();

  /*cout <<"** pinch" <<endl;
  sdh.movePinch(8);
  sdh.movePinch(0);
  sdh.closeSDH();
  return;*/
  
  cout <<"** single axis vel control" <<endl;
  sdh.hand->SetController( sdh.hand->eCT_POSE );
  //sdh.moveAll(ARR( 0,  -20,   0,  60,  90,  -20, 0),true);
  //MT::wait();
  sdh.moveAll(ARR( 0,  -20,  0,  0,  0,  -20, 0),true);
  MT::wait();
  sdh.moveAll(ARR( 0,  -20,   0,  60,  90,  -20, 0),true);
  MT::wait();
  sdh.moveAll(ARR( 0,  0, -20,  60,  90,  0, -20),true);
  MT::wait();
  sdh.moveAll(ARR( 0,  0,  0,  0,  0,  0,  0),true);
  MT::wait();
  
  std::cout << "Now move back and forth with \"velocity with acceleration ramp\" controller type:\n";
        
  sdh.hand->SetController( sdh.hand->eCT_VELOCITY_ACCELERATION );

  int axis_index = 2;

  // In this controller mode we must switch the power on explicitly:
  // (OK, here the power is switched on already since we used sdh.hand->MoveHand() before.)
  sdh.hand->SetAxisEnable( axis_index, true );

  struct sVA{
    double velocity;
    double acceleration;
  };
  struct sVA va[] = { {40.0,10.0}, {40.0,20.0}, {40.0,40.0}, {40.0,80.0}, {40.0,160.0}, {0.0,0.0} };
  for(int i=0; va[i].velocity != 0.0 ; i++){
    for ( double sign = -1.0; sign <= 1.0; sign += 2.0 ){
      std::cout << "Setting target acceleration,velocity= " << std::setw(7) << sign * va[i].acceleration << " deg/(s*s)  " << std::setw(7) << sign * va[i].velocity << " deg/s\n";
      bool position_reached = false;

      MT::timerStart();
      uint k=0;
      while(!position_reached ){
        sdh.setVelocity(axis_index, sign * va[i].velocity , va[i].acceleration); //set vel+acc in each step of a high freq loop
        
        //std::cout << "  Actual angle: " << std::setw(7) << sdh.hand->GetAxisActualAngle( axis_index ) << " deg";
        //std::cout << ",  actual velocity: " << std::setw(7) << sdh.hand->GetAxisActualVelocity(axis_index) << " deg/s";
        //std::cout << ",  reference velocity: " << std::setw(7) << sdh.hand->GetAxisReferenceVelocity(axis_index) << " deg/s";
        std::cout << ",  ellapsed time: " << MT::timerRead()/++k <<endl;
        if ( sign > 0.0 )
          position_reached = (sdh.hand->GetAxisActualAngle(axis_index) >= 10.0);
        else
          position_reached = (sdh.hand->GetAxisActualAngle(axis_index) <= -10.0);
        //MT::wait(0.05);
      }
    }
  }


  sdh.setVelocity(axis_index, 0., 100.);
  MT::wait( 1.);
  return;


  sdh.hand->SetController( sdh.hand->eCT_POSE );
  
  
  cout <<"** single axis position control" <<endl;
  int i;
  for(i=0;i<7;i++){
    cout <<"axis " <<i <<endl;
    double a=-30, b=0.;
    if(!i) a=30;
    sdh.move(i,a,true);
    sdh.move(i,b,true);
  }
  sdh.stop();
  return;

  cout <<"** all axis position control" <<endl;
  sdh.moveAll(ARR( 90,  0,  0,  0,  0,  0,  0),true);
  sdh.moveAll(ARR( 90,-20,-20,-20,-20,-20,-20),true);
  sdh.moveAll(ARR(  0,-20,-20,-20,-20,-20,-20),true);
  sdh.moveAll(ARR(  0,  0,  0,  0,  0,  0,  0),true);
  sdh.stop();

  sdh.close();
}

int main( int argc, char** argv ){
  testMove();
}

