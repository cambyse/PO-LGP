#define MT_IMPLEMENTATION
#include <Core/util.h>
#include <MT/threads.h>
//#include <MT/serial.h>
#include "term.h"

bool driveMode=false;
bool wheelsInitialized=false;

void initWheels(){
  /*
  int r=OpenPort(5,9600,0,NULL);
  cout <<"serial port open? " <<SerialPortIsOpen() <<endl;
  cout <<"Port to wheels opened? " <<r <<endl;
  cout <<"initializing driver ..." <<std::flush;
  MT::wait(2.);
  PutSerialChar(170);
  cout <<"done" <<endl;
  wheelsInitialized = true;
  */
}

void sendWheelCommand(byte address,byte command,byte speed){
  /*
  PutSerialChar(address);
  PutSerialChar(command);
  PutSerialChar(speed);
  PutSerialChar((address + command + speed) & 0x7f);
  */
}

void cmdWheels(int left,int right){
  if(left>=0)  sendWheelCommand(128,0, left);
  else         sendWheelCommand(128,1,-left);
  if(right>=0) sendWheelCommand(128,4, right);
  else         sendWheelCommand(128,5,-right);
}


void writeMem(int i=-1){
  bool stop=false;
  if(i==-1){ mvprintw(0,0,"*** MONITOR ***"); i=0; }
  else stop=true;
  mvprintw(0,20,"time=%.3f",MT::realTime());
  for(;i<nBlocks;i++){
    BlockDescription& b=blockDescription[i];
    mvprintw(1+i,0,b.name);
    for(int j=0;j<b.n;j++){
      switch(b.type){
      case byteT:    mvprintw(1+i,10+j*6,"%5b",((byte*)b.p)[j]);    break;
      case charT:    mvprintw(1+i,10+j*6,"%5c",((char*)b.p)[j]);    break;
      case uintT:    mvprintw(1+i,10+j*6,"%5u",((uint*)b.p)[j]);    break;
      case intT:     mvprintw(1+i,10+j*6,"%5i",((int*)b.p)[j]);     break;
      case doubleT:  mvprintw(1+i,10+j*6,"%5.2f",((double*)b.p)[j]);  break;
      case floatT:   mvprintw(1+i,10+j*6,"%5.2f",((float*)b.p)[j]);   break;
      case boolT:    mvprintw(1+i,10+j*6,"%5i",((bool*)b.p)[j]);    break;
      default: HALT("");
      }
    }
    if(stop) break;
  }
  move(nBlocks+2,0);
  refresh();
}

static void finish(int sig){ termClose(); closeRobotSharedMemory(); exit(0); }

void monitorMain(){
  signal(SIGINT, finish);
  termInit();
  uint i;
  openRobotSharedMemory();

  writeMem();

  for(;;){
    waitForSignal();
    for(i=0;i<nBlocks;i++) if(shm->changed[i]){
      shm->changed[i]=false;
      writeMem(i);
      switch(i){
      case joystickB:
        if(shm->joystick[0]&0x02) driveMode^=1;
        if(driveMode){
          if(!wheelsInitialized) initWheels(); 
          shm->wheels[0]=127-shm->joystick[2];
          shm->wheels[1]=127-shm->joystick[4];
          //cout <<"drive event: ";
          writeBlock(cout,wheelsB);
          //cout <<endl;
          cmdWheels(shm->wheels[0],shm->wheels[1]);
        }
      //case wheelsB:  rai.setWheelSpeed(shm.wheels[0],shm.wheels[1]);  break;
      //default:
        //HALT("");
      }
    }
  }

  closeRobotSharedMemory();
}

void testClient(){
  signal(SIGINT, finish);
  cout <<"*** TEST CLIENT ***" <<endl;
  openRobotSharedMemory();

  for(uint i=1;i<10;i++){
    shm->wheels[0]=i;
    shm->changed[wheelsB]=true;
    sendSignal();
    MT::wait();
  }

  closeRobotSharedMemory();
}

int main(int argc,char** argv){
  if(argc==1){ monitorMain(); return 0; }
  
  if(!strcmp(argv[1],"test"))    testClient();
  if(!strcmp(argv[1],"clean")) destroyRobotSharedMemory();
  
  return 0;
}
