//
// C++ Implementation: UrgModule
//
// Description: 
//
//
// Author: Nikolay Jetchev,,,, <nikolay@nikolay>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//

#include "UrgModule.h"
#ifdef URGLASER

extern "C" {
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include "urg_ctrl.h"
#include "delay.h"
#include <math.h>
}

void shutdownURG(void* p){ MT_MSG("...");  UrgModule *urg=(UrgModule*)p;  urg->close();  }

struct UrgWorkspace{
  MT::Array<long> data;
  bool isOpen;
  int scan_msec;
  urg_parameter_t *parameter;
  urg_t *urg;
};

arr  laserConvertCartesian(urg_t urg, MT::Array<long>& data,uint n){
  arr Coords(n,2);Coords.setZero();
  int min_length = 0, max_length = 0;
  uint j;
  min_length = urg_getDistanceMin(&urg);
  max_length = urg_getDistanceMax(&urg);
  //cout << "size" << n << " limits: " <<  min_length <<  " " <<max_length << endl;
  
  for (j = 0; j < n; ++j) {
    int x = 0, y = 0;
    long length = data(j);
 // Discard data not within the valid range 
    if (((length <= min_length) || (length >= max_length))) {
     //  cout << length<< " ";
     // x = 0; y = 0;
       // if(j > 0){
      //   x = Coords(j-1,1); y =  -Coords(j-1,0);
      // }
    }else{
      x = length * cos(urg_index2rad(&urg, j));
      y = length * sin(urg_index2rad(&urg, j));
     // printf("%d\t%d\t# %d, %ld\n", x, y, j, length);
    }
    Coords(j,1) = x;
    Coords(j,0) = -y;//switch to get good display, this is calibrated, opengl maps x,y ok, but original code was strange
  }
   
  int N = 0;//count nonzero points
  for(j = 0; j < Coords.d0; j++)
    if(Coords(j,0) !=0 && Coords(j,1) != 0)N++;
   
  arr Coords2(N,2);//remove 00 ppoints
  N = 0;
  for(j = 0; j < Coords.d0; j++)
    if(Coords(j,0) !=0 && Coords(j,1) != 0)Coords2[N++] = Coords[j];
   
  return Coords2;
}

#if 0

   
 MT::Array<arr> captureLaser(){

   
   const char device[] = "/dev/ttyACM3";
	
   int data_max;
   long* data;
   int timestamp;
   int scan_msec;
   urg_parameter_t parameter;
   int ret;
   int n;
   int i;


   /* Connection */
   urg_t urg;
   ret = urg_connect(&urg, device, 115200);//115200
   if (ret < 0) {
     printf("urg_connect: %s\n", urg_getError(&urg));
     exit(1);
   }

   /* Reserve the Receive data buffer */
   data_max = urg_getDataMax(&urg);
   data = (long*)malloc(sizeof(long) * data_max);
   if (data == NULL) {
     perror("data buffer");
     exit(1);
   }
   urg_getParameters(&urg, &parameter);
   scan_msec = urg_getScanMsec(&urg);
	  
   /* Request data using MD command. Receive count is equal to CaptureTimes*/
   /* To obtain continuous data above 100 times, set Data acquisition count to infinity */
   /* urg_setCaptureTimes(&urg, UrgInfinityTimes); */
   assert(CaptureTimes < 100);
   urg_setCaptureTimes(&urg,CaptureTimes);//hack, always set max, this avoid failed buffer !!    
   //if set too high - problems in hardware!!!                                                                       
   ret = urg_requestData(&urg, URG_MD, URG_FIRST, URG_LAST);
   if (ret < 0) {
     printf("error");
   // urg_exit(&urg, "urg_requestData()");
   }
   MT::Array<arr> laserpoints;
   laserpoints.resize(CaptureTimes);
  // Obtain Data  
   for (i = 0; i < CaptureTimes; ++i) {
     delay(scan_msec);
     n = urg_receiveData(&urg, data, data_max);
     if (n < 0) {
       printf("error");     // urg_exit(&urg, "urg_receiveData()");
     }
     timestamp = urg_getRecentTimestamp(&urg);  // Display front distance data with time stamp  
     printf("%d: %ld [mm], %d [msec]\n",
            i, data[parameter.area_front_], timestamp);
     arr cartesian = laserConvertCartesian(urg,data,n);
     laserpoints(i) = cartesian;
   }

   if (CaptureTimes > 99) {   
     urg_laserOff(&urg); // While receiving data more than 99 times, explicitly stop receiving data before disconnecting
   }
   urg_disconnect(&urg);
   free(data);
   return laserpoints;
 }
#endif
 
void UrgModule::scanLine(arr & line){
  //urg_setCaptureTimes(WS->urg,1);
  //int ret = urg_requestData(WS->urg, URG_MD, URG_FIRST, URG_LAST);
   int ret = urg_requestData(WS->urg, URG_GD, URG_FIRST, URG_LAST);                        
  if (ret < 0) {
    printf("error");
   // urg_exit(&WS->urg, "urg_requestData()");
  }
  //delay(WS->scan_msec);
  int n = urg_receiveData(WS->urg, WS->data.p, WS->data.N);
   
  //int timestamp = urg_getRecentTimestamp(WS->urg);  // Display front distance data with time stamp
  //printf("%d: %ld [mm], %d [msec]\n", 0, WS->data(WS->parameter->area_front_), timestamp);
  line = laserConvertCartesian(*WS->urg,WS->data,n);
}
 
UrgModule::UrgModule(){
  WS = new UrgWorkspace;
  WS->urg = new urg_t();
  WS->parameter = new urg_parameter_t();
  WS->isOpen = false;
  //cout <<"URG constructor" <<endl ;
}
 
UrgModule::~UrgModule(){
  if(WS->isOpen) close();
  //cout <<"URG destructor" <<endl;
  delete WS->parameter;
  delete WS->urg;
  delete WS;
}
 
void UrgModule::open(){
  cout <<" -- UrgModule init .." <<std::flush;
  const char device[] = "/dev/ttyURG";//link with sudo ln -s with real device name, e.g. ttyACM0
  int ret = urg_connect(WS->urg, device, 115200);//115200
  if (ret < 0) {
    cout << "bad error - have you created a symbolic link /dev/ttyURG?? share/bin/mountHardware!" << endl;
    printf("urg_connect: %s\n", urg_getError(WS->urg));
    exit(1);
  }
  /* Reserve the Receive data buffer */
  uint data_max = urg_getDataMax(WS->urg);
  WS->data.resize(data_max);
  urg_getParameters(WS->urg, WS->parameter);
  WS->scan_msec = urg_getScanMsec(WS->urg);
  
  urg_setCaptureTimes(WS->urg,100000);//should give latest data automatically
  cout <<" done" <<endl;
  WS->isOpen=true;
}
 
void UrgModule::close(){
  CHECK(WS->isOpen,"laser module was not open!");
  cout <<" -- UrgModule close .." <<std::flush;
  urg_laserOff  (WS->urg);//important or not ??
  urg_disconnect(WS->urg);
  cout <<" done" <<endl;
}
 
#else
void UrgModule::scanLine(arr & line){HALT("URG dummy");}
UrgModule::UrgModule():StepThread("Urg"){}
UrgModule::~UrgModule(){}
void UrgModule::open(){HALT("URG dummy");}
void UrgModule::close(){HALT("URG dummy");}
#endif