#include "act_Recorder.h"

struct sAct_Recorder : Thread{
  Access<arr> access;
  arr buffer;
  mlr::String filname;
  ofstream fil;
  uint subSample, lastSample=0;
  double lastTime=0.;

  sAct_Recorder(const char* var_name, uint subSample)
    : Thread(STRING("Act_Recorder_"<<var_name)), access(this, var_name, true), subSample(subSample){
    threadOpen();
  }
  ~sAct_Recorder(){ threadClose(); }

  void open(){
    system("mkdir -p log");
    filname <<"log/" <<access.name <<'.' <<mlr::getNowString() <<".dat";
    mlr::open(fil, filname);
  }
  void step(){
    uint rev = access.readAccess();
    if(rev/subSample <= lastSample){
      access.deAccess();
      return;
    }else{
      lastSample = rev/subSample;
    }
    buffer = access();
    double time = access.data->write_time;
    access.deAccess();
    mlr::String tag;
    tag.printf("%6i %13.6f", rev, time);
    fil <<tag <<' ' <<buffer <<endl;

    if(time-lastTime>.3){
      mlr::String plt;
      plt <<"plot";
      if(rev>500) plt <<" [" <<rev-500 <<':' <<rev+100 <<']';
      plt <<" '" <<filname <<"' ";
      for(uint i=0;i<buffer.N;i++) plt <<"us 1:" <<3+i <<" notitle, ''";
      plt.resize(plt.N-4, true);
      gnuplot(plt, false, false);
      lastTime = time;
    }
  }
  void close(){
    fil.close();
  }
};


Act_Recorder::Act_Recorder(Roopi* r, const char* var_name, uint subSample) : Act(r), s(new sAct_Recorder(var_name, subSample)){}
Act_Recorder::~Act_Recorder(){ delete s; }
