#include <GL/glew.h>
/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#include "util.h"
#include <math.h>
#include <string.h>
#include <signal.h>
#include <stdexcept>
#include <stdarg.h>
#if defined MLR_Linux || defined MLR_Cygwin || defined MLR_Darwin
#  include <limits.h>
#  include <sys/time.h>
#  include <sys/times.h>
#  include <sys/resource.h>
#  include <sys/inotify.h>
#  include <sys/stat.h>
#  include <poll.h>
#  include <X11/Xlib.h>
#  include <X11/Xutil.h>
#endif
#ifdef __CYGWIN__
#include "cygwin_compat.h"
#endif
#if defined MLR_MSVC
#  include <time.h>
#  include <sys/timeb.h>
#  include <windows.h>
#  undef min
#  undef max
#  define MLR_TIMEB
#  ifdef MLR_QT
#    undef  NOUNICODE
#    define NOUNICODE
#  endif
#  pragma warning(disable: 4305 4244 4250 4355 4786 4996)
#endif
#if defined MLR_MinGW
#  include <unistd.h>
#  include <sys/time.h>
#  include <sys/timeb.h>
#  define MLR_TIMEB
#endif

#ifdef MLR_QT
#  undef scroll
#  undef Unsorted
#  include <Qt/qmetatype.h>
#  include <Qt/qdatastream.h>
#  include <Qt/qapplication.h>
#  include <QThread>
#endif

#ifndef MLR_ConfigFileName
#  define MLR_ConfigFileName "MT.cfg"
#  define MLR_LogFileName "MT.log"
#endif

#include <errno.h>
#ifndef MLR_MSVC
#  include <unistd.h>
#  include <sys/syscall.h>
#endif


//===========================================================================
//
// Bag container
//

const char *mlr::String::readSkipSymbols = " \t";
const char *mlr::String::readStopSymbols = "\n\r";
int   mlr::String::readEatStopSymbol     = 1;
mlr::String mlr::errString;
Mutex coutMutex;
mlr::LogObject _log("global", 2, 3);


//===========================================================================
//
// utilities in mlr namespace
//

namespace mlr {
int argc;
char** argv;
bool IOraw=false;
bool noLog=true;
uint lineCount=1;
int verboseLevel=-1;
int interactivity=-1;

double startTime;
double timerStartTime=0.;
double timerPauseTime=-1.;
bool timerUseRealTime=false;

#ifdef MLR_QT
QApplication *myApp=NULL;
#endif

/// open an output-file with name '\c name'
void open(std::ofstream& fs, const char *name, const char *errmsg) {
  fs.clear();
  fs.open(name);
  LOG(3) <<"opening output file `" <<name <<"'" <<std::endl;
  if(!fs.good()) MLR_MSG("could not open file `" <<name <<"' for output" <<errmsg);
}

/// open an input-file with name '\c name'
void open(std::ifstream& fs, const char *name, const char *errmsg) {
  fs.clear();
  fs.open(name);
  LOG(3) <<"opening input file `" <<name <<"'" <<std::endl;
  if(!fs.good()) HALT("could not open file `" <<name <<"' for input" <<errmsg);
}

/// returns true if the (0-terminated) string s contains c
bool contains(const char *s, char c) {
  if(!s) return false;
  for(uint i=0; s[i]; i++) if(s[i]==c) return true;
  return false;
}

/// skips the chars (typically white characters) when parsing from the istream, returns first non-skipped char
char skip(std::istream& is, const char *skipSymbols, const char *stopSymbols, bool skipCommentLines) {
  char c;
  for(;;) {
    c=is.get();
    if(skipCommentLines && c=='#') { skipRestOfLine(is); continue; }
    if(skipSymbols && !contains(skipSymbols, c)) break;
    if(stopSymbols && contains(stopSymbols, c)) break;
    if(c=='\n') lineCount++;
  }
  is.putback(c);
  return c;
}

/// skips a newline character (same as skip(is, "\n");)
void skipRestOfLine(std::istream& is) {
  char c;
  do { c=is.get(); } while(c!='\n');
  is.putback(c);
}

/// skips the next character
void skipOne(std::istream& is) {
  is.get();
}

/// tell you about the next char (after skip()) but puts it back in the stream
char getNextChar(std::istream& is, const char *skipSymbols, bool skipCommentLines) {
  char c;
  skip(is, skipSymbols, NULL, skipCommentLines);
  is.get(c);
  if(!is.good()) return 0;
  return c;
}

/// tell you about the next char (after skip()) but puts it back in the stream
char peerNextChar(std::istream& is, const char *skipSymbols, bool skipCommentLines) {
  char c=getNextChar(is, skipSymbols, skipCommentLines);
  if(!is.good()) return 0;
  is.putback(c);
  return c;
}

/// skip throught a stream until the tag is found (which is eaten)
bool skipUntil(std::istream& is, const char *tag) {
  unsigned n=strlen(tag);
  char *buf=new char [n+1];
  memset(buf, 0, n+1);
  while(is.good()) {
    memmove(buf, buf+1, n);
    buf[n-1]=is.get();
    if(buf[n-1]=='\n') lineCount++;
    buf[n]=0;
    if(!strcmp(tag, buf)) { delete[] buf; return true; }
  };
  delete[] buf;
  return false;
}

/// a global operator to scan (parse) strings from a stream
bool parse(std::istream& is, const char *str, bool silent) {
  if(!is.good()) { if(!silent) MLR_MSG("bad stream tag when scanning for `" <<str <<"'"); return false; }  //is.clear(); }
  uint i, n=strlen(str);
  char buf[n+1]; buf[n]=0;
  mlr::skip(is, " \n\r\t");
  is.read(buf, n);
  if(!is.good() || strcmp(str, buf)) {
    for(i=n; i--;) is.putback(buf[i]);
    is.setstate(std::ios::failbit);
    if(!silent)  MLR_MSG("(LINE=" <<mlr::lineCount <<") parsing of constant string `" <<str
                          <<"' failed! (read instead: `" <<buf <<"')");
    return false;
  }
  return true;
}


/// returns the i-th of str
byte bit(byte *str, uint i) { return (str[i>>3] >>(7-(i&7))) & 1; }
//byte bit(byte b, uint i){ return (b >>(7-(i&7))) & 1; }
//void set(byte *state, uint i){ state[i>>3] |= 1 <<(7-(i&7)); }
//void del(byte *state, uint i){ state[i>>3] &= (byte)~(1 <<(7-(i&7))); }
//void flip(byte *str, uint i){ str[i>>3] ^= 1 <<(7-(i&7)); }

/// flips the i-th bit of b
void flip(byte& b, uint i) { b ^= 1 <<(7-(i&7)); }

/// filps the i-th bit of b
void flip(int& b, uint i) { b ^= 1 <<(7-(i&7)); }

double MIN(double a, double b) { return a<b?a:b; }
double MAX(double a, double b) { return a>b?a:b; }
uint MAX(uint a, uint b) { return a>b?a:b; }

double indicate(bool expr){ if(expr) return 1.; return 0.; }

/** @brief the distance between x and y w.r.t.\ a circular topology
    (e.g. modMetric(1, 8, 10)=3) */
double modMetric(double x, double y, double mod) {
  double d=fabs(x-y);
  d=fmod(d, mod);
  if(d>mod/2.) d=mod-d;
  return d;
}

/// the sign (+/-1) of x (+1 for zero)
double sign(double x) { if(x<0.) return -1.; return 1.; }

/// the sign (+/-1) of x (0 for zero)
double sign0(double x) { if(x<0.) return -1.; if(!x) return 0.; return 1.; }

/// returns 0 for x<0, 1 for x>1, x for 0<x<1
double linsig(double x) { if(x<0.) return 0.; if(x>1.) return 1.; return x; }

/// x ends up in the interval [a, b]
//void clip(double& x, double a, double b) { if(x<a) x=a; if(x>b) x=b; }

/// the angle of the vector (x, y) in [-pi, pi]
double phi(double x, double y) {
  if(x==0. || ::fabs(x)<1e-10) { if(y>0.) return MLR_PI/2.; else return -MLR_PI/2.; }
  double p=::atan(y/x);
  if(x<0.) { if(y<0.) p-=MLR_PI; else p+=MLR_PI; }
  if(p>MLR_PI)  p-=2.*MLR_PI;
  if(p<-MLR_PI) p+=2.*MLR_PI;
  return p;
}

/// the change of angle of the vector (x, y) when displaced by (dx, dy)
double dphi(double x, double y, double dx, double dy) {
  //return (dy*x - dx*y)/sqrt(x*x+y*y);
  if(x==0. || ::fabs(x)<1e-10) { if(y>0.) return -dx/y; else return dx/y; }
  double f=y/x;
  return 1./(1.+f*f)*(dy/x - f/x*dx);
}

/** @brief save division, checks for division by zero; force=true will return
  zero if y=0 */
double DIV(double x, double y, bool force) {
  if(x==0.) return 0.;
  if(force) { if(y==0.) return 0.; } else CHECK(y!=0, "Division by Zero!");
  return x/y;
}

double sigmoid11(double x) {
  return x/(1.+::fabs(x));
}

double sigmoid(double x) {
  return 1./(1.+exp(-x));
}

double dsigmoid(double x) {
  double y=sigmoid(x);
  return y*(1.-y);
}

#define AXETS 1280
#define AXETR 10.
/// approximate exp (sets up a static value table)
double approxExp(double x) {
  static bool initialized=false;
  static double ExpTable [AXETS]; //table ranges from x=-10 to x=10
  int i;
  if(!initialized) {
    for(i=0; i<AXETS; i++) ExpTable[i]=::exp(AXETR*(2*i-AXETS)/AXETS);
    initialized=true;
  }
  x*=.5*AXETS/AXETR;
  i=(int)x;
  x-=(double)i; //x = residual
  i+=AXETS/2; //zero offset
  if(i>=AXETS-1) return ExpTable[AXETS-1];
  if(i<=0) return 0.; //ExpTable[0];
  return (1.-x)*ExpTable[i] + x*ExpTable[i+1];
}

/// ordinary Log, but cutting off for small values
double Log(double x) {
  if(x<.001) x=.001; return ::log(x);
}

/// integer log2
uint Log2(uint n) {
  uint l=0;
  n=n>>1;
  while(n) { l++; n=n>>1; }
  return l;
}

/// square of a double
double sqr(double x) { return x*x; }

double sinc(double x) {
  if(fabs(x)<1e-10) return 1.-.167*x*x;
  return ::sin(x)/x;
}

double cosc(double x) {
  if(fabs(x)<1e-10) return 1.-.167*x*x;
  return ::cos(x)/x;
}

#define EXP ::exp //mlr::approxExp

double NNsdv(const double& a, const double& b, double sdv){
  double d=(a-b)/sdv;
  double norm = 1./(::sqrt(MLR_2PI)*sdv);
  return norm*EXP(-.5*d*d);
}

double NNsdv(double x, double sdv){
  x/=sdv;
  double norm = 1./(::sqrt(MLR_2PI)*sdv);
  return norm*EXP(-.5*x*x);
}

/* gnuplot:
heavy(x) = (1+sgn(x))/2
eps = 0.1
g(x) = heavy(x-eps)*(x-eps/2) + (1-heavy(x-eps))*x**2/(2*eps)
plot [-.5:.5] g(abs(x))
*/
double POW(double x, double power){ if(power==1.) return x; if(power==2.) return x*x; return pow(x,power); }
double smoothRamp(double x, double eps, double power){
  if(x<0.) return 0.;
  if(power!=1.) return pow(smoothRamp(x,eps,1.),power);
  if(!eps) return x;
  if(x>eps) return x - .5*eps;
  return x*x/(2*eps);
}

double d_smoothRamp(double x, double eps, double power){
  if(x<0.) return 0.;
  if(power!=1.) return power*pow(smoothRamp(x,eps,1.),power-1.)*d_smoothRamp(x,eps,1.);
  if(!eps || x>eps) return 1.;
  return x/eps;
}

/*
heavy(x) = (1+sgn(x))/2
power = 1.5
margin = 1.5
f(x) = heavy(x)*x**power
plot f(x/margin+1), 1
*/
double ineqConstraintCost(double g, double margin, double power){
  double y=g+margin;
  if(y<0.) return 0.;
  if(power==1.) return y;
  if(power==2.) return y*y;
  return pow(y,power);
}

double d_ineqConstraintCost(double g, double margin, double power){
  double y=g+margin;
  if(y<0.) return 0.;
  if(power==1.) return 1.;
  if(power==2.) return 2.*y;
  return power*pow(y,power-1.);
}

double eqConstraintCost(double h, double margin, double power){
  double y=h/margin;
  if(power==1.) return fabs(y);
  if(power==2.) return y*y;
  return pow(fabs(y),power);
}

double d_eqConstraintCost(double h, double margin, double power){
  double y=h/margin;
  if(power==1.) return mlr::sign(y)/margin;
  if(power==2.) return 2.*y/margin;
  return power*pow(y,power-1.)*mlr::sign(y)/margin;
}

/** @brief double time on the clock
  (probably in micro second resolution) -- Windows checked! */
double clockTime(bool today) {
#ifndef MLR_TIMEB
  timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  if(today) ts.tv_sec = ts.tv_sec%86400; //modulo TODAY
  return ((double)(ts.tv_sec) + 1e-9d*(double)(ts.tv_nsec));
//  static timeval t; gettimeofday(&t, 0);
//  return ((double)(t.tv_sec%86400) + //modulo TODAY
//          (double)(t.tv_usec)/1000000.);
#else
  static _timeb t; _ftime(&t);
  return ((double)(t.time%86400) + //modulo TODAY
          (double)(t.millitm-startTime.millitm)/1000.);
#endif
}

timespec clockTime2(){
  timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  return ts;
}

double toTime(const tm& t) {
    return (double)(mktime(const_cast<tm*>(&t)) % 86400);
}

/** @brief double time since start of the process in floating-point seconds
  (probably in micro second resolution) -- Windows checked! */
double realTime() {
  return clockTime(false)-startTime;
}

/** @brief user CPU time of this process in floating-point seconds (pure
  processor time) -- Windows checked! */
double cpuTime() {
#ifndef MLR_TIMEB
  tms t; times(&t);
  return ((double)t.tms_utime)/sysconf(_SC_CLK_TCK);
#else
  return ((double)clock())/CLOCKS_PER_SEC; //MSVC: CLOCKS_PER_SEC=1000
#endif
}

/** @brief system CPU time of this process in floating-point seconds (the
  time spend for file input/output, x-server stuff, etc.)
  -- not implemented for Windows! */
double sysTime() {
#ifndef MLR_TIMEB
  tms t; times(&t);
  return ((double)(t.tms_stime))/sysconf(_SC_CLK_TCK);
#else
  HALT("sysTime() is not implemented for Windows!");
  return 0.;
#endif
}

/** @brief total CPU time of this process in floating-point seconds (same
  as cpuTime + sysTime) -- not implemented for Windows! */
double totalTime() {
#ifndef MLR_TIMEB
  tms t; times(&t);
  return ((double)(t.tms_utime+t.tms_stime))/sysconf(_SC_CLK_TCK);
#else
  HALT("totalTime() is not implemented for Windows!");
  return 0.;
#endif
}

/// the absolute double time and date as string
char *date() {
  return date(clockTime(false));
}

char *date(double sec){
  time_t nowtime;
  struct tm *nowtm;
  static char tmbuf[64], buf[64];

  nowtime = (long)(floor(sec));
  sec -= (double)nowtime;
  nowtm = localtime(&nowtime);
  strftime(tmbuf, sizeof tmbuf, "%Y-%m-%d %H:%M:%S", nowtm);
  snprintf(buf, sizeof buf, "%s.%06ld", tmbuf, (long)(floor(1e6d*sec)));
  return buf;
}

/// wait double time
void wait(double sec, bool msg_on_fail) {
#if defined(MLR_Darwin)
  sleep((int)sec);
#elif !defined(MLR_MSVC)
  timespec ts;
  ts.tv_sec = (long)(floor(sec));
  sec -= (double)ts.tv_sec;
  ts.tv_nsec = (long)(floor(1e9d*sec));
  int rc = clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, NULL);
  if(rc && msg_on_fail){
    MLR_MSG("clock_nanosleep() failed " <<rc <<" '" <<strerror(rc) <<"' trying select instead");
    timeval tv;
    tv.tv_sec = ts.tv_sec;
    tv.tv_usec = ts.tv_nsec/1000l;
    rc = select(1, NULL, NULL, NULL, &tv);
    if(rc==-1) MLR_MSG("select() failed " <<rc <<" '" <<strerror(errno) <<"'");
  }
#else
  Sleep((int)(1000.*sec));
  //MsgWaitForMultipleObjects( 0, NULL, FALSE, (int)(1000.*sec), QS_ALLEVENTS);
#endif

#if 0
#ifndef MLR_TIMEB
  /* r=0 time is up
     r!=0 data in NULL stream available (nonsense)
     */
#else
  double t=realTime(); while(realTime()-t<sec);
#  endif
#endif
}

/// wait for an ENTER at the console
bool wait(bool useX11) {
  if(!mlr::getInteractivity()){
    mlr::wait(.05);
    return true;
  }
  if(!useX11){
    char c[10];
    std::cout <<" -- hit a key to continue..." <<std::flush;
    //cbreak(); getch();
    std::cin.getline(c, 10);
    std::cout <<"\r" <<std::flush;
    if(c[0]==' ') return true;
    else return false;
    return true;
  }else{
    char c = x11_getKey();
    if(c==' ') return true;
    return false;
  }
}

int x11_getKey(){
  mlr::String txt="PRESS KEY";
  int key=0;

  Display *disp = XOpenDisplay(NULL);
  CHECK(disp, "Cannot open display");

  Window win = XCreateSimpleWindow(disp, DefaultRootWindow(disp),
                                   10, 10, 80, 50, //24
                                   2, 0x000000, 0x20a0f0);
  XSelectInput (disp, win, KeyPressMask | ExposureMask | ButtonPressMask );
  XMapWindow(disp, win);

  GC gc = XCreateGC(disp, win, 0, NULL);
  XSetFont(disp, gc,  XLoadFont(disp,"fixed")); //-adobe-courier-bold-r-*-*-*-220-*-*-*-*-*-*"));
  XSetForeground(disp, gc, 0x000000);

  bool quit=false;
  for(;!quit;){
    XEvent ev;
    XNextEvent(disp, &ev);
    switch(ev.type){
      case Expose:
        if (ev.xexpose.count == 0) {
          XDrawString(disp, win, gc, 12, 30, txt.p, txt.N);
          XFlush(disp);
        }
        break;
      case KeyPress:
        char string[4];
        XLookupString(&ev.xkey, string, 4, NULL, NULL);
        key = string[0];
        quit=true;
        break;
      case ButtonPress:
        quit=true;
        break;
    }
  }

  XCloseDisplay(disp);
  return key;
}

/// the integral shared memory size -- not implemented for Windows!
long mem() {
#ifndef MLR_TIMEB
  static rusage r; getrusage(RUSAGE_SELF, &r);
  return r.ru_idrss;
#else
  HALT("mlr::mem() is not implemented for Windows!");
  return 0;
#endif
}

/// start and reset the timer (user CPU time)
void timerStart(bool useRealTime) {
  if(useRealTime) timerUseRealTime=true; else timerUseRealTime=false;
  timerPauseTime=-1.;
  timerStartTime=(timerUseRealTime?realTime():cpuTime());
}

/// read the timer and optionally also reset it (user CPU time)
double timerRead(bool reset) {
  double c;
  if(timerPauseTime!=-1.) c=timerPauseTime; else c=(timerUseRealTime?realTime():cpuTime())-timerStartTime;
  if(reset) timerStart(timerUseRealTime);
  return c;
}

/// read the timer relative to a given start time (user CPU time)
double timerRead(bool reset, double startTime) {
  double c=(timerUseRealTime?realTime():cpuTime())-startTime;
  if(reset) timerStart(timerUseRealTime);
  return c;
}

/// read and pause the timer
double timerPause() {
  timerPauseTime=(timerUseRealTime?realTime():cpuTime())-timerStartTime;
  return timerPauseTime;
}

/// resume the timer after a pause, neglecting the time inbetween
void timerResume() {
  timerStartTime=(timerUseRealTime?realTime():cpuTime())-timerPauseTime;
  timerPauseTime=-1.;
}

/// memorize the command line arguments and open a log file
void initCmdLine(int _argc, char *_argv[]) {
  argc=_argc; argv=_argv;
  mlr::String msg;
  msg <<"** cmd line arguments: '"; for(int i=0; i<argc; i++) msg <<argv[i] <<' ';
  msg <<"\b'";
  LOG(1) <<msg;
}

/// returns true if the tag was found on command line
bool checkCmdLineTag(const char *tag) {
  for(int n=1; n<argc; n++) if(argv[n][0]=='-' && !strcmp(tag, argv[n]+1)) {
      return true;
    }
  return false;
}

/// returns the argument after the cmd-line tag; NULL if the tag is not found
char *getCmdLineArgument(const char *tag) {
  int n;
  for(n=1; n<argc; n++) if(argv[n][0]=='-' && !strcmp(tag, argv[n]+1)) {
      if(n+1==argc) return (char*)"1";
      return argv[n+1];
    }
  return NULL;
}

String mlrPath(const char* rel){
  String path(MLR_CORE_PATH);
  path <<"/../../" <<rel;
  return path;
}

uint getVerboseLevel() {
  if(verboseLevel==-1) verboseLevel=getParameter<int>("verbose", 0);
  return verboseLevel;
}

bool getInteractivity(){
  if(interactivity==-1) interactivity=(checkParameter<bool>("noInteractivity")?0:1);
  return interactivity==1;
}

}//namespace mlr

//===========================================================================
//
// logging

namespace mlr {
  void handleSIGUSR2(int){
    int i=5;
    i*=i;    //set a break point here, if you want to catch errors directly
  }

struct LogServer {
  LogServer() {
    signal(SIGUSR2, mlr::handleSIGUSR2);
    timerStartTime=mlr::cpuTime();
    startTime = clockTime(false);
  }

  ~LogServer() {
  }
};

Singleton<mlr::LogServer> logServer;
}

mlr::LogObject::LogObject(const char* key, int defaultLogCoutLevel, int defaultLogFileLevel)
  : key(key), logCoutLevel(defaultLogCoutLevel), logFileLevel(defaultLogFileLevel){
  if(!strcmp(key,"global")){
    fil.open("z.log.global");
    fil <<"** compiled at:     " <<__DATE__ <<" " <<__TIME__ <<'\n';
    fil <<"** execution start: " <<mlr::date(mlr::startTime) <<std::endl;
  }else{
    logCoutLevel = mlr::getParameter<int>(STRING("logCoutLevel_"<<key), logCoutLevel);
    logFileLevel = mlr::getParameter<int>(STRING("logFileLevel_"<<key), logFileLevel);
  }
}

mlr::LogObject::~LogObject(){
  if(!strcmp(key,"global")){
    fil <<"** execution stop: " <<mlr::date()
       <<"\n** real time: " <<mlr::realTime()
      <<"sec\n** CPU time: " <<mlr::cpuTime()
     <<"sec\n** system (includes I/O) time: " <<mlr::sysTime() <<"sec" <<std::endl;
  }
  fil.close();
}

mlr::LogToken mlr::LogObject::getToken(int log_level, const char* code_file, const char* code_func, uint code_line) {
  return mlr::LogToken(*this, log_level, code_file, code_func, code_line);
}

mlr::LogToken::~LogToken(){
  auto mut = mlr::logServer(); //keep the mutex
  if(log.logFileLevel>=log_level){
    if(!log.fil.is_open()) mlr::open(log.fil, STRING("z.log."<<log.key));
    log.fil <<code_func <<':' <<code_file <<':' <<code_line <<'(' <<log_level <<") " <<msg <<endl;
  }
  if(log.logCoutLevel>=log_level){
    if(log_level>=0) std::cout <<code_func <<':' <<code_file <<':' <<code_line <<'(' <<log_level <<") " <<msg <<endl;
    if(log_level<0){
      mlr::errString.clear() <<code_func <<':' <<code_file <<':' <<code_line <<'(' <<log_level <<") " <<msg;
#ifdef MLR_ROS
      ROS_INFO("MLR-MSG: %s",mlr::errString.p);
#endif
      if(log_level==-1){ mlr::errString <<" -- WARNING";    cout <<mlr::errString <<endl; }
      if(log_level==-2){ mlr::errString <<" -- ERROR  ";    cerr <<mlr::errString <<endl; /*throw does not WORK!!! Because this is a destructor. The THROW macro does it inline*/ }
      if(log_level==-3){ mlr::errString <<" -- HARD EXIT!"; cerr <<mlr::errString <<endl; /*mlr::logServer().mutex.unlock();*/ exit(1); }
      if(log_level<=-2) raise(SIGUSR2);
    }
  }
//  mlr::logServer().mutex.unlock();
}

void setLogLevels(int fileLogLevel, int consoleLogLevel){
  _log.logCoutLevel=consoleLogLevel;
  _log.logFileLevel=fileLogLevel;
}


//===========================================================================
//
// parameters

namespace mlr{

}

/// a global operator to scan (parse) strings from a stream
std::istream& operator>>(std::istream& is, const PARSE& x) {
  mlr::parse(is, x.str); return is;
}

/// the same global operator for non-const string
std::istream& operator>>(std::istream& is, char *str) {
  mlr::parse(is, (const char*)str); return is;
}


//===========================================================================
//
// String class
//

int mlr::String::StringBuf::overflow(int C) {
  string->append(C);
  return C;
}

int mlr::String::StringBuf::sync() {
  if(string->flushCallback) string->flushCallback(*string);
  return 0;
}

void mlr::String::StringBuf::setIpos(char *p) { setg(string->p, p, string->p+string->N); }

char *mlr::String::StringBuf::getIpos() { return gptr(); }

//-- direct memory operations
void mlr::String::append(char x) { resize(N+1, true); operator()(N-1)=x; }

mlr::String& mlr::String::setRandom(){
  resize(rnd(2,6), false);
  for(uint i=0;i<N;i++) operator()(i)=rnd('a','z');
  return *this;
}

void mlr::String::resize(uint n, bool copy) {
  if(N==n && M>N) return;
  char *pold=p;
  uint Mold=M;
  //flexible allocation (more than needed in case of multiple resizes)
  if(M==0) {  //first time
    M=n+1;
  } else if(n+1>M || 10+2*n<M/2) {
    M=11+2*n;
  }
  if(M!=Mold) { //do we actually have to allocate?
    p=new char [M];
    if(!p) HALT("mlr::Mem failed memory allocation of " <<M <<"bytes");
    if(copy) memmove(p, pold, N<n?N:n);
    if(Mold) delete[] pold;
  }
  N=n;
  p[N]=0;
  resetIstream();
}

void mlr::String::init() { p=0; N=0; M=0; buffer.string=this; flushCallback=NULL; }

/// standard constructor
mlr::String::String() : std::iostream(&buffer) { init(); clearStream(); }

/// copy constructor
mlr::String::String(const String& s) : std::iostream(&buffer) { init(); this->operator=(s); }

/// copy constructor for an ordinary C-string (needs to be 0-terminated)
mlr::String::String(const char *s) : std::iostream(&buffer) { init(); this->operator=(s); }

mlr::String::String(const std::string& s) : std::iostream(&buffer) { init(); this->operator=(s.c_str()); }

mlr::String::String(std::istream& is) : std::iostream(&buffer) { init(); read(is, "", "", 0); }

mlr::String::~String() { if(M) delete[] p; }

/// returns a reference to this
std::iostream& mlr::String::stream() { return (std::iostream&)(*this); }

/// returns a reference to this
mlr::String& mlr::String::operator()() { return *this; }

/** @brief returns the true memory buffer (C-string) of this class (which is
always kept 0-terminated) */
mlr::String::operator char*() { return p; }

/// as above but const
mlr::String::operator const char*() const { return p; }

/// returns the i-th char
char& mlr::String::operator()(uint i) const { CHECK(i<=N, "String range error (" <<i <<"<=" <<N <<")"); return p[i]; }

/// return the substring from `start` to (exclusive) `end`.
mlr::String mlr::String::getSubString(uint start, uint end) const {
  CHECK(start < end, "getSubString: start should be smaller than end");
  clip(end, uint(0), N);
  String tmp;
  for (uint i = start; i < end; i++) {
    tmp.append((*this)(i));
  }
  return tmp;
}

/**
 * @brief Return the last `n` chars of the string.
 * @param n number of chars to return
 */
mlr::String mlr::String::getLastN(uint n) const {
  clip(n, uint(0), N);
  return getSubString(N-n, N);
}

/**
 * @brief Return the first `n` chars of the string.
 * @param n number of chars to return.
 */
mlr::String mlr::String::getFirstN(uint n) const {
  clip(n, uint(0), N);
  return getSubString(0, n);
}

/// copy operator
mlr::String& mlr::String::operator=(const String& s) {
  resize(s.N, false);
  memmove(p, s.p, N);
  return *this;
}

/// copies from the C-string
void mlr::String::operator=(const char *s) {
  if(!s){  clear();  return;  }
  uint ls = strlen(s);
  if(!ls){  clear();  return;  }
  if(s>=p && s<=p+N){ //s points to a substring within this string!
    memmove(p, s, ls);
    resize(ls, true);
  }else{
    resize(ls, false);
    memmove(p, s, ls);
  }
}

void mlr::String::set(const char *s, uint n) { resize(n, false); memmove(p, s, n); }

void mlr::String::printf(const char *format, ...){
  resize(100, false);
  va_list valist;
  va_start(valist, format);
  int len = vsnprintf(p, 100, format, valist);
  va_end(valist);
  resize(len, true);
}

/// shorthand for the !strcmp command
bool mlr::String::operator==(const char *s) const { return p && !strcmp(p, s); }
/// shorthand for the !strcmp command
bool mlr::String::operator==(const String& s) const { return p && s.p && !strcmp(p, s.p); }
bool mlr::String::operator!=(const char *s) const { return !operator==(s); }
bool mlr::String::operator!=(const String& s) const { return !(operator==(s)); }
bool mlr::String::operator<(const String& s) const { return p && s.p && strcmp(p, s.p)<0; }

bool mlr::String::contains(const String& substring) const {
  char* p = strstr(this->p, substring.p);
  return p != NULL;
}

/// Return true iff the string starts with `substring`.
bool mlr::String::startsWith(const String& substring) const {
  return N>=substring.N && this->getFirstN(substring.N) == substring;
}

/// Return true iff the string starts with `substring`.
bool mlr::String::startsWith(const char* substring) const {
  return this->startsWith(mlr::String(substring));
}

/// Return true iff the string ends with `substring`.
bool mlr::String::endsWith(const String& substring) const {
  return this->getLastN(substring.N) == substring;
}
/// Return true iff the string ends with `substring`.
bool mlr::String::endsWith(const char* substring) const {
  return this->endsWith(mlr::String(substring));
}

/// deletes all memory and resets all stream flags
mlr::String& mlr::String::clear() { resize(0, false); return *this; }

/// call IOstream::clear();
mlr::String& mlr::String::clearStream() { std::iostream::clear(); return *this; }

/** @brief when using this String as an istream (to read other variables
  from it), this method resets the reading-pointer to the beginning
  of the string and also clears all flags of the stream */
mlr::String& mlr::String::resetIstream() { buffer.setIpos(p); clearStream(); return *this; }

/// writes the string into some ostream
void mlr::String::write(std::ostream& os) const { if(N) os <<p; }

/** @brief reads the string from some istream: first skip until one of the stopSymbols
is encountered (default: newline symbols) */
uint mlr::String::read(std::istream& is, const char* skipSymbols, const char *stopSymbols, int eatStopSymbol) {
  if(!skipSymbols) skipSymbols=readSkipSymbols;
  if(!stopSymbols) stopSymbols=readStopSymbols;
  if(eatStopSymbol==-1) eatStopSymbol=readEatStopSymbol;
  mlr::skip(is, skipSymbols);
  clear();
  char c=is.get();
  while(c!=-1 && is.good() && !mlr::contains(stopSymbols, c)) {
    append(c);
    c=is.get();
  }
  if(c==-1) is.clear();
  if(c!=-1 && !eatStopSymbol) is.putback(c);
  return N;
}

//===========================================================================
//
// string-filling routines

/** @brief fills the string with the date and time in the format
 * yy-mm-dd--hh-mm-mm */
mlr::String mlr::getNowString() {
  time_t t = time(0);
  struct tm *now = localtime(&t);

  mlr::String str;
  str.resize(19, false); //-- just enough
  sprintf(str.p, "%02d-%02d-%02d-%02d:%02d:%02d",
    now->tm_year-100,
    now->tm_mon+1,
    now->tm_mday,
    now->tm_hour,
    now->tm_min,
    now->tm_sec);
  return str;
}


//===========================================================================
//
// FileToken
//

mlr::FileToken::FileToken(const char* filename, bool change_dir){
  name=filename;
  if(change_dir) changeDir();
//  if(!exists()) HALT("file '" <<filename <<"' does not exist");
}

mlr::FileToken::FileToken(const FileToken& ft){
  name=ft.name;
  if(ft.path.N){
    NIY;
    path=ft.path;
    cwd=ft.cwd;
  }
  is = ft.is;
  os = ft.os;
}

mlr::FileToken::~FileToken(){
  unchangeDir();
}

/// change to the directory of the given filename
void mlr::FileToken::decomposeFilename() {
  path = name;
  int i=path.N;
  for(; i--;) if(path(i)=='/' || path(i)=='\\') break;
  if(i==-1) {
    path=".";
  } else {
    path.resize(i, true);
    name = name+i+1;
  }
}

void mlr::FileToken::changeDir(){
  if(path.N){
    HALT("you've changed already?");
  }else{
    decomposeFilename();
    if(path.N && path!="."){
      cwd.resize(200, false);
      if(!getcwd(cwd.p, 200)) HALT("couldn't get current dir");
      cwd.resize(strlen(cwd.p), true);
      LOG(3) <<"entering path `" <<path<<"' from '" <<cwd <<"'" <<std::endl;
      if(chdir(path)) HALT("couldn't change to directory " <<path <<" (current dir: " <<cwd <<")");
    }
  }
}

void mlr::FileToken::unchangeDir(){
  if(cwd.N){
    LOG(3) <<"leaving path `" <<path<<"' back to '" <<cwd <<"'" <<std::endl;
    if(chdir(cwd)) HALT("couldn't change back to directory " <<cwd);
  }
}

bool mlr::FileToken::exists() {
  struct stat sb;
  int r=stat(name, &sb);
  return r==0;
}

std::ofstream& mlr::FileToken::getOs(){
  CHECK(!is,"don't use a FileToken both as input and output");
  if(!os){
    os = std::make_shared<std::ofstream>();
    os->open(name);
    LOG(3) <<"opening output file `" <<name <<"'" <<std::endl;
    if(!os->good()) MLR_MSG("could not open file `" <<name <<"' for output");
  }
  return *os;
}

std::ifstream& mlr::FileToken::getIs(bool change_dir){
  if(change_dir) changeDir();
  CHECK(!os,"don't use a FileToken both as input and output");
  if(!is){
    is = std::make_shared<std::ifstream>();
    is->open(name);
    LOG(3) <<"opening input file `" <<name <<"'" <<std::endl;
    if(!is->good()) THROW("could not open file `" <<name <<"' for input");
  }
  return *is;
}


//===========================================================================
//
// random number generator
//

mlr::Rnd rnd;

uint32_t mlr::Rnd::seed(uint32_t n) {
  uint32_t s, c;
  if(n>12345) { s=n; c=n%113; } else { s=12345; c=n; }
  while(c--) s*=65539;
  s=s>>1;
  seed250(s);
  ready=true;
  return n;
}

uint32_t mlr::Rnd::seed() {
  return seed(mlr::getParameter<uint32_t>("seed", 0));
}

uint32_t mlr::Rnd::clockSeed() {
  uint32_t s;
#ifndef MLR_TIMEB
  timeval t; gettimeofday(&t, 0); s=1000000L*t.tv_sec+t.tv_usec;
#else
  _timeb t; _ftime(&t); s=1000L*t.time+t.millitm;
#endif
  LOG(3) <<"random clock seed: " <<s <<std::endl;
  return seed(s);
}

double mlr::Rnd::gauss() {
  double w, v, rsq, fac;
  do {
    v   = 2 * uni() - 1;
    w   = 2 * uni() - 1;
    rsq = v*v + w*w;
  } while(rsq >= 1 || rsq == 0);
  fac  = ::sqrt(-2 * ::log(rsq) / rsq);
  return v*fac;
}

uint32_t mlr::Rnd::poisson(double mean) {
  if(mean>100) {
    uint32_t i=(uint32_t)::floor(mean+::sqrt(mean)*gauss()+.5);
    return (i>0)?(uint32_t)i:0;
  }
  uint32_t count = 0;
  double bound, product;
  if(mean>=0) {
    bound=::exp(-mean);
    product=uni();
    while(product>=bound) {
      count++;
      product*=uni();
    }
  }
  return count;
}

void  mlr::Rnd::seed250(int32_t seed) {
  int32_t      i;
  int32_t     j, k;
  
  if(seed<=0) seed=1;
  
  for(i=0; i<250; ++i) {  // Schleife ueber Zufallsfeld
    k = seed / 127773;          // Modulozufallszahlengenerator
    seed = 16807 * (seed - k*127773) - 2836 * k;
    if(seed<0) seed += 0x7FFFFFFF;
    rfield[i] = seed;
  }
  
  // Masken ueberlagern
  k = 0x7FFFFFFF;
  j = 0x40000000;
  for(i=1; i<250; i+=8) rfield[i] = (rfield[i] & k) | j;
  
  // rpoint initialisieren
  rpoint = 249;
  
  // Anfangszahlen verwerfen
  for(i=0; i<4711; ++i) rnd250();
}


//===========================================================================
//
// Inotify
//

Inotify::Inotify(const char* filename): fd(0), wd(0){
  fd = inotify_init();
  if(fd<0) HALT("Couldn't initialize inotify");
  fil = new mlr::FileToken(filename, false);
  fil->decomposeFilename();
  wd = inotify_add_watch( fd, fil->path,
			  IN_MODIFY | IN_CREATE | IN_DELETE );
  if(wd == -1) HALT("Couldn't add watch to " <<filename);
  buffer_size = 10*(sizeof(struct inotify_event)+64); 
  buffer = new char[buffer_size];
}

Inotify::~Inotify(){
  inotify_rm_watch( fd, wd );
  close( fd );
  delete buffer;
  delete fil;
}

bool Inotify::pollForModification(bool block, bool verbose){
  if(!block){
    struct pollfd fd_poll = {fd, POLLIN, 0};
    int r = poll(&fd_poll, 1, 0);
    CHECK(r>=0,"poll failed");
    if(!r) return false;
  }

  int length = read( fd, buffer, buffer_size );
  CHECK(length>=0, "read failed");

  //-- process event list
  for(int i=0;i<length;){
    struct inotify_event *event = ( struct inotify_event * ) &buffer[ i ];
    if(verbose){
      if(event->len) {
        if(event->mask & IN_CREATE)
          cout << "The "
               <<(event->mask&IN_ISDIR?"directory ":"file ")
              <<event->name <<" was created." <<endl;
        if ( event->mask & IN_DELETE )
          cout << "The "
               <<(event->mask&IN_ISDIR?"directory ":"file ")
              <<event->name <<" was deleted." <<endl;
        if ( event->mask & IN_MODIFY )
          cout << "The "
               <<(event->mask&IN_ISDIR?"directory ":"file ")
              <<event->name <<" was modified." <<endl;
      }else{
        cout <<"event of zero length" <<endl;
      }
    }
    if(event->len
       && (event->mask & (IN_MODIFY|IN_CREATE|IN_DELETE))
       && !strcmp(event->name, fil->name.p) ) return true; //report modification on specific file
    i += sizeof(struct inotify_event) + event->len;
  }

  return false;
}



//===========================================================================
//
// Mutex
//

#define MUTEX_DUMP(x) //x

#ifndef MLR_MSVC
Mutex::Mutex() {
  pthread_mutexattr_t atts;
  int rc;
  rc = pthread_mutexattr_init(&atts);  if(rc) HALT("pthread failed with err " <<rc <<strerror(rc));
  rc = pthread_mutexattr_settype(&atts, PTHREAD_MUTEX_RECURSIVE_NP);  if(rc) HALT("pthread failed with err " <<rc <<strerror(rc));
  rc = pthread_mutex_init(&mutex, &atts);
  //mutex = PTHREAD_RECURSIVE_MUTEX_INITIALIZER_NP;
  state=0;
  recursive=0;
}

Mutex::~Mutex() {
  if(state==-1){ //forced destroy
    int rc = pthread_mutex_destroy(&mutex);
    LOG(-1) <<"pthread forced destroy returned " <<rc <<" '" <<strerror(rc) <<"'";
    return;
  }
  CHECK(!state, "Mutex destroyed without unlocking first");
  int rc = pthread_mutex_destroy(&mutex);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
}

void Mutex::lock() {
  int rc = pthread_mutex_lock(&mutex);
  if(rc){
    //don't use HALT here, because log uses mutexing as well -> can lead to recursive HALT...
    cerr <<STRING("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
    exit(1);
  }
  recursive++;
  state=syscall(SYS_gettid);
  MUTEX_DUMP(cout <<"Mutex-lock: " <<state <<" (rec: " <<recursive << ")" <<endl);
}

void Mutex::unlock() {
  MUTEX_DUMP(cout <<"Mutex-unlock: " <<state <<" (rec: " <<recursive << ")" <<endl);
  if(--recursive == 0) state=0;
  int rc = pthread_mutex_unlock(&mutex);
  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
}

#else//MLR_MSVC
Mutex::Mutex() {}
Mutex::~Mutex() {}
void Mutex::lock() {}
void Mutex::unlock() {}
#endif

//===========================================================================
//
// gnuplot calls
//

struct GnuplotServer{
  FILE *gp;
  GnuplotServer():gp(NULL){}
  ~GnuplotServer(){
    if(gp){
      cout <<"Closing Gnuplot" <<endl;
//      send("set terminal wxt nopersist close\nexit", false);
//      fclose(gp);
    }
  }

  void send(const char *cmd, bool persist){
#ifndef MLR_MSVC
    if(!gp) {
      if(!persist) gp=popen("env gnuplot -noraise -geometry 600x600-0-0 2> /dev/null", "w");
      else         gp=popen("env gnuplot -noraise -persist -geometry 600x600-0-0 2> /dev/null", "w");
      CHECK(gp, "could not open gnuplot pipe");
    }
    FILE("z.plotcmd") <<cmd; //for debugging..
    fputs(cmd, gp);
    fflush(gp);
  #else
    NIY;
  #endif
  }
};

Singleton<GnuplotServer> gnuplotServer;

void gnuplot(const char *command, bool pauseMouse, bool persist, const char *PDFfile) {
  if(!mlr::getInteractivity()){
    pauseMouse=false;
    persist=false;
  }
  
  mlr::String cmd;
  cmd <<"set style data lines\n";
  
  // run standard files
  if(!access("~/gnuplot.cfg", R_OK)) cmd <<"load '~/gnuplot.cfg'\n";
  if(!access("gnuplot.cfg", R_OK)) cmd <<"load 'gnuplot.cfg'\n";
  
  cmd <<"set title '(Gui/plot.h -> gnuplot pipe)'\n"
      <<command <<std::endl;
      
  if(PDFfile) {
    cmd <<"set terminal push\n"
        <<"set terminal pdfcairo\n"
        <<"set output '" <<PDFfile <<"'\n"
        <<command <<std::endl
        <<"\nset terminal pop\n";
  }
  
  if(pauseMouse) cmd <<"\n pause mouse" <<std::endl;
  gnuplotServer()->send(cmd.p, persist);

  if(!mlr::getInteractivity()){
    mlr::wait(.05);
  }
}


//===========================================================================
//
// Cumulative probability for the Standard Normal Distribution
//

namespace mlr {
double erf(double x) {
  double t, z, retval;
  z = fabs(x);
  t = 1.0 / (1.0 + 0.5 * z);
  retval = t * exp(-z * z - 1.26551223 + t *
                   (1.00002368 + t *
                    (0.37409196 + t *
                     (0.09678418 + t *
                      (-0.18628806 + t *
                       (0.27886807 + t *
                        (-1.13520398 + t *
                         (1.48851587 + t *
                          (-0.82215223 + t *
                           0.1708727)))))))));
  if(x < 0.0) return retval - 1.0;
  return 1.0 - retval;
}

/// the integral of N(0, 1) from -infty to x
double gaussInt(double x) {
  return .5*(1.+erf(x/MLR_SQRT2));
}

/// expectation \f$\int_x^\infty {\cal N}(x) x dx\f$ when integrated from -infty to x
double gaussIntExpectation(double x) {
  double norm=gaussInt(x) / (::sqrt(MLR_2PI));
  return - norm*mlr::approxExp(-.5*x*x);
}
}

//===========================================================================
// MISC

/**
 * @brief Return the current working dir as std::string.
 */
std::string getcwd_string() {
   char buff[PATH_MAX];
   char *succ=getcwd( buff, PATH_MAX );
   CHECK(succ,"could not call getcwd: errno=" <<errno <<' ' <<strerror(errno));
   return std::string(buff);
}

const char* NAME(const std::type_info& type){
  const char* name = type.name();
  while(*name>='0' && *name<='9') name++;
  return name;
}

//===========================================================================
//
// explicit instantiations
//

#include "util.tpp"
template void mlr::getParameter(int&, const char*);
template void mlr::getParameter(int&, const char*, const int&);
template void mlr::getParameter(uint&, const char*);
template void mlr::getParameter(uint&, const char*, const uint&);
template void mlr::getParameter(bool&, const char*, const bool&);
template void mlr::getParameter(double&, const char*);
template void mlr::getParameter(double&, const char*, const double&);
template void mlr::getParameter(mlr::String&, const char*, const mlr::String&);
template void mlr::getParameter(mlr::String&, const char*);

template int mlr::getParameter<int>(const char*);
template int mlr::getParameter<int>(const char*, const int&);
template uint mlr::getParameter(const char*);
template uint mlr::getParameter<uint>(const char*, const uint&);
template float mlr::getParameter<float>(const char*);
template double mlr::getParameter<double>(const char*);
template double mlr::getParameter<double>(const char*, const double&);
template bool mlr::getParameter<bool>(const char*);
template bool mlr::getParameter<bool>(const char*, const bool&);
template long mlr::getParameter<long>(const char*);
template mlr::String mlr::getParameter<mlr::String>(const char*);
template mlr::String mlr::getParameter<mlr::String>(const char*, const mlr::String&);

template bool mlr::checkParameter<uint>(const char*);
template bool mlr::checkParameter<bool>(const char*);



/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */

#include "thread.h"
#include "graph.h"
#include <exception>
#include <signal.h>
#include <iomanip>

#ifndef MLR_MSVC
#ifndef __CYGWIN__
#  include <sys/syscall.h>
#else
#  include "cygwin_compat.h"
#endif //__CYGWIN __
#  include <unistd.h>
#endif
#include <errno.h>


#ifndef MLR_MSVC

//===========================================================================
//
// Access RWLock
//

RWLock::RWLock() {
  int rc = pthread_rwlock_init(&rwLock, NULL);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  rwCount=0;
}

RWLock::~RWLock() {
  CHECK(!rwCount, "Destroying locked RWLock");
  int rc = pthread_rwlock_destroy(&rwLock);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
}

void RWLock::readLock() {
  int rc = pthread_rwlock_rdlock(&rwLock);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  rwCountMutex.lock();
  rwCount++;
  rwCountMutex.unlock();
}

void RWLock::writeLock() {
  int rc = pthread_rwlock_wrlock(&rwLock);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  rwCountMutex.lock();
  rwCount=-1;
  rwCountMutex.unlock();
}

void RWLock::unlock() {
  rwCountMutex.lock();
  if(rwCount>0) rwCount--; else rwCount=0;
  rwCountMutex.unlock();
  int rc = pthread_rwlock_unlock(&rwLock);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
}

bool RWLock::isLocked() {
  return rwCount!=0;
}



//===========================================================================
//
// Signaler
//

Signaler::Signaler(int initialStatus)
  : status(initialStatus), registryNode(NULL){
  int rc = pthread_cond_init(&cond, NULL);    if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
}

Signaler::~Signaler() {
  for(Signaler *c:listensTo){
    c->statusLock();
    c->listeners.removeValue(this);
    c->statusUnlock();
  }
  for(Signaler *c:listeners){
    c->statusLock();
    c->listensTo.removeValue(this);
    c->messengers.removeValue(this, false);
    c->statusUnlock();
  }
  listDelete(callbacks);
  int rc = pthread_cond_destroy(&cond);    if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
}

void Signaler::setStatus(int i, Signaler* messenger) {
  statusMutex.lock();
  status=i;
  broadcast(messenger);
  statusMutex.unlock();
}

int Signaler::incrementStatus(Signaler* messenger) {
  statusMutex.lock();
  status++;
  broadcast(messenger);
  int i=status;
  statusMutex.unlock();
  return i;
}

void Signaler::broadcast(Signaler* messenger) {
  //remember the messengers:
  if(messenger) messengers.setAppend(messenger);
  //signal to all waiters:
  int rc = pthread_cond_signal(&cond);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  //int rc = pthread_cond_broadcast(&cond);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  //setStatus to all listeners:
  for(Signaler *c:listeners) if(c!=messenger){
    Thread *th = dynamic_cast<Thread*>(c);
    if(th) th->threadStep();
    else c->setStatus(1, this);
  }
  for(auto* c:callbacks) c->call()(this, status);
}

void Signaler::listenTo(Signaler& c){
  statusMutex.lock();
  c.statusLock();
  c.listeners.append(this);
  listensTo.append(&c);
  c.statusUnlock();
  statusMutex.unlock();
}

void Signaler::stopListenTo(Signaler& c){
  statusMutex.lock();
  c.statusLock();
  c.listeners.removeValue(this);
  listensTo.removeValue(&c);
  messengers.removeValue(&c, false);
  c.statusUnlock();
  statusMutex.unlock();
}

void Signaler::stopListening(){
  statusMutex.lock();
  for(Signaler *c:listensTo){
    c->statusLock();
    c->listeners.removeValue(this);
    c->statusUnlock();
  }
  listensTo.clear();
  messengers.clear();
  statusMutex.unlock();
}

void Signaler::statusLock() {
  statusMutex.lock();
}

void Signaler::statusUnlock() {
  statusMutex.unlock();
}

int Signaler::getStatus(bool userHasLocked) const {
  Mutex *m = (Mutex*)&statusMutex; //sorry: to allow for 'const' access
  if(!userHasLocked) m->lock(); else CHECK_EQ(m->state,syscall(SYS_gettid),"user must have locked before calling this!");
  int i=status;
  if(!userHasLocked) m->unlock();
  return i;
}

void Signaler::waitForSignal(bool userHasLocked) {
  if(!userHasLocked) statusMutex.lock(); // else CHECK_EQ(mutex.state, syscall(SYS_gettid), "user must have locked before calling this!");
  int rc = pthread_cond_wait(&cond, &statusMutex.mutex);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  if(!userHasLocked) statusMutex.unlock();
}

bool Signaler::waitForSignal(double seconds, bool userHasLocked) {
  struct timespec timeout;
  clock_gettime(CLOCK_REALTIME, &timeout); //CLOCK_MONOTONIC, &timeout);
  long secs = (long)(floor(seconds));
  seconds -= secs;
  timeout.tv_sec  += secs;
  timeout.tv_nsec += (long)(floor(1e9 * seconds));
  if(timeout.tv_nsec>1000000000l) {
    timeout.tv_sec+=1;
    timeout.tv_nsec-=1000000000l;
  }

  if(!userHasLocked) statusMutex.lock(); // else CHECK_EQ(mutex.state, syscall(SYS_gettid), "user must have locked before calling this!");
  int rc = pthread_cond_timedwait(&cond, &statusMutex.mutex, &timeout);
  if(rc && rc!=ETIMEDOUT) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  if(!userHasLocked) statusMutex.unlock();
  return rc!=ETIMEDOUT;
}

bool Signaler::waitForStatusEq(int i, bool userHasLocked, double seconds) {
  if(!userHasLocked) statusMutex.lock(); else CHECK_EQ(statusMutex.state, syscall(SYS_gettid), "user must have locked before calling this!");
  while(status!=i) {
    if(seconds<0.){
      int rc = pthread_cond_wait(&cond, &statusMutex.mutex);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
    }else{
      bool succ = waitForSignal(seconds, true);
      if(!succ){
        if(!userHasLocked) statusMutex.unlock();
        return false;
      }
    }
  }
  if(!userHasLocked) statusMutex.unlock();
  return true;
}

void Signaler::waitForStatusNotEq(int i, bool userHasLocked) {
  if(!userHasLocked) statusMutex.lock(); else CHECK_EQ(statusMutex.state, syscall(SYS_gettid), "user must have locked before calling this!");
  while(status==i) {
    int rc = pthread_cond_wait(&cond, &statusMutex.mutex);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  }
  if(!userHasLocked) statusMutex.unlock();
}

void Signaler::waitForStatusGreaterThan(int i, bool userHasLocked) {
  if(!userHasLocked) statusMutex.lock(); else CHECK_EQ(statusMutex.state,syscall(SYS_gettid),"user must have locked before calling this!");
  while(status<=i) {
    int rc = pthread_cond_wait(&cond, &statusMutex.mutex);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  }
  if(!userHasLocked) statusMutex.unlock();
}

void Signaler::waitForStatusSmallerThan(int i, bool userHasLocked) {
  if(!userHasLocked) statusMutex.lock(); else CHECK_EQ(statusMutex.state,syscall(SYS_gettid),"user must have locked before calling this!");
  while(status>=i) {
    int rc = pthread_cond_wait(&cond, &statusMutex.mutex);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  }
  if(!userHasLocked) statusMutex.unlock();
}


//===========================================================================
//
// VariableBase
//

//VariableBase::VariableBase(const char *_name):name(_name), revision(0), registryNode(NULL) {
////  registryNode = registry()->newNode<VariableBase* >({"VariableData", name}, {}, this);
//}

VariableBase::~VariableBase() {
//  registry()->delNode(registryNode);
}

//bool VariableBase::hasNewRevision(){
//  return revision.getStatus() > last_revision;
//}

int VariableBase::readAccess(Thread *th) {
//  engine().acc->queryReadAccess(this, p);
  rwlock.readLock();
//  engine().acc->logReadAccess(this, p);
  return getStatus();
}

int VariableBase::writeAccess(Thread *th) {
//  engine().acc->queryWriteAccess(this, p);
  rwlock.writeLock();
  write_time = mlr::clockTime();
//  engine().acc->logWriteAccess(this, p);
  return getStatus()+1;
}

int VariableBase::deAccess(Thread *th) {
//  Module_Thread *p = m?(Module_Thread*) m->thread:NULL;
  int i;
  if(rwlock.rwCount == -1) { //log a revision after write access
    i = incrementStatus(th);
//    engine().acc->logWriteDeAccess(this,p);
  } else {
//    engine().acc->logReadDeAccess(this,p);
    i = getStatus();
  }
  rwlock.unlock();
  return i;
}

//int VariableBase::waitForNextRevision(){
//  revision.statusLock();
//  revision.waitForSignal(true);
//  int rev = revision.status;
//  revision.statusUnlock();
//  return rev;
//}

//int VariableBase::waitForRevisionGreaterThan(int rev) {
//  revision.statusLock();
//  revision.waitForStatusGreaterThan(rev, true);
//  rev = revision.status;
//  revision.statusUnlock();
//  return rev;
//}


//===========================================================================
//
// Metronome
//

Metronome::Metronome(double ticIntervalSec) {
  reset(ticIntervalSec);
}

void Metronome::reset(double ticIntervalSec) {
  clock_gettime(CLOCK_MONOTONIC, &ticTime);
  tics=0;
  ticInterval = ticIntervalSec;
}

void Metronome::waitForTic() {
  //compute target time
  long secs = (long)(floor(ticInterval));
  ticTime.tv_sec  += secs;
  ticTime.tv_nsec += (long)(floor(1000000000. * (ticInterval-(double)secs)));
  while(ticTime.tv_nsec>1000000000l) {
    ticTime.tv_sec  += 1;
    ticTime.tv_nsec -= 1000000000l;
  }
  //wait for target time
  int rc = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ticTime, NULL);
  if(rc && errno) MLR_MSG("clock_nanosleep() failed " <<rc <<" errno=" <<errno <<' ' <<strerror(errno));

  tics++;
}

double Metronome::getTimeSinceTic() {
  timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  return double(now.tv_sec-ticTime.tv_sec) + 1e-9*(now.tv_nsec-ticTime.tv_nsec);
}


//===========================================================================
//
// CycleTimer
//

void updateTimeIndicators(double& dt, double& dtMean, double& dtMax, const timespec& now, const timespec& last, uint step) {
  dt=double(now.tv_sec-last.tv_sec-1)*1000. +
     double(1000000000l+now.tv_nsec-last.tv_nsec)/1000000.;
  if(dt<0.) dt=0.;
  double rate=.01;  if(step<100) rate=1./(1+step);
  dtMean = (1.-rate)*dtMean    + rate*dt;
  if(dt>dtMax || !(step%100)) dtMax = dt;
}

CycleTimer::CycleTimer(const char* _name) {
  reset();
  name=_name;
}

CycleTimer::~CycleTimer() {
}

void CycleTimer::reset() {
  steps=0;
  busyDt=busyDtMean=busyDtMax=1.;
  cyclDt=cyclDtMean=cyclDtMax=1.;
  clock_gettime(CLOCK_MONOTONIC, &lastTime);
}

void CycleTimer::cycleStart() {
  clock_gettime(CLOCK_MONOTONIC, &now);
  updateTimeIndicators(cyclDt, cyclDtMean, cyclDtMax, now, lastTime, steps);
  lastTime=now;
}

void CycleTimer::cycleDone() {
  clock_gettime(CLOCK_MONOTONIC, &now);
  updateTimeIndicators(busyDt, busyDtMean, busyDtMax, now, lastTime, steps);
  steps++;
}

mlr::String CycleTimer::report(){
  mlr::String s;
  s.printf("busy=[%5.1f %5.1f] cycle=[%5.1f %5.1f] load=%4.1f%% steps=%i", busyDtMean, busyDtMax, cyclDtMean, cyclDtMax, 100.*busyDtMean/cyclDtMean, steps);
  return s;
//  fflush(stdout);
}


//===========================================================================
//
// Thread
//

void* Thread_staticMain(void *_self) {
  Thread *th=(Thread*)_self;
  th->main();
  return NULL;
}

#ifdef MLR_QThread
class sThread:QThread {
  Q_OBJECT
public:
  Thread *th;
  sThread(Thread *_th, const char* name):th(_th){ setObjectName(name); }
  ~sThread(){}
  void open(){ start(); }
  void close(){ wait(); }
protected:
  void run(){ th->main();  }
};
#endif

Thread::Thread(const char* _name, double beatIntervalSec)
  : Signaler(tsIsClosed),
     name(_name),
     thread(0),
     tid(0),
     step_count(0),
     metronome(beatIntervalSec),
     verbose(0) {
  registryNode = registry()->newNode<Thread*>({"Thread", name}, {}, this);
  if(name.N>14) name.resize(14, true);
}

Thread::~Thread() {
  if(thread)
      HALT("Call 'threadClose()' in the destructor of the DERIVED class! \
           That's because the 'virtual table is destroyed' before calling the destructor ~Thread (google 'call virtual function\
           in destructor') but now the destructor has to call 'threadClose' which triggers a Thread::close(), which is\
           pure virtual while you're trying to call ~Thread.")
  registry()->delNode(registryNode);
}

void Thread::threadOpen(bool wait, int priority) {
  statusLock();
  if(thread){ statusUnlock(); return; } //this is already open -- or has just beend opened (parallel call to threadOpen)
#ifndef MLR_QThread
  int rc;
  pthread_attr_t atts;
  rc = pthread_attr_init(&atts); if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  rc = pthread_create(&thread, &atts, Thread_staticMain, this);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  /*if(priority){ //doesn't work - but setpriority does work!!
    rc = pthread_attr_setschedpolicy(&atts, SCHED_RR);  if(rc) HALT("pthread failed with err " <<rc <<strerror(rc));
    sched_param  param;
    rc = pthread_attr_getschedparam(&atts, &param);  if(rc) HALT("pthread failed with err " <<rc <<" '" <<strerror(rc) <<"'");
    std::cout <<"standard priority = " <<param.sched_priority <<std::endl;
    param.sched_priority += priority;
    std::cout <<"modified priority = " <<param.sched_priority <<std::endl;
    rc = pthread_attr_setschedparam(&atts, &param);  if(rc) HALT("pthread failed with err " <<rc <<strerror(rc));
  }*/
  //prctl(PR_SET_NAME, proc->name.p);
  if(name) pthread_setname_np(thread, name);
#else
  thread = new sThread(this, "hallo");
  thread->open();
#endif
  status=tsToOpen;
  statusUnlock();
  if(wait) waitForStatusNotEq(tsToOpen);
  if(metronome.ticInterval>0.){
      if(metronome.ticInterval>1e-10){
          setStatus(tsBEATING);
      }else{
          setStatus(tsLOOPING);
      }
  }
}

void Thread::threadClose(double timeoutForce) {
  stopListening();
  setStatus(tsToClose);
  if(!thread){ setStatus(tsIsClosed); return; }
  for(;;){
    bool ended = waitForStatusEq(tsIsClosed, false, .2);
    if(ended) break;
    LOG(-1) <<"timeout to end Thread::main of '" <<name <<"'";
//    if(timeoutForce>0.){
//      ended = waitForStatusEq(tsEndOfMain, false, timeoutForce);
//      if(!ended){
//        threadCancel();
//        return;
//      }
//    }
  }
#ifndef MLR_QThread
  int rc;
  rc = pthread_join(thread, NULL);     if(rc) HALT("pthread_join failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  thread=0;
#else
  thread->close();
  delete thread;
  thread=NULL;
#endif
}

void Thread::threadCancel() {
  stopListening();
  setStatus(tsToClose);
  if(!thread) return;
#ifndef MLR_QThread
  int rc;
  rc = pthread_cancel(thread);         if(rc) HALT("pthread_cancel failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  rc = pthread_join(thread, NULL);     if(rc) HALT("pthread_join failed with err " <<rc <<" '" <<strerror(rc) <<"'");
  thread=0;
#else
  NIY;
#endif
  stepMutex.state=-1; //forced destroy in the destructor
}

void Thread::threadStep() {
  threadOpen();
  setStatus(tsToStep);
}

//void Thread::listenTo(VariableBase& var) {
//#if 0
//  var.rwlock.writeLock();  //don't want to increase revision and broadcast!
//  var.listeners.setAppend(this);
//  var.rwlock.unlock();
//  listensTo.setAppend(&var);
//#else
//  listenTo(&var.revision);
//#endif
//}

//void Thread::stopListenTo(VariableBase& var){
//#if 0
//  listensTo.removeValue(&var);
//  var.rwlock.writeLock();
//  var.listeners.removeValue(this);
//  var.rwlock.unlock();
//#else
//  stopListenTo(&var.revision);
//#endif
//}

bool Thread::isIdle() {
  return getStatus()==tsIDLE;
}

bool Thread::isClosed() {
  return !thread; //getStatus()==tsIsClosed;
}

void Thread::waitForOpened() {
  waitForStatusNotEq(tsIsClosed);
  waitForStatusNotEq(tsToOpen);
}

void Thread::waitForIdle() {
  waitForStatusEq(tsIDLE);
}

void Thread::threadLoop(bool waitForOpened) {
  threadOpen(waitForOpened);
  if(metronome.ticInterval>1e-10){
    setStatus(tsBEATING);
  }else{
    setStatus(tsLOOPING);
  }
}

void Thread::threadStop(bool wait) {
  if(thread){
    setStatus(tsIDLE);
    if(wait) waitForIdle();
  }
}

void Thread::main() {
  tid = syscall(SYS_gettid);
  if(verbose>0) cout <<"*** Entering Thread '" <<name <<"'" <<endl;
  //http://linux.die.net/man/3/setpriority
  //if(Thread::threadPriority) setRRscheduling(Thread::threadPriority);
  //if(Thread::threadPriority) setNice(Thread::threadPriority);

  {
    auto mux = stepMutex();
    try{
      open(); //virtual open routine
    } catch(const std::exception& ex) {
      setStatus(tsFAILURE);
      cerr << "*** open() of Thread'" << name << "'failed: " << ex.what() << " -- closing it again" << endl;
    } catch(...) {
      setStatus(tsFAILURE);
      cerr <<"*** open() of Thread '" <<name <<"' failed! -- closing it again";
      return;
    }
  }

  statusLock();
  if(status==tsToOpen){
    status=tsIDLE;
    broadcast();
  }
  //if not =tsOPENING anymore -> the state was set on looping or beating already
  statusUnlock();


  timer.reset();
  bool waitForTic=false;
  for(;;){
    //-- wait for a non-idle state
    statusLock();
    waitForStatusNotEq(tsIDLE, true);
    if(status==tsToClose) { statusUnlock();  break; }
    if(status==tsBEATING) waitForTic=true; else waitForTic=false;
    if(status>0) status=tsIDLE; //step command -> reset to idle
    statusUnlock();

    if(waitForTic) metronome.waitForTic();

    //-- make a step
    //engine().acc->logStepBegin(module);
    timer.cycleStart();
    stepMutex.lock();
    step(); //virtual step routine
    stepMutex.unlock();
    step_count++;
    timer.cycleDone();
    //engine().acc->logStepEnd(module);

//    //-- broadcast in case somebody was waiting for a finished step
//    status.lock();
//    status.broadcast();
//    status.unlock();
  };

  stepMutex.lock();
  close(); //virtual close routine
  stepMutex.unlock();
  if(verbose>0) cout <<"*** Exiting Thread '" <<name <<"'" <<endl;

  setStatus(tsIsClosed);
}


//===========================================================================
//
// controlling threads
//

Singleton<Signaler> moduleShutdown;

void signalhandler(int s){
  int calls = moduleShutdown()->incrementStatus();
  cerr <<"\n*** System received signal " <<s <<" -- count=" <<calls <<endl;
  if(calls==1){
    LOG(0) <<" -- waiting for main loop to break on moduleShutdown()->getStatus()" <<endl;
  }
  if(calls==2){
    LOG(0) <<" -- smoothly closing modules directly" <<endl;
    threadCloseModules(); //might lead to a hangup of the main loop, but processes should close
    LOG(0) <<" -- DONE" <<endl;
  }
  if(calls==3){
    LOG(0) <<" -- cancelling threads to force closing" <<endl;
    threadCancelModules();
    LOG(0) <<" -- DONE" <<endl;
  }
  if(calls>3){
    LOG(3) <<" ** moduleShutdown failed - hard exit!" <<endl;
    exit(1);
  }
}

void openModules(){
  NodeL threads = registry()->getNodesOfType<Thread*>();
  for(Node* th:threads){ th->get<Thread*>()->open(); }
}

void stepModules(){
  NodeL threads = registry()->getNodesOfType<Thread*>();
  for(Node* th:threads){ th->get<Thread*>()->step(); }
}

void closeModules(){
  NodeL threads = registry()->getNodesOfType<Thread*>();
  for(Node* th:threads){ th->get<Thread*>()->close(); }
}

VariableBase::Ptr getVariable(const char* name){
  return registry()->get<VariableBase::Ptr>({"VariableData", name});
}

VariableBaseL getVariables(){
  return registry()->getValuesOfType<VariableBase::Ptr>();
}

void threadOpenModules(bool waitForOpened, bool setSignalHandler){
  if(setSignalHandler) signal(SIGINT, signalhandler);
  NodeL threads = registry()->getNodesOfType<Thread*>();
  for(Node *th: threads) th->get<Thread*>()->threadOpen();
  if(waitForOpened) for(Node *th: threads) th->get<Thread*>()->waitForOpened();
  for(Node *th: threads){
    Thread *mod=th->get<Thread*>();
    if(mod->metronome.ticInterval>=0.) mod->threadLoop();
    //otherwise the module is listening (hopefully)
  }
}

void threadCloseModules(){
  NodeL threads = registry()->getNodesOfType<Thread*>();
  for(Node *th: threads) th->get<Thread*>()->threadClose();
//  threadReportCycleTimes();
}

void threadCancelModules(){
  NodeL threads = registry()->getNodesOfType<Thread*>();
  for(Node *th: threads) th->get<Thread*>()->threadCancel();
}

void threadReportCycleTimes(){
  cout <<"Cycle times for all Threads (msec):" <<endl;
  NodeL threads = registry()->getNodesOfType<Thread*>();
  for(Node *th: threads){
    Thread *thread=th->get<Thread*>();
    cout <<std::setw(30) <<thread->name <<" : " <<thread->timer.report() <<endl;
  }
}


//===========================================================================
//
// Utils
//

//void stop(const ThreadL& P) {
//  for_list(Thread,  p,  P) p->threadStop();
//}

//void wait(const ThreadL& P) {
//  for_list(Thread,  p,  P) p->waitForIdle();
//}

//void close(const ThreadL& P) {
//  for_list(Thread,  p,  P) p->threadClose();
//}

//===========================================================================
//
// TStream class, for concurrent access to ostreams
//

TStream::TStream(std::ostream &o):out(o) { }

TStream::Access TStream::operator()(const void *obj) {
  return Access(this, obj);
}

TStream::Register TStream::reg(const void *obj) {
  return Register(this, obj);
}

bool TStream::get(const void *obj, char **head) {
  return get_private(obj, head, true);
}

bool TStream::get_private(const void *obj, char **head, bool l) {
  if(l) lock.readLock();
  bool ret = map.count(obj) == 1;
  if(head) *head = ret? (char*)map[obj]: NULL;
  if(l) lock.unlock();
  return ret;
}

void TStream::reg_private(const void *obj, char *p, bool l) {
  if(l) lock.writeLock();
  unreg_private(obj, false);
  map[obj] = p;
  if(l) lock.unlock();
}

void TStream::unreg(const void *obj) {
  unreg_private(obj, true);
}

void TStream::unreg_private(const void *obj, bool l) {
  if(l) lock.writeLock();
  if(get_private(obj, NULL, false)) {
    delete map[obj];
    map.erase(obj);
  }
  if(l) lock.unlock();
}

void TStream::unreg_all() {
  lock.writeLock();
  for(auto it = map.begin(); it != map.end(); )
    unreg_private((it++)->first, false); // always increment before actual deletion
  lock.unlock();
}

TStream::Access::Access(TStream *ts, const void *o):tstream(ts), obj(o) { }
TStream::Access::Access(const Access &a):tstream(a.tstream) { }
TStream::Access::~Access() {
  tstream->mutex.lock();
  char *head;
  tstream->lock.readLock();
  if(tstream->get_private(obj, &head, false))
    tstream->out << head;
  tstream->lock.unlock();
  tstream->out << stream.str();
  tstream->mutex.unlock();
}

TStream::Register::Register(TStream *ts, const void *o):tstream(ts), obj(o) {}
TStream::Register::Register(const Register &r):tstream(r.tstream) {}
TStream::Register::~Register() {
  const char *cstr = stream.str().c_str();
  size_t cstrl = strlen(cstr);
  char *p = new char[cstrl+1];
  memcpy(p, cstr, cstrl+1);

  tstream->reg_private(obj, p, true);
}

RUN_ON_INIT_BEGIN(thread)
VariableBaseL::memMove=true;
ThreadL::memMove=true;
SignalerL::memMove=true;
RUN_ON_INIT_END(thread)

#endif //MLR_MSVC
/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#include "array.h"
#include "util.h"

#ifdef MLR_LAPACK
extern "C" {
#include "cblas.h"
#ifdef MLR_MSVC
#  include <lapack/blaswrap.h>
#endif
#include "f2c.h"
#undef small
#undef large
#ifndef ATLAS
#  include <lapack/clapack.h>
#endif
#undef double
#undef max
#undef min
#undef abs
}

#ifdef ATLAS
#include <complex>
#define lapack_complex_float std::complex<float>
#define lapack_complex_double std::complex<double>

#include <lapack/lapacke.h>
#define integer int
#undef MAX
#undef MIN
#endif
#endif //MLR_LAPACK


namespace mlr {
//===========================================================================

bool useLapack=true;
#ifdef MLR_LAPACK
const bool lapackSupported=true;
#else
const bool lapackSupported=false;
#endif
uint64_t globalMemoryTotal=0, globalMemoryBound=1ull<<30; //this is 1GB
bool globalMemoryStrict=false;
const char* arrayElemsep=" ";
const char* arrayLinesep="\n ";
const char* arrayBrackets="  ";

//===========================================================================
}

arr& NoArr = *((arr*)NULL);
arrA& NoArrA = *((arrA*)NULL);
uintA& NoUintA = *((uintA*)NULL);
byteA& NoByteA = *((byteA*)NULL);
uintAA& NoUintAA = *((uintAA*)NULL);

/* LAPACK notes
Use the documentation at
  http://www.netlib.org/lapack/double/
  http://www.netlib.org/lapack/individualroutines.html
to find the right function! Also use the man tools with Debian package lapack-doc installed.

I've put the clapack.h directly into the mlr directory - one only has to link to the Fortran lib
*/


//===========================================================================
//
/// @name matrix operations
//

arr grid(const arr& lo, const arr& hi, const uintA& steps){
  CHECK(lo.N==hi.N && lo.N==steps.N,"");
  arr X;
  uint i, j, k;
  if(lo.N==1) {
    X.resize(steps(0)+1, 1);
    for(i=0; i<X.d0; i++) X.operator()(i, 0)=lo(0)+(hi(0)-lo(0))*i/steps(0);
    return X;
  }
  if(lo.N==2) {
     X.resize(steps(0)+1, steps(1)+1, 2);
    for(i=0; i<X.d0; i++) for(j=0; j<X.d1; j++) {
        X.operator()(i, j, 0)=lo(0)+(i?(hi(0)-lo(0))*i/steps(0):0.);
        X.operator()(i, j, 1)=lo(1)+(j?(hi(1)-lo(1))*j/steps(1):0.);
      }
    X.reshape(X.d0*X.d1, 2);
    return X;
  }
  if(lo.N==3) {
    X.resize(TUP(steps(0)+1, steps(1)+1, steps(2)+1, 3));
    for(i=0; i<X.d0; i++) for(j=0; j<X.d1; j++) for(k=0; k<X.d2; k++) {
          X.elem(TUP(i, j, k, 0))=lo(0)+(hi(0)-lo(0))*i/steps(0);
          X.elem(TUP(i, j, k, 1))=lo(1)+(hi(1)-lo(1))*j/steps(1);
          X.elem(TUP(i, j, k, 2))=lo(2)+(hi(2)-lo(2))*k/steps(2);
        }
    X.reshape(X.d0*X.d1*X.d2, 3);
    return X;
  }
  HALT("not implemented yet");

}

arr repmat(const arr& A, uint m, uint n) {
  CHECK(A.nd==1 || A.nd==2, "");
  arr B;
  B.referTo(A);
  if(B.nd==1) B.reshape(B.N, 1);
  arr z;
  z.resize(B.d0*m, B.d1*n);
  for(uint i=0; i<m; i++)
    for(uint j=0; j<n; j++)
      z.setMatrixBlock(B, i*B.d0, j*B.d1);
  return z;
}
arr rand(const uintA& d) {  arr z;  z.resize(d);  rndUniform(z, false); return z;  }
arr randn(const uintA& d) {  arr z;  z.resize(d);  rndGauss(z, 1., false);  return z;  }


arr diag(double d, uint n) {
  arr z;
  z.setDiag(d, n);
  return z;
}

void addDiag(arr& A, double d){
  if(isRowShifted(A)) {
    RowShifted *Aaux = (RowShifted*) A.special;
    if(!Aaux->symmetric) HALT("this is not a symmetric matrix");
    for(uint i=0; i<A.d0; i++) A(i,0) += d;
  }else{
    for(uint i=0; i<A.d0; i++) A(i,i) += d;
  }
}

/// make symmetric \f$A=(A+A^T)/2\f$
void makeSymmetric(arr& A) {
  CHECK(A.nd==2 && A.d0==A.d1, "not symmetric");
  uint n=A.d0, i, j;
  for(i=1; i<n; i++) for(j=0; j<i; j++) A(j, i) = A(i, j) = .5 * (A(i, j) + A(j, i));
}

/// make its transpose \f$A \gets A^T\f$
void transpose(arr& A) {
  CHECK(A.nd==2 && A.d0==A.d1, "not symmetric");
  uint n=A.d0, i, j;
  double z;
  for(i=1; i<n; i++) for(j=0; j<i; j++) { z=A(j, i); A(j, i)=A(i, j); A(i, j)=z; }
}

arr oneover(const arr& A){
  arr B = A;
  for(double& b:B) b=1./b;
  return B;
}

namespace mlr {
/// use this to turn on Lapack routines [default true if MLR_LAPACK is defined]
extern bool useLapack;
}



//===========================================================================
//
/// @name SVD etc
//

/// called from svd if MLR_LAPACK is not defined
uint own_SVD(
  arr& U,
  arr& w,
  arr& V,
  const arr& A,
  bool sort);

/** @brief Singular Value Decomposition (from Numerical Recipes);
  computes \f$U, D, V\f$ with \f$A = U D V^T\f$ from \f$A\f$ such that
  \f$U\f$ and \f$V\f$ are orthogonal and \f$D\f$ diagonal (the
  returned array d is 1-dimensional) -- uses LAPACK if MLR_LAPACK is
  defined */
uint svd(arr& U, arr& d, arr& V, const arr& A, bool sort) {
  uint r;
#ifdef MLR_LAPACK
  if(mlr::useLapack) {
    r=lapack_SVD(U, d, V, A);
    V=~V;
  } else {
    r=own_SVD(U, d, V, A, sort);
  }
#else
  r=own_SVD(U, d, V, A, sort);
#endif
  
#ifdef MLR_CHECK_SVD
  bool uselapack=mlr::useLapack;
  mlr::useLapack=false;
  double err;
  arr dD, I;
  setDiagonal(dD, d);
  //cout <<U <<dD <<Vt;
  //Atmp = V * D * U;
  arr Atmp;
  Atmp = U * dD * ~V;
  //cout <<"\nA=" <<A <<"\nAtmp=" <<Atmp <<"U=" <<U <<"W=" <<dD <<"~V=" <<~V <<endl;
  std::cout <<"SVD is correct:  " <<(err=maxDiff(Atmp, A)) <<' ' <<endl;    CHECK(err<MLR_CHECK_SVD, "");
  if(A.d0<=A.d1) {
    I.setId(U.d0);
    std::cout <<"U is orthogonal: " <<(err=maxDiff(U * ~U, I)) <<' ' <<endl;  CHECK(err<MLR_CHECK_SVD, "");
    I.setId(V.d1);
    std::cout <<"V is orthogonal: " <<(err=maxDiff(~V * V, I)) <<endl;        CHECK(err<MLR_CHECK_SVD, "");
  } else {
    I.setId(U.d1);
    std::cout <<"U is orthogonal: " <<(err=maxDiff(~U * U, I)) <<' ' <<endl;  CHECK(err<MLR_CHECK_SVD, "");
    I.setId(V.d0);
    std::cout <<"V is orthogonal: " <<(err=sqrDistance(V * ~V, I)) <<endl;        CHECK(err<1e-5, "");
  }
  mlr::useLapack=uselapack;
#endif
  
  return r;
}

/// gives a decomposition \f$A = U V^T\f$
void svd(arr& U, arr& V, const arr& A) {
  arr d, D;
  ::svd(U, d, V, A);
  D.resize(d.N, d.N); D=0.;
  for(uint i=0; i<d.N; i++) D(i, i)=::sqrt(d(i));
  U=U*D;
  V=V*D;
  //CHECK(maxDiff(A, U*~V) <1e-4, "");
}

void pca(arr &Y, arr &v, arr &W, const arr &X, uint npc) {
  CHECK(X.nd == 2 && X.d0 > 0 && X.d1 > 0, "Invalid data matrix X.");
  CHECK(npc <= X.d1, "More principal components than data matrix X can offer.");

  if(npc == 0)
    npc = X.d1;

  // centering around the mean
  arr m = sum(X, 0) / (double)X.d0;
  arr D = X;
  for(uint i = 0; i < D.d0; i++)
    D[i]() -= m;
  
  arr U;
  svd(U, v, W, D, true);
  v = v % v;
  /*
  cout << "X: " << X << endl;
  cout << "D: " << D << endl;
  cout << "~D*D: " << ~D*D << endl;
  cout << "UU: " << U << endl;
  cout << "vv: " << v << endl;
  cout << "WW: " << W << endl;
  */

  W = W.cols(0, npc);
  Y = D * W;

  v *= 1./sum(v);
  v.sub(0, npc-1);
}

void check_inverse(const arr& Ainv, const arr& A) {
#ifdef MLR_CHECK_INVERSE
  arr D, _D; D.setId(A.d0);
  uint me;
  _D=A*Ainv;
  double err=maxDiff(_D, D, &me);
  cout <<"inverse is correct: " <<err <<endl;
  if(A.d0<10) {
    CHECK(err<MLR_CHECK_INVERSE , "inverting failed, error=" <<err <<" " <<_D.elem(me) <<"!=" <<D.elem(me) <<"\nA=" <<A <<"\nAinv=" <<Ainv <<"\nA*Ainv=" <<_D);
  } else {
    CHECK(err<MLR_CHECK_INVERSE , "inverting failed, error=" <<err <<" " <<_D.elem(me) <<"!=" <<D.elem(me));
  }
#endif
}

uint inverse(arr& Ainv, const arr& A) {
  uint r=inverse_SVD(Ainv, A);
  //mlr::inverse_LU(inverse, A); return A.d0;
  return r;
}

/// calls inverse(B, A) and returns B
arr inverse(const arr& A) { arr B; inverse(B, A); return B; }

/// Pseudo Inverse based on SVD; computes \f$B\f$ such that \f$ABA = A\f$
uint inverse_SVD(arr& Ainv, const arr& A) {
  CHECK_EQ(A.nd, 2, "requires a matrix");
  unsigned i, j, k, m=A.d0, n=A.d1, r;
  arr U, V, w, winv;
  Ainv.resize(n, m);
  if(m==0 || n==0) return 0;
  if(m==n && m==1) { Ainv(0, 0)=1./A(0, 0); return 0; }
  if(m==n && m==2) { inverse2d(Ainv, A); return 0; }
  
  r=svd(U, w, V, A, true);
  
  //arr W;
  //setDiagonal(W, w);
  //CHECK(fabs(maxDiff(A, U*W*~V))<1e-10, "");
  
  winv.resizeAs(w);
  for(i=0; i<r; i++) {
    if(w(i)>1e-10) winv(i) = 1./w(i); else winv(i) = 1e10;
  }
  for(; i<w.N; i++) winv(i) = 0.;
  
#if 0
  //arr W;
  setDiagonal(W, winv);
  Ainv = V * W * ~U;
#else
  double *Ainvij=&Ainv(0, 0);
  for(i=0; i<n; i++) for(j=0; j<m; j++) {
      double* vi = &V(i, 0);
      double* uj = &U(j, 0);
      double  t  = 0.;
      for(k=0; k<w.N; k++) t += vi[k] * winv.p[k] * uj[k];
      *Ainvij = t;
      Ainvij++;
    }
#endif
  
#ifdef MLR_CHECK_INVERSE
  check_inverse(Ainv, A);
#endif
  return r;
}

void mldivide(arr& X, const arr& A, const arr& b) {
#ifdef MLR_LAPACK
  lapack_mldivide(X, A, b);
#else
  NIY;
#endif
}

void inverse_LU(arr& Xinv, const arr& X) {
  NIY;
#if 0
  CHECK(X.nd==2 && X.d0==X.d1, "");
  uint n=X.d0, i, j;
  Xinv.resize(n, n);
  if(n==0) return;
  if(n==n && n==1) { Xinv(0, 0)=1./X(0, 0); return; }
  if(n==n && n==2) { inverse2d(Xinv, X); return; }
  arr LU, piv;
  lapackLU(X, LU, piv);
  arr col(n);
  for(j=0; j<n; j++) {
    col.setZero();
    col(j)=1.0;
    lubksb(LU.pp, n, idx, col.p);
    for(i=0; i<n; i++) Xinv(i, j)=col(i);
  }
  
  delete[] idx;
  delete[] d;
  
#ifdef MLR_CHECK_INVERSE
  check_inverse(Xinv, X);
#endif
#endif
}

void inverse_SymPosDef(arr& Ainv, const arr& A) {
  CHECK_EQ(A.d0,A.d1, "");
#ifdef MLR_LAPACK
  lapack_inverseSymPosDef(Ainv, A);
#else
  inverse_SVD(Ainv, A);
#endif
#ifdef MLR_CHECK_INVERSE
  check_inverse(Ainv, A);
#endif
}

arr pseudoInverse(const arr& A, const arr& Winv, double eps) {
  arr AAt;
  arr At = ~A;
  if(&Winv){
    if(Winv.nd==1) AAt = A*(Winv%At); else AAt = A*Winv*At;
  }else AAt = A*At;
  if(eps) for(uint i=0;i<AAt.d0;i++) AAt(i,i) += eps;
  arr AAt_inv = inverse_SymPosDef(AAt);
  arr Ainv = At * AAt_inv;
  if(&Winv){ if(Winv.nd==1) Ainv = Winv%Ainv; else Ainv = Winv*Ainv; }
  return Ainv;
}

/// the determinant of a 2D squared matrix
double determinant(const arr& A);

/** @brief the cofactor is the determinant of a 2D squared matrix after removing
  the ith row and the jth column */
double cofactor(const arr& A, uint i, uint j);

void gaussFromData(arr& a, arr& A, const arr& X) {
  CHECK_EQ(X.nd,2, "");
  uint N=X.d0, n=X.d1;
  arr ones(N); ones=1.;
  a = ones*X/(double)N; a.reshape(n);
  A = (~X*X)/(double)N - (a^a);
}

/* compute a rotation matrix that rotates a onto v in arbitrary dimensions */
void rotationFromAtoB(arr& R, const arr& a, const arr& v) {
  CHECK_EQ(a.N,v.N, "");
  CHECK(fabs(1.-length(a))<1e-10 && fabs(1.-length(v))<1e-10, "");
  uint n=a.N, i, j;
  if(maxDiff(a, v)<1e-10) { R.setId(n); return; }  //nothing to rotate!!
  R.resize(n, n);
  //-- compute b orthogonal to a such that (a, b) span the rotation plane
  arr b;
  b = v - a*scalarProduct(a, v);
  b /= length(b);
  //-- compute rotation coefficients within the (a, b) plane, namely, R_2D=(v_a  -v_b ;  v_b  v_a)
  double v_a, v_b;
  v_a=scalarProduct(v, a);     //component along a
  v_b=scalarProduct(v, b);     //component along b
  //-- compute columns of R:
  arr x(n), x_res;
  double x_a, x_b;
  for(i=0; i<n; i++) {
    x.setZero(); x(i)=1.;       //x=i-th unit vector
    x_a=scalarProduct(x, a);     //component along a
    x_b=scalarProduct(x, b);     //component along b
    x_res = x - x_a*a - x_b*b;  //residual (rest) of the vector
    //rotated vector = residual + rotated a-component + rotated b-component
    x = x_res + (v_a*x_a-v_b*x_b)*a + (v_b*x_a+v_a*x_b)*b;
    for(j=0; j<n; j++) R(j, i)=x(j);  //store as column of the final rotation
  }
}

inline double MLR_SIGN_SVD(double a, double b) { return b>0 ? ::fabs(a) : -::fabs(a); }
#define MLR_max_SVD(a, b) ( (a)>(b) ? (a) : (b) )
#define MLR_SVD_MINVALUE .0 //1e-10

uint own_SVD(
  arr& U,
  arr& w,
  arr& V,
  const arr& A,
  bool sort) {
  //mlr::Array<double*> Apointers, Upointers, Vpointers;
  unsigned m = A.d0; /* rows */
  unsigned n = A.d1; /* cols */
  U.resize(m, n);
  V.resize(n, n);
  w.resize(n);
  mlr::Array<double*> Ap, Up, Vp;
  double **a = A.getCarray(Ap); //Pointers(Apointers); /* input matrix */
  double **u = U.getCarray(Up); //Pointers(Upointers); /* left vectors */
  double **v = V.getCarray(Vp); //Pointers(Vpointers); /* right vectors */
  
  int flag;
  unsigned i, its, j, jj, k, l, nm(0), r;
  double anorm, c, f, g, h, s, scale, x, y, z, t;
  
  arr rv1(n);
  
  /* copy A to U */
  for(i=0; i<m; i++) for(j=0; j<n; j++) u[i][j] = a[i][j];
  
  /* householder reduction to pickBiagonal form */
  g = scale = anorm = 0.0;
  
  for(i=0; i<n; i++) {
    l = i + 1;
    rv1(i) = scale * g;
    g = s = scale = 0.0;
    
    if(i<m) {
      for(k=i; k<m; k++) scale += fabs(u[k][i]);
      
      if(scale!=0.0) {
        for(k=i; k<m; k++) {
          u[k][i] /= scale;
          s += u[k][i] * u[k][i];
        }
        
        f = u[i][i];
        g = -MLR_SIGN_SVD(sqrt(s), f);
        h = f * g - s;
        u[i][i] = f - g;
        
        for(j=l; j<n; j++) {
          s = 0.0;
          for(k=i; k<m; k++) s += u[k][i] * u[k][j];
          
          f = s / h;
          for(k=i; k<m; k++) u[k][j] += f * u[k][i];
        }
        
        for(k=i; k<m; k++) u[k][i] *= scale;
      }
    }
    
    w(i) = scale * g;
    g = s = scale = 0.0;
    
    if(i<m && i!=n-1) {
      for(k=l; k<n; k++)scale += fabs(u[i][k]);
      
      if(scale!=0.0) {
        for(k=l; k<n; k++) {
          u[i][k] /= scale;
          s += u[i][k] * u[i][k];
        }
        
        f = u[i][l];
        g = -MLR_SIGN_SVD(sqrt(s), f);
        h = f * g - s;
        u[i][l] = f - g;
        
        for(k=l; k<n; k++) rv1(k) = u[i][k] / h;
        
        for(j=l; j<m; j++) {
          s = 0.0;
          for(k=l; k<n; k++) s += u[j][k] * u[i][k];
          
          for(k=l; k<n; k++) u[j][k] += s * rv1(k);
        }
        
        for(k=l; k<n; k++) u[i][k] *= scale;
      }
    }
    
    anorm = MLR_max_SVD(anorm, fabs(w(i)) + fabs(rv1(i)));
  }
  
  /* accumulation of right-hand transformations */
  for(l=i=n; i--; l--) {
    if(l<n) {
      if(g!=0.0) {
        /* double division avoids possible underflow */
        for(j=l; j<n; j++) v[j][i] = (u[i][j] / u[i][l]) / g;
        
        for(j=l; j<n; j++) {
          s = 0.0;
          for(k=l; k<n; k++) s += u[i][k] * v[k][j];
          
          for(k=l; k<n; k++) v[k][j] += s * v[k][i];
        }
      }
      
      for(j=l; j<n; j++) v[i][j] = v[j][i] = 0.0;
    }
    
    v[i][i] = 1.0;
    g = rv1(i);
  }
  
  /* accumulation of left-hand transformations */
  for(l=i=(m<n?m:n); i--; l--) {
    g = w(i);
    
    for(j=l; j<n; j++) u[i][j] = 0.0;
    
    if(g!=0.0) {
      g = 1.0 / g;
      
      for(j=l; j<n; j++) {
        s = 0.0;
        for(k=l; k<m; k++) s += u[k][i] * u[k][j];
        
        /* double division avoids possible underflow */
        f = (s / u[i][i]) * g;
        
        for(k=i; k<m; k++) u[k][j] += f * u[k][i];
      }
      
      for(j=i; j<m; j++) u[j][i] *= g;
    } else {
      for(j=i; j<m; j++) u[j][i] = 0.0;
    }
    
    u[i][i]++;
  }
  
  /* diagonalization of the pickBiagonal form */
  for(k=n; k--;) {
    for(its=1; its<=30; its++) {
      flag = 1;
      
      /* test for splitting */
      for(l = k + 1; l--;) {
        /* rv1 [0] is always zero, so there is no exit */
        nm = l - 1;
        
        if(fabs(rv1(l)) + anorm == anorm) {
          flag = 0;
          break;
        }
        
        //if(!l) break; //(mt 07-01-16)
        if(fabs(w(nm)) + anorm == anorm) break;
      }
      
      if(flag) {
        /* cancellation of rv1 [l] if l greater than 0 */
        c = 0.0;
        s = 1.0;
        
        for(i=l; i<=k; i++) {
          f = s * rv1(i);
          rv1(i) *= c;
          
          if(fabs(f) + anorm == anorm) break;
          
          g = w(i);
          h = hypot(f, g);
          w(i) = h;
          h = 1.0 / h;
          c = g * h;
          s = -f * h;
          
          for(j=0; j<m; j++) {
            y = u[j][nm];
            z = u[j][i];
            u[j][nm] = y * c + z * s;
            u[j][i] = z * c - y * s;
          }
        }
      }
      
      /* test for convergence */
      z = w(k);
      
      if(l==k) {
        if(z<0.0) {
          w(k) = -z;
          for(j=0; j<n; j++) v[j][k] = -v[j][k];
        }
        break;
      }
      
      if(its==50) HALT("svd failed");
      //if(its==30) throw k;
      
      /* shift from bottom 2 by 2 minor */
      x = w(l);
      nm = k - 1;
      y = w(nm);
      g = rv1(nm);
      h = rv1(k);
      f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0 * h * y);
      g = hypot(f, 1.0);
      f = ((x - z) * (x + z) + h * ((y / (f + MLR_SIGN_SVD(g, f))) - h)) / x;
      
      /* next qr transformation */
      c = s = 1.0;
      
      for(j=l; j<k; j++) {
        i = j + 1;
        g = rv1(i);
        y = w(i);
        h = s * g;
        g *= c;
        z = hypot(f, h);
        rv1(j) = z;
        c = f / z;
        s = h / z;
        f = x * c + g * s;
        g = g * c - x * s;
        h = y * s;
        y *= c;
        
        for(jj=0; jj<n; jj++) {
          x = v[jj][j];
          z = v[jj][i];
          v[jj][j] = x * c + z * s;
          v[jj][i] = z * c - x * s;
        }
        
        z = hypot(f, h);
        w(j) = z;
        
        /* rotation can be arbitrary if z is zero */
        if(z!=0.0) {
          z = 1.0 / z;
          c = f * z;
          s = h * z;
        }
        
        f = c * g + s * y;
        x = c * y - s * g;
        
        for(jj=0; jj<m; jj++) {
          y = u[jj][j];
          z = u[jj][i];
          u[jj][j] = y * c + z * s;
          u[jj][i] = z * c - y * s;
        }
      }
      
      rv1(l) = 0.0;
      rv1(k) = f;
      w(k) = x;
    }
  }
  
  //sorting:
  if(sort) {
    unsigned i, j, k;
    double   p;
    
    for(i=0; i<n-1; i++) {
      p = w(k=i);
      
      for(j=i+1; j<n; j++) if(w(j)>=p) p = w(k=j);
      
      if(k!=i) {
        w(k) = w(i);
        w(i) = p;
        
        for(j=0; j<n; j++) {
          p       = v[j][i];
          v[j][i] = v[j][k];
          v[j][k] = p;
        }
        
        for(j=0; j<m; j++) {
          p       = u[j][i];
          u[j][i] = u[j][k];
          u[j][k] = p;
        }
      }
    }
  }
  
  //rank analysis
  
  for(r=0; r<n && w(r)>MLR_SVD_MINVALUE; r++) {};
  
  t = r < n ? fabs(w(n-1)) : 0.0;
  r = 0;
  s = 0.0;
  while(r<n && w(r)>t && w(r)+s>s) s += w(r++);
  
  return r;
}

double determinantSubroutine(double **A, uint n) {
  if(n==1) return A[0][0];
  if(n==2) return A[0][0]*A[1][1]-A[0][1]*A[1][0];
  uint i, j;
  double d=0;
  double **B=new double*[n-1];
  for(i=0; i<n; i++) {
    for(j=0; j<n; j++) {
      if(j<i) B[j]=&A[j][1];
      if(j>i) B[j-1]=&A[j][1];
    }
    d+=((i&1)?-1.:1.) * A[i][0] * determinantSubroutine(B, n-1);
  }
  delete[] B; B=NULL;
  return d;
}

double determinant(const arr& A) {
  CHECK(A.nd==2 && A.d0==A.d1, "determinants require a squared 2D matrix");
  mlr::Array<double*> tmp;
  return determinantSubroutine(A.getCarray(tmp), A.d0);
}

double cofactor(const arr& A, uint i, uint j) {
  CHECK(A.nd==2 && A.d0==A.d1, "determinants require a squared 2D matrix");
  arr B=A;
  B.delRows(i);
  B.delColumns(j, 1);
  return ((i&1)^(j&1)?-1.:1) * determinant(B);
}

/** Given a distribution p over a discrete domain {0, .., p.N-1}
    Stochastic Universal Sampling draws n samples from this
    distribution, stored as integers in s */
uintA sampleMultinomial_SUS(const arr& p, uint n) {
  //following T. Baeck "EA in Theo. and Prac." p120
  uintA s(n);
  double sum=0, ptr=rnd.uni();
  uint i, j=0;
  for(i=0; i<p.N; i++) {
    sum+=p(i)*n;
    while(sum>ptr) { s(j)=i; j++; ptr+=1.; }
  }
  //now, 'sum' should = 'n' and 'ptr' has been 'n'-times increased -> 'j=n'
  CHECK_EQ(j,n, "error in rnd::sampleMultinomial_SUS(p, n) -> p not normalized?");
  return s;
}

uint sampleMultinomial(const arr& p) {
  double sum=0, ptr=rnd.uni();
  uint i;
  for(i=0; i<p.N; i++) {
    sum+=p(i);
    if(sum>ptr) return i;
  }
  HALT("error in rnd::sampleMultinomial(p) -> p not normalized? " <<p);
  return 0;
}

/// calls gnuplot to display the (n, 2) or (n, 3) array (n=number of points of line or surface)
void gnuplot(const arr& X, bool pauseMouse, bool persist, const char* PDFfile) {
  mlr::arrayBrackets="  ";
  if(X.nd==2 && X.d1!=2) {  //assume array -> splot
    FILE("z.pltX") <<X;
    gnuplot("splot 'z.pltX' matrix with pm3d, 'z.pltX' matrix with lines", pauseMouse, persist, PDFfile);
    return;
  }
  if(X.nd==2 && X.d1==2) {  //assume curve -> plot
    FILE("z.pltX") <<X;
    gnuplot("plot 'z.pltX' us 1:2", pauseMouse, persist, PDFfile);
    return;
  }
  if(X.nd==1) {  //assume curve -> plot
    arr Y;
    Y.referTo(X);
    Y.reshape(Y.N, 1);
    FILE("z.pltX") <<Y;
    gnuplot("plot 'z.pltX' us 1", pauseMouse, persist, PDFfile);
    return;
  }
}

arr bootstrap(const arr& x){
  arr y(x.N);
  for(uint i=0;i<y.N;i++) y(i) = x(rnd(y.N));
  return y;
}

void write(const arrL& X, const char *filename, const char *ELEMSEP, const char *LINESEP, const char *BRACKETS, bool dimTag, bool binary) {
  std::ofstream fil;
  mlr::open(fil, filename);
  catCol(X).write(fil, ELEMSEP, LINESEP, BRACKETS, dimTag, binary);
  fil.close();
}

//===========================================================================
//
/// @name simple image formats
//

/** save data as ppm or pgm. Images are (height, width, [0, 2, 3, 4])-dim
  byte arrays, where the 3rd dimension determines whether it's a grey
  (0), grey-alpha (2), RGB (3), or RGBA (4) image */
void write_ppm(const byteA &img, const char *file_name, bool swap_rows) {
  if(!img.N) MLR_MSG("empty image");
  CHECK(img.nd==2 || (img.nd==3 && img.d2==3), "only rgb or gray images to ppm");
  ofstream os;
  os.open(file_name, std::ios::out | std::ios::binary);
  if(!os.good()) HALT("could not open file `" <<file_name <<"' for output");
  switch(img.d2) {
    case 0:  os <<"P5 " <<img.d1 <<' ' <<img.d0 <<" 255\n";  break; //PGM
    case 3:  os <<"P6 " <<img.d1 <<' ' <<img.d0 <<" 255\n";  break; //PPM
  }
  if(!swap_rows) {
    os.write((char*)img.p, img.N);
  } else {
    if(img.d2)
      for(uint i=img.d0; i--;) os.write((char*)&img(i, 0, 0), img.d1*img.d2);
    else
      for(uint i=img.d0; i--;) os.write((char*)&img(i, 0), img.d1);
  }
}

/** read data from an ppm or pgm file */
void read_ppm(byteA &img, const char *file_name, bool swap_rows) {
  uint mode, width, height, max;
  ifstream is;
  is.open(file_name, std::ios::in | std::ios::binary);
  if(!is.good()) HALT("could not open file `" <<file_name <<"' for input");
  if(is.get()!='P') HALT("NO PPM FILE:" <<file_name);
  is >>mode;
  if(mlr::peerNextChar(is)=='#') mlr::skipRestOfLine(is);
  is >>width >>height >>max;
  is.get(); //MUST be a white character if everything went ok
  switch(mode) {
    case 5:  img.resize(height, width);    break; //PGM
    case 6:  img.resize(height, width, 3);  break; //PPM
  }
  if(!swap_rows) {
    is.read((char*)img.p, img.N);
  } else {
    for(uint i=img.d0; i--;) is.read((char*)&img(i, 0, 0), img.d1*img.d2);
  }
}

/// add an alpha channel to an image array
void add_alpha_channel(byteA &img, byte alpha) {
  uint w=img.d1, h=img.d0;
  img.reshape(h*w, 3);
  img.insColumns(3, 1);
  for(uint i=0; i<img.d0; i++) img(i, 3)=alpha;
  img.reshape(h, w, 4);
}

void flip_image(byteA &img) {
  if(!img.N) return;
  uint h=img.d0, n=img.N/img.d0;
  byteA line(n);
  byte *a, *b, *c;
  for(uint i=0; i<h/2; i++) {
    a=img.p+i*n;
    b=img.p+(h-1-i)*n;
    c=line.p;
    memmove(c, a, n);
    memmove(a, b, n);
    memmove(b, c, n);
  }
}

void flip_image(floatA &img) {
  if(!img.N) return;
  uint h=img.d0, n=img.N/img.d0;
  floatA line(n);
  float *a, *b, *c;
  uint s=sizeof(float);
  for(uint i=0; i<h/2; i++) {
    a=img.p+i*n;
    b=img.p+(h-1-i)*n;
    c=line.p;
    memmove(c, a, n*s);
    memmove(a, b, n*s);
    memmove(b, c, n*s);
  }
}

/// make grey scale image
void make_grey(byteA &img) {
  CHECK(img.nd==3 && (img.d2==3 || img.d1==4), "makeGray requires color image as input");
  byteA tmp;
  tmp.resize(img.d0, img.d1);
  for(uint i=0; i<img.d0; i++) for(uint j=0; j<img.d1; j++) {
      tmp(i, j) = ((uint)img(i, j, 0) + img(i, j, 1) + img(i, j, 2))/3;
    }
  img=tmp;
}

/// make a grey image and RGA image
void make_RGB(byteA &img) {
  CHECK_EQ(img.nd,2, "make_RGB requires grey image as input");
  byteA tmp;
  tmp.resize(img.d0, img.d1, 3);
  for(uint i=0; i<img.d0; i++) for(uint j=0; j<img.d1; j++) {
      tmp(i, j, 0) = img(i, j);
      tmp(i, j, 1) = img(i, j);
      tmp(i, j, 2) = img(i, j);
    }
  img=tmp;
}

/// make a grey image and RGA image
void make_RGB2BGRA(byteA &img) {
  CHECK(img.nd==3 && img.d2==3, "make_RGB2RGBA requires color image as input");
  byteA tmp;
  tmp.resize(img.d0, img.d1, 4);
  for(uint i=0; i<img.d0; i++) for(uint j=0; j<img.d1; j++) {
      tmp(i, j, 0) = img(i, j, 2);
      tmp(i, j, 1) = img(i, j, 1);
      tmp(i, j, 2) = img(i, j, 0);
      tmp(i, j, 3) = 255;
    }
  img=tmp;
}

/// make a grey image and RGA image
void swap_RGB_BGR(byteA &img) {
  CHECK(img.nd==3 && img.d2==3, "make_RGB2RGBA requires color image as input");
  byte *b=img.p, *bstop=img.p+img.N;
  byte z;
  for(;b<bstop; b+=3) {
    z=b[0]; b[0]=b[2]; b[2]=z;
  }
}


uintA getIndexTuple(uint i, const uintA &d) {
  CHECK(i<product(d), "out of range");
  uintA I(d.N);
  I.setZero();
  for(uint j=d.N; j--;) {
    I.p[j] = i%d.p[j];
    i -= I.p[j];
    i /= d.p[j];
  }
  return I;
}

void lognormScale(arr& P, double& logP, bool force) {
#ifdef MLR_NoLognormScale
  return;
#endif
  double Z=0.;
  for(uint i=0; i<P.N; i++) Z += fabs(P.elem(i));
  if(!force && Z>1e-3 && Z<1e3) return;
  if(fabs(Z-1.)<1e-10) return;
  if(Z>1e-100) {
    logP+=::log(Z);
    P/=Z;
  } else {
    logP+=::log(Z);
    P=1.;
    MLR_MSG("ill-conditioned table factor for norm scaling");
  }
}

void sparseProduct(arr& y, arr& A, const arr& x) {
  if(!A.special && !x.special) {
    innerProduct(y, A, x);
    return;
  }
  if(isSparseMatrix(A) && !isSparseVector(x)) {
    uint i, j, *k, *kstop;
    y.resize(A.d0); y.setZero();
    double *Ap=A.p;
    uintA& A_elems = dynamic_cast<mlr::SparseMatrix*>(A.special)->elems;
    for(k=A_elems.p, kstop=A_elems.p+A_elems.N; k!=kstop; Ap++) {
      i=*k; k++;
      j=*k; k++;
      y.p[i] += (*Ap) * x.p[j];
    }
    return;
  }
  if(isSparseMatrix(A) && isSparseVector(x)) {
    mlr::SparseVector *sx = dynamic_cast<mlr::SparseVector*>(x.special);
    CHECK(x.nd==1 && A.nd==2 && sx->N==A.d1, "not a proper matrix-vector multiplication");
    uint i, j, n, *k, *kstop, *l, *lstop;
    y.clear(); y.nd=1; y.d0=A.d0;
    y.special = new mlr::SparseMatrix(y, A.d0); //aux=y_sparse=new uintA [2];
    uintA& y_elems= dynamic_cast<mlr::SparseMatrix*>(y.special)->elems;
    uintA& y_col= dynamic_cast<mlr::SparseMatrix*>(y.special)->cols(0);
    y_col.resize(y.d0); y_col=(uint)-1;
    double *xp=x.p;
    uintA& x_elems = sx->elems;
    uint *slot;
    for(k=x_elems.p, kstop=x_elems.p+x_elems.N; k!=kstop; xp++) {
      j=*k; k++;
      uintA& A_col = dynamic_cast<mlr::SparseMatrix*>(A.special)->cols(j);
      for(l=A_col.p, lstop=A_col.p+A_col.N; l!=lstop;) {
        i =*l; l++;
        n =*l; l++;
        slot=&y_col(i);
        if(*slot==(uint)-1) {
          *slot=y.N;
          y.resizeMEM(y.N+1, true); y(y.N-1)=0.;
          y_elems.append(i);
          CHECK_EQ(y_elems.N,y.N, "");
        }
        i=*slot;
        y(i) += A.elem(n) * (*xp);
      }
    }
    return;
  }
  if(!isSparseMatrix(A) && isSparseVector(x)) {
    uint i, j, *k, *kstop, d1=A.d1;
    y.resize(A.d0); y.setZero();
    double *xp=x.p;
    uintA& elems = dynamic_cast<mlr::SparseMatrix*>(x.special)->elems;
    for(k=elems.p, kstop=elems.p+elems.N; k!=kstop; xp++) {
      j=*k; k++;
      for(i=0; i<A.d0; i++) {
        y.p[i] += A.p[i*d1+j] * (*xp);
      }
    }
    return;
  }
}

void scanArrFile(const char* name) {
  ifstream is(name, std::ios::binary);
  CHECK(is.good(), "couldn't open file " <<name);
  arr x;
  String tag;
  for(;;) {
    tag.read(is, " \n\r\t", " \n\r\t");
    if(!is.good() || tag.N==0) return;
    x.readTagged(is, NULL);
    x.writeTagged(cout, tag);  cout <<endl;
    if(!is.good()) return;
  }
}

#ifndef CHECK_EPS
#  define CHECK_EPS 1e-8
#endif

/// numeric (finite difference) computation of the gradient
arr finiteDifferenceGradient(const ScalarFunction& f, const arr& x, arr& Janalytic){
  arr dx, J;
  double y, dy;
  y=f(Janalytic, NoArr, x);

  J.resize(x.N);
  double eps=CHECK_EPS;
  uint i;
  for(i=0; i<x.N; i++) {
    dx=x;
    dx.elem(i) += eps;
    dy = f(NoArr, NoArr, dx);
    dy = (dy-y)/eps;
    J(i)=dy;
  }
  return J;
}

/// numeric (finite difference) check of the gradient of f at x
bool checkGradient(const ScalarFunction& f,
                   const arr& x, double tolerance, bool verbose) {
#if 1
  arr J;
  arr JJ = finiteDifferenceGradient(f, x, J);
#else
  arr J, dx, JJ;
  double y, dy;
  y=f(J, NoArr, x);

  JJ.resize(x.N);
  double eps=CHECK_EPS;
  for(uint i=0; i<x.N; i++) {
    dx=x;
    dx.elem(i) += eps;
    dy = f(NoArr, NoArr, dx);
    dy = (dy-y)/eps;
    JJ(i)=dy;
  }
  JJ.reshapeAs(J);
#endif
  uint i;
  double md=maxDiff(J, JJ, &i);
//   J >>FILE("z.J");
//   JJ >>FILE("z.JJ");
  if(md>tolerance) {
    MLR_MSG("checkGradient -- FAILURE -- max diff=" <<md <<" |"<<J.elem(i)<<'-'<<JJ.elem(i)<<"| (stored in files z.J_*)");
    J >>FILE("z.J_analytical");
    JJ >>FILE("z.J_empirical");
    //cout <<"\nmeasured grad=" <<JJ <<"\ncomputed grad=" <<J <<endl;
    //HALT("");
    return false;
  } else {
    cout <<"checkGradient -- SUCCESS (max diff error=" <<md <<")" <<endl;
  }
  return true;
}

bool checkHessian(const ScalarFunction& f, const arr& x, double tolerance, bool verbose) {
  arr g, H, dx, dy, Jg;
  f(g, H, x);
  if(isRowShifted(H)) H = unpack(H);

  Jg.resize(g.N, x.N);
  double eps=CHECK_EPS;
  uint i, k;
  for(i=0; i<x.N; i++) {
    dx=x;
    dx.elem(i) += eps;
    f(dy, NoArr, dx);
    dy = (dy-g)/eps;
    for(k=0; k<g.N; k++) Jg(k, i)=dy.elem(k);
  }
  Jg.reshapeAs(H);
  double md=maxDiff(H, Jg, &i);
  //   J >>FILE("z.J");
  //   JJ >>FILE("z.JJ");
  if(md>tolerance) {
    MLR_MSG("checkHessian -- FAILURE -- max diff=" <<md <<" |"<<H.elem(i)<<'-'<<Jg.elem(i)<<"| (stored in files z.J_*)");
    H >>FILE("z.J_analytical");
    Jg >>FILE("z.J_empirical");
    //cout <<"\nmeasured grad=" <<JJ <<"\ncomputed grad=" <<J <<endl;
    //HALT("");
    return false;
  } else {
    cout <<"checkHessian -- SUCCESS (max diff error=" <<md <<")" <<endl;
  }
  return true;
}

bool checkJacobian(const VectorFunction& f,
                   const arr& x, double tolerance, bool verbose) {
  arr x_copy=x;
  arr y, J, dx, dy, JJ;
  f(y, J, x_copy);
  if(isRowShifted(J)) J = unpack(J);

  JJ.resize(y.N, x.N);
  double eps=CHECK_EPS;
  uint i, k;
  for(i=0; i<x.N; i++) {
    dx=x_copy;
    dx.elem(i) += eps;
    f(dy, NoArr, dx);
    dy = (dy-y)/eps;
    for(k=0; k<y.N; k++) JJ(k, i)=dy.elem(k);
  }
  JJ.reshapeAs(J);
  double md=maxDiff(J, JJ, &i);
  if(md>tolerance) {
    MLR_MSG("checkJacobian -- FAILURE -- max diff=" <<md <<" |"<<J.elem(i)<<'-'<<JJ.elem(i)<<"| (stored in files z.J_*)");
    J >>FILE("z.J_analytical");
    JJ >>FILE("z.J_empirical");
    if(verbose){
      cout <<"J_analytical = " <<J
         <<"\nJ_empirical  = " <<JJ <<endl;
    }
    return false;
  } else {
    cout <<"checkJacobian -- SUCCESS (max diff error=" <<md <<")" <<endl;
  }
  return true;
}

#define EXP ::exp //mlr::approxExp

double NNinv(const arr& a, const arr& b, const arr& Cinv){
  double d=sqrDistance(Cinv, a, b);
  double norm = ::sqrt(lapack_determinantSymPosDef((1./MLR_2PI)*Cinv));
  return norm*EXP(-.5*d);
}
double logNNinv(const arr& a, const arr& b, const arr& Cinv){
  NIY;
  return 1;
  /*
  arr d=a-b;
  double norm = ::sqrt(fabs(mlr::determinant_LU((1./MLR_2PI)*Cinv)));
  return ::log(norm) + (-.5*scalarProduct(Cinv, d, d));
  */
}
double logNNprec(const arr& a, const arr& b, double prec){
  uint n=a.N;
  arr d=a-b;
  double norm = pow(prec/MLR_2PI, .5*n);
  return ::log(norm) + (-.5*prec*scalarProduct(d, d));
}
double logNN(const arr& a, const arr& b, const arr& C){
  arr Cinv;
  inverse_SymPosDef(Cinv, C);
  return logNNinv(a, b, Cinv);
}
double NN(const arr& a, const arr& b, const arr& C){
  arr Cinv;
  inverse_SymPosDef(Cinv, C);
  return NNinv(a, b, Cinv);
}
/// non-normalized!! Gaussian function (f(0)=1)
double NNNNinv(const arr& a, const arr& b, const arr& Cinv){
  double d=sqrDistance(Cinv, a, b);
  return EXP(-.5*d);
}
double NNNN(const arr& a, const arr& b, const arr& C){
  arr Cinv;
  inverse_SymPosDef(Cinv, C);
  return NNNNinv(a, b, Cinv);
}
double NNzeroinv(const arr& x, const arr& Cinv){
  double norm = ::sqrt(lapack_determinantSymPosDef((1./MLR_2PI)*Cinv));
  return norm*EXP(-.5*scalarProduct(Cinv, x, x));
}
/// gradient of a Gaussian
double dNNinv(const arr& x, const arr& a, const arr& Ainv, arr& grad){
  double y=NNinv(x, a, Ainv);
  grad = y * Ainv * (a-x);
  return y;
}
/// gradient of a non-normalized Gaussian
double dNNNNinv(const arr& x, const arr& a, const arr& Ainv, arr& grad){
  double y=NNNNinv(x, a, Ainv);
  grad = y * Ainv * (a-x);
  return y;
}
double NNsdv(const arr& a, const arr& b, double sdv){
  double norm = 1./(::sqrt(MLR_2PI)*sdv);
  return norm*EXP(-.5*sqrDistance(a, b)/(sdv*sdv));
}
double NNzerosdv(const arr& x, double sdv){
  double norm = 1./(::sqrt(MLR_2PI)*sdv);
  return norm*EXP(-.5*sumOfSqr(x)/(sdv*sdv));
}

mlr::String singleString(const StringA& strs){
  mlr::String s;
  for(const mlr::String& str:strs){
    if(s.N) s<<"_";
    s<<str;
  }
  return s;
}


//===========================================================================
//
// LAPACK
//


/*  ---------------------------------------------------------------------
    Copyright 2014 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */

// file:///usr/share/doc/liblapack-doc/lug/index.html

#ifdef MLR_LAPACK
#ifdef NO_BLAS
void blas_MM(arr& X, const arr& A, const arr& B) {       mlr::useLapack=false; innerProduct(X, A, B); mlr::useLapack=true; };
void blas_MsymMsym(arr& X, const arr& A, const arr& B) { mlr::useLapack=false; innerProduct(X, A, B); mlr::useLapack=true; };
void blas_Mv(arr& y, const arr& A, const arr& x) {       mlr::useLapack=false; innerProduct(y, A, x); mlr::useLapack=true; };
#else
void blas_MM(arr& X, const arr& A, const arr& B) {
  CHECK_EQ(A.d1,B.d0, "matrix multiplication: wrong dimensions");
  X.resize(A.d0, B.d1);
  cblas_dgemm(CblasRowMajor,
              CblasNoTrans, CblasNoTrans,
              A.d0, B.d1, A.d1,
              1., A.p, A.d1,
              B.p, B.d1,
              0., X.p, X.d1);
#if 0//test
  mlr::useLapack=false;
  std::cout  <<"blas_MM error = " <<maxDiff(A*B, X, 0) <<std::endl;
  mlr::useLapack=true;
#endif
}

void blas_A_At(arr& X, const arr& A) {
  uint n=A.d0;
  CHECK(n,"blas doesn't like n=0 !");
  X.resize(n,n);
  cblas_dsyrk(CblasRowMajor, CblasUpper, CblasNoTrans,
              X.d0, A.d1,
              1.f, A.p, A.d1,
              0., X.p, X.d1);
  for(uint i=0; i<n; i++) for(uint j=0; j<i; j++) X.p[i*n+j] = X.p[j*n+i]; //fill in the lower triangle
#if 0//test
  mlr::useLapack=false;
  std::cout  <<"blas_MM error = " <<maxDiff(A*~A, X, 0) <<std::endl;
  mlr::useLapack=true;
#endif
}

void blas_At_A(arr& X, const arr& A) {
  uint n=A.d1;
  CHECK(n,"blas doesn't like n=0 !");
  X.resize(n,n);
  cblas_dsyrk(CblasRowMajor, CblasUpper, CblasTrans,
              X.d0, A.d0,
              1.f, A.p, A.d1,
              0., X.p, X.d1);
  for(uint i=0; i<n; i++) for(uint j=0; j<i; j++) X.p[i*n+j] = X.p[j*n+i]; //fill in the lower triangle
#if 0//test
  mlr::useLapack=false;
  std::cout  <<"blas_MM error = " <<maxDiff(~A*A, X, 0) <<std::endl;
  mlr::useLapack=true;
#endif
}

void blas_Mv(arr& y, const arr& A, const arr& x) {
  CHECK_EQ(A.d1,x.N, "matrix multiplication: wrong dimensions");
  y.resize(A.d0);
  if(!x.N && !A.d1) { y.setZero(); return; }
  cblas_dgemv(CblasRowMajor,
              CblasNoTrans,
              A.d0, A.d1,
              1., A.p, A.d1,
              x.p, 1,
              0., y.p, 1);
#if 0 //test
  mlr::useLapack=false;
  std::cout  <<"blas_Mv error = " <<maxDiff(A*x, y, 0) <<std::endl;
  mlr::useLapack=true;
#endif
}

void blas_MsymMsym(arr& X, const arr& A, const arr& B) {
  CHECK_EQ(A.d1,B.d0, "matrix multiplication: wrong dimensions");
  X.resize(A.d0, B.d1);
  cblas_dsymm(CblasRowMajor,
              CblasLeft, CblasUpper,
              A.d0, B.d1,
              1., A.p, A.d1,
              B.p, B.d1,
              0., X.p, X.d1);
#if 0 //test
  arr Y(A.d0, B.d1);
  uint i, j, k;
  Y.setZero();
  for(i=0; i<Y.d0; i++) for(j=0; j<Y.d1; j++) for(k=0; k<A.d1; k++)
        Y(i, j) += A(i, k) * B(k, j);
  std::cout  <<"blas_MsymMsym error = " <<sqrDistance(X, Y) <<std::endl;
#endif
}

#endif //MLR_NOBLAS

arr lapack_Ainv_b_sym(const arr& A, const arr& b) {
  arr x;
  if(b.nd==2){ //b is a matrix (unusual) repeat for each col:
    MLR_MSG("TODO: directly call lapack with the matrix!")
    arr bT = ~b;
    x.resizeAs(bT);
    for(uint i=0;i<bT.d0;i++) x[i]() = lapack_Ainv_b_sym(A, bT[i]);
    x=~x;
    return x;
  }
  if(isRowShifted(A)) {
    RowShifted *Aaux = (RowShifted*) A.special;
    if(!Aaux->symmetric) HALT("this is not a symmetric matrix");
    for(uint i=0; i<A.d0; i++) if(Aaux->rowShift(i)!=i) HALT("this is not shifted as an upper triangle");
  }
  x=b;
  arr Acol=A;
  integer N=A.d0, KD=A.d1-1, NRHS=1, LDAB=A.d1, INFO;
  try{
    if(!isRowShifted(A)) {
      dposv_((char*)"L", &N, &NRHS, Acol.p, &N, x.p, &N, &INFO);
    } else {
      //assumes symmetric and upper banded
      dpbsv_((char*)"L", &N, &KD, &NRHS, Acol.p, &LDAB, x.p, &N, &INFO);
    }
  }catch(...){
    HALT("here");
  }
  if(INFO) {
#if 1
    uint k=(N>3?3:N); //number of required eigenvalues
    mlr::Array<integer> IWORK(5*N), IFAIL(N);
    arr WORK(10*(3*N)), Acopy=A;
    integer M, IL=1, IU=k, LDQ=0, LDZ=1, LWORK=WORK.N;
    double VL=0., VU=0., ABSTOL=1e-8;
    arr sig(N);
    if(!isRowShifted(A)) {
//      sig.resize(N);
//      dsyev_ ((char*)"N", (char*)"L", &N, A.p, &N, sig.p, WORK.p, &LWORK, &INFO);
//      lapack_EigenDecomp(A, sig, NoArr);
      dsyevx_((char*)"N", (char*)"I", (char*)"L", &N, Acopy.p, &LDAB, &VL, &VU, &IL, &IU, &ABSTOL, &M, sig.p, (double*)NULL, &LDZ, WORK.p, &LWORK, IWORK.p, IFAIL.p, &INFO);
    }else{
      dsbevx_((char*)"N", (char*)"I", (char*)"L", &N, &KD, Acopy.p, &LDAB, (double*)NULL, &LDQ, &VL, &VU, &IL, &IU, &ABSTOL, &M, sig.p, (double*)NULL, &LDZ, WORK.p, IWORK.p, IFAIL.p, &INFO);
    }
    sig.resizeCopy(k);
#else
    arr sig, eig;
    lapack_EigenDecomp(A, sig, eig);
#endif
    mlr::errString <<"lapack_Ainv_b_sym error info = " <<INFO
                  <<". Typically this is because A is not pos-def.\nsmallest "<<k<<" eigenvalues=" <<sig;
    throw(mlr::errString.p);
//    THROW("lapack_Ainv_b_sym error info = " <<INFO
//         <<". Typically this is because A is not pos-def.\nsmallest "<<k<<" eigenvalues=" <<sig);
  }
  return x;
}

uint lapack_SVD(
  arr& U,
  arr& d,
  arr& Vt,
  const arr& A) {
  arr Atmp, work;
  Atmp=A;
  //transpose(Atmp, A);
  integer M=A.d0, N=A.d1, D=M<N?M:N;
  U.resize(M, D);
  d.resize(D);
  Vt.resize(D, N);
  work.resize(10*(M+N));
  integer info, wn=work.N;
  dgesvd_((char*)"S", (char*)"S", &N, &M, Atmp.p, &N, d.p, Vt.p, &N, U.p, &D, work.p, &wn, &info);
  CHECK(!info, "LAPACK SVD error info = " <<info);
  return D;
}

void lapack_LU(arr& LU, const arr& A) {
  LU = A;
  integer M=A.d0, N=A.d1, D=M<N?M:N, info;
  intA piv(D);
  dgetrf_(&N, &M, LU.p, &N, (integer*)piv.p, &info);
  CHECK(!info, "LAPACK SVD error info = " <<info);
}

void lapack_RQ(arr& R, arr &Q, const arr& A) {
  transpose(Q, A);
  R.resizeAs(A); R.setZero();
  integer M=A.d0, N=A.d1, D=M<N?M:N, LWORK=M*N, info;
  arr tau(D), work(LWORK);
  dgerqf_(&N, &M, Q.p, &N, tau.p, work.p, &LWORK, &info);
  CHECK(!info, "LAPACK RQ error info = " <<info);
  for(int i=0; i<M; i++) for(int j=0; j<=i; j++) R(j, i) = Q(i, j); //copy upper triangle
  dorgrq_(&N, &M, &N, Q.p, &N, tau.p, work.p, &LWORK, &info);
  CHECK(!info, "LAPACK RQ error info = " <<info);
  Q=~Q;
  //cout <<"\nR=" <<R <<"\nQ=" <<Q <<"\nRQ=" <<R*Q <<"\nA=" <<A <<endl;
}

void lapack_EigenDecomp(const arr& symmA, arr& Evals, arr& Evecs) {
  CHECK(symmA.nd==2 && symmA.d0==symmA.d1, "not symmetric");
  arr work, symmAcopy = symmA;
  integer N=symmA.d0;
  Evals.resize(N);
  work.resize(10*(3*N));
  integer info, wn=work.N;
  if(&Evecs){
    dsyev_((char*)"V", (char*)"L", &N, symmAcopy.p, &N, Evals.p, work.p, &wn, &info);
    Evecs = symmAcopy;
  }else{
    dsyev_((char*)"N", (char*)"L", &N, symmAcopy.p, &N, Evals.p, work.p, &wn, &info);
  }
  CHECK(!info, "lapack_EigenDecomp error info = " <<info);
}

arr lapack_kSmallestEigenValues_sym(const arr& A, uint k){
  if(k>A.d0) k=A.d0; //  CHECK(k<=A.d0,"");
  integer N=A.d0, KD=A.d1-1, LDAB=A.d1, INFO;
  mlr::Array<integer> IWORK(5*N), IFAIL(N);
  arr WORK(10*(3*N)), Acopy=A;
  integer M, IL=1, IU=k, LDQ=0, LDZ=1, LWORK=WORK.N;
  double VL=0., VU=0., ABSTOL=1e-8;
  arr sig(N);
  if(!isRowShifted(A)) {
    dsyevx_((char*)"N", (char*)"I", (char*)"L", &N, Acopy.p, &LDAB, &VL, &VU, &IL, &IU, &ABSTOL, &M, sig.p, (double*)NULL, &LDZ, WORK.p, &LWORK, IWORK.p, IFAIL.p, &INFO);
  }else{
    dsbevx_((char*)"N", (char*)"I", (char*)"L", &N, &KD, Acopy.p, &LDAB, (double*)NULL, &LDQ, &VL, &VU, &IL, &IU, &ABSTOL, &M, sig.p, (double*)NULL, &LDZ, WORK.p, IWORK.p, IFAIL.p, &INFO);
  }
  sig.resizeCopy(k);
  return sig;
}

bool lapack_isPositiveSemiDefinite(const arr& symmA) {
  // Check that all eigenvalues are nonnegative.
  arr d, V;
  lapack_EigenDecomp(symmA, d, V);
  // d is nondecreasing ??!??
  for(double x:d) if(x<0.) return false;
  return true;
}

/// A=C^T C (C is upper triangular!)
void lapack_cholesky(arr& C, const arr& A) {
  CHECK_EQ(A.d0,A.d1, "");
  integer n=A.d0;
  integer info;
  C=A;
  //compute cholesky
  dpotrf_((char*)"L", &n, C.p, &n, &info);
  CHECK(!info, "LAPACK Cholesky decomp error info = " <<info);
  //clear the lower triangle:
  uint i, j;
  for(i=0; i<C.d0; i++) for(j=0; j<i; j++) C(i, j)=0.;
}

const char *potrf_ERR="\n\
*  INFO    (output) INTEGER\n\
*          = 0:  successful exit\n\
*          < 0:  if INFO = -i, the i-th argument had an illegal value\n\
*          > 0:  if INFO = i, the leading minor of order i is not\n\
*                positive definite, and the factorization could not be\n\
*                completed.\n";

void lapack_mldivide(arr& X, const arr& A, const arr& B) {
  CHECK_EQ(A.nd, 2, "A in Ax=b must be a NxN matrix.");
  CHECK_EQ(A.d0, A.d1, "A in Ax=b must be square matrix.");
  CHECK(B.nd==1 || B.nd==2, "b in Ax=b must be a vector or matrix.");
  CHECK_EQ(A.d0, B.d0, "b and A must have the same amount of rows in Ax=b.");

  X = ~B;
  arr LU = ~A;
  integer N = A.d0, NRHS = (B.nd==1?1:B.d1), LDA = A.d1, INFO;
  mlr::Array<integer> IPIV(N);

  dgesv_(&N, &NRHS, LU.p, &LDA, IPIV.p, X.p, &LDA, &INFO);
  CHECK(!INFO, "LAPACK gaussian elemination error info = " <<INFO);

  X = ~X;
}

void lapack_choleskySymPosDef(arr& Achol, const arr& A) {
  if(isRowShifted(A)) {
    RowShifted *Aaux = (RowShifted*) A.special;
    if(!Aaux->symmetric) HALT("this is not a symmetric matrix");
    for(uint i=0; i<A.d0; i++) if(Aaux->rowShift(i)!=i) HALT("this is not shifted as an upper triangle");

    Achol=A;
    integer N=A.d0, KD=A.d1-1, LDAB=A.d1, INFO;

    dpbtrf_((char*)"L", &N, &KD, Achol.p, &LDAB, &INFO);
    CHECK(!INFO, "LAPACK Cholesky decomp error info = " <<INFO);

  }else{
    NIY;
  }

}

void lapack_inverseSymPosDef(arr& Ainv, const arr& A) {
  Ainv=A;
  integer N=A.d0, LDAB=A.d1, INFO;
  //compute cholesky
  dpotrf_((char*)"L", &N, Ainv.p, &LDAB, &INFO);
  CHECK(!INFO, "LAPACK Cholesky decomp error info = " <<INFO <<potrf_ERR);
  //invert
  dpotri_((char*)"L", &N, Ainv.p, &N, &INFO);
  CHECK(!INFO, "lapack_inverseSymPosDef error info = " <<INFO);
  //fill in the lower triangular elements
  for(uint i=0; i<(uint)N; i++) for(uint j=0; j<i; j++) Ainv.p[i*N+j]=Ainv.p[j*N+i]; //fill in the lower triangle
}

double lapack_determinantSymPosDef(const arr& A) {
  arr C;
  lapack_cholesky(C, A);
  double det=1.;
  for(uint i=0; i<C.d0; i++) det *= C(i, i)*C(i, i);
  return det;
}

void lapack_min_Ax_b(arr& x,const arr& A, const arr& b) {
  CHECK(A.d0>=A.d1 && A.d0==b.N && b.nd==1 && A.nd==2, "");
  arr At = ~A;
  x=b;
  integer M=A.d0, N=A.d1, NRHS=1, LWORK=2*M*N, info;
  arr work(LWORK);
  dgels_((char*)"N", &M, &N, &NRHS, At.p, &M, x.p, &M, work.p, &LWORK, &info);
  CHECK(!info, "dgels_ error info = " <<info);
  x.resizeCopy(A.d1);
}

arr lapack_Ainv_b_symPosDef_givenCholesky(const arr& U, const arr& b) {
  //in lapack (or better fortran) the rows and columns are switched!! (ARGH)
  integer N = U.d0, LDA = U.d1, INFO, LDB = b.d0, NRHS = 1;
  arr x;
  if(b.nd > 1) {
    NRHS = b.d1;
    x = ~b; //TODO is there a chance to remove this?
    dpotrs_((char*)"L", &N, &NRHS, U.p, &LDA, x.p, &LDB, &INFO);
    CHECK(!INFO, "lapack dpotrs error info = " << INFO);
    return ~x;
  } else {
    x = b;
    dpotrs_((char*)"L", &N, &NRHS, U.p, &LDA, x.p, &LDB, &INFO);
    CHECK(!INFO, "lapack dpotrs error info = " << INFO);
    return x;
  }
}

arr lapack_Ainv_b_triangular(const arr& L, const arr& b) {
  //DTRTRS
  integer N = L.d0, LDA = L.d0, INFO, LDB = b.d0, NRHS = 1;
  arr x = b;
  dtrtrs_((char*)"L", (char*)"N", (char*)"N", &N, &NRHS, L.p, &LDA, x.p, &LDB, &INFO);
  CHECK(!INFO, "lapack dtrtrs error info = " << INFO);
  return x;
}

/*
dpotri uses:
dtrtri = invert triangular

dlauum = multiply L'*L
*/

#else //if defined MLR_LAPACK
#if !defined MLR_MSVC && defined MLR_NOCHECK
#  warning "MLR_LAPACK undefined - using inefficient implementations"
#endif
void blas_MM(arr& X, const arr& A, const arr& B) { mlr::useLapack=false; innerProduct(X, A, B); };
void blas_MsymMsym(arr& X, const arr& A, const arr& B) { mlr::useLapack=false; innerProduct(X, A, B); };
void blas_Mv(arr& y, const arr& A, const arr& x) {       mlr::useLapack=false; innerProduct(y, A, x); mlr::useLapack=true; };
void blas_A_At(arr& X, const arr& A) { NICO }
void blas_At_A(arr& X, const arr& A) { NICO }
void lapack_cholesky(arr& C, const arr& A) { NICO }
uint lapack_SVD(arr& U, arr& d, arr& Vt, const arr& A) { NICO; }
void lapack_LU(arr& LU, const arr& A) { NICO; }
void lapack_RQ(arr& R, arr &Q, const arr& A) { NICO; }
void lapack_EigenDecomp(const arr& symmA, arr& Evals, arr& Evecs) { NICO; }
bool lapack_isPositiveSemiDefinite(const arr& symmA) { NICO; }
void lapack_inverseSymPosDef(arr& Ainv, const arr& A) { NICO; }
arr lapack_kSmallestEigenValues_sym(const arr& A, uint k){ NICO; }
arr lapack_Ainv_b_sym(const arr& A, const arr& b) {
  arr invA;
  inverse(invA, A);
  return invA*b;
};
double lapack_determinantSymPosDef(const arr& A) { NICO; }
void lapack_mldivide(arr& X, const arr& A, const arr& b) { NICO; }
arr lapack_Ainv_b_symPosDef_givenCholesky(const arr& U, const arr&b) { return inverse(U)*b; }
arr lapack_Ainv_b_triangular(const arr& L, const arr& b) { return inverse(L)*b; }
#endif


//===========================================================================
//
// RowShifted
//

RowShifted::RowShifted(arr& X):Z(X), real_d1(0), symmetric(false) {
  type = SpecialArray::RowShiftedST;
  Z.special = this;
}

RowShifted::RowShifted(arr& X, RowShifted &aux):
  Z(X),
  real_d1(aux.real_d1),
  rowShift(aux.rowShift),
  rowLen(aux.rowLen),
  colPatches(aux.colPatches),
  symmetric(aux.symmetric)
{
  type = SpecialArray::RowShiftedST;
  Z.special=this;
}

RowShifted *makeRowShifted(arr& Z, uint d0, uint pack_d1, uint real_d1) {
  RowShifted *Zaux;
  if(!Z.special) {
    Zaux = new RowShifted(Z);
  } else {
    CHECK_EQ(Z.special->type, SpecialArray::RowShiftedST, "");
    Zaux = dynamic_cast<RowShifted*>(Z.special);
  }
  Z.resize(d0, pack_d1);
  Z.setZero();
  Zaux->real_d1=real_d1;
  Zaux->rowShift.resize(d0);
  Zaux->rowShift.setZero();
  Zaux->rowLen.resize(d0);
  if(d0) Zaux->rowLen = pack_d1;
  return Zaux;
}

RowShifted::~RowShifted() {
  Z.special = NULL;
}

double RowShifted::elem(uint i, uint j) {
  uint rs=rowShift(i);
  if(j<rs || j>=rs+Z.d1) return 0.;
  return Z(i, j-rs);
}

void RowShifted::reshift(){
  rowLen.resize(Z.d0);
  for(uint i=0; i<Z.d0; i++) {
#if 1
    //find number of leading and trailing zeros
    double *Zp = Z.p + i*Z.d1;
    double *Zlead = Zp;
    double *Ztrail = Zp + Z.d1-1;
    while(Ztrail>=Zlead && *Ztrail==0.) Ztrail--;
    while(Zlead<=Ztrail && *Zlead==0. ) Zlead++;
    if(Ztrail<Zlead){ //all zeros
      rowLen.p[i]=0.;
    }else{
      uint rs = Zlead-Zp;
      uint len = 1+Ztrail-Zlead;
      rowShift.p[i] += rs;
      rowLen.p[i] = len;
      if(Zlead!=Zp){
        memmove(Zp, Zlead, len*Z.sizeT);
        memset (Zp+len, 0, (Z.d1-len)*Z.sizeT);
      }
    }
#else
    //find number of leading zeros
    uint j=0;
    while(j<Z.d1 && Z(i,j)==0.) j++;
    //shift or so..
    if(j==Z.d1){ //all zeros...
    }else if(j){ //some zeros
      rowShift(i) += j;
      memmove(&Z(i,0), &Z(i,j), (Z.d1-j)*Z.sizeT);
      memset (&Z(i,Z.d1-j), 0, j*Z.sizeT);
    }
    //find number of trailing zeros
    j=Z.d1;
    while(j>0 && Z(i,j-1)==0.) j--;
    rowLen(i)=j;
#endif
  }
}

arr packRowShifted(const arr& X) {
#if 1
  arr Z;
  RowShifted *Zaux = makeRowShifted(Z, X.d0, X.d1, X.d1);
  memmove(Z.p, X.p, Z.N*Z.sizeT);
  Zaux->reshift();
  return Z;
#else
  arr Z;
  RowShifted *Zaux = makeRowShifted(Z, X.d0, 0, X.d1);
  Z.setZero();
  //-- compute rowShifts and pack_d1:
  uint pack_d1=0;
  for(uint i=0; i<X.d0; i++) {
    uint j=0,rs;
    while(j<X.d1 && X(i,j)==0.) j++;
    Zaux->rowShift(i)=rs=j;
    j=X.d1;
    while(j>rs && X(i,j-1)==0.) j--;
    if(j-rs>pack_d1) pack_d1=j-rs;
  }
  
  Z.resize(X.d0,pack_d1);
  Z.setZero();
  for(uint i=0; i<Z.d0; i++) for(uint j=0; j<Z.d1 && Zaux->rowShift(i)+j<X.d1; j++)
      Z(i,j) = X(i,Zaux->rowShift(i)+j);
  return Z;
#endif
}

arr unpackRowShifted(const arr& Y) {
  CHECK(isRowShifted(Y),"");
  RowShifted *Yaux = (RowShifted*)Y.special;
  arr X(Y.d0, Yaux->real_d1);
  CHECK(!Yaux->symmetric || Y.d0==Yaux->real_d1,"cannot be symmetric!");
  X.setZero();
  for(uint i=0; i<Y.d0; i++) {
    uint rs=Yaux->rowShift(i);
    for(uint j=0; j<Y.d1 && rs+j<X.d1; j++) {
      X(i,j+rs) = Y(i,j);
      if(Yaux->symmetric) X(j+rs,i) = Y(i,j);
    }
  }
  return X;
}

void RowShifted::computeColPatches(bool assumeMonotonic) {
  colPatches.resize(real_d1,2);
  uint a=0,b=Z.d0;
  if(!assumeMonotonic) {
    for(uint j=0; j<real_d1; j++) {
      a=0;
      while(a<Z.d0 && elem(a,j)==0) a++;
      b=Z.d0;
      while(b>a && elem(b-1,j)==0) b--;
      colPatches.p[2*j]=a;
      colPatches.p[2*j+1]=b;
    }
  } else {
    for(uint j=0; j<real_d1; j++) {
      while(a<Z.d0 && j>=rowShift.p[a]+Z.d1) a++;
      colPatches.p[2*j]=a;
    }
    for(uint j=real_d1; j--;) {
      while(b>0 && j<rowShift.p[b-1]) b--;
      colPatches.p[2*j+1]=b;
    }
  }
}

arr RowShifted::At_A() {
  //TODO use blas DSYRK instead?
  CHECK_EQ(rowLen.N, rowShift.N, "");
  arr R;
  RowShifted *Raux = makeRowShifted(R, real_d1, Z.d1, real_d1);
  R.setZero();
  for(uint i=0; i<R.d0; i++) Raux->rowShift(i) = i;
  Raux->symmetric=true;
  if(!Z.d1) return R; //Z is identically zero, all rows fully packed -> return zero R
  for(uint i=0; i<Z.d0; i++) {
    uint rs=rowShift.p[i];
    uint rlen=rowLen.p[i];
    double* Zi = Z.p+i*Z.d1;
    for(uint j=0; j<rlen/*Z.d1*/; j++) {
      uint real_j=j+rs;
      if(real_j>=real_d1) break;
      double Zij=Zi[j];
      if(Zij!=0.){
        double* Rp=R.p + real_j*R.d1;
        double* Jp=Zi+j;
        double* Jpstop=Zi+rlen; //Z.d1;
        for(; Jp!=Jpstop; Rp++,Jp++) if(*Jp!=0.) *Rp += Zij * *Jp;
      }
    }
  }
  return R;
}

arr RowShifted::A_At() {
  //-- determine pack_d1 for the resulting symmetric matrix
  uint pack_d1=1;
  for(uint i=0; i<Z.d0; i++) {
    uint rs_i=rowShift.p[i];
    for(uint j=Z.d0-1; j>=i+pack_d1; j--) {
      uint rs_j=rowShift.p[j];
      uint a,b;
      if(rs_i<rs_j){ a=rs_j; b=rs_i+Z.d1; }else{ a=rs_i; b=rs_j+Z.d1; }
      if(real_d1<b) b=real_d1;
      if(a<b) if(pack_d1<j-i+1) pack_d1=j-i+1;
    }
  }

  arr R;
  RowShifted *Raux = makeRowShifted(R, Z.d0, pack_d1, Z.d0);
  R.setZero();
  for(uint i=0; i<R.d0; i++) Raux->rowShift(i) = i;
  Raux->symmetric=true;
  if(!Z.d1) return R; //Z is identically zero, all rows fully packed -> return zero R
  for(uint i=0; i<Z.d0; i++) {
    uint rs_i=rowShift.p[i];
    double* Zi=&Z(i,0);
    for(uint j=i; j<Z.d0 && j<i+pack_d1; j++) {
      uint rs_j=rowShift.p[j];
      double* Zj=&Z(j,0);
      double* Rij=&R(i,j-i);

      uint a,b;
      if(rs_i<rs_j){ a=rs_j; b=rs_i+Z.d1; }else{ a=rs_i; b=rs_j+Z.d1; }
      if(real_d1<b) b=real_d1;
      for(uint k=a;k<b;k++) *Rij += Zi[k-rs_i]*Zj[k-rs_j];
    }
  }
  return R;
}

arr RowShifted::At_x(const arr& x) {
  CHECK_EQ(rowLen.N, rowShift.N, "");
  CHECK_EQ(x.N,Z.d0,"");
  arr y(real_d1);
  y.setZero();
//  cout <<"SPARSITY = " <<Z.sparsity() <<endl;
  if(!Z.d1) return y; //Z is identically zero, all rows fully packed -> return zero y
  for(uint i=0; i<Z.d0; i++) {
    double xi = x.p[i];
    uint rs=rowShift.p[i];
#if 0
    for(uint j=0; j<Z.d1; j++) y.p[rs+j] += xi * Z.p[i*Z.d1+j]; // sum += acc(i,j)*x(i);
#else //PROFILED
    double *Zp = Z.p + i*Z.d1;
    double *yp = y.p + rs;
    double *ypstop = yp + rowLen.p[i]; //+ Z.d1;
    for(;yp!=ypstop;){ *yp += xi * *Zp;  Zp++;  yp++; }
#endif
  }
  return y;
}

arr RowShifted::A_x(const arr& x) {
  if(x.nd==2){
    arr Y(x.d1, Z.d0);
    arr X = ~x;
    for(uint j=0;j<x.d1;j++) Y[j]() = A_x(X[j]);
    return ~Y;
  }
  CHECK_EQ(x.N,real_d1,"");
  arr y = zeros(Z.d0);
  if(!Z.d1) return y; //Z is identically zero, all rows fully packed -> return zero y
  for(uint i=0; i<Z.d0; i++) {
    double sum=0.;
    uint rs=rowShift.p[i];
    for(uint j=0; j<Z.d1 && j+rs<x.N; j++) {
      sum += Z(i,j)*x(j+rs);
    }
    y(i) = sum;
  }
  return y;
}

arr RowShifted::At(){
  uint width = 0;
  if(!colPatches.N) computeColPatches(false);
  for(uint i=0;i<colPatches.d0;i++){ uint a=colPatches(i,1)-colPatches(i,0); if(a>width) width=a; }

  arr At;
  RowShifted* At_ = makeRowShifted(At, real_d1, width, Z.d0);
  for(uint i=0;i<real_d1;i++){
    uint rs = colPatches(i,0);
    At_->rowShift(i) = rs;
    uint rlen = colPatches(i,1)-rs;
    for(uint j=0;j<rlen;j++) At_->Z(i,j) = elem(rs+j,i);
  }
  return At;
}



//===========================================================================
//
// generic special
//

arr unpack(const arr& X) {
  if(isNotSpecial(X)) HALT("this is not special");
  if(isRowShifted(X)) return unpackRowShifted(X);
  return NoArr;
}

arr comp_At_A(const arr& A) {
  if(isNotSpecial(A)) { arr X; blas_At_A(X,A); return X; }
  if(isRowShifted(A)) return ((RowShifted*)A.special)->At_A();
  return NoArr;
}

arr comp_A_At(const arr& A) {
  if(isNotSpecial(A)) { arr X; blas_A_At(X,A); return X; }
  if(isRowShifted(A)) return ((RowShifted*)A.special)->A_At();
  return NoArr;
}

//arr comp_A_H_At(arr& A, const arr& H){
//  if(isNotSpecial(A)) { arr X; blas_A_At(X,A); return X; }
//  if(isRowShifted(A)) return ((RowShifted*)A.aux)->A_H_At(H);
//  return NoArr;
//}

arr comp_At_x(const arr& A, const arr& x) {
  if(isNotSpecial(A)) { arr y; innerProduct(y, ~A, x); return y; }
  if(isRowShifted(A)) return ((RowShifted*)A.special)->At_x(x);
  return NoArr;
}

arr comp_At(const arr& A) {
  if(isNotSpecial(A)) { return ~A; }
  if(isRowShifted(A)) return ((RowShifted*)A.special)->At();
  return NoArr;
}

arr comp_A_x(const arr& A, const arr& x) {
  if(isNotSpecial(A)) { arr y; innerProduct(y, A, x); return y; }
  if(isRowShifted(A)) return ((RowShifted*)A.special)->A_x(x);
  return NoArr;
}


//===========================================================================
//
// conv with Eigen
//

#ifdef MT_EIGEN

arr conv_eigen2arr(const Eigen::MatrixXd& in) {
  arr out(in.rows(), in.cols());
  for(uint i = 0; i<in.rows(); i++)
    for(uint j = 0; j<in.cols(); j++)
      out(i, j) = in(i, j);
  return out; 
}

Eigen::MatrixXd conv_arr2eigen(const arr& in) {
  if(in.nd == 1) {
    Eigen::MatrixXd out(in.d0, 1);  
    for(uint i = 0; i<in.d0; i++)
        out(i, 0) = in(i);
    return out;
  }
  else if(in.nd == 2) {
    Eigen::MatrixXd out(in.d0, in.d1);  
    for(uint i = 0; i<in.d0; i++)
      for(uint j = 0; j<in.d1; j++)
        out(i, j) = in(i, j);
    return out;
  }
}

#endif

//===========================================================================
//
// graphs
//

void graphRandomUndirected(uintA& E, uint n, double connectivity) {
  uint i, j;
  for(i=0; i<n; i++) for(j=i+1; j<n; j++) {
      if(rnd.uni()<connectivity) E.append(TUP(i,j));
    }
  E.reshape(E.N/2,2);
}

void graphRandomTree(uintA& E, uint N, uint roots) {
  uint i;
  CHECK(roots>=1, "");
  for(i=roots; i<N; i++) E.append(TUP(rnd(i), i));
  E.reshape(E.N/2,2);
}

void graphRandomFixedDegree(uintA& E, uint N, uint d) {
  // --- from Joris' libDAI!!
  // Algorithm 1 in "Generating random regular graphs quickly"
  // by A. Steger and N.C. Wormald
  //
  // Draws a random graph with size N and uniform degree d
  // from an almost uniform probability distribution over these graphs
  // (which becomes uniform in the limit that d is small and N goes
  // to infinity).
  
  CHECK_EQ((N*d)%2,0, "It's impossible to create a graph with " <<N<<" nodes and fixed degree " <<d);
  
  uint j;
  
  bool ready = false;
  uint tries = 0;
  while(!ready) {
    tries++;
    
    // Start with N*d points {0, 1, ..., N*d-1} (N*d even) in N groups.
    // Put U = {0, 1, ..., N*d-1}. (U denotes the set of unpaired points.)
    uintA U;
    U.setStraightPerm(N*d);
    
    // Repeat the following until no suitable pair can be found: Choose
    // two random points i and j in U, and if they are suitable, pair
    // i with j and delete i and j from U.
    E.clear();
    bool finished = false;
    while(!finished) {
      U.permuteRandomly();
      uint i1, i2;
      bool suit_pair_found = false;
      for(i1=0; i1<U.N-1 && !suit_pair_found; i1++) {
        for(i2=i1+1; i2<U.N && !suit_pair_found; i2++) {
          if((U(i1)/d) != (U(i2)/d)) {  // they are suitable (refer to different nodes)
            suit_pair_found = true;
            E.append(TUP(U(i1)/d, U(i2)/d));
            U.remove(i2);  // first remove largest
            U.remove(i1);  // remove smallest
          }
          if(!suit_pair_found || !U.N)  finished = true;
        }
      }
    }
    E.reshape(E.N/2,2);
    if(!U.N) {
      // G is a graph with edge from vertex r to vertex s if and only if
      // there is a pair containing points in the r'th and s'th groups.
      // If G is d-regular, output, otherwise return to Step 1.
      uintA degrees(N);
      degrees.setZero();
      for(j=0; j<E.d0; j++) {
        degrees(E(j,0))++;
        degrees(E(j,1))++;
      }
      ready = true;
      for(uint n=0; n<N; n++) {
        CHECK(degrees(n)<=d, "");
        if(degrees(n)!=d) {
          ready = false;
          break;
        }
      }
    } else ready=false;
  }
  
  E.reshape(E.N/2,2);
}

//===========================================================================
//
// explicit instantiations
// (in old versions, array.tpp was not included by array.h -- one could revive this)
//

//#include "array.tpp"
//#define T double
//#  include "array_instantiate.cxx"
//#undef T

//#define NOFLOAT
//#define T float
//#  include "array_instantiate.cxx"
//#undef T

//#define T uint
//#  include "array_instantiate.cxx"
//#undef T

//#define T uint16_t
//#  include "array_instantiate.cxx"
//#undef T

//#define T int
//#  include "array_instantiate.cxx"
//#undef T

//#define T long
//#  include "array_instantiate.cxx"
//#undef T
//#define T byte
//#  include "array_instantiate.cxx"
//#undef T
//#undef NOFLOAT

template mlr::Array<mlr::String>::Array();
template mlr::Array<mlr::String>::~Array();

template mlr::Array<mlr::String*>::Array();
template mlr::Array<mlr::String*>::~Array();

template arrL::Array();
template arrL::Array(uint);
template arrL::~Array();

template mlr::Array<char const*>::Array();
template mlr::Array<char const*>::Array(uint);
template mlr::Array<char const*>::~Array();

template mlr::Array<uintA>::Array();
template mlr::Array<uintA>::Array(uint);
template mlr::Array<uintA>::~Array();

template mlr::Array<arr>::Array();
template mlr::Array<arr>::Array(uint);
template mlr::Array<arr>::~Array();

#include "util.tpp"

template mlr::Array<double> mlr::getParameter<arr>(char const*);
template mlr::Array<double> mlr::getParameter<arr>(char const*, const arr&);
template mlr::Array<float> mlr::getParameter<floatA>(char const*);
template mlr::Array<uint> mlr::getParameter<uintA>(char const*);
template bool mlr::checkParameter<arr>(char const*);
template void mlr::getParameter(uintA&, const char*, const uintA&);

void linkArray() { cout <<"*** libArray.so dynamically loaded ***" <<endl; }

//namespace mlr{
//template<> template<> Array<mlr::String>::Array(std::initializer_list<const char*> list) {
//  init();
//  for(const char* t : list) append(mlr::String(t));
//}
//}





/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#include <map>

#include "util.tpp"
#include "array.tpp"
#include "graph.h"

#define DEBUG(x)

NodeL& NoNodeL=*((NodeL*)NULL);
Graph& NoGraph=*((Graph*)NULL);

//===========================================================================
//
// annotations to a node while parting; can be used for highlighting and error messages
//

struct ParseInfo{
  istream::pos_type beg,end;
  istream::pos_type err_beg, err_end;
  istream::pos_type keys_beg, keys_end;
  istream::pos_type parents_beg, parents_end;
  istream::pos_type value_beg, value_end;
  enum Error{ good=0, unknownParent };
  void write(ostream& os) const{ os <<'<' <<beg <<',' <<end <<'>'; }
};
stdOutPipe(ParseInfo)


//===========================================================================
//
// retrieving types
//

//-- query existing types
inline Node *reg_findType(const char* key) {
  NodeL types = registry()->getNodesOfType<std::shared_ptr<Type> >();
  for(Node *ti: types) {
    if(mlr::String(ti->get<std::shared_ptr<Type> >()->typeId().name())==key) return ti;
    if(ti->matches(key)) return ti;
  }
  return NULL;
}


//===========================================================================
//
// read a value from a stream by looking up available registered types
//

inline Node* readTypeIntoNode(Graph& container, const char* key, std::istream& is) {
  Node *ti = reg_findType(key);
  if(ti) return ti->get<std::shared_ptr<Type> >()->readIntoNewNode(container, is);
  return NULL;
}

//===========================================================================
//
//  Node methods
//

Node::Node(const std::type_info& _type, void* _value_ptr, Graph& _container)
  : type(_type), value_ptr(_value_ptr), container(_container){
  CHECK(&container!=&NoGraph, "don't do that anymore!");
  index=container.N;
  container.NodeL::append(this);
}

Node::Node(const std::type_info& _type, void* _value_ptr, Graph& _container, const StringA& _keys, const NodeL& _parents)
  : type(_type), value_ptr(_value_ptr), container(_container), keys(_keys), parents(_parents){
  CHECK(&container!=&NoGraph, "This is a NoGraph (NULL) -- don't do that anymore!");
  index=container.N;
  container.NodeL::append(this);
  if(parents.N) for(Node *i: parents){
    CHECK(i,"you gave me a NULL parent");
    i->parentOf.append(this);
  }
}

Node::~Node() {
  for(Node *i: parents) i->parentOf.removeValue(this);
  for(Node *i: parentOf) i->parents.removeValue(this);
  container.removeValue(this);
  container.index();
}

bool Node::matches(const char *key){
  for(const mlr::String& k:keys) if(k==key) return true;
  return false;
}

bool Node::matches(const StringA &query_keys) {
  for(const mlr::String& k:query_keys) {
    if(!matches(k)) return false;
  }
  return true;
}

void Node::write(std::ostream& os) const {
  //-- write keys
  keys.write(os, " ", "", "\0\0");
  
  //-- write parents
  if(parents.N) {
    //    if(keys.N) os <<' ';
    os <<'(';
    for_list(Node, it, parents) {
      if(it_COUNT) os <<' ';
      if(it->keys.N){
        os <<it->keys.last();
      }else{  //relative numerical reference
        os <<(int)it->index - (int)index;
      }
    }
    os <<')';
  }
  
  //-- write value
  if(isGraph()) {
    os <<" {";
    graph().write(os, " ");
    os <<" }";
  } else if(isOfType<NodeL>()) {
    os <<"=(";
    for(Node *it: (*getValue<NodeL>())) os <<' ' <<it->keys.last();
    os <<" )";
  } else if(isOfType<mlr::String>()) {
    os <<"=\"" <<*getValue<mlr::String>() <<'"';
  } else if(isOfType<mlr::FileToken>()) {
    os <<"='" <<getValue<mlr::FileToken>()->name <<'\'';
  } else if(isOfType<arr>()) {
    os <<'='; getValue<arr>()->write(os, NULL, NULL, "[]");
  } else if(isOfType<double>()) {
    os <<'=' <<*getValue<double>();
  } else if(isOfType<bool>()) {
    if(*getValue<bool>()) os<<','; else os <<'!';
  } else if(isOfType<Type*>()) {
    os <<" = "; get<Type*>()->write(os);
  } else {
    Node *it = reg_findType(type.name());
    if(it && it->keys.N>1) {
      os <<" = <" <<it->keys(1) <<' ';
      writeValue(os);
      os <<'>';
    } else {
      os <<" = \" ";
      writeValue(os);
      os <<'"';
    }
  }
}

Nod::Nod(const char* key){
  n = G.newNode<bool>(true);
  n->keys.append(STRING(key));
}

Nod::Nod(const char* key, const char* stringValue){
  n = G.newNode<mlr::String>(STRING(stringValue));
  n->keys.append(STRING(key));
}



//===========================================================================
//
//  Graph methods
//

Graph::Graph() : isNodeOfGraph(NULL), pi(NULL), ri(NULL) {
}

Graph::Graph(const char* filename): Graph() {
  read(mlr::FileToken(filename).getIs());
}

Graph::Graph(istream& is) : Graph() {
  read(is);
}

Graph::Graph(const std::map<std::string, std::string>& dict) : Graph() {
  appendDict(dict);
}

Graph::Graph(std::initializer_list<Nod> list) : Graph() {
  for(const Nod& ni:list) newNode(ni);
}

Graph::Graph(const Graph& G) : Graph() {
  *this = G;
}

Graph::~Graph() {
  clear();
}

void Graph::clear() {
  if(ri){ delete ri; ri=NULL; }
  if(pi){ delete pi; pi=NULL; }
  while(N) delete last();
}

Graph& Graph::newNode(const Nod& ni){
  Node *clone = ni.n->newClone(*this); //this appends sequentially clones of all nodes to 'this'
  for(const mlr::String& s:ni.parents){
    Node *p = getNode(s);
    CHECK(p,"parent " <<p <<" of " <<*clone <<" does not exist!");
    clone->parents.append(p);
    p->parentOf.append(clone);
  }
  return *this;
}

Node_typed<Graph>* Graph::newSubgraph(const StringA& keys, const NodeL& parents, const Graph& x){
  Node_typed<Graph>* n = newNode<Graph>(keys, parents, Graph());
  DEBUG( CHECK(n->value.isNodeOfGraph && &n->value.isNodeOfGraph->container==this,"") )
  if(&x) n->value.copy(x);
  return n;
}

Node_typed<int>* Graph::newNode(const uintA& parentIdxs) {
  NodeL parents(parentIdxs.N);
  for(uint i=0;i<parentIdxs.N; i++) parents(i) = NodeL::elem(parentIdxs(i));
  return newNode<int>({STRING(NodeL::N)}, parents, 0);
}

void Graph::appendDict(const std::map<std::string, std::string>& dict){
  for(const std::pair<std::string,std::string>& p:dict){
    Node *n = readNode(STRING('='<<p.second), false, false, mlr::String(p.first));
    if(!n) MLR_MSG("failed to read dict entry <" <<p.first <<',' <<p.second <<'>');
  }
}

Node* Graph::findNode(const StringA& keys, bool recurseUp, bool recurseDown) const {
  for(Node* n: (*this)) if(n->matches(keys)) return n;
  Node* ret=NULL;
  if(recurseUp && isNodeOfGraph) ret = isNodeOfGraph->container.findNode(keys, true, false);
  if(ret) return ret;
  if(recurseDown) for(Node *n: (*this)) if(n->isGraph()){
    ret = n->graph().findNode(keys, false, true);
    if(ret) return ret;
  }
  return ret;
}

Node* Graph::findNodeOfType(const std::type_info& type, const StringA& keys, bool recurseUp, bool recurseDown) const {
  for(Node* n: (*this)) if(n->type==type && n->matches(keys)) return n;
  Node* ret=NULL;
  if(recurseUp && isNodeOfGraph) ret = isNodeOfGraph->container.findNodeOfType(type, keys, true, false);
  if(ret) return ret;
  if(recurseDown) for(Node *n: (*this)) if(n->isGraph()){
    ret = n->graph().findNodeOfType(type, keys, false, true);
    if(ret) return ret;
  }
  return ret;
}

NodeL Graph::findNodes(const StringA& keys, bool recurseUp, bool recurseDown) const {
  NodeL ret;
  for(Node *n: (*this)) if(n->matches(keys)) ret.append(n);
  if(recurseUp && isNodeOfGraph) ret.append( isNodeOfGraph->container.findNodes(keys, true, false) );
  if(recurseDown) for(Node *n: (*this)) if(n->isGraph()) ret.append( n->graph().findNodes(keys, false, true) );
  return ret;
}

NodeL Graph::findNodesOfType(const std::type_info& type, const StringA& keys, bool recurseUp, bool recurseDown) const {
  NodeL ret;
  for(Node *n: (*this)) if(n->type==type && n->matches(keys)) ret.append(n);
  if(recurseUp && isNodeOfGraph) ret.append( isNodeOfGraph->container.findNodesOfType(type, keys, true, false) );
  if(recurseDown) for(Node *n: (*this)) if(n->isGraph()) ret.append( n->graph().findNodesOfType(type, keys, false, true) );
  return ret;
}

//Node* Graph::getNode(const char *key) const {
//  for(Node *n: (*this)) if(n->matches(key)) return n;
//  if(isNodeOfGraph) return isNodeOfGraph->container.getNode(key);
//  return NULL;
//}

//Node* Graph::getNode(const StringA &keys) const {
//  for(Node *n: (*this)) if(n->matches(keys)) return n;
//  if(isNodeOfGraph) return isNodeOfGraph->container.getNode(keys);
//  return NULL;
//}

//NodeL Graph::getNodes(const StringA &keys) const {
//  NodeL ret;
//  for(Node *n: (*this)) if(n->matches(keys)) ret.append(n);
//  return ret;

//}

//NodeL Graph::getNodes(const char* key) const {
//  NodeL ret;
//  for(Node *n: (*this)) if(n->matches(key)) ret.append(n);
//  return ret;
//}

Node* Graph::getEdge(Node *p1, Node *p2) const{
  if(p1->parentOf.N < p2->parentOf.N){
    for(Node *i:p1->parentOf){
      if(p2->parentOf.findValue(i)!=-1) return i;
    }
  }else{
    for(Node *i:p2->parentOf){
      if(p1->parentOf.findValue(i)!=-1) return i;
    }
  }
  return NULL;
}

Node* Graph::getEdge(const NodeL& parents) const{
  CHECK(parents.N>0,"");
  //grap 'sparsest' parent:
  uint minSize = this->N;
  Node *sparsestParent = NULL;
  for(Node *p:parents) if(p->parentOf.N<minSize){ sparsestParent=p; minSize=p->parentOf.N; }
  if(!sparsestParent){
    for(Node *e:*this) if(e->parents==parents) return e;
  }else{
    for(Node *e:sparsestParent->parentOf) if(&e->container==this){
      if(e->parents==parents) return e;
    }
  }
  return NULL;
}

NodeL Graph::getNodesOfDegree(uint deg) {
  NodeL ret;
  for(Node *n: (*this)) if(n->parents.N==deg) ret.append(n);
  return ret;
}

Node* Graph::edit(Node *ed){
  NodeL KVG = findNodesOfType(ed->type, ed->keys);
  //CHECK(KVG.N<=1, "can't edit into multiple nodes yet");
  Node *n=NULL;
  if(KVG.N) n=KVG.elem(0);
  CHECK(n!=ed,"how is this possible?: You're trying to edit with '" <<*ed <<"' but this is the only node using these keys");
  if(n){
    CHECK(ed->type==n->type, "can't edit/merge nodes of different types!");
    if(n->isGraph()){ //merge the KVGs
      n->graph().edit(ed->graph());
    }else{ //overwrite the value
      n->copyValue(ed);
    }
    if(&ed->container==this){ delete ed; ed=NULL; }
  }else{ //nothing to merge, append
    if(&ed->container!=this){
      Node *it = ed->newClone(*this);
      for(uint i=0;i<it->parents.N;i++){
        it->parents(i) = elem(it->parents(i)->index);
        it->parents(i)->parentOf.append(it);
      }
    }
    return ed;
  }
  return NULL;
}

void Graph::copy(const Graph& G, bool appendInsteadOfClear, bool enforceCopySubgraphToNonsubgraph){
  DEBUG(G.checkConsistency());

  CHECK(this!=&G, "Graph self copy -- never do this");

  if(!enforceCopySubgraphToNonsubgraph){
    if(G.isNodeOfGraph && !this->isNodeOfGraph){
      HALT("Typically you should not copy a subgraph into a non-subgraph (or call the copy operator with a subgraph).\
           Use 'newSubgraph' instead\
           If you still want to do it you need to ensure that all node parents are declared, and then enforce it by setting 'enforceCopySubgraphToNonsubgraph'");
    }
  }else{
    if(this->isNodeOfGraph){
      HALT("You set 'enforceCopySubgraphToNonsubgraph', but this is not a Nonsubgraph");
    }
  }

  //-- first delete existing nodes
  if(!appendInsteadOfClear) clear();
  uint indexOffset=N;
  NodeL newNodes;

  //-- if either is a subgraph, ensure they're a subgraph of the same -- over restrictive!!
//  if(isNodeOfGraph || G.isNodeOfGraph){
//    CHECK(&isNodeOfGraph->container==&G.isNodeOfGraph->container,"is already subgraph of another container!");
//  }

  //-- first, just clone nodes with their values -- 'parents' still point to the origin nodes
  for(Node *n:G){
    Node *newn=NULL;
    if(n->isGraph()){
      // why we can't copy the subgraph yet:
      // copying the subgraph would require to fully rewire the subgraph (code below)
      // but if the subgraph refers to parents of this graph that are not create yet, requiring will fail
      // therefore we just insert an empty graph here; we then copy the subgraph once all nodes are created
      newn = this->newSubgraph(n->keys, n->parents);
    }else{
      newn = n->newClone(*this); //this appends sequentially clones of all nodes to 'this'
    }
    newNodes.append(newn);
  }

  //-- the new nodes are not parent of anybody yet
  for(Node *n:newNodes) CHECK(n->parentOf.N==0,"");

  //-- now copy subgraphs
  for(Node *n:newNodes) if(n->isGraph()){
    n->graph().copy(G.elem(n->index-indexOffset)->graph()); //you can only call the operator= AFTER assigning isNodeOfGraph
  }

  //-- now rewire parental links
  for(Node *n:newNodes){
    for(uint i=0;i<n->parents.N;i++){
      Node *p=n->parents(i); //the parent in the origin graph
      if(isChildOfGraph(p->container)) continue;
      if(&p->container==&G){ //parent is directly in G, no need for complicated search
        p->parentOf.removeValue(n);   //original parent is not parent of copy
        p = newNodes.elem(p->index);  //the true parent in the new graph
      }else{
        const Graph *newg=this, *oldg=&G;
        while(&p->container!=oldg){  //find the container while iterating backward also in the newG
          CHECK(oldg->isNodeOfGraph,"");
          CHECK(newg->isNodeOfGraph,"");
          newg = &newg->isNodeOfGraph->container;
          oldg = &oldg->isNodeOfGraph->container;
        }
        CHECK(newg->N==oldg->N,"different size!!\n" <<*newg <<"**\n" <<*oldg);
        CHECK(p==oldg->elem(p->index),""); //we found the parent in oldg
        p->parentOf.removeValue(n);   //original parent is not parent of copy
        p = newg->elem(p->index);     //the true parent in the new graph
      }
      p->parentOf.append(n);       //connect both ways
      n->parents(i)=p;
    }
  }

  DEBUG(this->checkConsistency();)
  DEBUG(G.checkConsistency();)
}

void Graph::read(std::istream& is, bool parseInfo) {
  if(parseInfo) getParseInfo(NULL).beg=is.tellg();
  for(;;) {
    DEBUG(checkConsistency();)
    char c=mlr::peerNextChar(is, " \n\r\t,");
    if(!is.good() || c=='}') { is.clear(); break; }
    Node *n = readNode(is, false, parseInfo);
    if(!n) break;
    if(n->keys.N==1 && n->keys.last()=="Include"){
      read(n->get<mlr::FileToken>().getIs(true));
      delete n; n=NULL;
    }else
    if(n->keys.N==1 && n->keys.last()=="ChDir"){
      n->get<mlr::FileToken>().changeDir();
    }else
    if(n->keys.N>0 && n->keys.first()=="Delete"){
      n->keys.remove(0);
      NodeL dels = getNodes(n->keys);
      for(Node* d: dels){ delete d; d=NULL; }
    }
  }
  if(parseInfo) getParseInfo(NULL).end=is.tellg();

  DEBUG(checkConsistency();)

  //-- merge all Merge keys
  NodeL edits = getNodes("Edit");
  for(Node *ed:edits){
    CHECK_EQ(ed->keys.first(), "Edit" , "an edit node needs Edit as first key");
    ed->keys.remove(0);
    edit(ed);
  }

  DEBUG(checkConsistency();)

  //-- delete all ChDir nodes in reverse order
  for(uint i=N;i--;){
    Node *n=elem(i);
    if(n->keys.N==1 && n->keys(0)=="ChDir"){
      n->get<mlr::FileToken>().unchangeDir();
      delete n; n=NULL;
    }
  }
}

void writeFromStream(std::ostream& os, std::istream& is, istream::pos_type beg, istream::pos_type end){
  istream::pos_type here=is.tellg();
  is.seekg(beg);
  char c;
  for(uint i=end-beg;i--;){
    is.get(c);
    os <<c;
  }
  is.seekg(here);
}

#define PARSERR(x, pinfo) { \
  cerr <<"[[error in parsing Graph file (line=" <<mlr::lineCount <<"): " <<x <<":\n  \""; \
  writeFromStream(cerr, is, pinfo.beg, is.tellg()); \
  cerr <<"<<<\"  ]]" <<endl; \
  is.clear(); }

//  if(node) cerr <<"  (node='" <<*node <<"')" <<endl;

Node* Graph::readNode(std::istream& is, bool verbose, bool parseInfo, mlr::String prefixedKey) {
  mlr::String str;
  StringA keys;
  NodeL parents;

  ParseInfo pinfo;
  pinfo.beg=is.tellg();

  if(verbose) { cout <<"\nNODE (line="<<mlr::lineCount <<")"; }

  //-- read keys
  if(!prefixedKey.N){
    mlr::skip(is," \t\n\r");
    pinfo.keys_beg=is.tellg();
    for(;;) {
      if(!str.read(is, " \t", " \t\n\r,;([{}=!", false)) break;
      keys.append(str);
      pinfo.keys_end=is.tellg();
    }
  }else{
    keys.append(prefixedKey);
  }
  DEBUG(checkConsistency();)

  if(verbose) { cout <<" keys:" <<keys <<flush; }

  //-- read parents
  char c=mlr::getNextChar(is," \t"); //don't skip new lines
  if(c=='(') {
    pinfo.parents_beg=is.tellg();
    for(uint j=0;; j++) {
      if(!str.read(is, " \t\n\r,", " \t\n\r,)", false)) break;
      Node *e = this->findNode({str}, true, false); //important: recurse up
      if(e) { //sucessfully found
        parents.append(e);
        pinfo.parents_end=is.tellg();
      } else { //this element is not known!!
        int rel=0;
        str >>rel;
        if(rel<0 && (int)this->N+rel>=0){
          e=elem(this->N+rel);
          parents.append(e);
          pinfo.parents_end=is.tellg();
        }else{
          PARSERR("unknown " <<j <<". parent '" <<str <<"'", pinfo);
          mlr::skip(is, NULL, ")", false);
        }
      }
    }
    mlr::parse(is, ")");
    c=mlr::getNextChar(is," \t");
  }
  DEBUG(checkConsistency();)

  if(verbose) { cout <<" parents:"; if(!parents.N) cout <<"none"; else listWrite(parents,cout," ","()"); cout <<flush; }

  //-- read value
  Node *node=NULL;
  pinfo.value_beg=(long int)is.tellg()-1;
  if(c=='=' || c=='{' || c=='[' || c=='<' || c=='!') {
    if(c=='=') c=mlr::getNextChar(is," \t");
    if((c>='a' && c<='z') || (c>='A' && c<='Z')) { //mlr::String or boolean
      is.putback(c);
      str.read(is, "", " \n\r\t,;}", false);
      if(str=="true") node = newNode<bool>(keys, parents, true);
      else if(str=="false") node = newNode<bool>(keys, parents, false);
      else node = newNode<mlr::String>(keys, parents, str);
    } else if(mlr::contains("-.0123456789", c)) {  //single double
      is.putback(c);
      double d;
      try { is >>d; } catch(...) PARSERR("can't parse the double number", pinfo);
      node = newNode<double>(keys, parents, d);
    } else switch(c) {
      case '!': { //boolean false
        node = newNode<bool>(keys, parents, false);
      } break;
      case '\'': { //mlr::FileToken
        str.read(is, "", "\'", true);
        try{
//          f->getIs();
          node = newNode<mlr::FileToken>(keys, parents, mlr::FileToken(str, false));
          node->get<mlr::FileToken>().getIs();  //creates the ifstream and might throw an error
        } catch(...){
          delete node; node=NULL;
          PARSERR("file " <<str <<" does not exist -> converting to string!", pinfo);
          node = newNode<mlr::String>(keys, parents, str);
//          delete f; f=NULL;
        }
      } break;
      case '\"': { //mlr::String
        str.read(is, "", "\"", true);
        node = newNode<mlr::String>(keys, parents, str);
      } break;
      case '[': { //arr
        is.putback(c);
        arr reals;
        is >>reals;
        node = newNode<arr>(keys, parents, reals);
      } break;
      case '<': { //any type parser
        str.read(is, " \t", " \t\n\r()`-=~!@#$%^&*()+[]{};'\\:|,./<>?", false);
        //      str.read(is, " \t", " \t\n\r()`1234567890-=~!@#$%^&*()_+[]{};'\\:|,./<>?", false);
        node = readTypeIntoNode(*this, str, is);
        if(!node) {
          is.clear();
          mlr::String substr;
          substr.read(is,"",">",false);
//          PARSERR("could not parse value of type '" <<str <<"' -- no such type has been registered; converting this to string: '"<<substr<<"'", pinfo);
          str = STRING('<' <<str <<' ' <<substr <<'>');
          node = newNode<mlr::String>(keys, parents, str);
        } else {
          node->keys = keys;
          node->parents = parents;
        }
        mlr::parse(is, ">");
      } break;
      case '{': { // sub graph
        Node_typed<Graph> *subgraph = this->newSubgraph(keys, parents);
        subgraph->value.read(is);
        mlr::parse(is, "}");
        node = subgraph;
      } break;
//      case '(': { // referring Graph
//        Graph *refs = new Graph;
//        refs->isReferringToNodesOf = this;
//        for(uint j=0;; j++) {
//          str.read(is, " , ", " , )", false);
//          if(!str.N) break;
//          Node *e = this->getNode(str);
//          if(e) { //sucessfully found
//            refs->NodeL::append(e);
//          } else { //this element is not known!!
//            HALT("line:" <<mlr::lineCount <<" reading node '" <<keys <<"': unknown "
//                 <<j <<"th linked element '" <<str <<"'"); //DON'T DO THIS YET
//          }
//        }
//        mlr::parse(is, ")");
//        node = newNode<Graph*>(keys, parents, refs, true);
//      } break;
      default: { //error
        is.putback(c);
        PARSERR("unknown value indicator '" <<c <<"'", pinfo);
        return NULL;
      }
    }
  } else { //no '=' or '{' -> boolean
    is.putback(c);
    node = newNode<bool>(keys, parents, true);
  }
  if(node) pinfo.value_end=is.tellg();
  pinfo.end=is.tellg();
  DEBUG(checkConsistency();)

  if(parseInfo && node) node->container.getParseInfo(node) = pinfo;

  if(verbose) {
    if(node) { cout <<" value:"; node->writeValue(cout); cout <<" FULL:"; node->write(cout); cout <<endl; }
    else { cout <<"FAILED" <<endl; }
  }

  if(!node){
    cerr <<"FAILED reading node with keys ";
    keys.write(cerr, " ", NULL, "()");
    cerr <<" and parents ";
    listWrite(parents,cerr," ","()");
    cerr <<endl;
  }

  //eat the next , or ;
  c=mlr::getNextChar(is," \n\r\t");
  if(c==',' || c==';') {} else is.putback(c);

  return node;
}

#undef PARSERR

void Graph::write(std::ostream& os, const char *ELEMSEP, const char *delim) const {
  if(delim) os <<delim[0];
  for(uint i=0; i<N; i++) { if(i) os <<ELEMSEP;  if(elem(i)) elem(i)->write(os); else os <<"<NULL>"; }
  if(delim) os <<delim[1] <<std::flush;
}

void Graph::writeParseInfo(std::ostream& os) {
  os <<"GRAPH " <<getParseInfo(NULL) <<endl;
  for(Node *n:*this)
    os <<"NODE '" <<*n <<"' " <<getParseInfo(n) <<endl;
}

void Graph::displayDot(Node *highlight){
  if(highlight){
    CHECK(&highlight->container==this,"");
    writeDot(FILE("z.dot"), false, false, 0, highlight->index);
  }else{
    writeDot(FILE("z.dot"), false, false, 0);
  }
  int r;
  r = system("dot -Tpdf z.dot > z.pdf");  if(r) LOG(-1) <<"could not startup dot";
  r = system("evince z.pdf &");  if(r) LOG(-1) <<"could not startup evince";
}

void Graph::writeHtml(std::ostream& os, std::istream& is) {
  char c;
  long int g=getParseInfo(NULL).beg;
  is.seekg(g);
#define GO { is.get(c); if(c=='\n') os <<"<br>" <<endl; else os <<c; g++; }
  for(Node *n:list()){
    ParseInfo& pinfo=getParseInfo(n);
    while(g<pinfo.keys_beg) GO
    os <<"<font color=\"0000ff\">";
    while(g<pinfo.keys_end) GO
    os <<"</font>";
    while(g<pinfo.parents_beg)GO
    os <<"<font color=\"00ff00\">";
    while(g<pinfo.parents_end)GO
    os <<"</font>";
    while(g<pinfo.value_beg)GO
    os <<"<font color=\"ff0000\">";
    while(g<pinfo.value_end)GO
    os <<"</font>";
  }
  while(g<getParseInfo(NULL).end)GO
#undef GO
}

void Graph::writeDot(std::ostream& os, bool withoutHeader, bool defaultEdges, int nodesOrEdges, int focusIndex) {
  if(!withoutHeader){
    os <<"digraph G{" <<endl;
    os <<"graph [ rankdir=\"LR\", ranksep=0.05";
    if(hasRenderingInfo(NULL)) os <<' ' <<getRenderingInfo(NULL).dotstyle;
    os << " ];" <<endl;
    os <<"node [ fontsize=9, width=.3, height=.3 ];" <<endl;
    os <<"edge [ arrowtail=dot, arrowsize=.5, fontsize=6 ];" <<endl;
    index(true);
  }
  for(Node *n: list()) {
    if(hasRenderingInfo(n) && getRenderingInfo(n).skip) continue;
    mlr::String label;
    if(n->keys.N){
      label <<"label=\"";
      bool newline=false;
      for(mlr::String& k:n->keys){
        if(newline) label <<"\\n";
        label <<k;
        newline=true;
      }
      label <<'"';
    }else if(n->parents.N){
      label <<"label=\"(" <<n->parents(0)->keys.last();
      for(uint i=1;i<n->parents.N;i++) label <<' ' <<n->parents(i)->keys.last();
      label <<")\"";
    }

    mlr::String shape;
    if(n->keys.contains("box")) shape <<", shape=box"; else shape <<", shape=ellipse";
    if(focusIndex==(int)n->index) shape <<", color=red";
    if(hasRenderingInfo(n)) shape <<' ' <<getRenderingInfo(n).dotstyle;


    if(defaultEdges && n->parents.N==2){ //an edge
      os <<n->parents(0)->index <<" -> " <<n->parents(1)->index <<" [ " <<label <<"];" <<endl;
    }else{
      if(n->isGraph()){
        os <<"subgraph cluster_" <<n->index <<" { " <<label /*<<" rank=same"*/ <<endl;
        n->graph().writeDot(os, true, defaultEdges, +1);
        os <<"}" <<endl;
        n->graph().writeDot(os, true, defaultEdges, -1);
      }else{//normal node
        if(nodesOrEdges>=0){
          os <<n->index <<" [ " <<label <<shape <<" ];" <<endl;
        }
        if(nodesOrEdges<=0){
          for_list(Node, pa, n->parents) {
            if(hasRenderingInfo(pa) && getRenderingInfo(pa).skip) continue;
            if(pa->index<n->index)
              os <<pa->index <<" -> " <<n->index <<" [ ";
            else
              os <<n->index <<" -> " <<pa->index <<" [ ";
            os <<"label=" <<pa_COUNT;
            os <<" ];" <<endl;
          }
        }
      }
    }
  }
  if(!withoutHeader){
    os <<"}" <<endl;
    index(false);
  }
}

void Graph::sortByDotOrder() {
  uintA perm;
  perm.setStraightPerm(N);
  for_list(Node, it, list()) {
    if(it->isGraph()) {
      double *order = it->graph().find<double>("dot_order");
      if(!order) { MLR_MSG("doesn't have dot_order attribute"); return; }
      perm(it_COUNT) = (uint)*order;
    }
  }
  permuteInv(perm);
  for_list(Node, it2, list()) it2->index=it2_COUNT;
}

ParseInfo& Graph::getParseInfo(Node* n){
  if(!pi) pi=new ArrayG<ParseInfo>(*this);
  return pi->operator ()(n);
//  if(pi.N!=N+1){
//    listResizeCopy(pi, N+1);
//    pi(0)->node=NULL;
//    for(uint i=1;i<pi.N;i++) pi(i)->node=elem(i-1);
//  }
//  if(!n) return *pi(0);
//  return *pi(n->index+1);
}

RenderingInfo& Graph::getRenderingInfo(Node* n){
  CHECK(!n || &n->container==this,"");
#if 1
  if(!ri) ri=new ArrayG<RenderingInfo>(*this);
  return ri->operator()(n);
#else
  if(ri.N!=N+1){
    ri.resizeCopy(N+1); //listResizeCopy(ri, N+1);
//    ri.elem(0)->node=NULL;
//    for(uint i=1;i<ri.N;i++) ri.elem(i)->node=elem(i-1);
  }
  if(!n) return ri.elem(0);
  return ri.elem(n->index+1);
#endif
}

const Graph* Graph::getRootGraph() const{
  const Graph* g=this;
  for(;;){
    const Node* n=g->isNodeOfGraph;
    if(!n) break;
    g = &n->container;
  }
  return g;
}

bool Graph::isChildOfGraph(const Graph& G) const{
  const Graph* g=this;
  for(;;){
    const Node* n=g->isNodeOfGraph;
    if(!n) break;
    g = &n->container;
    if(g==&G) return true;
  }
  return false;
}

bool Graph::checkConsistency() const{
  uint idx=0;
  for(Node *node: *this){
    CHECK_EQ(&node->container, this, "");
    CHECK_EQ(node->index, idx, "");
    for(Node *j: node->parents)  CHECK(j->parentOf.findValue(node) != -1,"");
    for(Node *j: node->parentOf) CHECK(j->parents.findValue(node) != -1,"");
    for(Node *parent: node->parents) if(&parent->container!=this){
      //check that parent is contained in a super-graph of this
      const Graph *parentGraph = this;
      const Node *parentGraphNode;
      while(&parent->container!=parentGraph){
        //we need to descend one more
        parentGraphNode = parentGraph->isNodeOfGraph;
        CHECK(parentGraphNode,"there is no more supergraph to find the parent");
        parentGraph = &parentGraphNode->container;
      }
      //check sorting
//      CHECK(parent->index < parentGraphNode->index,"subnode refers to parent that sorts below the subgraph");
    }else{
//      CHECK(parent->index < node->index,"node refers to parent that sorts below the node");
    }
    if(node->isGraph()){
      Graph& G = node->graph();
      CHECK_EQ(G.isNodeOfGraph, node, "");
      G.checkConsistency();
    }
    idx++;
  }
  return true;
}

uint Graph::index(bool subKVG, uint start){
  uint idx=start;
  for(Node *it: list()){
    it->index=idx;
    idx++;
    if(it->isGraph()){
      Graph& G=it->graph();
      if(subKVG) idx = G.index(true, idx);
      else G.index(false, 0);
    }
  }
  return idx;
}

bool operator==(const Graph& A, const Graph& B){
  if(A.N!=B.N) return false;
  for(uint i=0;i<A.N;i++){
    Node *a = A(i), *b = B(i);
    if(a->index!=b->index) return false;
    if(a->keys!=b->keys) return false;
    if(a->parents.N!=b->parents.N) return false;
    for(uint j=0;j<a->parents.N;j++) if(a->parents(j)->index!=b->parents(j)->index) return false;
    if(a->type!=b->type) return false;
    if(!a->hasEqualValue(b)) return false;
  }
  return true;
}

//===========================================================================

NodeL neighbors(Node* it){
  NodeL N;
  for(Node *e:it->parentOf){
    for(Node *n:e->parents) if(n!=it) N.setAppend(n);
  }
  return N;
}

//===========================================================================
//
// global singleton TypeRegistrationSpace
//

Singleton<Graph> registry;

struct RegistryInitializer{
  Mutex lock;
  RegistryInitializer(){
    int n;
    for(n=1; n<mlr::argc; n++){
      if(mlr::argv[n][0]=='-'){
        mlr::String key(mlr::argv[n]+1);
        if(n+1<mlr::argc && mlr::argv[n+1][0]!='-'){
          mlr::String value;
          value <<'=' <<mlr::argv[n+1];
          registry()->readNode(value, false, false, key);
          n++;
        }else{
          registry()->newNode<bool>({key}, {}, true);
        }
      }else{
        MLR_MSG("non-parsed cmd line argument:" <<mlr::argv[n]);
      }
    }

    mlr::String cfgFileName="MT.cfg";
    if(registry()()["cfg"]) cfgFileName = registry()->get<mlr::String>("cfg");
    LOG(3) <<"opening config file '" <<cfgFileName <<"'";
    ifstream fil;
    fil.open(cfgFileName);
    if(fil.good()){
      fil >>registry();
    }else{
      LOG(3) <<" - failed";
    }

  }
  ~RegistryInitializer(){
  }
};

Singleton<RegistryInitializer> registryInitializer;

bool getParameterFromGraph(const std::type_info& type, void* data, const char* key){
  registryInitializer()();
  Node *n = registry()->findNodeOfType(type, {key});
  if(n){
    n->copyValueInto(data);
    return true;
  }else{
    n = registry()->findNode({key});
    if(n && n->isOfType<double>()){
      if(type==typeid(int)){ *((int*)data) = (int)n->get<double>(); return true; }
      if(type==typeid(uint)){ *((uint*)data) = (uint)n->get<double>(); return true; }
      if(type==typeid(bool)){ *((bool*)data) = (bool)n->get<double>(); return true; }
    }
    if(n && n->isOfType<mlr::String>()){
      NIY;
//      n->get<mlr::String>() >>x;
    }
  }
  return false;
}

 //===========================================================================

RUN_ON_INIT_BEGIN(graph)
NodeL::memMove=true;
GraphEditCallbackL::memMove=true;
RUN_ON_INIT_END(graph)
/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#include "taskMap_FixSwitchedObjects.h"
#include "taskMap_qItself.h"
#include "taskMap_default.h"

uint TaskMap_FixSwichedObjects::dim_phi(const WorldL& G, int t){
  mlr::Array<mlr::Body*> switchedBodies = getSwitchedBodies(*G.elem(-2), *G.elem(-1));
  return switchedBodies.d0*7;
}

void TaskMap_FixSwichedObjects::phi(arr& y, arr& J, const WorldL& G, double tau, int t){
  //TODO: so far this only fixes switched objects to zero pose vel
  //better: constrain to zero relative velocity with BOTH, pre-attached and post-attached
  CHECK(order==1,"");

  uint M=7;
  mlr::Array<mlr::Body*> switchedBodies = getSwitchedBodies(*G.elem(-2), *G.elem(-1));
  y.resize(M*switchedBodies.d0).setZero();
  if(&J){
    uint xbarDim=0;
    for(auto& W:G) xbarDim+=W->q.N;
    J.resize(M*switchedBodies.d0, xbarDim).setZero();
  }
  for(uint i=0;i<switchedBodies .d0;i++){
    mlr::Body *b0 = switchedBodies(i,0);    CHECK(&b0->world==G.elem(-2),"");
    mlr::Body *b1 = switchedBodies(i,1);    CHECK(&b1->world==G.elem(-1),"");
    CHECK(b0->index == b1->index, "");
    CHECK(b0->shapes.first()->index == b1->shapes.first()->index, "");

    if(b0->name.startsWith("slider")) continue; //warning: this introduces zeros in y and J -- but should be ok

#if 1 //absolute velocities
    TaskMap_Default pos(posTMT, b0->shapes.first()->index);
    pos.order=1;
    pos.TaskMap::phi(y({M*i,M*i+2})(), (&J?J({M*i,M*i+2})():NoArr), G, tau, t);

    TaskMap_Default quat(quatTMT, b0->shapes.first()->index); //mt: NOT quatDiffTMT!! (this would compute the diff to world, which zeros the w=1...)
    // flip the quaternion sign if necessary
    quat.flipTargetSignOnNegScalarProduct = true;
    quat.order=1;
    quat.TaskMap::phi(y({M*i+3,M*i+6})(), (&J?J({M*i+3,M*i+6})():NoArr), G, tau, t);

//    if(sumOfSqr(y)>1e-3) cout <<"body " <<b0->name <<" causes switch costs " <<sumOfSqr(y) <<" at t=" <<t <<" y=" <<y <<endl;
#else //relative velocities
    TaskMap_Default pos(posDiffTMT, b0->shapes.first()->index, NoVector, b1->shapes.first()->index);
    pos.order=1;
    pos.TaskMap::phi(y({M*i,M*i+2})(), (&J?J({M*i,M*i+2})():NoArr), G, tau, t);

    TaskMap_Default quat(quatDiffTMT, j0->to->shapes.first()->index/*, NoVector, j0->from->shapes.first()->index*/);
    // flipp the quaternion sign if necessary
    quat.flipTargetSignOnNegScalarProduct = true;
    quat.order=1;
    quat.TaskMap::phi(y({M*i+3,M*i+6})(), (&J?J({M*i+3,M*i+6})():NoArr), G, tau, t);
#endif
  }
}





#if 0

(mt:) These are remains for debugging: testing the Jacobians (I used this to find the top sort error...)

    if(&J && t==3){
        cout <<"switched bodies:" <<b0->name <<' ' <<b1->name <<endl;
//        analyzeJointStateDimensions();
        //-- clean up the graph
//        G.elem(-1)->analyzeJointStateDimensions();
//        G.elem(-1)->checkConsistency();
//        G.elem(-1)->topSort();
//        G.elem(-1)->jointSort();
//        G.elem(-1)->calc_missingAB_from_BodyAndJointFrames();
//        G.elem(-1)->analyzeJointStateDimensions();
//        G.elem(-1)->calc_q_from_Q();
//        G.elem(-1)->calc_fwdPropagateFrames();

        arr yt,Jt;
        pos.phi(yt, Jt, *G.last(), t);

        G.elem(-1)->checkConsistency();
        G.elem(-2)->checkConsistency();
        FILE("z.last") <<*G.elem(-1);
        cout <<"\n*** A ***\n" <<J*sqrt(1000.) <<endl;
        cout <<"\n*** C ***\n" <<Jt*sqrt(1000.) <<endl;

        if(true){
//          const char* filename="z.last";
          const char* shape="obj1";
          mlr::KinematicWorld K(*G.elem(-1));
          FILE("z.last2") <<K;
//          mlr::KinematicWorld K(filename);
//          K.setJointState(G.elem(-1)->q);
          mlr::Shape *sh=K.getShapeByName(shape);
          TaskMap_Default pos(posDiffTMT, sh->index);
          arr y,J;
          pos.phi(y, J, K);
          cout <<"\n*** B ***\n" <<J*sqrt(1000.) <<endl;


          VectorFunction f = ( [&pos, &K](arr& y, arr& J, const arr& x) -> void
          {
              K.setJointState(x);
                  pos.phi(y,J,K);
          } );

          checkJacobian(f, K.q, 1e-4);

//          exit(0);
//          mlr::wait();
        }

    }
#endif
/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#include "taskMap_AlignStacking.h"

TaskMap_AlignStacking::TaskMap_AlignStacking(int iShape)
  : i(iShape){
}


TaskMap_AlignStacking::TaskMap_AlignStacking(const mlr::KinematicWorld& G, const char* iShapeName)
  :i(-1){
  mlr::Shape *a = iShapeName ? G.getShapeByName(iShapeName):NULL;
  if(a) i=a->index;
}

void TaskMap_AlignStacking::phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t){
  mlr::Shape *s=G.shapes(i);
  mlr::Body *b=s->body;

  mlr::Joint *j=b->inLinks.first();
  CHECK(j,"has no support??");

  mlr::Body *b_support=j->from;

#if 0//if there were multiple supporters
  uint n=G.getJointStateDimension();
  //-- compute the center of all supporters (here just one..)
  arr cen = zeros(3), cenJ = zeros(3,n);
  {
    arr y,J;
    G.kinematicsPos(y, J, b_support);
    cen += y;
    cenJ += J;
  }
  cen  /= (double)supporters.N;
  cenJ /= (double)supporters.N;

  //-- max distances to center
  prec=3e-1;
  for(Node *s:supporters){
    b=effKinematics.getBodyByName(s->keys.last());
    effKinematics.kinematicsPos(y, J, b);
    y -= cen;
    double d = length(y);
    arr normal = y/d;
    phi.append( prec*(1.-d) );
    if(&phiJ) phiJ.append( prec*(~normal*(-J+cenJ)) );
    if(&tt) tt.append(OT_sumOfSqr, 1);
  }

  //-- align center with object center
  prec=1e-1;
  b=effKinematics.getBodyByName(obj->keys.last());
  effKinematics.kinematicsPos(y, J, b);
  phi.append( prec*(y-cen) );
  if(&phiJ) phiJ.append( prec*(J-cenJ) );
  if(&tt) tt.append(OT_sumOfSqr, 3);
#else //just one supporter

  arr y1,J1,y2,J2;
//    if(verbose>1){ cout <<"Adding cost term Object" <<*obj <<" below "; listWrite(supporters, cout); cout <<endl; }
  G.kinematicsPos(y1, J1, b);
  G.kinematicsPos(y2, J2, b_support);
  y = (y1-y2)({0,1}); //only x-y-position
  if(&J) J = (J1-J2)({0,1});

#endif
}
/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */

#include "taskMap_proxy.h"

TaskMap_Proxy::TaskMap_Proxy(PTMtype _type,
                           uintA _shapes,
                           double _margin,
                           bool _useCenterDist,
                           bool _useDistNotCost) {
  type=_type;
  shapes=_shapes;
  margin=_margin;
  useCenterDist=_useCenterDist;
  useDistNotCost=_useDistNotCost;
  cout <<"creating TaskMap_Proxy with shape list" <<shapes <<endl;
}

void TaskMap_Proxy::phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t){
  uintA shapes_t;
  shapes_t.referTo(shapes);

  y.resize(1).setZero();
  if(&J) J.resize(1, G.getJointStateDimension(false)).setZero();

  switch(type) {
    case allPTMT:
      for(mlr::Proxy *p: G.proxies)  if(p->d<margin) {
        G.kinematicsProxyCost(y, J, p, margin, useCenterDist, true);
        p->colorCode = 1;
      }
//      cout <<"allPTMT=" <<y.scalar() <<endl;
//      G.reportProxies();
      break;
    case listedVsListedPTMT:
      for(mlr::Proxy *p: G.proxies)  if(p->d<margin) {
        if(shapes.contains(p->a) && shapes.contains(p->b)) {
          G.kinematicsProxyCost(y, J, p, margin, useCenterDist, true);
          p->colorCode = 2;
        }
      }
      break;
    case allVsListedPTMT: {
      if(t && shapes.nd==2) shapes_t.referToDim(shapes,t);
      for(mlr::Proxy *p: G.proxies)  if(p->d<margin) {
        if(shapes_t.contains(p->a) || shapes_t.contains(p->b)) {
          G.kinematicsProxyCost(y, J, p, margin, useCenterDist, true);
          p->colorCode = 2;
        }
      }
    } break;
    case allExceptListedPTMT:
      for(mlr::Proxy *p: G.proxies)  if(p->d<margin) {
        if(!(shapes.contains(p->a) && shapes.contains(p->b))) {
          G.kinematicsProxyCost(y, J, p, margin, useCenterDist, true);
          p->colorCode = 3;
        }
      }
      break;
    case bipartitePTMT:
      for(mlr::Proxy *p: G.proxies)  if(p->d<margin) {
        if((shapes.contains(p->a) && shapes2.contains(p->b)) ||
            (shapes.contains(p->b) && shapes2.contains(p->a))) {
          G.kinematicsProxyCost(y, J, p, margin, useCenterDist, true);
          p->colorCode = 4;
        }
      }
      break;
    case pairsPTMT: {
      shapes.reshape(shapes.N/2,2);
      // only explicit paris in 2D array shapes
      uint j;
      for(mlr::Proxy *p: G.proxies) if(p->d<margin) {
        for(j=0; j<shapes.d0; j++) {
          if((shapes(j,0)==(uint)p->a && shapes(j,1)==(uint)p->b) || (shapes(j,0)==(uint)p->b && shapes(j,1)==(uint)p->a))
            break;
        }
        if(j<shapes.d0) { //if a pair was found
          if(useDistNotCost) G.kinematicsProxyDist(y, J, p, margin, useCenterDist, true);
          else G.kinematicsProxyCost(y, J, p, margin, useCenterDist, true);
          p->colorCode = 5;
        }
      }
    } break;
    case allExceptPairsPTMT: {
      shapes.reshape(shapes.N/2,2);
      // only explicit paris in 2D array shapes
      uint j;
      for(mlr::Proxy *p: G.proxies)  if(p->d<margin) {
        for(j=0; j<shapes.d0; j++) {
          if((shapes(j,0)==(uint)p->a && shapes(j,1)==(uint)p->b) || (shapes(j,0)==(uint)p->b && shapes(j,1)==(uint)p->a))
            break;
        }
        if(j==shapes.d0) { //if a pair was not found
          G.kinematicsProxyCost(y, J, p, margin, useCenterDist, true);
          p->colorCode = 5;
        }
      }
    } break;
    case vectorPTMT: {
      //outputs a vector of collision meassures, with entry for each explicit pair
      shapes.reshape(shapes.N/2,2);
      y.resize(shapes.d0, 1);  y.setZero();
      if(&J){ J.resize(shapes.d0,J.d1);  J.setZero(); }
      uint j;
      for(mlr::Proxy *p: G.proxies)  if(p->d<margin) {
        for(j=0; j<shapes.d0; j++) {
          if((shapes(j,0)==(uint)p->a && shapes(j,1)==(uint)p->b) || (shapes(j,0)==(uint)p->b && shapes(j,1)==(uint)p->a))
            break;
        }
        if(j<shapes.d0) {
          G.kinematicsProxyCost(y[j](), (&J?J[j]():NoArr), p, margin, useCenterDist, true);
          p->colorCode = 5;
        }
      }
      y.reshape(shapes.d0);
    } break;
    default: NIY;
  }
}

uint TaskMap_Proxy::dim_phi(const mlr::KinematicWorld& G){
  switch(type) {
  case allPTMT:
  case listedVsListedPTMT:
  case allVsListedPTMT:
  case allExceptListedPTMT:
  case bipartitePTMT:
  case pairsPTMT:
  case allExceptPairsPTMT:
    return 1;
  case vectorPTMT:
    return shapes.d0;
  default: NIY;
  }
}

//===========================================================================


TaskMap_ProxyConstraint::TaskMap_ProxyConstraint(PTMtype _type,
                                 uintA _shapes,
                                 double _margin,
                                 bool _useCenterDist,
                                 bool _useDistNotCost)
  : proxyCosts(_type, _shapes, _margin, _useCenterDist, _useDistNotCost){
}

void TaskMap_ProxyConstraint::phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t){
  proxyCosts.phi(y, J, G, t);
  y -= .5;
}

/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


/**
 * @file
 * @ingroup group_ors
 */
/**
 * @ingroup group_ors
 * @{
 */



#include "kin_ode.h"

#ifdef MLR_ODE

#ifndef dDOUBLE
#  define dDOUBLE
#endif


#  include <ode/ode.h>
#  include <ode/internal/objects.h>
#  include <ode/internal/joints/joints.h>
#  include <ode/internal/collision_kernel.h>
#  include <ode/internal/collision_transform.h>

#  ifdef MLR_MSVC
#    undef HAVE_UNISTD_H
#    undef HAVE_SYS_TIME_H
#  endif

#define OUTs(x) x <<' '
#define OUTv(x) '(' <<OUTs(x[0]) <<OUTs(x[1]) <<OUTs(x[2]) <<')'
#define OUTq(x) OUTs(x[0]) <<OUTs(x[1]) <<OUTs(x[2]) <<OUTs(x[3])
#define CP4(x, y) memmove(x, y, 4*sizeof(double));
#define CP3(x, y) memmove(x, y, 3*sizeof(double));

static bool ODEinitialized=false;


//===========================================================================
//
// Ode implementations
//

OdeInterface::OdeInterface(mlr::KinematicWorld &_C):C(_C) {
  time=0.;
  
  noGravity=noContactJoints=false;
  
  ERP= 0.2;     //in [0, 1]: rate of error correction (makes more brittle) [default=.2]
  CFM=1e-5;  // >0: softness (makes more robust) [default=1e-10]
  
  coll_bounce = .0;
  coll_ERP = 0.2;   //usually .2!! stiffness (time-scale of contact reaction)
  coll_CFM = 1e-5;  //softness
  friction = 0.1;   //alternative: dInfinity;
  
  world=NULL;
  space=NULL;
  contactgroup=0;
  
  if(!ODEinitialized) {  dInitODE();  ODEinitialized=true; }
  clear();

  dBodyID b;
  dMass odeMass;
  dGeomID geom, trans;
  dSpaceID myspace;
  dxJointHinge* jointH=0;
  dxJointSlider* jointS=0;
  dxJointUniversal* jointU=0;
  dxJointFixed* jointF=0;
  dxJointAMotor* jointM=0;
  dJointFeedback* jointFB=0;
  mlr::Vector a;
  //double *mass, *shape, *type, *fixed, *cont, typeD=ST_capsule;
  //, *inertiaTensor, *realMass, *centerOfMass;

  clear();

  bodies.resize(C.bodies.N); bodies=0;
  geoms .resize(C.shapes.N); geoms =0;
  joints.resize(C.joints.N); joints=0;
  motors.resize(C.joints.N); motors=0;

  for_list(mlr::Body,  n,  C.bodies) {
    b=dBodyCreate(world);

    bodies(n->index)=b;
    b->userdata=n;

    //n->copyFrameToOde();
    CHECK(n->X.rot.isNormalized(), "quaternion is not normalized!");
    CP3(b->posr.pos, n->X.pos.p());                // dxBody changed in ode-0.6 ! 14. Jun 06 (hh)
    CP4(b->q, n->X.rot.p()); dQtoR(b->q, b->posr.R);
    CP3(b->lvel, n->X.vel.p());
    CP3(b->avel, n->X.angvel.p());

    // need to fix: "mass" is not a mass but a density (in dMassSetBox)
    for_list(mlr::Shape,  s,  n->shapes) {
      if(!(s->rel.rot.isZero) || !(s->rel.pos.isZero)) { //we need a relative transformation
        trans = dCreateGeomTransform(space);
        myspace = NULL; //the object is added to no space, but (below) associated with the transform
      } else {
        trans = NULL;
        myspace = space; //the object is added normally to the main space
      }

      switch(s->type) {
        default:
          for(int i=0; i<3; ++i) {
            if(s->size[i] == 0) s->size[i] = 0.001;
          }
        case mlr::ST_box:
          dMassSetBox(&odeMass, n->mass, s->size(0), s->size(1), s->size(2));
          dBodySetMass(b, &odeMass);
          geom=dCreateBox(myspace, s->size(0), s->size(1), s->size(2));
          break;
        case mlr::ST_sphere:
          dMassSetSphere(&odeMass, n->mass, s->size(3));
          dBodySetMass(b, &odeMass);
          geom=dCreateSphere(myspace, s->size(3));
          break;
        case mlr::ST_cylinder:
          dMassSetCylinder(&odeMass, n->mass, 3, s->size(3), s->size(2));
          dBodySetMass(b, &odeMass);
          geom=dCreateCylinder(myspace, s->size(3), s->size(2));
          break;
        case mlr::ST_capsule:
          dMassSetCylinder(&odeMass, n->mass, 3, s->size(3), s->size(2));
          //                 MLR_MSG("ODE: setting Cylinder instead of capped cylinder mass");
          dBodySetMass(b, &odeMass);
          geom=dCreateCCylinder(myspace, s->size(3), s->size(2));
          break;
        case mlr::ST_mesh: {
#if 0
          NIY;
#else
          s->mesh.computeNormals();
          // get inertia tensor and REAL mass (careful no density)
          //n->ats.get("I", inertiaTensor, 9);
          //n->ats.get("w", realMass, 1);
          //n->ats.get("X", centerOfMass, 3);

          // transform the mesh to ODE trimesh format;
          //i=0; j=0;

#if 0 //correct mass/density stuff
          trimeshPhysics triPhys;
          triPhys.reset(s->mesh.T.N);
          if(inertiaTensor && realMass && centerOfMass) { // are all important params set in the dcg file
            triPhys._mass = *realMass;
            triPhys.r[0]=centerOfMass[0];
            triPhys.r[1]=centerOfMass[1];
            triPhys.r[2]=centerOfMass[2];
            triPhys.J[0][0]= inertiaTensor[0];
            triPhys.J[1][1]= inertiaTensor[4];
            triPhys.J[2][2]= inertiaTensor[8];
            triPhys.J[0][1]= inertiaTensor[1];
            triPhys.J[0][2]= inertiaTensor[2];
            triPhys.J[1][2]= inertiaTensor[5];
          } else { // not all parametrs specified in dcg file....need to calculate them
            triPhys.calculateODEparams(&s->mesh, s->mass); // note: 2nd parameter is the density
          }

          dMassSetParameters(&odeMass, triPhys._mass,
                             triPhys.r[0], triPhys.r[1], triPhys.r[2],
                             triPhys.J[0][0], triPhys.J[1][1], triPhys.J[2][2],
                             triPhys.J[0][1], triPhys.J[0][2], triPhys.J[1][2]);

          dBodySetMass(b, &odeMass);
#else //don't care about mass...
          n->mass = .001;
          dMassSetBox(&odeMass, n->mass, s->size(0), s->size(1), s->size(2));
          dBodySetMass(b, &odeMass);
#endif

          dTriMeshDataID TriData;
          TriData = dGeomTriMeshDataCreate();
          dGeomTriMeshDataBuildDouble(TriData,
                                      s->mesh.V.p, 3*sizeof(double), s->mesh.V.d0,
                                      s->mesh.T.p, s->mesh.T.d0, 3*sizeof(uint));
          dGeomTriMeshDataPreprocess(TriData);

          geom = dCreateTriMesh(myspace, TriData, 0, 0, 0);

          dGeomTriMeshClearTCCache(geom);
#endif
        } break; //end of mesh
      }

      geoms(s->index) = geom;
      if(trans) {
        //geoms(s->index) = trans;
        dGeomTransformSetGeom(trans, geom);
        dGeomSetPosition(geom, s->rel.pos.x, s->rel.pos.y, s->rel.pos.z);
        dGeomSetQuaternion(geom, s->rel.rot.p());
        dGeomSetBody(trans, b); //attaches the geom to the body
      } else {
        dGeomSetBody(geom, b); //attaches the geom to the body
      }
    }//loop through shapes

    if(n->type==mlr::BT_static) {
      jointF=(dxJointFixed*)dJointCreateFixed(world, 0);
      dJointAttach(jointF, b, 0);
      dJointSetFixed(jointF);
    }
  }
#ifndef MLR_ode_nojoints
  for(mlr::Body *n: C.bodies) {
    for_list(mlr::Joint,  e,  n->inLinks) {
      switch(e->type) {
        case mlr::JT_rigid:
          jointF=(dxJointFixed*)dJointCreateFixed(world, 0);
          dJointAttach(jointF, bodies(e->from->index), bodies(e->to->index));
          dJointSetFixed(jointF);
          joints(e->index)=jointF;
          //    e->fixed=true;
          break;
        case mlr::JT_hingeX:
          jointH=(dxJointHinge*)dJointCreateHinge(world, 0);
          /*if(e->p[1]!=e->p[0]){
            dJointSetHingeParam(jointH, dParamLoStop, e->p[0]);
            dJointSetHingeParam(jointH, dParamHiStop, e->p[1]);

            //dJointSetHingeParam(jointH, dParamCFM, CFM);
            }*/
          dJointAttach(jointH, bodies(e->from->index), bodies(e->to->index));
          joints(e->index)=jointH;
          //e->copyFramesToOdeHinge();
          break;
        case mlr::JT_universal:
          jointU=(dxJointUniversal*)dJointCreateUniversal(world, 0);
          dJointAttach(jointU, bodies(e->from->index), bodies(e->to->index));
          joints(e->index)=jointU;
          //e->copyFramesToOdeUniversal();
          break;
        case mlr::JT_transX:
          jointS=(dxJointSlider*)dJointCreateSlider(world, 0);
          dJointAttach(jointS, bodies(e->from->index), bodies(e->to->index));
          joints(e->index)=jointS;
          //e->copyFramesToOdeSlider();
          break;
        default: NIY;
      }
      if(e->type==mlr::JT_hingeX) {
        jointM = (dxJointAMotor*)dJointCreateAMotor(world, 0);
        dJointSetAMotorNumAxes(jointM, 1);
        dJointAttach(jointM, bodies(e->from->index), bodies(e->to->index));
        motors(e->index)=jointM;
        a=e->from->X.rot*e->A.rot*mlr::Vector(1, 0, 0);
        dJointSetAMotorAxis(jointM, 0, 1, a.x, a.y, a.z);
        //dJointSetAMotorParam(jointM, dParamFMax, 1.);
        jointFB = new dJointFeedback;
        dJointSetFeedback(jointM, jointFB);
      }
    }
  }
#endif

  exportStateToOde();
}

OdeInterface::~OdeInterface() {
  if(contactgroup) dJointGroupDestroy(contactgroup);
  if(space) dSpaceDestroy(space);
  if(world) dWorldDestroy(world);
}

void OdeInterface::clear() {
  if(contactgroup) dJointGroupDestroy(contactgroup);
  if(space) dSpaceDestroy(space);
  if(world) dWorldDestroy(world);
  
  world=dWorldCreate();
  space=dSimpleSpaceCreate(0);
  contactgroup=dJointGroupCreate(0);
  //std::cout <<"default ERP=" <<dWorldGetERP(world) <<"default CFM=" <<dWorldGetCFM(world) <<std::endl;
  if(noGravity) {
    dWorldSetGravity(world, 0, 0, 0);
  } else {
    dWorldSetGravity(world, 0, 0, -9.81);
  }
  dWorldSetCFM(world, CFM);
  dWorldSetERP(world, ERP);
  dWorldSetContactSurfaceLayer(world, .0); //necessary penetration depth to trigger forces..
  double DAMPING_LINEAR_SCALE = 0.03;
  double DAMPING_ANGULAR_SCALE = 0.03;
  dWorldSetDamping(world, DAMPING_LINEAR_SCALE, DAMPING_ANGULAR_SCALE);
  
  plane0=dCreatePlane(space, 0, 0, 1, 0);
  
  // removed vertical planes,  5. Mar 06 (hh)
  //planex1=dCreatePlane(space, 1, 0, 0, -10);
  //planex2=dCreatePlane(space, -1, 0, 0, -10);
  //planey1=dCreatePlane(space, 0, 1, 0, -10);
  //planey2=dCreatePlane(space, 0, -1, 0, -10);
}

void OdeInterface::staticCallback(void *classP, dGeomID g1, dGeomID g2) {
  static dContact contacts[100]; // array with maximum number of contacts
  uint i, n;
  dJointID c;
  
  dBodyID b1 = dGeomGetBody(g1);
  dBodyID b2 = dGeomGetBody(g2);
  
  mlr::Body *db1 = b1?(mlr::Body*)b1->userdata:0;
  mlr::Body *db2 = b2?(mlr::Body*)b2->userdata:0;
  
  //-- rule out irrelevant contacts
  // exit without doing anything if the geoms stem from the same body
  if(b1==b2) return;
  
  // exit without doing anything if the two bodies are connected by a joint
  if(b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeSlider)) return;
  
  // exit if fixed body intersects with earth,  4. Mar 06 (hh)
  if(b1==0 && db2->type==mlr::BT_static) return;
  if(b2==0 && db1->type==mlr::BT_static) return;
  
  // exit if we have two fixed bodies,  6. Mar 06 (hh)
  if(db1 && db2 && db1->type==mlr::BT_static && db2->type==mlr::BT_static) return;
  
  // exit if none of the bodies have cont enabled (mt)
  //if(db1 && !db1->cont && db2 && !db2->cont) return;
  if(db1 && db2 && (!db1->shapes(0)->cont || !db2->shapes(0)->cont)) return;
  
  //-- ok, now compute the contact exactly
  // get contacts:
  n=dCollide(g1, g2, 100, &(contacts[0].geom), sizeof(dContact));
  
  if(!n) return;
  
  for(i=0; i<n; i++)((OdeInterface*)classP)->conts.append(&contacts[i].geom);
  
  if(((OdeInterface*)classP)->noContactJoints) return;
  
  for(i=0; i<n; i++) {
    contacts[i].surface.mode = dContactSoftCFM | dContactSoftERP | dContactBounce;
    //if(b1 && b2)
    contacts[i].surface.mu   = ((OdeInterface*)classP)->friction; //dInfinity; //friction
    //else contacts[i].surface.mu  = .01; //dInfinity; //friction
    contacts[i].surface.bounce   = ((OdeInterface*)classP)->coll_bounce;
    contacts[i].surface.soft_erp = ((OdeInterface*)classP)->coll_ERP;   //usually .2!! stiffness (time-scale of contact reaction)
    contacts[i].surface.soft_cfm = ((OdeInterface*)classP)->coll_CFM;  //softness
    
    if(!db1 || !db2 || (db1->shapes(0)->cont && db2->shapes(0)->cont)) {
      //normal contact:
      c=dJointCreateContact(((OdeInterface*)classP)->world, ((OdeInterface*)classP)->contactgroup, &contacts[i]);
      dJointAttach(c, b1, b2);
    } else {
      //special contact:
      //one of them ignores the contact
      //hence we consider it (kinematically) coupled to the background
      //.. we simulated relative motion for(parallel) friction
      mlr::Vector no; no.set(contacts[i].geom.normal);
      mlr::Vector d1; d1=db1->X.vel; d1.makeNormal(no); //bug: we should have sorted b1 and b2 before such that b1!=0
      double v=d1.length();
      if(v>1e-4) {
        d1 /= v;
        CP3(contacts[i].fdir1, d1.p());
        contacts[i].surface.mode = contacts[i].surface.mode | dContactMotion1 | dContactFDir1;
        contacts[i].surface.motion1 = v;
      }
      c=dJointCreateContact(((OdeInterface*)classP)->world, ((OdeInterface*)classP)->contactgroup, &contacts[i]);
      if(!db1->shapes(0)->cont) {
        dJointAttach(c, 0, b2);
      }
      if(!db2->shapes(0)->cont) {
        dJointAttach(c, b1, 0);
      }
    }
  }
}

void OdeInterface::setForceFree(bool free) {
  noGravity=free;
  if(free) {
    dWorldSetGravity(world, 0, 0, 0);
    dWorldSetCFM(world, CFM);
    dWorldSetERP(world, ERP);
  } else {
    dWorldSetGravity(world, 0, 0, -9.81);
    dWorldSetCFM(world, CFM);
    dWorldSetERP(world, ERP);
  }
}

void OdeInterface::step(double dtime) {
  conts.clear();
  dSpaceCollide(space, this, &staticCallback);
  //if(noContactJoints) contactForces();
  //dWorldStep(world, dtime);
  //dWorldQuickStep(world, dtime);
  dWorldStep(world, dtime);
  dJointGroupEmpty(contactgroup);
  time+=dtime;
}

void OdeInterface::reportContacts() {
  std::cout <<"contacts: " <<conts.N <<std::endl;
  dContactGeom *c;
  for(uint i=0; i<conts.N; i++) {
    c = conts(i);
    dBodyID b1 = dGeomGetBody(c->g1);
    dBodyID b2 = dGeomGetBody(c->g2);
    mlr::Body *db1 = b1?(mlr::Body*)b1->userdata:0;
    mlr::Body *db2 = b2?(mlr::Body*)b2->userdata:0;
    std::cout
        <<i <<": " <<(b1?db1->name.p:"GROUND") <<'-' <<(b2?db2->name.p:"GROUND")
        <<"\nposition=" <<OUTv(c->pos)
        <<"\nnormal=" <<OUTv(c->normal)
        <<"\npenetration depth=" <<OUTs(c->depth)
        <<std::endl;
  }
}

// - compute depth of penetration and normal for
//   body with touch sensor, 6. Mar 06 (hh).
// - fixed bug (I did not assume that each contact is
//   just represented once), 12. Mar 06 (hh).
// - copy penetration data to Body, 22. May 06 (hh)
void OdeInterface::penetration(mlr::Vector &p) {
  NIY;
#if 0
  dContactGeom *c;
  double d;
  dBodyID b1, b2;
  
  for(uint i=0; i<conts.N; i++) {
    c = conts(i);
    b1 = dGeomGetBody(c->g1);
    b2 = dGeomGetBody(c->g2);
    //get body attributes
    double *touch1=0, *touch2=0;
    if(b1) touch1=anyListGet<double>(((mlr::Body*)b1->userdata)->ats, "touchsensor", 0);
    if(b2) touch2=anyListGet<double>(((mlr::Body*)b2->userdata)->ats, "touchsensor", 0);
    
    if(touch1 || touch2) {
      d = c->depth;
      p.set(c->normal);
      p *= d;
      if(touch1)((mlr::Body*)b1->userdata)->touch = p;
      else((mlr::Body*)b2->userdata)->touch = -p;
      //printf("%lf\n", d);
      std::cout <<"contact " <<i <<": penetration: " <<OUTv(p.v) <<"\n";
    }
  }
#endif
}


void OdeInterface::contactForces() {
  uint i;
  dContactGeom *c;
  dBodyID b1, b2;
  mlr::Vector pos, normal, v1, v2, vrel;
  dMass mass;
  double force, d, m1, m2;
  for(i=0; i<conts.N; i++) {
    c = conts(i);
    pos.set(c->pos);
    d=c->depth*20.;
    normal.set(c->normal);
    normal *=-1.; //vector from b1 -> b2
    b1 = dGeomGetBody(c->g1);
    b2 = dGeomGetBody(c->g2);
    if(b1) { v1.set(b1->lvel); dBodyGetMass(b1, &mass); m1=mass.mass; } else { v1.setZero(); m1=0.; }
    if(b2) { v2.set(b1->lvel); dBodyGetMass(b2, &mass); m2=mass.mass; } else { v2.setZero(); m2=0.; }
    
    // ** normal force:
    force=0.;
    vrel=v2-v1; //inaccurate: should also include rotational velocities at contact point!
    vrel.makeColinear(normal);
    //spring force
    force = .03*d*d*d;
    if(b2) dBodyAddForceAtPos(b2, force*normal.x, force*normal.y, force*normal.z, pos.x, pos.y, pos.z);
    force *= -1.; //invert on other body
    if(b1) dBodyAddForceAtPos(b1, force*normal.x, force*normal.y, force*normal.z, pos.x, pos.y, pos.z);
    
    //inward viscosity
    if(vrel * normal <= 0) {
      force += 10.*vrel.length();
      if(b2) dBodyAddForceAtPos(b2, force*m2*normal.x, force*m2*normal.y, force*m2*normal.z, pos.x, pos.y, pos.z);
      force *= -1.; //invert on other body
      if(b1) dBodyAddForceAtPos(b1, force*m1*normal.x, force*m1*normal.y, force*m1*normal.z, pos.x, pos.y, pos.z);
    }
    
    if(!b2) std::cout <<"bodyForce = " <<force*normal <<std::endl;
    
#if 1
    // ** parallel (slip) force:
    force=0.;
    vrel=v2-v1; //inaccurate: should also include rotational velocities at contact point!
    vrel.makeNormal(normal);
    //viscosity
    force += 10.*vrel.length();
    
    vrel.normalize();
    force *= -1.;
    if(b2) dBodyAddForceAtPos(b2, force*m2*vrel.x, force*m2*vrel.y, force*m2*vrel.z, pos.x, pos.y, pos.z);
    force *= -1.;
    if(b1) dBodyAddForceAtPos(b1, force*m1*vrel.x, force*m1*vrel.y, force*m1*vrel.z, pos.x, pos.y, pos.z);
    
    if(!b2) std::cout <<"bodyForce = " <<force*normal <<std::endl;
    std::cout <<"slip force " <<force <<std::endl;
#endif
  }
}



//===========================================================================
//
// graph
//

void OdeInterface::exportStateToOde() {
  dBodyID b;
  
  for_list(mlr::Body,  n,  C.bodies) {
    CHECK(n->X.rot.isNormalized(), "quaternion is not normalized!");
    b = bodies(n->index);
    CP3(b->posr.pos, n->X.pos.p());                // dxBody changed in ode-0.6 ! 14. Jun 06 (hh)
    CP4(b->q, n->X.rot.p()); dQtoR(b->q, b->posr.R);
    CP3(b->lvel, n->X.vel.p());
    CP3(b->avel, n->X.angvel.p());
    //n->copyFrameToOde();
#ifndef MLR_ode_nojoints
    for_list(mlr::Joint,  e,  n->inLinks) if(e->type!=mlr::JT_glue) {
      dxJointHinge* hj=(dxJointHinge*)joints(e->index);
      dxJointUniversal* uj=(dxJointUniversal*)joints(e->index);
      dxJointSlider* sj=(dxJointSlider*)joints(e->index);
      switch(e->type) { //16. Mar 06 (hh)
        case mlr::JT_rigid:
          break;
        case mlr::JT_hingeX:
          CP4(hj->qrel, (e->A.rot*e->B.rot).p());
          CP3(hj->anchor1, (e->A.pos).p());
          CP3(hj->anchor2, (-(e->B.rot/e->B.pos)).p());
          CP3(hj->axis1, (e->A.rot*mlr::Vector(1, 0, 0)).p());
          CP3(hj->axis2, (e->B.rot/mlr::Vector(1, 0, 0)).p());
          break;
        case mlr::JT_universal:
          CP4(uj->qrel1, (e->A.rot).p());
          CP4(uj->qrel2, (e->B.rot).p());
          CP3(uj->anchor1, (e->A.pos).p());
          CP3(uj->anchor2, (-(e->B.rot/e->B.pos)).p());
          CP3(uj->axis1, (e->A.rot*mlr::Vector(1, 0, 0)).p());
          CP3(uj->axis2, (e->B.rot/mlr::Vector(1, 0, 0)).p());
          break;
        case mlr::JT_transX:
          CP4(sj->qrel, (e->A.rot*e->B.rot).p());
          CP3(sj->offset, (e->Q.pos).p());
          //printf("offset: (%lf; %lf; %lf)\n", (e->X.pos)[0], (e->X.pos)[1], (e->X.pos)[2]);
          CP3(sj->axis1, (e->A.rot*mlr::Vector(1, 0, 0)).p());
          break;
        default: NIY;
      }
    }
#endif
  }
}

void OdeInterface::pushPoseForShape(mlr::Shape *s){
  dGeomID geom = geoms(s->index);
  dGeomSetQuaternion(geom,*((dQuaternion*)s->rel.rot.p()));
  dGeomSetPosition(geom,s->rel.pos.x,s->rel.pos.y,s->rel.pos.z);
}

void OdeInterface::exportForcesToOde() {
  dBodyID b;
  for_list(mlr::Body,  n,  C.bodies) {
    b=bodies(n->index);
    CP3(b->facc, n->force.p());
    CP3(b->tacc, n->torque.p());
  }
}

void OdeInterface::importStateFromOde() {
  dBodyID b;
  for_list(mlr::Body,  n,  C.bodies) {
    //n->getFrameFromOde();
    b=bodies(n->index);
    CP3(n->X.pos.p(), b->posr.pos);
    CP4(n->X.rot.p(), b->q);
    CP3(n->X.vel.p(), b->lvel);
    CP3(n->X.angvel.p(), b->avel);
    CHECK(n->X.rot.isNormalized(), "quaternion is not normalized!");
  }
  C.calc_Q_from_BodyFrames();
  C.calc_fwdPropagateShapeFrames();
}

void OdeInterface::addJointForce(mlr::Joint *e, double f1, double f2) {
  switch(e->type) {
    case mlr::JT_hingeX:
      dJointAddHingeTorque(joints(e->index), -f1);
      break;
    case mlr::JT_rigid: // no torque
      break;
    case mlr::JT_universal:
      dJointAddUniversalTorques(joints(e->index), -f1, -f2);
      break;
    case mlr::JT_transX:
      dJointAddSliderForce(joints(e->index), -f1);
      break;
    default: NIY;
  }
}

void OdeInterface::addJointForce(doubleA& x) {
  uint n=0;
  
  for_list(mlr::Joint,  e,  C.joints) { // loop over edges, 16. Mar 06 (hh)
    switch(e->type) { //  3. Apr 06 (hh)
      case mlr::JT_hingeX:
        dJointAddHingeTorque(joints(e->index), -x(n));
        n++;
        break;
      case mlr::JT_rigid: // no torque
        break;
      case mlr::JT_universal:
        dJointAddUniversalTorques(joints(e->index), -x(n), -x(n+1));
        n+=2;
        break;
      case mlr::JT_transX:
        dJointAddSliderForce(joints(e->index), -x(n));
        n++;
        break;
      default: NIY;
    }
  }
  CHECK_EQ(n,x.N, "wrong dimensionality");
}

void OdeInterface::setMotorVel(const arr& qdot, double maxF) {
  uint n=0;
  
  for_list(mlr::Joint,  e,  C.joints) {
    switch(e->type) {
      case mlr::JT_hingeX:
        dJointSetAMotorParam(motors(e->index), dParamVel, -qdot(e->qIndex));
        dJointSetAMotorParam(motors(e->index), dParamFMax, maxF);
        n++;
        break;
      case mlr::JT_rigid:
        break;
      case mlr::JT_universal:
        n+=2;
        break;
      case mlr::JT_transX:
        n++;
        break;
      default: NIY;
    }
  }
  CHECK_EQ(n,qdot.N, "wrong dimensionality");
}

uint OdeInterface::getJointMotorDimension() {
  NIY;
  uint i=0;
  for_list(mlr::Body,  n,  C.bodies) {
    for_list(mlr::Joint,  e,  n->inLinks) { // if(!e->fixed && e->motor){ // 16. Mar 06 (hh)
      if(e->type==mlr::JT_universal) i+=2;
      else i++;
    }
  }
  return i;
}

void OdeInterface::setJointMotorPos(mlr::Joint *e, double x0, double maxF, double tau) {
  double x=-dJointGetHingeAngle(joints(e->index));
  setJointMotorVel(e, .1*(x0-x)/tau, maxF);
}

void OdeInterface::setJointMotorVel(mlr::Joint *e, double v0, double maxF) {
  dJointSetAMotorParam(motors(e->index), dParamVel, -v0);
  dJointSetAMotorParam(motors(e->index), dParamFMax, maxF);
}

void OdeInterface::setJointMotorPos(doubleA& x, double maxF, double tau) {
  //CHECK_EQ(x.N,E, "given joint state has wrong dimension, " <<x.N <<"=" <<E);
  NIY;
  uint i=0;
  for_list(mlr::Body,  n,  C.bodies) {
    for_list(mlr::Joint,  e,  n->inLinks) { // if(e->motor)
      setJointMotorPos(e, x(i), maxF, tau);
      i++;
      break;
    }
  }
  CHECK_EQ(x.N,i, "joint motor array had wrong dimension " <<x.N <<"!=" <<i);
}

void OdeInterface::setJointMotorVel(doubleA& x, double maxF) {
  //CHECK_EQ(x.N,E, "given joint state has wrong dimension, " <<x.N <<"=" <<E);
  NIY;
  uint i=0;
  for_list(mlr::Body,  n,  C.bodies) {
    for_list(mlr::Joint,  e,  n->inLinks) { // if(e->motor){
      setJointMotorVel(e, x(i), maxF);
      i++;
      break;
    }
  }
  CHECK_EQ(x.N,i, "joint motor array had wrong dimension " <<x.N <<"!=" <<i);
}

void OdeInterface::unsetJointMotors() {
  NIY;
  for_list(mlr::Body,  n,  C.bodies) for(mlr::Joint *e: n->inLinks) { // if(e->motor){
    unsetJointMotor(e);
    break;
  }
}

void OdeInterface::unsetJointMotor(mlr::Joint *e) {
  dJointSetAMotorParam(motors(e->index), dParamFMax, 0.);
}

void OdeInterface::getJointMotorForce(mlr::Joint *e, double& f) {
  dJointFeedback* fb=dJointGetFeedback(motors(e->index));
  CHECK(fb, "no feedback buffer set for this joint");
  //std::cout <<OUTv(fb->f1) <<' ' <<OUTv(fb->t1) <<' ' <<OUTv(fb->f2) <<' ' <<OUTv(fb->t2) <<std::endl;
  mlr::Vector t; t.x=fb->t1[0]; t.y=fb->t1[1]; t.z=fb->t1[2];
  f=t * (e->from->X.rot*(e->A.rot*mlr::Vector(1, 0, 0)));
  f=-f;
}

void OdeInterface::getJointMotorForce(doubleA& f) {
  NIY;
  uint i=0;
  for_list(mlr::Body,  n,  C.bodies) {
    for_list(mlr::Joint,  e,  n->inLinks) { // if(!e->fixed && e->motor){
      getJointMotorForce(e, f(i));
      i++;
      break;
    }
  }
  CHECK_EQ(f.N,i, "joint motor array had wrong dimension " <<f.N <<"!=" <<i);
}

void OdeInterface::pidJointPos(mlr::Joint *e, double x0, double v0, double xGain, double vGain, double iGain, double* eInt) {
  double x, v, f;
  //double a, b, c, dAcc;
  
  x=-dJointGetHingeAngle(joints(e->index));
  v=-dJointGetHingeAngleRate(joints(e->index));
  f = xGain*(x0-x) + vGain*(v0-v);
  if(eInt) {
    f+=iGain* (*eInt);
    (*eInt) = .99*(*eInt) + .01 *(x0-x);
  }
  std::cout <<"PID:" <<x0 <<' ' <<x <<' ' <<v <<" -> " <<f <<std::endl;
  dJointSetAMotorParam(motors(e->index), dParamFMax, 0);
  dJointAddHingeTorque(joints(e->index), -f);
}

void OdeInterface::pidJointVel(mlr::Joint *e, double v0, double vGain) {
  double v, f;
  
  v=-dJointGetHingeAngleRate(joints(e->index));
  f = vGain*(v0-v);
  if(fabs(f)>vGain) f=mlr::sign(f)*vGain;
  std::cout <<"PIDv:" <<v0 <<' ' <<v <<" -> " <<f <<std::endl;
  dJointSetAMotorParam(motors(e->index), dParamFMax, 0);
  dJointAddHingeTorque(joints(e->index), -f);
}


//===========================================================================
//
// higher level C
//

void OdeInterface::step(doubleA& force, uint steps, double tau) {
  addJointForce(force);
  for(; steps--;) step(tau);
  importStateFromOde();
}

void OdeInterface::step(uint steps, double tau) {
  for(; steps--;) step(tau);
  importStateFromOde();
}

void OdeInterface::getGroundContact(boolA& cts) {
  cts.resize(C.bodies.N);
  cts=false;
  //reportContacts();
  uint i;
  dContactGeom *c;
  for(i=0; i<conts.N; i++) {
    c = conts(i);
    dBodyID b1, b2;
    b1 = dGeomGetBody(c->g1);
    b2 = dGeomGetBody(c->g2);
    
    if(!b1 && b2) cts(((mlr::Body*)b2->userdata)->index)=true;
    if(b1 && !b2) cts(((mlr::Body*)b1->userdata)->index)=true;
  }
}

/*struct dContactGeom {
  dVector3 pos;       // contact position
  dVector3 normal;    // normal vector
  dReal depth;        // penetration depth
  dGeomID g1, g2;      // the colliding geoms
  };*/


void OdeInterface::importProxiesFromOde() {
  uint i;
  C.proxies.memMove=true;
  C.proxies.resizeCopy(conts.N);
  for(i=0; i<conts.N; i++) C.proxies(i) = new mlr::Proxy;
  dContactGeom *c;
  int a, b;
  mlr::Vector d, p;
  for(i=0; i<conts.N; i++) {
    c = conts(i);
    dBodyID b1, b2;
    b1 = dGeomGetBody(c->g1);
    b2 = dGeomGetBody(c->g2);
    
    a = b1 ? (((mlr::Body*)b1->userdata)->index) : (uint)-1;
    b = b2 ? (((mlr::Body*)b2->userdata)->index) : (uint)-1;
    
    d.set(c->normal);
    d *= -c->depth/2.;
    p.set(c->pos);
    
    C.proxies(i)->a = a;
    C.proxies(i)->b = b;
    C.proxies(i)->d = -c->depth;
    C.proxies(i)->normal.set(c->normal);
    C.proxies(i)->posA=p+d;
    C.proxies(i)->posB=p-d;
    //if(a!=-1) C.proxies(i)->velA=C.bodies(a)->X.vel + (C.bodies(a)->X.angvel^(p+d-(C.bodies(a)->X.pos))); else C.proxies(i)->velA.setZero();
    //if(b!=-1) C.proxies(i)->velB=C.bodies(b)->X.vel + (C.bodies(b)->X.angvel^(p-d-(C.bodies(b)->X.pos))); else C.proxies(i)->velB.setZero();
    C.proxies(i)->posB=p-d;
//    if(a!=-1 && b!=-1) C.proxies(i)->rel.setDifference(C.bodies(a)->X, C.bodies(b)->X);
//    else if(a!=-1) C.proxies(i)->rel.setInverse(C.bodies(a)->X);
//    else if(b!=-1) C.proxies(i)->rel = C.bodies(b)->X;
//    else           C.proxies(i)->rel.setZero();
//    C.proxies(i)->age=0;
  }
}

void OdeInterface::reportContacts2() {
  uint i;
  dBodyID b1, b2;
  mlr::Body* b;
  dContactGeom* c;
  mlr::Vector x;
  std::cout <<"contacts=" <<conts.N <<std::endl;
  for(i=0; i<conts.N; i++) {
    c = conts(i);
    std::cout <<i <<' ';
    b1=dGeomGetBody(c->g1);
    b2=dGeomGetBody(c->g2);
    if(b1) {
      b=(mlr::Body*)b1->userdata;
      std::cout <<b->name <<' ';
    } else std::cout <<"NIL ";
    if(b2) {
      b=(mlr::Body*)b2->userdata;
      std::cout <<b->name <<' ';
    } else std::cout <<"NIL ";
    x.set(c->pos);    std::cout <<"pos=" <<x <<' ';
    x.set(c->normal); std::cout <<"normal=" <<x <<' ';
    std::cout <<"depth=" <<c->depth <<std::endl;
  }
}

struct Bound { mlr::Vector p, n; };
bool OdeInterface::inFloorContacts(mlr::Vector& x) {
  uint i, j, k;
  dBodyID b1, b2;
  dContactGeom* c;
  mlr::Vector y;
  
  x.z=0.;
  
  mlr::Array<mlr::Vector> v;
  //collect list of floor contacts
  for(i=0; i<conts.N; i++) {
    c = conts(i);
    b1=dGeomGetBody(c->g1);
    b2=dGeomGetBody(c->g2);
    if((!b1 && b2) || (b1 &&!b2)) {
      y.set(c->pos);  y.z=0.;
      v.append(y);
    }
  }
  
  std::cout <<"\nfloor points: ";
  for(i=0; i<v.N; i++) std::cout <<v(i) <<'\n';
  
  if(!v.N) return false;
  
  //construct boundaries
  Bound b;
  mlr::Array<Bound> bounds;
  double s;
  bool f;
  for(i=0; i<v.N; i++) for(j=0; j<i; j++) {
      b.p=v(i); b.n=(v(j)-v(i)) ^ mlr::Vector(0, 0, 1);
      f=false;
      for(k=0; k<v.N; k++) if(k!=j && k!=i) {
          s=(v(k)-b.p) * b.n ;
          if(s<0) {
            if(f) break;
            b.n=-b.n;
          }
          if(s!=0.) f=true;
        }
      if(k==v.N) bounds.append(b);
    }
    
  std::cout <<"\nbounds: ";
  for(i=0; i<bounds.N; i++) std::cout <<bounds(i).p <<' ' <<bounds(i).n <<'\n';
  
  std::cout <<"\nquery: " <<x <<std::endl;
  //check for internal
  for(i=0; i<bounds.N; i++) {
    if((x-bounds(i).p) * bounds(i).n < 0) return false;
  }
  return true;
}

void OdeInterface::slGetProxies() {
  //C.setJointState(x);
  //dJointGroupEmpty(contactgroup);
  //exportStateToOde(C, ode);
  conts.clear();
  noContactJoints=true;
  dSpaceCollide(space, this, OdeInterface::staticCallback);
  importProxiesFromOde();
}

/*void OdeInterface::slGetProxyGradient(arr &dx, const arr &x, mlr::KinematicWorld &C){
  if(C.proxies.N){
    arr dp, J;
    C.getContactGradient(dp);
    C.jacobianAll(J, x);
    dx = 2.* (~J) * dp;
  }else{
    dx.resize(x.N);
    dx.setZero();
  }
}

void OdeInterface::slGetProxyGradient(arr &dx, const arr &x, mlr::KinematicWorld &C, OdeInterface &ode){
  slGetProxies(x, C, ode);
  slGetProxyGradient(dx, x, C);
}*/

#undef OUTs
#undef OUTv
#undef OUTq
#undef CP4
#undef CP3


//===========================================================================
//
// communication with Ode
//

#if 0 //documentation...
/** @brief copy all frame variables (positions, velocities, etc)
into the ODE engine (createOde had to be called before) */
void OdeInterface::exportStateToOde();

/** @brief copy the current state of the ODE engine into
the frame variables (positions, velocities, etc) */
void OdeInterface::importStateFromOde();

/** @brief copy all frame variables (positions, velocities, etc)
into the ODE engine (createOde had to be called before) */
void OdeInterface::exportForcesToOde();

void OdeInterface::setJointForce(mlr::Joint *e, double f1, double f2);

/** @brief applies forces on the configuration as given by the force vector x */
void OdeInterface::setJointForce(arr& f);

/** @brief returns the number of motors attached to joints */
uint getJointMotorDimension();

/** @brief set the desired positions of all motor joints */
void OdeInterface::setJointMotorPos(arr& x, double maxF, double tau);
/** @brief set the desired position of a specific motor joint */
void OdeInterface::setJointMotorPos(mlr::Joint *e, double x0, double maxF, double tau);

/** @brief set the desired velocities of all motor joints */
void OdeInterface::setJointMotorVel(arr& v, double maxF=1.);
/** @brief set the desired velocity of a specific motor joint */
void OdeInterface::setJointMotorVel(mlr::Joint *e, double v0, double maxF=1.);

/** @brief disable motors by setting maxForce parameter to zero */
void OdeInterface::unsetJointMotors(OdeInterface& ode);
/** @brief disable motor by setting maxForce parameter to zero */
void OdeInterface::unsetJointMotor(mlr::Joint *e);

/** @brief get the forces induced by all motor motor joints */
void OdeInterface::getJointMotorForce(arr& f);
/** @brief get the force induced by a specific motor joint */
void OdeInterface::getJointMotorForce(mlr::Joint *e, double& f);

void OdeInterface::pidJointPos(mlr::Joint *e, double x0, double v0, double xGain, double vGain, double iGain=0, double* eInt=0);

void OdeInterface::pidJointVel(mlr::Joint *e, double v0, double vGain);

/// [obsolete] \ingroup sl
void OdeInterface::getGroundContact(boolA& cts);

/// import the information from ODE's contact list into the proximity list \ingroup sl
void OdeInterface::importProxiesFromOde(OdeInterface& ode);

/** @brief simulates one time step with the ODE engine, starting from state
    vector in, returns state vector out \ingroup sl */
void OdeInterface::step(arr& in, arr& force, arr& out, uint steps=1);

/** @brief simulates one time step with the ODE engine, starting from state
    vector in, returns state vector out \ingroup sl */
void OdeInterface::step(arr& force, arr& out, uint steps=1);

/// \ingroup sl
void OdeInterface::step(uint steps=1, double tau=.01);

/** @brief instantiate the configuration in an ODE engine (first clears the
    ODE engine from all other objects) \ingroup sl */
void OdeInterface::createOde(OdeInterface& ode);

/// \ingroup sl
void OdeInterface::slGetProxies(OdeInterface &ode);

/// \ingroup sl
//void OdeInterface::slGetProxyGradient(arr &dx, const arr &x, mlr::KinematicWorld &C, OdeInterface &ode);

/// \ingroup sl
void OdeInterface::reportContacts(OdeInterface& ode);
/// \ingroup sl
bool inFloorContacts(mlr::Vector& x);

#endif

#else
OdeInterface::OdeInterface(mlr::KinematicWorld &_C):C(_C) { MLR_MSG("WARNING - creating dummy OdeInterface"); }
OdeInterface::~OdeInterface() {}
void OdeInterface::step(double dtime) {}
void OdeInterface::clear() {}
void OdeInterface::slGetProxies() {}
void OdeInterface::unsetJointMotors() {}
void OdeInterface::exportStateToOde() {}
void OdeInterface::importStateFromOde() {}
void OdeInterface::importProxiesFromOde() {}
void OdeInterface::addJointForce(arr& f) {}
void OdeInterface::pushPoseForShape(mlr::Shape *s) {};
#endif
/** @} */
/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#include "kinViewer.h"

#include <iomanip>

//===========================================================================

OrsViewer_old::OrsViewer_old(const char* varname, double beatIntervalSec, bool computeCameraView)
  : Thread(STRING("OrsViewer_old_"<<varname), beatIntervalSec),
    modelWorld(this, varname, (beatIntervalSec<0.)),
    modelCameraView(this, "modelCameraView"),
    modelDepthView(this, "modelDepthView"),
    computeCameraView(computeCameraView){
  if(beatIntervalSec>=0.) threadLoop(); else threadStep();
}

OrsViewer_old::~OrsViewer_old(){ threadClose(); }

void OrsViewer_old::open(){
  copy.gl(STRING("OrsViewer_old: "<<modelWorld.name));
}

void OrsViewer_old::step(){
  copy.gl().dataLock.writeLock();
  copy = modelWorld.get();
  copy.gl().dataLock.unlock();
  copy.gl().update(NULL, false, false, true);
  if(computeCameraView){
    mlr::Shape *kinectShape = copy.getShapeByName("endeffKinect");
    if(kinectShape){ //otherwise 'copy' is not up-to-date yet
      copy.gl().dataLock.writeLock();
      mlr::Camera cam = copy.gl().camera;
      copy.gl().camera.setKinect();
      copy.gl().camera.X = kinectShape->X * copy.gl().camera.X;
//      openGlLock();
      copy.gl().renderInBack(true, true, 580, 480);
//      copy.glGetMasks(580, 480, true);
//      openGlUnlock();
      modelCameraView.set() = copy.gl().captureImage;
      modelDepthView.set() = copy.gl().captureDepth;
      copy.gl().camera = cam;
      copy.gl().dataLock.unlock();
    }
  }
}

//===========================================================================

OrsViewer::OrsViewer(const char* world_name, double beatIntervalSec)
  : Thread(STRING("OrsViewer_"<<world_name), beatIntervalSec),
    world(this, world_name, (beatIntervalSec<0.)){
  if(beatIntervalSec>=0.) threadLoop(); else threadStep();
}

OrsViewer::~OrsViewer(){
  threadClose();
}

void OrsViewer::open(){
  gl = new OpenGL(STRING("OrsViewer: "<<world.name));
  gl->add(glStandardScene);
  gl->add(glDrawMeshes, &meshesCopy);
  gl->add(mlr::glDrawProxies, &proxiesCopy);
  gl->camera.setDefault();
}

void OrsViewer::close(){
  listDelete(proxiesCopy);
  delete gl;
}

void OrsViewer::step(){
  //-- get transforms, or all shapes if their number changed, and proxies
  mlr::Array<mlr::Transformation> X;
  world.readAccess();
  if(world->shapes.N!=meshesCopy.N){ //need to copy meshes
    uint n=world->shapes.N;
    gl->dataLock.writeLock();
    meshesCopy.resize(n);
    for(uint i=0;i<n;i++) meshesCopy.elem(i) = world->shapes.elem(i)->mesh;
    gl->dataLock.unlock();
  }
  X.resize(world->shapes.N);
  for(mlr::Shape *s:world().shapes) X(s->index) = s->X;
  gl->dataLock.writeLock();
  listCopy(proxiesCopy, world->proxies);
  gl->dataLock.unlock();
  world.deAccess();

  //-- set transforms to mesh display
  gl->dataLock.writeLock();
  CHECK_EQ(X.N, meshesCopy.N, "");
  for(uint i=0;i<X.N;i++) meshesCopy(i).glX = X(i);
  gl->dataLock.unlock();

  gl->update(NULL, false, false, true);
}

//===========================================================================

void OrsPathViewer::setConfigurations(const WorldL& cs){
  configurations.writeAccess();
  listResize(configurations(), cs.N);
  for(uint i=0;i<cs.N;i++) configurations()(i)->copy(*cs(i), true);
  configurations.deAccess();
}

void OrsPathViewer::clear(){
  listDelete(configurations.set()());
}

OrsPathViewer::OrsPathViewer(const char* varname, double beatIntervalSec, int tprefix)
  : Thread(STRING("OrsPathViewer_"<<varname), beatIntervalSec),
    configurations(this, varname, (beatIntervalSec<0.)),
    t(0), tprefix(tprefix), writeToFiles(false){
  if(beatIntervalSec>=0.) threadLoop(); else threadStep();
}

OrsPathViewer::~OrsPathViewer(){
  threadClose();
  clear();
}

void OrsPathViewer::open(){
  copy.gl(STRING("OrsPathViewer: "<<configurations.name));
}

void OrsPathViewer::step(){
  copy.gl().dataLock.writeLock();
  configurations.readAccess();
  uint T=configurations().N;
  if(t>=T*1.1) t=0;
  uint tt=t;
  if(tt>=T) tt=T-1;
  if(T) copy.copy(*configurations()(tt), true);
  configurations.deAccess();
  copy.gl().dataLock.unlock();
  if(T){
    copy.gl().captureImg=writeToFiles;
    copy.gl().update(STRING(" (time " <<tprefix+int(tt) <<'/' <<tprefix+int(T) <<')').p, false, false, true);
    if(writeToFiles) write_ppm(copy.gl().captureImage,STRING("vid/z.path."<<std::setw(3)<<std::setfill('0')<<tprefix+int(tt)<<".ppm"));
  }
  t++;
}

//===========================================================================

void renderConfigurations(const WorldL& cs, const char* filePrefix, int tprefix, int w, int h){
  mlr::KinematicWorld copy;
  for(uint t=0;t<cs.N;t++){
    copy.copy(*cs(t), true);
#if 0 //render on screen
    copy.gl().resize(w,h);
    copy.gl().captureImg=true;
    copy.gl().update(STRING(" (time " <<tprefix+int(t) <<'/' <<tprefix+int(cs.N) <<')').p, false, false, true);
#else
    copy.gl().text.clear() <<"time " <<tprefix+int(t) <<'/' <<tprefix+int(cs.N);
    copy.gl().renderInBack(true, false, w, h);
#endif
    write_ppm(copy.gl().captureImage, STRING(filePrefix<<std::setw(3)<<std::setfill('0')<<t<<".ppm"));
  }
}

//===========================================================================

OrsPoseViewer::OrsPoseViewer(const char* modelVarName, const StringA& poseVarNames, double beatIntervalSec)
  : Thread(STRING("OrsPoseViewer_"<<poseVarNames), beatIntervalSec),
    modelWorld(this, modelVarName, false),
    gl(STRING("OrsPoseViewer: " <<poseVarNames)){
  for(const String& varname: poseVarNames){
    poses.append( new Access<arr>(this, varname, (beatIntervalSec<0.)) ); //listen only when beatInterval=1.
    copies.append( new mlr::KinematicWorld() );
  }
  copy = modelWorld.get();
  computeMeshNormals(copy.shapes);
  for(mlr::KinematicWorld *w: copies) w->copy(copy, true);
  if(beatIntervalSec>=0.) threadLoop();
}

OrsPoseViewer::~OrsPoseViewer(){
  threadClose();
  listDelete(copies);
  listDelete(poses);
}

void OrsPoseViewer::recopyKinematics(const mlr::KinematicWorld& world){
  stepMutex.lock();
  if(&world) copy=world;
  else copy = modelWorld.get();
  computeMeshNormals(copy.shapes);
  for(mlr::KinematicWorld *w: copies) w->copy(copy, true);
  stepMutex.unlock();
}

void OrsPoseViewer::open() {
  gl.add(glStandardScene, 0);
  gl.camera.setDefault();

  for(uint i=0;i<copies.N;i++) gl.add(*copies(i));
  //  gl.camera.focus(0.6, -0.1, 0.65);
  //  gl.width = 1280;
  //  gl.height = 960;
}

void OrsPoseViewer::step(){
  listCopy(copies.first()->proxies, modelWorld.get()->proxies);
//  cout <<copy.proxies.N <<endl;
  gl.dataLock.writeLock();
  for(uint i=0;i<copies.N;i++){
    arr q=poses(i)->get();
    if(q.N==copies(i)->getJointStateDimension())
      copies(i)->setJointState(q);
  }
  gl.dataLock.unlock();
  gl.update(NULL, false, false, true);
}

void OrsPoseViewer::close(){
  gl.clear();
}

//===========================================================================

ComputeCameraView::ComputeCameraView(double beatIntervalSec, const char* modelWorld_name)
  : Thread("ComputeCameraView", beatIntervalSec),
    modelWorld(this, modelWorld_name, (beatIntervalSec<.0)),
    cameraView(this, "kinect_rgb"), //"cameraView"),
    cameraDepth(this, "kinect_depth"), //"cameraDepth"),
    cameraFrame(this, "kinect_frame"), //"cameraFrame"),
    getDepth(true){
  if(beatIntervalSec<0.) threadOpen();
  else threadLoop();
}

ComputeCameraView::~ComputeCameraView(){
  threadClose();
}

void ComputeCameraView::open(){
  gl.add(glStandardLight);
  gl.addDrawer(&copy);
}

void ComputeCameraView::close(){
  gl.clear();
}

void ComputeCameraView::step(){
  copy = modelWorld.get();
  copy.orsDrawJoints = copy.orsDrawMarkers = copy.orsDrawProxies = false;

  mlr::Shape *kinectShape = copy.getShapeByName("endeffKinect");
  if(kinectShape){ //otherwise 'copy' is not up-to-date yet
    gl.dataLock.writeLock();
    gl.camera.setKinect();
    gl.camera.X = kinectShape->X * gl.camera.X;
    gl.dataLock.unlock();
    gl.renderInBack(true, getDepth, 640, 480);
    flip_image(gl.captureImage);
    flip_image(gl.captureDepth);
    cameraView.set() = gl.captureImage;
    if(getDepth){
      floatA& D = gl.captureDepth;
      uint16A depth_image(D.d0, D.d1);
      for(uint i=0;i<D.N;i++){
        depth_image.elem(i)
            = (uint16_t) (gl.camera.glConvertToTrueDepth(D.elem(i)) * 1000.); // conv. from [m] -> [mm]
      }
      cameraDepth.set() = depth_image;
    }
    cameraFrame.set() = kinectShape->X;
  }
}


/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#include "taskMap_AboveBox.h"

TaskMap_AboveBox::TaskMap_AboveBox(int iShape, int jShape)
  : i(iShape), j(jShape){
}


TaskMap_AboveBox::TaskMap_AboveBox(const mlr::KinematicWorld& G, const char* iShapeName, const char* jShapeName)
  :i(-1), j(-1){
  mlr::Shape *a = iShapeName ? G.getShapeByName(iShapeName):NULL;
  mlr::Shape *b = jShapeName ? G.getShapeByName(jShapeName):NULL;
  if(a) i=a->index;
  if(b) j=b->index;
}

void TaskMap_AboveBox::phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t){
  mlr::Shape *s1=G.shapes(i);
  mlr::Shape *s2=G.shapes(j);
  if(s2->type!=mlr::ST_ssBox){ //switch roles
    mlr::Shape *z=s1;
    s1=s2; s2=z;
  }
  CHECK(s2->type==mlr::ST_ssBox,"");//s1 should be the board
  arr pos,posJ;
  G.kinematicsRelPos(pos, posJ, s1->body, s1->rel.pos, s2->body, s2->rel.pos);
  arr range(3);
  double d1 = .5*s1->size(0) + s1->size(3);
  d1 =.05; //TODO: fixed! support size/radius of object on top
  double d2 = .5*s2->size(0) + s2->size(3);
  range(0) = fabs(d1 - d2);
  d1 = .5*s1->size(1) + s1->size(3);
  d1 =.05; //TODO: fixed! support size/radius of object on top
  d2 = .5*s2->size(1) + s2->size(3);
  range(1) = fabs(d1 - d2);
  range(2)=0.;
//  if(verbose>2) cout <<pos <<range
//                    <<pos-range <<-pos-range
//                   <<"\n 10=" <<s1->size(0)
//                  <<" 20=" <<s2->size(0)
//                 <<" 11=" <<s1->size(1)
//                <<" 21=" <<s2->size(1)
//               <<endl;
  y.resize(4);
  y(0) =  pos(0) - range(0);
  y(1) = -pos(0) - range(0);
  y(2) =  pos(1) - range(1);
  y(3) = -pos(1) - range(1);
  if(&J){
    J.resize(4, posJ.d1);
    J[0] =  posJ[0];
    J[1] = -posJ[0];
    J[2] =  posJ[1];
    J[3] = -posJ[1];
  }
}
/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#include "taskMap_qLimits.h"

//===========================================================================

void TaskMap_qLimits::phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t) {
  if(!limits.N) limits=G.getLimits();
  G.kinematicsLimitsCost(y, J, limits);
}

//===========================================================================

void LimitsConstraint::phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t){
  if(!limits.N) limits = G.getLimits();
  G.kinematicsLimitsCost(y, J, limits, margin);
  y -= .5;
}

/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


/**
 * @file
 * @ingroup group_ors
 */
/**
 * @ingroup group_ors
 * @{
 */


#ifdef MLR_PHYSX

#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#include <physx/PxPhysicsAPI.h>
#include <physx/extensions/PxExtensionsAPI.h>
#include <physx/extensions/PxDefaultErrorCallback.h>
#include <physx/extensions/PxDefaultAllocator.h>
#include <physx/extensions/PxDefaultSimulationFilterShader.h>
#include <physx/extensions/PxDefaultCpuDispatcher.h>
#include <physx/extensions/PxShapeExt.h>
#include <physx/foundation/PxMat33.h>
#include <physx/pvd/PxVisualDebugger.h>
#include <physx/physxvisualdebuggersdk/PvdConnectionFlags.h>
//#include <PxMat33Legacy.h>
#include <physx/extensions/PxSimpleFactory.h>
#pragma GCC diagnostic pop

#include "kin_physx.h"
#include <Gui/opengl.h>

using namespace physx;

static PxFoundation* mFoundation = NULL;
static PxPhysics* mPhysics = NULL;
static PxCooking* mCooking = NULL;
static PxDefaultErrorCallback gDefaultErrorCallback;
static PxDefaultAllocator gDefaultAllocatorCallback;
static PxSimulationFilterShader gDefaultFilterShader=PxDefaultSimulationFilterShader;


// ============================================================================
/**
 * @brief Connect ors with PhysX and add a cmaera.
 *
 * See bindOrsToOpenGL for a similar function.
 *
 * @param graph the graph PhysX is going to use.
 * @param gl the gl output.
 * @param physx the PhyxXInteface which handles the ors graph.
 */
void bindOrsToPhysX(mlr::KinematicWorld& graph, OpenGL& gl, PhysXInterface& physx) {
//  physx.create(graph);
  
  MLR_MSG("I don't understand this: why do you need a 2nd opengl window? (This is only for sanity check in the example.)")
  gl.add(glStandardScene, NULL);
  gl.add(physx);
  gl.setClearColors(1., 1., 1., 1.);
  
  mlr::Body* glCamera = graph.getBodyByName("glCamera");
  if(glCamera) {
    gl.camera.X = glCamera->X;
  } else {
    gl.camera.setPosition(10., -15., 8.);
    gl.camera.focus(0, 0, 1.);
    gl.camera.upright();
  }
  gl.update();
}

// ============================================================================

void PxTrans2OrsTrans(mlr::Transformation& f, const PxTransform& pose) {
  f.pos.set(pose.p.x, pose.p.y, pose.p.z);
  f.rot.set(pose.q.w, pose.q.x, pose.q.y, pose.q.z);
}

PxTransform OrsTrans2PxTrans(const mlr::Transformation& f) {
  return PxTransform(PxVec3(f.pos.x, f.pos.y, f.pos.z), PxQuat(f.rot.x, f.rot.y, f.rot.z, f.rot.w));
}

// ============================================================================
//stuff from Samples/PxToolkit

namespace PxToolkit {
PxConvexMesh* createConvexMesh(PxPhysics& physics, PxCooking& cooking, const PxVec3* verts, PxU32 vertCount, PxConvexFlags flags) {
  PxConvexMeshDesc convexDesc;
  convexDesc.points.count     = vertCount;
  convexDesc.points.stride    = sizeof(PxVec3);
  convexDesc.points.data      = verts;
  convexDesc.flags        = flags;
  
  PxDefaultMemoryOutputStream buf;
  if(!cooking.cookConvexMesh(convexDesc, buf))
    return NULL;
    
  PxDefaultMemoryInputData input(buf.getData(), buf.getSize());
  return physics.createConvexMesh(input);
}

PxTriangleMesh* createTriangleMesh32(PxPhysics& physics, PxCooking& cooking, const PxVec3* verts, PxU32 vertCount, const PxU32* indices32, PxU32 triCount) {
  PxTriangleMeshDesc meshDesc;
  meshDesc.points.count     = vertCount;
  meshDesc.points.stride      = 3*sizeof(float);
  meshDesc.points.data      = verts;
  
  meshDesc.triangles.count    = triCount;
  meshDesc.triangles.stride   = 3*sizeof(uint);
  meshDesc.triangles.data     = indices32;
  
  PxDefaultMemoryOutputStream writeBuffer;
  bool status = cooking.cookTriangleMesh(meshDesc, writeBuffer);
  if(!status)
    return NULL;
    
  PxDefaultMemoryInputData readBuffer(writeBuffer.getData(), writeBuffer.getSize());
  return physics.createTriangleMesh(readBuffer);
}
}
// ============================================================================

struct sPhysXInterface {
  PxScene* gScene;
  mlr::Array<PxRigidActor*> actors;
  mlr::Array<PxD6Joint*> joints;

  debugger::comm::PvdConnection* connection;
  
  sPhysXInterface():gScene(NULL) {}

  void addBody(mlr::Body *b, physx::PxMaterial *material);
  void addJoint(mlr::Joint *jj);

  void lockJoint(PxD6Joint *joint, mlr::Joint *ors_joint);
  void unlockJoint(PxD6Joint *joint, mlr::Joint *ors_joint);
};

// ============================================================================

PhysXInterface::PhysXInterface(mlr::KinematicWorld& _world): world(_world), s(NULL) {
  s = new sPhysXInterface;

  if(!mFoundation) {
    mFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gDefaultAllocatorCallback, gDefaultErrorCallback);
    mPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation, PxTolerancesScale());
    PxCookingParams cookParams(mPhysics->getTolerancesScale());
    cookParams.skinWidth = .001f;
    mCooking = PxCreateCooking(PX_PHYSICS_VERSION, *mFoundation, cookParams);
    if(!mCooking) HALT("PxCreateCooking failed!");
    if(!mPhysics) HALT("Error creating PhysX3 device.");

    if(!PxInitExtensions(*mPhysics))
      HALT("PxInitExtensions failed!");
  }

  //PxExtensionVisualDebugger::connect(mPhysics->getPvdConnectionManager(),"localhost",5425, 10000, true);

  //-- Create the scene
  PxSceneDesc sceneDesc(mPhysics->getTolerancesScale());
  sceneDesc.gravity = PxVec3(0.f, 0.f, -9.8f);

  if(!sceneDesc.cpuDispatcher) {
    PxDefaultCpuDispatcher* mCpuDispatcher = PxDefaultCpuDispatcherCreate(1);
    if(!mCpuDispatcher) {
      cerr << "PxDefaultCpuDispatcherCreate failed!" << endl;
    }
    sceneDesc.cpuDispatcher = mCpuDispatcher;
  }
  if(!sceneDesc.filterShader) {
    sceneDesc.filterShader  = gDefaultFilterShader;
  }

  s->gScene = mPhysics->createScene(sceneDesc);
  if(!s->gScene) {
    cerr << "createScene failed!" << endl;
  }

  s->gScene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.0);
  s->gScene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES, 1.0f);

  //-- Create objects
  PxMaterial* mMaterial = mPhysics->createMaterial(10.f, 10.f, 0.1f);

  //Create ground plane
  //PxReal d = 0.0f;
  PxTransform pose = PxTransform(PxVec3(0.f, 0.f, 0.f), PxQuat(-PxHalfPi, PxVec3(0.0f, 1.0f, 0.0f)));

  PxRigidStatic* plane = mPhysics->createRigidStatic(pose);
  CHECK(plane, "create plane failed!");

  PxShape* planeShape = plane->createShape(PxPlaneGeometry(), *mMaterial);
  CHECK(planeShape, "create shape failed!");
  s->gScene->addActor(*plane);
  // create ORS equivalent in PhysX
  // loop through ors
  for_list(mlr::Body,  b,  world.bodies) s->addBody(b, mMaterial);

  /// ADD joints here!
  for(mlr::Joint *jj : world.joints) s->addJoint(jj);

  /// save data for the PVD
  if(mlr::getParameter<bool>("physx_debugger", false)) {
    const char* filename = "pvd_capture.pxd2";
    PxVisualDebuggerConnectionFlags connectionFlags = PxVisualDebuggerExt::getAllConnectionFlags();

    s->connection = PxVisualDebuggerExt::createConnection(mPhysics->getPvdConnectionManager(), filename, connectionFlags);
    mPhysics->getVisualDebugger()->setVisualDebuggerFlags(PxVisualDebuggerFlag::eTRANSMIT_CONTACTS | PxVisualDebuggerFlag::eTRANSMIT_CONSTRAINTS);
  }
}

PhysXInterface::~PhysXInterface() {
  if(s->connection)
    s->connection->release();
  delete s;
}

void PhysXInterface::step(double tau) {
  //-- push positions of all kinematic objects
  for_list(mlr::Body, b, world.bodies) if(b->type==mlr::BT_kinematic) {
    ((PxRigidDynamic*)s->actors(b_COUNT))->setKinematicTarget(OrsTrans2PxTrans(b->X));
  }

  //-- dynamic simulation
  s->gScene->simulate(tau);
  
  //...perform useful work here using previous frame's state data
  while(!s->gScene->fetchResults()) {
  }
  
  //-- pull state of all objects
  pullFromPhysx(tau);
}

void PhysXInterface::setArticulatedBodiesKinematic(uint agent){
  for(mlr::Joint* j:world.joints) if(j->type!=mlr::JT_free){
    if(j->agent==agent){
      if(j->from->type==mlr::BT_dynamic) j->from->type=mlr::BT_kinematic;
      if(j->to->type==mlr::BT_dynamic) j->to->type=mlr::BT_kinematic;
    }
  }
  for(mlr::Body *b: world.bodies) {
    if(b->type==mlr::BT_kinematic)
      ((PxRigidDynamic*)s->actors(b->index))->setRigidDynamicFlag(PxRigidDynamicFlag::eKINEMATIC, true);
    if(b->type==mlr::BT_dynamic)
      ((PxRigidDynamic*)s->actors(b->index))->setRigidDynamicFlag(PxRigidDynamicFlag::eKINEMATIC, false);
  }
}

/**
 * @brief Create the PhysX interface which then can be used by OpenGL.
 *
 * - setup some physx stuff
 * - create PhysX equivalent to the ors graph
 */

void sPhysXInterface::addJoint(mlr::Joint *jj) {
  while(joints.N <= jj->index+1)
    joints.append(NULL);
  PxTransform A = OrsTrans2PxTrans(jj->A);
  PxTransform B = OrsTrans2PxTrans(jj->B);
  switch(jj->type) {
    case mlr::JT_free: //do nothing
      break;
    case mlr::JT_hingeX:
    case mlr::JT_hingeY:
    case mlr::JT_hingeZ: {

      PxD6Joint *desc = PxD6JointCreate(*mPhysics, actors(jj->from->index), A, actors(jj->to->index), B.getInverse());
      CHECK(desc, "PhysX joint creation failed.");

      if(jj->ats.find<arr>("drive")) {
        arr drive_values = jj->ats.get<arr>("drive");
        PxD6JointDrive drive(drive_values(0), drive_values(1), PX_MAX_F32, true);
        desc->setDrive(PxD6Drive::eTWIST, drive);
      }
      
      if(jj->ats.find<arr>("limit")) {
        desc->setMotion(PxD6Axis::eTWIST, PxD6Motion::eLIMITED);

        arr limits = jj->ats.get<arr>("limit");
        PxJointAngularLimitPair limit(limits(0), limits(1), 0.1f);
        limit.restitution = limits(2);
          //limit.spring = limits(3);
          //limit.damping= limits(4);
        //}
        desc->setTwistLimit(limit);
      }
      else {
        desc->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
      }

      if(jj->ats.find<arr>("drive")) {
        arr drive_values = jj->ats.get<arr>("drive");
        PxD6JointDrive drive(drive_values(0), drive_values(1), PX_MAX_F32, false);
        desc->setDrive(PxD6Drive::eTWIST, drive);
        //desc->setDriveVelocity(PxVec3(0, 0, 0), PxVec3(5e-1, 0, 0));
      }
      joints(jj->index) = desc;
    }
    break;
    case mlr::JT_rigid: {
      // PxFixedJoint* desc =
      PxFixedJointCreate(*mPhysics, actors(jj->from->index), A, actors(jj->to->index), B.getInverse());
      // desc->setProjectionLinearTolerance(1e10);
      // desc->setProjectionAngularTolerance(3.14);
    }
    break;
    case mlr::JT_trans3: {
      break; 
    }
    case mlr::JT_transXYPhi: {
      PxD6Joint *desc = PxD6JointCreate(*mPhysics, actors(jj->from->index), A, actors(jj->to->index), B.getInverse());
      CHECK(desc, "PhysX joint creation failed.");

      desc->setMotion(PxD6Axis::eX, PxD6Motion::eFREE);
      desc->setMotion(PxD6Axis::eY, PxD6Motion::eFREE);
      desc->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);

      joints(jj->index) = desc;
      break;
    }
    case mlr::JT_transX:
    case mlr::JT_transY:
    case mlr::JT_transZ:
    {
      PxD6Joint *desc = PxD6JointCreate(*mPhysics, actors(jj->from->index), A, actors(jj->to->index), B.getInverse());
      CHECK(desc, "PhysX joint creation failed.");

      if(jj->ats.find<arr>("drive")) {
        arr drive_values = jj->ats.get<arr>("drive");
        PxD6JointDrive drive(drive_values(0), drive_values(1), PX_MAX_F32, true);
        desc->setDrive(PxD6Drive::eX, drive);
      }
      
      if(jj->ats.find<arr>("limit")) {
        desc->setMotion(PxD6Axis::eX, PxD6Motion::eLIMITED);

        arr limits = jj->ats.get<arr>("limit");
        PxJointLinearLimit limit(mPhysics->getTolerancesScale(), limits(0), 0.1f);
        limit.restitution = limits(2);
        //if(limits(3)>0) {
          //limit.spring = limits(3);
          //limit.damping= limits(4);
        //}
        desc->setLinearLimit(limit);
      }
      else {
        desc->setMotion(PxD6Axis::eX, PxD6Motion::eFREE);
      }
      joints(jj->index) = desc;
    }
    break;
    default:
      NIY;
  }
}
void sPhysXInterface::lockJoint(PxD6Joint *joint, mlr::Joint *ors_joint) {
  joint->setMotion(PxD6Axis::eX, PxD6Motion::eLOCKED);
  joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eLIMITED);
  joint->setTwistLimit(PxJointAngularLimitPair(joint->getTwist()-.001, joint->getTwist()+.001));
}
void sPhysXInterface::unlockJoint(PxD6Joint *joint, mlr::Joint *ors_joint) {
  switch(ors_joint->type) {
    case mlr::JT_hingeX:
    case mlr::JT_hingeY:
    case mlr::JT_hingeZ:
      //joint->setMotion(PxD6Axis::eX, PxD6Motion::eLIMITED);
      //joint->setLinearLimit(PxJointLimit(ors_joint->Q.rot.getRad(), ors_joint->Q.rot.getRad()));
      joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
      break;
    case mlr::JT_transX:
    case mlr::JT_transY:
    case mlr::JT_transZ:
      //joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eLOCKED);
      joint->setMotion(PxD6Axis::eX, PxD6Motion::eFREE);
      break;
    default:
      break;
  }
}

void sPhysXInterface::addBody(mlr::Body *b, physx::PxMaterial *mMaterial) {
  PxRigidDynamic* actor=NULL;
  switch(b->type) {
    case mlr::BT_static:
      actor = (PxRigidDynamic*) mPhysics->createRigidStatic(OrsTrans2PxTrans(b->X));
      break;
    case mlr::BT_dynamic:
      actor = mPhysics->createRigidDynamic(OrsTrans2PxTrans(b->X));
      break;
    case mlr::BT_kinematic:
      actor = mPhysics->createRigidDynamic(OrsTrans2PxTrans(b->X));
      actor->setRigidDynamicFlag(PxRigidDynamicFlag::eKINEMATIC, true);
      break;
    case mlr::BT_none:
      HALT("this shoudn't be none BT!?")
//      actor = mPhysics->createRigidDynamic(OrsTrans2PxTrans(b->X));
      break;
  }
  CHECK(actor, "create actor failed!");
  for_list(mlr::Shape,  s,  b->shapes) {
    if(s->name.startsWith("coll_")) continue; //these are the 'pink' collision boundary shapes..
    PxGeometry* geometry;
    switch(s->type) {
      case mlr::ST_box: {
        geometry = new PxBoxGeometry(.5*s->size(0), .5*s->size(1), .5*s->size(2));
      }
      break;
      case mlr::ST_sphere: {
        geometry = new PxSphereGeometry(s->size(3));
      }
      break;
      case mlr::ST_capsule: {
        geometry = new PxCapsuleGeometry(s->size(3), s->size(2));
      }
      break;
      case mlr::ST_cylinder:
      case mlr::ST_ssBox:
      case mlr::ST_mesh: {
        // Note: physx can't decompose meshes itself.
        // Physx doesn't support triangle meshes in dynamic objects! See:
        // file:///home/mtoussai/lib/PhysX/Documentation/PhysXGuide/Manual/Shapes.html
        // We have to decompose the meshes "by hand" and feed them to PhysX.

        // PhysX uses float for the vertices
        floatA Vfloat;

        Vfloat.clear();
        copy(Vfloat, s->mesh.V); //convert vertices from double to float array..
        PxConvexMesh* triangleMesh = PxToolkit::createConvexMesh(
            *mPhysics, *mCooking, (PxVec3*)Vfloat.p, Vfloat.d0,
            PxConvexFlag::eCOMPUTE_CONVEX | PxConvexFlag::eINFLATE_CONVEX);
        geometry = new PxConvexMeshGeometry(triangleMesh);
      }
      break;
      case mlr::ST_marker: {
        geometry = NULL;
      }
      break;
      default:
        NIY;
    }
    if(geometry) {
      PxShape* shape = actor->createShape(*geometry, *mMaterial, OrsTrans2PxTrans(s->rel));
      CHECK(shape, "create shape failed!");
    }
    //actor = PxCreateDynamic(*mPhysics, OrsTrans2PxTrans(s->X), *geometry, *mMaterial, 1.f);
  }
  if(b->type == mlr::BT_dynamic) {
    if(b->mass) {
      PxRigidBodyExt::setMassAndUpdateInertia(*actor, b->mass);
    }
    else {
      PxRigidBodyExt::updateMassAndInertia(*actor, 1.f);
    }
    actor->setAngularDamping(0.75);
//    actor->setLinearVelocity(PxVec3(b->X.vel.x, b->X.vel.y, b->X.vel.z));
//    actor->setAngularVelocity(PxVec3(b->X.angvel.x, b->X.angvel.y, b->X.angvel.z));
  }
  gScene->addActor(*actor);
  actor->userData = b;

  actors.append(actor);
  //WARNING: actors must be aligned (indexed) exactly as G->bodies
  // TODO: we could use the data void pointer of an actor instead?
}

void PhysXInterface::pullFromPhysx(double tau) {
  for_list(PxRigidActor, a, s->actors) {
    PxTrans2OrsTrans(world.bodies(a_COUNT)->X, a->getGlobalPose());
    if(a->getType() == PxActorType::eRIGID_DYNAMIC) {
#if 0
      PxRigidBody *px_body = (PxRigidBody*) a;
      PxVec3 vel = px_body->getLinearVelocity();
      PxVec3 angvel = px_body->getAngularVelocity();
      mlr::Vector newvel(vel[0], vel[1], vel[2]);
      mlr::Vector newangvel(angvel[0], angvel[1], angvel[2]);
      mlr::Body *b = world.bodies(a_COUNT);
      b->force = b->mass * ((b->X.vel - newvel)/tau);
      b->torque = b->mass * ((b->X.angvel - newangvel)/tau);
      b->X.vel = newvel;
      b->X.angvel = newangvel;
#endif
    }
  }
  world.calc_fwdPropagateShapeFrames();
  world.calc_Q_from_BodyFrames();
  world.calc_q_from_Q();
}

void PhysXInterface::pushToPhysx() {
  HALT("why here?");
  PxMaterial* mMaterial = mPhysics->createMaterial(3.f, 3.f, 0.2f);
  for_list(mlr::Body, b, world.bodies) {
    if(s->actors.N > b_COUNT) {
      s->actors(b_COUNT)->setGlobalPose(OrsTrans2PxTrans(b->X));
    } else {
      s->addBody(b, mMaterial);
    }
  }
}

void PhysXInterface::ShutdownPhysX() {
  for_list(PxRigidActor, a, s->actors) {
    s->gScene->removeActor(*a);
    a->release();
  }
  s->gScene->release();
  mPhysics->release();
}

void DrawActor(PxRigidActor* actor, mlr::Body *body) {
  PxU32 nShapes = actor->getNbShapes();
  PxShape** shapes=new PxShape*[nShapes];
  //cout <<"#shapes=" <<nShapes;
  
  actor->getShapes(shapes, nShapes);
  while(nShapes--) {
    PxShape *shape = shapes[nShapes];

    // use the color of the first shape of the body for the entire body
    mlr::Shape *s = body->shapes(0);
    glColor(s->mesh.C);

    mlr::Transformation f;
    double mat[16];
    PxTrans2OrsTrans(f, PxShapeExt::getGlobalPose(*shape, *actor));
    glLoadMatrixd(f.getAffineMatrixGL(mat));
    //cout <<"drawing shape " <<body->name <<endl;
    switch(shape->getGeometryType()) {
      case PxGeometryType::eBOX: {
        PxBoxGeometry g;
        shape->getBoxGeometry(g);
        //glutSolidCube(g.halfExtents.x*2, g.halfExtents.y*2, g.halfExtents.z*2);
        glDrawBox(g.halfExtents.x*2, g.halfExtents.y*2, g.halfExtents.z*2);
      } break;
      case PxGeometryType::eSPHERE: {
        PxSphereGeometry g;
        shape->getSphereGeometry(g);
        glutSolidSphere(g.radius, 10, 10);
      } break;
      case PxGeometryType::eCAPSULE: {
        PxCapsuleGeometry g;
        shape->getCapsuleGeometry(g);
        glDrawCappedCylinder(g.radius, g.halfHeight*2);
      } break;
      case PxGeometryType::eCONVEXMESH: {
#if 1
        PxConvexMeshGeometry g;
        shape->getConvexMeshGeometry(g);
        floatA Vfloat((float*)g.convexMesh->getVertices(), 3*g.convexMesh->getNbVertices()); //reference
        mlr::Mesh mesh;
        copy(mesh.V,Vfloat);
        mesh.V.reshape(g.convexMesh->getNbVertices(),3);
        mesh.makeConvexHull();
        mesh.glDraw(NoOpenGL);
#else
        s->mesh.glDraw();
#endif
      } break;
      
      default:
        MLR_MSG("can't draw this type");
    }
  }
  delete [] shapes;
}

void PhysXInterface::glDraw(OpenGL&) {
  for_list(PxRigidActor, a, s->actors)  DrawActor(a, world.bodies(a_COUNT));
}

void PhysXInterface::addForce(mlr::Vector& force, mlr::Body* b) {
  PxVec3 px_force = PxVec3(force.x, force.y, force.z);
  PxRigidBody *actor = (PxRigidBody*) (s->actors(b->index)); // dynamic_cast fails for missing RTTI in physx
  actor->addForce(px_force);
}

void PhysXInterface::addForce(mlr::Vector& force, mlr::Body* b, mlr::Vector& pos) {
  PxVec3 px_force = PxVec3(force.x, force.y, force.z);
  PxVec3 px_pos = PxVec3(pos.x, pos.y, pos.z);
  PxRigidBody *actor = (PxRigidBody*)(s->actors(b->index));
  PxRigidBodyExt::addForceAtPos(*actor, px_force, px_pos);
}

///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////

#else //MLR_PHYSX

#include "kin_physx.h"
PhysXInterface::PhysXInterface(mlr::KinematicWorld& _world) : world(_world), s(NULL) { NICO }
PhysXInterface::~PhysXInterface() { NICO }
  
void PhysXInterface::step(double tau) { NICO }
void PhysXInterface::pushToPhysx() { NICO }
void PhysXInterface::pullFromPhysx(double tau) { NICO }
void PhysXInterface::setArticulatedBodiesKinematic(uint agent) { NICO }
void PhysXInterface::ShutdownPhysX() { NICO }
void PhysXInterface::glDraw(OpenGL&) { NICO }
void PhysXInterface::addForce(mlr::Vector& force, mlr::Body* b) { NICO }
void PhysXInterface::addForce(mlr::Vector& force, mlr::Body* b, mlr::Vector& pos) { NICO }

void glPhysXInterface(void *classP) { NICO }
void bindOrsToPhysX(mlr::KinematicWorld& graph, OpenGL& gl, PhysXInterface& physx) { NICO }

#endif
/** @} */
#include "taskMap_linTrans.h"

void TaskMap_LinTrans::phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t){
  map->phi(y, J, G, t);
  if(!norm){
    y = A*y + a;
    if(&J) J = A*J;
  }
  if(norm){
    double l = sqrt(sumOfSqr(y));
    if(&J) J = ~(y/l)*J;
    y = ARR(l);
  }
}

uint TaskMap_LinTrans::dim_phi(const mlr::KinematicWorld& G){
  if(!norm) return a.N;
  if(norm) return 1;
  HALT("");
  return 0;
}
/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */

#include "roboticsCourse.h"
#include <Kin/kin.h>
#include <Kin/kin_swift.h>
#include <Kin/kin_physx.h>
#include <Gui/opengl.h>
#include <Plot/plot.h>
#include <Algo/algos.h>

void drawEnv(void*){ glStandardLight(NULL); glDrawFloor(10., .9, .9, .9); }
void drawBase(void*){ glDrawAxes(1.); }

struct sSimulator {
  mlr::KinematicWorld G;
  double margin;
  double dynamicNoise;
  bool gravity;
  
  //state
  arr qddot;

  sSimulator(){ margin=.1; dynamicNoise=0.; gravity=true; } //default margin = 10cm
};

void Simulator::anchorKinematicChainIn(const char* bodyName){
  s->G.reconfigureRoot(s->G.getBodyByName(bodyName));
  s->G.calc_fwdPropagateFrames();
  
  NIY;
//  if(s->G.swift().isOpen){
//    s->G.swift().close();
//    s->G.swift().init(s->G);
//    s->G.swift().setCutoff(.5);
//  }
  
//#ifdef MLR_ODE
//  if(s->ode.isOpen){
//    s->ode.clear();
//    s->ode.createOde(s->G);
//  }
//#endif
}


Simulator::Simulator(const char* orsFile){
  s = new sSimulator;
  
  //ORS
  s->G.init(orsFile);
  /*  if(s->G.getBodyByName("rfoot")){
    s->G.reconfigureRoot(s->G.getBodyByName("rfoot"));
    s->G.calcBodyFramesFromJoints();
    }*/
  
  //G.makeLinkTree();
  makeConvexHulls(s->G.shapes);

  //OPENGL
  s->G.gl().add(glDrawPlot, &plotModule);
  
  //SWIFT
  s->G.swift().setCutoff(.5);
}

Simulator::~Simulator(){
  delete s;
}


void Simulator::watch(bool pause, const char* txt){
  if(pause) s->G.gl().watch(txt);
  else s->G.gl().update(txt);
}

void Simulator::getJointAngles(arr& q){
  q = s->G.q;
}

void Simulator::getJointAnglesAndVels(arr& q, arr& qdot){
  q = s->G.q;
  qdot = s->G.qdot;
}


uint Simulator::getJointDimension(){
  return s->G.getJointStateDimension();
}

void Simulator::setJointAngles(const arr& q, bool updateDisplay){
  s->G.setJointState(q);
  s->G.stepSwift();
  if(updateDisplay) s->G.watch(false);
}

void Simulator::setJointAnglesAndVels(const arr& q, const arr& qdot, bool updateDisplay){
  s->G.setJointState(q, qdot);
  s->G.stepSwift();
  if(updateDisplay) s->G.watch(false);
}

void Simulator::kinematicsPos(arr& y, const char* shapeName, const arr* rel){
  if(rel){
    mlr::Vector v;  v.set(rel->p);
    s->G.kinematicsPos(y, NoArr, s->G.getShapeByName(shapeName)->body, v);
  }else{
    s->G.kinematicsPos(y, NoArr, s->G.getShapeByName(shapeName)->body);
  }
}

void Simulator::kinematicsVec(arr& y, const char* shapeName, const arr* vec){
  if(vec){
    mlr::Vector v;  v.set(vec->p);
    s->G.kinematicsVec(y, NoArr, s->G.getShapeByName(shapeName)->body, v);
  }else{
    s->G.kinematicsVec(y, NoArr, s->G.getShapeByName(shapeName)->body);
  }
}

void Simulator::jacobianPos(arr& J, const char* shapeName, const arr* rel){
  if(rel){
    mlr::Vector v;  v.set(rel->p);
    s->G.kinematicsPos(NoArr, J, s->G.getShapeByName(shapeName)->body, v);
  }else{
    s->G.kinematicsPos(NoArr, J, s->G.getShapeByName(shapeName)->body);
  }
}

void Simulator::jacobianVec(arr& J, const char* shapeName, const arr* vec){
  if(vec){
    mlr::Vector v;  v.set(vec->p);
    s->G.kinematicsVec(NoArr, J, s->G.getShapeByName(shapeName)->body, v);
  }else{
    s->G.kinematicsVec(NoArr, J, s->G.getShapeByName(shapeName)->body);
  }
}

void Simulator::kinematicsCOM(arr& y){
  s->G.getCenterOfMass(y);
  y.resizeCopy(2);
}

void Simulator::jacobianCOM(arr& J){
  s->G.getComGradient(J);
  J.resizeCopy(2, J.d1);
}

void Simulator::reportProxies(){
  s->G.reportProxies();
}

void Simulator::setContactMargin(double margin){
  s->margin = margin;
}

void Simulator::kinematicsContacts(arr& y){
  s->G.kinematicsProxyCost(y, NoArr, s->margin);
}

void Simulator::jacobianContacts(arr& J){
  arr y;
  s->G.kinematicsProxyCost(y, J, s->margin);
}

double Simulator::getEnergy(){
  return s->G.getEnergy();
}


  
void Simulator::setDynamicSimulationNoise(double noise){
  s->dynamicNoise = noise;
}

void Simulator::setDynamicGravity(bool gravity){
  s->gravity = gravity;
}

void Simulator::getDynamics(arr& M, arr& F){
  s->G.equationOfMotion(M, F);
}

void Simulator::stepDynamics(const arr& Bu, double tau){
  s->G.stepDynamics(Bu, tau, s->dynamicNoise);
}

void Simulator::stepOde(const arr& qdot, double tau){
#ifdef MLR_ODE
  s->G.ode().setMotorVel(qdot, 100.);
  s->G.ode().step(tau);
  s->G.ode().importStateFromOde();
#endif
}

void Simulator::stepPhysx(const arr& qdot, double tau){
  s->G.physx().step(tau);
}

mlr::KinematicWorld& Simulator::getOrsGraph(){
  return s->G;
}

struct sVisionSimulator {
  OpenGL gl;
  arr P;
  sVisionSimulator(){ }
};



VisionSimulator::VisionSimulator(){
  s = new sVisionSimulator;
  
  s->P.resize(3, 4);
  
  //OPENGL
  s->gl.add(drawEnv, 0);
  s->gl.add(drawBase, 0);
  s->gl.setClearColors(1., 1., 1., 1.);
  s->gl.camera.setPosition(10., -15., 8.);
  s->gl.camera.focus(0, 0, 0);
  s->gl.camera.upright();
  s->gl.update();
  s->gl.add(glDrawPlot, &plotModule);
  
}

VisionSimulator::~VisionSimulator(){
  delete s;
}

void VisionSimulator::watch(){
  s->gl.watch();
}

void VisionSimulator::getRandomWorldPoints(arr& X, uint N){
  //generate N random 3D world points
  X.resize(N, 4);
  rndUniform(X, -1., 1., false);  //each point is random in [-1, 1]^4
  for(uint i=0; i<N; i++){
    X(i, 3)=1.;                 //initialize 4th coordinate to 1
  }
}

arr VisionSimulator::getCameraTranslation(){
  return conv_vec2arr(s->gl.camera.X.pos);
}

void VisionSimulator::projectWorldPointsToImagePoints(arr& x, const arr& X, double noiseInPixel){
#ifdef FREEGLUT
	uint N=X.d0;
  x.resize(N, 3);
  
  //*
  arr y(3);
  arr Mmodel(4, 4), Mproj(4, 4); intA Mview(4);
  glGetDoublev(GL_MODELVIEW_MATRIX, Mmodel.p);
  glGetDoublev(GL_PROJECTION_MATRIX, Mproj.p);
  glGetIntegerv(GL_VIEWPORT, Mview.p);
	//cout <<Mview <<endl;
  //cout <<Mmodel <<endl;
  //cout <<Mproj <<s->P <<endl;
  //*/
  intA view(4);
  glGetIntegerv(GL_VIEWPORT, view.p);

  //project the points using the OpenGL matrix
  s->P = s->gl.P;
  s->P /= s->P(0, 0);
  cout <<"VisionSimulator:"
       <<"\n  projection matrix used: " <<s->P
       <<"\n  camera position and quaternion: " <<s->gl.camera.X.pos <<"  " <<s->gl.camera.X.rot
       <<"\n  camera f=" <<.5*view(2) <<" x0=" <<view(0)+.5*view(2) <<" y0=" <<view(1)+.5*view(2)
       <<endl;
  for(uint i=0; i<N; i++){
    x[i] = s->P*X[i];
    x[i]() /= x(i, 2);
    //gluProject(X(i, 0), X(i, 1), X(i, 2), Mmodel.p, Mproj.p, Mview.p, &y(0), &y(1), &y(2));
    //cout <<"y=" <<y <<" x=" <<x[i] <<endl;
  }
  rndGauss(x, noiseInPixel, true); //add Gaussian noise
  for(uint i=0; i<N; i++) x(i, 2)=1.;
  
  plotPoints(X);
  //s->gl.watch();
#endif
}

void glDrawCarSimulator(void *classP);

CarSimulator::CarSimulator(){
  //car parameters
  x=y=theta=0;
  tau=1.; //one second time steps
  L=2.; //2 meters between the wheels
  dynamicsNoise = .03;
  observationNoise = .5;

  //landmarks
  landmarks.resize(2,2);
  rndGauss(landmarks, 10.);
  //landmarks=ARR(10,0); landmarks.reshape(1,2);
  
  gl=new OpenGL;
  gl->add(drawEnv, this);
  gl->add(glDrawCarSimulator, this);
  gl->add(glDrawPlot,&plotModule);

  gl->camera.setPosition(10., -50., 100.);
  gl->camera.focus(0, 0, .5);
  gl->camera.upright();
  gl->update();
}

void CarSimulator::step(const arr& u){
  double v=u(0), phi=u(1);
  x += tau*v*cos(theta);
  y += tau*v*sin(theta);
  theta += tau*(v/L)*tan(phi);

  if(dynamicsNoise){
    x += dynamicsNoise*rnd.gauss();
    y += dynamicsNoise*rnd.gauss();
    theta += dynamicsNoise*rnd.gauss();
  }
  
  plotClear();
  for(uint i=0;i<gaussiansToDraw.N;i++) plotCovariance(gaussiansToDraw(i).a, gaussiansToDraw(i).A);
  gl->update();
}

void CarSimulator::getRealNoisyObservation(arr& Y){
  getMeanObservationAtState(Y, ARR(x,y,theta));
  rndGauss(Y,observationNoise,true);
}

void CarSimulator::getMeanObservationAtState(arr& Y, const arr& X){
  Y=landmarks;
  arr R = ARR(cos(X(2)), -sin(X(2)), sin(X(2)), cos(X(2)));
  R.reshape(2,2);
  arr p = ones(landmarks.d0,1)*~ARR(X(0),X(1));
  Y -= p;
  Y = Y*R;
  Y.reshape(Y.N);
}

void CarSimulator::getLinearObservationModelAtState(arr& C, arr& c, const arr& X){
  uint N=landmarks.d0;
  arr R = ARR(cos(X(2)), sin(X(2)), -sin(X(2)), cos(X(2)));
  R.reshape(2,2);
  C.resize(2*N,2*N);  C.setZero();
  for(uint i=0;i<N;i++) C.setMatrixBlock(R, 2*i, 2*i);
  cout <<C <<endl;
  c.resize(2*N);
  for(uint i=0;i<N;i++) c.setVectorBlock(ARR(X(0),X(1)), 2*i);
  c = - C * c;
}

void CarSimulator::getObservationJacobianAtState(arr& dy_dx, const arr& X){
  uint N=landmarks.d0;
  dy_dx = arr(2*N,3); dy_dx.setZero();
  for (uint i=0; i<N; i++){
    arr J(2,3);J.setZero();
    //by x
    J(0,0) = -cos(X(2));
    J(1,0) = sin(X(2));
    //by y
    J(0,1) = -sin(X(2));
    J(1,1) = -cos(X(2));
    //by theta
    J(0,2) = -sin(X(2))*(landmarks(i,0)-X(0)) + cos(X(2))*(landmarks(i,1)-X(1));
    J(1,2) = -cos(X(2))*(landmarks(i,0)-X(0)) - sin(X(2))*(landmarks(i,1)-X(1));
    dy_dx[i*2] = J[0];//copy in big J
    dy_dx[i*2+1] = J[1];
  }
}

void glDrawCarSimulator(void *classP){
#ifdef FREEGLUT
  CarSimulator *s=(CarSimulator*)classP;
  mlr::Transformation f;
  f.setZero();
  f.addRelativeTranslation(s->x,s->y,.3);
  f.addRelativeRotationRad(s->theta, 0., 0., 1.);
  f.addRelativeTranslation(1.,0.,0.);

  double GLmatrix[16];
  f.getAffineMatrixGL(GLmatrix);
  glLoadMatrixd(GLmatrix);
  glColor(.8,.2,.2);
  glDrawBox(3., 1.5, .5);
  
  for(uint l=0;l<s->landmarks.d0;l++){
    f.setZero();
    f.addRelativeTranslation(s->landmarks(l,0),s->landmarks(l,1), .5);
    f.getAffineMatrixGL(GLmatrix);
    glLoadMatrixd(GLmatrix);
    glColor(.2,.8,.2);
    glDrawCylinder(.1,1.);
  }
  
  glLoadIdentity();
  glColor(.2,.2,.8);
  for(uint l=0;l<s->particlesToDraw.d0;l++){
    glPushMatrix();
    glTranslatef(s->particlesToDraw(l,0), s->particlesToDraw(l,1), .6);
    glDrawDiamond(.1, .1, .1);
    glPopMatrix();
  }

  for(uint l=0;l<s->particlesToDraw.d0;l++){
  }
#endif
}

#include <Core/array.tpp>
template mlr::Array<CarSimulator::Gaussian>& mlr::Array<CarSimulator::Gaussian>::resize(uint);
/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#include "taskMap_GJK.h"

TaskMap_GJK::TaskMap_GJK(const mlr::Shape* s1, const mlr::Shape* s2, bool exact, bool negative) : exact(exact), negScalar(negative){
  CHECK(s1 && s2,"");
  i = s1->index;
  j = s2->index;
}

TaskMap_GJK::TaskMap_GJK(const mlr::KinematicWorld& W, const char* s1, const char* s2, bool exact, bool negative) : exact(exact), negScalar(negative){
  CHECK(s1 && s2,"");
  mlr::Shape *s;
  s=W.getShapeByName(s1); CHECK(s,"shape name '" <<s1 <<"' does not exist"); i=s->index;
  s=W.getShapeByName(s2); CHECK(s,"shape name '" <<s2 <<"' does not exist"); j=s->index;
}

TaskMap_GJK::TaskMap_GJK(const mlr::KinematicWorld& W, const Graph& specs, bool exact) : exact(exact){
  Node *it;
  if((it=specs["sym2"])){ auto name=it->get<mlr::String>(); auto *s=W.getShapeByName(name); CHECK(s,"shape name '" <<name <<"' does not exist"); i=s->index; }
  if((it=specs["sym3"])){ auto name=it->get<mlr::String>(); auto *s=W.getShapeByName(name); CHECK(s,"shape name '" <<name <<"' does not exist"); j=s->index; }
//  if((it=specs["vec1"])) vec1 = mlr::Vector(it->get<arr>());  else vec1.setZero();
//  if((it=specs["vec2"])) vec2 = mlr::Vector(it->get<arr>());  else vec2.setZero();
}

void TaskMap_GJK::phi(arr& v, arr& J, const mlr::KinematicWorld& W, int t){
  mlr::Shape *s1 = i<0?NULL: W.shapes(i);
  mlr::Shape *s2 = j<0?NULL: W.shapes(j);
  CHECK(s1 && s2,"");
  CHECK(s1->sscCore.V.N,"");
  CHECK(s2->sscCore.V.N,"");
  mlr::Vector p1, p2, e1, e2;
  GJK_point_type pt1, pt2;

  GJK_sqrDistance(s1->sscCore, s2->sscCore, s1->X, s2->X, p1, p2, e1, e2, pt1, pt2);
  //  if(d2<1e-10) LOG(-1) <<"zero distance";
  arr y1, J1, y2, J2;

  W.kinematicsPos(y1, (&J?J1:NoArr), s1->body, s1->body->X.rot/(p1-s1->body->X.pos));
  W.kinematicsPos(y2, (&J?J2:NoArr), s2->body, s2->body->X.rot/(p2-s2->body->X.pos));
  v = y1 - y2;
  if(&J){
    J = J1 - J2;
    if(exact){
      if((pt1==GJK_vertex && pt2==GJK_face) || (pt1==GJK_face && pt2==GJK_vertex)){
        arr vec, Jv, n = v/length(v);
        J = n*(~n*J);
        if(pt1==GJK_vertex) W.kinematicsVec(vec, Jv, s2->body, s2->body->X.rot/(p1-p2));
        if(pt2==GJK_vertex) W.kinematicsVec(vec, Jv, s1->body, s1->body->X.rot/(p1-p2));
        J += Jv;
      }
      if(pt1==GJK_edge && pt2==GJK_edge){
        arr vec, Jv, n, a, b;
        n = v/length(v);
        J = n*(~n*J);

        W.kinematicsVec(vec, Jv, s1->body, s1->body->X.rot/e1);
        a=conv_vec2arr(e1);
        b=conv_vec2arr(e2);
        double ab=scalarProduct(a,b);
        J += (a-b*ab) * (1./(1.-ab*ab)) * (~v*(b*~b -eye(3,3))) * Jv;

        W.kinematicsVec(vec, Jv, s2->body, s2->body->X.rot/e2);
        a=conv_vec2arr(e2);
        b=conv_vec2arr(e1);
        J += (a-b*ab) * (1./(1.-ab*ab)) * (~v*(b*~b -eye(3,3))) * Jv;
      }
      if((pt1==GJK_vertex && pt2==GJK_edge) || (pt1==GJK_edge && pt2==GJK_vertex)){
        arr vec, Jv, n;
        if(pt1==GJK_vertex) n=conv_vec2arr(e2); else n=conv_vec2arr(e1);
        J = J - n*(~n*J);
        if(pt1==GJK_vertex) W.kinematicsVec(vec, Jv, s2->body, s2->body->X.rot/(p1-p2));
        if(pt2==GJK_vertex) W.kinematicsVec(vec, Jv, s1->body, s1->body->X.rot/(p1-p2));
        J += n*(~n*Jv);
      }
    }
  }
  //reduce by radii
  double l2=sumOfSqr(v), l=sqrt(l2);
  double fac = (l-s1->size(3)-s2->size(3))/l;
  if(&J){
    arr d_fac = (1.-(l-s1->size(3)-s2->size(3))/l)/l2 *(~v)*J;
    J = J*fac + v*d_fac;
  }
  v *= fac;

  if(negScalar){
    if(&J) J = ~(v/(-l))*J;
    v = ARR(-l);
  }
//  CHECK_ZERO(l2-d2, 1e-6,"");
}

/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


/**
 * @file
 * @ingroup group_ors
 */
/**
 * @addtogroup group_ors
 * @{
 */


#undef abs
#include <algorithm>
#include <sstream>
#include <climits>
#include "kin.h"
#include "kin_swift.h"
#include "kin_physx.h"
#include "kin_ode.h"
#include <Geo/qhull.h>
#include <GeoOptim/geoOptim.h>
#include <Gui/opengl.h>
#include <Algo/algos.h>
#include <iomanip>

#ifndef MLR_ORS_ONLY_BASICS
#  include <Core/graph.h>
//#  include <Plot/plot.h>
#endif

#ifdef MLR_extern_ply
#  include <Geo/ply/ply.h>
#endif

#ifdef MLR_GL
#  include <GL/gl.h>
#  include <GL/glu.h>
#endif

#define ORS_NO_DYNAMICS_IN_FRAMES

#define SL_DEBUG_LEVEL 1
#define SL_DEBUG(l, x) if(l<=SL_DEBUG_LEVEL) x;

#define Qstate

void lib_ors(){ cout <<"force loading lib/ors" <<endl; }

#define LEN .2

#ifndef MLR_ORS_ONLY_BASICS

uint mlr::KinematicWorld::setJointStateCount = 0;

//===========================================================================
//
// contants
//

mlr::Body& NoBody = *((mlr::Body*)NULL);
mlr::Shape& NoShape = *((mlr::Shape*)NULL);
mlr::Joint& NoJoint = *((mlr::Joint*)NULL);
mlr::KinematicWorld& NoWorld = *((mlr::KinematicWorld*)NULL);

template<> const char* mlr::Enum<mlr::ShapeType>::names []={
  "ST_box", "ST_sphere", "ST_capsule", "ST_mesh", "ST_cylinder", "ST_marker", "ST_SSBox", "ST_pointCloud", "ST_ssCvx", "ST_ssBox", NULL
};

template<> const char* mlr::Enum<mlr::JointType>::names []={
  "JT_hingeX", "JT_hingeY", "JT_hingeZ", "JT_transX", "JT_transY", "JT_transZ", "JT_transXY", "JT_trans3", "JT_transXYPhi", "JT_universal", "JT_rigid", "JT_quatBall", "JT_phiTransXY", "JT_glue", "JT_free", NULL
};

template<> const char* mlr::Enum<mlr::KinematicSwitch::OperatorSymbol>::names []={
  "deleteJoint", "addJointZero", "addJointAtFrom", "addJointAtTo", "addArticulated", NULL
};

//===========================================================================
//
// Body implementations
//

//mlr::Body::Body() { reset(); }

//mlr::Body::Body(const Body& b) { reset(); *this=b; }

mlr::Body::Body(KinematicWorld& _world, const Body* copyBody):world(_world), vel(0), angvel(0) {
  reset();
  index=world.bodies.N;
  world.bodies.append(this);
  if(copyBody) *this=*copyBody;
}

mlr::Body::~Body() {
  reset();
  while(inLinks.N) delete inLinks.last();
  while(outLinks.N) delete outLinks.last();
  while(shapes.N) delete shapes.last();
  world.bodies.removeValue(this);
  listReindex(world.bodies);
}

void mlr::Body::reset() {
  ats.clear();
  X.setZero();
  type=BT_dynamic;
  shapes.memMove=true;
  com.setZero();
  mass = 0.;
  inertia.setZero();
  vel.setZero();
  angvel.setZero();
}

void mlr::Body::parseAts() {
  //interpret some of the attributes
  arr x;
  mlr::String str;
  ats.get(X, "X");
  ats.get(X, "pose");
  
  //mass properties
  double d;
  if(ats.get(d, "mass")) {
    mass=d;
    inertia.setId();
    inertia *= .2*d;
  }

  type=BT_dynamic;
  if(ats["fixed"])       type=BT_static;
  if(ats["static"])      type=BT_static;
  if(ats["kinematic"])   type=BT_kinematic;
  if(ats.get(d,"dyntype")) type=(BodyType)d;

  // SHAPE handling //TODO: remove this code!
  Node* item;
  // a mesh which consists of multiple convex sub meshes creates multiple
  // shapes that belong to the same body
  item = ats.getNode("meshes");
  if(item){
    HALT("this is deprecated");
    mlr::FileToken *file = item->getValue<mlr::FileToken>();
    CHECK(file,"somethings wrong");

    // if mesh is not .obj we only have one shape
    if(!file->name.endsWith("obj")) {
      new Shape(world, *this);
    }else{  // if .obj file create Shape for all submeshes
      auto subMeshPositions = getSubMeshPositions(file->name);
      for(uint i=0;i<subMeshPositions.d0;i++){
        auto parsing_pos = subMeshPositions[i];
        Shape *s = new Shape(world, *this);
        s->mesh.parsing_pos_start = parsing_pos(0);
        s->mesh.parsing_pos_end = parsing_pos(1);
	//TODO: use Shape::parseAts instead of doing the same things here again!!
        s->mesh.readObjFile(file->getIs()); 
        s->type=ST_mesh;
      }
    }
  }

  // add shape if there is no shape exists yet
  if(ats.getNode("type") && !shapes.N){
    Shape *s = new Shape(world, *this);
    s->name = name;
  }

  // copy body attributes to shapes 
  for(Shape *s:shapes) { s->ats=ats;  s->parseAts(); }
  //TODO check if this works! coupled to the listDelete below
  Node *it=ats["type"]; if(it){ delete it; }
}

void mlr::Body::write(std::ostream& os) const {
  if(!X.isZero()) os <<"pose=<T " <<X <<" > ";
  if(mass) os <<"mass=" <<mass <<' ';
  if(type!=BT_dynamic) os <<"dyntype=" <<(int)type <<' ';
//  uint i; Node *a;
//  for(Type *  a:  ats)
//      if(a->keys(0)!="X" && a->keys(0)!="pose") os <<*a <<' ';
}

void mlr::Body::read(std::istream& is) {
  reset();
  ats.read(is);
  if(!is.good()) HALT("body '" <<name <<"' read error: in ");
  parseAts();
}

namespace mlr {
std::ostream& operator<<(std::ostream& os, const Body& x) { x.write(os); return os; }
std::ostream& operator<<(std::ostream& os, const Shape& x) { x.write(os); return os; }
std::ostream& operator<<(std::ostream& os, const Joint& x) { x.write(os); return os; }
}

//===========================================================================
//
// Shape implementations
//

mlr::Shape::Shape(KinematicWorld &_world, Body& b, const Shape *copyShape, bool referenceMeshOnCopy): world(_world), /*ibody(UINT_MAX),*/ body(NULL) {
  reset();
  CHECK(&world,"you need at least a world to attach this shape to!");
  index=world.shapes.N;
  world.shapes.append(this);
  if(&b){
    body = &b;
    b.shapes.append(this);
  }
  if(copyShape) copy(*copyShape, referenceMeshOnCopy);
}

mlr::Shape::~Shape() {
  reset();
  if(body){
    body->shapes.removeValue(this);
    listReindex(body->shapes);
  }
  world.shapes.removeValue(this);
  listReindex(world.shapes);
}

void mlr::Shape::copy(const Shape& s, bool referenceMeshOnCopy){
  name=s.name; X=s.X; rel=s.rel; type=s.type;
  size=s.size;
  if(!referenceMeshOnCopy){
    mesh=s.mesh;
    sscCore=s.sscCore;
  }else{
    mesh.V.referTo(s.mesh.V);
    mesh.T.referTo(s.mesh.T);
    mesh.C.referTo(s.mesh.C);
    mesh.Vn.referTo(s.mesh.Vn);
    sscCore.V.referTo(s.sscCore.V);
    sscCore.T.referTo(s.sscCore.T);
    sscCore.C.referTo(s.sscCore.C);
    sscCore.Vn.referTo(s.sscCore.Vn);
  }
  mesh_radius=s.mesh_radius; cont=s.cont;
  ats=s.ats;
}

void mlr::Shape::parseAts() {
  double d;
  arr x;
  mlr::String str;
  mlr::FileToken fil;
  ats.get(rel, "rel");
  ats.get(size, "size");
  if(ats.get(mesh.C, "color")){
    CHECK(mesh.C.N==3 || mesh.C.N==4,"");
//    if(x.N==3){ memmove(color, x.p, 3*sizeof(double)); color[3]=1.; }
    //    else memmove(color, x.p, 4*sizeof(double));
  }
  if(ats.get(d, "type"))       { type=(ShapeType)(int)d;}
  else if(ats.get(str, "type")) { str>> type; }
  if(ats["contact"])           { cont=true; }
  if(ats.get(fil, "mesh"))     { mesh.read(fil.getIs(), fil.name.getLastN(3).p, fil.name); }
  if(ats.get(d, "meshscale"))  { mesh.scale(d); }

  //create mesh for basic shapes
  switch(type) {
    case mlr::ST_none: HALT("shapes should have a type - somehow wrong initialization..."); break;
    case mlr::ST_box:
      mesh.setBox();
      mesh.scale(size(0), size(1), size(2));
      break;
    case mlr::ST_sphere:
      mesh.setSphere();
      mesh.scale(size(3), size(3), size(3));
      break;
    case mlr::ST_cylinder:
      CHECK(size(3)>1e-10,"");
      mesh.setCylinder(size(3), size(2));
      break;
    case mlr::ST_capsule:
      CHECK(size(3)>1e-10,"");
//      mesh.setCappedCylinder(size(3), size(2));
      sscCore.setBox();
      sscCore.scale(0., 0., size(2));
      mesh.setSSCvx(sscCore, size(3));
      break;
    case mlr::ST_retired_SSBox:
      HALT("deprecated?");
      mesh.setSSBox(size(0), size(1), size(2), size(3));
      break;
    case mlr::ST_marker:
      break;
    case mlr::ST_mesh:
    case mlr::ST_pointCloud:
      CHECK(mesh.V.N, "mesh needs to be loaded to draw mesh object");
      sscCore = mesh;
      sscCore.makeConvexHull();
      break;
    case mlr::ST_ssCvx:
      CHECK(size(3)>1e-10,"");
      CHECK(mesh.V.N, "mesh needs to be loaded to draw mesh object");
      sscCore=mesh;
      mesh.setSSCvx(sscCore, size(3));
      break;
    case mlr::ST_ssBox:
      CHECK(size.N==4 && size(3)>1e-10,"");
      sscCore.setBox();
      sscCore.scale(size(0)-2.*size(3), size(1)-2.*size(3), size(2)-2.*size(3));
      mesh.setSSBox(size(0), size(1), size(2), size(3));
//      mesh.setSSCvx(sscCore, size(3));
      break;
    default: NIY;
  }

  //center the mesh:
  if(type==mlr::ST_mesh && mesh.V.N){
    Vector c = mesh.center();
    if(c.length()>1e-8 && !ats["rel_includes_mesh_center"]){
      rel.addRelativeTranslation(c);
      ats.newNode<bool>({"rel_includes_mesh_center"}, {}, true);
    }
  }

  //compute the bounding radius
  if(mesh.V.N) mesh_radius = mesh.getRadius();

  //colored box?
  if(ats["coloredBox"]){
    CHECK_EQ(mesh.V.d0, 8, "I need a box");
    arr col=mesh.C;
    mesh.C.resize(mesh.T.d0, 3);
    for(uint i=0;i<mesh.C.d0;i++){
      if(i==2 || i==3) mesh.C[i] = col; //arr(color, 3);
      else if(i>=4 && i<=7) mesh.C[i] = 1.;
      else mesh.C[i] = .5;
    }
  }


  //add inertia to the body
  if(body) {
    Matrix I;
    double mass=-1.;
    switch(type) {
      case ST_sphere:   inertiaSphere(I.p(), mass, 1000., size(3));  break;
      case ST_box:      inertiaBox(I.p(), mass, 1000., size(0), size(1), size(2));  break;
      case ST_capsule:
      case ST_cylinder: inertiaCylinder(I.p(), mass, 1000., size(2), size(3));  break;
      case ST_none:
      default: ;
    }
    if(mass>0.){
      body->mass += mass;
      body->inertia += I;
    }
  }
}

void mlr::Shape::reset() {
  type=ST_none;
  size = {1.,1.,1.};
  ats.clear();
  X.setZero();
  rel.setZero();
  mesh.V.clear();
  mesh.C.clear();
  mesh.C = consts<double>(.8, 3); //color[0]=color[1]=color[2]=.8; color[3]=1.;
  mesh_radius=0.;
  cont=false;
}

void mlr::Shape::write(std::ostream& os) const {
  os <<"type=" <<type <<' ';
//  os <<"size=[" <<size(0) <<' '<<size(1) <<' '<<size(2) <<' '<<size(3) <<"] ";
  if(!rel.isZero()) os <<"rel=<T " <<rel <<" > ";
  for(Node * a: ats)
    if(a->keys.N && a->keys(0)!="rel" && a->keys(0)!="type" /*&& a->keys(0)!="size"*/) os <<*a <<' ';
}

void mlr::Shape::read(std::istream& is) {
  reset();
  ats.read(is);
  if(!is.good()) HALT("shape read error");
  parseAts();
}

#ifdef MLR_GL
void mlr::Shape::glDraw(OpenGL& gl) {
  //set name (for OpenGL selection)
  glPushName((index <<2) | 1);
  if(world.orsDrawColors && !world.orsDrawIndexColors) glColor(mesh.C); //color[0], color[1], color[2], color[3]*world.orsDrawAlpha);
  if(world.orsDrawIndexColors) glColor3b((index>>16)&0xff, (index>>8)&0xff, index&0xff);


  double GLmatrix[16];
  X.getAffineMatrixGL(GLmatrix);
  glLoadMatrixd(GLmatrix);

  if(!world.orsDrawShapes) {
    double scale=.33*(size(0)+size(1)+size(2) + 2.*size(3)); //some scale
    if(!scale) scale=1.;
    scale*=.3;
    glDrawAxes(scale);
    glColor(0, 0, .5);
    glDrawSphere(.1*scale);
  }
  if(world.orsDrawShapes) {
    switch(type) {
      case mlr::ST_none: LOG(-1) <<"Shape '" <<name <<"' has no joint type";  break;
      case mlr::ST_box:
        if(world.orsDrawCores && sscCore.V.N) sscCore.glDraw(gl);
        else if(world.orsDrawMeshes && mesh.V.N) mesh.glDraw(gl);
        else glDrawBox(size(0), size(1), size(2));
        break;
      case mlr::ST_sphere:
        if(world.orsDrawCores && sscCore.V.N) sscCore.glDraw(gl);
        else if(world.orsDrawMeshes && mesh.V.N) mesh.glDraw(gl);
        else glDrawSphere(size(3));
        break;
      case mlr::ST_cylinder:
        if(world.orsDrawCores && sscCore.V.N) sscCore.glDraw(gl);
        else if(world.orsDrawMeshes && mesh.V.N) mesh.glDraw(gl);
        else glDrawCylinder(size(3), size(2));
        break;
      case mlr::ST_capsule:
        if(world.orsDrawCores && sscCore.V.N) sscCore.glDraw(gl);
        else if(world.orsDrawMeshes && mesh.V.N) mesh.glDraw(gl);
        else glDrawCappedCylinder(size(3), size(2));
        break;
      case mlr::ST_retired_SSBox:
        HALT("deprecated??");
        if(world.orsDrawCores && sscCore.V.N) sscCore.glDraw(gl);
        else if(world.orsDrawMeshes){
          if(!mesh.V.N) mesh.setSSBox(size(0), size(1), size(2), size(3));
          mesh.glDraw(gl);
        }else NIY;
        break;
      case mlr::ST_marker:
        if(world.orsDrawMarkers){
          glDrawDiamond(size(0)/5., size(0)/5., size(0)/5.); glDrawAxes(size(0));
        }
        break;
      case mlr::ST_mesh:
        CHECK(mesh.V.N, "mesh needs to be loaded to draw mesh object");
        if(world.orsDrawCores && sscCore.V.N) sscCore.glDraw(gl);
        else mesh.glDraw(gl);
        break;
      case mlr::ST_ssCvx:
        CHECK(sscCore.V.N, "sscCore needs to be loaded to draw mesh object");
        if(!mesh.V.N) mesh.setSSCvx(sscCore, size(3));
        if(world.orsDrawCores && sscCore.V.N) sscCore.glDraw(gl);
        else mesh.glDraw(gl);
        break;
      case mlr::ST_ssBox:
        if(!mesh.V.N || !sscCore.V.N){
          sscCore.setBox();
          sscCore.scale(size(0)-2.*size(3), size(1)-2.*size(3), size(2)-2.*size(3));
          mesh.setSSCvx(sscCore, size(3));
        }
        if(world.orsDrawCores && sscCore.V.N) sscCore.glDraw(gl);
        else mesh.glDraw(gl);
        break;
      case mlr::ST_pointCloud:
        CHECK(mesh.V.N, "mesh needs to be loaded to draw point cloud object");
        if(world.orsDrawCores && sscCore.V.N) sscCore.glDraw(gl);
        else mesh.glDraw(gl);
        break;

      default: HALT("can't draw that geom yet");
    }
  }
  if(world.orsDrawZlines) {
    glColor(0, .7, 0);
    glBegin(GL_LINES);
    glVertex3d(0., 0., 0.);
    glVertex3d(0., 0., -X.pos.z);
    glEnd();
  }

  if(world.orsDrawBodyNames && body){
    glColor(1,1,1);
    glDrawText(body->name, 0, 0, 0);
  }

  glPopName();
}
#endif

uintA stringListToShapeIndices(const mlr::Array<const char*>& names, const mlr::Array<mlr::Shape*>& shapes) {
  uintA I(names.N);
  for(uint i=0; i<names.N; i++) {
    mlr::Shape *s = listFindByName(shapes, names(i));
    if(!s) HALT("shape name '"<<names(i)<<"' doesn't exist");
    I(i) = s->index;
  }
  return I;
}

uintA shapesToShapeIndices(const mlr::Array<mlr::Shape*>& shapes) {
  uintA I;
  resizeAs(I, shapes);
  for(uint i=0; i<shapes.N; i++) I.elem(i) = shapes.elem(i)->index;
  return I;
}

void makeConvexHulls(ShapeL& shapes, bool onlyContactShapes){
  for(mlr::Shape *s: shapes) if(!onlyContactShapes || s->cont) s->mesh.makeConvexHull();
}

void computeOptimalSSBoxes(ShapeL& shapes){
//  for(mlr::Shape *s: shapes) s->mesh.computeOptimalSSBox(s->mesh.V);
  for(uint i=0;i<shapes.N;i++){
    mlr::Shape *s=shapes(i);
    if(!(s->type==mlr::ST_mesh && s->mesh.V.N)) continue;
    mlr::Transformation t;
    arr x;
    computeOptimalSSBox(s->mesh, x, t, s->mesh.V);
    s->type = mlr::ST_ssBox;
    s->size(0)=2.*x(0); s->size(1)=2.*x(1); s->size(2)=2.*x(2); s->size(3)=x(3);
    s->mesh.setSSBox(s->size(0), s->size(1), s->size(2), s->size(3));
    s->rel.appendTransformation(t);
  }
}

void computeMeshNormals(ShapeL& shapes){
  for(mlr::Shape *s: shapes){
    if(s->mesh.V.d0!=s->mesh.Vn.d0 || s->mesh.T.d0!=s->mesh.Tn.d0) s->mesh.computeNormals();
    if(s->sscCore.V.d0!=s->sscCore.Vn.d0 || s->sscCore.T.d0!=s->sscCore.Tn.d0) s->sscCore.computeNormals();
  }
}


//===========================================================================
//
// Joint implementations
//

bool always_unlocked(void*) { return false; }

mlr::Joint::Joint(KinematicWorld& G, Body *f, Body *t, const Joint* copyJoint)
  : world(G), index(0), qIndex(UINT_MAX), from(f), to(t), mimic(NULL), agent(0), constrainToZeroVel(false), q0(0.), H(1.) {
  reset();
  if(copyJoint) *this=*copyJoint;
  index=world.joints.N;
  world.joints.append(this);
  f->outLinks.append(this);
  t-> inLinks.append(this);
  world.q.clear();
  world.qdot.clear();
  world.qdim.clear();
}

mlr::Joint::~Joint() {
  world.checkConsistency();
  reset();
  if(from){ from->outLinks.removeValue(this); listReindex(from->outLinks); }
  if(to){   to->inLinks.removeValue(this); listReindex(to->inLinks); }
  world.joints.removeValue(this);
  listReindex(world.joints);
  world.q.clear();
  world.qdot.clear();
  world.qdim.clear();
}

void mlr::Joint::reset() { 
  ats.clear(); A.setZero(); B.setZero(); Q.setZero(); X.setZero(); axis.setZero(); limits.clear(); q0=0.; H=1.; type=JT_none;
//  locker=NULL;
}

void mlr::Joint::parseAts() {
  //interpret some of the attributes
  double d=0.;
  mlr::String str;
  ats.get(A, "A");
  ats.get(A, "from");
  if(ats["BinvA"]) B.setInverse(A);
  ats.get(B, "B");
  ats.get(B, "to");
  ats.get(Q, "Q");
  ats.get(X, "X");
  ats.get(H, "ctrl_H");
  if(ats.get(d, "type")) type=(JointType)d;
  else if(ats.get(str, "type")) { str>> type; }
  else type=JT_hingeX;
  if(type==JT_rigid && !Q.isZero()){ A.appendTransformation(Q); Q.setZero(); }
  if(ats.get(d, "q")){
    q0=d;
    if(type==JT_hingeX) Q.addRelativeRotationRad(d, 1., 0., 0.);
    if(type==JT_rigid)  A.addRelativeRotationRad(d, 1., 0., 0.);
    if(type==JT_transX) Q.addRelativeTranslation(d, 0., 0.);
  }else q0=0.;
  if(ats.get(d, "agent")) agent=(uint)d;
  if(ats["fixed"]) agent=UINT_MAX;
  //axis
  arr axis;
  ats.get(axis, "axis");
  if(axis.N) {
    CHECK_EQ(axis.N,3,"");
    Vector ax(axis);
    Transformation f;
    f.setZero();
    f.rot.setDiff(Vector_x, ax);
    A = A * f;
    B = -f * B;
  }
  //limit
  arr ctrl_limits;
  ats.get(limits, "limits");
  if(limits.N && type!=JT_rigid && !mimic){
    CHECK_EQ(limits.N,2*qDim(), "parsed limits have wrong dimension");
  }
  ats.get(ctrl_limits, "ctrl_limits");
  if(ctrl_limits.N && type!=JT_rigid){
    if(!limits.N) limits.resizeAs(ctrl_limits).setZero();
    CHECK_EQ(3,ctrl_limits.N, "parsed ctrl_limits have wrong dimension");
    limits.append(ctrl_limits);
  }
  //coupled to another joint requires post-processing by the Graph::read!!
  if(ats["mimic"]) mimic=(Joint*)1;
}

uint mlr::Joint::qDim() {
  if(mimic) return 0;
  if(type>=JT_hingeX && type<=JT_transZ) return 1;
  if(type==JT_transXY) return 2;
  if(type==JT_transXYPhi) return 3;
  if(type==JT_phiTransXY) return 3;
  if(type==JT_trans3) return 3;
  if(type==JT_universal) return 2;
  if(type==JT_quatBall) return 4;
  if(type==JT_free) return 7;
  if(type==JT_glue || type==JT_rigid || type==JT_none) return 0;
  HALT("shouldn't be here");
  return 0;
}

void mlr::Joint::applyTransformation(mlr::Transformation& f, const arr& q){
  switch(type) {
    case JT_hingeX:{
//      f.addRelativeRotationRad(q.elem(qIndex),1.,0.,0.);
      f.rot.addX(q.elem(qIndex));
    } break;

    case JT_hingeY: {
//      f.addRelativeRotationRad(q.elem(qIndex),0.,1.,0.);
      f.rot.addY(q.elem(qIndex));
    } break;

    case JT_hingeZ: {
//      f.addRelativeRotationRad(q.elem(qIndex),0.,0.,1.);
      f.rot.addZ(q.elem(qIndex));
    } break;

    case JT_universal:{
      f.addRelativeRotationRad(q.elem(qIndex),1.,0.,0.);
      f.addRelativeRotationRad(q.elem(qIndex+1),0.,1.,0.);
    } break;

    case JT_quatBall:{
      mlr::Quaternion rot;
      rot.set(q.p+qIndex);
      {
          double n=rot.normalization();
          if(n<.5 || n>2.) LOG(-1) <<"quat normalization is extreme: " <<n <<endl;
      }
      rot.normalize();
      rot.isZero=false;
      f.addRelativeRotation(rot);
    } break;

    case JT_free:{
      mlr::Transformation t;
      t.pos.set(q.p+qIndex);
      t.rot.set(q.p+qIndex+3);
      {
          double n=t.rot.normalization();
          if(n<.5 || n>2.) LOG(-1) <<"quat normalization is extreme: " <<n <<endl;
      }
      t.rot.normalize();
      t.rot.isZero=false;
      f.appendTransformation(t);
    } break;

    case JT_transX: {
      f.addRelativeTranslation(q.elem(qIndex),0.,0.);
    } break;

    case JT_transY: {
      f.addRelativeTranslation(0., q.elem(qIndex), 0.);
    } break;

    case JT_transZ: {
      f.addRelativeTranslation(0., 0., q.elem(qIndex));
    } break;

    case JT_transXY: {
      f.addRelativeTranslation(q.elem(qIndex), q.elem(qIndex+1), 0.);
    } break;

    case JT_trans3: {
      f.addRelativeTranslation(q.elem(qIndex), q.elem(qIndex+1), q.elem(qIndex+2));
    } break;

    case JT_transXYPhi: {
      f.addRelativeTranslation(q.elem(qIndex), q.elem(qIndex+1), 0.);
      f.addRelativeRotationRad(q.elem(qIndex+2),0.,0.,1.);
    } break;

    case JT_phiTransXY: {
      f.addRelativeRotationRad(q.elem(qIndex+2),0.,0.,1.);
      f.addRelativeTranslation(q.elem(qIndex), q.elem(qIndex+1), 0.);
    } break;

    case JT_glue:
    case JT_rigid:
      break;
    default: NIY;
  }
}

void mlr::Joint::makeRigid(){
    A.appendTransformation(Q);
    Q.setZero();
    type = JT_rigid;
}

void mlr::Joint::write(std::ostream& os) const {
  os <<"type=" <<type <<' ';
  if(!A.isZero()) os <<"from=<T " <<A <<" > ";
  if(!B.isZero()) os <<"to=<T " <<B <<" > ";
  if(!Q.isZero()) os <<"Q=<T " <<Q <<" > ";
  for(Node * a: ats)
  if(a->keys(0)!="A" && a->keys(0)!="from"
      && a->keys(0)!="axis" //because this was subsumed in A during read
      && a->keys(0)!="B" && a->keys(0)!="to"
      && a->keys(0)!="Q" && a->keys(0)!="q"
      && a->keys(0)!="type") os <<*a <<' ';
}

void mlr::Joint::read(std::istream& is) {
  reset();
  ats.read(is);
  if(!is.good()) HALT("joint (" <<from->name <<' ' <<to->name <<") read read error");
  parseAts();
}


mlr::Proxy::Proxy() {
  colorCode = 0;
}

void mlr::Proxy::glDraw(OpenGL& gl){
  glLoadIdentity();
  if(!colorCode){
    if(d>0.) glColor(.8,.2,.2);
    else glColor(1,0,0);
  }else glColor(colorCode);
  glBegin(GL_LINES);
  glVertex3dv(posA.p());
  glVertex3dv(posB.p());
  glEnd();
  mlr::Transformation f;
  f.pos=posA;
  f.rot.setDiff(mlr::Vector(0, 0, 1), posA-posB);
  double GLmatrix[16];
  f.getAffineMatrixGL(GLmatrix);
  glLoadMatrixd(GLmatrix);
  glDisable(GL_CULL_FACE);
  glDrawDisk(.02);
  glEnable(GL_CULL_FACE);

  f.pos=posB;
  f.getAffineMatrixGL(GLmatrix);
  glLoadMatrixd(GLmatrix);
  glDrawDisk(.02);
}

//===========================================================================
//
// Graph implementations
//

namespace mlr{
  struct sKinematicWorld{
    OpenGL *gl;
    SwiftInterface *swift;
    PhysXInterface *physx;
    OdeInterface *ode;
    bool swiftIsReference;
    sKinematicWorld():gl(NULL), swift(NULL), physx(NULL), ode(NULL), swiftIsReference(false) {}
    ~sKinematicWorld(){
      if(gl) delete gl;
      if(swift && !swiftIsReference) delete swift;
      if(physx) delete physx;
      if(ode) delete ode;
    }
  };
}

mlr::KinematicWorld::KinematicWorld():s(NULL),q_agent(0),isLinkTree(false) {
  bodies.memMove=joints.memMove=shapes.memMove=proxies.memMove=true;
  s=new sKinematicWorld;
}

mlr::KinematicWorld::KinematicWorld(const mlr::KinematicWorld& other):s(NULL),q_agent(0),isLinkTree(false)  {
  bodies.memMove=joints.memMove=shapes.memMove=proxies.memMove=true;
  s=new sKinematicWorld;
  copy( other );
}

mlr::KinematicWorld::KinematicWorld(const char* filename):s(NULL),q_agent(0),isLinkTree(false)  {
  bodies.memMove=joints.memMove=shapes.memMove=proxies.memMove=true;
  s=new sKinematicWorld;
  init(filename);
}
mlr::KinematicWorld::~KinematicWorld() {
  clear();
  delete s;
  s=NULL;
}

void mlr::KinematicWorld::init(const char* filename) {
  *this <<FILE(filename);
}

void mlr::KinematicWorld::clear() {
  qdim.clear();
  q.clear();
  qdot.clear();
  listDelete(proxies); checkConsistency();
  while(shapes.N){ delete shapes.last(); checkConsistency(); }
  while(joints.N){ delete joints.last(); checkConsistency();}
  while(bodies.N){ delete bodies.last(); checkConsistency();}
  isLinkTree=false;
}

void mlr::KinematicWorld::copy(const mlr::KinematicWorld& G, bool referenceMeshesAndSwiftOnCopy) {
  clear();
#if 1
  listCopy(proxies, G.proxies);
  for(Body *b:G.bodies) new Body(*this, b);
  for(Shape *s:G.shapes){
//    if(referenceMeshesAndSwiftOnCopy && !s->mesh.Vn.N) s->mesh.computeNormals(); // the copy references these normals -> if they're not precomputed, you can never display the copy
    new Shape(*this, (s->body?*bodies(s->body->index):NoBody), s, referenceMeshesAndSwiftOnCopy);
  }
  for(Joint *j:G.joints){
    Joint *jj=
        new Joint(*this, bodies(j->from->index), bodies(j->to->index), j);
    if(j->mimic) jj->mimic = joints(j->mimic->index);
  }
  if(referenceMeshesAndSwiftOnCopy){
    s->swift = G.s->swift;
    s->swiftIsReference=true;
  }
  q = G.q;
  qdot = G.qdot;
  qdim = G.qdim;
  q_agent = G.q_agent;
  isLinkTree = G.isLinkTree;
#else
  q = G.q;
  qdot = G.qdot;
  qdim = G.qdim;
  q_agent = G.q_agent;
  isLinkTree = G.isLinkTree;
  listCopy(proxies, G.proxies);
  listCopy(joints, G.joints);
  for(Joint *j: joints) if(j->mimic){
    mlr::String jointName;
    bool good = j->ats.find<mlr::String>(jointName, "mimic");
    CHECK(good, "something is wrong");
    j->mimic = listFindByName(G.joints, jointName);
    if(!j->mimic) HALT("The joint '" <<*j <<"' is declared coupled to '" <<jointName <<"' -- but that doesn't exist!");
    j->type = j->mimic->type;
  }
  listCopy(shapes, G.shapes);
  listCopy(bodies, G.bodies);
  graphMakeLists(bodies, joints);
  for(Body *  b:  bodies) b->shapes.clear();
  for(Shape *  s:  shapes) {
    b=bodies(s->ibody);
    s->body=b;
    b->shapes.append(s);
  }
#endif
}

/** @brief transforms (e.g., translates or rotates) the joints coordinate system):
  `adds' the transformation f to A and its inverse to B */
void mlr::KinematicWorld::transformJoint(mlr::Joint *e, const mlr::Transformation &f) {
  e->A = e->A * f;
  e->B = -f * e->B;
}

void mlr::KinematicWorld::makeLinkTree() {
//  for(Shape *s: shapes) {
//    if(s->mesh.V.N) s->mesh.transform(s->rel);
//    if(s->sscCore.V.N) s->sscCore.transform(s->rel);
//    s->rel.setZero();
//  }
  for(Joint *j: joints) {
    for(Shape *s: j->to->shapes)  s->rel = j->B * s->rel;
    for(Joint *j2: j->to->outLinks) j2->A = j->B * j2->A;
    j->B.setZero();
  }
  isLinkTree=true;
}

void mlr::KinematicWorld::jointSort(){
  BodyL order = graphGetTopsortOrder(bodies, joints);
  JointL joints_new;
  for(mlr::Body *b: order) if(b->inLinks.N) joints_new.append(b->inLinks.scalar());
  CHECK_EQ(joints_new.N, joints.N, "");
  joints = joints_new;
  for_list(mlr::Joint, j, joints) j->index=j_COUNT;
  qdim.clear();
  q.clear();
  qdot.clear();
  analyzeJointStateDimensions();
}

/** @brief KINEMATICS: given the (absolute) frames of root nodes and the relative frames
    on the edges, this calculates the absolute frames of all other nodes (propagating forward
    through trees and testing consistency of loops). */
void mlr::KinematicWorld::calc_fwdPropagateFrames() {
  for(Body *b: bodies) {
    for(Joint *j:b->outLinks){ //this has no bailout for loopy graphs!
      j->X = b->X;
      j->X.appendTransformation(j->A);
      if(j->type==JT_hingeX || j->type==JT_transX)  j->axis = j->X.rot.getX();
      if(j->type==JT_hingeY || j->type==JT_transY)  j->axis = j->X.rot.getY();
      if(j->type==JT_hingeZ || j->type==JT_transZ)  j->axis = j->X.rot.getZ();
      if(j->type==JT_transXYPhi)  j->axis = j->X.rot.getZ();
      if(j->type==JT_phiTransXY)  j->axis = j->X.rot.getZ();

      j->to->X=j->X;
#if 1
      j->to->X.appendTransformation(j->Q);
#else
      j->applyTransformation(j->to->X, q);
#endif
      CHECK_EQ(j->to->X.pos.x, j->to->X.pos.x, "");
      if(!isLinkTree) j->to->X.appendTransformation(j->B);
    }
  }
  calc_fwdPropagateShapeFrames();
}

void mlr::KinematicWorld::calc_fwdPropagateShapeFrames() {
  for(Shape *s: shapes) {
    if(s->body){
      s->X = s->body->X;
      /*if(!isLinkTree)*/ s->X.appendTransformation(s->rel);
    }else{
      s->X = s->rel;
    }
  }
}

void mlr::KinematicWorld::calc_fwdPropagateVelocities(){
  mlr::Transformation f;
  BodyL todoBodies = bodies;
  Vector q_vel, q_angvel;
  for(Body *b: todoBodies) {
    for(Joint *j:b->outLinks){ //this has no bailout for loopy graphs!

      Body *to = j->to;
      to->vel = b->vel;
      to->angvel = b->angvel;

      if(j->type==JT_hingeX){
        q_vel.setZero();
        q_angvel.set(qdot(j->qIndex) ,0., 0.);
      }else if(j->type==JT_transX){
        q_vel.set(qdot(j->qIndex), 0., 0.);
        q_angvel.setZero();
      }else if(j->type==JT_rigid){
        q_vel.setZero();
        q_angvel.setZero();
      }else if(j->type==JT_transXYPhi){
        q_vel.set(qdot(j->qIndex), qdot(j->qIndex+1), 0.);
        q_angvel.set(0.,0.,qdot(j->qIndex+2));
      }else NIY;

      Matrix R = j->X.rot.getMatrix();
      Vector qV(R*q_vel); //relative vel in global coords
      Vector qW(R*q_angvel); //relative ang vel in global coords
      to->vel += b->angvel^(to->X.pos - b->X.pos);
      if(!isLinkTree) to->vel += qW^(to->X.pos - j->X.pos);
      to->vel += qV;
      to->angvel += qW;
      todoBodies.setAppend(j->to);
    }
  }
}

void mlr::KinematicWorld::calc_missingAB_from_BodyAndJointFrames() {
  for(Joint *e: joints) {
    if(!e->X.isZero() && e->A.isZero() && e->B.isZero()) {
      e->A.setDifference(e->from->X, e->X);
      e->B.setDifference(e->X, e->to->X);
    }
  }
}

/** @brief given the absolute frames of all nodes and the two rigid (relative)
    frames A & B of each edge, this calculates the dynamic (relative) joint
    frame X for each edge (which includes joint transformation and errors) */
void mlr::KinematicWorld::calc_Q_from_BodyFrames() {
  for(Joint *j:joints) {
    mlr::Transformation A(j->from->X), B(j->to->X);
    A.appendTransformation(j->A);
    B.appendInvTransformation(j->B);
    j->Q.setDifference(A, B);
  }
}

/** @brief in all edge frames: remove any displacements, velocities and non-x rotations.
    After this, edges and nodes are not coherent anymore. You might want to call
    calcBodyFramesFromJoints() */
void mlr::KinematicWorld::clearJointErrors() {
  mlr::Vector xaxis(1, 0, 0);
  for(Joint *j:joints) {
    j->Q.rot.alignWith(xaxis);
  }
}

arr mlr::KinematicWorld::naturalQmetric(double power) const {
  HALT("don't use this anymore. use getHmetric instead");
#if 0
  if(!q.N) getJointStateDimension();
  arr Wdiag(q.N);
  Wdiag=1.;
  return Wdiag;
#else
  //compute generic q-metric depending on tree depth
  arr BM(bodies.N);
  BM=1.;
  for(uint i=BM.N; i--;) {
    for(uint j=0; j<bodies(i)->outLinks.N; j++) {
      BM(i) = mlr::MAX(BM(bodies(i)->outLinks(j)->to->index)+1., BM(i));
//      BM(i) += BM(bodies(i)->outLinks(j)->to->index);
    }
  }
  if(!q.N) getJointStateDimension();
  arr Wdiag(q.N);
  for(Joint *j: joints) {
    for(uint i=0; i<j->qDim(); i++) {
      if(j->agent==q_agent) Wdiag(j->qIndex+i) = ::pow(BM(j->to->index), power);
    }
  }
  return Wdiag;
#endif
}

/** @brief revert the topological orientation of a joint (edge),
   e.g., when choosing another body as root of a tree */
void mlr::KinematicWorld::revertJoint(mlr::Joint *j) {
  cout <<"reverting edge (" <<j->from->name <<' ' <<j->to->name <<")" <<endl;
  //revert
  j->from->outLinks.removeValue(j);
  j->to->inLinks.removeValue(j);
  Body *b=j->from; j->from=j->to; j->to=b;
  j->from->outLinks.append(j);
  j->to->inLinks.append(j);
  listReindex(j->from->outLinks);
  listReindex(j->from->inLinks);
  checkConsistency();

  mlr::Transformation f;
  f=j->A;
  j->A.setInverse(j->B);
  j->B.setInverse(f);
  f=j->Q;
  j->Q.setInverse(f);
}

/** @brief re-orient all joints (edges) such that n becomes
  the root of the configuration */
void mlr::KinematicWorld::reconfigureRoot(Body *root) {
  mlr::Array<Body*> list, list2;
  Body **m,**mstop;
  list.append(root);
  uintA level(bodies.N);
  level=0;
  int i=0;
  
  while(list.N>0) {
    i++;
    list2.clear();
    mstop=list.p+list.N;
    for(m=list.p; m!=mstop; m++) {
      level((*m)->index)=i;
      for_list(Joint,  e,  (*m)->inLinks) {
        if(!level(e->from->index)) { revertJoint(e); e_COUNT--; }
      }
      for(Joint *e: (*m)->outLinks) list2.append(e->to);
    }
    list=list2;
  }
  
  graphTopsort(bodies, joints);
}

void mlr::KinematicWorld::analyzeJointStateDimensions() {
  uint maxagent=0;
  for(Joint *j: joints) if(j->agent>maxagent) maxagent=j->agent;
  qdim.resize(maxagent+1);
  qdim.setZero();
  for(Joint *j: joints) {
    if(!j->mimic){
      j->qIndex = qdim(j->agent);
      qdim(j->agent) += j->qDim();
    }else{
      CHECK_EQ(j->agent, j->mimic->agent, "");
      j->qIndex = j->mimic->qIndex;
    }
  }
}

/** @brief returns the joint (actuator) dimensionality */
uint mlr::KinematicWorld::getJointStateDimension(int agent) const {
  if(agent==-1) agent=q_agent;
  CHECK(agent!=INT_MAX,"");
  if(!qdim.N){
    CHECK(!q.N && !qdim.N,"you've change q-dim (locked joints?) without clearing q,qdot");
    ((KinematicWorld*)this)->analyzeJointStateDimensions();
  }
  CHECK((uint)agent<qdim.N,"don't have agent # " <<agent <<" (analyzeJointStateDimensions returned [" <<qdim <<"])");
  return qdim(agent);
}

void mlr::KinematicWorld::getJointState(arr &_q, arr& _qdot, int agent) const {
  if(!qdim.N) ((KinematicWorld*)this)->analyzeJointStateDimensions();
  if(q.N!=getJointStateDimension(agent)) ((KinematicWorld*)this)->calc_q_from_Q(agent);

  _q=q;
  if(&_qdot){
    _qdot=qdot;
    if(!_qdot.N) _qdot.resizeAs(q).setZero();
  }
}

arr mlr::KinematicWorld::getJointState(int agent) const {
  if(!qdim.N) ((KinematicWorld*)this)->analyzeJointStateDimensions();
  if(q.N!=getJointStateDimension(agent)) ((KinematicWorld*)this)->calc_q_from_Q(agent);

  return q;
}

/** @brief returns the vector of joint limts */
arr mlr::KinematicWorld::getLimits() const {
  uint N=getJointStateDimension();
  arr limits(N,2);
  limits.setZero();
  for(Joint *j: joints) if(j->agent==q_agent){
    uint i=j->qIndex;
    uint d=j->qDim();
    for(uint k=0;k<d;k++){//in case joint has multiple dimensions
      if(j->limits.N){
        limits(i+k,0)=j->limits(0); //lo
        limits(i+k,1)=j->limits(1); //up
      }else{
        limits(i+k,0)=-1.; //lo
        limits(i+k,1)=+1.; //up
      }
    }
  }
//  cout <<"limits=" <<limits <<endl;
  return limits;
}

void mlr::KinematicWorld::zeroGaugeJoints() {
  Joint *e;
  mlr::Vector w;
  for(Body *n: bodies) if(n->type!=BT_static && n->inLinks.N) {
    e=n->inLinks(0);
    if(e) {
      e->A.appendTransformation(e->Q);
      e->Q.setZero();
    }
  }
}

arr mlr::KinematicWorld::calc_q_from_Q(mlr::Joint* j) {
  arr q;
  switch(j->type) {
    case JT_hingeX:
    case JT_hingeY:
    case JT_hingeZ: {
      q.resize(1);
      //angle
      mlr::Vector rotv;
      j->Q.rot.getRad(q(0), rotv);
      if(q(0)>MLR_PI) q(0)-=MLR_2PI;
      if(j->type==JT_hingeX && rotv*Vector_x<0.) q(0)=-q(0);
      if(j->type==JT_hingeY && rotv*Vector_y<0.) q(0)=-q(0);
      if(j->type==JT_hingeZ && rotv*Vector_z<0.) q(0)=-q(0);
    } break;

    case JT_universal: {
      q.resize(2);
      //angle
      if(fabs(j->Q.rot.w)>1e-15) {
        q(0) = 2.0 * atan(j->Q.rot.x/j->Q.rot.w);
        q(1) = 2.0 * atan(j->Q.rot.y/j->Q.rot.w);
      } else {
        q(0) = MLR_PI;
        q(1) = MLR_PI;
      }
    } break;

    case JT_quatBall: {
      q.resize(4);
      q(0)=j->Q.rot.w;
      q(1)=j->Q.rot.x;
      q(2)=j->Q.rot.y;
      q(3)=j->Q.rot.z;
    } break;

    case JT_transX: {
      q.resize(1);
      q(0)=j->Q.pos.x;
    } break;
    case JT_transY: {
      q.resize(1);
      q(0)=j->Q.pos.y;
    } break;
    case JT_transZ: {
      q.resize(1);
      q(0)=j->Q.pos.z;
    } break;
    case JT_transXY: {
      q.resize(1);
      q(0)=j->Q.pos.x;  
      q(1)=j->Q.pos.y;
    } break;
    case JT_transXYPhi: {
      q.resize(3);
      q(0)=j->Q.pos.x;
      q(1)=j->Q.pos.y;
      mlr::Vector rotv;
      j->Q.rot.getRad(q(2), rotv);
      if(q(2)>MLR_PI) q(2)-=MLR_2PI;
      if(rotv*Vector_z<0.) q(2)=-q(2);
    } break;
    case JT_phiTransXY: {
      q.resize(3);
      mlr::Vector rotv;
      j->Q.rot.getRad(q(0), rotv);
      if(q(0)>MLR_PI) q(0)-=MLR_2PI;
      if(rotv*Vector_z<0.) q(0)=-q(0);
      mlr::Vector relpos = j->Q.rot/j->Q.pos;
      q(1)=relpos.x;
      q(2)=relpos.y;
    } break;
    case JT_trans3: {
      q.resize(3);
      q(0)=j->Q.pos.x;
      q(1)=j->Q.pos.y;
      q(2)=j->Q.pos.z;
    } break;
    case JT_glue:
    case JT_rigid:
      break;
    case JT_free:
      q.resize(7);
      q(0)=j->Q.pos.x;
      q(1)=j->Q.pos.y;
      q(2)=j->Q.pos.z;
      q(3)=j->Q.rot.w;
      q(4)=j->Q.rot.x;
      q(5)=j->Q.rot.y;
      q(6)=j->Q.rot.z;
      break;
    default: NIY;
  }
  return q;
}

void mlr::KinematicWorld::calc_q_from_Q(int agent) {
  if(agent==-1) agent = q_agent;
//  mlr::Quaternion rot;
  
  uint N=getJointStateDimension(agent);
  q.resize(N);
  qdot.resize(N).setZero();

  uint n=0;
  for(Joint *j: joints) if(j->agent==(uint)agent){
    if(j->mimic) continue; //don't count dependent joints
    CHECK_EQ(j->qIndex,n,"joint indexing is inconsistent");
    arr joint_q = calc_q_from_Q(j);
    //TODO is there a better way?
    for(uint i=0; i<joint_q.N; ++i)
      q(n+i) = joint_q(i);
    n += joint_q.N;
  }
  CHECK_EQ(n,N,"");
}

void mlr::KinematicWorld::calc_Q_from_q(int agent){
  if(agent==-1) agent = q_agent;
  uint n=0;
  for(Joint *j: joints) if(j->agent==(uint)agent){
    if(j->mimic){
      j->Q=j->mimic->Q;
    }else{
      CHECK_EQ(j->qIndex,n,"joint indexing is inconsistent");
      switch(j->type) {
        case JT_hingeX: {
          j->Q.rot.setRadX(q.elem(n));
          n++;
        } break;

        case JT_hingeY: {
          j->Q.rot.setRadY(q.elem(n));
          n++;
        } break;

        case JT_hingeZ: {
          j->Q.rot.setRadZ(q.elem(n));
          n++;
        } break;

        case JT_universal:{
          mlr::Quaternion rot1, rot2;
          rot1.setRadX(q.elem(n));
          rot2.setRadY(q.elem(n+1));
          j->Q.rot = rot1*rot2;
          n+=2;
        } break;

        case JT_quatBall:{
          j->Q.rot.set(q.p+n);
          {
              double n=j->Q.rot.normalization();
              if(n<.5 || n>2.) LOG(-1) <<"quat normalization is extreme: " <<n <<endl;
          }
          j->Q.rot.normalize();
          j->Q.rot.isZero=false; //WHY? (gradient check fails without!)
          n+=4;
        } break;

        case JT_free:{
          j->Q.pos.set(q.p+n);
          j->Q.rot.set(q.p+n+3);
          {
              double n=j->Q.rot.normalization();
              if(n<.5 || n>2.) LOG(-1) <<"quat normalization is extreme: " <<n <<endl;
          }
          j->Q.rot.normalize();
          j->Q.rot.isZero=false;
          n+=7;
        } break;

        case JT_transX: {
          j->Q.pos = q.elem(n)*Vector_x;
          n++;
        } break;

        case JT_transY: {
          j->Q.pos = q.elem(n)*Vector_y;
          n++;
        } break;

        case JT_transZ: {
          j->Q.pos = q.elem(n)*Vector_z;
          n++;
        } break;

        case JT_transXY: {
          j->Q.pos.set(q.elem(n), q.elem(n+1), 0.);
          n+=2;
        } break;

        case JT_trans3: {
          j->Q.pos.set(q.elem(n), q.elem(n+1), q.elem(n+2));
          n+=3;
        } break;

        case JT_transXYPhi: {
          j->Q.pos.set(q.elem(n), q.elem(n+1), 0.);
          j->Q.rot.setRadZ(q.elem(n+2));
          n+=3;
        } break;

        case JT_phiTransXY: {
          j->Q.rot.setRadZ(q.elem(n));
          j->Q.pos = j->Q.rot*Vector(q.elem(n+1), q.elem(n+2), 0.);
          n+=3;
        } break;

        case JT_glue:
        case JT_rigid:
          j->Q.setZero();
          break;
        default: NIY;
      }
    }
  }

  CHECK_EQ(n,q.N,"");
}


/** @brief sets the joint state vectors separated in positions and
  velocities */
void mlr::KinematicWorld::setJointState(const arr& _q, const arr& _qdot, int agent) {
  setJointStateCount++; //global counter

  uint N=getJointStateDimension(agent);
  CHECK(_q.N==N && (!(&_qdot) || _qdot.N==N), "wrong joint state dimensionalities");
  q=_q;
  if(&_qdot) qdot=_qdot; else qdot.clear();

  calc_Q_from_q(agent);

  calc_fwdPropagateFrames();
}

void mlr::KinematicWorld::setAgent(uint agent){
  if(agent==q_agent) return; //nothing to do
  q_agent = agent;
  calc_q_from_Q();
}



//===========================================================================
//
// core: kinematics and dynamics
//

/** @brief return the jacobian \f$J = \frac{\partial\phi_i(q)}{\partial q}\f$ of the position
  of the i-th body (3 x n tensor)*/
void mlr::KinematicWorld::kinematicsPos(arr& y, arr& J, Body *b, const mlr::Vector& rel) const {
  if(!b){
    MLR_MSG("WARNING: calling kinematics for NULL body");
    if(&y) y.resize(3).setZero();
    if(&J) J.resize(3, getJointStateDimension()).setZero();
    return;
  }

  //get position
  mlr::Vector pos_world = b->X.pos;
  if(&rel) pos_world += b->X.rot*rel;
  if(&y) y = conv_vec2arr(pos_world); //return the output
  if(!&J) return; //do not return the Jacobian

  //get Jacobian
  uint N=getJointStateDimension();
  J.resize(3, N).setZero();
  if(b->inLinks.N) { //body a has no inLinks -> zero jacobian
    Joint *j=b->inLinks(0);
    while(j) { //loop backward down the kinematic tree
      uint j_idx=j->qIndex;
      if(j->agent==q_agent && j_idx>=N) CHECK(j->type==JT_glue || j->type==JT_rigid, "");
      if(j->agent==q_agent && j_idx<N){
        if(j->type==JT_hingeX || j->type==JT_hingeY || j->type==JT_hingeZ) {
          mlr::Vector tmp = j->axis ^ (pos_world-j->X.pos);
          J(0, j_idx) += tmp.x;
          J(1, j_idx) += tmp.y;
          J(2, j_idx) += tmp.z;
        }
        else if(j->type==JT_transX || j->type==JT_transY || j->type==JT_transZ) {
          J(0, j_idx) += j->axis.x;
          J(1, j_idx) += j->axis.y;
          J(2, j_idx) += j->axis.z;
        }
        else if(j->type==JT_transXY) {
          if(j->mimic) NIY;
          arr R = j->X.rot.getArr();
          J.setMatrixBlock(R.sub(0,-1,0,1), 0, j_idx);
        }
        else if(j->type==JT_transXYPhi) {
          if(j->mimic) NIY;
          arr R = j->X.rot.getArr();
          J.setMatrixBlock(R.sub(0,-1,0,1), 0, j_idx);
          mlr::Vector tmp = j->axis ^ (pos_world-(j->X.pos + j->X.rot*j->Q.pos));
          J(0, j_idx+2) += tmp.x;
          J(1, j_idx+2) += tmp.y;
          J(2, j_idx+2) += tmp.z;
        }
        else if(j->type==JT_phiTransXY) {
          if(j->mimic) NIY;
          mlr::Vector tmp = j->axis ^ (pos_world-j->X.pos);
          J(0, j_idx) += tmp.x;
          J(1, j_idx) += tmp.y;
          J(2, j_idx) += tmp.z;
          arr R = (j->X.rot*j->Q.rot).getArr();
          J.setMatrixBlock(R.sub(0,-1,0,1), 0, j_idx+1);
        }
        if(j->type==JT_trans3 || j->type==JT_free) {
          if(j->mimic) NIY;
          arr R = j->X.rot.getArr();
          J.setMatrixBlock(R, 0, j_idx);
        }
        if(j->type==JT_quatBall || j->type==JT_free) {
          uint offset = (j->type==JT_free)?3:0;
          arr Jrot = j->X.rot.getArr() * j->Q.rot.getJacobian(); //transform w-vectors into world coordinate
          Jrot = crossProduct(Jrot, conv_vec2arr(pos_world-(j->X.pos+j->X.rot*j->Q.pos)) ); //cross-product of all 4 w-vectors with lever
          Jrot /= sqrt(sumOfSqr( q({j->qIndex+offset, j->qIndex+offset+3}) )); //account for the potential non-normalization of q
          for(uint i=0;i<4;i++) for(uint k=0;k<3;k++) J(k,j_idx+offset+i) += Jrot(k,i);
        }
      }
      if(!j->from->inLinks.N) break;
      j=j->from->inLinks(0);
    }
  }
}

/** @brief return the jacobian \f$J = \frac{\partial\phi_i(q)}{\partial q}\f$ of the position
  of the i-th body W.R.T. the 6 axes of an arbitrary shape-frame, NOT the robot's joints (3 x 6 tensor)
  WARNING: this does not check if s is actually in the kinematic chain from root to b.
*/
void mlr::KinematicWorld::kinematicsPos_wrtFrame(arr& y, arr& J, Body *b, const mlr::Vector& rel, Shape *s) const {
  if(!b && &J){ J.resize(3, getJointStateDimension()).setZero();  return; }

  //get position
  mlr::Vector pos_world = b->X.pos;
  if(&rel) pos_world += b->X.rot*rel;
  if(&y) y = conv_vec2arr(pos_world); //return the output
  if(!&J) return; //do not return the Jacobian

  //get Jacobian
  J.resize(3, 6).setZero();
  mlr::Vector diff = pos_world - s->X.pos;
  mlr::Array<mlr::Vector> axes = {s->X.rot.getX(), s->X.rot.getY(), s->X.rot.getZ()};

  //3 translational axes
  for(uint i=0;i<3;i++){
    J(0, i) += axes(i).x;
    J(1, i) += axes(i).y;
    J(2, i) += axes(i).z;
  }

  //3 rotational axes
  for(uint i=0;i<3;i++){
    mlr::Vector tmp = axes(i) ^ diff;
    J(0, 3+i) += tmp.x;
    J(1, 3+i) += tmp.y;
    J(2, 3+i) += tmp.z;
  }
}

/** @brief return the Hessian \f$H = \frac{\partial^2\phi_i(q)}{\partial q\partial q}\f$ of the position
  of the i-th body (3 x n x n tensor) */
void mlr::KinematicWorld::hessianPos(arr& H, Body *b, mlr::Vector *rel) const {
  HALT("this is buggy: a sign error: see examples/Kin/ors testKinematics");
  Joint *j1, *j2;
  uint j1_idx, j2_idx;
  mlr::Vector tmp, pos_a;
  
  uint N=getJointStateDimension();
  
  //initialize Jacobian
  H.resize(3, N, N);
  H.setZero();
  
  //get reference frame
  pos_a = b->X.pos;
  if(rel) pos_a += b->X.rot*(*rel);
  
  if(b->inLinks.N) {
    j1=b->inLinks(0);
    while(j1) {
      CHECK_EQ(j1->agent,q_agent,"NIY");
      j1_idx=j1->qIndex;

      j2=j1;
      while(j2) {
        CHECK_EQ(j2->agent,q_agent,"NIY");
        j2_idx=j2->qIndex;

        if(j1->type>=JT_hingeX && j1->type<=JT_hingeZ && j2->type>=JT_hingeX && j2->type<=JT_hingeZ) { //both are hinges
          tmp = j2->axis ^ (j1->axis ^ (pos_a-j1->X.pos));
          H(0, j1_idx, j2_idx) = H(0, j2_idx, j1_idx) = tmp.x;
          H(1, j1_idx, j2_idx) = H(1, j2_idx, j1_idx) = tmp.y;
          H(2, j1_idx, j2_idx) = H(2, j2_idx, j1_idx) = tmp.z;
        }
        else if(j1->type>=JT_transX && j1->type<=JT_transZ && j2->type>=JT_hingeX && j2->type<=JT_hingeZ) { //i=trans, j=hinge
          tmp = j1->axis ^ j2->axis;
          H(0, j1_idx, j2_idx) = H(0, j2_idx, j1_idx) = tmp.x;
          H(1, j1_idx, j2_idx) = H(1, j2_idx, j1_idx) = tmp.y;
          H(2, j1_idx, j2_idx) = H(2, j2_idx, j1_idx) = tmp.z;
        }
        else if(j1->type==JT_transXY && j2->type>=JT_hingeX && j2->type<=JT_hingeZ) { //i=trans3, j=hinge
          NIY;
        }
        else if(j1->type==JT_transXYPhi && j1->type==JT_phiTransXY && j2->type>=JT_hingeX && j2->type<=JT_hingeZ) { //i=trans3, j=hinge
          NIY;
        }
        else if(j1->type==JT_trans3 && j2->type>=JT_hingeX && j2->type<=JT_hingeZ) { //i=trans3, j=hinge
          Matrix R,A;
          j1->X.rot.getMatrix(R.p());
          A.setSkew(j2->axis);
          R = R*A;
          H(0, j1_idx  , j2_idx) = H(0, j2_idx  , j1_idx) = R.m00;
          H(1, j1_idx  , j2_idx) = H(1, j2_idx  , j1_idx) = R.m10;
          H(2, j1_idx  , j2_idx) = H(2, j2_idx  , j1_idx) = R.m20;
          H(0, j1_idx+1, j2_idx) = H(0, j2_idx, j1_idx+1) = R.m01;
          H(1, j1_idx+1, j2_idx) = H(1, j2_idx, j1_idx+1) = R.m11;
          H(2, j1_idx+1, j2_idx) = H(2, j2_idx, j1_idx+1) = R.m21;
          H(0, j1_idx+2, j2_idx) = H(0, j2_idx, j1_idx+2) = R.m02;
          H(1, j1_idx+2, j2_idx) = H(1, j2_idx, j1_idx+2) = R.m12;
          H(2, j1_idx+2, j2_idx) = H(2, j2_idx, j1_idx+2) = R.m22;
        }
        else if(j1->type>=JT_hingeX && j1->type<=JT_hingeZ && j2->type>=JT_transX && j2->type<=JT_trans3) { //i=hinge, j=trans
          //nothing! Hessian is zero (ej is closer to root than ei)
        }
        else NIY;

        if(!j2->from->inLinks.N) break;
        j2=j2->from->inLinks(0);
      }
      if(!j1->from->inLinks.N) break;
      j1=j1->from->inLinks(0);
    }
  }
}

/* takes the joint state x and returns the jacobian dz of
   the position of the ith body (w.r.t. all joints) -> 2D array */
/// Jacobian of the i-th body's z-orientation vector
void mlr::KinematicWorld::kinematicsVec(arr& y, arr& J, Body *b, const mlr::Vector& vec) const {
  //get the vectoreference frame
  mlr::Vector vec_referene;
  if(&vec) vec_referene = b->X.rot*vec;
  else     vec_referene = b->X.rot.getZ();
  if(&y) y = conv_vec2arr(vec_referene); //return the vec
  if(&J){
    arr A;
    axesMatrix(A, b);
    J = crossProduct(A, conv_vec2arr(vec_referene));
  }
}

/* takes the joint state x and returns the jacobian dz of
   the position of the ith body (w.r.t. all joints) -> 2D array */
/// Jacobian of the i-th body's z-orientation vector
void mlr::KinematicWorld::kinematicsQuat(arr& y, arr& J, Body *b) const { //TODO: allow for relative quat
  mlr::Quaternion rot_b = b->X.rot;
  if(&y) y = conv_quat2arr(rot_b); //return the vec
  if(&J){
    arr A;
    axesMatrix(A, b);
    J.resize(4, A.d1);
    for(uint i=0;i<J.d1;i++){
      mlr::Quaternion tmp(0., 0.5*A(0,i), 0.5*A(1,i), 0.5*A(2,i) ); //this is unnormalized!!
      tmp = tmp * rot_b;
      J(0, i) = tmp.w;
      J(1, i) = tmp.x;
      J(2, i) = tmp.y;
      J(3, i) = tmp.z;
    }
  }
}

//* This Jacobian directly gives the implied rotation vector: multiplied with \dot q it gives the angular velocity of body b */
void mlr::KinematicWorld::axesMatrix(arr& J, Body *b) const {
  uint N = getJointStateDimension();
  J.resize(3, N).setZero();
  if(b->inLinks.N) {
    Joint *j=b->inLinks(0);
    while(j) { //loop backward down the kinematic tree
      uint j_idx=j->qIndex;
      if(j->agent==q_agent && j_idx>=N) CHECK(j->type==JT_glue || j->type==JT_rigid, "");
      if(j->agent==q_agent && j_idx<N){
        if((j->type>=JT_hingeX && j->type<=JT_hingeZ) || j->type==JT_transXYPhi || j->type==JT_phiTransXY) {
          if(j->type==JT_transXYPhi) j_idx += 2; //refer to the phi only
          J(0, j_idx) += j->axis.x;
          J(1, j_idx) += j->axis.y;
          J(2, j_idx) += j->axis.z;
        }
        if(j->type==JT_quatBall || j->type==JT_free) {
          uint offset = (j->type==JT_free)?3:0;
          arr Jrot = j->X.rot.getArr() * j->Q.rot.getJacobian(); //transform w-vectors into world coordinate
          Jrot /= sqrt(sumOfSqr(q({j->qIndex+offset,j->qIndex+offset+3}))); //account for the potential non-normalization of q
          for(uint i=0;i<4;i++) for(uint k=0;k<3;k++) J(k,j_idx+offset+i) += Jrot(k,i);
        }
        //all other joints: J=0 !!
      }
      if(!j->from->inLinks.N) break;
      j=j->from->inLinks(0);
    }
  }
}

/// The position vec1, attached to b1, relative to the frame of b2 (plus vec2)
void mlr::KinematicWorld::kinematicsRelPos(arr& y, arr& J, Body *b1, const mlr::Vector& vec1, Body *b2, const mlr::Vector& vec2) const {
  arr y1,y2,J1,J2;
  kinematicsPos(y1, J1, b1, vec1);
  kinematicsPos(y2, J2, b2, vec2);
  arr Rinv = ~(b2->X.rot.getArr());
  y = Rinv * (y1 - y2);
  if(&J){
    arr A;
    axesMatrix(A, b2);
    J = Rinv * (J1 - J2 - crossProduct(A, y1 - y2));
  }
}

/// The vector vec1, attached to b1, relative to the frame of b2
void mlr::KinematicWorld::kinematicsRelVec(arr& y, arr& J, Body *b1, const mlr::Vector& vec1, Body *b2) const {
  arr y1,J1;
  kinematicsVec(y1, J1, b1, vec1);
//  kinematicsVec(y2, J2, b2, vec2);
  arr Rinv = ~(b2->X.rot.getArr());
  y = Rinv * y1;
  if(&J){
    arr A;
    axesMatrix(A, b2);
    J = Rinv * (J1 - crossProduct(A, y1));
  }
}

/// The position vec1, attached to b1, relative to the frame of b2 (plus vec2)
void mlr::KinematicWorld::kinematicsRelRot(arr& y, arr& J, Body *b1, Body *b2) const {
  mlr::Quaternion rot_b = b1->X.rot;
  if(&y) y = conv_vec2arr(rot_b.getVec());
  if(&J){
    double phi=acos(rot_b.w);
    double s=2.*phi/sin(phi);
    double ss=-2./(1.-mlr::sqr(rot_b.w)) * (1.-phi/tan(phi));
    arr A;
    axesMatrix(A, b1);
    J = 0.5 * (rot_b.w*A*s + crossProduct(A, y));
    J -= 0.5 * ss/s/s*(y*~y*A);
  }
}

/** @brief return the configuration's inertia tensor $M$ (n x n tensor)*/
void mlr::KinematicWorld::inertia(arr& M) {
  uint j1_idx, j2_idx;
  mlr::Transformation Xa, Xi, Xj;
  Joint *j1, *j2;
  mlr::Vector vi, vj, ti, tj;
  double tmp;
  
  uint N=getJointStateDimension();
  
  //initialize Jacobian
  M.resize(N, N);
  M.setZero();
  
  for(Body *a: bodies) {
    //get reference frame
    Xa = a->X;
    
    j1=a->inLinks(0);
    while(j1) {
      j1_idx=j1->qIndex;
      
      Xi = j1->from->X;
      Xi.appendTransformation(j1->A);
      ti = Xi.rot.getX();
      
      vi = ti ^(Xa.pos-Xi.pos);
      
      j2=j1;
      while(j2) {
        j2_idx=j2->qIndex;
        
        Xj = j2->from->X;
        Xj.appendTransformation(j2->A);
        tj = Xj.rot.getX();
        
        vj = tj ^(Xa.pos-Xj.pos);
        
        tmp = a->mass * (vi*vj);
        //tmp += scalarProduct(a->a.inertia, ti, tj);
        
        M(j1_idx, j2_idx) += tmp;
        
        if(!j2->from->inLinks.N) break;
        j2=j2->from->inLinks(0);
      }
      if(!j1->from->inLinks.N) break;
      j1=j1->from->inLinks(0);
    }
  }
  //symmetric: fill in other half
  for(j1_idx=0; j1_idx<N; j1_idx++) for(j2_idx=0; j2_idx<j1_idx; j2_idx++) M(j2_idx, j1_idx) = M(j1_idx, j2_idx);
}

void mlr::KinematicWorld::equationOfMotion(arr& M, arr& F, bool gravity) {
  mlr::LinkTree tree; //TODO: HACK!! Danny: Why was there a static? This fails if there are more than 2 worlds
  if(!tree.N) GraphToTree(tree, *this);
  else updateGraphToTree(tree, *this);
  if(gravity){
    clearForces();
    gravityToForces();
  }
  if(!qdot.N) qdot.resize(q.N).setZero();
  mlr::equationOfMotion(M, F, tree, qdot);
}

/** @brief return the joint accelerations \f$\ddot q\f$ given the
  joint torques \f$\tau\f$ (computed via Featherstone's Articulated Body Algorithm in O(n)) */
void mlr::KinematicWorld::fwdDynamics(arr& qdd, const arr& qd, const arr& tau) {
  static mlr::LinkTree tree;
  if(!tree.N) GraphToTree(tree, *this);
  else updateGraphToTree(tree, *this);
  //cout <<tree <<endl;
  //mlr::fwdDynamics_aba_1D(qdd, tree, qd, tau);
  //mlr::fwdDynamics_aba_nD(qdd, tree, qd, tau);
  mlr::fwdDynamics_MF(qdd, tree, qd, tau);
}

/** @brief return the necessary joint torques \f$\tau\f$ to achieve joint accelerations
  \f$\ddot q\f$ (computed via the Recursive Newton-Euler Algorithm in O(n)) */
void mlr::KinematicWorld::inverseDynamics(arr& tau, const arr& qd, const arr& qdd) {
  static mlr::LinkTree tree;
  if(!tree.N) GraphToTree(tree, *this);
  else updateGraphToTree(tree, *this);
  mlr::invDynamics(tau, tree, qd, qdd);
}

/*void mlr::KinematicWorld::impulsePropagation(arr& qd1, const arr& qd0){
  static mlr::Array<Featherstone::Link> tree;
  if(!tree.N) GraphToTree(tree, *this);
  else updateGraphToTree(tree, *this);
  mimickImpulsePropagation(tree);
  Featherstone::RF_abd(qdd, tree, qd, tau);
}*/

/// [prelim] some heuristic measure for the joint errors
double mlr::KinematicWorld::getJointErrors() const {
  double err=0.0;
  for(Joint * e: joints) err+=e->Q.pos.lengthSqr();
  return ::sqrt(err);
}

/** @brief checks if all names of the bodies are disjoint */
bool mlr::KinematicWorld::checkUniqueNames() const {
  for(Body *  n:  bodies) for(Body *b: bodies) {
    if(n==b) break;
    if(n->name==b->name) return false;
  }
  return true;
}

/** @brief checks if all names of the bodies are disjoint */
void mlr::KinematicWorld::setShapeNames() {
  for(Body *b: bodies){
    uint i=0;
    for(Shape *s:b->shapes){
      if(!s->name.N){ s->name = b->name; s->name <<'_' <<i; }
      i++;
    }
  }
}

/// find body with specific name
mlr::Body* mlr::KinematicWorld::getBodyByName(const char* name, bool warnIfNotExist) const {
  for(Body *b: bodies) if(b->name==name) return b;
  if(strcmp("glCamera", name)!=0)
  if(warnIfNotExist) MLR_MSG("cannot find Body named '" <<name <<"' in Graph");
  return 0;
}

/// find shape with specific name
mlr::Shape* mlr::KinematicWorld::getShapeByName(const char* name, bool warnIfNotExist) const {
  for(Shape *s: shapes) if(s->name==name) return s;
  if(warnIfNotExist) MLR_MSG("cannot find Shape named '" <<name <<"' in Graph");
  return NULL;
}

/// find shape with specific name
mlr::Joint* mlr::KinematicWorld::getJointByName(const char* name, bool warnIfNotExist) const {
  for(Joint *j: joints) if(j->name==name) return j;
  if(warnIfNotExist) MLR_MSG("cannot find Joint named '" <<name <<"' in Graph");
  return NULL;
}

/// find joint connecting two bodies
mlr::Joint* mlr::KinematicWorld::getJointByBodies(const Body* from, const Body* to) const {
  for(Joint *j: to->inLinks) if(j->from==from) return j;
  return NULL;
}

/// find joint connecting two bodies with specific names
mlr::Joint* mlr::KinematicWorld::getJointByBodyNames(const char* from, const char* to) const {
  Body *f = getBodyByName(from);
  Body *t = getBodyByName(to);
  if(!f || !t) return NULL;
  return graphGetEdge<Body, Joint>(f, t);
}

/// find joint connecting two bodies with specific names
mlr::Joint* mlr::KinematicWorld::getJointByBodyIndices(uint ifrom, uint ito) const {
  if(ifrom>=bodies.N || ito>=bodies.N) return NULL;
  Body *f = bodies(ifrom);
  Body *t = bodies(ito);
  return getJointByBodies(f, t);
}

ShapeL mlr::KinematicWorld::getShapesByAgent(const uint agent) const {
  ShapeL agent_shapes;
  for(mlr::Joint *j : joints) {
    if(j->agent==agent) {
      ShapeL tmp;
      tmp.append(j->from->shapes);
      tmp.append(j->to->shapes);
      for(mlr::Shape* s : tmp) {
        if (!agent_shapes.contains(s)) agent_shapes.append(s);
      }
    }
  }
  return agent_shapes;
}

uintA mlr::KinematicWorld::getShapeIdxByAgent(const uint agent) const {
  uintA agent_shape_idx;
  ShapeL agent_shapes = getShapesByAgent(agent);
  for(mlr::Shape* s : agent_shapes)
    agent_shape_idx.append(s->index);
  return agent_shape_idx;
}

/** @brief creates uniques names by prefixing the node-index-number to each name */
void mlr::KinematicWorld::prefixNames() {
  for(Body * n: bodies) n->name=STRING(n->index<< n->name);
}

/// return a OpenGL extension
OpenGL& mlr::KinematicWorld::gl(const char* window_title){
  if(!s->gl){
    s->gl = new OpenGL(window_title);
    s->gl->add(glStandardScene, 0);
    s->gl->addDrawer(this);
    s->gl->camera.setDefault();
  }
  return *s->gl;
}

/// return a Swift extension
SwiftInterface& mlr::KinematicWorld::swift(){
  if(!s->swift) s->swift = new SwiftInterface(*this);
  return *s->swift;
}

void mlr::KinematicWorld::swiftDelete() {
  delete s->swift;
  s->swift = nullptr;
}

/// return a PhysX extension
PhysXInterface& mlr::KinematicWorld::physx(){
  if(!s->physx){
    s->physx = new PhysXInterface(*this);
    s->physx->setArticulatedBodiesKinematic();
  }
  return *s->physx;
}

/// return a ODE extension
OdeInterface& mlr::KinematicWorld::ode(){
  if(!s->ode) s->ode = new OdeInterface(*this);
  return *s->ode;
}

void mlr::KinematicWorld::watch(bool pause, const char* txt){
  if(pause) gl().watch(txt);
  else gl().update(txt);
}

void mlr::KinematicWorld::glAnimate(){
  animateConfiguration(*this, NULL);
}

void mlr::KinematicWorld::glGetMasks(int w, int h, bool rgbIndices){
  gl().clear();
  gl().addDrawer(this);
  if(rgbIndices){
    gl().setClearColors(0,0,0,0);
    orsDrawIndexColors = true;
    orsDrawMarkers = orsDrawJoints = orsDrawProxies = false;
  }
  gl().renderInBack(true, true, w, h);
//  indexRgb = gl().captureImage;
//  depth = gl().captureDepth;

  gl().clear();
  gl().add(glStandardScene, 0);
  gl().addDrawer(this);
  if(rgbIndices){
    gl().setClearColors(1,1,1,0);
    orsDrawIndexColors = false;
    orsDrawMarkers = orsDrawJoints = orsDrawProxies = true;
  }
}

void mlr::KinematicWorld::stepSwift(){
  swift().step(*this, false);
}

void mlr::KinematicWorld::stepPhysx(double tau){
  physx().step(tau);
}

void mlr::KinematicWorld::stepOde(double tau){
#ifdef MLR_ODE
  ode().setMotorVel(qdot, 100.);
  ode().step(tau);
  ode().importStateFromOde();
#endif
}

void mlr::KinematicWorld::stepDynamics(const arr& Bu_control, double tau, double dynamicNoise, bool gravity){

  struct DiffEqn:VectorFunction{
    mlr::KinematicWorld &S;
    const arr& Bu;
    bool gravity;
    DiffEqn(mlr::KinematicWorld& _S, const arr& _Bu, bool _gravity):S(_S), Bu(_Bu), gravity(_gravity){
      VectorFunction::operator=( [this](arr& y, arr& J, const arr& x) -> void {
        this->fv(y, J, x);
      } );
    }
    void fv(arr& y, arr& J, const arr& x){
      S.setJointState(x[0], x[1]);
      arr M,Minv,F;
      S.equationOfMotion(M, F, gravity);
      inverse_SymPosDef(Minv, M);
      //Minv = inverse(M); //TODO why does symPosDef fail?
      y = Minv * (Bu - F);
    }
  } eqn(*this, Bu_control, gravity);

#if 0
  arr M,Minv,F;
  getDynamics(M, F);
  inverse_SymPosDef(Minv,M);

  //noisy Euler integration (Runge-Kutte4 would be much more precise...)
  qddot = Minv * (u_control - F);
  if(dynamicNoise) rndGauss(qddot, dynamicNoise, true);
  q    += tau * qdot;
  qdot += tau * qddot;
  arr x1=cat(s->q, s->qdot).reshape(2,s->q.N);
#else
  arr x1;
  rk4_2ndOrder(x1, cat(q, qdot).reshape(2,q.N), eqn, tau);
  if(dynamicNoise) rndGauss(x1[1](), ::sqrt(tau)*dynamicNoise, true);
#endif

  setJointState(x1[0], x1[1]);
}

/** @brief prototype for \c operator<< */
void mlr::KinematicWorld::write(std::ostream& os) const {
  for(Body *b: bodies) {
    os <<"body " <<b->name <<" { ";
    b->write(os);  os <<" }\n";
  }
  os <<std::endl;
  for(Shape *s: shapes) {
    os <<"shape ";
    if(s->name.N) os <<s->name <<' ';
    os <<"(" <<(s->body?(char*)s->body->name:"") <<"){ ";
    s->write(os);  os <<" }\n";
  }
  os <<std::endl;
  for(Joint *j: joints) {
    os <<"joint ";
    if (j->name.N) os <<j->name <<' ';
    os <<"(" <<j->from->name <<' ' <<j->to->name <<"){ ";
    j->write(os);  os <<" }\n";
  }
}

#define DEBUG(x) //x

/** @brief prototype for \c operator>> */
void mlr::KinematicWorld::read(std::istream& is) {
  Graph G(is);
  G.checkConsistency();
//  cout <<"***KVG:\n" <<G <<endl;
  init(G);
}

void mlr::KinematicWorld::init(const Graph& G) {
  clear();

  NodeL bs = G.getNodes("body");
  for(Node *n:  bs) {
    CHECK_EQ(n->keys(0),"body","");
    CHECK(n->isGraph(), "bodies must have value Graph");
    
    Body *b=new Body(*this);
    if(n->keys.N>1) b->name=n->keys.last();
    b->ats.copy(n->graph(), false, true);
    if(n->keys.N>2) b->ats.newNode<bool>({n->keys.last(-1)});
    b->parseAts();
  }

  NodeL ss = G.getNodes("shape");
  for(Node *n: ss) {
    CHECK_EQ(n->keys(0),"shape","");
    CHECK(n->parents.N<=1,"shapes must have no or one parent");
    CHECK(n->isGraph(),"shape must have value Graph");
    
    Shape *s;
    if(n->parents.N==1){
      Body *b = listFindByName(bodies, n->parents(0)->keys.last());
      CHECK(b,"");
      s=new Shape(*this, *b);
    }else{
      s=new Shape(*this, NoBody);
    }
    if(n->keys.N>1) s->name=n->keys.last();
    s->ats.copy(n->graph(), false, true);
    if(n->keys.N>2) s->ats.newNode<bool>({n->keys.last(-1)});
    s->parseAts();
  }
  
  uint nCoupledJoints=0;
  NodeL js = G.getNodes("joint");
  for(Node *n: js) {
    CHECK_EQ(n->keys(0),"joint","joints must be declared as joint: specs=" <<*n <<' ' <<n->index);
    CHECK_EQ(n->parents.N,2,"joints must have two parents: specs=" <<*n <<' ' <<n->index);
    CHECK(n->isGraph(),"joints must have value Graph: specs=" <<*n <<' ' <<n->index);
    
    Body *from=listFindByName(bodies, n->parents(0)->keys.last());
    Body *to=listFindByName(bodies, n->parents(1)->keys.last());
    CHECK(from,"JOINT: from '" <<n->parents(0)->keys.last() <<"' does not exist ["<<*n <<"]");
    CHECK(to,"JOINT: to '" <<n->parents(1)->keys.last() <<"' does not exist ["<<*n <<"]");
    Joint *j=new Joint(*this, from, to);
    if(n->keys.N>1) j->name=n->keys.last();
    j->ats.copy(n->graph(), false, true);
    if(n->keys.N>2) j->ats.newNode<bool>({n->keys.last(-1)});
    j->parseAts();

    //if the joint is coupled to another:
    if(j->mimic) nCoupledJoints++;
  }

  if(nCoupledJoints){
    for(Joint *j: joints) if(j->mimic){
      mlr::String jointName;
      bool good = j->ats.get(jointName, "mimic");
      CHECK(good, "something is wrong");
      if(!jointName.N){ j->mimic=NULL; continue; }
      j->mimic = listFindByName(joints, jointName);
      if(!j->mimic) HALT("The joint '" <<*j <<"' is declared coupled to '" <<jointName <<"' -- but that doesn't exist!");
      j->type = j->mimic->type;
    }
  }

  //-- clean up the graph
  analyzeJointStateDimensions();
//  topSort();
  jointSort();
  checkConsistency();
  //makeLinkTree();
  calc_missingAB_from_BodyAndJointFrames();
  analyzeJointStateDimensions();
  calc_q_from_Q();
  calc_fwdPropagateFrames();
}

void mlr::KinematicWorld::writePlyFile(const char* filename) const {
  ofstream os;
  mlr::open(os, filename);
  uint nT=0,nV=0;
  uint j;
  mlr::Mesh *m;
  for(Shape *s: shapes) { nV += s->mesh.V.d0; nT += s->mesh.T.d0; }
  
  os <<"\
ply\n\
format ascii 1.0\n\
element vertex " <<nV <<"\n\
property float x\n\
property float y\n\
property float z\n\
property uchar red\n\
property uchar green\n\
property uchar blue\n\
element face " <<nT <<"\n\
property list uchar int vertex_index\n\
end_header\n";

  uint k=0;
  mlr::Transformation t;
  mlr::Vector v;
  for(Shape * s: shapes) {
    m = &s->mesh;
    arr col = m->C;
    CHECK(col.N==3,"");
    t = s->X;
    if(m->C.d0!=m->V.d0) {
      m->C.resizeAs(m->V);
      for(j=0; j<m->C.d0; j++) m->C[j]=col;
    }
    for(j=0; j<m->V.d0; j++) {
      v.set(m->V(j, 0), m->V(j, 1), m->V(j, 2));
      v = t*v;
      os <<' ' <<v.x <<' ' <<v.y <<' ' <<v.z
         <<' ' <<int(255.f*m->C(j, 0)) <<' ' <<int(255.f*m->C(j, 1)) <<' ' <<int(255.f*m->C(j, 2)) <<endl;
    }
    k+=j;
  }
  uint offset=0;
  for(Shape *s: shapes) {
    m=&s->mesh;
    for(j=0; j<m->T.d0; j++) {
      os <<"3 " <<offset+m->T(j, 0) <<' ' <<offset+m->T(j, 1) <<' ' <<offset+m->T(j, 2) <<endl;
    }
    offset+=m->V.d0;
  }
}

/// dump the list of current proximities on the screen
void mlr::KinematicWorld::reportProxies(std::ostream& os, double belowMargin, bool brief) const{
  os <<"Proximity report: #" <<proxies.N <<endl;
  for_list(Proxy, p, proxies) {
    if(belowMargin>0. && p->d>belowMargin) continue;
    mlr::Shape *a = shapes(p->a);
    mlr::Shape *b = shapes(p->b);
    os  <<p_COUNT <<" ("
        <<a->name <<':' <<a->body->name <<")-("
        <<b->name <<':' <<b->body->name
        <<") d=" <<p->d;
    if(!brief)
     os <<" |A-B|=" <<(p->posB-p->posA).length()
        <<" cenD=" <<p->cenD
//        <<" d^2=" <<(p->posB-p->posA).lengthSqr()
        <<" v=" <<(p->posB-p->posA)
        <<" normal=" <<p->normal
        <<" posA=" <<p->posA
        <<" posB=" <<p->posB;
    os <<endl;
  }
}

bool ProxySortComp(const mlr::Proxy *a, const mlr::Proxy *b) {
  return (a->a < b->a) || (a->a==b->a && a->b<b->b) || (a->a==b->a && a->b==b->b && a->d < b->d);
}

void mlr::KinematicWorld::glueBodies(Body *f, Body *t) {
  Joint *j = new Joint(*this, f, t);
  j->A.setDifference(f->X, t->X);
  j->type=JT_rigid;
  j->Q.setZero();
  j->B.setZero();
  isLinkTree=false;
}


/// clear all forces currently stored at bodies
void mlr::KinematicWorld::clearForces() {
  for(Body *  n:  bodies) {
    n->force.setZero();
    n->torque.setZero();
  }
}

/// apply a force on body n 
void mlr::KinematicWorld::addForce(mlr::Vector force, mlr::Body *n) {
  n->force += force;
  if (!s->physx) {
    NIY;
  }
  else {
    s->physx->addForce(force, n);
  }
  //n->torque += (pos - n->X.p) ^ force;
}

/// apply a force on body n at position pos (in world coordinates)
void mlr::KinematicWorld::addForce(mlr::Vector force, mlr::Body *n, mlr::Vector pos) {
  n->force += force;
  if (!s->physx) {
    NIY;
  }
  else {
    s->physx->addForce(force, n, pos);
  }
  //n->torque += (pos - n->X.p) ^ force;
}

void mlr::KinematicWorld::frictionToForces(double coeff) {
  HALT("never do this: add it directly in the equations...");
  mlr::Vector a;
  mlr::Transformation X;
  for(Joint *j:joints) {
    X = j->from->X;
    X.appendTransformation(j->A);
    a = X.rot.getX();//rotation axis

NIY;
//    v=j->Q.angvel.length();
//    if(j->Q.angvel*Vector_x<0.) v=-v;
    
//    j->from->torque -= (coeff*v)*a;
//    j->to->torque   += (coeff*v)*a;
  }
}

void mlr::KinematicWorld::gravityToForces() {
  mlr::Vector g(0, 0, -9.81);
  for(Body *  n:  bodies) n->force += n->mass * g;
}

/// compute forces from the current contacts
void mlr::KinematicWorld::contactsToForces(double hook, double damp) {
  mlr::Vector trans, transvel, force;
  uint i;
  int a, b;
  for(i=0; i<proxies.N; i++) if(proxies(i)->d<0.) {
      a=proxies(i)->a; b=proxies(i)->b;
      
      //if(!i || proxies(i-1).a!=a || proxies(i-1).b!=b) continue; //no old reference sticking-frame
      //trans = proxies(i)->rel.p - proxies(i-1).rel.p; //translation relative to sticking-frame
      trans    = proxies(i)->posB-proxies(i)->posA;
      //transvel = proxies(i)->velB-proxies(i)->velA;
      //d=trans.length();
      
      force.setZero();
      force += (hook) * trans; //*(1.+ hook*hook*d*d)
      //force += damp * transvel;
      SL_DEBUG(1, cout <<"applying force: [" <<a <<':' <<b <<"] " <<force <<endl);
      
      if(a!=-1) addForce(force, shapes(a)->body, proxies(i)->posA);
      if(b!=-1) addForce(-force, shapes(b)->body, proxies(i)->posB);
    }
}

void mlr::KinematicWorld::kinematicsProxyDist(arr& y, arr& J, Proxy *p, double margin, bool useCenterDist, bool addValues) const {
  mlr::Shape *a = shapes(p->a);
  mlr::Shape *b = shapes(p->b);

  y.resize(1);
  if(&J) J.resize(1, getJointStateDimension());
  if(!addValues){ y.setZero();  if(&J) J.setZero(); }

//  //costs
//  if(a->type==mlr::ST_sphere && b->type==mlr::ST_sphere){
//    mlr::Vector diff=a->X.pos-b->X.pos;
//    double d = diff.length() - a->size(3) - b->size(3);
//    y(0) = d;
//    if(&J){
//      arr Jpos;
//      arr normal = conv_vec2arr(diff)/diff.length(); normal.reshape(1, 3);
//      kinematicsPos(NoArr, Jpos, a->body);  J += (normal*Jpos);
//      kinematicsPos(NoArr, Jpos, b->body);  J -= (normal*Jpos);
//    }
//    return;
//  }
  y(0) = p->d;
  if(&J){
    arr Jpos;
    mlr::Vector arel, brel;
    if(p->d>0.) { //we have a gradient on pos only when outside
      arel=a->X.rot/(p->posA-a->X.pos);
      brel=b->X.rot/(p->posB-b->X.pos);
      CHECK(p->normal.isNormalized(), "proxy normal is not normalized");
      arr normal; normal.referTo(&p->normal.x, 3); normal.reshape(1, 3);
      kinematicsPos(NoArr, Jpos, a->body, arel);  J += (normal*Jpos);
      kinematicsPos(NoArr, Jpos, b->body, brel);  J -= (normal*Jpos);
    }
  }
}

void mlr::KinematicWorld::kinematicsProxyCost(arr& y, arr& J, Proxy *p, double margin, bool useCenterDist, bool addValues) const {
  mlr::Shape *a = shapes(p->a);
  mlr::Shape *b = shapes(p->b);
  CHECK(a->mesh_radius>0.,"");
  CHECK(b->mesh_radius>0.,"");

  y.resize(1);
  if(&J) J.resize(1, getJointStateDimension());
  if(!addValues){ y.setZero();  if(&J) J.setZero(); }

  //costs
  if(a->type==mlr::ST_sphere && b->type==mlr::ST_sphere){
    mlr::Vector diff=a->X.pos-b->X.pos;
    double d = diff.length() - a->size(3) - b->size(3);
    y(0) = 1. - d/margin;
    if(&J){
      arr Jpos;
      arr normal = conv_vec2arr(diff)/diff.length(); normal.reshape(1, 3);
      kinematicsPos(NoArr, Jpos, a->body);  J -= 1./margin*(normal*Jpos);
      kinematicsPos(NoArr, Jpos, b->body);  J += 1./margin*(normal*Jpos);
    }
    return;
  }
  double ab_radius = margin + 10.*(a->mesh_radius+b->mesh_radius);
  CHECK(p->d<(1.+1e-6)*margin, "something's really wierd here!");
  CHECK(p->cenD<(1.+1e-6)*ab_radius, "something's really wierd here! You disproved the triangle inequality :-)");
  double d1 = 1.-p->d/margin;
  double d2 = 1.-p->cenD/ab_radius;
  if(d2<0.) d2=0.;
  if(!useCenterDist) d2=1.;
  y(0) += d1*d2;
 
  //Jacobian
  if(&J){
    arr Jpos;
    mlr::Vector arel, brel;
    if(p->d>0.) { //we have a gradient on pos only when outside
      arel=a->X.rot/(p->posA-a->X.pos);
      brel=b->X.rot/(p->posB-b->X.pos);
      CHECK(p->normal.isNormalized(), "proxy normal is not normalized");
      arr normal; normal.referTo(&p->normal.x, 3); normal.reshape(1, 3);
          
      kinematicsPos(NoArr, Jpos, a->body, arel);  J -= d2/margin*(normal*Jpos);
      kinematicsPos(NoArr, Jpos, b->body, brel);  J += d2/margin*(normal*Jpos);
    }
        
    if(useCenterDist && d2>0.){
      arel=a->X.rot/(p->cenA-a->X.pos);
      brel=b->X.rot/(p->cenB-b->X.pos);
//      CHECK(p->cenN.isNormalized(), "proxy normal is not normalized");
      if(!p->cenN.isNormalized()){
        MLR_MSG("proxy->cenN is not normalized: objects seem to be at exactly the same place");
      }else{
        arr normal; normal.referTo(&p->cenN.x, 3); normal.reshape(1, 3);
        
        kinematicsPos(NoArr, Jpos, a->body, arel);  J -= d1/ab_radius*(normal*Jpos);
        kinematicsPos(NoArr, Jpos, b->body, brel);  J += d1/ab_radius*(normal*Jpos);
      }
    }
  }
}

/// measure (=scalar kinematics) for the contact cost summed over all bodies
void mlr::KinematicWorld::kinematicsProxyCost(arr &y, arr& J, double margin, bool useCenterDist) const {
  y.resize(1).setZero();
  if(&J) J.resize(1, getJointStateDimension()).setZero();
  for(Proxy *p:proxies) if(p->d<margin) {
    kinematicsProxyCost(y, J, p, margin, useCenterDist, true);
  }
}

void mlr::KinematicWorld::kinematicsProxyConstraint(arr& g, arr& J, Proxy *p, double margin) const {
  if(&J) J.resize(1, getJointStateDimension()).setZero();

  g.resize(1) = margin - p->d;

  //Jacobian
  if(&J){
    arr Jpos, normal;
    mlr::Vector arel,brel;
    mlr::Shape *a = shapes(p->a);
    mlr::Shape *b = shapes(p->b);
    if(p->d>0.) { //we have a gradient on pos only when outside
      arel=a->X.rot/(p->posA-a->X.pos);
      brel=b->X.rot/(p->posB-b->X.pos);
      CHECK(p->normal.isNormalized(), "proxy normal is not normalized");
      normal.referTo(&p->normal.x, 3);
    } else { //otherwise take gradient w.r.t. centers...
      arel=a->X.rot/(p->cenA-a->X.pos);
      brel=b->X.rot/(p->cenB-b->X.pos);
      CHECK(p->cenN.isNormalized(), "proxy normal is not normalized");
      normal.referTo(&p->cenN.x, 3);
    }
    normal.reshape(1, 3);

    kinematicsPos(NoArr, Jpos, a->body, arel);  J -= (normal*Jpos);
    kinematicsPos(NoArr, Jpos, b->body, brel);  J += (normal*Jpos);
  }
}

void mlr::KinematicWorld::kinematicsContactConstraints(arr& y, arr &J) const {
  J.clear();
  mlr::Vector normal;
  uint i, con=0;
  Shape *a, *b;
  arr Jpos, dnormal, grad(1, q.N);

  y.clear();
  for(i=0; i<proxies.N; i++) y.append(proxies(i)->d);

  if(!&J) return; //do not return the Jacobian

  mlr::Vector arel, brel;
  for(i=0; i<proxies.N; i++) {
    a=shapes(proxies(i)->a); b=shapes(proxies(i)->b);
    
    arel.setZero();  arel=a->X.rot/(proxies(i)->posA-a->X.pos);
    brel.setZero();  brel=b->X.rot/(proxies(i)->posB-b->X.pos);
    
    CHECK(proxies(i)->normal.isNormalized(), "proxy normal is not normalized");
    dnormal.referTo(proxies(i)->normal.p(), 3); dnormal.reshape(1, 3);
    grad.setZero();
    kinematicsPos(NoArr, Jpos, a->body, arel); grad += dnormal*Jpos; //moving a long normal b->a increases distance
    kinematicsPos(NoArr, Jpos, b->body, brel); grad -= dnormal*Jpos; //moving b long normal b->a decreases distance
    J.append(grad);
    con++;
  }
  J.reshape(con, q.N);
}

void mlr::KinematicWorld::kinematicsLimitsCost(arr &y, arr &J, const arr& limits, double margin) const {
  y.resize(1).setZero();
  if(&J) J.resize(1, getJointStateDimension()).setZero();
  double d;
  for(uint i=0; i<limits.d0; i++) if(limits(i,1)>limits(i,0)){ //only consider proper limits (non-zero interval)
    double m = margin*(limits(i,1)-limits(i,0));
    d = limits(i, 0) + m - q(i); //lo
    if(d>0.) {  y(0) += d/m;  if(&J) J(0, i)-=1./m;  }
    d = q(i) - limits(i, 1) + m; //up
    if(d>0.) {  y(0) += d/m;  if(&J) J(0, i)+=1./m;  }
  }
}

/// Compute the new configuration q such that body is located at ytarget (with deplacement rel).
void mlr::KinematicWorld::inverseKinematicsPos(Body& body, const arr& ytarget,
                                               const mlr::Vector& rel_offset, int max_iter) {
  arr q0, q;
  getJointState(q0);
  q = q0;
  arr y; // endeff pos
  arr J; // Jacobian
  arr invJ;
  arr I = eye(q.N);

  // general inverse kinematic update
  // first iteration: $q* = q' + J^# (y* - y')$
  // next iterations: $q* = q' + J^# (y* - y') + (I - J# J)(q0 - q')$
  for (int i = 0; i < max_iter; i++) {
    kinematicsPos(y, J, &body, rel_offset);
    invJ = ~J * inverse(J * ~J);  // inverse_SymPosDef should work!?
    q = q + invJ * (ytarget - y);

    if (i > 0) {
      q += (I - invJ * J) * (q0 - q);
    }
    setJointState(q);
  }
}

/// center of mass of the whole configuration (3 vector)
double mlr::KinematicWorld::getCenterOfMass(arr& x_) const {
  double M=0.;
  mlr::Vector x;
  x.setZero();
  for(Body *  n:  bodies) {
    M+=n->mass;
    x+=n->mass*n->X.pos;
  }
  x/=M;
  x_ = conv_vec2arr(x);
  return M;
}

/// gradient (Jacobian) of the COM w.r.t. q (3 x n tensor)
void mlr::KinematicWorld::getComGradient(arr &grad) const {
  double M=0.;
  arr J(3, getJointStateDimension());
  grad.resizeAs(J); grad.setZero();
  for(Body * n: bodies) {
    M += n->mass;
    kinematicsPos(NoArr, J, n);
    grad += n->mass * J;
  }
  grad/=M;
}

mlr::Proxy* mlr::KinematicWorld::getContact(uint a, uint b) const {
  uint i;
  for(i=0; i<proxies.N; i++) if(proxies(i)->d<0.) {
      if(proxies(i)->a==(int)a && proxies(i)->b==(int)b) return proxies(i);
      if(proxies(i)->a==(int)b && proxies(i)->b==(int)a) return proxies(i);
    }
  return NULL;
}

arr mlr::KinematicWorld::getHmetric() const{
  arr H = zeros(getJointStateDimension());
  for(mlr::Joint *j:joints) if(j->agent==q_agent){
    double h=j->H;
    CHECK(h>0.,"Hmetric should be larger than 0");
    if(j->type==JT_transXYPhi){
      H(j->qIndex+0)=h*10.;
      H(j->qIndex+1)=h*10.;
      H(j->qIndex+2)=h;
    }else{
      for(uint k=0;k<j->qDim();k++) H(j->qIndex+k)=h;
    }
  }
  return H;
}

/** @brief */
double mlr::KinematicWorld::getEnergy() {
  double m, v, E;
  mlr::Matrix I;
  mlr::Vector w;

  calc_fwdPropagateVelocities();
  
  E=0.;
  for(Body *b: bodies) {
    m=b->mass;
    mlr::Quaternion &rot = b->X.rot;
    I=(rot).getMatrix() * b->inertia * (-rot).getMatrix();
    v=b->vel.length();
    w=b->angvel;
    E += .5*m*v*v;
    E += 9.81 * m * (b->X*b->com).z;
    E += .5*(w*(I*w));
  }
  
  return E;
}

void mlr::KinematicWorld::removeUselessBodies(int verbose) {
  //-- remove bodies and their in-joints
  for_list_rev(Body, b, bodies) if(!b->shapes.N && !b->outLinks.N) {
    if(verbose>0) LOG(0) <<" -- removing useless body " <<b->name <<endl;
    delete b;
  }
  //-- reindex
  listReindex(bodies);
  listReindex(joints);
  checkConsistency();
//  for(Joint * j: joints) j->index=j_COUNT;  j->ifrom = j->from->index;  j->ito = j->to->index;  }
//  for(Shape *s: shapes) s->ibody = s->body->index;
  //-- clear all previous index related things
  qdim.clear();
  proxies.clear();
  analyzeJointStateDimensions();
  calc_q_from_Q();
}

bool mlr::KinematicWorld::checkConsistency(){
  if(qdim.N){
    uint N=getJointStateDimension();
    CHECK_EQ(N, qdim(q_agent), "");
    if(q.N) CHECK_EQ(N, q.N, "");
    if(qdot.N) CHECK_EQ(N, qdot.N, "");

    uintA myqdim(qdim.N);
    myqdim.setZero();
    for(Joint *j: joints){
      CHECK(j->agent<qdim.N, "");

      if(j->mimic){
        CHECK_EQ(j->qIndex, j->mimic->qIndex, "");
      }else{
        CHECK_EQ(j->qIndex, myqdim(j->agent), "joint indexing is inconsistent");
        myqdim(j->agent) += j->qDim();
      }
    }
    CHECK_EQ(myqdim, qdim, "qdim is wrong");
  }

  for(Body *b: bodies){
    CHECK(&b->world, "");
    CHECK(&b->world==this,"");
    CHECK_EQ(b, bodies(b->index), "");
    for(Joint *j: b->outLinks) CHECK_EQ(j->from,b,"");
    for(Joint *j: b->inLinks)  CHECK_EQ(j->to,b,"");
    for(Shape *s: b->shapes)   CHECK_EQ(s->body,b,"");
    b->ats.checkConsistency();
  }

  for(Joint *j: joints){
    CHECK(&j->world && j->from && j->to, "");
    CHECK(&j->world==this,"");
    CHECK_EQ(j, joints(j->index), "");
    CHECK(j->from->outLinks.findValue(j)>=0,"");
    CHECK(j->to->inLinks.findValue(j)>=0,"");
    CHECK_GE(j->type.x, 0, "");
    CHECK_LE(j->type.x, JT_free, "");
    j->ats.checkConsistency();
  }

  //check topsort
  intA level = consts<int>(0, bodies.N);
  //compute levels
  for(Joint *j: joints) level(j->to->index) = level(j->from->index)+1;

  for(Joint *j: joints){
      CHECK(level(j->from->index) < level(j->to->index), "joint does not go forward");
      if(j->index) CHECK_LE(level(joints(j->index-1)->to->index), level(j->to->index), "joints are not sorted");
  }
  for(Body *b: bodies){
    for(Joint *j: b->inLinks)  CHECK(level(j->from->index) < level(b->index), "topsort failed");
  }

  for(Shape *s: shapes){
    CHECK(&s->world, "");
    CHECK(&s->world==this,"");
    CHECK_EQ(s,shapes(s->index),"");
    if(s->body) CHECK(s->body->shapes.findValue(s)>=0,"");
    s->ats.checkConsistency();
  }
  return true;
}

void mlr::KinematicWorld::meldFixedJoints(int verbose) {
  checkConsistency();
  for(Joint *j: joints) if(j->type==JT_rigid) {
    if(verbose>0) LOG(0) <<" -- melding fixed joint " <<j->name <<" (" <<j->from->name <<' ' <<j->to->name <<" )" <<endl;
    Body *a = j->from;
    Body *b = j->to;
    Transformation bridge = j->A * j->Q * j->B;
    //reassociate shapes with a
    for(Shape *s: b->shapes) {
      s->body=a;
      s->rel = bridge * s->rel;
      a->shapes.append(s);
    }
    b->shapes.clear();
    //joints from b-to-c now become joints a-to-c
    for(Joint *jj: b->outLinks) {
      jj->from=a;
      jj->A = bridge * jj->A;
      a->outLinks.append(jj);
    }
    b->outLinks.clear();
    //reassociate mass
    a->mass += b->mass;
    a->inertia += b->inertia;
    b->mass = 0.;
  }
  jointSort();
  calc_q_from_Q();
  checkConsistency();
  //-- remove fixed joints and reindex
  for_list_rev(Joint, jj, joints) if(jj->type==JT_rigid) delete jj;
  listReindex(joints);
  //for(Joint * j: joints) { j->index=j_COUNT;  j->ifrom = j->from->index;  j->ito = j->to->index;  }
  checkConsistency();
}

/// GL routine to draw a mlr::KinematicWorld
#ifdef MLR_GL
void mlr::KinematicWorld::glDraw(OpenGL& gl) {
  uint i=0;
  mlr::Transformation f;
  double GLmatrix[16];

  glPushMatrix();

  glColor(.5, .5, .5);

  //bodies
  if(orsDrawBodies) for(Shape *s: shapes) {
    s->glDraw(gl);
    i++;
    if(orsDrawLimit && i>=orsDrawLimit) break;
  }

  //joints
  if(orsDrawJoints) for(Joint *e: joints) {
    //set name (for OpenGL selection)
    glPushName((e->index <<2) | 2);

    double s=e->A.pos.length()+e->B.pos.length(); //some scale
    s*=.25;

    //from body to joint
    f=e->from->X;
    f.getAffineMatrixGL(GLmatrix);
    glLoadMatrixd(GLmatrix);
    glColor(1, 1, 0);
    //glDrawSphere(.1*s);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(e->A.pos.x, e->A.pos.y, e->A.pos.z);
    glEnd();

    //joint frame A
    f.appendTransformation(e->A);
    f.getAffineMatrixGL(GLmatrix);
    glLoadMatrixd(GLmatrix);
    glDrawAxes(s);
    glColor(1, 0, 0);
    glRotatef(90, 0, 1, 0);  glDrawCylinder(.05*s, .3*s);  glRotatef(-90, 0, 1, 0);

    //joint frame B
    f.appendTransformation(e->Q);
    f.getAffineMatrixGL(GLmatrix);
    glLoadMatrixd(GLmatrix);
    glDrawAxes(s);

    //from joint to body
    glColor(1, 0, 1);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(e->B.pos.x, e->B.pos.y, e->B.pos.z);
    glEnd();
    glTranslatef(e->B.pos.x, e->B.pos.y, e->B.pos.z);
    //glDrawSphere(.1*s);

    glPopName();
    i++;
    if(orsDrawLimit && i>=orsDrawLimit) break;
  }

  //proxies
  if(orsDrawProxies) for(Proxy *proxy: proxies) proxy->glDraw(gl);

  glPopMatrix();
}
#endif

//===========================================================================
//
// Kinematic Switch
//

mlr::KinematicSwitch::KinematicSwitch()
  : symbol(none), jointType(JT_none), timeOfApplication(UINT_MAX), fromId(UINT_MAX), toId(UINT_MAX), agent(0){
  jA.setZero();
  jB.setZero();
}

mlr::KinematicSwitch::KinematicSwitch(OperatorSymbol op, JointType type, const char* ref1, const char* ref2, const mlr::KinematicWorld& K, uint _timeOfApplication, const mlr::Transformation& jFrom, const mlr::Transformation& jTo, uint agent)
  : symbol(op), jointType(type), timeOfApplication(_timeOfApplication), fromId(UINT_MAX), toId(UINT_MAX), agent(agent){
  if(ref1) fromId = K.getShapeByName(ref1)->index;
  if(ref2) toId = K.getShapeByName(ref2)->index;
  if(&jFrom) jA = jFrom;
  if(&jTo)   jB = jTo;
}

#define STEP(t) (floor(t*double(stepsPerPhase) + .500001))-1

void mlr::KinematicSwitch::setTimeOfApplication(double time, bool before, int stepsPerPhase, uint T){
  if(stepsPerPhase<0) stepsPerPhase=T;
  timeOfApplication = STEP(time)+(before?0:1);
}

void mlr::KinematicSwitch::apply(KinematicWorld& G){
  Shape *from=NULL, *to=NULL;
  if(fromId!=UINT_MAX) from=G.shapes(fromId);
  if(toId!=UINT_MAX) to=G.shapes(toId);
  if(fromId==UINT_MAX){
    CHECK_EQ(symbol, deleteJoint, "");
    CHECK(to,"");
    mlr::Body *b = to->body;
    CHECK_LE(b->inLinks.N, 1,"");
    if(b->inLinks.N){
      from = b->inLinks(0)->from->shapes.first();
    }else{
      return;
    }
  }

  if(symbol==deleteJoint){
    Joint *j = G.getJointByBodies(from->body, to->body);
    CHECK(j,"can't find joint between '"<<from->name <<"--" <<to->name <<"' Deleted before?");
    delete j;
    G.jointSort();
    return;
  }
  G.isLinkTree=false;
  if(symbol==addJointZero || symbol==addActuated){
    Joint *j = new Joint(G, from->body, to->body);
    if(agent) j->agent=agent;
    if(symbol==addJointZero) j->constrainToZeroVel=true;
    else                     j->constrainToZeroVel=false;
    j->type=jointType;
    j->A = from->rel * jA;
    j->B = jB * (-to->rel);
    G.jointSort();
    G.calc_fwdPropagateFrames();
    return;
  }
  if(symbol==addJointAtFrom){
    Joint *j = new Joint(G, from->body, to->body);
    if(agent) j->agent=agent;
    j->constrainToZeroVel=true;
    j->type=jointType;
    j->B.setDifference(from->body->X, to->body->X);
    j->A.setZero();
    G.jointSort();
    G.calc_fwdPropagateFrames();
    return;
  }
  if(symbol==addJointAtTo){
    Joint *j = new Joint(G, from->body, to->body);
    if(agent) j->agent=agent;
    j->constrainToZeroVel=true;
    j->type=jointType;
    j->A.setDifference(from->body->X, to->body->X);
    j->B.setZero();
    G.jointSort();
    G.calc_fwdPropagateFrames();
    return;
  }
  if(symbol==addSliderMechanism){
    HALT("I think it is better if there is fixed slider mechanisms in the world, that may jump; no dynamic creation of bodies");
    Body *slider1 = new Body(G); //{ type=ST_box size=[.2 .1 .05 0] color=[0 0 0] }
    Body *slider2 = new Body(G); //{ type=ST_box size=[.2 .1 .05 0] color=[1 0 0] }
    Shape *s1 = new Shape(G, *slider1); s1->type=ST_box; s1->size={.2,.1,.05}; s1->mesh.C={0.,0,0};
    Shape *s2 = new Shape(G, *slider2); s2->type=ST_box; s2->size={.2,.1,.05}; s2->mesh.C={1.,0,0};

    //placement of the slider1 on the table -> fixed
    Joint *j1 = new Joint(G, from->body, slider1);
    j1->type = JT_transXYPhi;
    j1->constrainToZeroVel=true;
    j1->A = from->rel * jA;
    //the actual sliding translation -> articulated
    Joint *j2 = new Joint(G, slider1, slider2);
    j2->type = JT_transX;
    j2->constrainToZeroVel=false;
    //orientation of the object on the slider2 -> fixed
    Joint *j3 = new Joint(G, slider2, to->body);
    j3->type = JT_hingeZ;
    j3->constrainToZeroVel=true;
    j3->B = jB * (-to->rel);

    G.jointSort();
    G.calc_fwdPropagateFrames();
    return;
  }
  HALT("shouldn't be here!");
}

void mlr::KinematicSwitch::temporallyAlign(const mlr::KinematicWorld& Gprevious, mlr::KinematicWorld& G, bool copyFromBodies){
  if(symbol==addJointAtFrom){
    Joint *j = G.getJointByBodies(G.shapes(fromId)->body, G.shapes(toId)->body);
    if(!j/* || j->type!=jointType*/) HALT("");
    if(copyFromBodies){
      j->B.setDifference(Gprevious.shapes(fromId)->body->X, Gprevious.shapes(toId)->body->X);
    }else{//copy from previous, if exists
      Joint *jprev = Gprevious.getJointByBodies(Gprevious.shapes(fromId)->body, Gprevious.shapes(toId)->body);
      if(!jprev || jprev->type!=j->type){//still copy from bodies
        j->B.setDifference(Gprevious.shapes(fromId)->body->X, Gprevious.shapes(toId)->body->X);
      }else{
        j->B = jprev->B;
      }
    }
//    j->A.setZero();
    G.calc_fwdPropagateFrames();
    return;
  }
  if(symbol==addJointAtTo){
    Joint *j = G.getJointByBodies(G.shapes(fromId)->body, G.shapes(toId)->body);
    if(!j || j->type!=jointType) return; //HALT(""); //return;
    if(copyFromBodies){
      j->A.setDifference(Gprevious.shapes(fromId)->body->X, Gprevious.shapes(toId)->body->X);
    }else{
      Joint *jprev = Gprevious.getJointByBodies(Gprevious.shapes(fromId)->body, Gprevious.shapes(toId)->body);
      if(!jprev || jprev->type!=j->type){
        j->A.setDifference(Gprevious.shapes(fromId)->body->X, Gprevious.shapes(toId)->body->X);
      }else{
        j->A = jprev->A;
      }
    }
//    j->B.setZero();
    G.calc_fwdPropagateFrames();
    return;
  }
}

mlr::String mlr::KinematicSwitch::shortTag(const mlr::KinematicWorld* G) const{
  mlr::String str;
  str <<"  timeOfApplication=" <<timeOfApplication;
  str <<"  symbol=" <<symbol;
  str <<"  jointType=" <<jointType;
  str <<"  fromId=" <<(fromId==UINT_MAX?"NULL":(G?G->shapes(fromId)->name:STRING(fromId)));
  str <<"  toId=" <<(G?G->shapes(toId)->name:STRING(toId)) <<endl;
  return str;
}

void mlr::KinematicSwitch::write(std::ostream& os) const{
  os <<"  timeOfApplication=" <<timeOfApplication;
  os <<"  symbol=" <<symbol;
  os <<"  jointType=" <<jointType;
  os <<"  fromId=" <<fromId;
  os <<"  toId=" <<toId <<endl;
}

//===========================================================================

mlr::KinematicSwitch* mlr::KinematicSwitch::newSwitch(const Node *specs, const mlr::KinematicWorld& world, int stepsPerPhase, uint T){
  if(specs->parents.N<2) return NULL;

  //-- get tags
  mlr::String& tt=specs->parents(0)->keys.last();
  mlr::String& type=specs->parents(1)->keys.last();
  const char *ref1=NULL, *ref2=NULL;
  if(specs->parents.N>2) ref1=specs->parents(2)->keys.last().p;
  if(specs->parents.N>3) ref2=specs->parents(3)->keys.last().p;

  if(tt!="MakeJoint") return NULL;
  mlr::KinematicSwitch* sw = newSwitch(type, ref1, ref2, world, stepsPerPhase + 1);

  if(specs->isGraph()){
    const Graph& params = specs->graph();
    sw->setTimeOfApplication(params.get<double>("time",1.), params.get<bool>("time", false), stepsPerPhase, T);
//    sw->timeOfApplication = *stepsPerPhase + 1;
    params.get(sw->jA, "from");
    params.get(sw->jB, "to");
  }
  return sw;
}

mlr::KinematicSwitch* mlr::KinematicSwitch::newSwitch(const mlr::String& type, const char* ref1, const char* ref2, const mlr::KinematicWorld& world, uint _timeOfApplication, const mlr::Transformation& jFrom, const mlr::Transformation& jTo){
  //-- create switch
  mlr::KinematicSwitch *sw= new mlr::KinematicSwitch();
  if(type=="addRigid"){ sw->symbol=mlr::KinematicSwitch::addJointZero; sw->jointType=mlr::JT_rigid; }
//  else if(type=="addRigidRel"){ sw->symbol = mlr::KinematicSwitch::addJointAtTo; sw->jointType=mlr::JT_rigid; }
  else if(type=="rigidAtTo"){ sw->symbol = mlr::KinematicSwitch::addJointAtTo; sw->jointType=mlr::JT_rigid; }
  else if(type=="rigidAtFrom"){ sw->symbol = mlr::KinematicSwitch::addJointAtFrom; sw->jointType=mlr::JT_rigid; }
  else if(type=="rigidZero"){ sw->symbol = mlr::KinematicSwitch::addJointZero; sw->jointType=mlr::JT_rigid; }
  else if(type=="transXActuated"){ sw->symbol = mlr::KinematicSwitch::addActuated; sw->jointType=mlr::JT_transX; }
  else if(type=="transXYPhiAtFrom"){ sw->symbol = mlr::KinematicSwitch::addJointAtFrom; sw->jointType=mlr::JT_transXYPhi; }
  else if(type=="transXYPhiZero"){ sw->symbol = mlr::KinematicSwitch::addJointZero; sw->jointType=mlr::JT_transXYPhi; }
  else if(type=="transXYPhiActuated"){ sw->symbol = mlr::KinematicSwitch::addActuated; sw->jointType=mlr::JT_transXYPhi; }
  else if(type=="freeAtTo"){ sw->symbol = mlr::KinematicSwitch::addJointAtTo; sw->jointType=mlr::JT_free; }
  else if(type=="freeZero"){ sw->symbol = mlr::KinematicSwitch::addJointZero; sw->jointType=mlr::JT_free; }
  else if(type=="freeActuated"){ sw->symbol = mlr::KinematicSwitch::addActuated; sw->jointType=mlr::JT_free; }
  else if(type=="ballZero"){ sw->symbol = mlr::KinematicSwitch::addJointZero; sw->jointType=mlr::JT_quatBall; }
  else if(type=="hingeZZero"){ sw->symbol = mlr::KinematicSwitch::addJointZero; sw->jointType=mlr::JT_hingeZ; }
  else if(type=="sliderMechanism"){ sw->symbol = mlr::KinematicSwitch::addSliderMechanism; }
  else if(type=="delete"){ sw->symbol = mlr::KinematicSwitch::deleteJoint; }
  else HALT("unknown type: "<< type);
  if(ref1) sw->fromId = world.getShapeByName(ref1)->index;
  if(ref2) sw->toId = world.getShapeByName(ref2)->index;
//  if(!ref2){
//    CHECK_EQ(sw->symbol, mlr::KinematicSwitch::deleteJoint, "");
//    mlr::Body *b = fromShape->body;
//    if(b->inLinks.N==1){
////      CHECK_EQ(b->outLinks.N, 0, "");
//      sw->toId = sw->fromId;
//      sw->fromId = b->inLinks(0)->from->shapes.first()->index;
//    }else if(b->outLinks.N==1){
//      CHECK_EQ(b->inLinks.N, 0, "");
//      sw->toId = b->outLinks(0)->from->shapes.first()->index;
//    }else if(b->inLinks.N==0 && b->outLinks.N==0){
//      MLR_MSG("No link to delete for shape '" <<ref1 <<"'");
//      delete sw;
//      return NULL;
//    }else HALT("that's ambiguous");
//  }else{

  sw->timeOfApplication = _timeOfApplication;
  if(&jFrom) sw->jA = jFrom;
  if(&jTo) sw->jB = jTo;
  return sw;
}

const char* mlr::KinematicSwitch::name(mlr::KinematicSwitch::OperatorSymbol s){
  HALT("deprecated");
  static const char* names[] = { "deleteJoint", "addJointZero", "addJointAtFrom", "addJointAtTo", "addArticulated" };
  if(s==none) return "none";
  return names[(int)s];
}


//===========================================================================
//
// helper routines -- in a classical C interface
//

#endif

#undef LEN

double forceClosureFromProxies(mlr::KinematicWorld& ORS, uint bodyIndex, double distanceThreshold, double mu, double torqueWeights) {
  mlr::Vector c, cn;
  arr C, Cn;
  for(mlr::Proxy * p: ORS.proxies){
    int body_a = ORS.shapes(p->a)->body?ORS.shapes(p->a)->body->index:-1;
    int body_b = ORS.shapes(p->b)->body?ORS.shapes(p->b)->body->index:-1;
    if(p->d<distanceThreshold && (body_a==(int)bodyIndex || body_b==(int)bodyIndex)) {
      if(body_a==(int)bodyIndex) {
        c = p->posA;
        cn=-p->normal;
      } else {
        c = p->posB;
        cn= p->normal;
      }
      C.append(conv_vec2arr(c));
      Cn.append(conv_vec2arr(cn));
    }
  }
  C .reshape(C.N/3, 3);
  Cn.reshape(C.N/3, 3);
  double fc=forceClosure(C, Cn, ORS.bodies(bodyIndex)->X.pos, mu, torqueWeights, NULL);
  return fc;
}

void transferQbetweenTwoWorlds(arr& qto, const arr& qfrom, const mlr::KinematicWorld& to, const mlr::KinematicWorld& from){
  arr q = to.getJointState();
  uint T = qfrom.d0;
  uint Nfrom = qfrom.d1;

  if (qfrom.d1==0) {T = 1; Nfrom = qfrom.d0;}

  qto = repmat(~q,T,1);

  intA match(Nfrom);
  match = -1;
  for(mlr::Joint* jfrom:from.joints){
    mlr::Joint* jto = to.getJointByBodyNames(jfrom->from->name, jfrom->to->name);
    if(!jto || !jfrom->qDim() || !jto->qDim()) continue;
    CHECK_EQ(jfrom->qDim(), jto->qDim(), "joints must have same dimensionality");
    for(uint i=0; i<jfrom->qDim(); i++){
      match(jfrom->qIndex+i) = jto->qIndex+i;
    }
  }

  for(uint i=0;i<match.N;i++) if(match(i)!=-1){
    for(uint t=0;t<T;t++){
      if (qfrom.d1==0) {
        qto(t, match(i)) = qfrom(i);
      } else {
        qto(t, match(i)) = qfrom(t,i);
      }
    }
  }

  if (qfrom.d1==0) qto.reshape(qto.N);
}

#if 0 //nonsensical
void transferQDotbetweenTwoWorlds(arr& qDotTo, const arr& qDotFrom, const mlr::KinematicWorld& to, const mlr::KinematicWorld& from){
  //TODO: for saveness reasons, the velocities are zeroed.
  arr qDot;
  qDot = zeros(to.getJointStateDimension());
  uint T, dim;
  if(qDotFrom.d1 > 0) {
    T = qDotFrom.d0;
    qDotTo = repmat(~qDot,T,1);
    dim = qDotFrom.d1;
  } else {
    T = 1;
    qDotTo = qDot;
    dim = qDotFrom.d0;
  }

  intA match(dim);
  match = -1;
  for(mlr::Joint* jfrom:from.joints){
    mlr::Joint* jto = to.getJointByName(jfrom->name, false); //OLD: to.getJointByBodyNames(jfrom->from->name, jfrom->to->name); why???
    if(!jto || !jfrom->qDim() || !jto->qDim()) continue;
    CHECK_EQ(jfrom->qDim(), jto->qDim(), "joints must have same dimensionality");
    for(uint i=0; i<jfrom->qDim(); i++){
      match(jfrom->qIndex+i) = jto->qIndex+i;
    }
  }
  if(qDotFrom.d1 > 0) {
    for(uint i=0;i<match.N;i++) if(match(i)!=-1){
      for(uint t=0;t<T;t++){
        qDotTo(t, match(i)) = qDotFrom(t,i);
      }
    }
  } else {
    for(uint i=0;i<match.N;i++) if(match(i)!=-1){
      qDotTo(match(i)) = qDotFrom(i);
    }
  }

}

void transferKpBetweenTwoWorlds(arr& KpTo, const arr& KpFrom, const mlr::KinematicWorld& to, const mlr::KinematicWorld& from){
  KpTo = zeros(to.getJointStateDimension(),to.getJointStateDimension());
  //use Kp gains from ors file for toWorld, if there are no entries of this joint in fromWorld
  for_list(mlr::Joint, j, to.joints) {
    if(j->qDim()>0) {
      arr *info;
      info = j->ats.find<arr>("gains");
      if(info) {
        KpTo(j->qIndex,j->qIndex)=info->elem(0);
      }
    }
  }

  intA match(KpFrom.d0);
  match = -1;
  for(mlr::Joint* jfrom : from.joints){
    mlr::Joint* jto = to.getJointByName(jfrom->name, false); // OLD: mlr::Joint* jto = to.getJointByBodyNames(jfrom->from->name, jfrom->to->name);
    if(!jto || !jfrom->qDim() || !jto->qDim()) continue;
    CHECK_EQ(jfrom->qDim(), jto->qDim(), "joints must have same dimensionality");
    for(uint i=0; i<jfrom->qDim(); i++){
      match(jfrom->qIndex+i) = jto->qIndex+i;
    }
  }

  for(uint i=0;i<match.N;i++) {
    for(uint j=0;j<match.N;j++){
      KpTo(match(i), match(j)) = KpFrom(i,j);
    }
  }
}

void transferKdBetweenTwoWorlds(arr& KdTo, const arr& KdFrom, const mlr::KinematicWorld& to, const mlr::KinematicWorld& from) {
  KdTo = zeros(to.getJointStateDimension(),to.getJointStateDimension());

  //use Kd gains from ors file for toWorld, if there are no entries of this joint in fromWorld
  for_list(mlr::Joint, j, to.joints) {
    if(j->qDim()>0) {
      arr *info;
      info = j->ats.find<arr>("gains");
      if(info) {
        KdTo(j->qIndex,j->qIndex)=info->elem(1);
      }
    }
  }

  intA match(KdFrom.d0);
  match = -1;
  for(mlr::Joint* jfrom : from.joints){
    mlr::Joint* jto = to.getJointByName(jfrom->name, false); // OLD: mlr::Joint* jto = to.getJointByBodyNames(jfrom->from->name, jfrom->to->name);
    if(!jto || !jfrom->qDim() || !jto->qDim()) continue;
    CHECK_EQ(jfrom->qDim(), jto->qDim(), "joints must have same dimensionality");
    for(uint i=0; i<jfrom->qDim(); i++){
      match(jfrom->qIndex+i) = jto->qIndex+i;
    }
  }

  for(uint i=0;i<match.N;i++) {
    for(uint j=0;j<match.N;j++){
      KdTo(match(i), match(j)) = KdFrom(i,j);
    }
  }
}


void transferU0BetweenTwoWorlds(arr& u0To, const arr& u0From, const mlr::KinematicWorld& to, const mlr::KinematicWorld& from){
  u0To = zeros(to.getJointStateDimension());

  intA match(u0From.d0);
  match = -1;
  for(mlr::Joint* jfrom : from.joints){
    mlr::Joint* jto = to.getJointByName(jfrom->name, false); // OLD: mlr::Joint* jto = to.getJointByBodyNames(jfrom->from->name, jfrom->to->name);
    if(!jto || !jfrom->qDim() || !jto->qDim()) continue;
    CHECK_EQ(jfrom->qDim(), jto->qDim(), "joints must have same dimensionality");
    for(uint i=0; i<jfrom->qDim(); i++){
      match(jfrom->qIndex+i) = jto->qIndex+i;
    }
  }

  for(uint i=0;i<match.N;i++) {
    u0To(match(i)) = u0From(i);
  }
}


void transferKI_ft_BetweenTwoWorlds(arr& KI_ft_To, const arr& KI_ft_From, const mlr::KinematicWorld& to, const mlr::KinematicWorld& from){
  uint numberOfColumns = KI_ft_From.d1;
  if(KI_ft_From.d1 == 0) {
    numberOfColumns = 1;
    KI_ft_To = zeros(to.getJointStateDimension());
  } else {
    KI_ft_To = zeros(to.getJointStateDimension(), KI_ft_From.d1);
  }

  intA match(KI_ft_From.d0);
  match = -1;
  for(mlr::Joint* jfrom : from.joints){
    mlr::Joint* jto = to.getJointByName(jfrom->name, false); // OLD: mlr::Joint* jto = to.getJointByBodyNames(jfrom->from->name, jfrom->to->name);
    if(!jto || !jfrom->qDim() || !jto->qDim()) continue;
    CHECK_EQ(jfrom->qDim(), jto->qDim(), "joints must have same dimensionality");
    for(uint i=0; i<jfrom->qDim(); i++){
      match(jfrom->qIndex+i) = jto->qIndex+i;
    }
  }

  for(uint i=0;i<match.N;i++) {
    for(uint j=0;j < numberOfColumns;j++){
      if(numberOfColumns > 1) {
        KI_ft_To(match(i), j) = KI_ft_From(i,j);
      } else {
        KI_ft_To(match(i)) = KI_ft_From(i);
      }
    }
  }
}
#endif

//===========================================================================
//===========================================================================
// opengl
//===========================================================================
//===========================================================================



#ifndef MLR_ORS_ONLY_BASICS

/**
 * @brief Bind ors to OpenGL.
 * Afterwards OpenGL can show the ors graph.
 *
 * @param graph the ors graph.
 * @param gl OpenGL which shows the ors graph.
 */
void bindOrsToOpenGL(mlr::KinematicWorld& graph, OpenGL& gl) {
  gl.add(glStandardScene, 0);
  gl.add(mlr::glDrawGraph, &graph);
//  gl.setClearColors(1., 1., 1., 1.);

  mlr::Body* glCamera = graph.getBodyByName("glCamera");
  if(glCamera) {
    gl.camera.X = glCamera->X;
    gl.resize(500,500);
  } else {
    gl.camera.setPosition(10., -15., 8.);
    gl.camera.focus(0, 0, 1.);
    gl.camera.upright();
  }
  gl.update();
}
#endif

#ifndef MLR_ORS_ONLY_BASICS

/// static GL routine to draw a mlr::KinematicWorld
void mlr::glDrawGraph(void *classP) {
  ((mlr::KinematicWorld*)classP)->glDraw(NoOpenGL);
}

void mlr::glDrawProxies(void *P){
  ProxyL& proxies = *((ProxyL*)P);
  glPushMatrix();
  for(mlr::Proxy* p:proxies) p->glDraw(NoOpenGL);
  glPopMatrix();
}



void displayState(const arr& x, mlr::KinematicWorld& G, const char *tag){
  G.setJointState(x);
  G.gl().watch(tag);
}

void displayTrajectory(const arr& _x, int steps, mlr::KinematicWorld& G, const KinematicSwitchL& switches, const char *tag, double delay, uint dim_z, bool copyG) {
  if(!steps) return;
  for(mlr::Shape *s : G.shapes) if(s->mesh.V.d0!=s->mesh.Vn.d0 || s->mesh.T.d0!=s->mesh.Tn.d0) {
    s->mesh.computeNormals();
  }
  mlr::KinematicWorld *Gcopy;
  if(switches.N) copyG=true;
  if(!copyG) Gcopy=&G;
  else{
    Gcopy = new mlr::KinematicWorld;
    Gcopy->copy(G,true);
  }
  arr x,z;
  if(dim_z){
    x.referToRange(_x,0,-dim_z-1);
    z.referToRange(_x,-dim_z,-1);
  }else{
    x.referTo(_x);
  }
  uint n=Gcopy->getJointStateDimension()-dim_z;
  x.reshape(x.N/n,n);
  uint num, T=x.d0-1;
  if(steps==1 || steps==-1) num=T; else num=steps;
  for(uint k=0; k<=(uint)num; k++) {
    uint t = (T?(k*T/num):0);
    if(switches.N){
      for(mlr::KinematicSwitch *sw: switches)
        if(sw->timeOfApplication==t)
          sw->apply(*Gcopy);
    }
    if(dim_z) Gcopy->setJointState(cat(x[t], z));
    else Gcopy->setJointState(x[t]);
    if(delay<0.){
      if(delay<-10.) FILE("z.graph") <<*Gcopy;
      Gcopy->gl().watch(STRING(tag <<" (time " <<std::setw(3) <<t <<'/' <<T <<')').p);
    }else{
      Gcopy->gl().update(STRING(tag <<" (time " <<std::setw(3) <<t <<'/' <<T <<')').p);
      if(delay) mlr::wait(delay);
    }
  }
  if(steps==1)
    Gcopy->gl().watch(STRING(tag <<" (time " <<std::setw(3) <<T <<'/' <<T <<')').p);
  if(copyG) delete Gcopy;
}

/* please don't remove yet: code for displaying edges might be useful...

void glDrawOdeWorld(void *classP){
  _glDrawOdeWorld((dWorldID)classP);
}

void _glDrawOdeWorld(dWorldID world)
{
  glStandardLight();
  glColor(3);
  glDrawFloor(4);
  uint i;
  Color c;
  dVector3 vec, vec2;
  dBodyID b;
  dGeomID g, gg;
  dJointID j;
  dReal a, al, ah, r, len;
  glPushName(0);
  int t;

  //bodies
  for(i=0, b=world->firstbody;b;b=(dxBody*)b->next){
    i++;
    glPushName(i);

    //if(b->userdata){ glDrawBody(b->userdata); }
    c.setIndex(i); glColor(c.r, c.g, c.b);
    glShadeModel(GL_FLAT);

    //bodies
    for(g=b->geom;g;g=dGeomGetBodyNext(g)){
      if(dGeomGetClass(g)==dGeomTransformClass){
  ((dxGeomTransform*)g)->computeFinalTx();
        glTransform(((dxGeomTransform*)g)->final_pos, ((dxGeomTransform*)g)->final_R);
  gg=dGeomTransformGetGeom(g);
      }else{
  glTransform(g->pos, g->R);
  gg=g;
      }
      b = dGeomGetBody(gg);
      // set the color of the body, 4. Mar 06 (hh)
      c.r = ((Body*)b->userdata)->cr;
      c.g = ((Body*)b->userdata)->cg;
      c.b = ((Body*)b->userdata)->cb;
      glColor(c.r, c.g, c.b);

      switch(dGeomGetClass(gg))
  {
  case dSphereClass:
    glDrawSphere(dGeomSphereGetRadius(gg));
    break;
  case dBoxClass:
    dGeomBoxGetLengths(gg, vec);
    glDrawBox(vec[0], vec[1], vec[2]);
    break;
  case dCCylinderClass: // 6. Mar 06 (hh)
    dGeomCCylinderGetParams(gg, &r, &len);
    glDrawCappedCylinder(r, len);
    break;
  default: HALT("can't draw that geom yet");
  }
      glPopMatrix();
    }

    // removed shadows,  4. Mar 06 (hh)

    // joints

      dxJointNode *n;
      for(n=b->firstjoint;n;n=n->next){
      j=n->joint;
      t=dJointGetType(j);
      if(t==dJointTypeHinge){
      dJointGetHingeAnchor(j, vec);
      a=dJointGetHingeAngle(j);
      al=dJointGetHingeParam(j, dParamLoStop);
      ah=dJointGetHingeParam(j, dParamHiStop);
      glPushMatrix();
      glTranslatef(vec[0], vec[1], vec[2]);
      dJointGetHingeAxis(j, vec);
      glBegin(GL_LINES);
      glColor3f(1, 0, 0);
      glVertex3f(0, 0, 0);
      glVertex3f(LEN*vec[0], LEN*vec[1], LEN*vec[2]);
      glEnd();
      //glDrawText(STRING(al <<'<' <<a <<'<' <<ah), LEN*vec[0], LEN*vec[1], LEN*vec[2]);
      glPopMatrix();
      }
      if(t==dJointTypeAMotor){
  glPushMatrix();
  glTranslatef(b->pos[0], b->pos[1], b->pos[2]);
  dJointGetAMotorAxis(j, 0, vec);
  glBegin(GL_LINES);
  glColor3f(1, 1, 0);
  glVertex3f(0, 0, 0);
  glVertex3f(LEN*vec[0], LEN*vec[1], LEN*vec[2]);
  glEnd();
  glPopMatrix();
      }
      if(t==dJointTypeBall){
  dJointGetBallAnchor(j, vec);
  dJointGetBallAnchor2(j, vec2);
  glPushMatrix();
  glTranslatef(vec[0], vec[1], vec[2]);
  glBegin(GL_LINES);
  glColor3f(1, 0, 0);
  glVertex3f(-.05, 0, 0);
  glVertex3f(.05, 0, 0);
  glVertex3f(0, -.05, 0);
  glVertex3f(0, .05, 0);
  glVertex3f(0, 0, -.05);
  glVertex3f(0, 0, .05);
  glEnd();
  glPopMatrix();
  glPushMatrix();
  glTranslatef(vec2[0], vec2[1], vec2[2]);
  glBegin(GL_LINES);
  glColor3f(1, 0, 0);
  glVertex3f(-.05, 0, 0);
  glVertex3f(.05, 0, 0);
  glVertex3f(0, -.05, 0);
  glVertex3f(0, .05, 0);
  glVertex3f(0, 0, -.05);
  glVertex3f(0, 0, .05);
  glEnd();
  glPopMatrix();
      }
    }
      glPopName();
  }
  glPopName();
}
*/

int animateConfiguration(mlr::KinematicWorld& C, Inotify *ino) {
  arr x, x0;
  uint t, i;
  C.getJointState(x0);
  arr lim = C.getLimits();
  C.gl().pressedkey=0;
  const int steps = 50;
  for(i=x0.N; i--;) {
    x=x0;
    double upper_lim = lim(i,1);
    double lower_lim = lim(i,0);
    double delta = upper_lim - lower_lim;
    double center = lower_lim + .5*delta;
    if(delta<=1e-10){ center=x0(i); delta=1.; }
    double offset = acos( 2. * (x0(i) - center) / delta );
    if(offset!=offset) offset=0.; //if NAN

    for(t=0; t<steps; t++) {
      if(C.gl().pressedkey==13 || C.gl().pressedkey==27 || C.gl().pressedkey=='q') return C.gl().pressedkey;
      if(ino && ino->pollForModification()) return 13;

      x(i) = center + (delta*(0.5*cos(MLR_2PI*t/steps + offset)));
      // Joint limits
      checkNan(x);
      C.setJointState(x);
      C.gl().update(STRING("DOF = " <<i), false, false, true);
      mlr::wait(0.01);
    }
  }
  C.setJointState(x0);
  return C.gl().update("", false, false, true);
}


mlr::Body *movingBody=NULL;
mlr::Vector selpos;
double seld, selx, sely, selz;

struct EditConfigurationClickCall:OpenGL::GLClickCall {
  mlr::KinematicWorld *ors;
  EditConfigurationClickCall(mlr::KinematicWorld& _ors) { ors=&_ors; }
  bool clickCallback(OpenGL& gl) {
    OpenGL::GLSelect *top=gl.topSelection;
    if(!top) return false;
    uint i=top->name;
    cout <<"CLICK call: id = 0x" <<std::hex <<gl.topSelection->name <<" : ";
    gl.text.clear();
    if((i&3)==1) {
      mlr::Shape *s=ors->shapes(i>>2);
      gl.text <<"shape selection: shape=" <<s->name <<" body=" <<s->body->name <<" X=" <<s->X <<endl;
//      listWrite(s->ats, gl.text, "\n");
      cout <<gl.text;
    }
    if((i&3)==2) {
      mlr::Joint *j=ors->joints(i>>2);
      gl.text
          <<"edge selection: " <<j->from->name <<' ' <<j->to->name
         <<"\nA=" <<j->A <<"\nQ=" <<j->Q <<"\nB=" <<j->B <<endl;
//      listWrite(j->ats, gl.text, "\n");
      cout <<gl.text;
    }
    cout <<endl;
    return true;
  }
};

struct EditConfigurationHoverCall:OpenGL::GLHoverCall {
  mlr::KinematicWorld *ors;
  EditConfigurationHoverCall(mlr::KinematicWorld& _ors);// { ors=&_ors; }
  bool hoverCallback(OpenGL& gl) {
//    if(!movingBody) return false;
    if(!movingBody) {
      mlr::Joint *j=NULL;
      mlr::Shape *s=NULL;
      mlr::timerStart(true);
      gl.Select(true);
      OpenGL::GLSelect *top=gl.topSelection;
      if(!top) return false;
      uint i=top->name;
      cout <<mlr::timerRead() <<"HOVER call: id = 0x" <<std::hex <<gl.topSelection->name <<endl;
      if((i&3)==1) s=ors->shapes(i>>2);
      if((i&3)==2) j=ors->joints(i>>2);
      gl.text.clear();
      if(s) {
        gl.text <<"shape selection: body=" <<s->body->name <<" X=" <<s->body->X <<" ats=" <<endl;
        listWrite(s->ats, gl.text, "\n");
      }
      if(j) {
        gl.text
            <<"edge selection: " <<j->from->name <<' ' <<j->to->name
           <<"\nA=" <<j->A <<"\nQ=" <<j->Q <<"\nB=" <<j->B <<endl;
        listWrite(j->ats, gl.text, "\n");
      }
    } else {
      //gl.Select();
      //double x=0, y=0, z=seld;
      //double x=(double)gl.mouseposx/gl.width(), y=(double)gl.mouseposy/gl.height(), z=seld;
      double x=gl.mouseposx, y=gl.mouseposy, z=seld;
      gl.unproject(x, y, z, true);
      cout <<"x=" <<x <<" y=" <<y <<" z=" <<z <<" d=" <<seld <<endl;
      movingBody->X.pos = selpos + ARR(x-selx, y-sely, z-selz);
    }
    return true;
  }
};

EditConfigurationHoverCall::EditConfigurationHoverCall(mlr::KinematicWorld& _ors) {
  ors=&_ors;
}

struct EditConfigurationKeyCall:OpenGL::GLKeyCall {
  mlr::KinematicWorld &K;
  bool &exit;
  EditConfigurationKeyCall(mlr::KinematicWorld& _K, bool& _exit): K(_K), exit(_exit){}
  bool keyCallback(OpenGL& gl) {
    if(gl.pressedkey==' '){ //grab a body
      if(movingBody) { movingBody=NULL; return true; }
      mlr::Joint *j=NULL;
      mlr::Shape *s=NULL;
      gl.Select();
      OpenGL::GLSelect *top=gl.topSelection;
      if(!top) { cout <<"No object below mouse!" <<endl;  return false; }
      uint i=top->name;
      //cout <<"HOVER call: id = 0x" <<std::hex <<gl.topSelection->name <<endl;
      if((i&3)==1) s=K.shapes(i>>2);
      if((i&3)==2) j=K.joints(i>>2);
      if(s) {
        cout <<"selected shape " <<s->name <<" of body " <<s->body->name <<endl;
        selx=top->x;
        sely=top->y;
        selz=top->z;
        seld=top->dmin;
        cout <<"x=" <<selx <<" y=" <<sely <<" z=" <<selz <<" d=" <<seld <<endl;
        selpos = s->body->X.pos;
        movingBody=s->body;
      }
      if(j) {
        cout <<"selected joint " <<j->index <<" connecting " <<j->from->name <<"--" <<j->to->name <<endl;
      }
      return true;
    }else switch(gl.pressedkey) {
      case '1':  K.orsDrawBodies^=1;  break;
      case '2':  K.orsDrawShapes^=1;  break;
      case '3':  K.orsDrawJoints^=1;  K.orsDrawMarkers^=1; break;
      case '4':  K.orsDrawProxies^=1;  break;
      case '5':  gl.reportSelects^=1;  break;
      case '6':  gl.reportEvents^=1;  break;
      case '7':  K.writePlyFile("z.ply");  break;
      case 'j':  gl.camera.X.pos += gl.camera.X.rot*mlr::Vector(0, 0, .1);  break;
      case 'k':  gl.camera.X.pos -= gl.camera.X.rot*mlr::Vector(0, 0, .1);  break;
      case 'i':  gl.camera.X.pos += gl.camera.X.rot*mlr::Vector(0, .1, 0);  break;
      case ',':  gl.camera.X.pos -= gl.camera.X.rot*mlr::Vector(0, .1, 0);  break;
      case 'l':  gl.camera.X.pos += gl.camera.X.rot*mlr::Vector(.1, .0, 0);  break;
      case 'h':  gl.camera.X.pos -= gl.camera.X.rot*mlr::Vector(.1, 0, 0);  break;
      case 'a':  gl.camera.focus(
          (gl.camera.X.rot*(gl.camera.foc - gl.camera.X.pos)
           ^ gl.camera.X.rot*mlr::Vector(1, 0, 0)) * .001
          + gl.camera.foc);
        break;
      case 's':  gl.camera.X.pos +=
          (
            gl.camera.X.rot*(gl.camera.foc - gl.camera.X.pos)
            ^(gl.camera.X.rot * mlr::Vector(1., 0, 0))
          ) * .01;
        break;
      case 'q' :
        cout <<"EXITING" <<endl;
        exit=true;
        break;
    }
    gl.postRedrawEvent(true);
    return true;
  }
};

void editConfiguration(const char* filename, mlr::KinematicWorld& C) {
//  gl.exitkeys="1234567890qhjklias, "; //TODO: move the key handling to the keyCall!
  bool exit=false;
//  C.gl().addHoverCall(new EditConfigurationHoverCall(C));
  C.gl().addKeyCall(new EditConfigurationKeyCall(C,exit));
  C.gl().addClickCall(new EditConfigurationClickCall(C));
  Inotify ino(filename);
  for(;!exit;) {
    cout <<"reloading `" <<filename <<"' ... " <<std::endl;
    mlr::KinematicWorld W;
    try {
      mlr::lineCount=1;
      W <<FILE(filename);
      C.gl().dataLock.writeLock();
      C = W;
      C.gl().dataLock.unlock();
    } catch(const char* msg) {
      cout <<"line " <<mlr::lineCount <<": " <<msg <<" -- please check the file and press ENTER" <<endl;
      C.gl().watch();
      continue;
    }
    C.gl().update();
    if(exit) break;
    cout <<"animating.." <<endl;
    //while(ino.pollForModification());
    animateConfiguration(C, &ino);
    if(exit) break;
    cout <<"watching..." <<endl;
#if 0
    ino.waitForModification();
#else
    C.gl().watch();
#endif
    if(!mlr::getInteractivity()){
      exit=true;
    }
  }
}

//#endif

#else ///MLR_GL
#ifndef MLR_ORS_ONLY_BASICS
void bindOrsToOpenGL(mlr::KinematicWorld&, OpenGL&) { NICO };
void mlr::KinematicWorld::glDraw(OpenGL&) { NICO }
void mlr::glDrawGraph(void *classP) { NICO }
void editConfiguration(const char* orsfile, mlr::KinematicWorld& C) { NICO }
void animateConfiguration(mlr::KinematicWorld& C, Inotify*) { NICO }
void glTransform(const mlr::Transformation&) { NICO }
void displayTrajectory(const arr&, int, mlr::KinematicWorld&, const char*, double) { NICO }
void displayState(const arr&, mlr::KinematicWorld&, const char*) { NICO }
#endif
#endif
/** @} */

//===========================================================================
//===========================================================================
// featherstone
//===========================================================================
//===========================================================================

/** interface and implementation to Featherstone's Articulated Body Algorithm

  See resources from http://users.rsise.anu.edu.au/~roy/spatial/index.html

  This is a direct port of the following two files
  http://users.rsise.anu.edu.au/~roy/spatial/ws04spatial.txt
  http://users.rsise.anu.edu.au/~roy/spatial/ws04abadyn.txt

  See also his slides on the spatial vector algebra
  http://users.rsise.anu.edu.au/~roy/spatial/slidesX4.pdf

  main changes for porting to C:

  indexing of arrays start from 0

  referencing sub arrays with [i] rather than {i}

  NOTE: Featherstone's rotation matricies have opposite convention than mine
*/
namespace Featherstone {
/// returns a cross-product matrix X such that \f$v \times y = X y\f$
void skew(arr& X, const double *v);

/// as above
arr skew(const double *v);

/** @brief MM6 coordinate transform from X-axis rotation.  Xrotx(h)
  calculates the MM6 coordinate transform matrix (for motion
  vectors) induced by a rotation about the +X axis by an angle h (in
  radians).  Positive rotation is anticlockwise: +Y axis rotates
  towards +Z axis.
*/
void Xrotx(arr& X, double h);

/** @brief MM6 coordinate transform from Y-axis rotation.  Xroty(h)
  calculates the MM6 coordinate transform matrix (for motion
  vectors) induced by a rotation about the +Y axis by an angle h (in
  radians).  Positive rotation is anticlockwise: +Z axis rotates
  towards +X axis.
*/
void Xroty(arr& X, double h);

/** @brief MM6 coordinate transform from Z-axis rotation.  Xrotz(h)
  calculates the MM6 coordinate transform matrix (for motion
  vectors) induced by a rotation about the +Z axis by an angle h (in
  radians).  Positive rotation is anticlockwise: +X axis rotates
  towards +Y axis.
*/
void Xrotz(arr& X, double h);


/** @brief MM6 coordinate transform from 3D translation vector.
  Xtrans(r) calculates the MM6 coordinate transform matrix (for
  motion vectors) induced by a shift of origin specified by the 3D
  vector r, which contains the x, y and z coordinates of the new
  location of the origin relative to the old.
*/
void Xtrans(arr& X, double* r);


/** @brief Calculate RBI from mass, CoM and rotational inertia.
  RBmci(m, c, I) calculate MF6 rigid-body inertia tensor for a body
  with mass m, centre of mass at c, and (3x3) rotational inertia
  about CoM of I.
*/
void RBmci(arr& rbi, double m, double *c, const mlr::Matrix& I);

/** @brief MM6 cross-product tensor from M6 vector.  crossM(v)
  calculates the MM6 cross-product tensor of motion vector v such
  that crossM(v) * m = v X m (cross-product of v and m) where m is
  any motion vector or any matrix or tensor mapping to M6.
*/
void crossM(arr& vcross, const arr& v);

/// as above
arr crossM(const arr& v);

/** @brief FF6 cross-product tensor from M6 vector.  crossF(v)
  calculates the FF6 cross-product tensor of motion vector v such
  that crossF(v) * f = v X f (cross-product of v and f) where f is
  any force vector or any matrix or tensor mapping to F6.
*/
void crossF(arr& vcross, const arr& v);

/// as above
arr crossF(const arr& v);
}
//#define Qstate

#if 0
mlr::Body *robotbody(uint i, const Featherstone::Robot& robot) { return robot.C->nodes(i); }

uint Featherstone::Robot::N() const { return C->nodes.N; }

int Featherstone::Robot::parent(uint i) const {
  mlr::Joint *e=C->nodes(i)->inLinks(0);
  if(e) return e->from->index;
  return -1;
}

byte Featherstone::Robot::dof(uint i) const {
  mlr::Body *n=C->nodes(i);
  if(n->fixed) return 0;
  if(n->inLinks.N) {
    switch(n->inLinks(0)->type) {
      case 0: return 1;
#ifndef Qstate
      case 4: return 3;
#else
      case 4: return 4;
#endif
    }
  }
  return 6;
}

const arr Featherstone::Robot::S(uint i) const {
  byte d_i=dof(i);
  arr S;
  mlr::Quaternion r;
  mlr::Matrix R;
  arr Ss, rr;
  switch(d_i) {
    case 0: S.resize(6, (uint)0); S.setZero(); break;
    case 1: S.resize(6, 1); S.setZero(); S(0, 0)=1.; break;
    case 3:
      S.resize(6, 3); S.setZero();
      S(0, 0)=1.; S(1, 1)=1.; S(2, 2)=1.;
      break;
      r = C->nodes(i)->inLinks(0)->X.r;
      r.invert();
      r.getMatrix(R.m);
      memmove(S.p, R.m, 9*sizeof(double));
      break;
    case 4:
      S.resize(6, 4); S.setZero();
      r = C->nodes(i)->inLinks(0)->X.r;
      r.invert();
      r.getMatrix(R.m);
      S(0, 0)= 0;  S(0, 1)= R(0, 0);  S(0, 2)= R(0, 1);  S(0, 3)= R(0, 2);
      S(1, 0)= 0;  S(1, 1)= R(1, 0);  S(1, 2)= R(1, 1);  S(1, 3)= R(1, 2);
      S(2, 0)= 0;  S(2, 1)= R(2, 0);  S(2, 2)= R(2, 1);  S(2, 3)= R(2, 2);
      S *= 2.;
      break;
    case 6:
      S.resize(6, 6); S.setZero();
      //S(0, 3)=1.; S(1, 4)=1.; S(2, 5)=1.;
      S(3, 0)=1.; S(4, 1)=1.; S(5, 2)=1.;
      break; //S(1, 1)=S(2, 2)=1.; break;
      //case 6: S.setId(6); break;
    default: NIY;
  }
  return S;
}

/* returns the transformation from the parent link to the i-th link */
const arr Featherstone::Robot::Xlink(uint i) const {
  //slide 15
  mlr::Transformation f;
  mlr::Joint *e1=C->nodes(i)->firstIn;
  if(!e1) {
    f.setZero();
  } else {
    mlr::Joint *e0=e1->from->firstIn;
    if(e0) {
      f = e0->B;
      f.addRelativeFrame(e1->A);
      //f.addRelativeFrame(e1->X);
    } else {
      //NIY;
      f = e1->from->X;
      f.addRelativeFrame(e1->A);
      //f.addRelativeFrame(e1->X);
    }
  }
  arr X;
  FrameToMatrix(X, f);
  return X;
}

const arr Featherstone::Robot::Ilink(uint i) const {
  //taken from slide 27
  mlr::Joint *e=C->nodes(i)->firstIn;
  double m=C->nodes(i)->mass;
  mlr::Vector com;
  if(e) com = e->B.p; else com = C->nodes(i)->X.p;
  //arr Ic(3, 3);  Ic.setDiag(.1*m);
  arr I;
  RBmci(I, m, com.v, C->nodes(i)->inertia);
  return I;
}

const arr Featherstone::Robot::force(uint i) const {
  mlr::Body *n=C->nodes(i);
  CHECK(n, "is not a body with input joint");
  mlr::Joint *e=n->firstIn;
  //CHECK(e, "is not a body with input joint");
  mlr::Transformation g;
  g=n->X;
  if(e) g.subRelativeFrame(e->B);
  mlr::Vector fo = g.r/n->force;
  mlr::Vector to;
  if(e) to = g.r/(n->torque + (g.r*e->B.p)^n->force);
  else  to = g.r/(n->torque);
  arr f(6);
  f(0)=to.x;  f(1)=to.y;  f(2)=to.z;
  f(3)=fo.x;  f(4)=fo.y;  f(5)=fo.z;
  return f;
}
#endif

void Featherstone::skew(arr& X, const double *v) {
  X.resize(3, 3);  X.setZero();
  X(0, 1) = -v[2];  X(1, 0) = v[2];
  X(1, 2) = -v[0];  X(2, 1) = v[0];
  X(2, 0) = -v[1];  X(0, 2) = v[1];
}

arr Featherstone::skew(const double *v) { arr X; skew(X, v); return X; }

void FrameToMatrix(arr &X, const mlr::Transformation& f) {
  arr z(3, 3);  z.setZero();
  arr r(3, 3);  Featherstone::skew(r, &f.pos.x);
  arr R(3, 3);  f.rot.getMatrix(R.p);
  transpose(R);
  X.resize(6, 6);  X.setBlockMatrix(R, z, R*~r, R); //[[unklar!!]]
  //cout <<"\nz=" <<z <<"\nr=" <<r <<"\nR=" <<R <<"\nX=" <<X <<endl;
}

void mlr::Link::setFeatherstones() {
  switch(type) {
    case -1:     CHECK_EQ(parent,-1, ""); _h.clear();  break;
    case JT_rigid:
    case JT_transXYPhi:
      qIndex=-1;
      _h=zeros(6);
      break;
    case JT_hingeX: _h.resize(6); _h.setZero(); _h(0)=1.; break;
    case JT_hingeY: _h.resize(6); _h.setZero(); _h(1)=1.; break;
    case JT_hingeZ: _h.resize(6); _h.setZero(); _h(2)=1.; break;
    case JT_transX: _h.resize(6); _h.setZero(); _h(3)=1.; break;
    case JT_transY: _h.resize(6); _h.setZero(); _h(4)=1.; break;
    case JT_transZ: _h.resize(6); _h.setZero(); _h(5)=1.; break;
    default: NIY;
  }
  Featherstone::RBmci(_I, mass, com.p(), inertia);
  
  updateFeatherstones();
}

void mlr::Link::updateFeatherstones() {
  FrameToMatrix(_A, A);
  FrameToMatrix(_Q, Q);
  
  mlr::Transformation XQ;
  XQ=X;
  XQ.appendTransformation(Q);
  mlr::Vector fo = XQ.rot/force;
  mlr::Vector to = XQ.rot/(torque + ((XQ.rot*com)^force));
  _f.resize(6);
  _f(0)=to.x;  _f(1)=to.y;  _f(2)=to.z;
  _f(3)=fo.x;  _f(4)=fo.y;  _f(5)=fo.z;
}

void GraphToTree(mlr::Array<mlr::Link>& tree, const mlr::KinematicWorld& C) {
  tree.resize(C.bodies.N);
  
  for(mlr::Link& link:tree){ link.parent=-1; link.qIndex=-1; }

  for(mlr::Body* body:C.bodies) {
    mlr::Link& link=tree(body->index);
    if(body->inLinks.N && body->inLinks(0)->qDim()) { //is not a root
      CHECK_EQ(body->inLinks.N,1, "this is not a tree");
      mlr::Joint *j=body->inLinks(0);
      
      link.type   = j->type;
      link.qIndex = j->qIndex;
      link.parent = j->from->index;
      
      link.com = j->B*body->com;

      if(j->from->inLinks.N) link.A=j->from->inLinks(0)->B;
      else link.A=j->from->X;
      link.A.appendTransformation(j->A);
      
      link.X = j->from->X;
      link.X.appendTransformation(j->A);
      
      link.Q=j->Q;
    } else {
//      CHECK_EQ(body->inLinks.N,0, "dammit");
      
      link.type=-1;
      link.qIndex=-1;
      link.parent=-1;
      link.com=body->X*body->com;
      link.A.setZero();
      link.X.setZero();
      link.Q.setZero();
    }
    link.mass=body->mass; CHECK(link.mass>0. || link.qIndex==-1, "a moving link without mass -> this will diverge");
    link.inertia=body->inertia;
    link.force=body->force;
    link.torque=body->torque;
  }

  for(mlr::Link& link:tree) link.setFeatherstones();
}

void updateGraphToTree(mlr::Array<mlr::Link>& tree, const mlr::KinematicWorld& C) {
  CHECK_EQ(tree.N,C.bodies.N, "");
  
  uint i;
  mlr::Body *p;
  mlr::Joint *e;
  mlr::Transformation f;
  i=0;
  for_list(mlr::Body, n, C.bodies) {
    i=n_COUNT;
    if(n->inLinks.N) {
      e=n->inLinks(0);
      p=e->from;
      
      if(!p->inLinks.N) {
        f=p->X;
        f.appendTransformation(e->A);
        tree(i).A=f;
      }
      
      f = p->X;
      f.appendTransformation(e->A);
      tree(i).X=f;
      
      tree(i).Q=e->Q;
    } else {
      tree(i).com=n->X.pos;
    }
    tree(i).force=n->force;
    tree(i).torque=n->torque;
  }
  for(i=0; i<tree.N; i++) tree(i).updateFeatherstones();
}


/*
----------- Xrotx.m ----------------------------------------------------------
*/
void Featherstone::Xrotx(arr& X, double h) {
  /*
  % Xrotx  MM6 coordinate transform from X-axis rotation.
  % Xrotx(h) calculates the MM6 coordinate transform matrix (for motion
  % vectors) induced by a rotation about the +X axis by an angle h (in radians).
  % Positive rotation is anticlockwise: +Y axis rotates towards +Z axis.
  */
  double c = cos(h), s = sin(h);
  X.resize(6, 6); X.setZero();
  X(0, 0)= X(3, 3)= 1.;
  X(1, 1)= X(2, 2)= X(4, 4)= X(5, 5)= c;
  X(1, 2)= X(4, 5)=  s;
  X(2, 1)= X(5, 4)= -s;
  /* X = [
     1  0  0  0  0  0 ;
     0  c  s  0  0  0 ;
     0 -s  c  0  0  0 ;
     0  0  0  1  0  0 ;
     0  0  0  0  c  s ;
     0  0  0  0 -s  c
      ]; */
}

/*
----------- Xroty.m ----------------------------------------------------------
*/
void Featherstone::Xroty(arr& X, double h) {
  /*
  % Xroty  MM6 coordinate transform from Y-axis rotation.
  % Xroty(h) calculates the MM6 coordinate transform matrix (for motion
  % vectors) induced by a rotation about the +Y axis by an angle h (in radians).
  % Positive rotation is anticlockwise: +Z axis rotates towards +X axis.
  */
  double c = cos(h), s = sin(h);
  X.resize(6, 6);  X.setZero();
  X(1, 1)= X(4, 4)= 1.;
  X(0, 0)= X(2, 2)= X(3, 3)= X(5, 5)= c;
  X(0, 2)= X(3, 5)= -s;
  X(2, 0)= X(5, 3)=  s;
  /* X = [
     c  0 -s  0  0  0 ;
     0  1  0  0  0  0 ;
     s  0  c  0  0  0 ;
     0  0  0  c  0 -s ;
     0  0  0  0  1  0 ;
     0  0  0  s  0  c
     ]; */
}

/*
----------- Xrotz.m ----------------------------------------------------------
*/
void Featherstone::Xrotz(arr& X, double h) {
  /*
  % Xrotz  MM6 coordinate transform from Z-axis rotation.
  % Xrotz(h) calculates the MM6 coordinate transform matrix (for motion
  % vectors) induced by a rotation about the +Z axis by an angle h (in radians).
  % Positive rotation is anticlockwise: +X axis rotates towards +Y axis.
  */
  double c = cos(h), s = sin(h);
  X.resize(6, 6);  X.setZero();
  X(2, 2)= X(5, 5)= 1.;
  X(0, 0)= X(1, 1)= X(3, 3)= X(4, 4)= c;
  X(0, 1)= X(3, 4)=  s;
  X(1, 0)= X(4, 3)= -s;
  /* X = [
     c  s  0  0  0  0 ;
     -s  c  0  0  0  0 ;
     0  0  1  0  0  0 ;
     0  0  0  c  s  0 ;
     0  0  0 -s  c  0 ;
     0  0  0  0  0  1 ]; */
}


/*
----------- Xtrans.m ---------------------------------------------------------
*/
void Featherstone::Xtrans(arr& X, double* r) {
  /*
  % Xtrans  MM6 coordinate transform from 3D translation vector.
  % Xtrans(r) calculates the MM6 coordinate transform matrix (for motion
  % vectors) induced by a shift of origin specified by the 3D vector r, which
  % contains the x, y and z coordinates of the new location of the origin
  % relative to the old.
  */
  X.resize(6, 6);  X.setId();
  X.setMatrixBlock(-skew(r), 3, 0);
  /* X = [
      1     0     0    0  0  0 ;
      0     1     0    0  0  0 ;
      0     0     1    0  0  0 ;
      0     r(3) -r(2) 1  0  0 ;
     -r(3)  0     r(1) 0  1  0 ;
      r(2) -r(1)  0    0  0  1
     ]; */
}

/*
----------- RBmci.m ----------------------------------------------------------
*/
void Featherstone::RBmci(arr& rbi, double m, double *c, const mlr::Matrix& I) {
  /*
  % RBmci  Calculate RBI from mass, CoM and rotational inertia.
  % RBmci(m, c, I) calculate MF6 rigid-body inertia tensor for a body with
  % mass m, centre of mass at c, and (3x3) rotational inertia about CoM of I.
  */
  arr C(3, 3);
  skew(C, c);
  //C = [ 0, -c(3), c(2); c(3), 0, -c(1); -c(2), c(1), 0 ];
  arr II;
  II.referTo(&I.m00, 9);
  II.reshape(3, 3);
  
  rbi.setBlockMatrix(II + m*C*~C, m*C, m*~C, m*eye(3));
  //rbi = [ I + m*C*C', m*C; m*C', m*eye(3) ];
}

//===========================================================================
void Featherstone::crossF(arr& vcross, const arr& v) {
  /*
  % crossF  FF6 cross-product tensor from M6 vector.
  % crossF(v) calculates the FF6 cross-product tensor of motion vector v
  % such that crossF(v) * f = v X f (cross-product of v and f) where f is any
  % force vector or any matrix or tensor mapping to F6.
  */
  crossM(vcross, v);
  transpose(vcross);
  vcross *= (double)-1.;
  //vcross = -crossM(v)';
}

arr Featherstone::crossF(const arr& v) { arr X; crossF(X, v); return X; }

//===========================================================================
void Featherstone::crossM(arr& vcross, const arr& v) {
  /*
  % crossM  MM6 cross-product tensor from M6 vector.
  % crossM(v) calculates the MM6 cross-product tensor of motion vector v
  % such that crossM(v) * m = v X m (cross-product of v and m) where m is any
  % motion vector or any matrix or tensor mapping to M6.
  */
  CHECK(v.nd==1 && v.N==6, "");
  vcross.resize(6, 6);  vcross.setZero();
  
  arr vc;  skew(vc, v.p);
  vcross.setMatrixBlock(vc, 0, 0);
  vcross.setMatrixBlock(vc, 3, 3);
  vcross.setMatrixBlock(skew(v.p+3), 3, 0);
  /* vcross = [
      0    -v(3)  v(2)   0     0     0    ;
      v(3)  0    -v(1)   0     0     0    ;
     -v(2)  v(1)  0      0     0     0    ;
      0    -v(6)  v(5)   0    -v(3)  v(2) ;
      v(6)  0    -v(4)   v(3)  0    -v(1) ;
     -v(5)  v(4)  0     -v(2)  v(1)  0
     ]; */
}

arr Featherstone::crossM(const arr& v) { arr X; crossM(X, v); return X; }

//===========================================================================
#if 0
void Featherstone::invdyn_old(arr& tau, const Robot& robot, const arr& qd, const arr& qdd, const arr& grav) {
  /*
  % INVDYN  Calculate robot inverse dynamics.
  % invdyn(robot, q, qd, qdd) calculates the inverse dynamics of a robot using
  % the recursive Newton-Euler algorithm, evaluated in link coordinates.
  % Gravity is simulated by a fictitious base acceleration of [0, 0, 9.81] m/s^2
  % in base coordinates.  This can be overridden by supplying a 3D vector as
  % an optional fifth argument.
  */
  
  arr grav_accn(6);
  grav_accn.setZero();
  if(!grav.N) {
    //grav_accn(5)=9.81;
  } else {
    grav_accn.setVectorBlock(grav, 3);
  }
  
  uint i, N=robot.N(), d_i, n;
  mlr::Array<arr> S(N), qd_i(N), qdd_i(N), tau_i(N);
  arr Xup(N, 6, 6), v(N, 6), f(N, 6), a(N, 6);
  arr Q;
  
  for(i=0, n=0; i<N; i++) {
    d_i=robot.dof(i);
    if(d_i) {
      qd_i(i) .referToRange(qd , n, n+d_i-1);
      qdd_i(i).referToRange(qdd, n, n+d_i-1);
      tau_i(i).referToRange(tau, n, n+d_i-1);
    } else {
      qd_i(i) .clear(); qd_i(i). resize(0);
      qdd_i(i).clear(); qdd_i(i).resize(0);
      tau_i(i).clear(); tau_i(i).resize(0);
    }
    n += d_i;
    S(i) = robot.S(i);
    if(robot.C->nodes(i)->inLinks.N) {
      FrameToMatrix(Q, robot.C->nodes(i)->inLinks(0)->X);
      Xup[i] = Q * robot.Xlink(i); //the transformation from the i-th to the j-th
    } else {
      Xup[i] = robot.Xlink(i); //the transformation from the i-th to the j-th
    }
  }
  CHECK(n==qd.N && n==qdd.N && n==tau.N, "")
  
  for(i=0; i<N; i++) {
    if(robot.parent(i) == -1) {
      v[i] = S(i) * qd_i(i);
      a[i] = Xup[i]*grav_accn + S(i)*qdd_i(i);
    } else {
      v[i] = Xup[i] * v[robot.parent(i)] + S(i) * qd_i(i);
      a[i] = Xup[i] * a[robot.parent(i)] + S(i) * qdd_i(i) + crossM(v[i])*S(i)*qd_i(i);
    }
    f[i] = robot.Ilink(i)*a[i] + crossF(v[i])*robot.Ilink(i)*v[i] - robot.force(i);
    
#if 0
    if(i) {
      mlr::Transformation f, r, g;
      f=robot.C->nodes(i)->X;
      f.subRelativeFrame(robot.C->nodes(i)->inLinks(0)->B);
      arr vi(6);  vi.setVectorBlock(arr((f.r/f.w).v, 3), 0);  vi.setVectorBlock(arr((f.r/f.v).v, 3), 3);
      arr ai(6);  ai.setVectorBlock(arr((f.r/f.b).v, 3), 0);  ai.setVectorBlock(arr((f.r/f.a).v, 3), 3);
      
      cout <<"\ni=" <<i <<"\nv_i=" <<v[i] <<"\nf.(w, v)=" <<vi <<endl;
      cout <<"\na_i=" <<a[i] <<"\nf.(b, a)=" <<ai <<endl;
      CHECK(maxDiff(vi, v[i])<1e-4, "");
    }
#endif
  }
  
  
  for(i=N; i--;) {
    if(robot.dof(i)) {
      tau_i(i) = ~S(i) * f[i];
    }
    if(robot.parent(i) != -1) {
      f[robot.parent(i)] = f[robot.parent(i)] + ~Xup[i]*f[i];
    }
  }
}


//===========================================================================

void Featherstone::fwdDynamics_old(arr& qdd,
                                   const Robot& robot,
                                   const arr& qd,
                                   const arr& tau,
                                   const arr& grav) {
  /*
  % FDab  Forward Dynamics via Articulated-Body Algorithm
  % FDab(model, q, qd, tau, f_ext, grav_accn) calculates the forward dynamics of a
  % kinematic tree via the articulated-body algorithm.  q, qd and tau are
  % vectors of joint position, velocity and force variables; and the return
  % value is a vector of joint acceleration variables.  f_ext is a cell array
  % specifying external forces acting on the bodies.  If f_ext == {} then
  % there are no external forces; otherwise, f_ext{i} is a spatial force
  % vector giving the force acting on body i, expressed in body i
  % coordinates.  Empty cells in f_ext are interpreted as zero forces.
  % grav_accn is a 3D vector expressing the linear acceleration due to
  % gravity.  The arguments f_ext and grav_accn are optional, and default to
  % the values {} and [0, 0, -9.81], respectively, if omitted.
  */
  
  //CHANGE: default is gravity zero (assume to be included in external forces)
  arr a_grav(6);
  a_grav.setZero();
  if(grav.N) {
    a_grav.setVectorBlock(grav, 3);
  }
  
  int par;
  uint i, N=robot.N(), d_i, n;
  mlr::Array<arr> h(N), qd_i(N), qdd_i(N), tau_i(N), I_h(N), h_I_h(N), inv_h_I_h(N), tau__h_fA(N);
  arr Xup(N, 6, 6), v(N, 6), dh_dq(N, 6), f(N, 6), IA(N, 6, 6), fA(N, 6), a(N, 6);
  arr vJ, Ia, fa;
  arr Q;
  
  for(i=0, n=0; i<N; i++) {
    //for general multi-dimensional joints, pick the sub-arrays
    d_i=robot.dof(i);
    if(d_i) {
      qd_i(i) .referToRange(qd , n, n+d_i-1);
      qdd_i(i).referToRange(qdd, n, n+d_i-1);
      tau_i(i).referToRange(tau, n, n+d_i-1);
    } else {
      qd_i(i) .clear(); qd_i(i). resize(0);
      qdd_i(i).clear(); qdd_i(i).resize(0);
      tau_i(i).clear(); tau_i(i).resize(0);
    }
    n += d_i;
    
    h(i) = robot.S(i);
    vJ = h(i) * qd_i(i); //equation (2), vJ = relative vel across joint i
    if(robot.C->nodes(i)->inLinks.N) {
      FrameToMatrix(Q, robot.C->nodes(i)->inLinks(0)->X);
      Xup[i] = Q * robot.Xlink(i); //the transformation from the i-th to the j-th
    } else {
      Xup[i] = robot.Xlink(i); //the transformation from the i-th to the j-th
    }
    if(robot.parent(i) == -1) {
      v[i] = vJ;
      dh_dq[i] = 0.;
    } else {
      v[i] = Xup[i] * v[robot.parent(i)] + vJ;
      dh_dq[i] = crossM(v[i]) * vJ;  //WHY??
    }
    // v[i] = total velocity, but in joint coordinates
    IA[i] = robot.Ilink(i);
    fA[i] = crossF(v[i]) * robot.Ilink(i) * v[i] - robot.force(i); //1st equation below (13)
  }
  
  for(i=N; i--;) {
    I_h(i) = IA[i] * h(i);
    if(robot.dof(i)) {
      h_I_h(i)      = ~h(i)*I_h(i);
      tau__h_fA(i) = tau_i(i) - ~h(i)*fA[i]; //[change from above] last term in (13), 2nd equation below (13)
    } else {
      h_I_h(i).clear();
      tau__h_fA(i).clear();
    }
    inverse(inv_h_I_h(i), h_I_h(i));
    par = robot.parent(i);
    if(par != -1) {
      Ia = IA[i] - I_h(i)*inv_h_I_h(i)*~I_h(i);
      fa = fA[i] + Ia*dh_dq[i] + I_h(i)*inv_h_I_h(i)*tau__h_fA(i);
      IA[par] = IA[par] + ~Xup[i] * Ia * Xup[i];         //equation (12)
      fA[par] = fA[par] + ~Xup[i] * fa;                  //equation (13)
    }
  }
  
  for(i=0; i<N; i++) {
    par=robot.parent(i);
    if(par == -1) {
      a[i] = Xup[i] * a_grav + dh_dq[i]; //[change from above]
    } else {
      a[i] = Xup[i] * a[par] + dh_dq[i]; //[change from above]
    }
    if(robot.dof(i)) {
      qdd_i(i) = inverse(h_I_h(i))*(tau__h_fA(i) - ~I_h(i)*a[i]); //equation (14)
    }
    a[i] = a[i] + h(i)*qdd_i(i); //equation above (14)
  }
}
#endif

//===========================================================================

/* Articulated Body Dynamics - exactly as in my `simulationSoftware notes',
   following the notation of Featherstone's recent short survey paper */
void mlr::fwdDynamics_aba_nD(arr& qdd,
                             const mlr::LinkTree& tree,
                             const arr& qd,
                             const arr& tau) {
  int par;
  uint i, N=tree.N, d_i, n;
  mlr::Array<arr> h(N), qd_i(N), qdd_i(N), tau_i(N), I_h(N), h_I_h(N), u(N);
  arr Xup(N, 6, 6), v(N, 6), dh_dq(N, 6), IA(N, 6, 6), fA(N, 6), a(N, 6);
  qdd.resizeAs(tau);
  
  for(i=0, n=0; i<N; i++) {
    d_i=tree(i).dof();
    if(d_i) {
      qd_i(i) .referToRange(qd , n, n+d_i-1);
      qdd_i(i).referToRange(qdd, n, n+d_i-1);
      tau_i(i).referToRange(tau, n, n+d_i-1);
    } else {
      qd_i(i) .clear(); qd_i(i). resize(0);
      qdd_i(i).clear(); qdd_i(i).resize(0);
      tau_i(i).clear(); tau_i(i).resize(0);
    }
    n += d_i;
    h(i) = tree(i)._h;
    h(i).reshape(6, d_i);
    Xup[i] = tree(i)._Q * tree(i)._A; //the transformation from the i-th to the j-th
  }
  CHECK(n==qd.N && n==qdd.N && n==tau.N, "")
  
  for(i=0; i<N; i++) {
    par = tree(i).parent;
    if(par == -1) {
      v[i] = h(i) * qd_i(i);
      dh_dq[i] = 0.;
    } else {
      v[i] = Xup[i] * v[par] + h(i) * qd_i(i);
      dh_dq[i] = Featherstone::crossM(v[i]) * h(i) * qd_i(i);
    }
    IA[i] = tree(i)._I;
    fA[i] = Featherstone::crossF(v[i]) * tree(i)._I * v[i] - tree(i)._f;
  }
  
  for(i=N; i--;) {
    par = tree(i).parent;
    I_h(i) = IA[i] * h(i);
    if(tree(i).dof()) {
      h_I_h(i) = ~h(i)*I_h(i);
      u(i) = tau_i(i) - ~I_h(i)*dh_dq[i] - ~h(i)*fA[i];
    } else {
      h_I_h(i).clear(); h_I_h(i).resize(0);
      u(i).clear(); u(i).resize(0);
    }
    if(par != -1) {
      IA[par]() += ~Xup[i] * (IA[i] - I_h(i)*inverse(h_I_h(i))*~I_h(i)) * Xup[i];
      fA[par]() += ~Xup[i] * (fA[i] + IA[i]*dh_dq[i] + I_h(i)*inverse(h_I_h(i))*u(i));
    }
  }
  
  for(i=0; i<N; i++) {
    par=tree(i).parent;
    if(par == -1) {
      a[i] = 0; //Xup[i] * grav_accn;
    } else {
      a[i]() = Xup[i] * a[par];
    }
    if(tree(i).dof()) {
      qdd_i(i) = inverse(h_I_h(i))*(u(i) - ~I_h(i)*a[i]);
    }
    a[i] = a[i] + dh_dq[i] + h(i)*qdd_i(i);
  }
}

//===========================================================================

void mlr::fwdDynamics_aba_1D(arr& qdd,
                             const mlr::LinkTree& tree,
                             const arr& qd,
                             const arr& tau) {
  int par;
  uint i, N=tree.N, iq;
  arr h(N, 6), I_h(N, 6), h_I_h(N), inv_h_I_h(N), taui(N), tau__h_fA(N);
  arr Xup(N, 6, 6), v(N, 6), dh_dq(N, 6), IA(N, 6, 6), fA(N, 6), a(N, 6);
  arr vJ, Ia, fa;
  qdd.resizeAs(tau);
  
  //fwd: compute the velocities v[i] and external + Coriolis forces fA[i] of all bodies
  for(i=0; i<N; i++) {
    iq = tree(i).qIndex;
    par = tree(i).parent;
    Xup[i]() = tree(i)._Q * tree(i)._A; //the transformation from the i-th to the j-th
    if(par != -1) {
      h[i]() = tree(i)._h;
      vJ = h[i] * qd(iq); //equation (2), vJ = relative vel across joint i
      v[i]() = Xup[i] * v[par] + vJ; //eq (27)
      dh_dq[i]() = Featherstone::crossM(v[i]) * vJ;  //WHY??
      taui(i)=tau(iq);
    } else {
      h[i]() = 0.;
      v[i]() = 0.;
      dh_dq[i] = 0.;
      taui(i)=0.;
    }
    // v[i] = total velocity, but in joint coordinates
    IA[i] = tree(i)._I;
    //first part of eq (29)
    fA[i] = Featherstone::crossF(v[i]) * (tree(i)._I * v[i]) - tree(i)._f;
  }
  
  //bwd: propagate tree inertia
  for(i=N; i--;) {
    par = tree(i).parent;
    //eq (28)
    I_h[i]()      = IA[i] * h[i];
    h_I_h(i)      = scalarProduct(h[i], I_h[i]);
    inv_h_I_h(i)  = 1./h_I_h(i);
    tau__h_fA(i) = taui(i) - scalarProduct(h[i], fA[i]); //[change from above] last term in (13), 2nd equation below (13)
    if(par != -1) {
      Ia = IA[i] - I_h[i]*(inv_h_I_h(i)*~I_h[i]);
      fa = fA[i] + Ia*dh_dq[i] + I_h[i]*(inv_h_I_h(i)*tau__h_fA(i));
      IA[par] = IA[par] + ~Xup[i] * Ia * Xup[i];         //equation (12)
      fA[par] = fA[par] + ~Xup[i] * fa;                  //equation (13)
    }
  }
  
  for(i=0; i<N; i++) {
    iq = tree(i).qIndex;
    par= tree(i).parent;
    if(par != -1) {
      a[i] = Xup[i] * a[par] + dh_dq[i]; //[change from above]
      qdd(iq) = inv_h_I_h(i)*(tau__h_fA(i) - scalarProduct(I_h[i], a[i])); //equation (14)
      a[i] = a[i] + h[i]*qdd(iq); //equation above (14)
    } else {
      a[i] = dh_dq[i]; //[change from above]
    }
  }
}

//===========================================================================

void mlr::invDynamics(arr& tau,
                      const mlr::LinkTree& tree,
                      const arr& qd,
                      const arr& qdd) {
  int par;
  uint i, N=tree.N, d_i, n;
  mlr::Array<arr> h(N), qd_i(N), qdd_i(N), tau_i(N);
  arr Xup(N, 6, 6), v(N, 6), fJ(N, 6), a(N, 6);
  tau.resizeAs(qdd);
  
  for(i=0, n=0; i<N; i++) {
    d_i=tree(i).dof();
    if(d_i) {
      qd_i(i) .referToRange(qd , n, n+d_i-1);
      qdd_i(i).referToRange(qdd, n, n+d_i-1);
      tau_i(i).referToRange(tau, n, n+d_i-1);
    } else {
      qd_i(i) .clear(); qd_i(i). resize(0);
      qdd_i(i).clear(); qdd_i(i).resize(0);
      tau_i(i).clear(); tau_i(i).resize(0);
    }
    n += d_i;
    h(i) = tree(i)._h;
    h(i).reshape(6, d_i);
    Xup[i] = tree(i)._Q * tree(i)._A; //the transformation from the i-th to the j-th
  }
  CHECK(n==qd.N && n==qdd.N && n==tau.N, "")
  
  for(i=0; i<N; i++) {
    par = tree(i).parent;
    if(par == -1) {
      v[i] = h(i) * qd_i(i);
      a[i] = h(i) * qdd_i(i);
    } else {
      v[i] = Xup[i] * v[par] + h(i) * qd_i(i);
      a[i] = Xup[i] * a[par] + h(i) * qdd_i(i) + Featherstone::crossM(v[i]) * h(i) * qd_i(i);
    }
    //see featherstone-orin paper for definition of fJ (different to fA; it's about force equilibrium at a joint)
    fJ[i] = tree(i)._I*a[i] + Featherstone::crossF(v[i]) * tree(i)._I * v[i] - tree(i)._f;
  }
  
  for(i=N; i--;) {
    par = tree(i).parent;
    if(tree(i).dof()) tau_i(i) = ~h(i) * fJ[i];
    if(par != -1)     fJ[par]() += ~Xup[i] * fJ[i];
  }
}


//===========================================================================

void mlr::equationOfMotion(arr& H, arr& C,
                           const mlr::LinkTree& tree,
                           const arr& qd) {
                           
  /*function  [H, C] = HandC( model, q, qd, f_ext, grav_accn )
  
  % HandC  Calculate coefficients of equation of motion.
  % [H, C]=HandC(model, q, qd, f_ext, grav_accn) calculates the coefficients of
  % the joint-space equation of motion, tau=H(q)qdd+C(d, qd, f_ext), where q,
  % qd and qdd are the joint position, velocity and acceleration vectors, H
  % is the joint-space inertia matrix, C is the vector of gravity,
  % external-force and velocity-product terms, and tau is the joint force
  % vector.  Algorithm: recursive Newton-Euler for C, and
  % Composite-Rigid-Body for H.  f_ext is a cell array specifying external
  % forces acting on the bodies.  If f_ext == {} then there are no external
  % forces; otherwise, f_ext{i} is a spatial force vector giving the force
  % acting on body i, expressed in body i coordinates.  Empty cells in f_ext
  % are interpreted as zero forces.  grav_accn is a 3D vector expressing the
  % linear acceleration due to gravity.  The arguments f_ext and grav_accn
  % are optional, and default to the values {} and [0, 0, -9.81], respectively,
  % if omitted.
  */
  
  int par;
  int iq, jq;
  uint i, j, N=tree.N;
  //CHECK_EQ(N-1,qd.N,"vels don't have right dimension")
  arr h(N, 6);
  arr Xup(N, 6, 6), v(N, 6), dh_dq(N, 6), IC(N, 6, 6), fvp(N, 6), avp(N, 6);
  arr vJ, fh;
  
  avp.setZero();
  
  for(i=0; i<N; i++) {
    iq  = tree(i).qIndex;
    par = tree(i).parent;
    Xup[i]() = tree(i)._Q * tree(i)._A; //the transformation from the i-th to the j-th
    if(par!=-1) {
      h[i]() = tree(i)._h;
      if(iq!=-1) {//is not a fixed joint
        vJ = h[i] * qd(iq); //equation (2), vJ = relative vel across joint i
      } else{
        vJ = zeros(6);
      }
      v[i]() = Xup[i] * v[par] + vJ;
      dh_dq[i]() = Featherstone::crossM(v[i]) * vJ;  //WHY??
      avp[i]() = Xup[i]*avp[par] + Featherstone::crossM(v[i])*vJ;
    } else {
      h[i]() = 0.;
      v[i]() = 0.;
      dh_dq[i] = 0.;
      avp[i]() = 0.;
    }
    // v[i] = total velocity, but in joint coordinates
    IC[i]() = tree(i)._I;
    fvp[i] = tree(i)._I*avp[i] + Featherstone::crossF(v[i])*(tree(i)._I*v[i]) - tree(i)._f;
  }
  
  C = zeros(qd.N);
  
  for(i=N; i--;) {
    iq  = tree(i).qIndex;
    par = tree(i).parent;
    if(iq!=-1) {
      C(iq) += scalarProduct(h[i], fvp[i]);
    }
    if(par!=-1) {
      fvp[par]() += ~Xup[i] * fvp[i];
      IC[par]() += ~Xup[i] * IC[i] * Xup[i];
    }
  }
  
  H = zeros(qd.N, qd.N);
  
  for(i=0; i<N; i++) {
    iq = tree(i).qIndex;
    fh = IC[i] * h[i];
    if((int)iq!=-1) {
      H(iq, iq) += scalarProduct(h[i], fh);
    }
    j = i;
    while(tree(j).parent!=-1) {
      fh = ~Xup[j] * fh;
      j  = tree(j).parent;
      jq = tree(j).qIndex;
      if(jq!=-1 && iq!=-1) {
        double Hij = scalarProduct(h[j], fh);
        H(iq, jq) += Hij;
        H(jq, iq) += Hij;
      }
    }
  }
  
  //add friction for non-filled joints
  boolA filled(qd.N);
  filled=false;
  for(i=0;i<N;i++){ iq = tree(i).qIndex; if(iq!=-1) filled(iq)=true; }
  for(i=0;i<qd.N;i++) if(!filled(i)){
    H(i,i) = 1.;
//    C(i) = -100.*qd(i);
  }
}

//===========================================================================

void mlr::fwdDynamics_MF(arr& qdd,
                         const mlr::LinkTree& tree,
                         const arr& qd,
                         const arr& u) {
                         
  arr M, Minv, F;
  equationOfMotion(M, F, tree, qd);
  inverse(Minv, M);
//  inverse_SymPosDef(Minv, M);
  
  qdd = Minv * (u - F);
}

// #else ///MLR_FEATHERSTONE
// void GraphToTree(mlr::LinkTree& tree, const mlr::KinematicWorld& C) { NIY; }
// void updateGraphToTree(mlr::LinkTree& tree, const mlr::KinematicWorld& C) { NIY; }
// void Featherstone::equationOfMotion(arr& H, arr& C,
//                                     const mlr::LinkTree& tree,
//                                     const arr& qd) { NIY; }
// void Featherstone::fwdDynamics_MF(arr& qdd,
//                                   const mlr::LinkTree& tree,
//                                   const arr& qd,
//                                   const arr& tau) { NIY; }
// void Featherstone::invDynamics(arr& tau,
//                                const mlr::LinkTree& tree,
//                                const arr& qd,
//                                const arr& qdd) { NIY; }
// #endif
/** @} */


//===========================================================================
//===========================================================================
// template instantiations
//===========================================================================
//===========================================================================

#include <Core/util.tpp>

#ifndef  MLR_ORS_ONLY_BASICS
template mlr::Array<mlr::Shape*>::Array(uint);
template mlr::Shape* listFindByName(const mlr::Array<mlr::Shape*>&,const char*);

#include <Core/array.tpp>
template mlr::Array<mlr::Joint*>::Array();
#endif
/** @} */
/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#include <climits>
#include "taskMap_qItself.h"

TaskMap_qItself::TaskMap_qItself(bool relative_q0) : moduloTwoPi(true), relative_q0(relative_q0) {}

//TaskMap_qItself::TaskMap_qItself(uint singleQ, uint qN) : moduloTwoPi(true), relative_q0(false) { M=zeros(1,qN); M(0,singleQ)=1.; }

//TaskMap_qItself::TaskMap_qItself(const mlr::KinematicWorld& G, mlr::Joint* j)
//  : moduloTwoPi(true), relative_q0(false)  {
//  M = zeros(j->qDim(), G.getJointStateDimension() );
//  M.setMatrixBlock(eye(j->qDim()), 0, j->qIndex);
//}

//TaskMap_qItself::TaskMap_qItself(const mlr::KinematicWorld& G, const char* jointName)
//  : moduloTwoPi(true), relative_q0(false)  {
//  mlr::Joint *j = G.getJointByName(jointName);
//  if(!j) return;
//  M = zeros(j->qDim(), G.getJointStateDimension() );
//  M.setMatrixBlock(eye(j->qDim()), 0, j->qIndex);
//}

//TaskMap_qItself::TaskMap_qItself(const mlr::KinematicWorld& G, const char* jointName1, const char* jointName2)
//  : moduloTwoPi(true), relative_q0(false)  {
//  mlr::Joint *j1 = G.getJointByName(jointName1);
//  mlr::Joint *j2 = G.getJointByName(jointName2);
//  M = zeros(j1->qDim() + j2->qDim(), G.getJointStateDimension() );
//  M.setMatrixBlock(eye(j1->qDim()), 0, j1->qIndex);
//  M.setMatrixBlock(eye(j2->qDim()), j1->qDim(), j2->qIndex);
//}

TaskMap_qItself::TaskMap_qItself(TaskMap_qItself_PickMode pickMode, const StringA& picks, const mlr::KinematicWorld& K, bool relative_q0)
  : moduloTwoPi(true), relative_q0(relative_q0) {
  if(pickMode==QIP_byJointGroups){
    for(mlr::Joint *j:K.joints){
      bool pick=false;
      for(const mlr::String& s:picks) if(j->ats.getNode(s)){ pick=true; break; }
      if(pick) selectedBodies.append(j->to->index);
    }
    return;
  }
  if(pickMode==QIP_byJointNames){
    for(mlr::Joint *j:K.joints){
      bool pick=false;
      for(const mlr::String& s:picks) if(j->name==s){ pick=true; break; }
      if(pick) selectedBodies.append(j->to->index);
    }
    return;
  }
  NIY
}

TaskMap_qItself::TaskMap_qItself(uintA _selectedBodies, bool relative_q0)
  : selectedBodies(_selectedBodies), moduloTwoPi(true), relative_q0(relative_q0){
}

void TaskMap_qItself::phi(arr& q, arr& J, const mlr::KinematicWorld& G, int t) {
  if(!selectedBodies.N){
    G.getJointState(q);
    if(relative_q0){
      for(mlr::Joint* j:G.joints) if(j->q0 && j->qDim()==1) q(j->qIndex) -= j->q0;
    }
//    if(M.N){
//      if(M.nd==1){
//        q=M%q; if(&J) J.setDiag(M); //this fails if the dimensionalities of q are non-stationary!
//      }else{
//        q=M*q; if(&J) J=M;
//      }
//    }else{
      if(&J) J.setId(q.N);
//    }
  }else{
    uint n=dim_phi(G);
    q.resize(n);
    G.getJointState();
    if(&J) J.resize(n, G.q.N).setZero();
    uint m=0;
    uint qIndex=0;
    for(uint b:selectedBodies){
      mlr::Joint *j = G.bodies.elem(b)->inLinks.scalar();
//      CHECK_GE(j->qIndex, qIndex, "selectedBodies does not add joints in sorted order! I'm not sure this is correct!");
      qIndex = j->qIndex;
      for(uint k=0;k<j->qDim();k++){
        q(m) = G.q.elem(qIndex+k);
        if(relative_q0) q(m) -= j->q0;
        if(&J) J(m,qIndex+k) = 1.;
        m++;
      }
    }
    CHECK_EQ(n,m,"");
  }
}

void TaskMap_qItself::phi(arr& y, arr& J, const WorldL& G, double tau, int t){
  CHECK(G.N>=order+1,"I need at least " <<order+1 <<" configurations to evaluate");
  uint k=order;
  if(k==0) return TaskMap::phi(y, J, G, tau, t);

  double tau2=tau*tau, tau3=tau2*tau;
  arrA q_bar(k+1), J_bar(k+1);
  //-- read out the task variable from the k+1 configurations
  uint offset = G.N-1-k; //G.N might contain more configurations than the order of THIS particular task -> the front ones are not used
  for(uint i=0;i<=k;i++){
    phi(q_bar(i), J_bar(i), *G(offset+i), t-k+i);
//    q_bar(i) = G(offset+i)->q;
//    J_bar(i).setId(q_bar(i).N);
  }
  bool handleSwitches=false;
  uint qN=q_bar(0).N;
  for(uint i=0;i<=k;i++) if(q_bar(i).N!=qN){ handleSwitches=true; break; }
  if(handleSwitches){
    CHECK(!selectedBodies.N,"doesn't work for this...")
    uint nJoints = G(offset)->joints.N;
    JointL jointMatchLists(k+1, nJoints); //for each joint of [0], find if the others have it
    jointMatchLists.setZero();
    boolA useIt(nJoints);
    useIt = true;
    for(uint j_idx=0; j_idx<nJoints; j_idx++){
      mlr::Joint *j=G(offset)->joints(j_idx);
      for(uint i=0;i<=k;i++){
        mlr::Joint *jmatch = G(offset+i)->getJointByBodyNames(j->from->name, j->to->name);
        if(jmatch && j->type!=jmatch->type) jmatch=NULL;
        if(!jmatch){ useIt(j_idx) = false; break; }
        jointMatchLists(i, j_idx) = jmatch;
      }
    }

    arrA q_bar_mapped(k+1), J_bar_mapped(k+1);
    uint qidx, qdim;
    for(uint j_idx=0; j_idx<nJoints; j_idx++){
      if(useIt(j_idx)){
        for(uint i=0;i<=k;i++){
          qidx=jointMatchLists(i,j_idx)->qIndex;
          qdim=jointMatchLists(i,j_idx)->qDim();
          if(qdim){
            q_bar_mapped(i).append(q_bar(i)({qidx, qidx+qdim-1}));
            J_bar_mapped(i).append(J_bar(i)({qidx, qidx+qdim-1}));
          }
        }
      }
    }

    q_bar = q_bar_mapped;
    J_bar = J_bar_mapped;
  }

  if(k==1)  y = (q_bar(1)-q_bar(0))/tau; //penalize velocity
  if(k==2)  y = (q_bar(2)-2.*q_bar(1)+q_bar(0))/tau2; //penalize acceleration
  if(k==3)  y = (q_bar(3)-3.*q_bar(2)+3.*q_bar(1)-q_bar(0))/tau3; //penalize jerk
  if(&J) {
#if 1
    uintA qidx(G.N);
    qidx(0)=0;
    for(uint i=1;i<G.N;i++) qidx(i) = qidx(i-1)+G(i-1)->q.N;
    J = zeros(y.N, qidx.last()+G.last()->q.N);
    if(k==1){ J.setMatrixBlock(J_bar(1), 0, qidx(offset+1));  J.setMatrixBlock(   -J_bar(0), 0, qidx(offset+0));  J/=tau; }
    if(k==2){ J.setMatrixBlock(J_bar(2), 0, qidx(offset+2));  J.setMatrixBlock(-2.*J_bar(1), 0, qidx(offset+1));  J.setMatrixBlock(J_bar(0)   , 0, qidx(offset+0));  J/=tau2; }
    if(k==3){ J.setMatrixBlock(J_bar(3), 0, qidx(offset+3));  J.setMatrixBlock(-3.*J_bar(2), 0, qidx(offset+2));  J.setMatrixBlock(3.*J_bar(1), 0, qidx(offset+1));  J.setMatrixBlock(-J_bar(0), 0, qidx(offset+0));  J/=tau3; }
#else
    J = zeros(G.N, y.N, J_bar(0).d1);
    if(k==1){ J[offset+1]() =  J_bar(1);  J[offset+0]() =    -J_bar(0);  J/=tau; }
    if(k==2){ J[offset+2]() =  J_bar(2);  J[offset+1]() = -2.*J_bar(1);  J[offset+0]() = J_bar(0);  J/=tau2; }
    if(k==3){ J[offset+3]() =  J_bar(3);  J[offset+2]() = -3.*J_bar(2);  J[offset+1]() = 3.*J_bar(1);  J[offset+0]() = -J_bar(0);  J/=tau3; }
    arr tmp(J);
    tensorPermutation(J, tmp, TUP(1u,0u,2u));
    J.reshape(y.N, G.N*J_bar(0).d1);
#endif
  }
}


uint TaskMap_qItself::dim_phi(const mlr::KinematicWorld& G) {
  if(selectedBodies.N){
    uint n=0;
    for(uint b:selectedBodies) n+=G.bodies.elem(b)->inLinks.scalar()->qDim();
    return n;
  }
//  if(M.nd==2) return M.d0;
  return G.getJointStateDimension();
}

uint TaskMap_qItself::dim_phi(const WorldL& G, int t){
  if(t<0) return dim_phi(*G.last());

  while(dimPhi.N<=(uint)t) dimPhi.append(UINT_MAX);

  //empirically test the dimension:
  if(dimPhi(t)==UINT_MAX){
    arr y;
    phi(y, NoArr, G, 0.01, t);
    dimPhi(t) = y.N;
  }

  return dimPhi(t);
}

mlr::String TaskMap_qItself::shortTag(const mlr::KinematicWorld& G){
  mlr::String s="qItself";
  if(selectedBodies.N){
    if(selectedBodies.N<=3){
      for(uint b:selectedBodies) s <<':' <<G.bodies(b)->name;
    }else{
      s <<'#' <<selectedBodies.N;
    }
  }else{
    s <<":ALL";
  }
  return s;
}

//===========================================================================

void TaskMap_qZeroVels::phi(arr& y, arr& J, const WorldL& G, double tau, int t){
  CHECK(order==1,"NIY");
  CHECK(G.N>=order+1,"I need at least " <<order+1 <<" configurations to evaluate");
  uint k=order;

  double tau2=tau*tau, tau3=tau2*tau;
  arrA q_bar(k+1), J_bar(k+1);
  //-- read out the task variable from the k+1 configurations
  uint offset = G.N-1-k; //G.N might contain more configurations than the order of THIS particular task -> the front ones are not used

  for(mlr::Joint *j:G.last()->joints) if(j->constrainToZeroVel){
    mlr::Joint *jmatch = G.last(-2)->getJointByBodyIndices(j->from->index, j->to->index);
    if(jmatch && j->type!=jmatch->type) jmatch=NULL;
    if(jmatch){
      for(uint i=0;i<j->qDim();i++){
        q_bar(0).append(G.last(-2)->q(jmatch->qIndex+i));
        q_bar(1).append(G.last(-1)->q(j     ->qIndex+i));
        J_bar(0).append(eyeVec(G.last(-2)->q.N, jmatch->qIndex+i));
        J_bar(1).append(eyeVec(G.last(-1)->q.N, j     ->qIndex+i));
      }
    }
  }
  if(!q_bar(0).N){ y.clear(); if(&J) J.clear(); return; }
  J_bar(0).reshape(q_bar(0).N, J_bar(0).N/q_bar(0).N);
  J_bar(1).reshape(q_bar(1).N, J_bar(1).N/q_bar(1).N);

  if(k==1)  y = (q_bar(1)-q_bar(0))/tau; //penalize velocity
  if(k==2)  y = (q_bar(2)-2.*q_bar(1)+q_bar(0))/tau2; //penalize acceleration
  if(k==3)  y = (q_bar(3)-3.*q_bar(2)+3.*q_bar(1)-q_bar(0))/tau3; //penalize jerk
  if(&J) {
    uintA qidx(G.N);
    qidx(0)=0;
    for(uint i=1;i<G.N;i++) qidx(i) = qidx(i-1)+G(i-1)->q.N;
    J = zeros(y.N, qidx.last()+G.last()->q.N);
    if(k==1){ J.setMatrixBlock(J_bar(1), 0, qidx(offset+1));  J.setMatrixBlock(   -J_bar(0), 0, qidx(offset+0));  J/=tau; }
    if(k==2){ J.setMatrixBlock(J_bar(2), 0, qidx(offset+2));  J.setMatrixBlock(-2.*J_bar(1), 0, qidx(offset+1));  J.setMatrixBlock(J_bar(0)   , 0, qidx(offset+0));  J/=tau2; }
    if(k==3){ J.setMatrixBlock(J_bar(3), 0, qidx(offset+3));  J.setMatrixBlock(-3.*J_bar(2), 0, qidx(offset+2));  J.setMatrixBlock(3.*J_bar(1), 0, qidx(offset+1));  J.setMatrixBlock(-J_bar(0), 0, qidx(offset+0));  J/=tau3; }
  }
}

uint TaskMap_qZeroVels::dim_phi(const WorldL& G, int t){
  CHECK(t>=0,"");

  while(dimPhi.N<=(uint)t) dimPhi.append(UINT_MAX);

  //empirically test the dimension:
  if(dimPhi(t)==UINT_MAX){
    arr y;
    phi(y, NoArr, G, 0.01, t);
    dimPhi(t) = y.N;
  }

  return dimPhi(t);
}

//===========================================================================

mlr::Array<mlr::Joint*> getMatchingJoints(const WorldL& G, bool zeroVelJointsOnly){
  mlr::Array<mlr::Joint*> matchingJoints;
  mlr::Array<mlr::Joint*> matches(G.N);
  bool matchIsGood;

  for(mlr::Joint *j:G.last()->joints) if(!zeroVelJointsOnly || j->constrainToZeroVel){
    matches.setZero();
    matches.last() = j;
    matchIsGood=true;

    for(uint k=0;k<G.N-1;k++){ //go through other configs
      mlr::Joint *jmatch = G(k)->getJointByBodyIndices(j->from->index, j->to->index);
      if(!jmatch || j->type!=jmatch->type || j->constrainToZeroVel!=jmatch->constrainToZeroVel){
        matchIsGood=false;
        break;
      }
      matches(k) = jmatch;
    }

    if(matchIsGood) matchingJoints.append(matches);
  }
  matchingJoints.reshape(matchingJoints.N/G.N, G.N);
  return matchingJoints;
}

//===========================================================================

mlr::Array<mlr::Joint*> getSwitchedJoints(const mlr::KinematicWorld& G0, const mlr::KinematicWorld& G1, int verbose){

  HALT("retired: we only look at switched objects");

  mlr::Array<mlr::Joint*> switchedJoints;

  for(mlr::Joint *j1:G1.joints) {
    if(j1->from->index>=G0.bodies.N || j1->to->index>=G0.bodies.N){
      switchedJoints.append({NULL,j1});
      continue;
    }
    mlr::Joint *j0 = G0.getJointByBodyIndices(j1->from->index, j1->to->index);
    if(!j0 || j0->type!=j1->type){
      if(G0.bodies(j1->to->index)->inLinks.N==1){ //out-body had (in G0) one inlink...
        j0 = G0.bodies(j1->to->index)->inLinks.scalar();
      }
      switchedJoints.append({j0,j1});
//      }
    }
  }
  switchedJoints.reshape(switchedJoints.N/2, 2);

  if(verbose){
    for(uint i=0;i<switchedJoints.d0;i++){
      cout <<"Switch: "
          <<switchedJoints(i,0)->from->name <<'-' <<switchedJoints(i,0)->to->name
         <<" -> " <<switchedJoints(i,1)->from->name <<'-' <<switchedJoints(i,1)->to->name <<endl;
    }
  }

  return switchedJoints;
}

//===========================================================================

mlr::Array<mlr::Body*> getSwitchedBodies(const mlr::KinematicWorld& G0, const mlr::KinematicWorld& G1, int verbose){
  mlr::Array<mlr::Body*> switchedBodies;

  for(mlr::Body *b1:G1.bodies) {
    if(b1->index>=G0.bodies.N) continue; //b1 does not exist in G0 -> not a switched body
    mlr::Body *b0 = G0.bodies(b1->index);
    if(b0->inLinks.N != b1->inLinks.N){ switchedBodies.append({b0,b1}); continue; }
    if(b0->inLinks.N){
      CHECK(b0->inLinks.N==1,"not a tree!?");
      mlr::Joint *j0 = b0->inLinks.scalar();
      mlr::Joint *j1 = b1->inLinks.scalar();
      if(j0->type!=j1->type || j0->from->index!=j1->from->index){ //different joint type; or attached to different parent
        switchedBodies.append({b0,b1});
      }
    }
  }
  switchedBodies.reshape(switchedBodies.N/2, 2);

  if(verbose){
    for(uint i=0;i<switchedBodies.d0;i++){
      cout <<"Switch: "
          <<switchedBodies(i,0)->name /*<<'-' <<switchedBodies(i,0)->name*/
         <<" -> " <<switchedBodies(i,1)->name /*<<'-' <<switchedJoints(i,1)->to->name*/ <<endl;
    }
  }

  return switchedBodies;
}
/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#include "taskMap.h"
#include "taskMap_qItself.h"
#include "taskMap_GJK.h"
#include "taskMap_FixSwitchedObjects.h"

//===========================================================================

void TaskMap::phi(arr& y, arr& J, const WorldL& G, double tau, int t){
  CHECK(G.N>=order+1,"I need at least " <<order+1 <<" configurations to evaluate");
  uint k=order;
  if(k==0){// basic case: order=0
    arr J_bar;
    phi(y, (&J?J_bar:NoArr), *G.last(), t);
    if(&J){
      uint qidx=0;
      for(uint i=0;i<G.N;i++) qidx+=G(i)->q.N;
      J.resize(y.N, qidx).setZero();
      J.setMatrixBlock(J_bar, 0, qidx-J_bar.d1);
    }
    return;
  }
  arrA y_bar, J_bar;
  double tau2=tau*tau, tau3=tau2*tau;
  y_bar.resize(k+1);
  J_bar.resize(k+1);
  //-- read out the task variable from the k+1 configurations
  uint offset = G.N-1-k; //G.N might contain more configurations than the order of THIS particular task -> the front ones are not used
  for(uint i=0;i<=k;i++)
    phi(y_bar(i), (&J?J_bar(i):NoArr), *G(offset+i), t-k+i);

  // check for quaternion flipping
  if(k==1 && flipTargetSignOnNegScalarProduct && scalarProduct(y_bar(1), y_bar(0))<-.0){
      if(&J) J_bar(0) *= -1.;
      y_bar(0) *= -1.;
  }
  // NIY
  if(k==2 && flipTargetSignOnNegScalarProduct) HALT("Quaternion flipping NIY for acceleration");
  if(k==3 && flipTargetSignOnNegScalarProduct) HALT("Quaternion flipping NIY for jerk");

  if(k==1)  y = (y_bar(1)-y_bar(0))/tau; //penalize velocity
  if(k==2)  y = (y_bar(2)-2.*y_bar(1)+y_bar(0))/tau2; //penalize acceleration
  if(k==3)  y = (y_bar(3)-3.*y_bar(2)+3.*y_bar(1)-y_bar(0))/tau3; //penalize jerk
  if(&J) {
#if 1
    uintA qidx(G.N);
    qidx(0)=0;
    for(uint i=1;i<G.N;i++) qidx(i) = qidx(i-1)+G(i-1)->q.N;
    J = zeros(y.N, qidx.last()+G.last()->q.N);
    if(k==1){ J.setMatrixBlock(J_bar(1), 0, qidx(offset+1));  J.setMatrixBlock(   -J_bar(0), 0, qidx(offset+0));  J/=tau; }
    if(k==2){ J.setMatrixBlock(J_bar(2), 0, qidx(offset+2));  J.setMatrixBlock(-2.*J_bar(1), 0, qidx(offset+1));  J.setMatrixBlock(J_bar(0)   , 0, qidx(offset+0));  J/=tau2; }
    if(k==3){ J.setMatrixBlock(J_bar(3), 0, qidx(offset+3));  J.setMatrixBlock(-3.*J_bar(2), 0, qidx(offset+2));  J.setMatrixBlock(3.*J_bar(1), 0, qidx(offset+1));  J.setMatrixBlock(-J_bar(0), 0, qidx(offset+0));  J/=tau3; }
#else
    J = zeros(G.N, y.N, J_bar(0).d1);
    if(k==1){ J[offset+1]() =  J_bar(1);  J[offset+0]() =    -J_bar(0);  J/=tau; }
    if(k==2){ J[offset+2]() =  J_bar(2);  J[offset+1]() = -2.*J_bar(1);  J[offset+0]() = J_bar(0);  J/=tau2; }
    if(k==3){ J[offset+3]() =  J_bar(3);  J[offset+2]() = -3.*J_bar(2);  J[offset+1]() = 3.*J_bar(1);  J[offset+0]() = -J_bar(0);  J/=tau3; }
    arr tmp(J);
    tensorPermutation(J, tmp, TUP(1u,0u,2u));
    J.reshape(y.N, G.N*J_bar(0).d1);
#endif
  }
}

//===========================================================================

TaskMap *TaskMap::newTaskMap(const Graph& params, const mlr::KinematicWorld& world){
  //-- get tags
  mlr::String type = params.get<mlr::String>("map", "default");

  //-- create a task map
  TaskMap *map;
  if(type=="wheels"){
    map = new TaskMap_qItself(QIP_byJointNames, {"worldTranslationRotation"}, world);
  }else if(type=="collisionIneq"){
    map = new CollisionConstraint( params.get<double>("margin", 0.1) );
  }else if(type=="limitIneq"){
    map = new LimitsConstraint();
  }else if(type=="proxy"){
    map = new TaskMap_Proxy(allPTMT, {0u}, params.get<double>("margin", 0.1) );
  }else if(type=="collisionPairs"){
    uintA shapes;
    NIY;
//    for(uint i=2;i<params->parents.N;i++){
//      mlr::Shape *s = world.getShapeByName(params->parents(i)->keys.last());
//      CHECK(s,"No Shape '" <<params->parents(i)->keys.last() <<"'");
//      shapes.append(s->index);
//    }
//    map = new ProxyConstraint(pairsPTMT, shapes, (params?params->get<double>("margin", 0.1):0.1));
  }else if(type=="collisionExceptPairs"){
    uintA shapes;
    NIY;
//    for(uint i=2;i<params->parents.N;i++){
//      mlr::Shape *s = world.getShapeByName(params->parents(i)->keys.last());
//      CHECK(s,"No Shape '" <<params->parents(i)->keys.last() <<"'");
//      shapes.append(s->index);
//    }
//    map = new ProxyConstraint(allExceptPairsPTMT, shapes, (params?params->get<double>("margin", 0.1):0.1));
  }else if(type=="collisionExcept"){
    uintA shapes;
    NIY;
//    for(uint i=2;i<params->parents.N;i++){
//      mlr::Shape *s = world.getShapeByName(params->parents(i)->keys.last());
//      if(!s){
//        mlr::Body *b = world.getBodyByName(params->parents(i)->keys.last());
//        CHECK(b,"No shape or body '" <<params->parents(i)->keys.last() <<"'");
//        for(mlr::Shape *s:b->shapes) shapes.append(s->index);
//      }else{
//        shapes.append(s->index);
//      }
//    }
//    map = new ProxyConstraint(allExceptListedPTMT, shapes, (params?params->get<double>("margin", 0.1):0.1));
  }else if(type=="qItself"){
    if(params["ref1"] && params["ref2"]){
      mlr::Joint *j=world.getJointByBodyNames(params.get<mlr::String>("ref1"), params.get<mlr::String>("ref2"));
      if(!j) return NULL;
      map = new TaskMap_qItself({j->to->index}, false);
    }else if(params["ref1"]) map = new TaskMap_qItself(QIP_byJointNames, {params.get<mlr::String>("ref1")}, world);
    else if(params["Hmetric"]){ NIY /* map = new TaskMap_qItself(params.get<double>("Hmetric")*world.getHmetric());*/ } //world.naturalQmetric()); //
    else map = new TaskMap_qItself();
  }else if(type=="qZeroVels"){
    map = new TaskMap_qZeroVels();
  }else if(type=="GJK"){
    map = new TaskMap_GJK(world, params.get<mlr::String>("ref1"), params.get<mlr::String>("ref2"), true);
  }else{
    map = new TaskMap_Default(params, world);
  }

  map->order = params.get<double>("order", 0);
  return map;
}

//===========================================================================

TaskMap *TaskMap::newTaskMap(const Node* specs, const mlr::KinematicWorld& world){
  if(specs->parents.N<2) return NULL; //these are not task specs

  //-- get tags
  mlr::String& type=specs->parents(1)->keys.last();
  const char *ref1=NULL, *ref2=NULL;
  if(specs->parents.N>2) ref1=specs->parents(2)->keys.last().p;
  if(specs->parents.N>3) ref2=specs->parents(3)->keys.last().p;

  //-- create a task map
  TaskMap *map;
  const Graph* params=NULL;
  if(specs->isGraph()) params = &specs->graph();
//  mlr::String type = specs.get<mlr::String>("type", "pos");
  if(type=="wheels"){
    map = new TaskMap_qItself(QIP_byJointNames, {"worldTranslationRotation"}, world);
  }else if(type=="collisionIneq"){
    map = new CollisionConstraint( (params?params->get<double>("margin", 0.1):0.1) );
  }else if(type=="limitIneq"){
    map = new LimitsConstraint();
  }else if(type=="collisionPairs"){
    uintA shapes;
    for(uint i=2;i<specs->parents.N;i++){
      mlr::Shape *s = world.getShapeByName(specs->parents(i)->keys.last());
      CHECK(s,"No Shape '" <<specs->parents(i)->keys.last() <<"'");
      shapes.append(s->index);
    }
    map = new TaskMap_ProxyConstraint(pairsPTMT, shapes, (params?params->get<double>("margin", 0.1):0.1));
  }else if(type=="collisionExceptPairs"){
    uintA shapes;
    for(uint i=2;i<specs->parents.N;i++){
      mlr::Shape *s = world.getShapeByName(specs->parents(i)->keys.last());
      CHECK(s,"No Shape '" <<specs->parents(i)->keys.last() <<"'");
      shapes.append(s->index);
    }
    map = new TaskMap_ProxyConstraint(allExceptPairsPTMT, shapes, (params?params->get<double>("margin", 0.1):0.1));
  }else if(type=="collisionExcept"){
    uintA shapes;
    for(uint i=2;i<specs->parents.N;i++){
      mlr::Shape *s = world.getShapeByName(specs->parents(i)->keys.last());
      if(!s){
        mlr::Body *b = world.getBodyByName(specs->parents(i)->keys.last());
        CHECK(b,"No shape or body '" <<specs->parents(i)->keys.last() <<"'");
        for(mlr::Shape *s:b->shapes) shapes.append(s->index);
      }else{
        shapes.append(s->index);
      }
    }
    map = new TaskMap_ProxyConstraint(allExceptListedPTMT, shapes, (params?params->get<double>("margin", 0.1):0.1));
  }else if(type=="proxy"){
    map = new TaskMap_Proxy(allPTMT, {0u}, (params?params->get<double>("margin", 0.1):0.1) );
  }else if(type=="qItself"){
    if(ref1 && ref2){
      mlr::Joint *j=world.getJointByBodyNames(ref1, ref2);
      if(!j) return NULL;
      map = new TaskMap_qItself({j->to->index}, false);
    }else if(ref1) map = new TaskMap_qItself(QIP_byJointNames, {ref1}, world);
    else if(params && params->getNode("Hmetric")){ NIY /*map = new TaskMap_qItself(params->getNode("Hmetric")->get<double>()*world.getHmetric()); */}//world.naturalQmetric()); //
    else if(params && params->getNode("relative")) map = new TaskMap_qItself(true); //world.naturalQmetric()); //
    else map = new TaskMap_qItself();
  }else if(type=="qZeroVels"){
    map = new TaskMap_qZeroVels();
  }else if(type=="GJK"){
    map = new TaskMap_GJK(world, ref1, ref2, true);
  }else if(type=="Transition"){
    map = new TaskMap_Transition(world);
  }else if(type=="FixJointVelocities"){
    map = new TaskMap_Transition(world, true);
    dynamic_cast<TaskMap_Transition*>(map)->velCoeff = 1.;
  }else if(type=="FixSwichedObjects"){
    map = new TaskMap_FixSwichedObjects();
  }else{
    map = new TaskMap_Default(specs, world);
  }

  //-- check additional real-valued parameters: order
  if(specs->isGraph()){
    const Graph& params = specs->graph();
    map->order = params.get<double>("order", 0);
  }
  return map;
}
#include "PhysXThread.h"
#include <Kin/kin_physx.h>
#include <Kin/kinViewer.h>

struct PhysXThread : Thread{
  ACCESS(mlr::KinematicWorld, modelWorld)
  ACCESS(mlr::KinematicWorld, physxWorld)
  ACCESS(arr, ctrl_q_ref)
  PhysXInterface *px;
  OrsViewer *view;
  OpenGL *gl;

  PhysXThread() : Thread("PhysX", .03), px(NULL), view(NULL), gl(NULL){
    threadLoop(true);
  }

  ~PhysXThread(){
    threadClose();
  }

  void open(){
    physxWorld.writeAccess();
    physxWorld() = modelWorld.get();
    for(uint i=physxWorld().joints.N;i--;){
      mlr::Joint *j = physxWorld().joints.elem(i);
      if(j->type==mlr::JT_rigid){
        LOG(0) <<"removing fixed joint '" <<j->tag() <<"' (assuming it is not articulated)";
        delete j;
      }
    }
    physxWorld.deAccess();
    px = new PhysXInterface(physxWorld.set());
    px->setArticulatedBodiesKinematic();
    view = new OrsViewer("physxWorld", .1);
    view->threadLoop();
   }

  void step(){
    physxWorld.writeAccess();
    physxWorld().setJointState(ctrl_q_ref.get());
    px->step();
    physxWorld.deAccess();
    if(gl) if(!(step_count%10)) gl->update(NULL, false, false, true);
  }

  void close(){
    if(gl) delete gl; gl=NULL;
    if(view) delete view; view=NULL;
    delete px; px=NULL;
  }

  void showInternalOpengl(){
    if(!gl){
      stepMutex.lock();
      gl = new OpenGL("Internal PhyesX display");
      gl->add(glStandardScene);
      gl->add(*px);
      gl->camera.setDefault();
      stepMutex.unlock();
    }
  }
};

Thread* newPhysXThread(){ return new PhysXThread(); }

#include "taskMap_pushConsistent.h"

TaskMap_PushConsistent::TaskMap_PushConsistent(int iShape, int jShape) : i(iShape), j(jShape){
  order=1;
}

TaskMap_PushConsistent::TaskMap_PushConsistent(const mlr::KinematicWorld &G,
                                               const char* iShapeName, const char* jShapeName) : i(-1), j(-1){
  mlr::Shape *a = iShapeName ? G.getShapeByName(iShapeName):NULL;
  mlr::Shape *b = jShapeName ? G.getShapeByName(jShapeName):NULL;
  if(a) i=a->index;
  if(b) j=b->index;
  order=1;
}

//void crossProduct(arr& y, arr& J1, arr& J2, const arr& x1, const arr& x2){
//  CHECK(x1.nd==1 && x2.nd==1, "");
//  CHECK(x1.N==3 && x2.N==3,"cross product only works for 3D vectors!");
//  y = crossProduct(x1,x2);
//  if(&J1) J1 = skew(x2);
//  if(&J2) J2 = -skew(x1);
//}


void TaskMap_PushConsistent::phi(arr& y, arr& J, const WorldL& G, double tau, int t){
  CHECK(G.N>=order+1,"I need at least " <<order+1 <<" configurations to evaluate");

  const mlr::KinematicWorld& G2 = *G.elem(-1);
  const mlr::KinematicWorld& G1 = *G.elem(-2);

  mlr::Body *body_i1 = G1.shapes(i)->body;
  mlr::Body *body_i2 = G2.shapes(i)->body;
  mlr::Body *body_j2 = G2.shapes(j)->body;

  arr yi1, yi2, yj2, Ji1, Ji2, Jj2;
  G1.kinematicsPos(yi1, Ji1, body_i1);
  G2.kinematicsPos(yi2, Ji2, body_i2);
  G2.kinematicsPos(yj2, Jj2, body_j2);

#if 1
//  tau = 1.; //1e-5;
  y = crossProduct((yi2-yi1)/tau, yi2-yj2);
//  cout <<"PC " <<t <<' ' <<(yi2-yi1)/tau <<' ' <<yi2-yj2 <<' ' <<y <<endl;
  if(&J){
    uint qidx=0;
    for(uint i=0;i<G.N;i++) qidx+=G(i)->q.N;
    J.resize(y.N, qidx).setZero();

    arr J1 = skew(yi2-yj2)*Ji1/tau;
    arr J2 = skew((yi2-yi1)/tau)*(Ji2-Jj2) - skew(yi2-yj2)*Ji2/tau;

    J.setMatrixBlock(J1, 0, qidx-(J1.d1+J2.d1));
    J.setMatrixBlock(J2, 0, qidx-J2.d1);
  }
#else
  arr a = ARR(1.,0,0);
  y = crossProduct(yi2-yi1, a);
  if(&J){
    uint qidx=0;
    for(uint i=0;i<G.N;i++) qidx+=G(i)->q.N;
    J.resize(y.N, qidx).setZero();

    arr J1 = skew(a)*Ji1;
    arr J2 = -skew(a)*Ji2;

    J.setMatrixBlock(J1, 0, qidx-(J1.d1+J2.d1));
    J.setMatrixBlock(J2, 0, qidx-J2.d1);
  }
#endif
}
/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */

#include "taskMap_default.h"

const char* TaskMap_DefaultType2String[] = {
"no",      ///< non-initialization
"pos",     ///< 3D position of reference
"vec",     ///< 3D vec (orientation)
"quat",    ///< 4D quaterion
"posDiff", ///< the difference of two positions (NOT the relative position)
"vecDiff", ///< the difference of two vectors (NOT the relative position)
"quatDiff",///< the difference of 2 quaternions (NOT the relative quaternion)
"vecAlign",///< 1D vector alignment, can have 2nd reference, param (optional) determins alternative reference world vector
"gazeAt",  ///< 2D orthogonality measure of object relative to camera plane
"pos1D"
};

TaskMap_Default::TaskMap_Default(TaskMap_DefaultType _type,
                               int iShape, const mlr::Vector& _ivec,
                               int jShape, const mlr::Vector& _jvec)
  :type(_type), i(iShape), j(jShape){
  if(&_ivec) ivec=_ivec; else ivec.setZero();
  if(&_jvec) jvec=_jvec; else jvec.setZero();
}

TaskMap_Default::TaskMap_Default(TaskMap_DefaultType _type, const mlr::KinematicWorld &G,
                               const char* iShapeName, const mlr::Vector& _ivec,
                               const char* jShapeName, const mlr::Vector& _jvec)
  :type(_type), i(-1), j(-1){
  mlr::Shape *a = iShapeName ? G.getShapeByName(iShapeName):NULL;
  mlr::Shape *b = jShapeName ? G.getShapeByName(jShapeName):NULL;
  if(a) i=a->index;
  if(b) j=b->index;
  if(&_ivec) ivec=_ivec; else ivec.setZero();
  if(&_jvec) jvec=_jvec; else jvec.setZero();
}

TaskMap_Default::TaskMap_Default(const Graph& specs, const mlr::KinematicWorld& G)
  :type(noTMT), i(-1), j(-1){
  Node *it=specs["type"];
  if(!it) it=specs["map"];
  if(!it) HALT("no type given");
  mlr::String Type=it->get<mlr::String>();
  if(Type=="pos") type=posTMT;
  else if(Type=="vec") type=vecTMT;
  else if(Type=="quat") type=quatTMT;
  else if(Type=="posDiff") type=posDiffTMT;
  else if(Type=="vecDiff") type=vecDiffTMT;
  else if(Type=="quatDiff") type=quatDiffTMT;
  else if(Type=="vecAlign") type=vecAlignTMT;
  else if(Type=="gazeAt") type=gazeAtTMT;
  else HALT("unknown type " <<Type);
  if((it=specs["sym2"]) || (it=specs["ref1"])){ auto name=it->get<mlr::String>(); auto *s=G.getShapeByName(name); CHECK(s,"shape name '" <<name <<"' does not exist"); i=s->index; }
  if((it=specs["sym3"]) || (it=specs["ref2"])){ auto name=it->get<mlr::String>(); auto *s=G.getShapeByName(name); CHECK(s,"shape name '" <<name <<"' does not exist"); j=s->index; }
  if((it=specs["vec1"])) ivec = mlr::Vector(it->get<arr>());  else ivec.setZero();
  if((it=specs["vec2"])) jvec = mlr::Vector(it->get<arr>());  else jvec.setZero();
}

TaskMap_Default::TaskMap_Default(const Node *specs, const mlr::KinematicWorld& G)
  :type(noTMT), i(-1), j(-1){
  CHECK(specs->parents.N>1,"");
//  mlr::String& tt=specs->parents(0)->keys.last();
  mlr::String& Type=specs->parents(1)->keys.last();
  const char *ref1=NULL, *ref2=NULL;
  if(specs->parents.N>2) ref1=specs->parents(2)->keys.last().p;
  if(specs->parents.N>3) ref2=specs->parents(3)->keys.last().p;
       if(Type=="pos") type=posTMT;
  else if(Type=="vec") type=vecTMT;
  else if(Type=="quat") type=quatTMT;
  else if(Type=="posDiff") type=posDiffTMT;
  else if(Type=="vecDiff") type=vecDiffTMT;
  else if(Type=="quatDiff") type=quatDiffTMT;
  else if(Type=="vecAlign") type=vecAlignTMT;
  else if(Type=="gazeAt") type=gazeAtTMT;
  else HALT("unknown type " <<Type);
  if(ref1){ mlr::Shape *s=G.getShapeByName(ref1); CHECK(s,"shape name '" <<ref1 <<"' does not exist"); i=s->index; }
  if(ref2){ mlr::Shape *s=G.getShapeByName(ref2); CHECK(s,"shape name '" <<ref2 <<"' does not exist"); j=s->index; }
  if(specs->isGraph()){
    const Graph& params = specs->graph();
    Node *it;
    if((it=params.getNode("vec1"))) ivec = mlr::Vector(it->get<arr>());  else ivec.setZero();
    if((it=params.getNode("vec2"))) jvec = mlr::Vector(it->get<arr>());  else jvec.setZero();
  }
}


void TaskMap_Default::phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t) {
  if(t>=0 && referenceIds.N){
    if(referenceIds.nd==1){  i=referenceIds(t); j=-1; }
    if(referenceIds.nd==2){  i=referenceIds(t,0); j=referenceIds(t,1); }
  }

  mlr::Body *body_i = i<0?NULL: G.shapes(i)->body;
  mlr::Body *body_j = j<0?NULL: G.shapes(j)->body;

  if(type==posTMT){
    mlr::Vector vec_i = i<0?ivec: G.shapes(i)->rel*ivec;
    mlr::Vector vec_j = j<0?jvec: G.shapes(j)->rel*jvec;
    CHECK(body_i,"");
    if(body_j==NULL) { //simple, no j reference
      G.kinematicsPos(y, J, body_i, vec_i);
      y -= conv_vec2arr(vec_j);
      return;
    }//else...
    mlr::Vector pi = body_i->X * vec_i;
    mlr::Vector pj = body_j->X * vec_j;
    y = conv_vec2arr(body_j->X.rot / (pi-pj));
    if(&J) {
      arr Ji, Jj, JRj;
      G.kinematicsPos(NoArr, Ji, body_i, vec_i);
      G.kinematicsPos(NoArr, Jj, body_j, vec_j);
      G.axesMatrix(JRj, body_j);
      J.resize(3, Jj.d1);
      for(uint k=0; k<Jj.d1; k++) {
        mlr::Vector vi(Ji(0, k), Ji(1, k), Ji(2, k));
        mlr::Vector vj(Jj(0, k), Jj(1, k), Jj(2, k));
        mlr::Vector r (JRj(0, k), JRj(1, k), JRj(2, k));
        mlr::Vector jk =  body_j->X.rot / (vi-vj);
        jk -= body_j->X.rot / (r ^ (pi-pj));
        J(0, k)=jk.x;
        J(1, k)=jk.y;
        J(2, k)=jk.z;
      }
    }
    return;
  }

  if(type==posDiffTMT){
    mlr::Vector vec_i = i<0?ivec: G.shapes(i)->rel*ivec;
    mlr::Vector vec_j = j<0?jvec: G.shapes(j)->rel*jvec;
    G.kinematicsPos(y, J, body_i, vec_i);
    if(!body_j){ //relative to world
      y -= conv_vec2arr(vec_j);
    }else{
      arr y2, J2;
      G.kinematicsPos(y2, (&J?J2:NoArr), body_j, vec_j);
      y -= y2;
      if(&J) J -= J2;
    }
    return;
  }

  if(type==vecTMT){
    mlr::Vector vec_i = i<0?ivec: G.shapes(i)->rel.rot*ivec;
//    mlr::Vector vec_j = j<0?jvec: G.shapes(j)->rel.rot*jvec;
    if(vec_i.isZero) MLR_MSG("attached vector is zero -- can't control that");
    if(body_j==NULL) { //simple, no j reference
      G.kinematicsVec(y, J, body_i, vec_i);
      return;
    }//else...
    //relative
    MLR_MSG("warning - don't have a correct Jacobian for this TMType yet");
    //      fi = G.bodies(body_i)->X; fi.appendTransformation(irel);
    //      fj = G.bodies(body_j)->X; fj.appendTransformation(jrel);
    //      f.setDifference(fi, fj);
    //      f.rot.getZ(c);
    //      y = conv_vec2arr(c);
    NIY; //TODO: Jacobian?
    return;
  }

  if(type==vecDiffTMT){
    mlr::Vector vec_i = i<0?ivec: G.shapes(i)->rel.rot*ivec;
    mlr::Vector vec_j = j<0?jvec: G.shapes(j)->rel.rot*jvec;
    G.kinematicsVec(y, J, body_i, vec_i);
    if(!body_j){ //relative to world
      if(vec_i.isZero) MLR_MSG("attached vector is zero -- can't control that");
      y -= conv_vec2arr(vec_j);
    }else{
      if(vec_i.isZero) MLR_MSG("attached vector1 is zero -- can't control that");
      if(vec_j.isZero) MLR_MSG("attached vector2 is zero -- can't control that");
      arr y2, J2;
      G.kinematicsVec(y2, J2, body_j, vec_j);
      y -= y2;
      J -= J2;
    }
    return;
  }

  if(type==vecAlignTMT) {
    CHECK(fabs(ivec.length()-1.)<1e-4,"vector references must be normalized");
    CHECK(fabs(jvec.length()-1.)<1e-4,"vector references must be normalized");
    mlr::Vector vec_i = i<0?ivec: G.shapes(i)->rel.rot*ivec;
    mlr::Vector vec_j = j<0?jvec: G.shapes(j)->rel.rot*jvec;
    arr zi,Ji,zj,Jj;
    G.kinematicsVec(zi, Ji, body_i, vec_i);
    if(body_j==NULL) {
      zj = conv_vec2arr(vec_j);
      if(&J) { Jj.resizeAs(Ji); Jj.setZero(); }
    } else {
      G.kinematicsVec(zj, Jj, body_j, vec_j);
    }
    y.resize(1);
    y(0) = scalarProduct(zi, zj);
    if(&J) {
      J = ~zj * Ji + ~zi * Jj;
      J.reshape(1, G.getJointStateDimension());
    }
    return;
  }

  if(type==pos1DTMT) {
    CHECK(fabs(ivec.length()-1.)<1e-10,"vector references must be normalized");
    arr orientation = conv_vec2arr(ivec);
    G.kinematicsPos(y, NoArr, body_i);
    y = ~orientation*y;
    if(&J) {
      G.kinematicsPos(NoArr, J, body_i);
      J = ~orientation*J;
      J.reshape(1, J.N);
    }
    return;
  }

  if(type==gazeAtTMT){
    CHECK(i>=0, "sym2 is not set!");

    // i    := index of shape to look with (i.e. the shape with the camera)
    // ivec := relative position of the camera center
    // j    := index of shape to look at
    // jvec := relative position on the target shape; where in the target shape should we look.
    //         If j is not set, the target shape is WORLD and jvec is a vector in world coordinates

    mlr::Vector vec_i = G.shapes(i)->rel*ivec;
    mlr::Vector vec_xi = G.shapes(i)->rel.rot*Vector_x;
    mlr::Vector vec_yi = G.shapes(i)->rel.rot*Vector_y;
    mlr::Vector vec_j = j<0?jvec: G.shapes(j)->rel*jvec;
    arr pi,Jpi, xi,Jxi, yi,Jyi, pj,Jpj;
    G.kinematicsPos(pi, Jpi, body_i, vec_i);
    G.kinematicsVec(xi, Jxi, body_i, vec_xi);
    G.kinematicsVec(yi, Jyi, body_i, vec_yi);
    if(body_j==NULL) { //we look at WORLD
      pj = conv_vec2arr(vec_j);
      if(&J) { Jpj.resizeAs(Jpi); Jpj.setZero(); }
    } else {
      G.kinematicsPos(pj, Jpj, body_j, vec_j);
    }
    y.resize(2);
    y(0) = scalarProduct(xi, (pj-pi));
    y(1) = scalarProduct(yi, (pj-pi));
    if(&J) {
      J = cat( ~xi * (Jpj-Jpi) + ~(pj-pi) * Jxi,
               ~yi * (Jpj-Jpi) + ~(pj-pi) * Jyi );
      J.reshape(2, G.getJointStateDimension());
    }
    return;
  }

  if(type==quatTMT){
    if(body_j==NULL) { //simple, no j reference
      G.kinematicsQuat(y, J, body_i);
      return;
    }//else...
    NIY;
  }

  if(type==quatDiffTMT){
    G.kinematicsQuat(y, J, body_i);
    if(!body_j){ //relative to world
       //diff to world, which is Id
      if(y(0)>=0.) y(0) -= 1.; else y(0) += 1.;
    }else{
      arr y2, J2;
      G.kinematicsQuat(y2, J2, body_j);
      if(scalarProduct(y,y2)>=0.){
        y -= y2;
        if(&J) J -= J2;
      }else{
        y += y2;
        if(&J) J += J2;
      }
    }
    return;
  }

  if(type==poseDiffTMT){
    arr yq, Jq;
    TaskMap_Default tmp(*this);
    tmp.type = posDiffTMT;
    tmp.phi(y, J, G);
    tmp.type = quatDiffTMT;
    tmp.phi(yq, (&J?Jq:NoArr), G);
    y.append(yq);
    if(&J) J.append(Jq);
    return;
  }

  HALT("no such TVT");
}

uint TaskMap_Default::dim_phi(const mlr::KinematicWorld& G) {
  switch(type) {
    case posTMT: return 3;
    case vecTMT: return 3;
    case quatTMT: return 4;
    case posDiffTMT: return 3;
    case vecDiffTMT: return 3;
    case quatDiffTMT: return 4;
    case vecAlignTMT: return 1;
    case gazeAtTMT: return 2;
    case pos1DTMT: return 1;
    default:  HALT("no such TMT");
  }
}

mlr::String TaskMap_Default::shortTag(const mlr::KinematicWorld& G){
  mlr::String s="Default";
  s <<':' <<TaskMap_DefaultType2String[type];
  s <<':' <<(i<0?"WORLD":G.shapes(i)->name);
  s <<'/' <<(j<0?"WORLD":G.shapes(j)->name);
  return s;
}


//===========================================================================

/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */

#include "taskMaps.h"

//===========================================================================

void CollisionConstraint::phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t){
  G.kinematicsProxyCost(y, J, margin, false);
  y -= .5;
}

//===========================================================================

void PairCollisionConstraint::phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t){
  if(t>=0 && referenceIds.N){
    if(referenceIds.nd==1){  i=referenceIds(t); j=-1; }
    if(referenceIds.nd==2){  i=referenceIds(t,0); j=referenceIds(t,1); }
  }

  y.resize(1) = -1.; //default value if not overwritten below
  if(&J) J.resize(1,G.q.N).setZero();
  if(j>=0){ //against a concrete j
    for(mlr::Proxy *p: G.proxies){
      if((p->a==i && p->b==j) || (p->a==j && p->b==i)){
        G.kinematicsProxyConstraint(y, J, p, margin);
        break;
      }
    }
  }else if(j==-1){ //against all objects
    NIY; //this doesn't work, don't know why
    //first collect all relevant proxies
    ProxyL P;
    for(mlr::Proxy *p: G.proxies) if((p->a==i) || (p->b==i)) P.append(p);
    //Compute the softmax
    double alpha = 10.;
    double yHat=0.,yNorm=0.;
    for(mlr::Proxy *p: P){
      G.kinematicsProxyConstraint(y, NoArr, p, margin);
      double yi=y.scalar();
      double expyi=::exp(alpha*yi);
      yNorm += expyi;
      yHat  += expyi * yi;
    }
    yHat /= yNorm;
    //compute derivative
    if(&J){
      J.resize(1,G.getJointStateDimension()).setZero();
      arr Ji;
      for(mlr::Proxy *p: P){
        G.kinematicsProxyConstraint(y, Ji, p, margin);
        double yi=y.scalar();
        double expyi=::exp(alpha*yi);
        J += expyi * (1.+alpha*(yi-yHat)) * Ji;
      }
      J /= yNorm;
    }
    y.scalar() = yHat;
  }
}

//===========================================================================

void PlaneConstraint::phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t){
  mlr::Body *body_i = G.shapes(i)->body;
  mlr::Vector vec_i = G.shapes(i)->rel.pos;

  arr y_eff, J_eff;
  G.kinematicsPos(y_eff, (&J?J_eff:NoArr), body_i, vec_i);

  y_eff.append(1.); //homogeneous coordinates
  if(&J) J_eff.append(zeros(1,J_eff.d1));

  y.resize(1);
  y(0) = scalarProduct(y_eff, planeParams);
  if(&J) J = ~planeParams * J_eff;
}

//===========================================================================

void ConstraintStickiness::phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t){
  map.phi(y, J, G);
  for(uint j=0;j<y.N;j++) y(j) = -y(j);
  if(&J) for(uint j=0;j<J.d0;j++) J[j]() *= -1.;
}

//===========================================================================

void PointEqualityConstraint::phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t){
  mlr::Vector vec_i = i<0?ivec: G.shapes(i)->rel*ivec;
  mlr::Vector vec_j = j<0?jvec: G.shapes(j)->rel*jvec;
  mlr::Body *body_i = i<0?NULL: G.shapes(i)->body;
  mlr::Body *body_j = j<0?NULL: G.shapes(j)->body;
  mlr::Vector pi = body_i ? body_i->X * vec_i : vec_i;
  mlr::Vector pj = body_j ? body_j->X * vec_j : vec_j;
  y = conv_vec2arr(pi-pj);
  if(&J) {
    arr Ji, Jj;
    G.kinematicsPos(NoArr, Ji, body_i, vec_i);
    if(body_j){
      G.kinematicsPos(NoArr, Jj, body_j, vec_j);
      J = Ji - Jj;
    }else{
      J = Ji;
    }
  }
}

//===========================================================================

void ContactEqualityConstraint::phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t){
  y.resize(1) = 0.;
  if(&J) J.resize(1,G.q.N).setZero();
  for(mlr::Proxy *p: G.proxies){
    if((p->a==i && p->b==j) || (p->a==j && p->b==i)){
      G.kinematicsProxyConstraint(y, J, p, margin);
      break;
    }
  }
}

//===========================================================================


VelAlignConstraint::VelAlignConstraint(const mlr::KinematicWorld& G,
                   const char* iShapeName, const mlr::Vector& _ivec,
                   const char* jShapeName, const mlr::Vector& _jvec, double _target) {
  mlr::Shape *a = iShapeName ? G.getShapeByName(iShapeName):NULL;
  mlr::Shape *b = jShapeName ? G.getShapeByName(jShapeName):NULL;
  if(a) i=a->index;
  if(b) j=b->index;
  if(&_ivec) ivec=_ivec; else ivec.setZero();
  if(&_jvec) jvec=_jvec; else jvec.setZero();
  order = 1;
  target = _target;
}

void VelAlignConstraint::phi(arr& y, arr& J, const WorldL& G, double tau, int t) {
  uint k=order;

  // compute body j orientation
  arr y_j,J_j,J_bar_j;
  G(G.N-1)->kinematicsVec(y_j, (&J?J_bar_j:NoArr), G(G.N-1)->shapes(j)->body, jvec);

  if(&J){
    J_j = zeros(G.N, y_j.N, J_bar_j.d1);
    J_j[G.N-1]() = J_bar_j;
    arr tmp(J_j);
    tensorPermutation(J_j, tmp, TUP(1u,0u,2u));
    J_j.reshape(y_j.N, G.N*J_bar_j.d1);
  }

  // compute body i velocity
  arrA y_bar, J_bar;
  y_bar.resize(k+1);
  J_bar.resize(k+1);

  for(uint c=0;c<=k;c++) {
    G(G.N-1-c)->kinematicsPos(y_bar(c), (&J?J_bar(c):NoArr), G(G.N-1-c)->shapes(i)->body, ivec);
  }

  arr dy_i, dJ_i;
  dy_i = (y_bar(0)-y_bar(1));

  if (&J) {
    dJ_i = zeros(G.N, dy_i.N, J_bar(0).d1);
    dJ_i[G.N-1-1]() = -J_bar(1);
    dJ_i[G.N-1-0]() = J_bar(0);
    arr tmp(dJ_i);
    tensorPermutation(dJ_i, tmp, TUP(1u,0u,2u));
    dJ_i.reshape(dy_i.N, G.N*J_bar(0).d1);
  }

  // normalize dy_i
  if (length(dy_i) != 0) {
    if (&J) {
      double tmp = (~dy_i*dy_i).scalar();
      dJ_i = ( eye(dJ_i.d0) - (dy_i*~dy_i)/(tmp) )*dJ_i/(length(dy_i));
    }
    dy_i = dy_i/(length(dy_i));
  }

  innerProduct(y,~dy_i,y_j);

  if (&J) {
    J = ~dy_i*J_j + ~y_j*dJ_i;
    J = -J;
  }
  y = -y+target;

}

//===========================================================================

void qItselfConstraint::phi(arr& q, arr& J, const mlr::KinematicWorld& G, int t) {
  G.getJointState(q);
  if(M.N){
    if(M.nd==1){
      q=M%q; if(&J) J.setDiag(M);
    }else{
      q=M*q; if(&J) J=M;
    }
  }else{
    if(&J) J.setId(q.N);
  }
}


/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */

/**
 * @file
 * @ingroup group_ors
 */
/**
 * @ingroup group_ors
 * @{
 */


#include "kin_swift.h"
#include <Algo/ann.h>

#ifdef MLR_extern_SWIFT

#ifdef MLR_SINGLE
#  define SWIFT_USE_FLOAT
#endif

#include "SWIFT/SWIFT.h"
#undef min
#undef max

ANN *global_ANN=NULL;
mlr::Shape *global_ANN_shape;

SwiftInterface::~SwiftInterface() {
  if(scene) delete scene;
  if(global_ANN) delete global_ANN;
  scene=NULL;
  cout <<" -- SwiftInterface closed" <<endl;
}

SwiftInterface::SwiftInterface(const mlr::KinematicWorld& world, double _cutoff)
  : scene(NULL), cutoff(_cutoff) {
  bool r, add;
  
  if(scene) delete scene;

  scene=new SWIFT_Scene(false, false);

  INDEXswift2shape.resize(world.shapes.N);  INDEXswift2shape=-1;
  INDEXshape2swift.resize(world.shapes.N);  INDEXshape2swift=-1;
  
  //cout <<" -- SwiftInterface init";
  for_list(mlr::Shape, s,  world.shapes) if(s->cont){
    //cout <<'.' <<flush;
    add=true;
    switch(s->type) {
      case mlr::ST_none: HALT("shapes should have a type - somehow wrong initialization..."); break;
      case mlr::ST_mesh: {
        //check if there is a specific swiftfile!
        mlr::String *filename;
        filename=s->ats.find<mlr::String>("swiftfile");
        if(!filename)
          filename=s->body->ats.find<mlr::String>("swiftfile");
        if(filename) {
          r=scene->Add_General_Object(*filename, INDEXshape2swift(s->index), false);
          if(!r) HALT("--failed!");
        }
      } break;
      case mlr::ST_pointCloud: {
        //for now, assume there is only ONE pointCloudObject!
        CHECK(s->mesh.V.N, "");
        global_ANN=new ANN;
        global_ANN_shape=s;
        global_ANN->setX(s->mesh.V);
        global_ANN->calculate();
        add=false;
      } break;
      case mlr::ST_marker:
        add=false; // ignore (no collisions)
        break;
      default:
        break;
    }
    if(add) {
      if(!s->mesh.V.d0){
        switch(s->type) {
          case mlr::ST_box:
            s->mesh.setBox();
            s->mesh.scale(s->size(0), s->size(1), s->size(2));
            break;
          case mlr::ST_sphere:
            s->mesh.setSphere();
            s->mesh.scale(s->size(3), s->size(3), s->size(3));
            break;
          case mlr::ST_cylinder:
            CHECK(s->size(3)>1e-10,"");
            s->mesh.setCylinder(s->size(3), s->size(2));
            break;
          case mlr::ST_capsule:
            CHECK(s->size(3)>1e-10,"");
            s->mesh.setCappedCylinder(s->size(3), s->size(2));
            break;
          case mlr::ST_retired_SSBox:
            s->mesh.setSSBox(s->size(0), s->size(1), s->size(2), s->size(3));
            break;
          default:
            break;
        }
        s->mesh_radius = s->mesh.getRadius();
      }
      CHECK(s->mesh.V.d0,"no mesh to add to SWIFT, something was wrongly initialized");
      r=scene->Add_Convex_Object(
          s->mesh.V.p, (int*)s->mesh.T.p,
          s->mesh.V.d0, s->mesh.T.d0, INDEXshape2swift(s->index), false,
          DEFAULT_ORIENTATION, DEFAULT_TRANSLATION, DEFAULT_SCALE,
          DEFAULT_BOX_SETTING, DEFAULT_BOX_ENLARGE_REL, 2.);
      if(!r) HALT("--failed!");
      
      INDEXswift2shape(INDEXshape2swift(s->index)) = s->index;
    }
  }
  
  initActivations(world, 4);
  
  pushToSwift(world);
  //cout <<"...done" <<endl;
}

void SwiftInterface::reinitShape(const mlr::Shape *s) {
  int sw = INDEXshape2swift(s->index);
  scene->Delete_Object(sw);
  INDEXswift2shape(sw) = -1;
  
  bool r=scene->Add_Convex_Object(s->mesh.V.p, (int*)s->mesh.T.p,
                                  s->mesh.V.d0, s->mesh.T.d0, sw, false,
                                  DEFAULT_ORIENTATION, DEFAULT_TRANSLATION, DEFAULT_SCALE,
                                  DEFAULT_BOX_SETTING, DEFAULT_BOX_ENLARGE_REL, cutoff);
  if(!r) HALT("--failed!");
  
  INDEXshape2swift(s->index) = sw;
  INDEXswift2shape(sw) = s->index;
  
  if(s->cont) scene->Activate(sw);
}

void SwiftInterface::initActivations(const mlr::KinematicWorld& world, uint parentLevelsToDeactivate) {
  /* deactivate some collision pairs:
    -- no `cont' -> no collisions with this object at all
    -- no collisions between shapes of same body
    -- no collisions between linked bodies
    -- no collisions between bodies liked via the tree via 3 links
  */
  
  //cout <<"collision active shapes: ";
  //for_list(Type,  s,  world.shapes) if(s->cont) cout <<s->name <<' ';
  
  for_list(mlr::Shape, s, world.shapes) {
    if(!s->cont) {
      if(INDEXshape2swift(s->index)!=-1) scene->Deactivate(INDEXshape2swift(s->index));
    } else {
      if(INDEXshape2swift(s->index)!=-1) scene->Activate(INDEXshape2swift(s->index));
    }
  }
  //shapes within a body
  for(mlr::Body *b: world.bodies) deactivate(b->shapes);
  //deactivate along edges...
  for_list(mlr::Joint, e, world.joints) {
    //cout <<"deactivating edge pair"; listWriteNames({e->from, e->to}, cout); cout <<endl;
    deactivate(mlr::Array<mlr::Body*>({ e->from, e->to }));
  }
  //deactivate along trees...
  for_list(mlr::Body,  b,  world.bodies) {
    mlr::Array<mlr::Body*> group, children;
    group.append(b);
    for(uint l=0; l<parentLevelsToDeactivate; l++) {
      //listWriteNames(group, cout);
      children.clear();
      for_list(mlr::Body,  b2,  group) {
        for_list(mlr::Joint,  e,  b2->outLinks) {
          children.setAppend(e->to);
          //listWriteNames(children, cout);
        }
      }
      group.setAppend(children);
    }
    deactivate(group);
  }
}

void SwiftInterface::deactivate(const mlr::Array<mlr::Body*>& bodies) {
  //cout <<"deactivating body group "; listWriteNames(bodies, cout); cout <<endl;
  mlr::Array<mlr::Shape*> shapes;
  for_list(mlr::Body, b, bodies) shapes.setAppend(b->shapes);
  deactivate(shapes);
}

void SwiftInterface::deactivate(const mlr::Array<mlr::Shape*>& shapes) {
  //cout <<"deactivating shape group "; listWriteNames(shapes, cout); cout <<endl;
  for_list(mlr::Shape, s1, shapes){
    for_list(mlr::Shape, s2, shapes) {
      if(s1_COUNT>s2_COUNT) deactivate(s1, s2);
    }
  }
}

void SwiftInterface::deactivate(mlr::Shape *s1, mlr::Shape *s2) {
  if(INDEXshape2swift(s1->index)==-1 || INDEXshape2swift(s2->index)==-1) return;
  //cout <<"deactivating shape pair " <<s1->name <<'-' <<s2->name <<endl;
  scene->Deactivate(INDEXshape2swift(s1->index), INDEXshape2swift(s2->index));
}

void SwiftInterface::activate(mlr::Shape *s1, mlr::Shape *s2) {
  if(INDEXshape2swift(s1->index)==-1 || INDEXshape2swift(s2->index)==-1) return;
  //cout <<"deactivating shape pair " <<s1->name <<'-' <<s2->name <<endl;
  scene->Activate(INDEXshape2swift(s1->index), INDEXshape2swift(s2->index));
}

void SwiftInterface::activate(mlr::Shape *s) {
  if(INDEXshape2swift(s->index)==-1) return;
  scene->Activate(INDEXshape2swift(s->index));
}

void SwiftInterface::deactivate(mlr::Shape *s) {
  if(INDEXshape2swift(s->index)==-1) return;
  scene->Deactivate(INDEXshape2swift(s->index));
}

void SwiftInterface::pushToSwift(const mlr::KinematicWorld& world) {
  //CHECK_EQ(INDEXshape2swift.N,world.shapes.N,"the number of shapes has changed");
  CHECK(INDEXshape2swift.N <= world.shapes.N, "the number of shapes has changed");
  mlr::Matrix rot;
  for_list(mlr::Shape,  s,  world.shapes) {
    rot = s->X.rot.getMatrix();
    if(s->index<INDEXshape2swift.N && INDEXshape2swift(s->index)!=-1) {
      scene->Set_Object_Transformation(INDEXshape2swift(s->index), rot.p(), s->X.pos.p());
      if(!s->cont) scene->Deactivate(INDEXshape2swift(s->index));
      //else         scene->Activate( INDEXshape2swift(s->index) );
    }
  }
}

void SwiftInterface::pullFromSwift(mlr::KinematicWorld& world, bool dumpReport) {
  int i, j, k, np;
  int *oids, *num_contacts;
  SWIFT_Real *dists, *nearest_pts, *normals;
  
  try {
    scene->Query_Contact_Determination(
      false, cutoff, np,
      &oids, &num_contacts,
      &dists,
      &nearest_pts,
      &normals);
  } catch(const char *msg) {
    listResize(world.proxies, 0);
    cout <<"... catching error '" <<msg <<"' -- SWIFT failed! .. no proxies for this posture!!..." <<endl;
    return;
  }
  
  if(dumpReport) {
    cout <<"contacts: np=" <<np <<endl;
    for(k=0, i=0; i<np; i++) {
      cout <<"* Shape '" <<world.shapes(oids[i <<1])->name <<"' vs. Shape '" <<world.shapes(oids[(i <<1)+1])->name <<"'" <<endl;
      cout <<"  #contacts = " <<num_contacts[i] <<endl;
      for(j=0; j<num_contacts[i]; j++, k++) {
        cout <<"  - contact " <<j <<endl;
        cout <<"    distance= " <<dists[k] <<endl;
        cout <<"    points  = " <<nearest_pts[6*k+0] <<' ' <<nearest_pts[6*k+1] <<' ' <<nearest_pts[6*k+2] <<' ' <<nearest_pts[6*k+3] <<' ' <<nearest_pts[6*k+4] <<' ' <<nearest_pts[6*k+5] <<endl;
        cout <<"    normals = " <<normals[3*k+0] <<' ' <<normals[3*k+1] <<' ' <<normals[3*k+2] <<endl;
      }
    }
  }
  
  //count total number of new proxies:
  for(k=0, i=0; i<np; i++) {
    if(num_contacts[i]>=0) k+=num_contacts[i];
    if(num_contacts[i]==-1) k++;
  }
  
  listResize(world.proxies, k);
  
  //add contacts to list
  mlr::Proxy *proxy;
  int a, b;
  for(k=0, i=0; i<np; i++) {
    a=INDEXswift2shape(oids[i <<1]);
    b=INDEXswift2shape(oids[(i <<1)+1]);
    //CHECK(ids(a)==a && ids(b)==b, "shape index does not coincide with swift index");
    
    //non-penetrating pair of objects
    if(num_contacts[i]>=0) for(j=0; j<num_contacts[i]; j++, k++) {
        proxy=world.proxies(k);
        proxy->a=a;
        proxy->b=b;
        proxy->d = dists[k];
        proxy->normal.set(&normals[3*k+0]);
        proxy->normal.normalize();
        //swift returns nearest points in the local frame -> transform them
        proxy->posA.set(&nearest_pts[6*k+0]);  proxy->posA = world.shapes(a)->X * proxy->posA;
        proxy->posB.set(&nearest_pts[6*k+3]);  proxy->posB = world.shapes(b)->X * proxy->posB;
        proxy->cenA = world.shapes(a)->X.pos;
        proxy->cenB = world.shapes(b)->X.pos;
//        if(world.shapes(a)->type==mlr::ST_mesh) proxy->cenA = world.shapes(a)->X * world.shapes(a)->mesh.getMeanVertex(); else proxy->cenA = world.shapes(a)->X.pos;
//        if(world.shapes(b)->type==mlr::ST_mesh) proxy->cenB = world.shapes(b)->X * world.shapes(b)->mesh.getMeanVertex(); else proxy->cenB = world.shapes(b)->X.pos;
        proxy->cenN = proxy->cenA - proxy->cenB; //normal always points from b to a
        proxy->cenD = proxy->cenN.length();
        proxy->cenN /= proxy->cenD;
      }
      
    //penetrating pair of objects
    if(num_contacts[i]==-1) {
      proxy=world.proxies(k);
      proxy->a=a;
      proxy->b=b;
      proxy->d = -.0;
//      if(world.shapes(a)->type==mlr::ST_mesh) proxy->cenA = world.shapes(a)->X * world.shapes(a)->mesh.getMeanVertex(); else proxy->cenA = world.shapes(a)->X.pos;
//      if(world.shapes(b)->type==mlr::ST_mesh) proxy->cenB = world.shapes(b)->X * world.shapes(b)->mesh.getMeanVertex(); else proxy->cenB = world.shapes(b)->X.pos;
      proxy->cenA = world.shapes(a)->X.pos;
      proxy->cenB = world.shapes(b)->X.pos;
      proxy->cenN = proxy->cenA - proxy->cenB; //normal always points from b to a
      proxy->cenD = proxy->cenN.length();
      proxy->cenN /= proxy->cenD;
      
      //copy to pos..
      proxy->posA = proxy->cenA;
      proxy->posB = proxy->cenB;
      proxy->normal = proxy->cenN;
      
      ///! IN PENETRATION we measure d as -1+(distance between object centers) - that gives a well-defined (though non-smooth) gradient!
//      proxy->d += -1.+(proxy->posA-proxy->posB).length();
      k++;
    }

    double ab_radius = mlr::MAX(proxy->d,0.) + 1.1*(world.shapes(a)->mesh_radius + world.shapes(b)->mesh_radius);
    if(proxy->cenD>ab_radius){
      //MLR_MSG("shit");
    }
  }
  CHECK_EQ(k , (int)world.proxies.N, "");
  
  //add pointClound stuff to list
  if(global_ANN) {
    uint i, _i=0;
    arr R(3, 3), t(3);
    arr v, dists, _dists;
    intA idx, _idx;
    for_list(mlr::Shape,  s,  world.shapes) {
      if(!s->cont || s==global_ANN_shape) continue;
      
      //relative rotation and translation of shapes
      mlr::Transformation rel;
      rel.setDifference(global_ANN_shape->X, s->X);
      rel.rot.getMatrix(R.p);
      t = conv_vec2arr(rel.pos);
      
      //check for each vertex
      for(i=0; i<s->mesh.V.d0; i++) {
        v = s->mesh.V[i];
        v = R*v + t;
        global_ANN->getkNN(dists, idx, v, 1);
        if(!i || dists(0)<_dists(0)) {
          _i=i;  _dists=dists;  _idx=idx;
        }
      }
      if(_dists(0)>cutoff) continue;
      
      proxy = new mlr::Proxy;
      world.proxies.append(proxy);
      proxy->a=global_ANN_shape->index;
      proxy->b=s->index;
      proxy->d = _dists(0);
      proxy->posA.set(&global_ANN_shape->mesh.V(_idx(0), 0));  proxy->posA = global_ANN_shape->X * proxy->posA;
      proxy->posB.set(&s->mesh.V(_i, 0));                      proxy->posB = s->X * proxy->posB;
      proxy->normal = proxy->posA - proxy->posB;
      proxy->normal.normalize();
    }
  }
}

void SwiftInterface::step(mlr::KinematicWorld& world, bool dumpReport) {
  pushToSwift(world);
  pullFromSwift(world, dumpReport);
}

void SwiftInterface::swiftQueryExactDistance() {
  int i, np;
  int *oids;
  SWIFT_Real *dists;
  
  scene->Query_Exact_Distance(false, SWIFT_INFINITY, np, &oids, &dists);
  
  cout <<"exact distances: np=" <<np <<endl;
  for(i=0; i<np; i++) {
    cout <<"    Object " <<oids[i <<1] <<" vs. Object "
         <<oids[(i <<1)+1] <<" = " <<dists[i] <<endl;
  }
}

#else
#include <Core/util.h>
  void SwiftInterface::step(mlr::KinematicWorld &world, bool dumpReport=false){}
  void SwiftInterface::pushToSwift() {}
  void SwiftInterface::pullFromSwift(const KinematicWorld &world, bool dumpReport) {}

  void SwiftInterface::reinitShape(const mlr::Shape *s) {}
//  void close();
  void SwiftInterface::deactivate(mlr::Shape *s1, mlr::Shape *s2) {}
  void SwiftInterface::deactivate(const mlr::Array<mlr::Shape*>& shapes) {}
  void SwiftInterface::deactivate(const mlr::Array<mlr::Body*>& bodies) {}
  void SwiftInterface::initActivations(const KinematicWorld &world, uint parentLevelsToDeactivate=3) {}
  void SwiftInterface::swiftQueryExactDistance() {}
#endif
/** @} */
/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#include "taskMap_transition.h"
#include "taskMap_qItself.h"

TaskMap_Transition::TaskMap_Transition(const mlr::KinematicWorld& G, bool effectiveJointsOnly)
  : effectiveJointsOnly(effectiveJointsOnly){
  posCoeff = mlr::getParameter<double>("Motion/TaskMapTransition/posCoeff",.0);
  velCoeff = mlr::getParameter<double>("Motion/TaskMapTransition/velCoeff",.0);
  accCoeff = mlr::getParameter<double>("Motion/TaskMapTransition/accCoeff",1.);

  //transition cost metric
  H_rate = mlr::getParameter<double>("Hrate", 1.);
  arr H_diag;
  if(mlr::checkParameter<arr>("Hdiag")) {
    H_diag = mlr::getParameter<arr>("Hdiag");
  } else {
    H_diag = G.getHmetric(); //G.naturalQmetric();
  }
  H_rate_diag = H_rate*H_diag;
}

uint TaskMap_Transition::dim_phi(const WorldL& G, int t){
  bool handleSwitches=effectiveJointsOnly;
  uint qN=G(0)->q.N;
  for(uint i=0;i<G.N;i++) if(G.elem(i)->q.N!=qN){ handleSwitches=true; break; }
//  handleSwitches=true;

  if(!handleSwitches){
    return G.last()->getJointStateDimension();
  }else{
//    for(uint i=0;i<G.N;i++) cout <<i <<' ' <<G(i)->joints.N <<' ' <<G(i)->q.N <<' ' <<G(i)->getJointStateDimension() <<endl;
    mlr::Array<mlr::Joint*> matchingJoints = getMatchingJoints(G.sub(-1-order,-1), effectiveJointsOnly);
    uint ydim=0;
    for(uint i=0;i<matchingJoints.d0;i++){
//      cout <<i <<' ' <<matchingJoints(i,0)->qIndex <<' ' <<matchingJoints(i,0)->qDim() <<' ' <<matchingJoints(i,0)->name <<endl;
      ydim += matchingJoints(i,0)->qDim();
    }
    return ydim;
  }
  return uint(-1);
}

void TaskMap_Transition::phi(arr& y, arr& J, const WorldL& G, double tau, int t){
  if(G.last()->q_agent!=0){ //we're referring to a graph set to non-zero agent!
    HALT("what is this code about? why is the q_agent involved here?");
    uint n=G.last()->getJointStateDimension();
    CHECK(n!=H_rate_diag.N,"just checking...");
    y.resize(n).setZero();
    if(&J) J.resize(y.N, order+1, n).setZero();
    return;
  }

  bool handleSwitches=effectiveJointsOnly;
  uint qN=G(0)->q.N;
  for(uint i=0;i<G.N;i++) if(G(i)->q.N!=qN){ handleSwitches=true; break; }
//  handleSwitches=true;

  if(!handleSwitches){ //simple implementation
    //-- transition costs
    double h = H_rate*sqrt(tau), tau2=tau*tau;
//    arr h = sqrt(H_rate_diag)*sqrt(tau);
    y.resize(G.last()->q.N).setZero();
    if(order==1) velCoeff = 1.;
    if(order>=0 && posCoeff) y +=  posCoeff      *(G.elem(-1)->q); //penalize position
#if 0
    if(order>=1 && velCoeff) y += (velCoeff/tau) *(G.elem(-2)->q - G.elem(-1)->q); //penalize velocity
    if(order>=2 && accCoeff) y += (accCoeff/tau2)*(G.elem(-3)->q - 2.*G.elem(-2)->q + G.elem(-1)->q); //penalize acceleration
#else //EQUIVALENT, but profiled - optimized for speed
    if(order>=1 && velCoeff){
      arr v=G.elem(-2)->q;
      v -= G.elem(-1)->q;
      v *= (velCoeff/tau);
      y += v;
    }
    if(order>=2 && accCoeff){
      arr a=G.elem(-2)->q;
      a *= -2.;
      a += G.elem(-3)->q;
      a += G.elem(-1)->q;
      a *= (accCoeff/tau2);
      y += a;
    }
#endif
    if(order>=3) NIY; //  y = (x_bar[3]-3.*x_bar[2]+3.*x_bar[1]-x_bar[0])/tau3; //penalize jerk

    //multiply with h...
    for(mlr::Joint *j:G.last()->joints) for(uint i=0;i<j->qDim();i++)
      y(j->qIndex+i) *= h*j->H;

    if(&J) {
      uint n = G.last()->q.N;
      J.resize(y.N, G.N, n).setZero();
      for(uint i=0;i<n;i++){
        if(order>=0 && posCoeff){ J(i,G.N-1-0,i) += posCoeff; }
        if(order>=1 && velCoeff){ J(i,G.N-1-1,i) += velCoeff/tau;  J(i,G.N-1-0,i) += -velCoeff/tau; }
#if 0
        if(order>=2 && accCoeff){ J(i,G.N-1-2,i) += accCoeff/tau2; J(i,G.N-1-1,i) += -2.*accCoeff/tau2;  J(i,G.N-1-0,i) += accCoeff/tau2; }
#else //EQUIVALENT, but profiled - optimized for speed
        if(order>=2 && accCoeff){
          uint j = i*J.d1*J.d2 + i;
          J.elem(j+(G.N-3)*J.d2) += accCoeff/tau2;
          J.elem(j+(G.N-2)*J.d2) += -2.*accCoeff/tau2;
          J.elem(j+(G.N-1)*J.d2) += accCoeff/tau2;
        }
#endif
        //      if(order>=3){ J(i,3,i) = 1.;  J(i,2,i) = -3.;  J(i,1,i) = +3.;  J(i,0,i) = -1.; }
      }
      J.reshape(y.N, G.N*n);
      for(mlr::Joint *j:G.last()->joints) for(uint i=0;i<j->qDim();i++){
#if 0
        J[j->qIndex+i] *= h*j->H;
#else //EQUIVALENT, but profiled - optimized for speed
        uint l = (j->qIndex+i)*J.d1;
        for(uint k=0;k<J.d1;k++) J.elem(l+k) *= h*j->H;
#endif
      }
    }
  }else{ //with switches
    mlr::Array<mlr::Joint*> matchingJoints = getMatchingJoints(G.sub(-1-order,-1), effectiveJointsOnly);
    double h = H_rate*sqrt(tau), tau2=tau*tau;

//    getSwitchedJoints(*G.elem(-2), *G.elem(-1), true);

    uint ydim=0;
    uintA qidx(G.N);
    for(uint i=0;i<matchingJoints.d0;i++) ydim += matchingJoints(i,0)->qDim();
    y.resize(ydim).setZero();
    if(&J) {
      qidx(0)=0;
      for(uint i=1;i<G.N;i++) qidx(i) = qidx(i-1)+G(i-1)->q.N;
      J.resize(ydim, qidx.last()+G.last()->q.N).setZero();
    }

    uint m=0;
    for(uint i=0;i<matchingJoints.d0;i++){
      mlr::Array<mlr::Joint*> joints = matchingJoints[i];
      uint jdim = joints(0)->qDim(), qi1=0, qi2=0, qi3=0;
      for(uint j=0;j<jdim;j++){
        if(order>=0) qi1 = joints.elem(-1)->qIndex+j;
        if(order>=1) qi2 = joints.elem(-2)->qIndex+j;
        if(order>=2 && accCoeff) qi3 = joints.elem(-3)->qIndex+j;
        double hj = h * joints.last()->H;
        //TODO: adding vels + accs before squareing does not make much sense?
        if(order>=0 && posCoeff) y(m) += posCoeff*hj       * (G.elem(-1)->q(qi1));
        if(order>=1 && velCoeff) y(m) += (velCoeff*hj/tau) * (G.elem(-1)->q(qi1) -    G.elem(-2)->q(qi2));
        if(order>=2 && accCoeff) y(m) += (accCoeff*hj/tau2)* (G.elem(-1)->q(qi1) - 2.*G.elem(-2)->q(qi2) + G.elem(-3)->q(qi3));
        if(&J){
          if(order>=0 && posCoeff){ J(m, qidx.elem(-1)+qi1) += posCoeff*hj; }
          if(order>=1 && velCoeff){ J(m, qidx.elem(-1)+qi1) += velCoeff*hj/tau;  J(m, qidx.elem(-2)+qi2) += -velCoeff*hj/tau; }
          if(order>=2 && accCoeff){ J(m, qidx.elem(-1)+qi1) += accCoeff*hj/tau2; J(m, qidx.elem(-2)+qi2) += -2.*accCoeff*hj/tau2; J(m, qidx.elem(-3)+qi3) += accCoeff*hj/tau2; }
        }
        m++;
      }
    }
    CHECK(m==ydim,"");
  }
}

/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#include "taskMap_QuaternionNorms.h"
#include "taskMap_qItself.h"


void TaskMap_QuaternionNorms::phi(arr &y, arr &J, const mlr::KinematicWorld &G, int t){
    uint n=dim_phi(G);
    y.resize(n);
    if(&J) J.resize(n, G.q.N).setZero();
    uint i=0;
    for(const mlr::Joint* j:G.joints) if(j->type==mlr::JT_quatBall || j->type==mlr::JT_free){
        arr q;
        if(j->type==mlr::JT_quatBall) q.referToRange(G.q, j->qIndex, j->qIndex+3);
        if(j->type==mlr::JT_free)     q.referToRange(G.q, j->qIndex+3, j->qIndex+6);
        double norm = sumOfSqr(q);
        y(i) = norm - 1.;

        if(&J){
            if(j->type==mlr::JT_quatBall) J(i, {j->qIndex, j->qIndex+3}) = 2.*q;
            if(j->type==mlr::JT_free)     J(i, {j->qIndex+3, j->qIndex+6}) = 2.*q;
        }
        i++;
    }
}

uint TaskMap_QuaternionNorms::dim_phi(const mlr::KinematicWorld &G){
    uint i=0;
    for(const mlr::Joint* j:G.joints){
        if(j->type==mlr::JT_quatBall || j->type==mlr::JT_free) i++;
    }
    return i;
}
/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */

/**
 * @file
 * @ingroup group_ors
 */
/**
 * @ingroup group_ors
 * @{
 */


#ifdef MLR_QHULL

#include "mesh.h"

extern "C" {
  #include <qhull/qhull_a.h>
}
#undef dX
#undef dY
#undef dZ
#undef dW

int QHULL_DEBUG_LEVEL=0;

static Mutex qhullMutex;

//===========================================================================

const char* qhullVersion() {
  return qh_version;
}

//===========================================================================

void getQhullState(uint D, arr& points, arr& vertices, arr& lines) {
  uint i;
  double *point, *pointtemp;
  vertexT *vertex, **vertexp;
  facetT *facet;
  
//  plotOpengl();
//  plotClear();
  
  cout <<"\n** points:";
  FORALLpoints {
    points.setCarray(point, D);
    cout <<"\n  " <<points;
//    plotPoints(x);
  }
  
  cout <<"\n** vertices:";
  FORALLvertices {
    vertices.setCarray(vertex->point, D);
    i = (vertex->point - (qh first_point))/D;
    cout <<"\n  " <<vertex->id <<"(" <<i <<")" <<":" <<points;
  }
  
  cout <<"\n** facets:";
  arr x;
  FORALLfacets {
    cout <<"\n  " <<facet->id <<":";
    lines.clear();
    FOREACHvertex_(facet->vertices) {
      cout <<' ' <<vertex->id;
      x.setCarray(vertex->point, D);
      lines.append(x);
    }
    x.setCarray(((vertexT*)(facet->vertices->e[0].p))->point, D);
    lines.append(x);
    lines.reshape(lines.N/D, D);
//    plotLine(line);
  }
  cout <<endl;
}

//===========================================================================

double distanceToConvexHull(const arr &X, const arr &y, arr *projectedPoint, uintA *faceVertices, bool freeqhull) {
  auto lock = qhullMutex();

  int exitcode;
  //static const char* cmd = "qhull Tv i p";
  static char* cmd = (char*) "qhull ";
  exitcode = qh_new_qhull(X.d1, X.d0, X.p, false, cmd, NULL, stderr);
  if(exitcode) HALT("qh_new_qhull error - exitcode " <<exitcode);
  
  uint i;
  facetT *bestfacet;
  double bestdist;
  boolT isoutside;
  int totpart;
  
  bestfacet = qh_findbest(y.p, qh facet_list,
                          !qh_ALL, !qh_ISnewfacets, !qh_ALL,
                          &bestdist, &isoutside, &totpart);
                          
  /*alternatives??
  //qh_findbestfacet(origin0, qh_ALL, &bestdist, &isoutside);
  
  //bestfacet= qh_findbest (origin0, qh facet_list,
   //  qh_ALL, !qh_ISnewfacets, qh_ALL , // qh_NOupper
  //        &bestdist, &isoutside, &totpart);
  */
  
  CHECK(length(y)>1e-10 || fabs(bestdist-bestfacet->offset)<1e-10, "inconsistent!");
  CHECK((isoutside && bestdist>-1e-10) || (!isoutside && bestdist<1e-10), "");
  
  if(projectedPoint) {
    *projectedPoint=y;
    double *normal=bestfacet->normal;
    for(i=X.d1; i--;)(*projectedPoint)(i) -= bestdist * normal[i];
  }
  
  if(faceVertices) {
    faceVertices->clear();
    vertexT *vertex, **vertexp;
    FOREACHvertex_(bestfacet->vertices) {
      i = (vertex->point - (qh first_point))/X.d1;
      faceVertices->append(i);
    }
  }
  
//  if(QHULL_DEBUG_LEVEL>1) {
//    arr line;
//    NIY;
////    plotQhullState(X.d1);
////    plotPoints(y);
//    if(projectedPoint) {
//      line.clear();
//      line.append(y);
//      line.append(*projectedPoint);
////      plotPoints(*projectedPoint);
//      line.reshape(2, X.d1);
////      plotLine(line);
//    }
//    plot();
    
//    //cout <<"**best facet: " <<bestfacet->id <<endl;
//    //FOREACHvertex_(facet->vertices) cout <<vertex->id <<' ';
//  }
  
  if(freeqhull) {
    qh_freeqhull(!qh_ALL);
    int curlong, totlong;
    qh_memfreeshort(&curlong, &totlong);
    if(curlong || totlong)
      MLR_MSG("qhull internal warning (main): did not free " <<totlong <<" bytes of long memory (" <<curlong <<" pieces)\n");
  }
  
  return bestdist;
}

//===========================================================================

void makeNormal(arr& a, const arr& b) { a -= b * scalarProduct(a, b)/sumOfSqr(b); }

double distanceToConvexHullGradient(arr& dDdX, const arr &X, const arr &y, bool freeqhull) {
  arr p;
  uintA vertices;
  double d;
  
  d=distanceToConvexHull(X, y, &p, &vertices, freeqhull);
  
  dDdX.resizeAs(X);
  dDdX.setZero();
  
  uint i, j, k, l;
  arr v, f, w, v_f, y_f, dv, subn, wk, W;
  double dd;
  for(i=0; i<vertices.N; i++) {
    v.referToDim(X, vertices(i)); //v is the vertex in question
    
    // subn: normal of the sub-facet opposit to v
    if(i) j=0; else j=1;
    w.referToDim(X, vertices(j)); //take w as origin of local frame
    CHECK(vertices.N>=X.d1, ""); //won't work otherwise..
    W.resize(vertices.N, X.d1);      //compose matrix of basis vectors
    for(k=0, l=0; k<vertices.N; k++) if(k!=i && k!=j) {
        wk.referToDim(X, vertices(k));
        W[l]() = wk-w;
        l++;
      }
    CHECK_EQ(l,vertices.N-2, "");
    W[l]() = v-w;
    W[l+1]() = p-y; //not important (is already orthogonal to the full facet)
    mlr::Array<double*> tmp;
    qh_gram_schmidt(X.d1, W.getCarray(tmp)); //orthogonalize local basis vectors
    subn = W[l]; //this entry should now be orthogonal to the sub-facet
    
    //f: axis point: projection of v along p onto the sub-facet (``Dreisatz'')
    double alpha = scalarProduct(w-v, subn)/scalarProduct(p-v, subn);
    f = v + alpha*(p-v);
    
    v_f = v-f;
    y_f = y-f;
    double yf_vf=scalarProduct(y_f, v_f);
    double yf_vf_norm=yf_vf/sumOfSqr(v_f);
    // check pythagoras
    dd = sumOfSqr(y_f) - yf_vf * yf_vf_norm;
    CHECK(fabs(dd - d*d)<1e-8, "");
    
    //compute gradient
    dv.referToDim(dDdX, vertices(i));
    dv = f - y + yf_vf_norm*v_f;
    dv *= 2.*yf_vf_norm;
    dv *= .5/d;
  }
  
  return d;
}

//===========================================================================

double forceClosure(const arr& C, const arr& Cn, const mlr::Vector& center,
                    double mu, double torqueWeights, arr *dFdC) { //, arr *dFdCn
  CHECK_EQ(C.d0,Cn.d0, "different number of points and normals");
  CHECK_EQ(C.d1,3, "");
  
  uint i, j, S=7;
  mlr::Vector c, n;
  
  arr X;
  if(torqueWeights>0.)  X.resize(C.d0*S, 6);  //store 6d points for convex wrench hull
  else X.resize(C.d0*S, 3);                //store 3d points for convex force hull
  
  arr dXdC;
  if(dFdC) {
    dXdC.resize(X.d0,X.d1, 3);
    dXdC.setZero();
  }
  /*if(dFdCn){
    dXdCn.resize(C.d0*S, 6, 3);
    dXdCn.setZero();
  }*/
  
  for(i=0; i<C.d0; i++) {  //each contact point contributes a friction cone
    c.set(&C(i, 0));                    //contact point
    n.set(&Cn(i, 0));                   //contact normal
    c -= center;
    
    mlr::Quaternion r;
    r.setDiff(Vector_z, n);//rotate cone's z-axis into contact normal n
    
    for(j=0; j<S; j++) {   //each sample, equidistant on a circle
      double angle = j*MLR_2PI/S;
      mlr::Vector f(cos(angle)*mu, sin(angle)*mu, 1.);  //force point sampled from cone
      
      f = r*f;                         //rotate
      mlr::Vector c_f = c^f;
      
      //what about different scales in force vs torque??!!
      if(torqueWeights>=0.) { //forceClosure
        X(i*S+j, 0) = f.x;
        X(i*S+j, 1) = f.y;
        X(i*S+j, 2) = f.z;
      } else { //torqueClosure
        X(i*S+j, 0) = c_f.x;
        X(i*S+j, 1) = c_f.y;
        X(i*S+j, 2) = c_f.z;
      }
      if(torqueWeights>0.) { //both (wrench)
        X(i*S+j, 3) = torqueWeights * c_f.x;
        X(i*S+j, 4) = torqueWeights * c_f.y;
        X(i*S+j, 5) = torqueWeights * c_f.z;
      }
      if(dFdC) {
        dXdC(i*S+j, 3, 0) =  0   ;  dXdC(i*S+j, 3, 1) =  f.z;  dXdC(i*S+j, 3, 2) = -f.y;
        dXdC(i*S+j, 4, 0) = -f.z;  dXdC(i*S+j, 4, 1) =  0   ;  dXdC(i*S+j, 4, 2) =  f.x;
        dXdC(i*S+j, 5, 0) =  f.y;  dXdC(i*S+j, 5, 1) = -f.x;  dXdC(i*S+j, 5, 2) =  0   ;
      }
      /*if(dFdCn){
      HALT("");
      dXdCn(i*S+j, 3, 0) =  0   ; dXdCn(i*S+j, 3, 1) = -c(2); dXdCn(i*S+j, 3, 2) =  c(1);
      dXdCn(i*S+j, 4, 0) =  c(2); dXdCn(i*S+j, 4, 1) =  0   ; dXdCn(i*S+j, 4, 2) = -c(0);
      dXdCn(i*S+j, 5, 0) = -c(1); dXdCn(i*S+j, 5, 1) =  c(0); dXdCn(i*S+j, 5, 2) =  0   ;
      }*/
    }
  }
  
  if(dFdC)  dXdC *= (double)torqueWeights;
  
  double d;
  arr origin(X.d1);
  origin.setZero();
  if(!dFdC) {
    //note: distance to hull is negative if inside the hull
    d = -distanceToConvexHull(X, origin, 0, 0, true);
  } else {
    arr dFdX;
    d = -distanceToConvexHullGradient(dFdX, X, origin, true);
    dFdX *= -1.;
    dFdX.reshape(TUP(C.d0, S, origin.N));
    dXdC.reshape(TUP(C.d0, S, origin.N, 3));
    dFdC->resize(TUP(C.d0, 3));
    tensorEquation(*dFdC, dFdX, TUP(0u, 2u, 3u), dXdC, TUP(0u, 2u, 3u, 1u), 2);
  }
  return d;
}

//===========================================================================

arr getHull(const arr& V, uintA& T) {
  auto lock = qhullMutex();

  int exitcode;
  uint dim=V.d1;
  static char* cmd = (char*) "qhull Qt ";
  exitcode = qh_new_qhull(V.d1, V.d0, V.p, false, cmd, NULL, stderr);
  if(exitcode) HALT("qh_new_qhull error - exitcode " <<exitcode);


  qh_triangulate();

  facetT *facet;
  vertexT *vertex, **vertexp;
  uint f, i, v;

  arr Vnew;
  Vnew.resize(qh num_vertices, dim);
  i=0;
  FORALLvertices {
    vertex->id = i;
    memmove(&Vnew(i, 0), vertex->point,  dim*sizeof(double));
    i++;
  }
  if(&T){ //retrieve also the triangulation
    T.resize(qh num_facets, dim);
    f=0;
    FORALLfacets {
      i=0;
      FOREACHvertex_(facet->vertices) {
        if(i<3) T(f, i)=vertex->id; else MLR_MSG("face " <<f <<" has " <<i <<" vertices" <<endl);
        i++;
      }
      if(facet->toporient) {
        v=T(f, 0);  T(f, 0)=T(f, 1);  T(f, 1)=v;
      }
      f++;
    }
    CHECK_EQ(f,T.d0, "");
  }

  qh_freeqhull(!qh_ALL);
  int curlong, totlong;
  qh_memfreeshort(&curlong, &totlong);
  if(curlong || totlong)
    MLR_MSG("qhull internal warning (main): did not free " <<totlong <<" bytes of long memory (" <<curlong <<" pieces)\n");
    
  return Vnew;
}

void getDelaunayEdges(uintA& E, const arr& V) {
  auto lock = qhullMutex();

  if(V.d0<3) { E.clear(); return; }
  int exitcode;
  static char* cmd = (char*) "qhull d Qbb Qt ";
  exitcode = qh_new_qhull(V.d1, V.d0, V.p, false, cmd, NULL, stderr);
  if(exitcode) HALT("qh_new_qhull error - exitcode " <<exitcode);
  
  facetT *facet;
  vertexT *vertex, **vertexp;
  uint i, j, k, dim=V.d1;
  
  E.clear();
  uint face[dim+1];
  FORALLfacets {
    if(!facet->upperdelaunay) {
      i=0;
      FOREACHvertex_(facet->vertices) face[i++]=qh_pointid(vertex->point);//vertex->id;
      CHECK_EQ(i,dim+1, "strange number of vertices of a facet!");
      for(j=0; j<dim+1; j++) for(k=j+1; k<dim+1; k++) {
          E.append(TUP(face[j], face[k]));
        }
    }
  }
  E.reshape(E.N/2,2);
  
  qh_freeqhull(!qh_ALL);
  int curlong, totlong;
  qh_memfreeshort(&curlong, &totlong);
  if(curlong || totlong)
    MLR_MSG("qhull internal warning (main): did not free " <<totlong <<" bytes of long memory (" <<curlong <<" pieces)\n");
}


//===========================================================================

#ifdef OLD_CODE
#include "graph.h"

/** this calls the delaunay triangulation of the qhull library

    It first deletes all existing edges! Then adds edges according to
    the delaunay triangulation.

    PRECONDITION: It is assumed that the node type #N# can be casted
    into an #doubleA&# with proper dimensionality
    (#dynamic_cast<doubleA& >(N& n)# has to be defined); if
    the node does not have this member, the code won't compile... */
template<class N, class E>
void delaunay(Graph<N, E>& g, uint dim=2) {
  uint i;
  
  g.clear_edges();
  
  doubleA P;
  P.resize(g.N, dim);
  for(i=0; i<g.N; i++) {
    CHECK_EQ(g.nodes(i)->point.N,dim, "point doesn't have expected dim in delaunay");
    P[i]=(doubleA&)(*(g.nodes(i)));
    //P(i, 0)=g.nodes(i)->feat.x;
    //P(i, 1)=g.nodes(i)->feat.y;
    //P(i, 2)=g.nodes(i)->feat.z;
  }
  
  if(!qh_new_qhull(dim, g.N, P.p, false, "qhull d Qbb T0", NULL, stderr)) {
    facetT *facet;
    vertexT *vertex, **vertexp;
    uint *face, k, l;
    face=new uint[dim+1];
    
    FORALLfacets {
      if(!facet->upperdelaunay) {
        uint j=0;
        FOREACHvertex_(facet->vertices) face[j++]=qh_pointid(vertex->point);
        CHECK_EQ(j,dim+1, "strange number of vertices of a facet!");
        for(k=0; k<dim+1; k++) for(l=0; l<dim+1; l++) if(k!=l)
              if(!g.getEdge(g.nodes(face[k]), g.nodes(face[l])))
                g.new_edge(g.nodes(face[k]), g.nodes(face[l]));
        i++;
      }
    }
    
    delete[] face;
  }
  
  int curlong, totlong;
  qh_freeqhull(!qh_ALL);                 //free long memory
  qh_memfreeshort(&curlong, &totlong);   //free short memory and memory allocator
  
  if(curlong || totlong)
    MLR_MSG("qhull did not free " <<totlong <<" bytes of long memory (" <<curlong <<" pieces)");
}

#endif

#else ///MLR_QHULL
#include <Core/util.h>
#include <Core/array.h>
#include "geo.h"
int QHULL_DEBUG_LEVEL=0;
const char* qhullVersion() { return "NONE"; }
void getTriangulatedHull(uintA& T, arr& V) { NICO }
double forceClosure(const arr& C, const arr& Cn, const mlr::Vector& center,
                    double mu, double torqueWeights, arr *dFdC) { NICO }
double distanceToConvexHull(const arr &X, const arr &y, arr *projectedPoint, uintA *faceVertices, bool freeqhull) { NICO }
double distanceToConvexHullGradient(arr& dDdX, const arr &X, const arr &y, bool freeqhull) { NICO }
void getDelaunayEdges(uintA& E, const arr& V) { NICO }
#endif
/** @} */


/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


// (c) Marc Toussaint:
/* This is a modified version of 'RectDist.h' that was shipped with pgp-1.3
 * The modifications replace the data types and structures used. Also RectDist
 * is extended to also return the nearest points on the Rects. Otherwise the
 * algorithm is untouched */

/*************************************************************************\

  Copyright 1999 The University of North Carolina at Chapel Hill.
  All Rights Reserved.

  Permission to use, copy, modify and distribute this software and its
  documentation for educational, research and non-profit purposes, without
  fee, and without a written agreement is hereby granted, provided that the
  above copyright notice and the following three paragraphs appear in all
  copies.

  IN NO EVENT SHALL THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL BE
  LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
  CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE
  USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY
  OF NORTH CAROLINA HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH
  DAMAGES.

  THE UNIVERSITY OF NORTH CAROLINA SPECIFICALLY DISCLAIM ANY
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE
  PROVIDED HEREUNDER IS ON AN "AS IS" BASIS, AND THE UNIVERSITY OF
  NORTH CAROLINA HAS NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT,
  UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

  The authors may be contacted via:

  US Mail:             E. Larsen
                       Department of Computer Science
                       Sitterson Hall, CB #3175
                       University of N. Carolina
                       Chapel Hill, NC 27599-3175

  Phone:               (919)962-1749

  EMail:               geom@cs.unc.edu


\**************************************************************************/

#ifndef PQP_RECTDIST_H
#define PQP_RECTDIST_H

#include <math.h>

#define PQP_REAL double
#define DBG_PRINT(x) //printf(x)

// ClipToRange
//
// clips val between a and b

inline 
void
ClipToRange(PQP_REAL &val, const PQP_REAL &a, const PQP_REAL &b)
{
  if (val < a) val = a;
  else if (val > b) val = b;
}

inline
void
MTxV(PQP_REAL Vr[3], const PQP_REAL M1[9], const PQP_REAL V1[3])
{
  Vr[0] = (M1[0] * V1[0] +
           M1[3] * V1[1] +
           M1[6] * V1[2]);
  Vr[1] = (M1[1] * V1[0] +
           M1[4] * V1[1] +
           M1[7] * V1[2]);
  Vr[2] = (M1[2] * V1[0] +
           M1[5] * V1[1] +
           M1[8] * V1[2]);
}

inline
PQP_REAL
VdotV(const PQP_REAL V1[3], const PQP_REAL V2[3])
{
  return (V1[0]*V2[0] + V1[1]*V2[1] + V1[2]*V2[2]);
}

// SegCoords
//
// finds the parameters t & u corresponding to the two closest points 
// on a pair of line segments 
//
// The first segment is defined as 
//
// Pa + A*t, 0 <= t <= a, 
// 
// where "Pa" is one endpoint of the segment, "A" is a unit vector 
// pointing to the other endpoint, and t is a scalar that produces
// all the points between the two endpoints. Since "A" is a unit
// vector, "a" is the segment's length.
//
// The second segment is 
//
// Pb + B*u, 0 <= u <= b
//
// In my application, many of the terms needed by the algorithm
// are already computed for other purposes, so I pass these terms to 
// the function instead of complete specifications of each segment. 
// "T" in the dot products is the vector between Pa and Pb.
//
// The algorithm is from
//
// Vladimir J. Lumelsky,
// On fast computation of distance between line segments.
// In Information Processing Letters, no. 21, pages 55-61, 1985.   

inline
void 
SegCoords(PQP_REAL& t, PQP_REAL& u,
          const PQP_REAL& a, const PQP_REAL& b,
          const PQP_REAL& A_dot_B,
          const PQP_REAL& A_dot_T,
          const PQP_REAL& B_dot_T)
{  
  PQP_REAL denom = 1 - (A_dot_B)*(A_dot_B);

  if (denom == 0) t = 0;
  else
  {
    t = (A_dot_T - B_dot_T*A_dot_B)/denom;
    ClipToRange(t,0,a);
  }
  
  u = t*A_dot_B - B_dot_T;
  if (u < 0) 
  {
    u = 0;
    t = A_dot_T;
    ClipToRange(t,0,a);
  }
  else if (u > b) 
  {
    u = b;
    t = u*A_dot_B + A_dot_T;
    ClipToRange(t,0,a);
  }
}

// InVoronoi
//
// returns whether the nearest point on rectangle edge 
// Pb + B*u, 0 <= u <= b, to the rectangle edge,
// Pa + A*t, 0 <= t <= a, is within the half space 
// determined by the point Pa and the direction Anorm.
//
// A,B, and Anorm are unit vectors.
// T is the vector between Pa and Pb.

inline
int 
InVoronoi(const PQP_REAL &a,
          const PQP_REAL &b,
          const PQP_REAL &Anorm_dot_B,
          const PQP_REAL &Anorm_dot_T,
          const PQP_REAL &A_dot_B,
          const PQP_REAL &A_dot_T,
          const PQP_REAL &B_dot_T)
{ 
  if (fabs(Anorm_dot_B) < 1e-7) return 0;

  PQP_REAL t, u, v;
 
  u = -Anorm_dot_T / Anorm_dot_B; 
  ClipToRange(u,0,b);
  
  t = u*A_dot_B + A_dot_T; 
  ClipToRange(t,0,a);
  
  v = t*A_dot_B - B_dot_T; 
  
  if (Anorm_dot_B > 0) 
  {
    if (v > (u + 1e-7)) return 1;
  }
  else 
  {
    if (v < (u - 1e-7)) return 1;
  }
  return 0; 
} 


// RectDist
//
// Finds the distance between two rectangles A and B.  A is assumed
// to have its corner on the origin, one side aligned with
// x, the other side aligned with y, and its normal aligned with z.
// 
// [Rab,Tab] gives the orientation and corner position of rectangle B
// 
// a[2] are the side lengths of A, b[2] are the side lengths of B

PQP_REAL
pqp_RectDist(PQP_REAL Rab[9], PQP_REAL Tab[3],
          PQP_REAL a[2], PQP_REAL b[2], PQP_REAL Pa[3], PQP_REAL Pb[3])
{
  PQP_REAL A0_dot_B0, A0_dot_B1, A1_dot_B0, A1_dot_B1;

  A0_dot_B0 = Rab[0];
  A0_dot_B1 = Rab[1];
  A1_dot_B0 = Rab[3];
  A1_dot_B1 = Rab[4];

  PQP_REAL aA0_dot_B0, aA0_dot_B1, aA1_dot_B0, aA1_dot_B1;
  PQP_REAL bA0_dot_B0, bA0_dot_B1, bA1_dot_B0, bA1_dot_B1;
 
  aA0_dot_B0 = a[0]*A0_dot_B0;
  aA0_dot_B1 = a[0]*A0_dot_B1;
  aA1_dot_B0 = a[1]*A1_dot_B0;
  aA1_dot_B1 = a[1]*A1_dot_B1;
  bA0_dot_B0 = b[0]*A0_dot_B0;
  bA1_dot_B0 = b[0]*A1_dot_B0;
  bA0_dot_B1 = b[1]*A0_dot_B1;
  bA1_dot_B1 = b[1]*A1_dot_B1;

  PQP_REAL Tba[3];
  MTxV(Tba,Rab,Tab);

  PQP_REAL S[3], t, u;

  // determine if any edge pair contains the closest points

  PQP_REAL ALL_x, ALU_x, AUL_x, AUU_x;
  PQP_REAL BLL_x, BLU_x, BUL_x, BUU_x;
  PQP_REAL LA1_lx, LA1_ux, UA1_lx, UA1_ux, LB1_lx, LB1_ux, UB1_lx, UB1_ux;

  ALL_x = -Tba[0];
  ALU_x = ALL_x + aA1_dot_B0;
  AUL_x = ALL_x + aA0_dot_B0;
  AUU_x = ALU_x + aA0_dot_B0;

  if (ALL_x < ALU_x)
  { 
    LA1_lx = ALL_x;
    LA1_ux = ALU_x;
    UA1_lx = AUL_x;    
    UA1_ux = AUU_x;
  }
  else
  { 
    LA1_lx = ALU_x;
    LA1_ux = ALL_x;
    UA1_lx = AUU_x;    
    UA1_ux = AUL_x;
  }

  BLL_x = Tab[0];
  BLU_x = BLL_x + bA0_dot_B1;
  BUL_x = BLL_x + bA0_dot_B0;
  BUU_x = BLU_x + bA0_dot_B0;
  
  if (BLL_x < BLU_x)
  { 
    LB1_lx = BLL_x;
    LB1_ux = BLU_x;
    UB1_lx = BUL_x;    
    UB1_ux = BUU_x;
  }
  else
  { 
    LB1_lx = BLU_x;
    LB1_ux = BLL_x;
    UB1_lx = BUU_x;    
    UB1_ux = BUL_x;
  }

  // UA1, UB1
  
  if ((UA1_ux > b[0]) && (UB1_ux > a[0]))
  {
    if (((UA1_lx > b[0]) || 
          InVoronoi(b[1],a[1],A1_dot_B0,aA0_dot_B0 - b[0] - Tba[0],
                                A1_dot_B1, aA0_dot_B1 - Tba[1],
                    -Tab[1] - bA1_dot_B0))
        &&
	
        ((UB1_lx > a[0]) || 
          InVoronoi(a[1],b[1],A0_dot_B1,Tab[0] + bA0_dot_B0 - a[0],
                    A1_dot_B1,Tab[1] + bA1_dot_B0,Tba[1] - aA0_dot_B1)))
    {
      SegCoords(t,u,a[1],b[1],A1_dot_B1,Tab[1] + bA1_dot_B0,
                Tba[1] - aA0_dot_B1);
      
      Pb[0] = Tab[0] + Rab[0]*b[0] + Rab[1]*u;  Pa[0] = a[0];
      Pb[1] = Tab[1] + Rab[3]*b[0] + Rab[4]*u;  Pa[1] = t;
      Pb[2] = Tab[2] + Rab[6]*b[0] + Rab[7]*u;  Pa[2] = 0.;
      S[0] = Pb[0]-Pa[0];  S[1] = Pb[1]-Pa[1];  S[2] = Pb[2]-Pa[2];
      DBG_PRINT("case 1"); return sqrt(VdotV(S,S));
    }    
  }


  // UA1, LB1

  if ((UA1_lx < 0) && (LB1_ux > a[0]))
  {
    if (((UA1_ux < 0) ||
          InVoronoi(b[1],a[1],-A1_dot_B0,Tba[0] - aA0_dot_B0,
                    A1_dot_B1, aA0_dot_B1 - Tba[1], -Tab[1]))
        &&

        ((LB1_lx > a[0]) ||
          InVoronoi(a[1],b[1],A0_dot_B1,Tab[0] - a[0],
                    A1_dot_B1,Tab[1],Tba[1] - aA0_dot_B1)))
    {
      SegCoords(t,u,a[1],b[1],A1_dot_B1,Tab[1],Tba[1] - aA0_dot_B1);

      Pb[0] = Tab[0] + Rab[1]*u;  Pa[0] = a[0];
      Pb[1] = Tab[1] + Rab[4]*u;  Pa[1] = t;
      Pb[2] = Tab[2] + Rab[7]*u;  Pa[2] = 0.;
      S[0] = Pb[0]-Pa[0];  S[1] = Pb[1]-Pa[1];  S[2] = Pb[2]-Pa[2];
      DBG_PRINT("case 2"); return sqrt(VdotV(S,S));
    }
  }

  // LA1, UB1

  if ((LA1_ux > b[0]) && (UB1_lx < 0))
  {
    if (((LA1_lx > b[0]) || 
          InVoronoi(b[1],a[1],A1_dot_B0,-Tba[0] - b[0],
                    A1_dot_B1,-Tba[1], -Tab[1] - bA1_dot_B0))
          &&
	
	((UB1_ux < 0) ||
          InVoronoi(a[1],b[1],-A0_dot_B1, -Tab[0] - bA0_dot_B0,
                    A1_dot_B1, Tab[1] + bA1_dot_B0,Tba[1])))
    {

      SegCoords(t,u,a[1],b[1],A1_dot_B1,Tab[1] + bA1_dot_B0,Tba[1]);

      Pb[0] = Tab[0] + Rab[0]*b[0] + Rab[1]*u;  Pa[0] = 0.;
      Pb[1] = Tab[1] + Rab[3]*b[0] + Rab[4]*u;  Pa[1] = t;
      Pb[2] = Tab[2] + Rab[6]*b[0] + Rab[7]*u;  Pa[2] = 0.;
      S[0] = Pb[0]-Pa[0];  S[1] = Pb[1]-Pa[1];  S[2] = Pb[2]-Pa[2];
      DBG_PRINT("case 3"); return sqrt(VdotV(S,S));
    }
  }

  // LA1, LB1

  if ((LA1_lx < 0) && (LB1_lx < 0))
  {   
    if (((LA1_ux < 0) ||
          InVoronoi(b[1],a[1],-A1_dot_B0,Tba[0],A1_dot_B1,
                    -Tba[1],-Tab[1]))
          &&

        ((LB1_ux < 0) ||
          InVoronoi(a[1],b[1],-A0_dot_B1,-Tab[0],A1_dot_B1,
                    Tab[1], Tba[1])))
    {
      SegCoords(t,u,a[1],b[1],A1_dot_B1,Tab[1],Tba[1]);    

      Pb[0] = Tab[0] + Rab[1]*u;  Pa[0] = 0.;
      Pb[1] = Tab[1] + Rab[4]*u;  Pa[1] = t;
      Pb[2] = Tab[2] + Rab[7]*u;  Pa[2] = 0.;
      S[0] = Pb[0]-Pa[0];  S[1] = Pb[1]-Pa[1];  S[2] = Pb[2]-Pa[2];
      DBG_PRINT("case 4"); return sqrt(VdotV(S,S));
    }
  }

  PQP_REAL ALL_y, ALU_y, AUL_y, AUU_y;

  ALL_y = -Tba[1];
  ALU_y = ALL_y + aA1_dot_B1;
  AUL_y = ALL_y + aA0_dot_B1;
  AUU_y = ALU_y + aA0_dot_B1;
  
  PQP_REAL LA1_ly, LA1_uy, UA1_ly, UA1_uy, LB0_lx, LB0_ux, UB0_lx, UB0_ux;

  if (ALL_y < ALU_y)
  { 
    LA1_ly = ALL_y;
    LA1_uy = ALU_y;
    UA1_ly = AUL_y;    
    UA1_uy = AUU_y;
  }
  else
  { 
    LA1_ly = ALU_y;
    LA1_uy = ALL_y;
    UA1_ly = AUU_y;    
    UA1_uy = AUL_y;
  }

  if (BLL_x < BUL_x)
  {
    LB0_lx = BLL_x;
    LB0_ux = BUL_x;
    UB0_lx = BLU_x;
    UB0_ux = BUU_x;
  }
  else
  {
    LB0_lx = BUL_x;
    LB0_ux = BLL_x;
    UB0_lx = BUU_x;
    UB0_ux = BLU_x;
  }

  // UA1, UB0

  if ((UA1_uy > b[1]) && (UB0_ux > a[0]))
  {   
    if (((UA1_ly > b[1]) || 
          InVoronoi(b[0],a[1],A1_dot_B1, aA0_dot_B1 - Tba[1] - b[1],
                    A1_dot_B0, aA0_dot_B0 - Tba[0], -Tab[1] - bA1_dot_B1))
          &&
	
        ((UB0_lx > a[0]) || 
          InVoronoi(a[1],b[0],A0_dot_B0, Tab[0] - a[0] + bA0_dot_B1,
                    A1_dot_B0, Tab[1] + bA1_dot_B1, Tba[0] - aA0_dot_B0)))
    {
      SegCoords(t,u,a[1],b[0],A1_dot_B0,Tab[1] + bA1_dot_B1,
                Tba[0] - aA0_dot_B0);

      Pb[0] = Tab[0] + Rab[1]*b[1] + Rab[0]*u;  Pa[0] = a[0] ;
      Pb[1] = Tab[1] + Rab[4]*b[1] + Rab[3]*u;  Pa[1] = t;
      Pb[2] = Tab[2] + Rab[7]*b[1] + Rab[6]*u;  Pa[2] = 0.;
      S[0] = Pb[0]-Pa[0];  S[1] = Pb[1]-Pa[1];  S[2] = Pb[2]-Pa[2];
      DBG_PRINT("case 5"); return sqrt(VdotV(S,S));
    }
  }

  // UA1, LB0

  if ((UA1_ly < 0) && (LB0_ux > a[0]))
  {
    if (((UA1_uy < 0) ||
          InVoronoi(b[0],a[1],-A1_dot_B1, Tba[1] - aA0_dot_B1,A1_dot_B0,
                    aA0_dot_B0 - Tba[0], -Tab[1]))
          &&

        ((LB0_lx > a[0]) || 
          InVoronoi(a[1],b[0],A0_dot_B0,Tab[0] - a[0],
                    A1_dot_B0,Tab[1],Tba[0] - aA0_dot_B0)))
    {
      SegCoords(t,u,a[1],b[0],A1_dot_B0,Tab[1],Tba[0] - aA0_dot_B0);

      Pb[0] = Tab[0] + Rab[0]*u;  Pa[0] = a[0];
      Pb[1] = Tab[1] + Rab[3]*u;  Pa[1] = t;
      Pb[2] = Tab[2] + Rab[6]*u;  Pa[2] = 0.;
      S[0] = Pb[0]-Pa[0];  S[1] = Pb[1]-Pa[1];  S[2] = Pb[2]-Pa[2];
      DBG_PRINT("case 6"); return sqrt(VdotV(S,S));
    }
  }

  // LA1, UB0

  if ((LA1_uy > b[1]) && (UB0_lx < 0))
  {
    if (((LA1_ly > b[1]) || 
        InVoronoi(b[0],a[1],A1_dot_B1,-Tba[1] - b[1],
                  A1_dot_B0, -Tba[0], -Tab[1] - bA1_dot_B1))     
        &&

        ((UB0_ux < 0) ||
          InVoronoi(a[1],b[0],-A0_dot_B0, -Tab[0] - bA0_dot_B1,A1_dot_B0,
                    Tab[1] + bA1_dot_B1,Tba[0])))
    {
      SegCoords(t,u,a[1],b[0],A1_dot_B0,Tab[1] + bA1_dot_B1,Tba[0]);

      Pb[0] = Tab[0] + Rab[1]*b[1] + Rab[0]*u;  Pa[0] = 0.;
      Pb[1] = Tab[1] + Rab[4]*b[1] + Rab[3]*u;  Pa[1] = t;
      Pb[2] = Tab[2] + Rab[7]*b[1] + Rab[6]*u;  Pa[2] = 0.;
      S[0] = Pb[0]-Pa[0];  S[1] = Pb[1]-Pa[1];  S[2] = Pb[2]-Pa[2];
      DBG_PRINT("case 7"); return sqrt(VdotV(S,S));
    }
  }

  // LA1, LB0

  if ((LA1_ly < 0) && (LB0_lx < 0))
  {
    if (((LA1_uy < 0) ||
          InVoronoi(b[0],a[1],-A1_dot_B1,Tba[1],A1_dot_B0,
                    -Tba[0],-Tab[1]))
        && 

        ((LB0_ux < 0) ||
          InVoronoi(a[1],b[0],-A0_dot_B0,-Tab[0],A1_dot_B0,
                    Tab[1],Tba[0])))
    {
      SegCoords(t,u,a[1],b[0],A1_dot_B0,Tab[1],Tba[0]);
	
      Pb[0] = Tab[0] + Rab[0]*u;  Pa[0] = 0.;
      Pb[1] = Tab[1] + Rab[3]*u;  Pa[1] = t;
      Pb[2] = Tab[2] + Rab[6]*u;  Pa[2] = 0.;
      S[0] = Pb[0]-Pa[0];  S[1] = Pb[1]-Pa[1];  S[2] = Pb[2]-Pa[2];
      DBG_PRINT("case 8"); return sqrt(VdotV(S,S));
    }
  }

  PQP_REAL BLL_y, BLU_y, BUL_y, BUU_y;

  BLL_y = Tab[1];
  BLU_y = BLL_y + bA1_dot_B1;
  BUL_y = BLL_y + bA1_dot_B0;
  BUU_y = BLU_y + bA1_dot_B0;

  PQP_REAL LA0_lx, LA0_ux, UA0_lx, UA0_ux, LB1_ly, LB1_uy, UB1_ly, UB1_uy;

  if (ALL_x < AUL_x)
  {
    LA0_lx = ALL_x;
    LA0_ux = AUL_x;
    UA0_lx = ALU_x;
    UA0_ux = AUU_x;
  }
  else
  {
    LA0_lx = AUL_x;
    LA0_ux = ALL_x;
    UA0_lx = AUU_x;
    UA0_ux = ALU_x;
  }

  if (BLL_y < BLU_y)
  {
    LB1_ly = BLL_y;
    LB1_uy = BLU_y;
    UB1_ly = BUL_y;
    UB1_uy = BUU_y;
  }
  else
  {
    LB1_ly = BLU_y;
    LB1_uy = BLL_y;
    UB1_ly = BUU_y;
    UB1_uy = BUL_y;
  }
    
  // UA0, UB1
  
  if ((UA0_ux > b[0]) && (UB1_uy > a[1]))
  {
    if (((UA0_lx > b[0]) || 
          InVoronoi(b[1],a[0],A0_dot_B0, aA1_dot_B0 - Tba[0] - b[0],
                    A0_dot_B1,aA1_dot_B1 - Tba[1], -Tab[0] - bA0_dot_B0))
        &&
	
        ((UB1_ly > a[1]) ||
          InVoronoi(a[0],b[1],A1_dot_B1, Tab[1] - a[1] + bA1_dot_B0,
                    A0_dot_B1,Tab[0] + bA0_dot_B0, Tba[1] - aA1_dot_B1)))
    {
      SegCoords(t,u,a[0],b[1],A0_dot_B1,Tab[0] + bA0_dot_B0,
                Tba[1] - aA1_dot_B1);
    
      Pb[0] = Tab[0] + Rab[0]*b[0] + Rab[1]*u;  Pa[0] = t;
      Pb[1] = Tab[1] + Rab[3]*b[0] + Rab[4]*u;  Pa[1] = a[1];
      Pb[2] = Tab[2] + Rab[6]*b[0] + Rab[7]*u;  Pa[2] = 0.;
      S[0] = Pb[0]-Pa[0];  S[1] = Pb[1]-Pa[1];  S[2] = Pb[2]-Pa[2];
      DBG_PRINT("case 9"); return sqrt(VdotV(S,S));
    }
  }

  // UA0, LB1

  if ((UA0_lx < 0) && (LB1_uy > a[1]))
  {
    if (((UA0_ux < 0) ||
          InVoronoi(b[1],a[0],-A0_dot_B0, Tba[0] - aA1_dot_B0,A0_dot_B1,
                    aA1_dot_B1 - Tba[1],-Tab[0]))
        &&

        ((LB1_ly > a[1]) || 
          InVoronoi(a[0],b[1],A1_dot_B1,Tab[1] - a[1],A0_dot_B1,Tab[0],
                    Tba[1] - aA1_dot_B1)))
    {
      SegCoords(t,u,a[0],b[1],A0_dot_B1,Tab[0],Tba[1] - aA1_dot_B1);

      Pb[0] = Tab[0] + Rab[1]*u;  Pa[0] = t;
      Pb[1] = Tab[1] + Rab[4]*u;  Pa[1] = a[1];
      Pb[2] = Tab[2] + Rab[7]*u;  Pa[2] = 0.;
      S[0] = Pb[0]-Pa[0];  S[1] = Pb[1]-Pa[1];  S[2] = Pb[2]-Pa[2];
      DBG_PRINT("case 10"); return sqrt(VdotV(S,S));
    }
  }

  // LA0, UB1

  if ((LA0_ux > b[0]) && (UB1_ly < 0))
  {
    if (((LA0_lx > b[0]) || 
          InVoronoi(b[1],a[0],A0_dot_B0,-b[0] - Tba[0],A0_dot_B1,-Tba[1],
                    -bA0_dot_B0 - Tab[0]))
        &&

        ((UB1_uy < 0) ||
          InVoronoi(a[0],b[1],-A1_dot_B1, -Tab[1] - bA1_dot_B0,A0_dot_B1,
                    Tab[0] + bA0_dot_B0,Tba[1])))
    {
      SegCoords(t,u,a[0],b[1],A0_dot_B1,Tab[0] + bA0_dot_B0,Tba[1]);

      Pb[0] = Tab[0] + Rab[0]*b[0] + Rab[1]*u;  Pa[0] = t;
      Pb[1] = Tab[1] + Rab[3]*b[0] + Rab[4]*u;  Pa[1] = 0.;
      Pb[2] = Tab[2] + Rab[6]*b[0] + Rab[7]*u;  Pa[2] = 0.;
      S[0] = Pb[0]-Pa[0];  S[1] = Pb[1]-Pa[1];  S[2] = Pb[2]-Pa[2];
      DBG_PRINT("case 11"); return sqrt(VdotV(S,S));
    }
  }
  
  // LA0, LB1

  if ((LA0_lx < 0) && (LB1_ly < 0))
  {
    if (((LA0_ux < 0) ||
          InVoronoi(b[1],a[0],-A0_dot_B0,Tba[0],A0_dot_B1,-Tba[1],
                    -Tab[0]))
        &&
	
	((LB1_uy < 0) ||
          InVoronoi(a[0],b[1],-A1_dot_B1,-Tab[1],A0_dot_B1,
                    Tab[0],Tba[1])))
    {
      SegCoords(t,u,a[0],b[1],A0_dot_B1,Tab[0],Tba[1]);

      Pb[0] = Tab[0] + Rab[1]*u;  Pa[0] = t;
      Pb[1] = Tab[1] + Rab[4]*u;  Pa[1] = 0.;
      Pb[2] = Tab[2] + Rab[7]*u;  Pa[2] = 0.;
      S[0] = Pb[0]-Pa[0];  S[1] = Pb[1]-Pa[1];  S[2] = Pb[2]-Pa[2];
      DBG_PRINT("case 12"); return sqrt(VdotV(S,S));
    }
  }

  PQP_REAL LA0_ly, LA0_uy, UA0_ly, UA0_uy, LB0_ly, LB0_uy, UB0_ly, UB0_uy;

  if (ALL_y < AUL_y)
  {
    LA0_ly = ALL_y;
    LA0_uy = AUL_y;
    UA0_ly = ALU_y;
    UA0_uy = AUU_y;
  }
  else
  {
    LA0_ly = AUL_y;
    LA0_uy = ALL_y;
    UA0_ly = AUU_y;
    UA0_uy = ALU_y;
  }

  if (BLL_y < BUL_y)
  {
    LB0_ly = BLL_y;
    LB0_uy = BUL_y;
    UB0_ly = BLU_y;
    UB0_uy = BUU_y;
  }
  else
  {
    LB0_ly = BUL_y;
    LB0_uy = BLL_y;
    UB0_ly = BUU_y;
    UB0_uy = BLU_y;
  }

  // UA0, UB0

  if ((UA0_uy > b[1]) && (UB0_uy > a[1]))
  {
    if (((UA0_ly > b[1]) || 
          InVoronoi(b[0],a[0],A0_dot_B1, aA1_dot_B1 - Tba[1] - b[1],
                    A0_dot_B0, aA1_dot_B0 - Tba[0], -Tab[0] - bA0_dot_B1))
        &&
	
        ((UB0_ly > a[1]) || 
          InVoronoi(a[0],b[0],A1_dot_B0,Tab[1] - a[1] + bA1_dot_B1,A0_dot_B0,
                    Tab[0] + bA0_dot_B1, Tba[0] - aA1_dot_B0)))
    {
      SegCoords(t,u,a[0],b[0],A0_dot_B0,Tab[0] + bA0_dot_B1,
                Tba[0] - aA1_dot_B0);
      
      Pb[0] = Tab[0] + Rab[1]*b[1] + Rab[0]*u;  Pa[0] = t;
      Pb[1] = Tab[1] + Rab[4]*b[1] + Rab[3]*u;  Pa[1] = a[1];
      Pb[2] = Tab[2] + Rab[7]*b[1] + Rab[6]*u;  Pa[2] = 0.;
      S[0] = Pb[0]-Pa[0];  S[1] = Pb[1]-Pa[1];  S[2] = Pb[2]-Pa[2];
      DBG_PRINT("case 13"); return sqrt(VdotV(S,S));
    }
  }

  // UA0, LB0

  if ((UA0_ly < 0) && (LB0_uy > a[1]))
  {
    if (((UA0_uy < 0) ||
          InVoronoi(b[0],a[0],-A0_dot_B1,Tba[1] - aA1_dot_B1,A0_dot_B0,
                    aA1_dot_B0 - Tba[0],-Tab[0]))
        &&      

        ((LB0_ly > a[1]) || 
          InVoronoi(a[0],b[0],A1_dot_B0,Tab[1] - a[1],
                    A0_dot_B0,Tab[0],Tba[0] - aA1_dot_B0)))
    {
      SegCoords(t,u,a[0],b[0],A0_dot_B0,Tab[0],Tba[0] - aA1_dot_B0);

      Pb[0] = Tab[0] + Rab[0]*u;  Pa[0] = t;
      Pb[1] = Tab[1] + Rab[3]*u;  Pa[1] = a[1];
      Pb[2] = Tab[2] + Rab[6]*u;  Pa[2] = 0.;
      S[0] = Pb[0]-Pa[0];  S[1] = Pb[1]-Pa[1];  S[2] = Pb[2]-Pa[2];
      DBG_PRINT("case 14"); return sqrt(VdotV(S,S));
    }
  }

  // LA0, UB0

  if ((LA0_uy > b[1]) && (UB0_ly < 0))
  {  
    if (((LA0_ly > b[1]) ||
          InVoronoi(b[0],a[0],A0_dot_B1,-Tba[1] - b[1], A0_dot_B0,-Tba[0],
                    -Tab[0] - bA0_dot_B1))
        &&
	
	((UB0_uy < 0) ||
          InVoronoi(a[0],b[0],-A1_dot_B0, -Tab[1] - bA1_dot_B1, A0_dot_B0,
                    Tab[0] + bA0_dot_B1,Tba[0])))
    {
      SegCoords(t,u,a[0],b[0],A0_dot_B0,Tab[0] + bA0_dot_B1,Tba[0]);
      
      Pb[0] = Tab[0] + Rab[1]*b[1] + Rab[0]*u;  Pa[0] = t;
      Pb[1] = Tab[1] + Rab[4]*b[1] + Rab[3]*u;  Pa[1] = 0.;
      Pb[2] = Tab[2] + Rab[7]*b[1] + Rab[6]*u;  Pa[2] = 0.;
      S[0] = Pb[0]-Pa[0];  S[1] = Pb[1]-Pa[1];  S[2] = Pb[2]-Pa[2];
      DBG_PRINT("case 15"); return sqrt(VdotV(S,S));
    }
  }

  // LA0, LB0

  if ((LA0_ly < 0) && (LB0_ly < 0))
  {   
    if (((LA0_uy < 0) ||
          InVoronoi(b[0],a[0],-A0_dot_B1,Tba[1],A0_dot_B0,
                    -Tba[0],-Tab[0]))
        &&
	
	((LB0_uy < 0) ||
          InVoronoi(a[0],b[0],-A1_dot_B0,-Tab[1],A0_dot_B0,
                    Tab[0],Tba[0])))
    {
      SegCoords(t,u,a[0],b[0],A0_dot_B0,Tab[0],Tba[0]);

      Pb[0] = Tab[0] + Rab[0]*u;  Pa[0] = t;
      Pb[1] = Tab[1] + Rab[3]*u;  Pa[1] = 0.;
      Pb[2] = Tab[2] + Rab[6]*u;  Pa[2] = 0.;
      S[0] = Pb[0]-Pa[0];  S[1] = Pb[1]-Pa[1];  S[2] = Pb[2]-Pa[2];
      DBG_PRINT("case 16"); return sqrt(VdotV(S,S));
    }
  }

  // no edges passed, take max separation along face normals

  Pa[0]=Pa[1]=Pa[2]=0.;
  Pb[0]=Tab[0];
  Pb[1]=Tab[1];
  Pb[2]=Tab[2];

  PQP_REAL sep1, sep2;

  //corner of B, interior of A
  if (Tab[2] > 0.0) {
    DBG_PRINT("case 17a");
    sep1 = Tab[2];
    if (Rab[6] < 0.0){ u=b[0];  sep1 += u*Rab[6];  Pb[0] += Rab[0]*u;  Pb[1] += Rab[3]*u;  Pb[2] += Rab[6]*u; }
    if (Rab[7] < 0.0){ u=b[1];  sep1 += u*Rab[7];  Pb[0] += Rab[1]*u;  Pb[1] += Rab[4]*u;  Pb[2] += Rab[7]*u; }
  } else {
    DBG_PRINT("case 17b");
    sep1 = -Tab[2];
    if (Rab[6] > 0.0){ u=b[0];  sep1 -= u*Rab[6];  Pb[0] += Rab[0]*u;  Pb[1] += Rab[3]*u;  Pb[2] += Rab[6]*u; }
    if (Rab[7] > 0.0){ u=b[1];  sep1 -= u*Rab[7];  Pb[0] += Rab[1]*u;  Pb[1] += Rab[4]*u;  Pb[2] += Rab[7]*u; }
  }
  
  //corner of A, interior of B
  if (Tba[2] < 0) {
    DBG_PRINT("case 18a");
    sep2 = -Tba[2];
    if (Rab[2] < 0.0){ t=a[0];  sep2 += t*Rab[2];  Pa[0] += t; }
    if (Rab[5] < 0.0){ t=a[1];  sep2 += t*Rab[5];  Pa[1] += t; }
  } else {
    DBG_PRINT("case 18b");
    sep2 = Tba[2];
    if (Rab[2] > 0.0){ t=a[0];  sep2 -= t*Rab[2];  Pa[0] += t; }
    if (Rab[5] > 0.0){ t=a[1];  sep2 -= t*Rab[5];  Pa[1] += t; }
  }

  PQP_REAL sep;
  if(sep1>sep2){ //use 1st case
    DBG_PRINT("case 17");
    sep=sep1;
    Pa[0]=Pb[0]; Pa[1]=Pb[1];
  }else{
    DBG_PRINT("case 18");
    sep=sep2;
    //Pb += ((Pa-Pb)*X)*X + ((Pa-Pb)*Y)*Y; IMPLEMENTED BELOW:
    PQP_REAL dx = (Pa[0]-Pb[0])*Rab[0] + (Pa[1]-Pb[1])*Rab[3] + (Pa[2]-Pb[2])*Rab[6];
    PQP_REAL dy = (Pa[0]-Pb[0])*Rab[1] + (Pa[1]-Pb[1])*Rab[4] + (Pa[2]-Pb[2])*Rab[7];
    Pb[0] += Rab[0]*dx;  Pb[1] += Rab[3]*dx;  Pb[2] += Rab[6]*dx;
    Pb[0] += Rab[1]*dy;  Pb[1] += Rab[4]*dy;  Pb[2] += Rab[7]*dy;
  }
  return (sep > 0? sep : 0);
}

#endif
/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */

#include "mesh.h"
#include "qhull.h"

#include <limits>

#ifdef MLR_extern_ply
#  include "ply/ply.h"
#endif

#ifdef MLR_extern_GJK
extern "C"{
#  include "GJK/gjk.h"
}
#endif

#ifdef MLR_GL
#  include <GL/gl.h>
#endif

bool orsDrawWires=false;

//==============================================================================
//
// Mesh code
//

mlr::Mesh::Mesh()
  : glX(0)
    /*parsing_pos_start(0),
    parsing_pos_end(std::numeric_limits<long>::max())*/{}

void mlr::Mesh::clear() {
  V.clear(); Vn.clear(); T.clear(); Tn.clear(); C.clear(); //strips.clear();
}

void mlr::Mesh::setBox() {
  double verts[24] = {
    -.5, -.5, -.5 ,
    +.5, -.5, -.5 ,
    +.5, +.5, -.5 ,
    -.5, +.5, -.5 ,
    -.5, -.5, +.5 ,
    +.5, -.5, +.5 ,
    +.5, +.5, +.5 ,
    -.5, +.5, +.5
  };
  uint   tris [36] = {
    0, 3, 2, 2, 1, 0, //bottom
    4, 5, 6, 6, 7, 4, //top
    1, 5, 4, 4, 0, 1,
    3, 7, 6, 6, 2, 3,
    2, 6, 5, 5, 1, 2,
    0, 4, 7, 7, 3, 0
  };
  V.setCarray(verts, 24);
  T.setCarray(tris , 36);
  V.reshape(8, 3);
  T.reshape(12, 3);
  //cout <<V <<endl;  for(uint i=0;i<4;i++) cout <<length(V[i]) <<endl;
}

void mlr::Mesh::setTetrahedron() {
  double s2=MLR_SQRT2/3., s6=sqrt(6.)/3.;
  double verts[12] = { 0., 0., 1. , 2.*s2, 0., -1./3., -s2, s6, -1./3., -s2, -s6, -1./3. };
  uint   tris [12] = { 0, 1, 2, 0, 2, 3, 0, 3, 1, 1, 3, 2 };
  V.setCarray(verts, 12);
  T.setCarray(tris , 12);
  V.reshape(4, 3);
  T.reshape(4, 3);
  //cout <<V <<endl;  for(uint i=0;i<4;i++) cout <<length(V[i]) <<endl;
}

void mlr::Mesh::setOctahedron() {
  double verts[18] = {
    1, 0, 0,
    -1, 0, 0,
    0, 1, 0,
    0, -1, 0,
    0, 0, 1,
    0, 0, -1
  };
  uint   tris [24] = {
    4, 0, 2,  4, 2, 1,  4, 1, 3,  4, 3, 0,
    5, 2, 0,  5, 1, 2,  5, 3, 1,  5, 0, 3
  };
  V.setCarray(verts, 18);
  T.setCarray(tris , 24);
  V.reshape(6, 3);
  T.reshape(8, 3);
  //cout <<V <<endl;  for(uint i=0;i<4;i++) cout <<length(V[i]) <<endl;
}

void mlr::Mesh::setDodecahedron() {
  double a = 1/sqrt(3.), b = sqrt((3.-sqrt(5.))/6.), c=sqrt((3.+sqrt(5.))/6.);
  double verts[60] = {
    a, a, a,
    a, a, -a,
    a, -a, a,
    a, -a, -a,
    -a, a, a,
    -a, a, -a,
    -a, -a, a,
    -a, -a, -a,
    b, c, 0,
    -b, c, 0,
    b, -c, 0,
    -b, -c, 0,
    c, 0, b,
    c, 0, -b,
    -c, 0, b,
    -c, 0, -b,
    0, b, c,
    0, -b, c,
    0, b, -c,
    0, -b, -c
  };
  uint tris [108] = {
    0, 8, 9, 0, 9, 4, 0, 4, 16, 0, 12, 13, 0, 13, 1, 0, 1, 8,
    0, 16, 17, 0, 17, 2, 0, 2, 12, 8, 1, 18, 8, 18, 5, 8, 5, 9,
    12, 2, 10, 12, 10, 3, 12, 3, 13, 16, 4, 14, 16, 14, 6, 16, 6, 17,
    9, 5, 15, 9, 15, 14, 9, 14, 4, 6, 11, 10, 6, 10, 2, 6, 2, 17,
    3, 19, 18, 3, 18, 1, 3, 1, 13, 7, 15, 5, 7, 5, 18, 7, 18, 19,
    7, 11, 6, 7, 6, 14, 7, 14, 15, 7, 19, 3, 7, 3, 10, 7, 10, 11
  };
  V.setCarray(verts, 60);
  T.setCarray(tris , 108);
  V.reshape(20, 3);
  T.reshape(36, 3);
}

void mlr::Mesh::setSphere(uint fineness) {
  setOctahedron();
  for(uint k=0; k<fineness; k++) {
    subDivide();
    for(uint i=0; i<V.d0; i++) V[i]() /= length(V[i]);
  }
}

void mlr::Mesh::setHalfSphere(uint fineness) {
  setOctahedron();
  V.resizeCopy(5, 3);
  T.resizeCopy(4, 3);
  for(uint k=0; k<fineness; k++) {
    subDivide();
    for(uint i=0; i<V.d0; i++) V[i]() /= length(V[i]);
  }
}

void mlr::Mesh::setCylinder(double r, double l, uint fineness) {
  uint div = 4 * (1 <<fineness);
  V.resize(2*div+2, 3);
  T.resize(4*div, 3);
  uint i, j;
  double phi;
  for(i=0; i<div; i++) {  //vertices
    phi=MLR_2PI*i/div;
    V(i, 0)=r*::cos(phi);
    V(i, 1)=r*::sin(phi);
    V(i, 2)=.5*l;
    V(i+div, 0)=V(i, 0);
    V(i+div, 1)=V(i, 1);
    V(i+div, 2)=-.5*l;
  }
  V(2*div+0, 0)=V(2*div+0, 1)=.0;  V(2*div+0, 2)=+.5*l; //upper center
  V(2*div+1, 0)=V(2*div+1, 1)=.0;  V(2*div+1, 2)=-.5*l; //lower center
  for(i=0; i<div; i++) {  //triangles
    j=(i+1)%div;
    T(4*i  , 0)=i;
    T(4*i  , 1)=j+div;
    T(4*i  , 2)=j;
    
    T(4*i+2, 0)=i;
    T(4*i+2, 1)=j;
    T(4*i+2, 2)=2*div+0;
    
    T(4*i+1, 0)=i;
    T(4*i+1, 1)=i+div;
    T(4*i+1, 2)=j+div;
    
    T(4*i+3, 0)=j+div;
    T(4*i+3, 1)=i+div;
    T(4*i+3, 2)=2*div+1;
  }
}

void mlr::Mesh::setSSBox(double x_width, double y_width, double z_height, double r, uint fineness){
  CHECK(r>=0. && x_width>=2.*r && y_width>=2.*r && z_height>=2.*r, "width/height includes radius!");
  setSphere(fineness);
  scale(r);
  for(uint i=0;i<V.d0;i++){
    V(i,0) += mlr::sign(V(i,0))*(.5*x_width-r);
    V(i,1) += mlr::sign(V(i,1))*(.5*y_width-r);
    V(i,2) += mlr::sign(V(i,2))*(.5*z_height-r);
  }
}

void mlr::Mesh::setCappedCylinder(double r, double l, uint fineness) {
  uint i;
  setSphere(fineness);
  scale(r);
  for(i=0; i<V.d0; i++) V(i, 2) += .5*mlr::sign(V(i, 2))*l;
}

/** @brief add triangles according to the given grid; grid has to be a 2D
  Array, the elements of which are indices referring to vertices in
  the vertex list (V) */
void mlr::Mesh::setGrid(uint X, uint Y) {
  CHECK(X>1 && Y>1, "grid has to be at least 2x2");
  CHECK_EQ(V.d0,X*Y, "don't have X*Y mesh-vertices to create grid faces");
  uint i, j, k=T.d0;
  T.resizeCopy(k+(Y-1)*2*(X-1), 3);
  for(j=0; j<Y-1; j++) {
    for(i=0; i<X-1; i++) {
      T(k, 0)=j*X+i; T(k, 1)=(j+1)*X+i; T(k, 2)=(j+1)*X+(i+1);
      k++;
      T(k, 0)=j*X+i; T(k, 1)=(j+1)*X+(i+1); T(k, 2)=j*X+(i+1);
      k++;
    }
  }
}

void mlr::Mesh::setRandom(uint vertices){
  V.resize(vertices,3);
  rndUniform(V, -1., 1.);
  makeConvexHull();
}

void mlr::Mesh::subDivide() {
  uint v=V.d0, t=T.d0;
  V.resizeCopy(v+3*t, 3);
  uintA newT(4*t, 3);
  uint a, b, c, i, k, l;
  for(i=0, k=v, l=0; i<t; i++) {
    a=T(i, 0); b=T(i, 1); c=T(i, 2);
    V[k+0]() = (double).5*(V[a] + V[b]);
    V[k+1]() = (double).5*(V[b] + V[c]);
    V[k+2]() = (double).5*(V[c] + V[a]);
    newT(l, 0)=a;   newT(l, 1)=k+0; newT(l, 2)=k+2; l++;
    newT(l, 0)=k+0; newT(l, 1)=b;   newT(l, 2)=k+1; l++;
    newT(l, 0)=k+0; newT(l, 1)=k+1; newT(l, 2)=k+2; l++;
    newT(l, 0)=k+2; newT(l, 1)=k+1; newT(l, 2)=c;   l++;
    k+=3;
  }
  T = newT;
}

void mlr::Mesh::scale(double f) {  V *= f; }

void mlr::Mesh::scale(double sx, double sy, double sz) {
  uint i;
  for(i=0; i<V.d0; i++) {  V(i, 0)*=sx;  V(i, 1)*=sy;  V(i, 2)*=sz;  }
}

void mlr::Mesh::translate(double dx, double dy, double dz) {
  uint i;
  for(i=0; i<V.d0; i++) {  V(i, 0)+=dx;  V(i, 1)+=dy;  V(i, 2)+=dz;  }
}

void mlr::Mesh::translate(const arr& d){
  CHECK_EQ(d.N,3,"");
  translate(d.elem(0), d.elem(1), d.elem(2));
}

void mlr::Mesh::transform(const mlr::Transformation& t){
  t.applyOnPointArray(V);
}

mlr::Vector mlr::Mesh::center() {
  arr Vmean = mean(V);
  for(uint i=0; i<V.d0; i++) V[i]() -= Vmean;
  return Vector(Vmean);
}

void mlr::Mesh::box() {
  double x, X, y, Y, z, Z, m;
  x=X=V(0, 0);
  y=Y=V(0, 1);
  z=Z=V(0, 2);
  for(uint i=0; i<V.d0; i++) {
    if(V(i, 0)<x) x=V(i, 0);
    if(V(i, 0)>X) X=V(i, 0);
    if(V(i, 1)<y) y=V(i, 1);
    if(V(i, 1)>Y) Y=V(i, 1);
    if(V(i, 2)<z) z=V(i, 2);
    if(V(i, 2)>Z) Z=V(i, 2);
  }
  translate(-.5*(x+X), -.5*(y+Y), -.5*(z+Z));
  m=X-x;
  if(Y-y>m) m=Y-y;
  if(Z-z>m) m=Z-z;
  scale(1./m);
}

void mlr::Mesh::addMesh(const Mesh& mesh2) {
  uint n=V.d0, t=T.d0;
  V.append(mesh2.V);
  T.append(mesh2.T);
  for(; t<T.d0; t++) {  T(t, 0)+=n;  T(t, 1)+=n;  T(t, 2)+=n;  }
}

void mlr::Mesh::makeConvexHull() {
  if(!V.N) return;
#if 1
  V = getHull(V, T);
  if(C.nd==2) C = mean(C);
#else
  uintA H = getHullIndices(V, T);
  intA Hinv = consts<int>(-1, V.d0);
  for(uint i=0;i<H.N;i++) Hinv(H(i)) = i;

//  if(C.N==V.N){
//    arr Cnew(H.N, 3);
//    for(uint i=0;i<H.N;i++) Cnew[i] = C[H.elem(i)];
//    C=Cnew;
//  }

  arr Vnew(H.N, 3);
  for(uint i=0;i<H.N;i++) Vnew[i] = V[H.elem(i)];
  V=Vnew;

  for(uint i=0;i<T.d0;i++){
    T(i,0) = Hinv(T(i,0));
    T(i,1) = Hinv(T(i,1));
    T(i,2) = Hinv(T(i,2));
  }

#endif
}

void mlr::Mesh::makeTriangleFan(){
  T.clear();
  for(uint i=1;i+1<V.d0;i++){
    T.append(TUP(0,i,i+1));
    T.append(TUP(0,i+1,i));
  }
  T.reshape(T.N/3,3);
}

void mlr::Mesh::makeLineStrip(){
  T.resize(V.d0, 2);
  T[0] = {V.d0-1, 0};
  for(uint i=1;i<V.d0;i++){
    T[i] = {i-1, i};
  }
}

void mlr::Mesh::setSSCvx(const mlr::Mesh& m, double r, uint fineness){
  Mesh ball;
  ball.setSphere(fineness);
  ball.scale(r);

  clear();
  for(uint i=0;i<m.V.d0;i++){
    ball.translate(m.V(i,0), m.V(i,1), m.V(i,2));
    addMesh(ball);
    ball.translate(-m.V(i,0), -m.V(i,1), -m.V(i,2));
  }
  makeConvexHull();
}


/** @brief calculate the normals of all triangles (Tn) and the average
  normals of the vertices (N); average normals are averaged over
  all adjacent triangles that are in the triangle list or member of
  a strip */
void mlr::Mesh::computeNormals() {
  uint i;
  Vector a, b, c;
  Tn.resize(T.d0, 3);
  Tn.setZero();
  Vn.resize(V.d0, 3);
  Vn.setZero();
  //triangle normals and contributions
  for(i=0; i<T.d0; i++) {
    uint *t=T.p+3*i;
    a.set(V.p+3*t[0]);
    b.set(V.p+3*t[1]);
    c.set(V.p+3*t[2]);

    b-=a; c-=a; a=b^c; if(!a.isZero) a.normalize();
    Tn(i, 0)=a.x;  Tn(i, 1)=a.y;  Tn(i, 2)=a.z;
    Vn(t[0], 0)+=a.x;  Vn(t[0], 1)+=a.y;  Vn(t[0], 2)+=a.z;
    Vn(t[1], 0)+=a.x;  Vn(t[1], 1)+=a.y;  Vn(t[1], 2)+=a.z;
    Vn(t[2], 0)+=a.x;  Vn(t[2], 1)+=a.y;  Vn(t[2], 2)+=a.z;
  }
  Vector d;
  for(i=0; i<Vn.d0; i++) { d.set(&Vn(i, 0)); Vn[i]()/=d.length(); }
}

/** @brief add triangles according to the given grid; grid has to be a 2D
  Array, the elements of which are indices referring to vertices in
  the vertex list (V) */
/*void mlr::Mesh::gridToTriangles(const uintA &grid){
  uint i, j, k=T.d0;
  T.resizeCopy(T.d0+2*(grid.d0-1)*(grid.d1-1), 3);
  for(i=0;i<grid.d0-1;i++) for(j=0;j<grid.d1-1;j++){
    if((i+j)&1){
      T(k, 0)=grid(i+1, j  );
      T(k, 1)=grid(i  , j  );
      T(k, 2)=grid(i  , j+1);
      k++;
      T(k, 0)=grid(i+1, j  );
      T(k, 1)=grid(i  , j+1);
      T(k, 2)=grid(i+1, j+1);
      k++;
    }else{
      T(k, 0)=grid(i+1, j  );
      T(k, 1)=grid(i  , j  );
      T(k, 2)=grid(i+1, j+1);
      k++;
      T(k, 0)=grid(i+1, j+1);
      T(k, 1)=grid(i  , j  );
      T(k, 2)=grid(i  , j+1);
      k++;
    }
  }
}*/

/** @brief add strips according to the given grid (sliced in strips along
  the x-axis (the first index)); grid has to be a 2D Array, the
  elements of which are indices referring to vertices in the vertex
  list (V) */
/*void mlr::Mesh::gridToStrips(const uintA& grid){
  CHECK(grid.d0>1 && grid.d1>1, "grid has to be at least 2x2");
  uint i, j, k=strips.N, l;
  strips.resizeCopy(strips.N+grid.d0-1);
  for(i=0;i<grid.d0-1;i++){
    strips(k).resize(2*grid.d1);
    l=0;
    for(j=0;j<grid.d1;j++){
      strips(k)(l)=grid(i+1, j); l++;
      strips(k)(l)=grid(i  , j); l++;
    }
#if 0 //code to make it less symmetric
      //}else{
    strips(k)(l)=grid(i, 0); l++;
    for(j=0;j<grid.d1;j++){
      strips(k)(l)=grid(i  , j); l++;
      strips(k)(l)=grid(i+1, j); l++;
    }
#endif
    k++;
  }
}*/

/** @brief add strips according to the given grid (sliced in strips along
  the x-axis (the first index)); it is assumed that the vertices in
  the list V linearly correspond to points in the XxY grid */
/*void mlr::Mesh::gridToStrips(uint X, uint Y){
  CHECK(X>1 && Y>1, "grid has to be at least 2x2");
  uint i, j, k=strips.N, l;
  strips.resizeCopy(strips.N+Y-1);
  for(j=0;j<Y-1;j++){
    strips(k).resize(2*X);
    l=0;
    for(i=0;i<X;i++){
      strips(k)(l)=(j+1)*X+i;
      l++;
      strips(k)(l)=    j*X+i;
      l++;
    }
    k++;
  }
}*/

void deleteZeroTriangles(mlr::Mesh& m) {
  uintA newT;
  newT.resizeAs(m.T);
  uint i, j;
  for(i=0, j=0; i<m.T.d0; i++) {
    if(m.T(i, 0)!=m.T(i, 1) && m.T(i, 0)!=m.T(i, 2) && m.T(i, 1)!=m.T(i, 2))
      memmove(&newT(j++, 0), &m.T(i, 0), 3*newT.sizeT);
  }
  newT.resizeCopy(j, 3);
  m.T=newT;
}

void permuteVertices(mlr::Mesh& m, uintA& p) {
  CHECK_EQ(p.N,m.V.d0, "");
  uint i;
  arr x(p.N, 3);
  for(i=0; i<p.N; i++) { x(i, 0)=m.V(p(i), 0); x(i, 1)=m.V(p(i), 1); x(i, 2)=m.V(p(i), 2); }
  m.V=x;
  if(m.Vn.N) {
    for(i=0; i<p.N; i++) { x(i, 0)=m.Vn(p(i), 0); x(i, 1)=m.Vn(p(i), 1); x(i, 2)=m.Vn(p(i), 2); }
    m.Vn=x;
  }
  if(m.C.N) {
    for(i=0; i<p.N; i++) { x(i, 0)=m.C(p(i), 0); x(i, 1)=m.C(p(i), 1); x(i, 2)=m.C(p(i), 2); }
    m.C=x;
  }
  uintA y(m.T.d0, 3);
  uintA p2(p.N); //inverse permutation
  for(i=0; i<p.N; i++) p2(p(i))=i;
  for(i=0; i<m.T.d0; i++) { y(i, 0)=p2(m.T(i, 0)); y(i, 1)=p2(m.T(i, 1)); y(i, 2)=p2(m.T(i, 2)); }
  m.T=y;
}

/** @brief delete all void triangles (with vertex indices (0, 0, 0)) and void
  vertices (not used for triangles or strips) */
void mlr::Mesh::deleteUnusedVertices() {
  if(!V.N) return;
  uintA p;
  uintA u;
  uint i, Nused;
  
  deleteZeroTriangles(*this);
  
  //count vertex usage
  u.resize(V.d0);
  u.setZero();
  for(i=0; i<T.d0; i++) { u(T(i, 0))++; u(T(i, 1))++; u(T(i, 2))++; }
  //for(i=0;i<strips.N;i++) for(j=0;j<strips(i).N;j++) u(strips(i)(j))=true;
  
  //find proper permutation of vertex list
  p.setStraightPerm(V.d0);
  Nused=p.N;
  for(i=0; i<Nused; i++) if(!u(i)) { Nused--; p.permute(i, Nused); u.permute(i, Nused); i--; }
  
  permuteVertices(*this, p);
  V.resizeCopy(Nused, 3);
}

arr *COMP_V;
bool COMP(uint i, uint j) {
  bool r=(*COMP_V)[i]<(*COMP_V)[j];
  return r;
}

/** @brief delete all void triangles (with vertex indices (0, 0, 0)) and void
  vertices (not used for triangles or strips) */
void mlr::Mesh::fuseNearVertices(double tol) {
  if(!V.N) return;
  uintA p;
  uint i, j;
  
  cout <<"fusing vertices: #V=" <<V.d0 <<", sorting.." <<std::flush;
  //cout <<V <<endl;
  //sort vertices lexically
  p.setStraightPerm(V.d0);
  COMP_V=&V;
  uint *pstop=p.p+p.N;
  std::sort(p.p, pstop, COMP);
  permuteVertices(*this, p);
  
  cout <<"permuting.." <<std::flush;
  //cout <<V <<endl;
  p.setStraightPerm(V.d0);
  for(i=0; i<V.d0; i++) {
    if(p(i)!=i) continue;  //i has already been fused with p(i), and p(i) has already been checked...
    for(j=i+1; j<V.d0; j++) {
      if(V(j, 0)-V(i, 0)>tol) break;
      if(mlr::sqr(V(j, 0)-V(i, 0))+mlr::sqr(V(j, 1)-V(i, 1))+mlr::sqr(V(j, 2)-V(i, 2))<tol*tol) {
        //cout <<"fusing " <<i <<" " <<j <<" " <<V[i] <<" " <<V[j] <<endl;
        p(j)=i;
      }
    }
  }
  
  uintA y(T.d0, 3);
  for(i=0; i<T.d0; i++) { y(i, 0)=p(T(i, 0)); y(i, 1)=p(T(i, 1)); y(i, 2)=p(T(i, 2)); }
  T=y;
  
  cout <<"deleting tris.." <<std::flush;
  deleteZeroTriangles(*this);
  
  cout <<"deleting verts.." <<std::flush;
  deleteUnusedVertices();
  
  cout <<"#V=" <<V.d0 <<", done" <<endl;
}

void getVertexNeighorsList(const mlr::Mesh& m, intA& Vt, intA& VT) {
  uint i, j;
  Vt.resize(m.V.d0);  Vt.setZero();
  VT.resize(m.V.d0, 100);
  for(i=0; i<m.T.d0; i++) {
    j=m.T(i, 0);  VT(j, Vt(j))=i;  Vt(j)++;
    j=m.T(i, 1);  VT(j, Vt(j))=i;  Vt(j)++;
    j=m.T(i, 2);  VT(j, Vt(j))=i;  Vt(j)++;
  }
}

void getTriNormals(const mlr::Mesh& m, arr& Tn) {
  uint i;
  mlr::Vector a, b, c;
  Tn.resize(m.T.d0, 3); //tri normals
  for(i=0; i<m.T.d0; i++) {
    a.set(&m.V(m.T(i, 0), 0)); b.set(&m.V(m.T(i, 1), 0)); c.set(&m.V(m.T(i, 2), 0));
    b-=a; c-=a; a=b^c; a.normalize();
    Tn(i, 0)=a.x;  Tn(i, 1)=a.y;  Tn(i, 2)=a.z;
  }
}

/// flips all faces
void mlr::Mesh::flipFaces() {
  uint i, a;
  for(i=0; i<T.d0; i++) {
    a=T(i, 0);
    T(i, 0)=T(i, 1);
    T(i, 1)=a;
  }
}

/// check whether this is really a closed mesh, and flip inconsistent faces
void mlr::Mesh::clean() {
  uint i, j, idist=0;
  Vector a, b, c, m;
  double mdist=0.;
  arr Tc(T.d0, 3); //tri centers
  arr Tn(T.d0, 3); //tri normals
  uintA Vt(V.d0);
  intA VT(V.d0, 100); //tri-neighbors to a vertex
  Vt.setZero(); VT=-1;
  
  for(i=0; i<T.d0; i++) {
    a.set(&V(T(i, 0), 0)); b.set(&V(T(i, 1), 0)); c.set(&V(T(i, 2), 0));
    
    //tri center
    m=(a+b+c)/3.;
    Tc(i, 0)=m.x;  Tc(i, 1)=m.y;  Tc(i, 2)=m.z;
    
    //farthest tri
    if(m.length()>mdist) { mdist=m.length(); idist=i; }
    
    //tri normal
    b-=a; c-=a; a=b^c; a.normalize();
    Tn(i, 0)=a.x;  Tn(i, 1)=a.y;  Tn(i, 2)=a.z;
    
    //vertex neighbor count
    j=T(i, 0);  VT(j, Vt(j))=i;  Vt(j)++;
    j=T(i, 1);  VT(j, Vt(j))=i;  Vt(j)++;
    j=T(i, 2);  VT(j, Vt(j))=i;  Vt(j)++;
  }
  
  //step through tri list and flip them if necessary
  boolA Tisok(T.d0); Tisok=false;
  uintA Tok; //contains the list of all tris that are ok oriented
  uintA Tnew(T.d0, T.d1);
  Tok.append(idist);
  Tisok(idist)=true;
  int A=0, B=0, D;
  uint r, k, l;
  intA neighbors;
  for(k=0; k<Tok.N; k++) {
    i=Tok(k);
    Tnew(k, 0)=T(i, 0); Tnew(k, 1)=T(i, 1); Tnew(k, 2)=T(i, 2);
    
    for(r=0; r<3; r++) {
      if(r==0) { A=T(i, 0);  B=T(i, 1);  /*C=T(i, 2);*/ }
      if(r==1) { A=T(i, 1);  B=T(i, 2);  /*C=T(i, 0);*/ }
      if(r==2) { A=T(i, 2);  B=T(i, 0);  /*C=T(i, 1);*/ }
      
      //check all triangles that share A & B
      setSection(neighbors, VT[A], VT[B]);
      neighbors.removeAllValues(-1);
      if(neighbors.N>2) MLR_MSG("edge shared by more than 2 triangles " <<neighbors);
      neighbors.removeValue(i);
      //if(!neighbors.N) cout <<"mesh.clean warning: edge has only one triangle that shares it" <<endl;
      
      //orient them correctly
      for(l=0; l<neighbors.N; l++) {
        j=neighbors(l); //j is a neighboring triangle sharing A & B
        D=-1;
        //align the neighboring triangle and let D be its 3rd vertex
        if((int)T(j, 0)==A && (int)T(j, 1)==B) D=T(j, 2);
        if((int)T(j, 0)==A && (int)T(j, 2)==B) D=T(j, 1);
        if((int)T(j, 1)==A && (int)T(j, 2)==B) D=T(j, 0);
        if((int)T(j, 1)==A && (int)T(j, 0)==B) D=T(j, 2);
        if((int)T(j, 2)==A && (int)T(j, 0)==B) D=T(j, 1);
        if((int)T(j, 2)==A && (int)T(j, 1)==B) D=T(j, 0);
        if(D==-1) HALT("dammit");
        //determine orientation
        if(!Tisok(j)) {
          T(j, 0)=B;  T(j, 1)=A;  T(j, 2)=D;
          Tok.append(j);
          Tisok(j)=true;
        } else {
          //check if consistent!
        }
      }
      
#if 0
      //compute their rotation
      if(neighbors.N>1) {
        double phi, phimax;
        int jmax=-1;
        Vector ni, nj;
        for(l=0; l<neighbors.N; l++) {
          j=neighbors(l); //j is a neighboring triangle sharing A & B
          
          a.set(&V(T(i, 0), 0)); b.set(&V(T(i, 1), 0)); c.set(&V(T(i, 2), 0));
          b-=a; c-=a; a=b^c; a.normalize();
          ni = a;
          
          a.set(&V(T(j, 0), 0)); b.set(&V(T(j, 1), 0)); c.set(&V(T(j, 2), 0));
          b-=a; c-=a; a=b^c; a.normalize();
          nj = a;
          
          Quaternion q;
          q.setDiff(ni, -nj);
          q.getDeg(phi, c);
          a.set(&V(A, 0)); b.set(&V(B, 0));
          if(c*(a-b) < 0.) phi+=180.;
          
          if(jmax==-1 || phi>phimax) { jmax=j; phimax=phi; }
        }
        if(!Tisok(jmax)) {
          Tok.append(jmax);
          Tisok(jmax)=true;
        }
      } else {
        j = neighbors(0);
        if(!Tisok(j)) {
          Tok.append(j);
          Tisok(j)=true;
        }
      }
#endif
    }
  }
  if(k<T.d0) {
    cout <<"mesh.clean warning: not all triangles connected: " <<k <<"<" <<T.d0 <<endl;
    cout <<"WARNING: cutting of all non-connected triangles!!" <<endl;
    Tnew.resizeCopy(k, 3);
    T=Tnew;
    deleteUnusedVertices();
  }
  computeNormals();
}

void getEdgeNeighborsList(const mlr::Mesh& m, uintA& EV, uintA& Et, intA& ET) {
  intA Vt, VT;
  getVertexNeighorsList(m, Vt, VT);
  
  uint A=0, B=0, t, tt, i, r, k;
  //build edge list
  EV.resize(m.T.d0*3, 2);   EV=0;     //edge vert neighbors
  ET.resize(m.T.d0*3, 10);  ET=-1;    //edge tri neighbors
  Et.resize(m.T.d0*3); Et.setZero(); //#edge tri neighbors
  boolA done(m.T.d0); done=false;
  for(t=0, k=0; t<m.T.d0; t++) {
    for(r=0; r<3; r++) {
      if(r==0) { A=m.T(t, 0);  B=m.T(t, 1);  }
      if(r==1) { A=m.T(t, 1);  B=m.T(t, 2);  }
      if(r==2) { A=m.T(t, 2);  B=m.T(t, 0);  }
      
      //has AB already been taken care of?
      bool yes=false;
      for(i=0; i<(uint)Vt(A); i++) {
        tt=VT(A, i);
        if(m.T(tt, 0)==B || m.T(tt, 1)==B || m.T(tt, 2)==B) {
          if(done(tt)) yes=true;
        }
      }
      if(yes) continue;
      
      //if not, then do it
      EV(k, 0)=A;
      EV(k, 1)=B;
      for(i=0; i<(uint)Vt(A); i++) {
        tt=VT(A, i);
        if(m.T(tt, 0)==B || m.T(tt, 1)==B || m.T(tt, 2)==B) {
          ET(k, Et(k))=tt;
          Et(k)++;
        }
      }
      k++;
    }
    done(t)=true;
  }
  
  EV.resizeCopy(k, 2);
  ET.resizeCopy(k, 10);
  Et.resizeCopy(k);
  
  cout <<"\n#edges=" <<k
       <<"\nedge=\n" <<EV
       <<"\n@neighs=\n" <<Et
       <<"\nneighs=\n" <<ET <<endl;
}

void getTriNeighborsList(const mlr::Mesh& m, uintA& Tt, intA& TT) {
  intA Vt, VT;
  getVertexNeighorsList(m, Vt, VT);
  
  uint A=0, B=0, t, tt, r, i;
  Tt.resize(m.T.d0, 3);     Tt.setZero();
  TT.resize(m.T.d0, 3, 100); TT=-1;
  for(t=0; t<m.T.d0; t++) {
    for(r=0; r<3; r++) {
      if(r==0) { A=m.T(t, 0);  B=m.T(t, 1);  }
      if(r==1) { A=m.T(t, 1);  B=m.T(t, 2);  }
      if(r==2) { A=m.T(t, 2);  B=m.T(t, 0);  }
      
      for(i=0; i<(uint)Vt(A); i++) {
        tt=VT(A, i);
        if(tt!=t && (m.T(tt, 0)==B || m.T(tt, 1)==B || m.T(tt, 2)==B)) {
          TT(t, r, Tt(t, r))=tt;
          Tt(t, r)++;
        }
      }
    }
  }
  
  //cout <<Tt <<TT <<endl;
}

void mlr::Mesh::skin(uint start) {
  intA TT;
  uintA Tt;
  getTriNeighborsList(*this, Tt, TT);
  arr Tn;
  getTriNormals(*this, Tn);
  
  uintA goodTris;
  boolA added(T.d0);
  goodTris.append(start);
  added=false;
  added(start)=true;
  uint t, tt, r, i, k;
  int m;
  double p, mp=0;
  for(k=0; k<goodTris.N; k++) {
    t=goodTris(k);
    for(r=0; r<3; r++) {
      //select from all neighbors the one most parallel
      m=-1;
      for(i=0; i<Tt(t, r); i++) {
        tt=TT(t, r, i);
        p=scalarProduct(Tn[t], Tn[tt]);
        if(m==-1 || p>mp) { m=tt; mp=p; }
      }
      if(m!=-1 && !added(m)) { goodTris.append(m); added(m)=true; }
    }
  }
  
  uintA Tnew(k, 3);
  for(k=0; k<goodTris.N; k++) {
    t=goodTris(k);
    Tnew(k, 0)=T(t, 0); Tnew(k, 1)=T(t, 1); Tnew(k, 2)=T(t, 2);
  }
  T=Tnew;
  cout <<T <<endl;
}

arr mlr::Mesh::getMean() const {
  return mean(V);
}

void mlr::Mesh::getBox(double& dx, double& dy, double& dz) const {
  dx=dy=dz=0.;
  for(uint i=0;i<V.d0;i++){
    dx=mlr::MAX(dx, fabs(V(i,0)));
    dy=mlr::MAX(dy, fabs(V(i,1)));
    dz=mlr::MAX(dz, fabs(V(i,2)));
  }
}

double mlr::Mesh::getRadius() const {
  double r=0.;
  for(uint i=0;i<V.d0;i++) r=mlr::MAX(r, sumOfSqr(V[i]));
  return sqrt(r);
}

double triArea(const arr& a, const arr& b, const arr& c){
  return .5*length(crossProduct(b-a, c-a));
}

double mlr::Mesh::getArea() const{
  CHECK(T.d1==3,"");
  double A=0.;
  mlr::Vector a,b,c;
  for(uint i=0;i<T.d0;i++){
    a.set(V.p+3*T.p[3*i+0]);
    b.set(V.p+3*T.p[3*i+1]);
    c.set(V.p+3*T.p[3*i+2]);
    A += ((b-a)^(c-a)).length();
  }
  return .5*A;
}

double mlr::Mesh::getVolume() const{
  CHECK(T.d1==3,"");
  mlr::Vector z = getMean();
  mlr::Vector a,b,c;
  double vol=0.;
  for(uint i=0;i<T.d0;i++){
    a.set(V.p+3*T.p[3*i+0]);
    b.set(V.p+3*T.p[3*i+1]);
    c.set(V.p+3*T.p[3*i+2]);
    vol += (a-z) * ((b-a)^(c-a));
  }
  return vol/6.;
}

double mlr::Mesh::meshMetric(const mlr::Mesh& trueMesh, const mlr::Mesh& estimatedMesh) {
  //basically a Haussdorf metric, stupidly realized by brute force algorithm
  auto haussdorfDistanceOneSide = [](const arr& V1, const arr& V2)->double {
    double distance = 0.0;
    for(uint i = 0; i < V1.d0; i++) {
      double shortestDistance = std::numeric_limits<double>::infinity();
      for(uint j = 0; j < V2.d0; j++) {
        double d = length(V2[j]-V1[i]);
        if(d < shortestDistance) {
          shortestDistance = d;
        }
      }
      if(shortestDistance > distance) {
        distance = shortestDistance;
      }
    }
    return distance;
  };

  return mlr::MAX(haussdorfDistanceOneSide(trueMesh.V, estimatedMesh.V), haussdorfDistanceOneSide(estimatedMesh.V, trueMesh.V));
}

double mlr::Mesh::getCircum() const{
  if(!T.N) return 0.;
  CHECK(T.d1==2,"");
  double A=0.;
  for(uint i=0;i<T.d0;i++) A += length(V[T(i,0)] - V[T(i,1)]);
  return A;
}

void mlr::Mesh::write(std::ostream& os) const {
  os <<"Mesh: " <<V.d0 <<" vertices, " <<T.d0 <<" triangles" <<endl;
}

void mlr::Mesh::readFile(const char* filename) {
  read(FILE(filename).getIs(), filename+(strlen(filename)-3), filename);
}

void mlr::Mesh::read(std::istream& is, const char* fileExtension, const char* filename) {
  bool loaded=false;
  if(!strcmp(fileExtension, "obj")) { readObjFile(is); loaded=true; }
  if(!strcmp(fileExtension, "off")) { readOffFile(is); loaded=true; }
  if(!strcmp(fileExtension, "ply")) { readPLY(filename); loaded=true; }
  if(!strcmp(fileExtension, "tri")) { readTriFile(is); loaded=true; }
  if(!strcmp(fileExtension, "stl") || !strcmp(fileExtension, "STL")) { loaded = readStlFile(is); }
  if(!loaded) HALT("can't read fileExtension '" <<fileExtension <<"' file '" <<filename <<"'");
}

void mlr::Mesh::writeTriFile(const char* filename) {
  ofstream os;
  mlr::open(os, filename);
  os <<"TRI" <<endl <<endl
     <<V.d0 <<endl
     <<T.d0 <<endl <<endl;
     
  V.write(os, " ", "\n ", "  ");
  os <<endl <<endl;
  T.write(os, " ", "\n ", "  ");
}

void mlr::Mesh::readTriFile(std::istream& is) {
  uint i, nV, nT;
  is >>PARSE("TRI") >>nV >>nT;
  V.resize(nV, 3);
  T.resize(nT, 3);
  for(i=0; i<V.N; i++) is >>V.elem(i);
  for(i=0; i<T.N; i++) is >>T.elem(i);
}

void mlr::Mesh::writeOffFile(const char* filename) {
  ofstream os;
  mlr::open(os, filename);
  uint i;
  os <<"OFF\n" <<V.d0 <<' ' <<T.d0 <<' ' <<0 <<endl;
  for(i=0; i<V.d0; i++) os <<V(i, 0) <<' ' <<V(i, 1) <<' ' <<V(i, 2) <<endl;
  for(i=0; i<T.d0; i++) os <<3 <<' ' <<T(i, 0) <<' ' <<T(i, 1) <<' ' <<T(i, 2) <<endl;
}

void mlr::Mesh::readOffFile(std::istream& is) {
  uint i, k, nVertices, nFaces, nEdges, alpha;
  bool color;
  mlr::String tag;
  is >>tag;
  if(tag=="OFF") color=false;
  else if(tag=="COFF") color=true;
  else HALT("");
  is >>nVertices >>nFaces >>nEdges;
  CHECK(!nEdges, "can't read edges in off file");
  V.resize(nVertices, 3);
  T.resize(nFaces   , 3);
  if(color) C.resize(nVertices, 3);
  for(i=0; i<V.d0; i++){
    is >>V(i, 0) >>V(i, 1) >>V(i, 2);
    if(color) is >>C(i,0) >>C(i,1) >>C(i,2) >>alpha;
  }
  for(i=0; i<T.d0; i++) {
    is >>k;
    CHECK_EQ(k,3, "can only read triangles from OFF");
    is >>T(i, 0) >>T(i, 1) >>T(i, 2);
  }
}

void mlr::Mesh::readPlyFile(std::istream& is) {
  uint i, k, nVertices, nFaces;
  mlr::String str;
  is >>PARSE("ply") >>PARSE("format") >>str;
  if(str=="ascii") {
    is >>PARSE("1.0");
    is >>PARSE("element vertex") >>nVertices;
    is >>PARSE("property float32 x") >>PARSE("property float32 y") >>PARSE("property float32 z");
    is >>PARSE("property float32 nx") >>PARSE("property float32 ny") >>PARSE("property float32 nz");
    is >>PARSE("element face") >>nFaces;
    is >>PARSE("property list uint8 int32 vertex_indices") >>PARSE("end_header");
    V.resize(nVertices, 3);
    T.resize(nFaces   , 3);
    double nx, ny, nz;
    for(i=0; i<V.d0; i++) {
      is >>V(i, 0) >>V(i, 1) >>V(i, 2) >>nx >>ny >>nz;
    }
    for(i=0; i<T.d0; i++) {
      is >>k >>T(i, 0) >>T(i, 1) >>T(i, 2);
      CHECK_EQ(k,3, "can only read triangles from ply");
    }
  }
}

#ifdef MLR_extern_ply
void mlr::Mesh::writePLY(const char *fn, bool bin) {
  struct PlyFace { unsigned char nverts;  int *verts; };
  struct Vertex { float x,  y,  z ;  };
  uint _nverts = V.d0;
  floatA Vfloat; copy(Vfloat, V);
  Vertex *_vertices  = (Vertex*) Vfloat.p;
  
  PlyProperty vert_props[]  = { /* list of property information for a PlyVertex */
    {"x", Float32, Float32, offsetof(Vertex,x), 0, 0, 0, 0},
    {"y", Float32, Float32, offsetof(Vertex,y), 0, 0, 0, 0},
    {"z", Float32, Float32, offsetof(Vertex,z), 0, 0, 0, 0}
//    {"nx", Float64, Float64, offsetof( Vertex,nx ), 0, 0, 0, 0},
//    {"ny", Float64, Float64, offsetof( Vertex,ny ), 0, 0, 0, 0},
//    {"nz", Float64, Float64, offsetof( Vertex,nz ), 0, 0, 0, 0}
  };
  
  PlyProperty face_props[]  = { /* list of property information for a PlyFace */
    {"vertex_indices", Int32, Int32, offsetof(PlyFace,verts), 1, Uint8, Uint8, offsetof(PlyFace,nverts)},
  };
  
  PlyFile    *ply;
  FILE       *fp = fopen(fn, "w");
  
  const char  *elem_names[]  = { "vertex", "face" };
  ply = write_ply(fp, 2, elem_names, bin? PLY_BINARY_LE : PLY_ASCII);
  
  /* describe what properties go into the PlyVertex elements */
  describe_element_ply(ply, "vertex", _nverts);
  describe_property_ply(ply, &vert_props[0]);
  describe_property_ply(ply, &vert_props[1]);
  describe_property_ply(ply, &vert_props[2]);
//  describe_property_ply(ply, &vert_props[3]);
//  describe_property_ply(ply, &vert_props[4]);
//  describe_property_ply(ply, &vert_props[5]);

  /* describe PlyFace properties (just list of PlyVertex indices) */
  describe_element_ply(ply, "face", T.d0);
  describe_property_ply(ply, &face_props[0]);
  
  header_complete_ply(ply);
  
  //-- put vertices
  put_element_setup_ply(ply, "vertex");
  for(uint i = 0; i < _nverts; i++)  put_element_ply(ply, (void *) &(_vertices[i]));
  
  //-- put tris
  put_element_setup_ply(ply, "face");
  int verts[3] ;
  PlyFace     face ;
  face.nverts = 3 ;
  face.verts  = verts ;
  for(uint i = 0; i < T.d0; i++) {
    face.verts[0] = T(i,0);
    face.verts[1] = T(i,1);
    face.verts[2] = T(i,2);
    put_element_ply(ply, (void *) &face);
  }
  
  close_ply(ply); //calls fclose
  free_ply(ply);
}

void mlr::Mesh::readPLY(const char *fn) {
  struct PlyFace {    unsigned char nverts;  int *verts; };
  struct Vertex {    double x,  y,  z ;  byte r,g,b; };
  uint _nverts=0, _ntrigs=0;
  
  PlyProperty vert_props[]  = { /* list of property information for a PlyVertex */
                                {"x", Float64, Float64, offsetof(Vertex,x), 0, 0, 0, 0},
                                {"y", Float64, Float64, offsetof(Vertex,y), 0, 0, 0, 0},
                                {"z", Float64, Float64, offsetof(Vertex,z), 0, 0, 0, 0},
                                //    {"nx", Float64, Float64, offsetof( Vertex,nx ), 0, 0, 0, 0},
                                //    {"ny", Float64, Float64, offsetof( Vertex,ny ), 0, 0, 0, 0},
                                //    {"nz", Float64, Float64, offsetof( Vertex,nz ), 0, 0, 0, 0}
                                {"red", Uint8, Uint8, offsetof(Vertex,r), 0,0,0,0},
                                {"green", Uint8, Uint8, offsetof(Vertex,g), 0,0,0,0},
                                {"blue", Uint8, Uint8, offsetof(Vertex,b), 0,0,0,0}
                              };
  
  PlyProperty face_props[]  = { /* list of property information for a PlyFace */
                                {"vertex_indices", Int32, Int32, offsetof(PlyFace,verts), 1, Uint8, Uint8, offsetof(PlyFace,nverts)},
  };
  
  FILE    *fp  = fopen(fn, "r");
  if(!fp) return;
  PlyFile *ply = read_ply(fp);
  
  //-- get the number of faces and vertices
  for(uint i = 0; i < (uint)ply->num_elem_types; ++i) {
    int elem_count ;
    char *elem_name = setup_element_read_ply(ply, i, &elem_count);
    if(equal_strings("vertex", elem_name)) _nverts = elem_count;
    if(equal_strings("face",   elem_name)) _ntrigs = elem_count;
  }
  V.resize(_nverts,3);
  C.resize(_nverts,3);
  T.resize(_ntrigs,3);

  //-- examine each element type that is in the file (PlyVertex, PlyFace)
  for(int i = 0; i < ply->num_elem_types; ++i)  {
    int elem_count ;
    char *elem_name = setup_element_read_ply(ply, i, &elem_count);
    
    if(equal_strings("vertex", elem_name))   {
      /* set up for getting PlyVertex elements */
      setup_property_ply(ply, &vert_props[0]);
      setup_property_ply(ply, &vert_props[1]);
      setup_property_ply(ply, &vert_props[2]);
      setup_property_ply(ply, &vert_props[3]);
      setup_property_ply(ply, &vert_props[4]);
      setup_property_ply(ply, &vert_props[5]);

      Vertex vertex;
      for(uint j = 0; j < _nverts; ++j){
        get_element_ply(ply, &vertex);
        V(j,0) = vertex.x;
        V(j,1) = vertex.y;
        V(j,2) = vertex.z;
        C(j,0) = vertex.r;
        C(j,1) = vertex.g;
        C(j,2) = vertex.b;
      }
    } else if(equal_strings("face", elem_name))  {
      /* set up for getting PlyFace elements */
      /* (all we need are PlyVertex indices) */
      setup_property_ply(ply, &face_props[0]) ;
      PlyFace     face ;
      for(uint j = 0; j < _ntrigs; ++j)   {
        get_element_ply(ply, (void *) &face);
        if(face.nverts != 3)
          HALT("not a triangulated surface: polygon " <<j <<" has " <<face.nverts <<" sides") ;
          
        T(j,0) = face.verts[0];
        T(j,1) = face.verts[1];
        T(j,2) = face.verts[2];
        
        free(face.verts) ;
      }
    } else /* all non-PlyVertex and non-PlyFace elements are grabbed here */
      get_other_element_ply(ply);
  }
  
  close_ply(ply); //calls fclose
  free_ply(ply);
}
#else
void mlr::Mesh::writePLY(const char *fn, bool bin) { NICO }
void mlr::Mesh::readPLY(const char *fn) { NICO }
#endif

bool mlr::Mesh::readStlFile(std::istream& is) {
  //first check if binary
  if(mlr::parse(is, "solid", true)) { //is ascii
    mlr::String name;
    is >>name;
    uint i, k=0, k0;
    double x, y, z;
//    cout <<"reading STL file -- object name '" <<name <<"'..." <<endl;
    V.resize(10000);
    //1st pass
    for(i=0, k=0;; i++) {
      k0=k;
      if(k>V.N-10) V.resizeCopy(2*V.N);
      if(!(i%100)) cout <<"\r" <<i <<' ' <<i*7;
      if(mlr::peerNextChar(is)!='f') break;
      is >>PARSE("facet");
      is >>PARSE("normal") >>x >>y >>z;  mlr::skip(is);
      is >>PARSE("outer") >>PARSE("loop");      mlr::skip(is);
      is >>PARSE("vertex")>>V(k++); is>>V(k++); is>>V(k++);   mlr::skip(is);
      is >>PARSE("vertex")>>V(k++); is>>V(k++); is>>V(k++);   mlr::skip(is);
      is >>PARSE("vertex")>>V(k++); is>>V(k++); is>>V(k++);   mlr::skip(is);
      is >>PARSE("endloop");             mlr::skip(is);
      is >>PARSE("endfacet");            mlr::skip(is);
      if(!is.good()) {
        MLR_MSG("reading error - skipping facet " <<i <<" (line " <<i*7+2 <<")");
        is.clear();
        cout <<1 <<endl;
        mlr::skipUntil(is, "endfacet");
        cout <<2 <<endl;
        k=k0;
      }
    }
    is >>PARSE("endsolid");
    if(!is.good()) MLR_MSG("couldn't read STL end tag (line" <<i*7+2);
    cout <<"... STL file read: #tris=" <<i <<" #lines=" <<i*7+2 <<endl;
    CHECK(!(k%9), "not mod 9..");
    V.resizeCopy(k/3, 3);
    T.resize(k/9, 3);
    for(i=0; i<T.N; i++) { T.elem(i)=i; }
  } else { //is binary
    is.clear();
    is.seekg(0, std::ios::beg);
    char header[80];
    is.read(header, 80);
    uint ntri;
    is.read((char*)&ntri, sizeof(ntri));
    T.resize(ntri,3);
    floatA Vfloat(3*ntri,3);
    float normal[3];
    uint16_t att;
    for(uint i=0; i<ntri; i++) {
      is.read((char*)&normal, 3*Vfloat.sizeT);
      is.read((char*)&Vfloat(3*i,0), 9*Vfloat.sizeT);
      T(i,0)=3*i+0;  T(i,1)=3*i+1;  T(i,2)=3*i+2;
      is.read((char*)&att, 2);
      CHECK_EQ(att,0,"this stl file is broke");
    }
    copy(V,Vfloat);
  }
  return true;
}

/*void mlr::Mesh::getOBJ(char* filename){
  if(!glm){
  glm = glmReadOBJ(filename);
  glmReverseWinding(glm);
  }

  ////glmUnitize(glm);
  glmFacetNormals(glm);
  glmVertexNormals(glm, 90.0);

  // creates a display list for the OBJ
  ////  g._pmodel_displaylist = glmList(glm, GLM_SMOOTH | GLM_MATERIAL);
  }*/

uint& Tni(uint, uint) { static uint dummy; return dummy; } //normal index

uint& Tti(uint, uint) { static uint dummy; return dummy; } //texture index


mlr::String str;

char *strn(std::istream& is){
  str.read(is," \n\t\r"," \n\t\r",true); //we once had a character '\d' in there -- for Windows?
  CHECK(is.good(),"could not read line");
  return str.p;
}

/** initialises the ascii-obj file "filename"*/
void mlr::Mesh::readObjFile(std::istream& is) {
  // make a first pass through the file to get a count of the number
  // of vertices, normals, texcoords & triangles
  uint nV, nN, nTex, nT;
  nV = nN = nTex = nT = 0;
  int v, n, t;

//  // we only want to parse the relevant subpart/submesh of the mesh therefore
//  // jump to the right position and stop parsing at the right positon.
//  if (parsing_pos_start > -1)
//    is.seekg(parsing_pos_start); //  fseek(file, parsing_pos_start, SEEK_SET);

//  while ((sscanf(strn(is), "%s", str.p) != EOF) && (ftell(file) < parsing_pos_end)) {
  strn(is);
  for(bool ex=false;!ex;){
//    if(parsing_pos_start>-1 && is.tellg()>=parsing_pos_end) break;
    switch(str.p[0]) {
      case '\0':
        is.clear();
        ex=true;
        break; //EOF
      case '#':
        mlr::skipRestOfLine(is);
        strn(is);
        break;
      case 'v':
        switch(str.p[1]) {
          case '\0': nV++;    mlr::skipRestOfLine(is); break;  // vertex
          case 'n':  nN++;    mlr::skipRestOfLine(is); break;  // normal
          case 't':  nTex++;  mlr::skipRestOfLine(is); break;  // texcoord
          default: HALT("firstPass(): Unknown token '" <<str.p <<"'");  break;
        }
        strn(is);
        break;
      case 'f':               // face
        v = n = t = 0;
        strn(is);
        // can be one of %d, %d//%d, %d/%d, %d/%d/%d %d//%d
        if(strstr(str.p, "//")) {
          // v//n
          CHECK(sscanf(str.p   , "%d//%d", &v, &n), "fscan failed");
          CHECK(sscanf(strn(is), "%d//%d", &v, &n), "fscan failed");
          CHECK(sscanf(strn(is), "%d//%d", &v, &n), "fscan failed");
          nT++;
          while(sscanf(strn(is), "%d//%d", &v, &n) > 0) nT++;
        } else if(sscanf(str.p, "%d/%d/%d", &v, &t, &n) == 3) {
          // v/t/n
          CHECK(sscanf(strn(is), "%d/%d/%d", &v, &t, &n), "fscan failed");
          CHECK(sscanf(strn(is), "%d/%d/%d", &v, &t, &n), "fscan failed");
          nT++;
          while(sscanf(strn(is), "%d/%d/%d", &v, &t, &n) > 0) nT++;
        } else if(sscanf(str.p, "%d/%d", &v, &t) == 2) {
          // v/t
          CHECK(sscanf(strn(is), "%d/%d", &v, &t), "fscan failed");
          CHECK(sscanf(strn(is), "%d/%d", &v, &t), "fscan failed");
          nT++;
          while(sscanf(strn(is), "%d/%d", &v, &t) > 0) nT++;
        } else {
          // v
          CHECK(sscanf(strn(is), "%d", &v), "fscan failed");
          CHECK(sscanf(strn(is), "%d", &v), "fscan failed");
          nT++;
          while(sscanf(strn(is), "%d", &v) > 0) nT++;
        }
        break;
        
      default:  MLR_MSG("unsupported .obj file tag '" <<str <<"'");  mlr::skipRestOfLine(is);  strn(is);  break;
    }
  }
  
  //allocate memory
  V.resize(nV, 3);
  Vn.resize(nN, 3);
  T.resize(nT, 3);
  Tn.resize(nT, 3);
  //if(nVN) N.resize(nVN, 3);
  //if(nTex) Tex.tesize(nTex, 2);
  
  // rewind to beginning of file and read in the data this pass
  is.seekg(0);
  is.clear();
//  if (parsing_pos_start > -1)
//    is.seekg(parsing_pos_start); //  fseek(file, parsing_pos_start, SEEK_SET);
  
  /* on the second pass through the file, read all the data into the
     allocated arrays */
  nV = nN = nTex = nT = 0;
  ////_material = 0;
  
//  while ((sscanf(strn(is), "%s", str.p) != EOF) && (ftell(file) < parsing_pos_end)) {
  strn(is);
  for(bool ex=false;!ex;){
//    if(parsing_pos_start>-1 && is.tellg()>=parsing_pos_end) break;
    switch(str.p[0]) {
      case '\0':
        is.clear();
        ex=true;
        break; //EOF
      case '#':
        mlr::skipRestOfLine(is);
        strn(is);
        break;  //comment
      case 'v':               // v, vn, vt
        switch(str.p[1]) {
          case '\0': is >>V(nV, 0) >>V(nV, 1) >> V(nV, 2);  nV++;  break;  //vertex
          case 'n':  is >>Vn(nN, 0) >>Vn(nN, 1) >>Vn(nN, 2);  nN++;  break;  //normal
          case 't':  /*CHECK(sscanf(strn(is), "%f %f", &Tex(nTex, 0), &Tex(nTex, 1)), "fscan failed");  nTex++;*/  break;  //texcoord
        }
        strn(is);
        break;
      case 'f':               // face
        v = n = t = 0;
        strn(is);
        if(strstr(str.p, "//")) {
          // v//n
          sscanf(str.p, "%d//%d", &v, &n);
          
          T(nT, 0) = v < 0 ? v + nV : v;
          Tni(nT, 0) = n < 0 ? n + nN : n;
          CHECK(sscanf(strn(is), "%d//%d", &v, &n), "fscan failed");
          T(nT, 1) = v < 0 ? v + nV : v;
          Tni(nT, 1) = n < 0 ? n + nN : n;
          CHECK(sscanf(strn(is), "%d//%d", &v, &n), "fscan failed");
          T(nT, 2) = v < 0 ? v + nV : v;
          Tni(nT, 2) = n < 0 ? n + nN : n;
          //// group->triangles[group->nT++] = nT;
          nT++;
          while(sscanf(strn(is), "%d//%d", &v, &n) > 0) {
            T(nT, 0) = T(nT-1, 0);
            Tni(nT, 0) = Tni(nT-1, 0);
            T(nT, 1) = T(nT-1, 2);
            Tni(nT, 1) = Tni(nT-1, 2);
            T(nT, 2) = v < 0 ? v + nV : v;
            Tni(nT, 2) = n < 0 ? n + nN : n;
            //// group->triangles[group->numtriangles++] = numtriangles;
            nT++;
          }
        } else if(sscanf(str.p, "%d/%d/%d", &v, &t, &n) == 3) {
          // v/t/n
          T(nT, 0) = v < 0 ? v + nV : v;
          Tti(nT, 0) = t < 0 ? t + nTex : t;
          Tni(nT, 0) = n < 0 ? n + nN : n;
          CHECK(sscanf(strn(is), "%d/%d/%d", &v, &t, &n), "fscan failed");
          T(nT, 1) = v < 0 ? v + nV : v;
          Tti(nT, 1) = t < 0 ? t + nTex : t;
          Tni(nT, 1) = n < 0 ? n + nN : n;
          CHECK(sscanf(strn(is), "%d/%d/%d", &v, &t, &n), "fscan failed");
          T(nT, 2) = v < 0 ? v + nV : v;
          Tti(nT, 2) = t < 0 ? t + nTex : t;
          Tni(nT, 2) = n < 0 ? n + nN : n;
          //// group->triangles[group->numtriangles++] = numtriangles;
          nT++;
          while(sscanf(strn(is), "%d/%d/%d", &v, &t, &n) > 0) {
            T(nT, 0) = T(nT-1, 0);
            Tti(nT, 0) = Tti(nT-1, 0);
            Tni(nT, 0) = Tni(nT-1, 0);
            T(nT, 1) = T(nT-1, 2);
            Tti(nT, 1) = Tti(nT-1, 2);
            Tni(nT, 1) = Tni(nT-1, 2);
            T(nT, 2) = v < 0 ? v + nV : v;
            Tti(nT, 2) = t < 0 ? t + nTex : t;
            Tni(nT, 2) = n < 0 ? n + nN : n;
            //// group->triangles[group->numtriangles++] = numtriangles;
            nT++;
          }
        } else if(sscanf(str.p, "%d/%d", &v, &t) == 2) {
          // v/t
          
          T(nT, 0) = v < 0 ? v + nV : v;
          Tti(nT, 0) = t < 0 ? t + nTex : t;
          CHECK(sscanf(strn(is), "%d/%d", &v, &t), "fscan failed");
          T(nT, 1) = v < 0 ? v + nV : v;
          Tti(nT, 1) = t < 0 ? t + nTex : t;
          CHECK(sscanf(strn(is), "%d/%d", &v, &t), "fscan failed");
          T(nT, 2) = v < 0 ? v + nV : v;
          Tti(nT, 2) = t < 0 ? t + nTex : t;
          //// group->triangles[group->numtriangles++] = numtriangles;
          nT++;
          while(sscanf(strn(is), "%d/%d", &v, &t) > 0) {
            T(nT, 0) = T(nT-1, 0);
            Tti(nT, 0) = Tti(nT-1, 0);
            T(nT, 1) = T(nT-1, 2);
            Tti(nT, 1) = Tti(nT-1, 2);
            T(nT, 2) = v < 0 ? v + nV : v;
            Tti(nT, 2) = t < 0 ? t + nTex : t;
            //// group->triangles[group->numtriangles++] = numtriangles;
            nT++;
          }
        } else {
          // v
          sscanf(str.p, "%d", &v);
          T(nT, 0) = v < 0 ? v + nV : v;
          CHECK(sscanf(strn(is), "%d", &v), "fscan failed");
          T(nT, 1) = v < 0 ? v + nV : v;
          CHECK(sscanf(strn(is), "%d", &v), "fscan failed");
          T(nT, 2) = v < 0 ? v + nV : v;
          //// group->triangles[group->numtriangles++] = nT;
          nT++;
          while(sscanf(strn(is), "%d", &v) > 0) {
            T(nT, 0) = T(nT-1, 0);
            T(nT, 1) = T(nT-1, 2);
            T(nT, 2) = v < 0 ? v + nV : v;
            //// group->triangles[group->numtriangles++] = numtriangles;
            nT++;
          }
        }
        break;
        
      default:  mlr::skipRestOfLine(is);  strn(is);  break;
    }
  }
  
  //CONVENTION!: start counting vertex indices from 0!!
  T -= T.min();
}

//===========================================================================
// Util
/**
 * @brief Return the position of the submesh in the obj file in bytes (can be
 * used by fseek).
 *
 * @param filename file to parse.
 */
uintA getSubMeshPositions(const char* filename) {
  CHECK(mlr::String(filename).endsWith("obj"),
        "getSubMeshPositions parses only obj files.");
  FILE* file;
  char buf[128];
  file = fopen(filename, "r");
  CHECK(file,
        "can't open data file " << filename << "; cwd is " << getcwd_string());

  int flag = 0;
  long start_pos = 0;
  long end_pos = 0;

  uintA result;
  while(fscanf(file, "%s", buf) != EOF) {
    switch(buf[0]) {
      case 'v': {
        if (flag > 0) {
          end_pos = ftell(file) - 1;
          result.append(TUP((uint)start_pos, (uint)end_pos));
          start_pos = end_pos;
          flag =0; }
      } break;
      case 'f': {
        flag=1;
      } break;
    }
  }

  end_pos = ftell(file) - 1;
  result.append(TUP((uint)start_pos, (uint)end_pos));
  result.reshape(result.N/2,2);
  return result;
}

#ifdef MLR_GL

extern void glColor(float r, float g, float b, float alpha);

/// GL routine to draw a mlr::Mesh
void mlr::Mesh::glDraw(struct OpenGL&) {
  if(C.nd==1){
    CHECK(C.N==3 || C.N==4, "need a basic color");
    GLboolean light=true;
    glGetBooleanv(GL_LIGHTING, &light); //this doesn't work!!?? even when disabled, returns true; never changes 'light'
    GLfloat col[4] = { (float)C(0), (float)C(1), (float)C(2), (C.N==3?1.f:(float)C(3)) };
    glColor4fv(col);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, col);
  }

  if(!T.N){  //-- draw point cloud
    if(!V.N) return;
    CHECK(V.nd==2 && V.d1==3, "wrong dimension");
    glPointSize(3.);
    glDisable(GL_LIGHTING);

    glEnableClientState(GL_VERTEX_ARRAY);
    if(C.N==V.N) glEnableClientState(GL_COLOR_ARRAY); else glDisableClientState(GL_COLOR_ARRAY);

    glVertexPointer(3, GL_DOUBLE, V.d1-3, V.p);
    if(C.N==V.N) glColorPointer(3, GL_DOUBLE, C.d1-3, C.p );

    glDrawArrays(GL_POINTS, 0, V.d0);
    glDisableClientState(GL_VERTEX_ARRAY);

    glEnable(GL_LIGHTING);
    glPointSize(1.);
    return;
  }

  if(T.d1==2){ //-- draw lines
    glLineWidth(3.f);
    glBegin(GL_LINES);
    for(uint t=0; t<T.d0; t++) {
      glVertex3dv(&V(T(t, 0), 0));
      glVertex3dv(&V(T(t, 1), 0));
    }
    glEnd();
    return;
  }

  //-- draw a mesh
  if(V.d0!=Vn.d0 || T.d0!=Tn.d0) computeNormals();

#if 1
  if(!C.N || C.nd==1 || C.d0==V.d0){ //we have colors for each vertex -> use index arrays

    //  glShadeModel(GL_FLAT);
    glShadeModel(GL_SMOOTH);
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);
    if(C.N==V.N) glEnableClientState(GL_COLOR_ARRAY); else glDisableClientState(GL_COLOR_ARRAY);
    if(C.N==V.N) glDisable(GL_LIGHTING); //because lighting requires ambiance colors to be set..., not just color..

    glVertexPointer(3, GL_DOUBLE, 0, V.p);
    glNormalPointer(GL_DOUBLE, 0, Vn.p);
    if(C.N==V.N) glColorPointer(3, GL_DOUBLE, 0, C.p);

    glDrawElements(GL_TRIANGLES, T.N, GL_UNSIGNED_INT, T.p);

    if(C.N) glEnable(GL_LIGHTING);

  }else{ //we have colors for each tri -> render tris directly and with tri-normals

    CHECK_EQ(C.d0, T.d0, "");
    CHECK_EQ(Tn.d0, T.d0, "");
    glShadeModel(GL_FLAT);
    glBegin(GL_TRIANGLES);
    GLboolean light=true;
    glGetBooleanv(GL_LIGHTING, &light); //this doesn't work!!?? even when disabled, returns true; never changes 'light'
    for(uint t=0; t<T.d0; t++) {
      uint   *tri  = T.p  + 3*t; //&T(t, 0);
      double *col  = C.p  + 3*t; //&C(t, 0);
      double *norm = Tn.p + 3*t; //&Tn(t, 0);

      GLfloat ambient[4] = { (float)col[0], (float)col[1], (float)col[2], 1.f };
      if(!light) glColor4fv(ambient);
      else       glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, ambient);

      glNormal3dv(norm);
      glVertex3dv(V.p + 3*tri[0]); //&V(tri[0],0);
      glVertex3dv(V.p + 3*tri[1]);
      glVertex3dv(V.p + 3*tri[2]);
    }
    glEnd();
  }
#elif 1 //simple with vertex normals
  uint i, v;
  glShadeModel(GL_SMOOTH);
  glBegin(GL_TRIANGLES);
  for(i=0; i<T.d0; i++) {
    if(C.d0==T.d0)  glColor(C(i, 0), C(i, 1), C(i, 2),1.);
    v=T(i, 0);  glNormal3dv(&Vn(v, 0));  if(C.d0==V.d0) glColor3dv(&C(v, 0));  glVertex3dv(&V(v, 0));
    v=T(i, 1);  glNormal3dv(&Vn(v, 0));  if(C.d0==V.d0) glColor3dv(&C(v, 0));  glVertex3dv(&V(v, 0));
    v=T(i, 2);  glNormal3dv(&Vn(v, 0));  if(C.d0==V.d0) glColor3dv(&C(v, 0));  glVertex3dv(&V(v, 0));
  }
  glEnd();
#else //simple with triangle normals
#if 0 //draw normals
  glColor(.5, 1., .0);
  Vector a, b, c, x;
  for(i=0; i<T.d0; i++) {
    glBegin(GL_LINES);
    a.set(&V(T(i, 0), 0)); b.set(&V(T(i, 1), 0)); c.set(&V(T(i, 2), 0));
    x.setZero(); x+=a; x+=b; x+=c; x/=3;
    glVertex3dv(x.v);
    a.set(&Tn(i, 0));
    x+=.05*a;
    glVertex3dv(x.v);
    glEnd();
  }
#endif
#endif

  if(orsDrawWires) { //on top of mesh
#if 0
    uint t;
    for(t=0; t<T.d0; t++) {
      glBegin(GL_LINE_LOOP);
      glVertex3dv(&V(T(t, 0), 0));
      glVertex3dv(&V(T(t, 1), 0));
      glVertex3dv(&V(T(t, 2), 0));
      glEnd();
    }
#else
    glEnableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
    if(C.N==V.N) glEnableClientState(GL_COLOR_ARRAY); else glDisableClientState(GL_COLOR_ARRAY);

    glVertexPointer(3, GL_DOUBLE, 0, V.p);
    if(C.N==V.N) glColorPointer(3, GL_DOUBLE, 0, C.p);
    glDrawElements(GL_LINE_STRIP, T.N, GL_UNSIGNED_INT, T.p);
#endif
  }

}

#else //MLR_GL

void mlr::Mesh::glDraw(struct OpenGL&) { NICO }
void glDrawMesh(void*) { NICO }
void glTransform(const mlr::Transformation&) { NICO }
#endif

//==============================================================================

extern OpenGL& NoOpenGL;

void glDrawMeshes(void *P){
  MeshA& meshes = *((MeshA*)P);
  double GLmatrix[16];
  for(mlr::Mesh& mesh:meshes){
    glPushMatrix();
    mesh.glX.getAffineMatrixGL(GLmatrix);
    glLoadMatrixd(GLmatrix);
    mesh.glDraw(NoOpenGL);
    glPopMatrix();
  }
}

//==============================================================================

void inertiaSphere(double *I, double& mass, double density, double radius) {
  double r2=radius*radius;
  if(density) mass=density*4./3.*MLR_PI*r2*radius;
  I[1]=I[2]=I[3]=I[5]=I[6]=I[7]=0.;
  I[0]=.4*mass*r2;
  I[4]=.4*mass*r2;
  I[8]=.4*mass*r2;
}

void inertiaBox(double *I, double& mass, double density, double dx, double dy, double dz) {
  if(density) mass=density*dx*dy*dz;
  I[1]=I[2]=I[3]=I[5]=I[6]=I[7]=0.;
  double x2=dx*dx, y2=dy*dy, z2=dz*dz;
  I[0]=mass/12.*(y2+z2);
  I[4]=mass/12.*(x2+z2);
  I[8]=mass/12.*(x2+y2);
}

void inertiaCylinder(double *I, double& mass, double density, double height, double radius) {
  double r2=radius*radius, h2=height*height;
  if(density) mass=density*MLR_PI*r2*height;
  I[1]=I[2]=I[3]=I[5]=I[6]=I[7]=0.;
  I[0]=mass/12.*(3.*r2+h2);
  I[4]=mass/12.*(3.*r2+h2);
  I[8]=mass/2.*r2;
}


//===========================================================================
//
// GJK interface
//

#ifdef MLR_extern_GJK
GJK_point_type& NoPointType = *((GJK_point_type*)NULL);
double GJK_sqrDistance(const mlr::Mesh& mesh1, const mlr::Mesh& mesh2,
                       const mlr::Transformation& t1, const mlr::Transformation& t2,
                       mlr::Vector& p1, mlr::Vector& p2,
                       mlr::Vector& e1, mlr::Vector& e2,
                       GJK_point_type& pt1, GJK_point_type& pt2){
  // convert meshes to 'Object_structures'
  Object_structure m1,m2;
  mlr::Array<double*> Vhelp1, Vhelp2;
  m1.numpoints = mesh1.V.d0;  m1.vertices = mesh1.V.getCarray(Vhelp1);  m1.rings=NULL; //TODO: rings would make it faster
  m2.numpoints = mesh2.V.d0;  m2.vertices = mesh2.V.getCarray(Vhelp2);  m2.rings=NULL;

  // convert transformations to affine matrices
  arr T1,T2;
  mlr::Array<double*> Thelp1, Thelp2;
  if(&t1){  T1=t1.getAffineMatrix();  T1.getCarray(Thelp1);  }
  if(&t2){  T2=t2.getAffineMatrix();  T2.getCarray(Thelp2);  }

  // call GJK
  simplex_point simplex;
  double d2 = gjk_distance(&m1, Thelp1.p, &m2, Thelp2.p, (&p1?p1.p():NULL), (&p2?p2.p():NULL), &simplex, 0);

//  cout <<"simplex npts=" <<simplex.npts <<endl;
//  cout <<"simplex lambda=" <<arr(simplex.lambdas, 4) <<endl;
//  cout <<"simplex 1=" <<intA(simplex.simplex1, 4) <<endl;
//  cout <<"simplex 2=" <<intA(simplex.simplex2, 4) <<endl;

//  arr P1=zeros(3), P2=zeros(3);
//  for(int i=0;i<simplex.npts;i++) P1 += simplex.lambdas[i] * arr(simplex.coords1[i],3);
//  for(int i=0;i<simplex.npts;i++) P2 += simplex.lambdas[i] * arr(simplex.coords2[i],3);
//  cout <<"P1=" <<P1 <<", " <<p1 <<endl;
//  cout <<"P2=" <<P2 <<", " <<p2 <<endl;

  // analyze point types
  if(&e1 && &e2){
    e1.setZero();
    e2.setZero();
    pt1=GJK_vertex;
    pt2=GJK_vertex;
    if(d2<1e-6) return d2;

    if(simplex.npts==1){

    }else if(simplex.npts==2){

      if(simplex.simplex1[0]==simplex.simplex1[1]){
        pt1=GJK_vertex;
      }else{
        pt1=GJK_edge;
        for(uint i=0;i<3;i++) e1(i) = simplex.coords1[0][i] - simplex.coords1[1][i];
        e1.normalize();
      }
      if(simplex.simplex2[0]==simplex.simplex2[1]){
        pt2=GJK_vertex;
      }else{
        pt2=GJK_edge;
        for(uint i=0;i<3;i++) e2(i) = simplex.coords2[0][i] - simplex.coords2[1][i];
        e2.normalize();
      }

    }else if(simplex.npts==3){

      // 1st point
      if(simplex.simplex1[0]==simplex.simplex1[1] && simplex.simplex1[0]==simplex.simplex1[2]){
        pt1=GJK_vertex;
      }else if(simplex.simplex1[0]!=simplex.simplex1[1] && simplex.simplex1[0]!=simplex.simplex1[2] && simplex.simplex1[1]!=simplex.simplex1[2]){
        pt1=GJK_face;
      }else{
        pt1=GJK_edge;
        if(simplex.simplex1[0]==simplex.simplex1[1]){
          for(uint i=0;i<3;i++) e1(i) = simplex.coords1[0][i] - simplex.coords1[2][i];
        }else{
          for(uint i=0;i<3;i++) e1(i) = simplex.coords1[0][i] - simplex.coords1[1][i];
        }
        e1.normalize();
      }

      // 2nd point
      if(simplex.simplex2[0]==simplex.simplex2[1] && simplex.simplex2[0]==simplex.simplex2[2]){
        pt2=GJK_vertex;
      }else if(simplex.simplex2[0]!=simplex.simplex2[1] && simplex.simplex2[0]!=simplex.simplex2[2] && simplex.simplex2[1]!=simplex.simplex2[2]){
        pt2=GJK_face;
      }else{
        pt2=GJK_edge;
        if(simplex.simplex2[0]==simplex.simplex2[1]){
          for(uint i=0;i<3;i++) e2(i) = simplex.coords2[0][i] - simplex.coords2[2][i];
        }else{
          for(uint i=0;i<3;i++) e2(i) = simplex.coords2[0][i] - simplex.coords2[1][i];
        }
        e2.normalize();
      }

    }else{
      if(d2>EPSILON) LOG(-2) <<"GJK converges to simplex!";
    }

//    cout <<"point types= " <<pt1 <<' ' <<pt2 <<endl;
//    CHECK(!(pt1==3 && pt2==3),"");
//    CHECK(!(pt1==2 && pt2==3),"");
//    CHECK(!(pt1==3 && pt2==2),"");
  }

  return d2;
}
#else
double GJK_distance(mlr::Mesh& mesh1, mlr::Mesh& mesh2,
                    mlr::Transformation& t1, mlr::Transformation& t2,
                    mlr::Vector& p1, mlr::Vector& p2){ NICO }
#endif


//===========================================================================
//
// Lewiner interface
//

#ifdef MLR_extern_Lewiner
#  include "Lewiner/MarchingCubes.h"


void mlr::Mesh::setImplicitSurface(ScalarFunction f, double lo, double hi, uint res) {
  MarchingCubes mc(res, res, res);
  mc.init_all() ;
  double startTime = mlr::timerRead();
  //compute data
  uint k=0, j=0, i=0;
  float x=lo, y=lo, z=lo;
  for(k=0; k<res; k++) {
    z = lo+k*(hi-lo)/res;
    for(j=0; j<res; j++) {
      y = lo+j*(hi-lo)/res;
      for(i=0; i<res; i++) {
        x = lo+i*(hi-lo)/res;
        mc.set_data(f(NoArr, NoArr, ARR((double)x, (double)y, (double)z)), i, j, k) ;
      }
    }
  }
  cout << "calculation of data took: " << mlr::timerRead() - startTime << " seconds" << endl;
  mc.run();
  mc.clean_temps();

  //convert to Mesh
  clear();
  V.resize(mc.nverts(), 3);
  T.resize(mc.ntrigs(), 3);
  for(i=0; i<V.d0; i++) {
    V(i, 0)=lo+mc.vert(i)->x*(hi-lo)/res;
    V(i, 1)=lo+mc.vert(i)->y*(hi-lo)/res;
    V(i, 2)=lo+mc.vert(i)->z*(hi-lo)/res;
  }
  for(i=0; i<T.d0; i++) {
    T(i, 0)=mc.trig(i)->v1;
    T(i, 1)=mc.trig(i)->v2;
    T(i, 2)=mc.trig(i)->v3;
  }
}


void mlr::Mesh::setImplicitSurface(ScalarFunction f, double xLo, double xHi, double yLo, double yHi, double zLo, double zHi, uint res) {
  MarchingCubes mc(res, res, res);
  mc.init_all() ;

  //compute data
  uint k=0, j=0, i=0;
  float x, y, z;
  for(k=0; k<res; k++) {
    z = zLo+k*(zHi-zLo)/res;
    for(j=0; j<res; j++) {
      y = yLo+j*(yHi-yLo)/res;
      for(i=0; i<res; i++) {
        x = xLo+i*(xHi-xLo)/res;
        mc.set_data(f(NoArr, NoArr, ARR((double)x, (double)y, (double)z)), i, j, k) ;
      }
    }
  }

  mc.run();
  mc.clean_temps();

  //convert to Mesh
  clear();
  V.resize(mc.nverts(), 3);
  T.resize(mc.ntrigs(), 3);
  for(i=0; i<V.d0; i++) {
    V(i, 0)=xLo+mc.vert(i)->x*(xHi-xLo)/res;
    V(i, 1)=yLo+mc.vert(i)->y*(yHi-yLo)/res;
    V(i, 2)=zLo+mc.vert(i)->z*(zHi-zLo)/res;
  }
  for(i=0; i<T.d0; i++) {
    T(i, 0)=mc.trig(i)->v1;
    T(i, 1)=mc.trig(i)->v2;
    T(i, 2)=mc.trig(i)->v3;
  }
}

#else //extern_Lewiner
void mlr::Mesh::setImplicitSurface(ScalarFunction f, double lo, double hi, uint res) {
  NICO
}
#endif
/** @} */

//===========================================================================

DistanceFunction_Sphere::DistanceFunction_Sphere(const mlr::Transformation& _t, double _r):t(_t),r(_r){
  ScalarFunction::operator=( [this](arr& g, arr& H, const arr& x)->double{ return f(g,H,x); } );
}

double DistanceFunction_Sphere::f(arr& g, arr& H, const arr& x){
  arr d = x-conv_vec2arr(t.pos);
  double len = length(d);
  if(&g) g = d/len;
  if(&H) H = 1./len * (eye(3) - (d^d)/(len*len));
  return len-r;
}

//===========================================================================

//double DistanceFunction_InfCylinder::fs(arr& g, arr& H, const arr& x){
//  z = z / length(z);
//  arr a = (x-c) - scalarProduct((x-c), z) * z;
//  arr I(x.d0,x.d0);
//  uint i;
//  double na = length(a);

//  if(&g) g = s*a/na;
//  if(&H){
//    I.setZero();
//    for(i=0;i<x.d0;++i) I(i,i)=1;
//    H = s/na * (I - z*(~z) - 1/(na*na) * a*(~a));
//  }
//  return s*(na-r);
//}

//===========================================================================

DistanceFunction_Cylinder::DistanceFunction_Cylinder(const mlr::Transformation& _t, double _r, double _dz):t(_t),r(_r),dz(_dz){
  ScalarFunction::operator=( [this](arr& g, arr& H, const arr& x)->double{ return f(g,H,x); } );
}

double DistanceFunction_Cylinder::f(arr& g, arr& H, const arr& x){
  arr z = conv_vec2arr(t.rot.getZ());
  arr c = conv_vec2arr(t.pos);
  arr b = scalarProduct(x-c, z) * z;
  arr a = (x-c) - b;
  arr I(3,3);
  double la = length(a);
  double lb = length(b);
  arr aaTovasq = 1/(la*la) * (a^a);
  arr zzT = z^z;

  if ( lb < dz/2. ){ // x projection on z is inside cyl
    if(la<r && (dz/2.-lb)<(r-la)){ // x is INSIDE the cyl and closer to the lid than the wall
      if(&g) g = 1./lb*b; //z is unit: s*z*|z|*sgn(b*z) = s*b/nb
      if(&H) { I.setZero(); H=I; }
      return lb-dz/2.;
    }else{ // closer to the side than to a lid (inc. cases in- and outside the tube, because (r-na)<0 then)
      if(&g) g = a/la;
      if(&H){
        I.setId(3);
        H = 1./la * (I - zzT - aaTovasq);
      }
      return la-r;
    }
  }else{// x projection on z is outside cylinder
    if ( la < r ){// inside the infinite cylinder
      if(&g) g = b/lb;
      if(&H) H.resize(3,3).setZero();
      return lb-dz/2.;
    }else{ // outside the infinite cyl
      arr v =  b/lb * (lb-dz/2.)  + a/la * (la-r); //MT: good! (note: b/nb is the same as z) SD: well, b/nb is z or -z.
      double nv=length(v);
      if(&g) g = v/nv;
      if(&H){
        I.setId(3);
        arr dvdx = (la-r)/la*( I - zzT - aaTovasq )
                   + aaTovasq + zzT;
        H = 1./nv* (dvdx - 1/nv/nv * (v^v) * (~dvdx) );
      }
      return nv;
    }
  }
  HALT("You shouldn't be here!");
}

//===========================================================================

/// dx, dy, dz are box-wall-coordinates: width=2*dx...; t is box transform; x is query point in world
void closestPointOnBox(arr& closest, arr& signs, const mlr::Transformation& t, double dx, double dy, double dz, const arr& x){
  arr rot = t.rot.getArr();
  arr a_rel = (~rot)*(x-conv_vec2arr(t.pos)); //point in box coordinates
  arr dim = {dx, dy, dz};
  signs.resize(3);
  signs.setZero();
  closest = a_rel;
  arr del_abs = fabs(a_rel)-dim;
  if(del_abs.max()<0.){ //inside
    uint side=del_abs.maxIndex(); //which side are we closest to?
     //in positive or neg direction?
    if(a_rel(side)>0){ closest(side) = dim(side);  signs(side)=+1.; }
    else             { closest(side) =-dim(side);  signs(side)=-1.; }
  }else{ //outside
    for(uint side=0;side<3;side++){
      if(closest(side)<-dim(side)){ signs(side)=-1.; closest(side)=-dim(side); }
      if(closest(side)> dim(side)){ signs(side)=+1.; closest(side)= dim(side); }
    }
  }
  closest = rot*closest + t.pos.getArr();
}

//===========================================================================

DistanceFunction_Box::DistanceFunction_Box(const mlr::Transformation& _t, double _dx, double _dy, double _dz, double _r):t(_t),dx(_dx),dy(_dy),dz(_dz), r(_r){
  ScalarFunction::operator=( [this](arr& g, arr& H, const arr& x)->double{ return f(g,H,x); } );
}

double DistanceFunction_Box::f(arr& g, arr& H, const arr& x){
  arr rot = t.rot.getArr();
  arr a_rel = (~rot)*(x-conv_vec2arr(t.pos)); //point in box coordinates
  arr dim = {dx, dy, dz};

  arr closest = a_rel;
  arr del_abs = fabs(a_rel)-dim;
  //-- find closest point on box and distance to it
  if(del_abs.max()<0.){ //inside
    uint side=del_abs.maxIndex(); //which side are we closest to?
    if(a_rel(side)>0) closest(side) = dim(side);  else  closest(side)=-dim(side); //in positive or neg direction?
  }else{ //outside
    closest = elemWiseMax(-dim,closest);
    closest = elemWiseMin(dim,closest);
  }

  arr del = a_rel-closest;
  double d = length(del);
  if(&g) g = rot*del/d; //transpose(R) rotates the gradient back to world coordinates
  if(&H){
    if(d<0.){ //inside
      H.resize(3,3).setZero();
    }else{ //outside
      if(del_abs.min()>0.){ //outside on all 3 axis
        H = 1./d * (eye(3) - (del^del)/(d*d));
      }else{
        arr edge=del_abs;
        for(double& z: edge) z=(z<0.)?0.:1.;
        if(sum(edge)<=1.1){ //closest to the plane (equals 1.)
          H.resize(3,3).setZero();
        }else{ //closest to an edge
          edge = 1.-edge;
          H = 1./d * (eye(3) - (del^del)/(d*d) - (edge^edge));
        }
      }
      H = rot*H*(~rot);
    }
  }

  return d-r;
}

ScalarFunction DistanceFunction_SSBox = [](arr& g, arr& H, const arr& x) -> double{
  // x{0,2} are box-wall-coordinates, not width!
  CHECK_EQ(x.N, 14, "query-pt + abcr + pose");
  mlr::Transformation t;
  t.pos.set( x({7,9}) );
  t.rot.set( x({10,13}) );
  t.rot.normalize();
  arr closest, signs;
  closestPointOnBox(closest, signs, t, x(3), x(4), x(5), x({0,2}));
  arr grad = x({0,2}) - closest;
  double d = length(grad);
  grad /= d;
  d -= x(6);
  if(&g){
    g.resize(14);
    g.setZero();
    g({0,2}) = grad;
    g({7,9}) = - grad;
    g({3,5}) = - signs%(t.rot / mlr::Vector(grad)).getArr();
    g(6) = -1.;
    g({10,13}) = ~grad*crossProduct(t.rot.getJacobian(), (x({0,2})-t.pos.getArr()));
    g({10,13})() /= -sqrt(sumOfSqr(x({10,13}))); //account for the potential non-normalization of q
  }
  return d;
};





/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#include <algorithm>
#include <Core/array.h>
#include "geo.h"

#ifndef MLR_NO_REGISTRY
#include <Core/graph.h>
REGISTER_TYPE(T, mlr::Transformation);
#endif


#include <GL/glu.h>

const mlr::Vector Vector_x(1, 0, 0);
const mlr::Vector Vector_y(0, 1, 0);
const mlr::Vector Vector_z(0, 0, 1);
const mlr::Transformation Transformation_Id(mlr::Transformation().setZero());
const mlr::Quaternion Quaternion_Id(1, 0, 0, 0);
const mlr::Quaternion Quaternion_x(MLR_SQRT2/2., MLR_SQRT2/2., 0, 0);
const mlr::Quaternion Quaternion_y(MLR_SQRT2/2., 0, MLR_SQRT2/2., 0);
const mlr::Quaternion Quaternion_z(MLR_SQRT2/2., 0, 0, MLR_SQRT2/2.);
mlr::Vector& NoVector = *((mlr::Vector*)NULL);
mlr::Transformation& NoTransformation = *((mlr::Transformation*)NULL);

namespace mlr {

double scalarProduct(const mlr::Quaternion& a, const mlr::Quaternion& b);

double& Vector::operator()(uint i) {
  CHECK(i<3,"out of range");
  isZero=false;
  return (&x)[i];
}

/// set the vector
void Vector::set(double _x, double _y, double _z) { x=_x; y=_y; z=_z; isZero=(x==0. && y==0. && z==0.); }

/// set the vector
void Vector::set(double* p) { x=p[0]; y=p[1]; z=p[2]; isZero=(x==0. && y==0. && z==0.); }

/// set the vector
void Vector::setZero() { memset(this, 0, sizeof(Vector)); isZero=true; }

/// a random vector in [-1, 1]^3
void Vector::setRandom(double range) { x=rnd.uni(-range, range); y=rnd.uni(-range, range); z=rnd.uni(-range, range); isZero=false; }

//{ vector operations

/// this=this/length(this)
void Vector::normalize() {
  if(isZero){
    MLR_MSG("can't normalize length of null vector");
  }
  (*this)/=length();
}

/// this=this*l/length(this)
void Vector::setLength(double l) {
  if(isZero) MLR_MSG("can't change length of null vector");
  (*this)*=l/length();
}

/// this=component of this normal to \c b, (unnormalized!)
void Vector::makeNormal(const Vector& b) {
  if(b.isZero) MLR_MSG("can't makeNormal with null vector");
  double l=b.length(), s=x*b.x+y*b.y+z*b.z;
  s/=l*l;
  x-=s*b.x; y-=s*b.y; z-=s*b.z;
}

/// this=component of this colinear to \c b, (unnormalized!)
void Vector::makeColinear(const Vector& b) {
  if(b.isZero) MLR_MSG("can't makeColinear with null vector");
  // *this = ((*this)*b)/b.length()) * (*this);
  double l=b.length(), s=x*b.x+y*b.y+z*b.z;
  s/=l*l;
  x=s*b.x; y=s*b.y; z=s*b.z;
}

//{ measuring the vector

/// L1-norm to zero
double Vector::diffZero() const { return fabs(x)+fabs(y)+fabs(z); }

/// is it normalized?
bool Vector::isNormalized() const { return fabs(lengthSqr()-1.)<1e-6; }

/// returns the length of this
double Vector::length() const { return ::sqrt(lengthSqr()); }

/// returns the square of length |a|^2
double Vector::lengthSqr() const { return x*x + y*y + z*z; }

/// angle in [0..pi] between this and b
double Vector::angle(const Vector& b) const {
  double a=((*this)*b)/(length()*b.length());
  if(a<-1.) a=-1.;
  if(a>1.) a=1.;
  return ::acos(a);
}

/** @brief if \c this and \c b are colinear, it returns the factor c such
    that this=c*b; otherwise it returns zero */
double Vector::isColinear(const Vector& b) const {
  double c=x/b.x;
  if(y==c*b.y && z==c*b.z) return c;
  return 0.;
}

//{ sphere coordinates

/// the radius in the x/y-plane
double Vector::radius() const { return ::sqrt(x*x+y*y); }

/// the angle in the x/y-plane in [-pi, pi]
double Vector::phi() const {
  double ph;
  if(x==0. || ::fabs(x)<1e-10) ph=MLR_PI/2.; else ph=::atan(y/x);
  if(x<0.) { if(y<0.) ph-=MLR_PI; else ph+=MLR_PI; }
  return ph;
}

/// the angle from the x/y-plane
double Vector::theta() const { return ::atan(z/radius())+MLR_PI/2.; }

Vector Vector::getNormalVectorNormalToThis() const {
  if(isZero){
    MLR_MSG("every vector is normal to a zero vector");
  }
  arr s = ARR(fabs(x), fabs(y), fabs(z));
  uint c = s.maxIndex();
  double xv, yv, zv;
  if(c == 0) {
    xv = -(y+z)/x;
    yv = 1.0;
    zv = 1.0;
  } else if(c == 1) {
    xv = 1.0;
    yv = -(x+z)/y;
    zv = 1.0;
  } else {
    xv = 1.0;
    yv = 1.0;
    zv = -(x+y)/z;
  }
  Vector v(xv,yv,zv);
  v.normalize();
  return v;
}

void Vector::generateOrthonormalSystem(Vector& u, Vector& v) const {
  u = getNormalVectorNormalToThis();
  v = (*this)^u;
  v.normalize();
}

arr Vector::generateOrthonormalSystemMatrix() const {
  arr V;
  Vector n = *this;
  n.normalize();
  Vector u = getNormalVectorNormalToThis();
  Vector v = n^u;
  v.normalize();
  V.append(~conv_vec2arr(n));
  V.append(~conv_vec2arr(u));
  V.append(~conv_vec2arr(v));
  return ~V;
}

//{ I/O
void Vector::write(std::ostream& os) const {
  if(!mlr::IOraw) os <<'(' <<x <<' ' <<y <<' ' <<z <<')';
  else os <<' ' <<x <<' ' <<y <<' ' <<z;
}

void Vector::read(std::istream& is) {
  if(!mlr::IOraw) is >>PARSE("(") >>x >>y >>z >>PARSE(")");
  else is >>x >>y >>z;
}
//}

/// scalar product (inner product)
double operator*(const Vector& a, const Vector& b) {
  return a.x*b.x+a.y*b.y+a.z*b.z;
}

/// cross product (corresponds to antisymmetric exterior product)
Vector operator^(const Vector& b, const Vector& c) {
  Vector a;
  a.x=b.y*c.z-b.z*c.y;
  a.y=b.z*c.x-b.x*c.z;
  a.z=b.x*c.y-b.y*c.x;
  a.isZero = (a.x==0. && a.y==0. && a.z==0.);
  return a;
}

/// sum of two vectors
Vector operator+(const Vector& b, const Vector& c) {
  Vector a;
  a.x=b.x+c.x;
  a.y=b.y+c.y;
  a.z=b.z+c.z;
  a.isZero = (a.x==0. && a.y==0. && a.z==0.);
  return a;
}

/// difference between two vectors
Vector operator-(const Vector& b, const Vector& c) {
  Vector a;
  a.x=b.x-c.x;
  a.y=b.y-c.y;
  a.z=b.z-c.z;
  a.isZero = (a.x==0. && a.y==0. && a.z==0.);
  return a;
}

/// multiplication with a scalar
Vector operator*(double b, const Vector& c) {
  Vector a;
  a.x=b*c.x;
  a.y=b*c.y;
  a.z=b*c.z;
  a.isZero = c.isZero && (b==0.);
  return a;
}

/// multiplication with a scalar
Vector operator*(const Vector& b, double c) { return c*b; }

/// division by a scalar
Vector operator/(const Vector& b, double c) { return (1./c)*b; }

/// multiplication with a scalar
Vector& operator*=(Vector& a, double c) {
  a.x*=c; a.y*=c; a.z*=c;
  a.isZero = a.isZero && (c==0.);
  return a;
}

/// divide by a scalar
Vector& operator/=(Vector& a, double c) {
  a.x/=c; a.y/=c; a.z/=c;
  return a;
}

/// add a vector
Vector& operator+=(Vector& a, const Vector& b) {
  a.x+=b.x; a.y+=b.y; a.z+=b.z;
  a.isZero = (a.x==0. && a.y==0. && a.z==0.);
  return a;
}

/// subtract a vector
Vector& operator-=(Vector& a, const Vector& b) {
  a.x-=b.x; a.y-=b.y; a.z-=b.z;
  a.isZero = (a.x==0. && a.y==0. && a.z==0.);
  return a;
}

/// return the negative of a vector
Vector operator-(const Vector& b) {
  Vector a;
  a.x=-b.x; a.y=-b.y; a.z=-b.z;
  a.isZero = b.isZero;
  return a;
}

// all operator== and operator!=
bool operator==(const Quaternion& lhs, const Quaternion& rhs) {
  return lhs.w == rhs.w && lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
}

bool operator!=(const Quaternion& lhs, const Quaternion& rhs) {
  return !(lhs == rhs);
}

bool operator==(const Transformation& lhs, const Transformation& rhs) {
  if(!&rhs) return !&lhs; //if rhs==NoArr
  return lhs.pos == rhs.pos && lhs.rot == rhs.rot;
}

bool operator==(const DynamicTransformation& lhs, const DynamicTransformation& rhs) {
  bool vel_equal = false;
  if(lhs.zeroVels == rhs.zeroVels && rhs.zeroVels == false)
    vel_equal = lhs.vel == rhs.vel && lhs.angvel == rhs.angvel;
  else if(lhs.zeroVels == rhs.zeroVels && rhs.zeroVels == true)
    vel_equal = true;
  return vel_equal && lhs.pos == rhs.pos && lhs.rot == rhs.rot;
}

bool operator!=(const Transformation& lhs, const Transformation& rhs) {
  return !(lhs == rhs);
}

bool operator==(const Matrix& lhs, const Matrix& rhs) {
  return lhs.m00 == rhs.m00 && lhs.m01 == rhs.m01 && lhs.m02 == rhs.m02 &&
         lhs.m10 == rhs.m10 && lhs.m11 == rhs.m11 && lhs.m12 == rhs.m12 &&
         lhs.m20 == rhs.m20 && lhs.m21 == rhs.m21 && lhs.m22 == rhs.m22;
}

bool operator!=(const Matrix& lhs, const Matrix& rhs) {
  return !(lhs == rhs);
}

bool operator==(const Vector& lhs, const Vector& rhs) {
  return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
}

bool operator!=(const Vector& lhs, const Vector& rhs) {
  return !(lhs == rhs);
}

//==============================================================================

/// reset to zero
void Matrix::setZero() { memset(this, 0, sizeof(Matrix)); }

void Matrix::setRandom(double range) {
  for(uint i=0; i<9; i++) p()[i]=rnd.uni(-range, range);
}

/// reset to identity
void Matrix::setId() {
  m00=m11=m22=1.;
  m01=m02=m10=m12=m20=m21=0.;
}

/// set the matrix
void Matrix::set(double* p) {
  m00=p[0]; m01=p[1]; m02=p[2];
  m10=p[3]; m11=p[4]; m12=p[5];
  m20=p[6]; m21=p[7]; m22=p[8];
}

/// assign the matrix to the transformation from unit frame to given XYZ frame
void Matrix::setFrame(Vector& X, Vector& Y, Vector& Z) {
  m00=X.x; m01=Y.x; m02=Z.x;
  m10=X.y; m11=Y.y; m12=Z.y;
  m20=X.z; m21=Y.z; m22=Z.z;
}

/// assign the matrix to the transformation from the ORTHOGONAL XYZ frame to the unit frame
void Matrix::setInvFrame(Vector& X, Vector& Y, Vector& Z) {
  m00=X.x; m01=X.y; m02=X.z;
  m10=Y.x; m11=Y.y; m12=Y.z;
  m20=Z.x; m21=Z.y; m22=Z.z;
}

/// assign the matrix to a rotation around the X-axis with angle a (in rad units)
void Matrix::setXrot(double a) {
  m00=1.; m01=0.;     m02=0.;
  m10=0.; m11=cos(a); m12=-sin(a);
  m20=0.; m21=sin(a); m22= cos(a);
}

void Matrix::setSkew(const Vector& a) {
  m00=  0.; m01=-a.z; m02= a.y;
  m10= a.z; m11=  0.; m12=-a.x;
  m20=-a.y; m21= a.x; m22=  0.;
}

void Matrix::setExponential(const Vector& a) {
  Matrix S;
  double phi=a.length();
  if(phi<1e-10) { setId(); return; }
  S.setSkew(a/phi);
  *this = sin(phi)*S + (1.-cos(phi))*S*S;
  m00+=1.; m11+=1.; m22+=1.;
}

void Matrix::setOdeMatrix(double* o) {
  m00=o[0]; m01=o[1]; m02=o[2];
  m10=o[4]; m11=o[5]; m12=o[6];
  m20=o[8]; m21=o[9]; m22=o[10];
}

void Matrix::setTensorProduct(const Vector& b, const Vector& c) {
  m00=b.x*c.x; m01=b.x*c.y; m02=b.x*c.z;
  m10=b.y*c.x; m11=b.y*c.y; m12=b.y*c.z;
  m20=b.z*c.x; m21=b.z*c.y; m22=b.z*c.z;
}

/// 1-norm to zero
double Matrix::diffZero() const {
  double d=0.;
  for(uint i=0; i<9; i++) d += (&m00)[i];
  return d;
}

void Matrix::write(std::ostream& os) const {
  os <<"\n " <<m00 <<' ' <<m01 <<' ' <<m02;
  os <<"\n " <<m10 <<' ' <<m11 <<' ' <<m12;
  os <<"\n " <<m20 <<' ' <<m21 <<' ' <<m22;
  os <<endl;
}
void Matrix::read(std::istream& is) {
  NIY;
}
//}

/// multiplication of two matrices
Matrix operator*(const Matrix& b, const Matrix& c) {
  Matrix a;
  a.m00=b.m00*c.m00+b.m01*c.m10+b.m02*c.m20;
  a.m01=b.m00*c.m01+b.m01*c.m11+b.m02*c.m21;
  a.m02=b.m00*c.m02+b.m01*c.m12+b.m02*c.m22;
  
  a.m10=b.m10*c.m00+b.m11*c.m10+b.m12*c.m20;
  a.m11=b.m10*c.m01+b.m11*c.m11+b.m12*c.m21;
  a.m12=b.m10*c.m02+b.m11*c.m12+b.m12*c.m22;
  
  a.m20=b.m20*c.m00+b.m21*c.m10+b.m22*c.m20;
  a.m21=b.m20*c.m01+b.m21*c.m11+b.m22*c.m21;
  a.m22=b.m20*c.m02+b.m21*c.m12+b.m22*c.m22;
  return a;
}
/// sum of two matrices
Matrix operator+(const Matrix& b, const Matrix& c) {
  Matrix a;
  a.m00=b.m00+c.m00; a.m01=b.m01+c.m01; a.m02=b.m02+c.m02;
  a.m10=b.m10+c.m10; a.m11=b.m11+c.m11; a.m12=b.m12+c.m12;
  a.m20=b.m20+c.m20; a.m21=b.m21+c.m21; a.m22=b.m22+c.m22;
  return a;
}
/// transformation of a vector
Vector operator*(const Matrix& b, const Vector& c) {
  Vector a;
  a.x=b.m00*c.x+b.m01*c.y+b.m02*c.z;
  a.y=b.m10*c.x+b.m11*c.y+b.m12*c.z;
  a.z=b.m20*c.x+b.m21*c.y+b.m22*c.z;
  a.isZero = (a.x==0. && a.y==0. && a.z==0.);
  return a;
}
/// multiplication with a scalar
Matrix& operator*=(Matrix& a, double c) {
  a.m00*=c; a.m01*=c; a.m02*=c;
  a.m10*=c; a.m11*=c; a.m12*=c;
  a.m20*=c; a.m21*=c; a.m22*=c;
  return a;
}
/// multiplication with scalar
Matrix operator*(double b, const Matrix& c) {
  Matrix a;
  a=c;
  a*=b;
  return a;
}
/// sum of two matrices
Matrix& operator+=(Matrix& a, const Matrix& b) {
  a.m00+=b.m00; a.m01+=b.m01; a.m02+=b.m02;
  a.m10+=b.m10; a.m11+=b.m11; a.m12+=b.m12;
  a.m20+=b.m20; a.m21+=b.m21; a.m22+=b.m22;
  return a;
}

//==============================================================================

/// inverts the current rotation
Quaternion& Quaternion::invert() { w=-w; return *this; }

/// flips the sign of the quaterion -- which still represents the same rotation
void Quaternion::flipSign() { w=-w; x=-x; y=-y; z=-z; }

/// multiplies the rotation by a factor f (i.e., makes f-times the rotation)
void Quaternion::multiply(double f) {
  if(w==1. || f==1.) return;
  double phi=acos(w);
  phi*=f;
  w=cos(phi);
  f=sin(phi)/sqrt(x*x + y*y + z*z);
  x*=f; y*=f; z*=f;
}

double Quaternion::normalization() const{
  return sqrt(w*w + x*x + y*y + z*z);
}

bool Quaternion::isNormalized() const {
  double n=w*w + x*x + y*y + z*z;
  return fabs(n-1.)<1e-6;
}

void Quaternion::normalize() {
  double n=w*w + x*x + y*y + z*z;
  n=sqrt(n);
  w/=n; x/=n; y/=n; z/=n;
  isZero=(w==1. || w==-1.);
}

/** @brief roughly, removes all ``components'' of the rotation that are not
    around the given vector v. More precisely, aligns/projects
    the rotation axis (given by q[1], q[2], q[3] of the quaternion)
    with v and re-normalizes afterwards. */
void Quaternion::alignWith(const Vector& v) {
  double s=x*v.x + y*v.y + z*v.z;
  if(!s) { setZero(); return; }  // are orthogonal
  s/=v*v;
  x=s*v.x; y=s*v.y; z=s*v.z;
  normalize();
}

void Quaternion::addX(double angle){
  if(!angle){ return; }
  angle/=2.;
  double cw=cos(angle);
  double cx=sin(angle);

  Quaternion a;
  a.w = w*cw - x*cx;
  a.x = w*cx + x*cw;
  a.y = y*cw + z*cx;
  a.z = z*cw - y*cx;

  set(a.w, a.x, a.y, a.z);
}

void Quaternion::addY(double angle){
  if(!angle){ return; }
  angle/=2.;
  double cw=cos(angle);
  double cy=sin(angle);

  Quaternion a;
  a.w = w*cw - y*cy;
  a.x = x*cw - z*cy;
  a.y = w*cy + y*cw;
  a.z = z*cw + x*cy;

  set(a.w, a.x, a.y, a.z);
}

void Quaternion::addZ(double radians){
  if(!radians){ return; }
  radians/=2.;
  double cw=cos(radians);
  double cz=sin(radians);

  Quaternion a;
  a.w = w*cw - z*cz;
  a.x = x*cw + y*cz;
  a.y = y*cw - x*cz;
  a.z = w*cz + z*cw;

  set(a.w, a.x, a.y, a.z);
}

void Quaternion::append(const Quaternion& q){
  if(q.isZero) return;
  double aw = w*q.w;
  double ax = x*q.w;
  double ay = y*q.w;
  double az = z*q.w;
  if(q.x){ aw -= x*q.x;  ax += w*q.x;  ay += z*q.x;  az -= y*q.x; }
  if(q.y){ aw -= y*q.y;  ax -= z*q.y;  ay += w*q.y;  az += x*q.y; }
  if(q.z){ aw -= z*q.z;  ax += y*q.z;  ay -= x*q.z;  az += w*q.z; }
  w=aw; x=ax; y=ay; z=az; isZero=false;
}

/// set the quad
void Quaternion::set(double* p) { w=p[0]; x=p[1]; y=p[2]; z=p[3]; isZero=(w==1. || w==-1.); }

/// set the quad
void Quaternion::set(const arr& q) { CHECK_EQ(q.N,4, "");  set(q.p); }

/// set the quad
void Quaternion::set(double _w, double _x, double _y, double _z) { w=_w; x=_x; y=_y; z=_z; isZero=(w==1. || w==-1.); }

/// reset the rotation to identity
void Quaternion::setZero() { memset(this, 0, sizeof(Quaternion));  w=1.; isZero=true; }

/// samples the rotation uniformly from the whole SO(3)
void Quaternion::setRandom() {
  double s, s1, s2, t1, t2;
  s=rnd.uni();
  s1=sqrt(1-s);
  s2=sqrt(s);
  t1=MLR_2PI*rnd.uni();
  t2=MLR_2PI*rnd.uni();
  w=cos(t2)*s2;
  x=sin(t1)*s1;
  y=cos(t1)*s1;
  z=sin(t2)*s2;
  isZero=false;
}

/// sets this to a smooth interpolation between two rotations
void Quaternion::setInterpolate(double t, const Quaternion& a, const Quaternion b) {
  double sign=1.;
  if(scalarProduct(a, b)<0) sign=-1.;
  w=a.w+t*(sign*b.w-a.w);
  x=a.x+t*(sign*b.x-a.x);
  y=a.y+t*(sign*b.y-a.y);
  z=a.z+t*(sign*b.z-a.z);
  normalize();
  isZero=false;
}

/// assigns the rotation to \c a DEGREES around the vector (x, y, z)
void Quaternion::setDeg(double degree, double _x, double _y, double _z) { setRad(degree*MLR_PI/180., _x, _y, _z); }

void Quaternion::setDeg(double degree, const Vector& vec) { setRad(degree*MLR_PI/180., vec.x, vec.y, vec.z); }

/// assigns the rotation to \c a RADIANTS (2*PI-units) around the vector (x, y, z)
void Quaternion::setRad(double angle, double _x, double _y, double _z) {
  if(!angle){ setZero(); return; }
  double l = _x*_x + _y*_y + _z*_z;
  if(l<1e-15) { setZero(); return; }
  angle/=2.;
  l=sin(angle)/sqrt(l);
  w=cos(angle);
  x=_x*l;
  y=_y*l;
  z=_z*l;
  isZero=false;
}

/// ..
void Quaternion::setRad(double angle, const Vector &axis) {
  setRad(angle, axis.x, axis.y, axis.z);
}

/// assigns the rotation to \c a RADIANTS (2*PI-units) around the current axis
void Quaternion::setRad(double angle) {
  if(!angle){ setZero(); return; }
  double l = x*x + y*y + z*z;
  if(l<1e-15) { setZero(); return; }
  angle/=2.;
  l=sin(angle)/sqrt(l);
  w=cos(angle);
  x*=l;
  y*=l;
  z*=l;
  isZero=false;
}

/// rotation around X-axis by given radiants
void Quaternion::setRadX(double angle) {
  if(!angle){ setZero(); return; }
  angle/=2.;
  w=cos(angle);
  x=sin(angle);
  y=z=0.;
  isZero=false;
}

/// rotation around Y-axis by given radiants
void Quaternion::setRadY(double angle) {
  if(!angle){ setZero(); return; }
  angle/=2.;
  w=cos(angle);
  y=sin(angle);
  x=z=0.;
  isZero=false;
}

/// rotation around Z-axis by given radiants
void Quaternion::setRadZ(double angle) {
  if(!angle){ setZero(); return; }
  angle/=2.;
  w=cos(angle);
  z=sin(angle);
  x=y=0.;
  isZero=false;
}

Quaternion& Quaternion::setRpy(double r, double p, double y) {
#if 1
  Quaternion q;
  setZero();
  q.setRadZ(y); *this = *this * q;
  q.setRadY(p); *this = *this * q;
  q.setRadX(r); *this = *this * q;
  return *this;
#else
  double cr=::cos(.5*r), sr=::sin(.5*r);
  double cp=::cos(.5*p), sp=::sin(.5*p);
  double cy=::cos(.5*y), sy=::sin(.5*y);
  w = cr*cp*cy + sr*sp*sy;
  x = sr*cp*cy - cr*sp*sy;
  y = cr*sp*cy + sr*cp*sy;
  z = cr*cp*sy - sr*sp*cy;
#endif
  isZero=(w==1. || w==-1.);
  CHECK(isNormalized(),"bad luck");
  return *this;
}

/// rotation around the given vector with angle (in rad) equal to norm of the vector
void Quaternion::setVec(Vector w) {
  double phi=w.length();
  setRad(phi, w.x, w.y, w.z);
}

/// rotation that will rotate 'from' to 'to' on direct path
void Quaternion::setDiff(const Vector& from, const Vector& to) {
  double phi=acos(from*to/(from.length()*to.length()));
  if(!phi){ setZero(); return; }
  Vector axis(from^to);
  if(axis.isZero) axis=Vector(0, 0, 1)^to;
  setRad(phi, axis);
}

/// L1-norm to zero (i.e., identical rotation)
double Quaternion::diffZero() const { return (w>0.?fabs(w-1.):fabs(w+1.))+fabs(x)+fabs(y)+fabs(z); }

double Quaternion::sqrDiffZero() const { return (w>0.?mlr::sqr(w-1.):mlr::sqr(w+1.))+mlr::sqr(x)+mlr::sqr(y)+mlr::sqr(z); }

/// return the squared-error between two quads, modulo flipping
double Quaternion::sqrDiff(const Quaternion& _q2) const{
  arr q1(&w, 4, true);
  arr q2(&_q2.w, 4, true);
  if(scalarProduct(q1,q2)>=0) return sqrDistance(q1, q2);
  return sqrDistance(-q1,q2);
}

/// gets rotation angle (in rad [0, 2pi])
double Quaternion::getRad() const {
  if(w>=1. || w<=-1. || (x==0. && y==0. && z==0.)) return 0;
  return 2.*acos(w);
}

/// gets rotation angle (in degree [0, 360])
double Quaternion::getDeg() const {
  if(w>=1. || w<=-1. || (x==0. && y==0. && z==0.)) return 0;
  return 360./MLR_PI*acos(w);
}

/// gets rotation angle (in degree [0, 360]) and vector
void Quaternion::getDeg(double& degree, Vector& vec) const {
  if(w>=1. || w<=-1. || (x==0. && y==0. && z==0.)) { degree=0.; vec.set(0., 0., 1.); return; }
  degree=acos(w);
  double s=sin(degree);
  degree*=360./MLR_PI;
  vec.x=x/s; vec.y=y/s; vec.z=z/s;
}

/// gets rotation angle (in rad [0, 2pi]) and vector
void Quaternion::getRad(double& angle, Vector& vec) const {
  if(w>=1. || w<=-1. || (x==0. && y==0. && z==0.)) { angle=0.; vec.set(0., 0., 1.); return; }
  angle=acos(w);
  double s=1./sin(angle);
  angle*=2;
  vec.x=s*x; vec.y=s*y; vec.z=s*z;
  CHECK(angle>=0. && angle<=MLR_2PI, "");
}

/// gets the axis rotation vector with length equal to the rotation angle in rad
Vector Quaternion::getVec() const {
  Vector vec;
  if(w>=1. || w<=-1. || (x==0. && y==0. && z==0.)) { vec.setZero(); return vec; }
  double phi=acos(w);
  double s=2.*phi/sin(phi);
  vec.x=s*x; vec.y=s*y; vec.z=s*z;
  return vec;
}

Vector Quaternion::getX() const {
  Vector Rx;
  double q22 = 2.*y*y;
  double q33 = 2.*z*z;
  double q12 = 2.*x*y;
  double q13 = 2.*x*z;
  double q02 = 2.*w*y;
  double q03 = 2.*w*z;
  Rx.x=1-q22-q33;
  Rx.y=q12+q03;
  Rx.z=q13-q02;
  return Rx;
}
Vector Quaternion::getY() const { return (*this)*Vector_y; }
Vector Quaternion::getZ() const { return (*this)*Vector_z; }

void Quaternion::setMatrix(double* m) {
  w = .5*sqrt(1.+m[0]+m[4]+m[8]); //sqrt(1.-(3.-(m[0]+m[4]+m[8]))/4.);
  z = (m[3]-m[1])/(4.*w);
  y = (m[2]-m[6])/(4.*w);
  x = (m[7]-m[5])/(4.*w);
  isZero=(w==1. || w==-1.);
  normalize();
  //CHECK(normalized(), "failed :-(");
}

/// exports the rotation to a double[9] matrix, row-by-row
Matrix Quaternion::getMatrix() const {
  Matrix R;
  double P1=2.*x, P2=2.*y, P3=2.*z;
  double q11 = x*P1;
  double q22 = y*P2;
  double q33 = z*P3;
  double q12 = x*P2;
  double q13 = x*P3;
  double q23 = y*P3;
  double q01 = w*P1;
  double q02 = w*P2;
  double q03 = w*P3;
  R.m00=1.-q22-q33; R.m01=q12-q03;     R.m02=q13+q02;
  R.m10=q12+q03;    R.m11=1.-q11-q33;  R.m12=q23-q01;
  R.m20=q13-q02;    R.m21=q23+q01;     R.m22=1.-q11-q22;
  return R;
}

arr Quaternion::getArr() const {
  arr R(3,3);
  getMatrix(R.p);
  return R;
}

double* Quaternion::getMatrix(double* m) const {
  double P1=2.*x, P2=2.*y, P3=2.*z;
  double q11 = x*P1;
  double q22 = y*P2;
  double q33 = z*P3;
  double q12 = x*P2;
  double q13 = x*P3;
  double q23 = y*P3;
  double q01 = w*P1;
  double q02 = w*P2;
  double q03 = w*P3;
  m[0]=1.-q22-q33; m[1]=q12-q03;    m[2] =q13+q02;
  m[3]=q12+q03;    m[4]=1.-q11-q33; m[5] =q23-q01;
  m[6]=q13-q02;    m[7]=q23+q01;    m[8]=1.-q11-q22;
  return m;
}

/// exports the rotation to an ODE format matrix of type double[12]
double* Quaternion::getMatrixOde(double* m) const {
  double P1=2.*x, P2=2.*y, P3=2.*z;
  double q11 = x*P1;
  double q22 = y*P2;
  double q33 = z*P3;
  double q12 = x*P2;
  double q13 = x*P3;
  double q23 = y*P3;
  double q01 = w*P1;
  double q02 = w*P2;
  double q03 = w*P3;
  m[0]=1.-q22-q33; m[1]=q12-q03;    m[2] =q13+q02;
  m[4]=q12+q03;    m[5]=1.-q11-q33; m[6] =q23-q01;
  m[8]=q13-q02;    m[9]=q23+q01;    m[10]=1.-q11-q22;
  m[3]=m[7]=m[11]=0.;
  return m;
}

/// exports the rotation to an OpenGL format matrix of type double[16]
double* Quaternion::getMatrixGL(double* m) const {
  double P1=2.*x, P2=2.*y, P3=2.*z;
  double q11 = x*P1;
  double q22 = y*P2;
  double q33 = z*P3;
  double q12 = x*P2;
  double q13 = x*P3;
  double q23 = y*P3;
  double q01 = w*P1;
  double q02 = w*P2;
  double q03 = w*P3;
  m[0]=1.-q22-q33; m[4]=q12-q03;    m[8] =q13+q02;
  m[1]=q12+q03;    m[5]=1.-q11-q33; m[9] =q23-q01;
  m[2]=q13-q02;    m[6]=q23+q01;    m[10]=1.-q11-q22;
  m[3]=m[7]=m[11]=m[12]=m[13]=m[14]=0.;
  m[15]=1.;
  return m;
}

/// this is a 3-by-4 matrix $J$, giving the angular velocity vector $w = J \dot q$  induced by a $\dot q$
arr Quaternion::getJacobian() const{
  arr J(3,4);
  mlr::Quaternion e;
  for(uint i=0;i<4;i++){
    if(i==0) e.set(1.,0.,0.,0.);
    if(i==1) e.set(0.,1.,0.,0.);
    if(i==2) e.set(0.,0.,1.,0.);
    if(i==3) e.set(0.,0.,0.,1.);//TODO: the following could be simplified/compressed/made more efficient
    e = e / *this;
    J(0, i) = -2.*e.x;
    J(1, i) = -2.*e.y;
    J(2, i) = -2.*e.z;
  }
  return J;
}

/// this is a 4x(3x3) matrix, such that ~(J*x) is the jacobian of (R*x), and ~qdelta*J is (del R/del q)(qdelta)
arr Quaternion::getMatrixJacobian() const{
  arr J(4,9); //transpose!
  double r0=w, r1=x, r2=y, r3=z;
  J[0] = {      0,    -r3,     r2,
               r3,      0,    -r1,
              -r2,     r1,      0 };
  J[1] = {      0,     r2,     r3,
               r2, -2.*r1,    -r0,
               r3,     r0, -2.*r1 };
  J[2] = { -2.*r2,     r1,    r0,
               r1,      0,     r3,
              -r0,     r3, -2.*r2 };
  J[3] = { -2.*r3,    -r0,     r1,
               r0, -2.*r3,     r2,
               r1,     r2,      0};
  J *= 2.;
  J.reshape(4,3,3);
  return J;
}

void Quaternion::writeNice(std::ostream& os) const { os <<"Quaternion: " <<getDeg() <<" around " <<getVec() <<"\n"; }
void Quaternion::write(std::ostream& os) const {
  if(!mlr::IOraw) os <<'(' <<w <<' ' <<x <<' ' <<y <<' ' <<z <<')';
  else os <<' ' <<w <<' ' <<x <<' ' <<y <<' ' <<z;
}
void Quaternion::read(std::istream& is) { is >>PARSE("(") >>w >>x >>y  >>z >>PARSE(")"); normalize();}
//}

/// inverse rotation
Quaternion operator-(const Quaternion& b) {
  return Quaternion(b).invert();
}

/// compound of two rotations (A=B*C)
Quaternion operator*(const Quaternion& b, const Quaternion& c) {
  if(c.isZero) return b;
  if(b.isZero) return c;
  Quaternion a;
#if 0
  a.w = b.w*c.w - b.x*c.x - b.y*c.y - b.z*c.z;
  a.x = b.w*c.x + b.x*c.w + b.y*c.z - b.z*c.y;
  a.y = b.w*c.y + b.y*c.w + b.z*c.x - b.x*c.z;
  a.z = b.w*c.z + b.z*c.w + b.x*c.y - b.y*c.x;
#else
  a.w = b.w*c.w;
  a.x = b.x*c.w;
  a.y = b.y*c.w;
  a.z = b.z*c.w;
  if(c.x){ a.w -= b.x*c.x;  a.x += b.w*c.x;  a.y += b.z*c.x;  a.z -= b.y*c.x; }
  if(c.y){ a.w -= b.y*c.y;  a.x -= b.z*c.y;  a.y += b.w*c.y;  a.z += b.x*c.y; }
  if(c.z){ a.w -= b.z*c.z;  a.x += b.y*c.z;  a.y -= b.x*c.z;  a.z += b.w*c.z; }
#endif
  a.isZero=(a.w==1. || a.w==-1.);
  return a;
}

/// A=B*C^{-1}
Quaternion operator/(const Quaternion& b, const Quaternion& c) {
  Quaternion a;
  a.w =-b.w*c.w - b.x*c.x - b.y*c.y - b.z*c.z;
  a.x = b.w*c.x - b.x*c.w + b.y*c.z - b.z*c.y;
  a.y = b.w*c.y - b.y*c.w + b.z*c.x - b.x*c.z;
  a.z = b.w*c.z - b.z*c.w + b.x*c.y - b.y*c.x;
  a.isZero=(a.w==1. || a.w==-1.);
  return a;
}

void mult(Vector& a, const Quaternion& b, const Vector& c,bool add){
  if(c.isZero){
    if(!add) a.setZero();
    return;
  }
  double Bx=2.*b.x, By=2.*b.y, Bz=2.*b.z;
  double q11 = b.x*Bx;
  double q22 = b.y*By;
  double q33 = b.z*Bz;
  double q12 = b.x*By;
  double q13 = b.x*Bz;
  double q23 = b.y*Bz;
  double q01 = b.w*Bx;
  double q02 = b.w*By;
  double q03 = b.w*Bz;
  if(!add) a.x=a.y=a.z=0.;
  if(c.x){ a.x += (1.-q22-q33)*c.x; a.y += (q12+q03)*c.x; a.z += (q13-q02)*c.x; }
  if(c.y){ a.x += (q12-q03)*c.y; a.y += (1.-q11-q33)*c.y; a.z += (q23+q01)*c.y; }
  if(c.z){ a.x += (q13+q02)*c.z; a.y += (q23-q01)*c.z; a.z += (1.-q11-q22)*c.z; }
  a.isZero = false;
}

/// transform of a vector by a rotation
Vector operator*(const Quaternion& b, const Vector& c) {
  if(c.isZero) return Vector(0);
  Vector a;
  mult(a,b,c,false);
  return a;
}

/// inverse transform of a vector by a rotation
Vector operator/(const Quaternion& b, const Vector& c) {
  Matrix M = b.getMatrix();
  Vector a;
  a.x = M.m00*c.x + M.m10*c.y + M.m20*c.z;
  a.y = M.m01*c.x + M.m11*c.y + M.m21*c.z;
  a.z = M.m02*c.x + M.m12*c.y + M.m22*c.z;
  a.isZero = (a.x==0. && a.y==0. && a.z==0.);
  return a;
}

Transformation operator-(const Transformation& X) {
  Transformation Y;
  Y.setInverse(X);
  return Y;
}

Transformation operator*(const Transformation& X, const Transformation& c) {
  Transformation f(X);
  f.appendTransformation(c);
  return f;
}

Transformation operator/(const Transformation& X, const Transformation& c) {
  //TODO: check whether this is sensible, where is it used??
#if 0
  Transformation f(X);
  f.appendInvTransformation(c);
#else
  Transformation f;
  f.setDifference(c,X);
#endif
  return f;
}

/// transform of a vector by a frame
Vector operator*(const Transformation& X, const Vector& c) {
  Vector a;
  a = X.rot * c;
  a += X.pos;
  return a;
}

/// inverse transform of a vector by a frame
Vector operator/(const Transformation& X, const Vector& c) {
  Vector a(c);
  a -= X.pos;
  a = X.rot / a;
  return a;
}

//==============================================================================

/// initialize by reading from the string
Transformation& Transformation::setText(const char* txt) { read(mlr::String(txt).stream()); return *this; }

/// resets the position to origin, rotation to identity, velocities to zero, scale to unit
Transformation& Transformation::setZero() {
  memset(this, 0, sizeof(Transformation));
  rot.w = 1.;
  pos.isZero = rot.isZero = true;
  return *this;
}

/// randomize the frame
void Transformation::setRandom() {
  rot.setRandom();
  pos.setRandom();
}

/// move the turtle by the vector (x, z, y) WITH RESPECT TO the current orientation/scale
void Transformation::addRelativeTranslation(double x, double y, double z) {
  addRelativeTranslation(Vector(x, y, z));
}

void Transformation::addRelativeTranslation(const Vector& x_rel){
  pos += rot*x_rel;
}


/// rotate the turtle orientation
void Transformation::addRelativeRotation(const Quaternion& q) {
  rot = rot*q;
}

/// rotate the turtle orientation by an angle (given in DEGREE) around the vector (x, y, z) (given relative to the current orientation)
void Transformation::addRelativeRotationDeg(double degree, double x, double y, double z) {
  Quaternion R;
  R.setDeg(degree, x, y, z);
  rot = rot*R;
}

/// rotate the turtle orientation by an angle (given in radiants) around the vector (x, y, z) (given relative to the current orientation)
void Transformation::addRelativeRotationRad(double rad, double x, double y, double z) {
  Quaternion R;
  R.setRad(rad, x, y, z);
  rot = rot*R;
}

/// rotate the turtle orientation as given by a quaternion
void Transformation::addRelativeRotationQuat(double w, double x, double y, double z) {
  Quaternion R(w, x, y, z);
  rot = rot*R;
}

/** @brief transform the turtle into the frame f,
    which is interpreted RELATIVE to the current frame
    (new = old * f) */
void Transformation::appendTransformation(const Transformation& f) {
  if(!f.pos.isZero){
    if(rot.isZero) pos += f.pos;
    else mult(pos, rot, f.pos, true);
  }
  if(!f.rot.isZero){
    if(rot.isZero) rot = f.rot;
    else rot.append(f.rot);
  }
}

/// inverse transform (new = old * f^{-1})
void Transformation::appendInvTransformation(const Transformation& f) {
  rot = rot/f.rot;
  pos -= rot*f.pos;
}

/// this = f^{-1}
void Transformation::setInverse(const Transformation& f) {
  rot = -f.rot;
  pos = - (rot * f.pos);
}

/// set double[4*4] to Transformation. Matrix needs to be orthogonal
void Transformation::setAffineMatrix(const double *m) {
  double M[9];
  uint i, j;
  for(i=0; i<3; ++i)
    for(j=0; j<3; ++j)
      M[i*3+j] = m[i*4+j];
  rot.setMatrix(M);                 // set 3x3 submatrix as rotation
  pos.x=m[3];  // set last column as translation
  pos.y=m[7];  // set last column as translation
  pos.z=m[11];  // set last column as translation
}

///  to = new * from
void Transformation::setDifference(const Transformation& from, const Transformation& to) {
  rot = Quaternion_Id / from.rot * to.rot;
  pos = from.rot/(to.pos-from.pos);
}

/// get the current position/orientation/scale in an OpenGL format matrix (of type double[16])
double* Transformation::getAffineMatrix(double *m) const {
  Matrix M = rot.getMatrix();
  m[0] = M.m00; m[1] = M.m01; m[2] = M.m02; m[3] =pos.x;
  m[4] = M.m10; m[5] = M.m11; m[6] = M.m12; m[7] =pos.y;
  m[8] = M.m20; m[9] = M.m21; m[10]= M.m22; m[11]=pos.z;
  m[12]=0.;    m[13]=0.;    m[14]=0.;    m[15]=1.;
  return m;
}

arr Transformation::getAffineMatrix() const{
  arr T(4,4);
  getAffineMatrix(T.p);
  return T;
}

/// get inverse OpenGL matrix for this frame (of type double[16])
double* Transformation::getInverseAffineMatrix(double *m) const {
  Matrix M = rot.getMatrix();
  Vector pinv; pinv=rot/pos;
  m[0] =M.m00; m[1] =M.m10; m[2] =M.m20; m[3] =-pinv.x;
  m[4] =M.m01; m[5] =M.m11; m[6] =M.m21; m[7] =-pinv.y;
  m[8] =M.m02; m[9] =M.m12; m[10]=M.m22; m[11]=-pinv.z;
  m[12]=0.;   m[13]=0.;   m[14]=0.;   m[15]=1.;
  return m;
}

/// get the current position/orientation/scale in an OpenGL format matrix (of type double[16])
double* Transformation::getAffineMatrixGL(double *m) const {
  Matrix M = rot.getMatrix();
  m[0]=M.m00; m[4]=M.m01; m[8] =M.m02; m[12]=pos.x;
  m[1]=M.m10; m[5]=M.m11; m[9] =M.m12; m[13]=pos.y;
  m[2]=M.m20; m[6]=M.m21; m[10]=M.m22; m[14]=pos.z;
  m[3]=0.;   m[7]=0.;   m[11]=0.;   m[15]=1.;
  return m;
}

/// get inverse OpenGL matrix for this frame (of type double[16]) */
double* Transformation::getInverseAffineMatrixGL(double *m) const {
  Matrix M = rot.getMatrix();
  Vector pinv; pinv=rot/pos;
  m[0]=M.m00; m[4]=M.m10; m[8] =M.m20; m[12]=-pinv.x;
  m[1]=M.m01; m[5]=M.m11; m[9] =M.m21; m[13]=-pinv.y;
  m[2]=M.m02; m[6]=M.m12; m[10]=M.m22; m[14]=-pinv.z;
  m[3]=0.;   m[7]=0.;   m[11]=0.;   m[15]=1.;
  return m;
}

arr Transformation::getArr7d(){
  arr t(7);
  t.p[0]=pos.x;
  t.p[1]=pos.y;
  t.p[2]=pos.z;
  t.p[3]=rot.w;
  t.p[4]=rot.x;
  t.p[5]=rot.y;
  t.p[6]=rot.z;
  return t;
}

void Transformation::applyOnPointArray(arr& pts) const{
  if(!((pts.nd==2 && pts.d1==3) || (pts.nd==3 && pts.d2==3))){
    LOG(-1) <<"wrong pts dimensions for transformation:" <<pts.dim();
    return;
  }
  arr R = ~rot.getArr(); //transposed, only to make it applicable to an n-times-3 array
  arr t = conv_vec2arr(pos);
  pts = pts * R;
  for(double *p=pts.p, *pstop=pts.p+pts.N; p<pstop; p+=3){
    p[0] += pos.x;
    p[1] += pos.y;
    p[2] += pos.z;
  }
// for(uint i=0;i<pts.d0;i++) pts[i]() += t; //inefficient...
}

bool Transformation::isZero() const {
  return pos.isZero && rot.isZero;
}

/// L1-norm to zero
double Transformation::diffZero() const {
  return pos.diffZero() + rot.diffZero();
}

/// operator<<
void Transformation::write(std::ostream& os) const {
  os <<pos.x <<' ' <<pos.y <<' ' <<pos.z <<' '
     <<rot.w <<' ' <<rot.x <<' ' <<rot.y <<' ' <<rot.z;
}

/// operator>>
void Transformation::read(std::istream& is) {
  setZero();
  char c;
  double x[4];
  mlr::skip(is, " \n\r\t<|");
  for(;;) {
    is >>c;
    if(is.fail()) return;  //EOF I guess
    //if(c==';') break;
    //if(c==',') is >>c;
    if((c>='0' && c<='9') || c=='.' || c=='-') {  //read a 7-vector (pos+quat) for the transformation
      is.putback(c);
      is>>x[0]>>x[1]>>x[2];       addRelativeTranslation(x[0], x[1], x[2]);
      is>>x[0]>>x[1]>>x[2]>>x[3]; addRelativeRotationQuat(x[0], x[1], x[2], x[3]);
    } else switch(c) {
          //case '<': break; //do nothing -- assume this is an opening tag
        case 't': is>>PARSE("(")>>x[0]>>x[1]>>x[2]>>PARSE(")");       addRelativeTranslation(x[0], x[1], x[2]); break;
        case 'q': is>>PARSE("(")>>x[0]>>x[1]>>x[2]>>x[3]>>PARSE(")"); addRelativeRotationQuat(x[0], x[1], x[2], x[3]); break;
        case 'r': is>>PARSE("(")>>x[0]>>x[1]>>x[2]>>x[3]>>PARSE(")"); addRelativeRotationRad(x[0], x[1], x[2], x[3]); break;
        case 'd': is>>PARSE("(")>>x[0]>>x[1]>>x[2]>>x[3]>>PARSE(")"); addRelativeRotationDeg(x[0], x[1], x[2], x[3]); break;
        case 'E': is>>PARSE("(")>>x[0]>>x[1]>>x[2]>>PARSE(")"); addRelativeRotation(Quaternion().setRpy(x[0], x[1], x[2])); break;
          //case 's': is>>PARSE("(")>>x[0]>>PARSE(")");                   scale(x[0]); break;
        case '|':
        case '>': is.putback(c); return; //those symbols finish the reading without error
        default: MLR_MSG("unknown Transformation read tag: " <<c <<"abort reading this frame"); is.putback(c); return;
      }
    if(is.fail()) HALT("error reading '" <<c <<"' parameters in frame");
  }
  if(is.fail()) HALT("could not read Transformation struct");
}

//==============================================================================

/// initialize by reading from the string
DynamicTransformation& DynamicTransformation::setText(const char* txt) { read(mlr::String(txt)()); return *this; }

/// resets the position to origin, rotation to identity, velocities to zero, scale to unit
DynamicTransformation& DynamicTransformation::setZero() {
  memset(this, 0, sizeof(DynamicTransformation));
  rot.w = 1.;
  pos.isZero = rot.isZero = vel.isZero = angvel.isZero = true;
  zeroVels = true;
  return *this;
}

/// randomize the frame
void DynamicTransformation::setRandom() {
  rot.setRandom();
  pos.setRandom();
  if(rnd.uni()<.8) {
    vel.setZero(); angvel.setZero(); zeroVels=true;
  } else {
    vel.setRandom(); angvel.setRandom(); zeroVels = false;
  }
}

/// move the turtle by the vector (x, z, y) WITH RESPECT TO the current orientation/scale
void DynamicTransformation::addRelativeTranslation(double x, double y, double z) {
  addRelativeTranslation(Vector(x, y, z));
}

void DynamicTransformation::addRelativeTranslation(const Vector& x_rel){
  Vector x = rot*x_rel;
  pos+=x;
  if(!zeroVels) vel+=angvel^x;
}

/// add a velocity to the turtle's inertial frame
void DynamicTransformation::addRelativeVelocity(double x, double y, double z) {
  Vector X(x, y, z);
  //v+=r*(s*X);
  vel+=rot*X;
  zeroVels = false;
}

/// add an angular velocity to the turtle inertial frame
void DynamicTransformation::addRelativeAngVelocityDeg(double degree, double x, double y, double z) {
  Vector W(x, y, z); W.normalize();
  W*=degree*MLR_PI/180.;
  angvel+=rot*W;
  zeroVels = false;
}

/// add an angular velocity to the turtle inertial frame
void DynamicTransformation::addRelativeAngVelocityRad(double rad, double x, double y, double z) {
  Vector W(x, y, z); W.normalize();
  W*=rad;
  angvel+=rot*W;
  zeroVels = false;
}

/// add an angular velocity to the turtle inertial frame
void DynamicTransformation::addRelativeAngVelocityRad(double wx, double wy, double wz) {
  Vector W(wx, wy, wz);
  angvel+=rot*W;
  zeroVels = false;
}


/** @brief transform the turtle into the frame f,
    which is interpreted RELATIVE to the current frame
    (new = f * old) */
void DynamicTransformation::appendTransformation(const DynamicTransformation& f) {
  if(zeroVels && f.zeroVels) {
    if(!f.pos.isZero){ if(rot.isZero) pos += f.pos; else pos += rot*f.pos; }
    if(!f.rot.isZero){ if(rot.isZero) rot = f.rot; else rot = rot*f.rot; }
  } else {
    //Vector P(r*(s*f.p)); //relative offset in global coords
    //Vector V(r*(s*f.v)); //relative vel in global coords
    Matrix R = rot.getMatrix();
    Vector P(R*f.pos); //relative offset in global coords
    Vector V(R*f.vel); //relative vel in global coords
    Vector W(R*f.angvel); //relative ang vel in global coords
    pos += P;
    vel += angvel^P;
    vel += V;
    //a += b^P;
    //a += w^((w^P) + 2.*V);
    //a += r*(s*f.a);
    //b += w^W;
    //b += r*f.b;
    angvel += W;
    rot = rot*f.rot;
    //s*=f.s;
    zeroVels = false;
  }
}

/// inverse transform (new = f^{-1} * old) or (old = f * new)
void DynamicTransformation::appendInvTransformation(const DynamicTransformation& f) {
  if(zeroVels && f.zeroVels) {
    rot = rot/f.rot;
    pos -= rot*f.pos;
  } else {
    rot=rot/f.rot;
    Matrix R = rot.getMatrix();
    Vector P(R*f.pos);
    angvel -= R*f.angvel;
    vel -= R*f.vel;
    vel -= angvel^P;
    pos -= P;
    zeroVels = false;
  }
}

/// this = f^{-1}
void DynamicTransformation::setInverse(const DynamicTransformation& f) {
  if(f.zeroVels) {
    rot = -f.rot;
    pos = - (rot * f.pos);
    vel.setZero();
    angvel.setZero();
    zeroVels = true;
  } else {
    rot = -f.rot;
    Matrix R = rot.getMatrix();
    pos = - (R * f.pos);
    vel = R * ((f.angvel^f.pos) - f.vel);
    angvel = - (R * f.angvel);
    zeroVels = false;
  }
}

/// set double[4*4] to Transformation. Matrix needs to be orthogonal
void DynamicTransformation::setAffineMatrix(const double *m) {
  double M[9];
  uint i, j;
  for(i=0; i<3; ++i)
    for(j=0; j<3; ++j)
      M[i*3+j] = m[i*4+j];
  rot.setMatrix(M);                 // set 3x3 submatrix as rotation
  pos.x=m[3];  // set last column as translation
  pos.y=m[7];  // set last column as translation
  pos.z=m[11];  // set last column as translation
  zeroVels=true;
}

///  to = new * from
void DynamicTransformation::setDifference(const DynamicTransformation& from, const DynamicTransformation& to) {
  if(from.zeroVels && to.zeroVels) {
    rot = Quaternion_Id / from.rot * to.rot;
    pos = from.rot/(to.pos-from.pos);
    zeroVels = true;
  } else {
    rot = Quaternion_Id / from.rot * to.rot;
    angvel = from.rot/(to.angvel-from.angvel);
    vel = from.rot/(to.vel-from.vel);
    vel-= from.rot/(from.angvel^(to.pos-from.pos));
    pos = from.rot/(to.pos-from.pos);
    zeroVels = false;
  }
}

bool DynamicTransformation::isZero() const {
  return pos.isZero && rot.isZero && vel.isZero && angvel.isZero;
}

/// L1-norm to zero
double DynamicTransformation::diffZero() const {
  return pos.diffZero() + rot.diffZero() + vel.diffZero() + angvel.diffZero();
}

/// operator<<
void DynamicTransformation::write(std::ostream& os) const {
  os <<pos.x <<' ' <<pos.y <<' ' <<pos.z <<' '
     <<rot.w <<' ' <<rot.x <<' ' <<rot.y <<' ' <<rot.z;
  if(!zeroVels) {
    os <<" v" <<vel <<" w" <<angvel;
  }
}

/// operator>>
void DynamicTransformation::read(std::istream& is) {
  setZero();
  char c;
  double x[4];
  mlr::skip(is, " \n\r\t<|");
  for(;;) {
    is >>c;
    if(is.fail()) return;  //EOF I guess
    //if(c==';') break;
    //if(c==',') is >>c;
    if((c>='0' && c<='9') || c=='.' || c=='-') {  //read a 7-vector (pos+quat) for the transformation
      is.putback(c);
      is>>x[0]>>x[1]>>x[2];       addRelativeTranslation(x[0], x[1], x[2]);
      is>>x[0]>>x[1]>>x[2]>>x[3]; addRelativeRotationQuat(x[0], x[1], x[2], x[3]);
    } else switch(c) {
          //case '<': break; //do nothing -- assume this is an opening tag
        case 't': is>>PARSE("(")>>x[0]>>x[1]>>x[2]>>PARSE(")");       addRelativeTranslation(x[0], x[1], x[2]); break;
        case 'q': is>>PARSE("(")>>x[0]>>x[1]>>x[2]>>x[3]>>PARSE(")"); addRelativeRotationQuat(x[0], x[1], x[2], x[3]); break;
        case 'r': is>>PARSE("(")>>x[0]>>x[1]>>x[2]>>x[3]>>PARSE(")"); addRelativeRotationRad(x[0], x[1], x[2], x[3]); break;
        case 'd': is>>PARSE("(")>>x[0]>>x[1]>>x[2]>>x[3]>>PARSE(")"); addRelativeRotationDeg(x[0], x[1], x[2], x[3]); break;
        case 'E': is>>PARSE("(")>>x[0]>>x[1]>>x[2]>>PARSE(")"); addRelativeRotation(Quaternion().setRpy(x[0], x[1], x[2])); break;
        case 'v': is>>PARSE("(")>>x[0]>>x[1]>>x[2]>>PARSE(")");       addRelativeVelocity(x[0], x[1], x[2]); break;
        case 'w': is>>PARSE("(")>>x[0]>>x[1]>>x[2]>>PARSE(")");       addRelativeAngVelocityRad(x[0], x[1], x[2]); break;
          //case 's': is>>PARSE("(")>>x[0]>>PARSE(")");                   scale(x[0]); break;
        case '|':
        case '>': is.putback(c); return; //those symbols finish the reading without error
        default: MLR_MSG("unknown DynamicTransformation read tag: " <<c <<"abort reading this frame"); is.putback(c); return;
      }
    if(is.fail()) HALT("error reading '" <<c <<"' parameters in frame");
  }
  if(is.fail()) HALT("could not read DynamicTransformation struct");
  zeroVels = vel.isZero && angvel.isZero;
}

//===========================================================================
//
// camera class
//

/** @brief constructor; specify a frame if the camera is to be attached
   to an existing frame. Otherwise the camera creates its own
      frame */
Camera::Camera() {
  setZero();

  setPosition(0., 0., 10.);
  focus(0, 0, 0);
  setZRange(.1, 1000.);
  setHeightAngle(12.);
}

void Camera::setZero() {
  X.setZero();
  foc.setZero();
  heightAngle=90.;
  heightAbs=10.;
  focalLength=1.;
  whRatio=1.;
  zNear=.1;
  zFar=1000.;
}

/// the height angle (in degrees) of the camera perspective; set it 0 for orthogonal projection
void Camera::setHeightAngle(float a) { heightAngle=a; heightAbs=0.;}
/// the absolute height of the camera perspective (automatically also sets heightAngle=0)
void Camera::setHeightAbs(float h) { heightAngle=0.; heightAbs=h; }
/// the z-range (depth range) visible for the camera
void Camera::setZRange(float znear, float zfar) { zNear=znear; zFar=zfar; }
/// set the width/height ratio of your viewport to see a non-distorted picture
void Camera::setWHRatio(float ratio) { whRatio=ratio; }
/// the frame's position
void Camera::setPosition(float x, float y, float z) { X.pos.set(x, y, z); }
/// rotate the frame to focus the absolute coordinate origin (0, 0, 0)
void Camera::focusOrigin() { foc.setZero(); focus(); }
/// rotate the frame to focus the point (x, y, z)
void Camera::focus(float x, float y, float z) { foc.set(x, y, z); focus(); }
/// rotate the frame to focus the point given by the vector
void Camera::focus(const Vector& v) { foc=v; focus(); }
/// rotate the frame to focus (again) the previously given focus
void Camera::focus() { watchDirection(foc-X.pos); } //X.Z=X.pos; X.Z-=foc; X.Z.normalize(); upright(); }
/// rotate the frame to watch in the direction vector D
void Camera::watchDirection(const Vector& d) {
  if(d.x==0. && d.y==0.) {
    X.rot.setZero();
    if(d.z>0) X.rot.setDeg(180, 1, 0, 0);
    return;
  }
  Quaternion r;
  r.setDiff(-X.rot.getZ(), d);
  X.rot=r*X.rot;
}
/// rotate the frame to set it upright (i.e. camera's y aligned with world's z)
void Camera::upright(const Vector& up) {
#if 1
  //construct desired X:
  Vector fwd(0, 0, -1), x(1, 0, 0), xDesired;
  x=X.rot*x; //true X
  fwd=X.rot*fwd;
//  if(fabs(fwd.z)<1.) up.set(0, 0, 1); else up.set(0, 1, 0);
  xDesired=up^fwd; //desired X
  if(xDesired*x<=0) xDesired=-xDesired;
  Quaternion r;
  r.setDiff(x, xDesired);
  X.rot=r*X.rot;
#else
  if(X.Z[2]<1.) X.Y.set(0, 0, 1); else X.Y.set(0, 1, 0);
  X.X=X.Y^X.Z; X.X.normalize();
  X.Y=X.Z^X.X; X.Y.normalize();
#endif
}

//}

void Camera::setCameraProjectionMatrix(const arr& P) {
  //P is in standard convention -> computes fixedProjectionMatrix in OpenGL convention from this
  cout <<"desired P=" <<P <<endl;
  arr Kview=ARR(200., 0., 200., 0., 200., 200., 0., 0., 1.); //OpenGL's calibration matrix
  Kview.reshape(3, 3);
  //arr glP=inverse(Kview)*P;
  arr glP=P;
  //glP[2]()*=-1.;
  glP.append(glP[2]);
  glP[2]()*=.99; glP(2, 2)*=1.02; //some hack to invent a culling coordinate (usually determined via near and far...)
  glP = ~glP;
  glP *= 1./glP(3, 3);
  cout <<"glP=" <<glP <<endl;
  //glLoadMatrixd(glP.p);
  //fixedProjectionMatrix = glP;
}

/** sets OpenGL's GL_PROJECTION matrix accordingly -- should be
    called in an opengl draw routine */
void Camera::glSetProjectionMatrix() {
//  if(fixedProjectionMatrix.N) {
//    glLoadMatrixd(fixedProjectionMatrix.p);
//  } else {
  if(heightAngle==0) {
    if(heightAbs==0) {
      arr P(4,4);
      P.setZero();
      P(0,0) = 2.*focalLength/whRatio;
      P(1,1) = 2.*focalLength;
      P(2,2) = (zFar + zNear)/(zNear-zFar);
      P(2,3) = -1.;
      P(3,2) = 2. * zFar * zNear / (zNear-zFar);
      glLoadMatrixd(P.p);
    }else{
      glOrtho(-whRatio*heightAbs/2, whRatio*heightAbs/2,
              -heightAbs/2, heightAbs/2, zNear, zFar);
    }
  } else
    gluPerspective(heightAngle, whRatio, zNear, zFar);
  double m[16];
  glMultMatrixd(X.getInverseAffineMatrixGL(m));
}

/// convert from gluPerspective's non-linear [0, 1] depth to the true [zNear, zFar] depth
double Camera::glConvertToTrueDepth(double d) {
  return zNear + (zFar-zNear)*d/(zFar/zNear*(1.-d)+1.);
}

/// convert from gluPerspective's non-linear [0, 1] depth to the linear [0, 1] depth
double Camera::glConvertToLinearDepth(double d) {
  return d/(zFar/zNear*(1.-d)+1.);
}

void Camera::setKinect(){
  setZero();
  setPosition(0., 0., 0.);
  focus(0., 0., 5.);
  setZRange(.1, 50.);
#if 1
  heightAbs=heightAngle = 0;
  focalLength = 580./480.;
#else
  heightAngle=45;
#endif
  whRatio = 640./480.;
}

void Camera::setDefault(){
  setHeightAngle(12.);
  setPosition(10., -15., 8.);
  focus(0, 0, 1.);
  upright();
}

//==============================================================================

/// use as similarity measure (distance = 1 - |scalarprod|)
double scalarProduct(const Quaternion& a, const Quaternion& b) {
  return a.w*b.w+a.x*b.x+a.y*b.y+a.z*b.z;
}

std::istream& operator>>(std::istream& is, Vector& x)    { x.read(is); return is; }
std::istream& operator>>(std::istream& is, Matrix& x)    { x.read(is); return is; }
std::istream& operator>>(std::istream& is, Quaternion& x) { x.read(is); return is; }
std::istream& operator>>(std::istream& is, Transformation& x)     { x.read(is); return is; }
std::ostream& operator<<(std::ostream& os, const Vector& x)    { x.write(os); return os; }
std::ostream& operator<<(std::ostream& os, const Matrix& x)    { x.write(os); return os; }
std::ostream& operator<<(std::ostream& os, const Quaternion& x) { x.write(os); return os; }
std::ostream& operator<<(std::ostream& os, const Transformation& x)     { x.write(os); return os; }

} //namespace mlr


//===========================================================================
//
// low level drivers
//

/** distance to surface, distance gradient, and hessian for this shape
 *
 * Details in inf cylinder section of
 * mlr/stanio/concepts/note-analytic-impl-shapes-hessian
 */



//===========================================================================
//
// explicit instantiations
//

template mlr::Array<mlr::Vector>::Array();
template mlr::Array<mlr::Vector>::~Array();

template mlr::Array<mlr::Transformation*>::Array();
template mlr::Array<mlr::Transformation*>::~Array();
#include <Optim/lagrangian.h>

#include "geoOptim.h"

void fitSSBox(arr& x, double& f, double& g, const arr& X, int verbose){
  struct fitSSBoxProblem : ConstrainedProblem{
    const arr& X;
    fitSSBoxProblem(const arr& X):X(X){}
    void phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& tt, const arr& x){
      phi.resize(5+X.d0);
      if(&tt){ tt.resize(5+X.d0); tt=OT_ineq; }
      if(&J) {  J.resize(5+X.d0,11); J.setZero(); }
      if(&H) {  H.resize(11,11); H.setZero(); }

      //-- the scalar objective
      double a=x(0), b=x(1), c=x(2), r=x(3); //these are box-wall-coordinates --- not WIDTH!
      phi(0) = a*b*c + 2.*r*(a*b + a*c +b*c) + 4./3.*r*r*r;
      if(&tt) tt(0) = OT_f;
      if(&J){
        J(0,0) = b*c + 2.*r*(b+c);
        J(0,1) = a*c + 2.*r*(a+c);
        J(0,2) = a*b + 2.*r*(a+b);
        J(0,3) = 2.*(a*b + a*c +b*c) + 4.*r*r;
      }
      if(&H){
        H(0,1) = H(1,0) = c + 2.*r;
        H(0,2) = H(2,0) = b + 2.*r;
        H(0,3) = H(3,0) = 2.*(b+c);

        H(1,2) = H(2,1) = a + 2.*r;
        H(1,3) = H(3,1) = 2.*(a+c);

        H(2,3) = H(3,2) = 2.*(a+b);

        H(3,3) = 8.*r;
      }

      //-- positive
      double w=100.;
      phi(1) = -w*(a-.001);
      phi(2) = -w*(b-.001);
      phi(3) = -w*(c-.001);
      phi(4) = -w*(r-.001);
      if(&J){
        J(1,0) = -w;
        J(2,1) = -w;
        J(3,2) = -w;
        J(4,3) = -w;
      }

      //-- all constraints
      for(uint i=0;i<X.d0;i++){
        arr y, Jy;
        y = X[i];
        y.append(x);
        phi(i+5) = DistanceFunction_SSBox(Jy, NoArr, y);
        //      Jy({3,5})() *= -1.;
        if(&J) J[i+5] = Jy({3,-1});
      }
    }
  } F(X);

  //initialization
  x.resize(11);
  mlr::Quaternion rot;
  rot.setRandom();
  arr tX = X * rot.getArr(); //rotate points (with rot^{-1})
  arr ma = max(tX,0), mi = min(tX,0);  //get coordinate-wise min and max
  x({0,2})() = (ma-mi)/2.;   //sizes
  x(3) = 1.; //sum(ma-mi)/6.;  //radius
  x({4,6})() = rot.getArr() * (mi+.5*(ma-mi)); //center (rotated back)
  x({7,10})() = conv_quat2arr(rot);
  rndGauss(x({7,10})(), .1, true);
  x({7,10})() /= length(x({7,10})());

  if(verbose>1){
    checkJacobianCP(F, x, 1e-4);
    checkHessianCP(F, x, 1e-4);
  }

  OptConstrained opt(x, NoArr, F, OPT(
                   stopTolerance = 1e-4,
                   stopFTolerance = 1e-3,
                   damping=1,
                   maxStep=-1,
                   constrainedMethod = augmentedLag,
                   aulaMuInc = 1.1
                   ));
  opt.run();

  if(verbose>1){
    checkJacobianCP(F, x, 1e-4);
    checkHessianCP(F, x, 1e-4);
  }

  f = opt.UCP.get_costs();
  g = opt.UCP.get_sumOfGviolations();
}

void computeOptimalSSBox(mlr::Mesh& mesh, arr& x_ret, mlr::Transformation& t_ret, const arr& X, uint trials, int verbose){
  if(!X.N){ mesh.clear(); return; }

  arr x,x_best;
  double f,g, f_best, g_best;
  fitSSBox(x_best, f_best, g_best, X, verbose);
  for(uint k=1;k<trials;k++){
    fitSSBox(x, f, g, X, verbose);
    if(g<g_best-1e-4 ||
       (g<1e-4 && f<f_best)){ x_best=x; f_best=f; g_best=g; }
  }

  x = x_best;

  //convert box wall coordinates to box width (incl radius)
  x(0) = 2.*(x(0)+x(3));
  x(1) = 2.*(x(1)+x(3));
  x(2) = 2.*(x(2)+x(3));

  if(x_ret!=NoArr)
    x_ret=x;

  if(verbose>2){
    cout <<"x=" <<x;
    cout <<"\nf = " <<f_best <<"\ng-violations = " <<g_best <<endl;
  }

  mlr::Transformation t;
  t.setZero();
  t.pos.set( x({4,6}) );
  t.rot.set( x({7,-1}) );
  t.rot.normalize();
  mesh.setSSBox(x(0), x(1), x(2), x(3));
  t.applyOnPointArray(mesh.V);

  if(t_ret!=NoTransformation)
    t_ret = t;
}
#include "viewer.h"
#include <Gui/opengl.h>
#include <Geo/mesh.h>

//===========================================================================
//
// ImageViewer
//

struct sImageViewer{
  OpenGL gl;
  sImageViewer(const char* tit) : gl(tit) {}
};

ImageViewer::ImageViewer(const char* img_name)
  : Thread(STRING("ImageViewer_"<<img_name), -1),
    img(this, img_name, true){
  threadOpen();
}

ImageViewer::~ImageViewer(){
  threadClose();
}

void ImageViewer::open(){
  s = new sImageViewer(STRING("ImageViewer: "<<img.data->name));
  s->gl.openWindow();
  s->gl.update();
}

void ImageViewer::close(){ delete s; }

void ImageViewer::step(){
  s->gl.dataLock.writeLock();
  s->gl.background = img.get();
  if(flipImage) flip_image(s->gl.background);
#if 1 //draw a center
  uint ci = s->gl.background.d0/2;
  uint cj = s->gl.background.d1/2;
  uint skip = s->gl.background.d1*s->gl.background.d2;
  byte *p, *pstop;
  p=&s->gl.background(ci-5, cj-5, 0);
  pstop=&s->gl.background(ci-5, cj+5, 0);
  for(;p<=pstop;p++) *p = 0;
  p=&s->gl.background(ci+5, cj-5, 0);
  pstop=&s->gl.background(ci+5, cj+5, 0);
  for(;p<=pstop;p++) *p = 0;
  p=&s->gl.background(ci-5, cj-5, 0);
  pstop=&s->gl.background(ci+5, cj-5, 0);
  for(;p<=pstop;p+=skip) p[0]=p[1]=p[2]=0;
  p=&s->gl.background(ci-5, cj+5, 0);
  pstop=&s->gl.background(ci+5, cj+5, 0);
  for(;p<=pstop;p+=skip) p[0]=p[1]=p[2]=0;
#endif
  s->gl.dataLock.unlock();
  if(!s->gl.background.N) return;
  if(s->gl.height!= s->gl.background.d0 || s->gl.width!= s->gl.background.d1)
    s->gl.resize(s->gl.background.d1, s->gl.background.d0);

  s->gl.update(name, false, false, true);
}


//===========================================================================
//
// PointCloudViewer
//

struct sPointCloudViewer{
  OpenGL gl;
  sPointCloudViewer(const char* tit) : gl(tit,640,480){}
  mlr::Mesh pc;
};

void glDrawAxes(void*){
  glDrawAxes(1.);
}

PointCloudViewer::PointCloudViewer(const char* pts_name, const char* rgb_name)
  : Thread(STRING("PointCloudViewer_"<<pts_name <<'_' <<rgb_name), .1),
    pts(this, pts_name),
    rgb(this, rgb_name){
  threadLoop();
}

PointCloudViewer::~PointCloudViewer(){
  threadClose();
}

void PointCloudViewer::open(){
  s = new sPointCloudViewer(STRING("PointCloudViewer: "<<pts.name <<' ' <<rgb.name));
#if 0
  s->gl.add(glDrawAxes);
  s->gl.add(s->pc);
  s->gl.camera.setKinect();
  //  s->gl.reportSelects = true;
#else
  s->gl.add(glStandardScene);
  s->gl.add(s->pc);
  s->gl.camera.setDefault();
#endif
}

void PointCloudViewer::close(){
  delete s;
}

void PointCloudViewer::step(){
  s->gl.dataLock.writeLock();
  s->pc.V=pts.get();
  copy(s->pc.C, rgb.get()());
  uint n=s->pc.V.N/3;
  if(n!=s->pc.C.N/3){
    s->gl.dataLock.unlock();
    return;
  }
  s->pc.C /= 255.;
  s->pc.V.reshape(n,3);
  s->pc.C.reshape(n,3);
  s->gl.dataLock.unlock();

  s->gl.update(NULL, false, false, true);
}

//===========================================================================
//
// MeshAViewer
//

MeshAViewer::MeshAViewer(const char* meshes_name)
  : Thread(STRING("MeshAViewer_"<<meshes_name), .1),
    meshes(this, meshes_name){
  threadLoop();
}

MeshAViewer::~MeshAViewer(){
  threadClose();
}

void MeshAViewer::open(){
  gl = new OpenGL(STRING("MeshAViewer: "<<meshes.name));
  gl->add(glStandardScene);
  gl->add(glDrawMeshes, &copy);
  gl->camera.setDefault();
}

void MeshAViewer::close(){
  delete gl;
}

void MeshAViewer::step(){
  gl->dataLock.writeLock();
  copy = meshes.get();
  gl->dataLock.unlock();

  gl->update(NULL, false, false, true);
}

/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#include "graphview.h"
#include <Core/util.tpp>
#include <Core/array.tpp>
#include <Gui/gtk.h>

#if defined MLR_GTK and defined MLR_GRAPHVIZ

#include <graphviz/graphviz_version.h>
#if defined PACKAGE_URL //check the graphviz version (awkward...)

#include <gtk/gtk.h>
#include <graphviz/gvc.h>
#include <graphviz/gvplugin_device.h>
#undef MIN
#undef MAX


#define INFO(x) if(gv->p->verbose) printf("CALLBACK: %s\n",#x);

extern "C" {
  GVJ_t *gvjobs_first(GVC_t * gvc);
}

struct sGraphView {
  Graph *G;
  GraphView *p;
  mlr::String title;
  
  // on gtk side
  GtkWidget *drawingarea,*container;
  
  // on graphviz side
  graph_t *gvGraph;
  mlr::Array<Agnode_t *> gvNodes;
  GVC_t *gv_context;
  GVJ_t *gvJob() { return gvjobs_first(gv_context); }

  sGraphView():gvGraph(NULL){}
  void init();
  void updateGraphvizGraph(bool isSubGraph=false);
  void writeFile(const char* filename);
  
  static bool on_drawingarea_expose_event(GtkWidget *widget,    GdkEventExpose  *event,    gpointer     user_data);
  static bool on_drawingarea_motion_notify_event(GtkWidget *widget,       GdkEventMotion  *event,       gpointer     user_data);
  static bool on_container_delete_event(GtkWidget *widget,   GdkEvent    *event,   gpointer     user_data);
  static bool on_drawingarea_configure_event(GtkWidget *widget,   GdkEventConfigure *event,   gpointer     user_data);
  static bool on_drawingarea_button_press_event(GtkWidget *widget,      GdkEventButton  *event,      gpointer     user_data);
  static bool on_drawingarea_button_release_event(GtkWidget *widget,    GdkEventButton  *event,    gpointer     user_data);
  static bool on_drawingarea_scroll_event(GtkWidget   *widget, GdkEventScroll    *event,    gpointer     user_data);
};

GraphView::GraphView(Graph& G, const char* title, void *container)
  : verbose(false) {
  gtkCheckInitialized();
  
  s = new sGraphView;
  s->p=this;
  s->title=title;
  s->container=GTK_WIDGET(container);
  s->G = &G;
  s->init();
}

GraphView::~GraphView() {
  delete s;
}

void GraphView::update() {
  gtkLock();
  s->updateGraphvizGraph();
  gvLayoutJobs(s->gv_context, s->gvGraph);
  gvRenderJobs(s->gv_context, s->gvGraph);
  gtkUnlock();
  gtkProcessEvents();
}

void GraphView::watch() {
  update();
  if(mlr::getInteractivity()){
    gtk_main();
  }
}

void GraphView::writeFile(const char* filename){
  s->writeFile(filename);
}

#define STR(s) (char*)s

mlr::String label(Node *it){
  mlr::String label;
#if 1
  if(it->keys.N) {
    label <<it->keys(0);
    for(uint j=1; j<it->keys.N; j++) label <<'\n' <<it->keys(j);
  }
  if(it->keys.N) label <<'\n';
  label <<'=';
  it->writeValue(label);
#else
  label <<it->index;
#endif
  return label;
}

void sGraphView::updateGraphvizGraph(bool isSubGraph) {
  if(gvGraph) agclose(gvGraph);

  if(!isSubGraph){
    gvGraph = agopen(STR("new_graph"), Agdirected, NULL);
    agattr(gvGraph, AGRAPH,STR("rankdir"), STR("LR"));
    agattr(gvGraph, AGRAPH,STR("ranksep"), STR("0.05"));

    agattr(gvGraph, AGNODE, STR("label"), STR(""));
    agattr(gvGraph, AGNODE, STR("shape"), STR(""));
    agattr(gvGraph, AGNODE, STR("fontsize"), STR("11"));
    agattr(gvGraph, AGNODE, STR("width"), STR(".3"));
    agattr(gvGraph, AGNODE, STR("height"), STR(".3"));

    agattr(gvGraph, AGEDGE, STR("label"), STR(""));
//    agattr(gvGraph, AGEDGE, STR("arrowhead"), STR("none"));
    agattr(gvGraph, AGEDGE, STR("arrowsize"), STR(".5"));
    agattr(gvGraph, AGEDGE, STR("fontsize"), STR("6"));

    uint nNodes = G->index(true);
    gvNodes.resize(nNodes);
  }

  //first add `nodes' for all items
  for(Node *e: *G) {
    gvNodes(e->index) = agnode(gvGraph, STRING(e->index), true);
    agset(gvNodes(e->index), STR("label"), label(e));
    if(e->parents.N) {
      agset(gvNodes(e->index), STR("shape"), STR("box"));
      agset(gvNodes(e->index), STR("fontsize"), STR("6"));
      agset(gvNodes(e->index), STR("width"), STR(".1"));
      agset(gvNodes(e->index), STR("height"), STR(".1"));
    } else {
      agset(gvNodes(e->index), STR("shape"), STR("ellipse"));
    }
  }
  
  //now add 'edges' for all relations
  for(Node *e: (*G)) {
    /*if(e->parents.N==2){ //is an edge
      gvNodes(i) = (Agnode_t*)agedge(gvGraph, gvNodes(e->parents(0)->id), gvNodes(e->parents(1)->id)); //, STRING(i <<"_" <<e->name), true);
    }else*/
    if(e->parents.N) {
      uint linkId=0;
      for(Node *n: e->parents) {
        if(n->index<e->index){
//          ge=agedge(gvGraph, gvNodes(n->index), gvNodes(e->index), STRING(label(n) <<"--" <<label(e)), true);
          agedge(gvGraph, gvNodes(n->index), gvNodes(e->index), NULL, true);
        }else{
//          ge=agedge(gvGraph, gvNodes(n->index), gvNodes(e->index), STRING(label(n) <<"--" <<label(e)), true);
          agedge(gvGraph, gvNodes(n->index), gvNodes(e->index), NULL, true);
        }
//        agset(ge, STR("label"), STRING(linkId));

        linkId++;
      }
    }
  }

  if(!isSubGraph){
    G->index(false);
  }
}

void sGraphView::writeFile(const char* filename){
  gtkLock();
  GVC_t *gvc = gvContext();
  updateGraphvizGraph();
  gvLayout(gvc, gvGraph, "dot");
  gvRenderFilename(gvc, gvGraph, "canon", filename);
  gvFreeLayout(gvc, gvGraph);
  gvFreeContext(gvc);
  gtkUnlock();
}

void sGraphView::init() {
  gtkLock();
  gv_context = ::gvContext();
  char *bla[] = {STR("dot"), STR("-Tx11"), NULL};
  gvParseArgs(gv_context, 2, bla);
  
  if(!container) {
    container = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    g_object_set_data(G_OBJECT(container), "GraphvizGtk", (gpointer) this);
    gtk_window_set_title(GTK_WINDOW(container), title);
  }
  
  drawingarea = gtk_drawing_area_new();
  g_object_set_data(G_OBJECT(drawingarea), "GraphvizGtk", (gpointer) this);
  gtk_widget_show(drawingarea);
  gtk_container_add(GTK_CONTAINER(container), drawingarea);
  gtk_widget_set_size_request(drawingarea, 300, 300);
  gtk_widget_set_events(drawingarea, GDK_EXPOSURE_MASK | GDK_POINTER_MOTION_MASK | GDK_BUTTON_MOTION_MASK | GDK_BUTTON_PRESS_MASK | GDK_BUTTON_RELEASE_MASK | GDK_ENTER_NOTIFY_MASK | GDK_LEAVE_NOTIFY_MASK);
  
  g_signal_connect((gpointer) container, "delete_event",  G_CALLBACK(on_container_delete_event),  NULL);
  g_signal_connect((gpointer) drawingarea, "expose_event",  G_CALLBACK(on_drawingarea_expose_event),  NULL);
  g_signal_connect((gpointer) drawingarea, "motion_notify_event",  G_CALLBACK(on_drawingarea_motion_notify_event),  NULL);
  g_signal_connect((gpointer) drawingarea, "configure_event",  G_CALLBACK(on_drawingarea_configure_event),  NULL);
  g_signal_connect((gpointer) drawingarea, "button_press_event",  G_CALLBACK(on_drawingarea_button_press_event),  NULL);
  g_signal_connect((gpointer) drawingarea, "button_release_event",  G_CALLBACK(on_drawingarea_button_release_event),  NULL);
  g_signal_connect((gpointer) drawingarea, "scroll_event",  G_CALLBACK(on_drawingarea_scroll_event),  NULL);
  
  gtk_widget_show(container);
  gtkUnlock();
}

bool sGraphView::on_drawingarea_expose_event(GtkWidget *widget, GdkEventExpose  *event, gpointer user_data) {
  sGraphView *gv = (sGraphView*)g_object_get_data(G_OBJECT(widget),"GraphvizGtk");
  INFO(on_drawingarea_expose_event);

  GVJ_t *job = gv->gvJob();
  cairo_t *cr = gdk_cairo_create(gtk_widget_get_window(widget));
  
  job->context = (void*)cr;
  job->external_context = TRUE;
  job->width = widget->allocation.width; //gtk_widget_get_allocated_width(widget);
  job->height = widget->allocation.height; //gtk_widget_get_allocated_height(widget);
  if(job->has_been_rendered){
    if(job->callbacks) (job->callbacks->refresh)(job);
  }else{
    if(job->callbacks) (job->callbacks->refresh)(job);
  }
    
  cairo_destroy(cr);
  
  if(job->current_obj) {
    if(agobjkind(job->current_obj)==AGNODE || agobjkind(job->current_obj)==AGEDGE) {
      int i=gv->gvNodes.findValue((Agnode_t*)job->current_obj);
      if(i<0 || i>=(int)gv->G->N) {
        MLR_MSG("This is no object id:" <<i);
      } else {
        cout <<"current object:" <<i <<' ' <<*(*gv->G)(i) <<endl;
      }
    }
  }
  if(job->selected_obj) {
    if(agobjkind(job->selected_obj)==AGNODE) {
      int i=gv->gvNodes.findValue((Agnode_t*)job->selected_obj);
      if(i<0) {
        MLR_MSG("???");
      } else {
      
        cout <<"selected object:" <<i <<' ' <<*(*gv->G)(i) <<endl;
      }
    }
  }
  
  return FALSE;
}


bool sGraphView::on_drawingarea_motion_notify_event(GtkWidget       *widget,                   GdkEventMotion  *event,                   gpointer         user_data) {
  sGraphView *gv = (sGraphView*)g_object_get_data(G_OBJECT(widget),"GraphvizGtk");
  INFO(on_drawingarea_motion_notify_event);
  
  GVJ_t *job = gv->gvJob();
  if(!job) return false;
  job->pointer.x = event->x;
  job->pointer.y = event->y;
  if(job->callbacks) (job->callbacks->motion)(job, job->pointer);
  
  gtk_widget_queue_draw(widget);
  
  return FALSE;
}

bool sGraphView::on_container_delete_event(GtkWidget       *widget,       GdkEvent        *event,       gpointer         user_data) {
  sGraphView *gv = (sGraphView*)g_object_get_data(G_OBJECT(widget),"GraphvizGtk");
  INFO(on_container_delete_event);
  
  gtk_main_quit();
  return FALSE;
}


bool sGraphView::on_drawingarea_configure_event(GtkWidget       *widget,               GdkEventConfigure *event,               gpointer         user_data) {
  sGraphView *gv = (sGraphView*)g_object_get_data(G_OBJECT(widget),"GraphvizGtk");
  INFO(on_drawingarea_configure_event);
  
  /*FIXME - should allow for margins */
  /*      - similar zoom_to_fit code exists in: */
  /*      plugin/gtk/callbacks.c */
  /*      plugin/xlib/gvdevice_xlib.c */
  /*      lib/gvc/gvevent.c */
  

  GVJ_t *job = gv->gvJob();
  if(!job) return false;

  double zoom_to_fit;
//  if(!job->has_been_rendered) {
//    zoom_to_fit = 1.0;
//    mlr::MIN((double) event->width / (double) job->width, (double) event->height / (double) job->height);
//    if(zoom_to_fit < 1.0)  /* don't make bigger */
//      job->zoom *= zoom_to_fit;
//  } else if(job->fit_mode) {
    zoom_to_fit = mlr::MIN((double) event->width / (double) job->width, (double) event->height / (double) job->height);
    job->zoom *= zoom_to_fit;
//  }
  if(event->width > (int)job->width || event->height > (int)job->height)
    job->has_grown = TRUE;
  job->width = event->width;
  job->height = event->height;
  job->needs_refresh = TRUE;
  
  return FALSE;
}


bool sGraphView::on_drawingarea_button_press_event(GtkWidget       *widget,                  GdkEventButton  *event,                  gpointer         user_data) {
  sGraphView *gv = (sGraphView*)g_object_get_data(G_OBJECT(widget),"GraphvizGtk");
  INFO(on_drawingarea_button_press_event);
  
  GVJ_t *job = gv->gvJob();
  if(!job) return false;

  pointf pointer;
  pointer.x = event->x;
  pointer.y = event->y;
  (job->callbacks->button_press)(job, event->button, pointer);
  
  gtk_widget_queue_draw(widget);
  
  return FALSE;
}

bool sGraphView::on_drawingarea_button_release_event(GtkWidget *widget, GdkEventButton  *event, gpointer user_data) {
  sGraphView *gv = (sGraphView*)g_object_get_data(G_OBJECT(widget),"GraphvizGtk");
  INFO(on_drawingarea_button_release_event);
  
  GVJ_t *job = gv->gvJob();
  if(!job) return false;

  pointf pointer;
  pointer.x = event->x;
  pointer.y = event->y;
  (job->callbacks->button_release)(job, event->button, pointer);
  
  gtk_widget_queue_draw(widget);
  
  return FALSE;
}

bool sGraphView::on_drawingarea_scroll_event(GtkWidget       *widget,            GdkEventScroll        *event,            gpointer         user_data) {
  sGraphView *gv = (sGraphView*)g_object_get_data(G_OBJECT(widget),"GraphvizGtk");
  INFO(on_drawingarea_scroll_event);

  GVJ_t *job = gv->gvJob();
  if(!job) return false;

  pointf pointer;
  pointer.x = event->x;
  pointer.y = event->y;
  switch(((GdkEventScroll *)event)->direction) {
    case GDK_SCROLL_UP:
      (job->callbacks->button_press)(job, 4, pointer);
      break;
    case GDK_SCROLL_DOWN:
      (job->callbacks->button_press)(job, 5, pointer);
      break;
    case GDK_SCROLL_LEFT:
    case GDK_SCROLL_RIGHT:
      break;
  }
  gtk_widget_queue_draw(widget);
  
  return FALSE;
}

#undef STR

#else //for bad versions
GraphView::GraphView(Graph& G, const char* title, void *container) { NICO }
GraphView::~GraphView() { NICO }
void GraphView::watch() { NICO }
void GraphView::update() { NICO }
#endif

#else //defined MLR_GTK and defined MLR_GRAPHVIZ
#include "graphview.h"
GraphView::GraphView(Graph& G, const char* title, void *container) { NICO }
GraphView::~GraphView() { NICO }
void GraphView::watch() { NICO }
void GraphView::update() { NICO }
#endif


//===========================================================================
//
// explicit instantiations
//

#if defined MLR_GTK and defined MLR_GRAPHVIZ and defined PACKAGE_URL
template mlr::Array<Agnode_t*>::Array();
template mlr::Array<Agnode_t*>::~Array();
#endif
/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#include <Core/array.tpp>
#include <Geo/geo.h>
#include <GL/glew.h>
#include "opengl.h"


OpenGL& NoOpenGL = *((OpenGL*)(NULL));

//===========================================================================

Singleton<SingleGLAccess> singleGLAccess;

//===========================================================================

#ifdef MLR_FREEGLUT

#include <GL/freeglut.h>
#include <GL/glx.h>
//===========================================================================
//
// A singleton to ensure once initialization
//

Mutex& OpenGLMutex();
static uint GLProcessCount = 0;

struct GLThread : Thread {
  GLThread() : Thread("GLSpinner", .01) {}
  ~GLThread() { threadClose(); } //when this is destroyed in deinit, looping had to be stopped (otherwise pthread error)
  void open(){}
  void step(){
    OpenGLMutex().lock();
    glutMainLoopEvent();
    OpenGLMutex().unlock();
  }
  void close(){}
};

class OpenGLProcess {
private:
  uint numWins;
  mlr::Array<OpenGL*> glwins;
  GLThread th;

public:
  OpenGLProcess() : numWins(0) {
    CHECK(GLProcessCount==0,"");
    GLProcessCount++;
    int argc=1;
    char *argv[1]={(char*)"x"};
    glutInit(&argc, argv);
  }
  ~OpenGLProcess(){
    CHECK(!numWins, "there are still OpenGL windows open");
    glutExit(); //also glut as already shut down during deinit
  }

  void addGL(uint i, OpenGL* gl){
    if(glwins.N<=i) glwins.resizeCopy(i+1);
    glwins(i) = gl;
    if(!numWins) th.threadLoop(); //start looping
    numWins++;
  }
  void delGL(uint i, OpenGL* gl){
    CHECK_EQ(glwins(i), gl, "");
    glwins(i)=NULL;
    numWins--;
    if(!numWins){
        OpenGLMutex().unlock();
        th.threadClose(); //Stop(true); //stop looping (maybe wait here; or even threadClose())
        OpenGLMutex().lock();
    }
  }
  OpenGL* getGL(uint i){
    return glwins(i);
  }
};

Singleton<OpenGLProcess> singleFreeglut;

Mutex& OpenGLMutex(){ return singleFreeglut.mutex; }



struct SFG_Display_dummy {
  _XDisplay *Display;
};

extern SFG_Display_dummy fgDisplay;


//===========================================================================
//
// OpenGL hidden self
//

struct sOpenGL {
  sOpenGL(OpenGL *gl): gl(gl), windowID(-1) {}
  sOpenGL(OpenGL *gl, void *container){ NIY }
  ~sOpenGL(){ gl->closeWindow(); }

  void beginGlContext(){}
  void endGlContext(){}

  //-- private OpenGL data
  OpenGL *gl;
  mlr::Vector downVec,downPos,downFoc;
  mlr::Quaternion downRot;

  //-- engine specific data
  int windowID;                        ///< id of this window in the global glwins list

  //-- callbacks
  static void _Void() { }
  static void _Draw() { auto fg=singleFreeglut();  OpenGL *gl=fg->getGL(glutGetWindow()); gl->Draw(gl->width,gl->height); glutSwapBuffers(); gl->isUpdating.setStatus(0); }
  static void _Key(unsigned char key, int x, int y) {        singleFreeglut()->getGL(glutGetWindow())->Key(key,x,y); }
  static void _Mouse(int button, int updown, int x, int y) { singleFreeglut()->getGL(glutGetWindow())->Mouse(button,updown,x,y); }
  static void _Motion(int x, int y) {                        singleFreeglut()->getGL(glutGetWindow())->Motion(x,y); }
  static void _PassiveMotion(int x, int y) {                 singleFreeglut()->getGL(glutGetWindow())->Motion(x,y); }
  static void _Reshape(int w,int h) {                        singleFreeglut()->getGL(glutGetWindow())->Reshape(w,h); }
  static void _MouseWheel(int wheel, int dir, int x, int y){ singleFreeglut()->getGL(glutGetWindow())->MouseWheel(wheel,dir,x,y); }

  void accessWindow() {  //same as above, but also sets gl cocntext (glXMakeCurrent)
    CHECK(windowID>=0,"window is not created");
    glutSetWindow(windowID);
  }
  void deaccessWindow() {
#ifndef MLR_MSVC
    glXMakeCurrent(fgDisplay.Display, None, NULL);
#endif
  }
};


//===========================================================================
//
// OpenGL implementations
//

void OpenGL::openWindow(){
  if(s->windowID==-1){
    {
      auto fg = singleFreeglut();
      glutInitWindowSize(width, height);
      //  glutInitWindowPosition(posx,posy);
      glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);

      s->windowID = glutCreateWindow(title);
      fg->addGL(s->windowID, this);

      glutDisplayFunc(s->_Draw);
      glutKeyboardFunc(s->_Key);
      glutMouseFunc(s->_Mouse) ;
      glutMotionFunc(s->_Motion) ;
      glutPassiveMotionFunc(s->_PassiveMotion) ;
      glutReshapeFunc(s->_Reshape);
      glutMouseWheelFunc(s->_MouseWheel) ;
    }
  }
}

void OpenGL::closeWindow(){
  if(s->windowID!=-1){
    auto fg=singleFreeglut();
    glutDestroyWindow(s->windowID);
    fg->delGL(s->windowID, this);
  }
}

void OpenGL::postRedrawEvent(bool fromWithinCallback) {
  auto fg=singleFreeglut();
  s->accessWindow();
  glutPostRedisplay();
  s->deaccessWindow();
}

void OpenGL::resize(int w,int h) {
  openWindow();
  {
    auto fg=singleFreeglut();
    s->accessWindow();
    glutReshapeWindow(w,h);
    s->deaccessWindow();
  }
}

#endif

//===========================================================================
//
// force instantiations
//

template mlr::Array<glUI::Button>::Array();
template mlr::Array<glUI::Button>::~Array();



//===========================================================================
//
// static objects
//

OpenGL *staticgl [10]; //ten pointers to be potentially used as display windows
uint OpenGL::selectionBuffer[1000];


//===========================================================================
//
// utility implementations
//

#ifdef MLR_GL
void glStandardLight(void*) {
  glEnable(GL_LIGHTING);
  static GLfloat ambient[]   = { .5, .5, .5, 1.0 };
  static GLfloat diffuse[]   = { .2, .2, .2, 1.0 };
  static GLfloat specular[]  = { .3, .3, .3, 1.0 };
  static GLfloat position[]  = { 100.0, -100.0, 100.0, 1.0 };
  static GLfloat direction[] = { -1.0, 1.0, -1.0 };
  glLightfv(GL_LIGHT0, GL_POSITION, position);
  glLightfv(GL_LIGHT0, GL_SPOT_DIRECTION, direction);
  glLighti(GL_LIGHT0, GL_SPOT_CUTOFF,   90);
  glLighti(GL_LIGHT0, GL_SPOT_EXPONENT, 10);
  glLightfv(GL_LIGHT0, GL_AMBIENT,  ambient);
  glLightfv(GL_LIGHT0, GL_DIFFUSE,  diffuse);
  glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
  glEnable(GL_LIGHT0);
}

void glStandardScene(void*) {
  glStandardLight(NULL);
  //  glDrawFloor(10, .8, .8, .8);
  //  glDrawFloor(10, 1.5, 0.83, .0);
  glDrawFloor(10., 108./255., 123./255., 139./255.);
  glDrawAxes(.1);
}

void glColor(int col) {
  static const GLfloat colorsTab[6][4] = {
    {0.2, 0.2, 1.0, 1.0}, // blue
    {1.0, 0.8, 0.0, 1.0}, // gold
    {1.0, 0.0, 0.0, 1.0}, // red
    {0.7, 0.7, 0.7, 1.0}, // gray
    {1.0, 1.0, 1.0, 1.0}, // white
    {0.2, 1.0, 0.2, 1.0}
  }; // green

  col = col%6; //if(col<0) col=0; if(col>5) col=5;
  glColor(colorsTab[col][0], colorsTab[col][1], colorsTab[col][2], colorsTab[col][3]);
}

void glColor(float r, float g, float b, float alpha) {
  float amb=1.f, diff=1.f, spec=.25f;
  GLfloat ambient[4]  = { r*amb , g*amb , b*amb , alpha };
  GLfloat diffuse[4]  = { r*diff, g*diff, b*diff, alpha };
  GLfloat specular[4] = { spec*(1.f+r), spec*(1.f+g), spec*(1.f+b), alpha };
#if 0
  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, diffuse);
#else
  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambient);
  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuse);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
  glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 1.0f);
#endif
  glColor4f(r, g, b, alpha);
}

void glColor(float *rgb) { glColor(rgb[0], rgb[1], rgb[2], 1.); }

void glColor(const arr& col){
  if(col.N==3) glColor(col.p[0], col.p[1], col.p[2], 1.);
  if(col.N==4) glColor(col.p[0], col.p[1], col.p[2], col.p[3]);
}

/* // shadows do not work with a light source;
   // thus, we need to leave this out. 4. Mar 06 (hh)
void glShadowTransform()
{
  GLfloat matrix[16];
  for(int i=0; i<16; i++) matrix[i] = 0;
  matrix[0]=1;
  matrix[5]=1;
  matrix[8]=-1;  //light_x
  matrix[9]=-1;  //light_y
  matrix[14]=.02; //ground offset
  matrix[15]=1;
  glPushMatrix();
  glMultMatrixf (matrix);
}
*/

void glTransform(const mlr::Transformation& t){
  double GLmatrix[16];
  t.getAffineMatrixGL(GLmatrix);
  glLoadMatrixd(GLmatrix);
}

void glRotate(const mlr::Quaternion& rot){
  double GLmatrix[16];
  rot.getMatrixGL(GLmatrix);
  glMultMatrixd(GLmatrix);
}


void glTransform(const double pos[3], const double R[12]) {
  GLfloat matrix[16];
  matrix[0]=R[0];
  matrix[1]=R[4];
  matrix[2]=R[8];
  matrix[3]=0;
  matrix[4]=R[1];
  matrix[5]=R[5];
  matrix[6]=R[9];
  matrix[7]=0;
  matrix[8]=R[2];
  matrix[9]=R[6];
  matrix[10]=R[10];
  matrix[11]=0;
  matrix[12]=pos[0];
  matrix[13]=pos[1];
  matrix[14]=pos[2];
  matrix[15]=1;
  glPushMatrix();
  glMultMatrixf(matrix);
}

static GLboolean glLightIsOn = false;
void glPushLightOff() { glGetBooleanv(GL_LIGHTING, &glLightIsOn); glDisable(GL_LIGHTING); }
void glPopLight() { if(glLightIsOn) glEnable(GL_LIGHTING); }

void glDrawText(const char* txt, float x, float y, float z, bool largeFont) {
  if(!txt) return;
  glDisable(GL_DEPTH_TEST);
  glPushLightOff();
  glRasterPos3f(x, y, z);
  void *font=GLUT_BITMAP_HELVETICA_12;
  if(largeFont) font = GLUT_BITMAP_HELVETICA_18;
  while(*txt) {
    switch(*txt) {
      case '\n':
        y+=15;
        glRasterPos3f(x, y, z);
        break;
      case '\b':
        if(font==GLUT_BITMAP_HELVETICA_12) font=GLUT_BITMAP_HELVETICA_18;
        else font=GLUT_BITMAP_HELVETICA_12;
        break;
      default:{
        glutBitmapCharacter(font, *txt);
      }
    }
    txt++;
  }
  glPopLight();
  glEnable(GL_DEPTH_TEST);
}

void glDrawRect(float x1, float y1, float z1, float x2, float y2, float z2,
                float x3, float y3, float z3, float x4, float y4, float z4) {
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  glBegin(GL_POLYGON);
  glVertex3f(x1, y1, z1);
  glVertex3f(x2, y2, z2);
  glVertex3f(x3, y3, z3);
  glVertex3f(x4, y4, z4);
  glVertex3f(x1, y1, z1);
  glEnd();
}

void glDrawRect(float x1, float y1, float z1, float x2, float y2, float z2,
                float x3, float y3, float z3, float x4, float y4, float z4,
                float r, float g, float b) {
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  glBegin(GL_POLYGON);
  glColor(r, g, b);
  glVertex3f(x1, y1, z1);
  glVertex3f(x2, y2, z2);
  glVertex3f(x3, y3, z3);
  glVertex3f(x4, y4, z4);
  glVertex3f(x1, y1, z1);
  glEnd();
}

void glDrawRect(float x, float y, float z, float r) {
  glDrawRect(x-r, y-r, z, x-r, y+r, z, x+r, y+r, z, x+r, y-r, z);
}

void glDrawFloor(float x, float r, float g, float b) {
  x/=2.;
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  glColor(r, g, b);
  glBegin(GL_POLYGON);
  glNormal3f(0, 0, 1);
  glVertex3f(-x, -x, 0.);
  glVertex3f(x, -x, 0.);
  glVertex3f(x, x, 0.);
  glVertex3f(-x, x, 0.);
  glVertex3f(-x, -x, 0.);
  glEnd();
#if 1
  glColor(.75, .75, .75);
  for(int i=-4; i<=4; i++) {
    glBegin(GL_LINES);
    glVertex3f(i*x/5., -x, 0.001);
    glVertex3f(i*x/5., x, 0.001);
    glEnd();
    glBegin(GL_LINES);
    glVertex3f(-x, i*x/5., 0.001);
    glVertex3f(x, i*x/5., 0.001);
    glEnd();
  }

  glColor(.25, .25, .25);
  glBegin(GL_LINE_STRIP);
  glVertex3f(-x, -x, 0.002);
  glVertex3f(-x, x, 0.002);
  glVertex3f(x, x, 0.002);
  glVertex3f(x, -x, 0.002);
  glVertex3f(-x, -x, 0.002);
  glEnd();
#endif
}

void glDrawBox(float x, float y, float z, bool linesOnly) {
  static GLfloat n[6][3] = {
    {-1.0, 0.0, 0.0},
    {0.0, 1.0, 0.0},
    {1.0, 0.0, 0.0},
    {0.0, -1.0, 0.0},
    {0.0, 0.0, 1.0},
    {0.0, 0.0, -1.0}
  };
  static GLint faces[6][4] = {
    {0, 1, 2, 3},
    {3, 2, 6, 7},
    {7, 6, 5, 4},
    {4, 5, 1, 0},
    {5, 6, 2, 1},
    {7, 4, 0, 3}
  };
  static GLint edges[12][2] = {
    {0,1}, {1,2}, {2,3}, {3,0},
    {4,5}, {5,6}, {6,7}, {7,4},
    {0,4}, {1,5}, {2,6}, {3,7}
  };
  GLfloat v[8][3];
  GLint i;

  v[0][0] = v[1][0] = v[2][0] = v[3][0] = -x / 2;
  v[4][0] = v[5][0] = v[6][0] = v[7][0] =  x / 2;
  v[0][1] = v[1][1] = v[4][1] = v[5][1] =  -y / 2;
  v[2][1] = v[3][1] = v[6][1] = v[7][1] =  y / 2;
  v[0][2] = v[3][2] = v[4][2] = v[7][2] =  -z / 2;
  v[1][2] = v[2][2] = v[5][2] = v[6][2] =  z / 2;

  if(!linesOnly){
    glBegin(GL_QUADS);
    for(i = 5; i >= 0; i--) {
      glNormal3fv(n[i]);
      glVertex3fv(v[faces[i][0]]);
      glVertex3fv(v[faces[i][1]]);
      glVertex3fv(v[faces[i][2]]);
      glVertex3fv(v[faces[i][3]]);
    }
    glEnd();
  }else{
    glBegin(GL_LINES);
    for(uint i=0;i<12;i++) {
      glVertex3fv(v[edges[i][0]]);
      glVertex3fv(v[edges[i][1]]);
    }
    glEnd();
  }
}

void glDrawDiamond(float x, float y, float z) {
//  glDisable(GL_CULL_FACE);
  glBegin(GL_TRIANGLE_FAN);
  glVertex3f(.0, .0, z);
  glVertex3f(x, .0, .0);
  glVertex3f(.0, y, .0);
  glVertex3f(-x, .0, .0);
  glVertex3f(.0, -y, .0);
  glVertex3f(x, .0, .0);
  glEnd();
  glBegin(GL_TRIANGLE_FAN);
  glVertex3f(.0, .0, -z);
  glVertex3f(x, .0, .0);
  glVertex3f(.0, -y, .0);
  glVertex3f(-x, .0, .0);
  glVertex3f(.0, y, .0);
  glVertex3f(x, .0, .0);
  glEnd();
//  glEnable(GL_CULL_FACE);
}

void glDrawDiamond(float x, float y, float z, float dx, float dy, float dz) {
  glPushMatrix();
  glTranslated(x, y, z);
  glDrawDiamond(dx, dy, dz);
  glPopMatrix();
}

void glDrawAxis() {
//    glDisable(GL_CULL_FACE);
  GLUquadric *style=gluNewQuadric();
  glBegin(GL_LINES);
  glVertex3f(0, 0, 0);
  glVertex3f(.95, 0, 0);
  glEnd();
  glTranslatef(.8, 0, 0);
  glRotatef(90, 0, 1, 0);
  gluCylinder(style, .08, 0, .2, 20, 1);
  gluDeleteQuadric(style);
//    glEnable(GL_CULL_FACE);
}

void glDrawAxes(double scale) {
  for(uint i=0; i<3; i++) {
    glPushMatrix();
    glScalef(scale, scale, scale);
    switch(i) {
      case 0:  glColor(.7, 0, 0);  break;
      case 1:  glColor(0, .7, 0);  glRotatef(90, 0, 0, 1);  break;
      case 2:  glColor(0, 0, .7);  glRotatef(90, 0, -1, 0);  break;
    }
    glDrawAxis();
    glPopMatrix();
  }
}

void glDrawSphere(float radius) {
  GLUquadric *style=gluNewQuadric();
  gluSphere(style, radius, 10, 10); // last two value for detail
  gluDeleteQuadric(style);
}

void glDrawDisk(float radius) {
  GLUquadric *style=gluNewQuadric();
  gluDisk(style, 0, radius, 10, 1);
  gluDeleteQuadric(style);
}

void glDrawCylinder(float radius, float length, bool closed) {
  GLUquadric *style=gluNewQuadric();
  glTranslatef(0, 0, -length/2);
  gluCylinder(style, radius, radius, length, 20, 1);
  if(closed) {
    glScalef(-1, 1, 1);  //flip orientation of triangles...
    gluDisk(style, 0, radius, 20, 1);
    glTranslatef(0, 0, length);
    glScalef(-1, 1, 1);
    gluDisk(style, 0, radius, 20, 1);
    glTranslatef(0, 0, -length/2);
  } else {
    glTranslatef(0, 0, length/2);
  }
  gluDeleteQuadric(style);
}

void glDrawCappedCylinder(float radius, float length) {
  GLUquadric *style1=gluNewQuadric();
  GLUquadric *style2=gluNewQuadric();
  GLUquadric *style3=gluNewQuadric();

  glTranslatef(0, 0, -length/2);
  gluCylinder(style1, radius, radius, length, 20, 1);
  glTranslatef(0, 0, length);
  gluSphere(style2, radius, 10, 10);
  glTranslatef(0, 0, -length);
  gluSphere(style3, radius, 10, 10);

  gluDeleteQuadric(style1);
  gluDeleteQuadric(style2);
  gluDeleteQuadric(style3);
}

void glDrawGridBox(float x=10.) {
  x/=2.;
  glBegin(GL_LINE_LOOP);
  glVertex3f(-x, -x, -x);
  glVertex3f(-x, -x, x);
  glVertex3f(-x, x, x);
  glVertex3f(-x, x, -x);
  glEnd();
  glBegin(GL_LINE_LOOP);
  glVertex3f(x, -x, -x);
  glVertex3f(x, -x, x);
  glVertex3f(x, x, x);
  glVertex3f(x, x, -x);
  glEnd();
  glBegin(GL_LINES);
  glVertex3f(x, x, x);
  glVertex3f(-x, x, x);
  glVertex3f(x, -x, x);
  glVertex3f(-x, -x, x);
  glVertex3f(x, x, -x);
  glVertex3f(-x, x, -x);
  glVertex3f(x, -x, -x);
  glVertex3f(-x, -x, -x);
  glEnd();
}

void glDrawGridBox(float x1, float y1, float z1, float x2, float y2, float z2) {
  glBegin(GL_LINE_STRIP);
  glVertex3f(x1, y1, z1+0.001);
  glVertex3f(x2, y1, z1+0.001);
  glVertex3f(x2, y2, z1+0.001);
  glVertex3f(x1, y2, z1+0.001);
  glVertex3f(x1, y1, z1+0.001);
  glEnd();

  glBegin(GL_LINES);
  glVertex3f(x2, y2, z1 +0.001);
  glVertex3f(x2, y2, z2 +0.001);
  glEnd();
}

void glDrawKhepera() {
  GLUquadric *style=gluNewQuadric();
  glPushMatrix();
  glRotatef(-90, 1, 0, 0);
  glColor3f(.3, .3, .3);
  gluCylinder(style, 1.5, 1.5, 2, 20, 1);
  glPopMatrix();

  glColor3f(1, 0, 0);
  glBegin(GL_LINES);
  glVertex3f(0, 2, 0);
  glVertex3f(0, 2, -2.5);
  glEnd();
  gluDeleteQuadric(style);
}

void glMakeSquare(int num) {
  glNewList(num, GL_COMPILE);
  glColor3f(1., 0., 0.);
  glBegin(GL_LINE_LOOP);
  glVertex3f(-1 , -1 , 0.);
  glVertex3f(-1 , +1 , 0.);
  glVertex3f(+1 , +1 , 0.);
  glVertex3f(+1 , -1 , 0.);
  glEnd();
  glEndList();
}

void glMakeStdSimplex(int num) {
  glNewList(num, GL_COMPILE);
  //glPolygonMode(GL_BACK, GL_FILL);
  glShadeModel(GL_SMOOTH);
  glBegin(GL_TRIANGLE_FAN);
  glColor3f(1., 1., 1.);
  glVertex3f(0., 0., 0.);
  glColor3f(1., 0., 0.);
  glVertex3f(1., 0., 0.);
  glColor3f(0., 1., 0.);
  glVertex3f(0., 1., 0.);
  glColor3f(0., 0., 1.);
  glVertex3f(0., 0., 1.);
  glColor3f(1., 0., 0.);
  glVertex3f(1., 0., 0.);
  glEnd();
  /*
    glColor4f(.5, .5, .5, .9);
    glBegin(GL_POLYGON);
    glVertex3f( 1. , 0. , 0. );
    glVertex3f( 0. , 1. , 0. );
    glVertex3f( 0. , 0. , 1. );
    glEnd();
  */
  glEndList();
}

void glMakeTorus(int num) {
  glNewList(num, GL_COMPILE);

  GLint i, j, rings, sides;
  float theta1, phi1, theta2, phi2;
  float v0[03], v1[3], v2[3], v3[3];
  float t0[03], t1[3], t2[3], t3[3];
  float n0[3], n1[3], n2[3], n3[3];
  float innerRadius=0.4;
  float outerRadius=0.8;
  float scalFac;

  rings = 8;
  sides = 10;
  scalFac=1/(outerRadius*2);

  for(i=0; i<rings; i++) {
    theta1 = (float)i * 2.0 * MLR_PI / rings;
    theta2 = (float)(i + 1) * 2.0 * MLR_PI / rings;
    for(j=0; j<sides; j++) {
      phi1 = (float)j * 2.0 * MLR_PI / sides;
      phi2 = (float)(j + 1) * 2.0 * MLR_PI / sides;

      v0[0] = cos(theta1) * (outerRadius + innerRadius * cos(phi1));
      v0[1] =-sin(theta1) * (outerRadius + innerRadius * cos(phi1));
      v0[2] = innerRadius * sin(phi1);

      v1[0] = cos(theta2) * (outerRadius + innerRadius * cos(phi1));
      v1[1] =-sin(theta2) * (outerRadius + innerRadius * cos(phi1));
      v1[2] = innerRadius * sin(phi1);

      v2[0] = cos(theta2) * (outerRadius + innerRadius * cos(phi2));
      v2[1] =-sin(theta2) * (outerRadius + innerRadius * cos(phi2));
      v2[2] = innerRadius * sin(phi2);

      v3[0] = cos(theta1) * (outerRadius + innerRadius * cos(phi2));
      v3[1] =-sin(theta1) * (outerRadius + innerRadius * cos(phi2));
      v3[2] = innerRadius * sin(phi2);

      n0[0] = cos(theta1) * (cos(phi1));
      n0[1] =-sin(theta1) * (cos(phi1));
      n0[2] = sin(phi1);

      n1[0] = cos(theta2) * (cos(phi1));
      n1[1] =-sin(theta2) * (cos(phi1));
      n1[2] = sin(phi1);

      n2[0] = cos(theta2) * (cos(phi2));
      n2[1] =-sin(theta2) * (cos(phi2));
      n2[2] = sin(phi2);

      n3[0] = cos(theta1) * (cos(phi2));
      n3[1] =-sin(theta1) * (cos(phi2));
      n3[2] = sin(phi2);

      t0[0] = v0[0]*scalFac + 0.5;
      t0[1] = v0[1]*scalFac + 0.5;
      t0[2] = v0[2]*scalFac + 0.5;

      t1[0] = v1[0]*scalFac + 0.5;
      t1[1] = v1[1]*scalFac + 0.5;
      t1[2] = v1[2]*scalFac + 0.5;

      t2[0] = v2[0]*scalFac + 0.5;
      t2[1] = v2[1]*scalFac + 0.5;
      t2[2] = v2[2]*scalFac + 0.5;

      t3[0] = v3[0]*scalFac + 0.5;
      t3[1] = v3[1]*scalFac + 0.5;
      t3[2] = v3[2]*scalFac + 0.5;

      if((i+j)%2) glColor3f(0., 1., 0.);
      else glColor3f(0., 0., 1.);

      glBegin(GL_POLYGON);
      glNormal3fv(n3); glTexCoord3fv(t3); glVertex3fv(v3);
      glNormal3fv(n2); glTexCoord3fv(t2); glVertex3fv(v2);
      glNormal3fv(n1); glTexCoord3fv(t1); glVertex3fv(v1);
      glNormal3fv(n0); glTexCoord3fv(t0); glVertex3fv(v0);
      glEnd();
    }
  }
  glEndList();
}

uint glImageTexture(const byteA &img) {
  GLuint texName;

  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glGenTextures(1, &texName);

//  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, texName);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  switch(img.d2) {
    case 0:
    case 1:
      glTexImage2D(GL_TEXTURE_2D, 0, 4, img.d1, img.d0, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, img.p);
      break;
    case 2:
      glTexImage2D(GL_TEXTURE_2D, 0, 4, img.d1, img.d0, 0, GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, img.p);
      break;
    case 3:
      glTexImage2D(GL_TEXTURE_2D, 0, 4, img.d1, img.d0, 0, GL_RGB, GL_UNSIGNED_BYTE, img.p);
      break;
    case 4:
      glTexImage2D(GL_TEXTURE_2D, 0, 4, img.d1, img.d0, 0, GL_RGBA, GL_UNSIGNED_BYTE, img.p);
      break;
    default:
      HALT("no image fomat");
  }
  return texName;
}

void glDrawTexQuad(const byteA& texImg,
                   float x1, float y1, float z1, float x2, float y2, float z2,
                   float x3, float y3, float z3, float x4, float y4, float z4,
                   float mulX, float mulY) {
  glDisable(GL_CULL_FACE);
  glEnable(GL_TEXTURE_2D);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL); //GL_MODULATE);
  glTexImage2D(GL_TEXTURE_2D, 0, 3, texImg.d1, texImg.d0, 0, GL_RGB, GL_UNSIGNED_BYTE, texImg.p);
//  glBindTexture(GL_TEXTURE_2D, texture);
  glBegin(GL_QUADS);
  glTexCoord2f(0.0,  mulY); glVertex3f(x1, y1, z1);
  glTexCoord2f(mulX, mulY); glVertex3f(x2, y2, z2);
  glTexCoord2f(mulX, 0.0);  glVertex3f(x3, y3, z3);
  glTexCoord2f(0.0,  0.0);  glVertex3f(x4, y4, z4);
  glEnd();
  glDisable(GL_TEXTURE_2D);
  glEnable(GL_CULL_FACE);

}

#ifdef MLR_GLUT
/** @brief return the RGBA-image of scenery drawn just before; the image
  buffer has to have either 2 dimensions [width, height] for a
  gray-scale luminance image or 3 dimensions [width, height, 4] for an
  RGBA-image. */
void glGrabImage(byteA& image) {
  if(!image.N) image.resize(glutGet(GLUT_WINDOW_HEIGHT), glutGet(GLUT_WINDOW_WIDTH), 3);
  CHECK(image.nd==2 || image.nd==3, "not an image format");
  GLint w=image.d1, h=image.d0;
  //CHECK(w<=glutGet(GLUT_WINDOW_WIDTH) && h<=glutGet(GLUT_WINDOW_HEIGHT), "grabbing large image from small window:" <<w <<' ' <<h <<' ' <<glutGet(GLUT_WINDOW_WIDTH) <<' ' <<glutGet(GLUT_WINDOW_HEIGHT));
  if(image.d1%4) {  //necessary: extend the image to have width mod 4
    uint add=4-(image.d1%4);
    if(image.nd==2) image.resize(image.d0, image.d1+add);
    if(image.nd==3) image.resize(image.d0, image.d1+add, image.d2);
  }
  glReadBuffer(GL_FRONT);
//  glReadBuffer(GL_BACK);

  //glPixelStorei(GL_PACK_SWAP_BYTES, 0);
  glPixelStorei(GL_PACK_ALIGNMENT,4);
  switch(image.d2) {
    case 0:
    case 1:
      glPixelTransferf(GL_RED_SCALE, .3333);
      glPixelTransferf(GL_GREEN_SCALE, .3333);
      glPixelTransferf(GL_BLUE_SCALE, .3333);
      glReadPixels(0, 0, w, h, GL_LUMINANCE, GL_UNSIGNED_BYTE, image.p);
      glPixelTransferf(GL_RED_SCALE, 1.);
      glPixelTransferf(GL_GREEN_SCALE, 1.);
      glPixelTransferf(GL_BLUE_SCALE, 1.);
      break;
//    case 2:
//      //glReadPixels(0, 0, w, h, GL_GA, GL_UNSIGNED_BYTE, image.p);
//      break;
    case 3:
      glReadPixels(0, 0, w, h, GL_BGR, GL_UNSIGNED_BYTE, image.p);
    break;
    case 4:
#if defined MLR_SunOS
      glReadPixels(0, 0, w, h, GL_ABGR_EXT, GL_UNSIGNED_BYTE, image.p);
#else
#if defined MLR_Cygwin
      glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, image.p);
#else
      //glReadPixels(0, 0, w, h, GL_BGRA_EXT, GL_UNSIGNED_BYTE, image.p);
      glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, image.p);
#endif
#endif
      break;
    default: HALT("wrong image format");
  }
}
#else
void glGrabImage(byteA& image) { NICO }
#endif

/** @brief return the depth map of the scenery drawn just before; the depth
    buffer has to be a 2-dimensional [width, height] and is filled with
    depth values between 0 and 1. */
void glGrabDepth(byteA& depth) {
  if(!depth.N) depth.resize(glutGet(GLUT_WINDOW_HEIGHT), glutGet(GLUT_WINDOW_WIDTH));
  CHECK_EQ(depth.nd,2, "depth buffer has to be either 2-dimensional");
  GLint w=depth.d1, h=depth.d0;
  glReadPixels(0, 0, w, h, GL_DEPTH_COMPONENT, GL_UNSIGNED_BYTE, depth.p);
}

/** @brief return the depth map of the scenery drawn just before; the depth
    buffer has to be a 2-dimensional [width, height] and is filled with
    depth values between 0 and 1. */
void glGrabDepth(floatA& depth) {
  if(!depth.N) depth.resize(glutGet(GLUT_WINDOW_HEIGHT), glutGet(GLUT_WINDOW_WIDTH));
  CHECK_EQ(depth.nd,2, "depth buffer has to be 2-dimensional");
  GLint w=depth.d1, h=depth.d0;
  glReadPixels(0, 0, w, h, GL_DEPTH_COMPONENT, GL_FLOAT, depth.p);
}

void glRasterImage(float x, float y, byteA &img, float zoom) {
  glRasterPos3f(x, y, 0.); //(int)(y+zoom*img.d0)); (the latter was necessary for other pixel/raster coordinates)
  glPixelZoom(zoom, -zoom);
  if(img.d1%4) {  //necessary: extend the image to have width mod 4
    uint P=img.d2;
    if(!P) P=1;
    uint add=4-(img.d1%4);
    img.reshape(img.d0, img.d1*P);
    img.insColumns(img.d1, add*P);
    if(P>1) img.reshape(img.d0, img.d1/P, P);
  }

  switch(img.d2) {
    case 0:
    case 1:  glDrawPixels(img.d1, img.d0, GL_LUMINANCE, GL_UNSIGNED_BYTE, img.p);        break;
    case 2:  glDrawPixels(img.d1, img.d0, GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, img.p);  break;
    case 3:  glDrawPixels(img.d1, img.d0, GL_RGB, GL_UNSIGNED_BYTE, img.p);              break;
    case 4:  glDrawPixels(img.d1, img.d0, GL_RGBA, GL_UNSIGNED_BYTE, img.p);             break;
    default: HALT("no image format");
  };
}

void OpenGL::watchImage(const floatA &_img, bool wait, float _zoom) {
  static byteA img;
  resizeAs(img, _img);
  float x;
  for(uint i=0; i<img.N; i++) {
    x=_img.elem(i);
    img.elem(i) = (x<0.)?0:((x>255.)?255:x);
  }
  watchImage(img, wait, _zoom);
}

void OpenGL::watchImage(const byteA &_img, bool wait, float _zoom) {
  background=_img;
  backgroundZoom=_zoom;
  //resize(img->d1*zoom,img->d0*zoom);
  if(wait) watch(); else update();
}

/*void glWatchImage(const floatA &x, bool wait, float zoom){
  double ma=x.max();
  double mi=x.min();
  if(wait) cout <<"watched image min/max = " <<mi <<' ' <<ma <<endl;
  byteA img;
  img.resize(x.d0*x.d1);
  img.setZero();
  for(uint i=0;i<x.N;i++){
    img(i)=(byte)(255.*(x.elem(i)-mi)/(ma-mi));
  }
  img.reshape(x.d0, x.d1);
  glWatchImage(img, wait, 20);
}*/

void OpenGL::displayGrey(const arr &x, bool wait, float _zoom) {
  static byteA img;
  resizeAs(img, x);
  double mi=x.min(), ma=x.max();
  text.clear() <<"displayGrey" <<" max=" <<ma <<"min=" <<mi <<endl;
  for(uint i=0; i<x.N; i++) {
    img.elem(i)=(byte)(255.*(x.elem(i)-mi)/(ma-mi));
  }
  watchImage(img, wait, _zoom);
}

void OpenGL::displayRedBlue(const arr &x, bool wait, float _zoom) {
  double mi=x.min(), ma=x.max();
  text.clear() <<"max=" <<ma <<"min=" <<mi <<endl;
//  cout <<"\rdisplay" <<win <<" max=" <<ma <<"min=" <<mi;
  static byteA img;
  img.resize(x.d0*x.d1, 3);
  img.setZero();
  for(uint i=0; i<x.N; i++) {
    if(x.elem(i)>0.) img(i, 0)=(byte)(255.*x.elem(i)/ma);
    if(x.elem(i)<0.) img(i, 2)=(byte)(255.*x.elem(i)/mi);
  }
  img.reshape(x.d0, x.d1, 3);
  watchImage(img, wait, _zoom);
}

void glDrawUI(void *p) {
  glPushName(0x10);
  ((glUI*)p)->glDraw();
  glPopName();
}

bool glUI::hoverCallback(OpenGL& gl) {
  //bool b=
  checkMouse(gl.mouseposx, gl.mouseposy);
  //if(b) postRedrawEvent(false);
  return true;
}

bool glUI::clickCallback(OpenGL& gl) {
  bool b=checkMouse(gl.mouseposx, gl.mouseposy);
  if(b) gl.postRedrawEvent(true);
  int t=top;
  if(t!=-1) {
    cout <<"CLICK! on button #" <<t <<endl;
    gl.watching.setStatus(0);
    return false;
  }
  return true;
}

#ifdef MLR_FREEGLUT
void glSelectWin(uint win) {
  if(!staticgl[win]) staticgl[win]=new OpenGL;
  glutSetWindow(staticgl[win]->s->windowID);
}
#endif

#else /// MLR_GL
void glColor(int col) { NICO }
void glColor(float, float, float, float) { NICO }
void glDrawDiamond(float, float, float, float, float, float) { NICO }
// void glStandardLight(void*) { NICO }   // TOBIAS: das hier wird doch schon ueber opengl_void.cxx definiert
void glStandardScene(void*) { NICO }
uint glImageTexture(const byteA &img) { NICO }
void glDrawTexQuad(uint texture,
                   float x1, float y1, float z1, float x2, float y2, float z2,
                   float x3, float y3, float z3, float x4, float y4, float z4,
                   float mulX, float mulY) { NICO }
void OpenGL::watchImage(const floatA &_img, bool wait, float _zoom) { NICO }
void OpenGL::watchImage(const byteA &_img, bool wait, float _zoom) { NICO }
void glDrawUI(void *p) { NICO }
bool glUI::hoverCallback(OpenGL& gl) { NICO }
bool glUI::clickCallback(OpenGL& gl) { NICO }
#endif

//===========================================================================
//
// standalone draw routines for large data structures
//

#ifdef MLR_GL
#endif

//===========================================================================
//
// OpenGL implementations
//

OpenGL::OpenGL(const char* _title, int w, int h, int posx, int posy)
  : s(NULL), title(_title), width(w), height(h), reportEvents(false), topSelection(NULL), captureImg(false), captureDep(false), fboId(0), rboColor(0), rboDepth(0){
  //MLR_MSG("creating OpenGL=" <<this);
  Reshape(w,h);
  s=new sOpenGL(this); //this might call some callbacks (Reshape/Draw) already!
  init();
}

OpenGL::OpenGL(void *container)
  : s(NULL), width(0), height(0), reportEvents(false), topSelection(NULL), captureImg(false), captureDep(false), fboId(0), rboColor(0), rboDepth(0){
  s=new sOpenGL(this,container); //this might call some callbacks (Reshape/Draw) already!
  init();
}

OpenGL::~OpenGL() {
  clear();
  delete s;
  s=NULL;
}

OpenGL* OpenGL::newClone() const {
  OpenGL* gl=new OpenGL;
  //*(gl->s) = *s; //don't clone internal stuff!
  gl->drawers = drawers;
  gl->camera = camera;
  return gl;
}

void OpenGL::init() {
  camera.setPosition(0., 0., 10.);
  camera.focus(0, 0, 0);
  camera.setZRange(.1, 1000.);
  camera.setHeightAngle(12.);

  drawFocus=false;
  clearR=clearG=clearB=1.; clearA=0.;
  drawers.memMove=true;
  mouseposx=mouseposy=0;
  mouse_button=0;
  mouseIsDown=false;
  mouseView=-1;

  reportEvents=false;
  reportSelects=false;
  immediateExitLoop=false;
  exitkeys="";

  backgroundZoom=1;
};

struct CstyleDrawer : GLDrawer{
  void *classP;
  void (*call)(void*);
  CstyleDrawer(void (*call)(void*), void* classP): classP(classP), call(call){}
  void glDraw(OpenGL&){ call(classP); }
};

struct CstyleInitCall : OpenGL::GLInitCall{
  void *classP;
  void (*call)(void*);
  CstyleInitCall(void (*call)(void*), void* classP): classP(classP), call(call){}
  bool glInit(OpenGL&){ call(classP); return true; }
};

/// add a draw routine
void OpenGL::add(void (*call)(void*), void* classP) {
  CHECK(call!=0, "OpenGL: NULL pointer to drawing routine");
  dataLock.writeLock();
  drawers.append(new CstyleDrawer(call, classP));
  dataLock.unlock();
}

/// add a draw routine
void OpenGL::addInit(void (*call)(void*), void* classP) {
  CHECK(call!=0, "OpenGL: NULL pointer to drawing routine");
  dataLock.writeLock();
  initCalls.append(new CstyleInitCall(call, classP));
  dataLock.unlock();
}

/// add a draw routine to a view
void OpenGL::addView(uint v, void (*call)(void*), void* classP) {
  CHECK(call!=0, "OpenGL: NULL pointer to drawing routine");
  dataLock.writeLock();
  if(v>=views.N) views.resizeCopy(v+1);
  views(v).drawers.append(new CstyleDrawer(call, classP));
  dataLock.unlock();
}

void OpenGL::setViewPort(uint v, double l, double r, double b, double t) {
  dataLock.writeLock();
  if(v>=views.N) views.resizeCopy(v+1);
  views(v).le=l;  views(v).ri=r;  views(v).bo=b;  views(v).to=t;
  dataLock.unlock();
}

/// remove a draw routine
//void OpenGL::remove(void (*call)(void*), const void* classP) {
//  CHECK(call!=0, "OpenGL: NULL pointer to drawing routine");
//  uint i;
//  for(i=0; i<drawers.N; i++) if(drawers(i).call==call && drawers(i).classP==classP) break;
//  CHECK(i<drawers.N, "value to remove not found");
//  drawers.remove(i, 1);
//}

/// clear the list of all draw and callback routines
void OpenGL::clear() {
  dataLock.writeLock();
  for(auto& x:drawers) if(CstyleDrawer* d = dynamic_cast<CstyleDrawer*>(x)) delete d;
  views.clear();
  drawers.clear();
  initCalls.clear();
  hoverCalls.clear();
  clickCalls.clear();
  keyCalls.clear();
  dataLock.unlock();
}

void OpenGL::Draw(int w, int h, mlr::Camera *cam, bool callerHasAlreadyLocked) {
#ifdef MLR_GL

  if(!callerHasAlreadyLocked){
    singleGLAccess.mutex.lock();
    dataLock.readLock(); //now accessing user data
  }

  //clear bufferer
  GLint viewport[4] = {0, 0, w, h};
  glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
  glClearColor(clearR, clearG, clearB, clearA);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  //raster an image as background
  if(background.N) {
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glOrtho(0, 1., 1., 0., -1., 1.); //only affects the offset - the rest is done with raster zooms
    glDisable(GL_DEPTH_TEST);
    glRasterImage(0, 0, background, backgroundZoom); //.5*w/background.d1);
  }

  //OpenGL initialization
  glEnable(GL_DEPTH_TEST);  glDepthFunc(GL_LESS);
  glEnable(GL_BLEND);  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_CULL_FACE);  glFrontFace(GL_CCW);
  glShadeModel(GL_FLAT);  //glShadeModel(GL_SMOOTH);

  //select mode?
  GLint mode;
  glGetIntegerv(GL_RENDER_MODE, &mode);

  //projection
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  if(mode==GL_SELECT) gluPickMatrix((GLdouble)mouseposx, (GLdouble)mouseposy, 2., 2., viewport);
  if(!cam) camera.glSetProjectionMatrix();
  else     cam->glSetProjectionMatrix();
  //glLineWidth(2);

  //**extract the camera projection matrix
#if 0
  //this is the calibration matrix corresponding to OpenGL's ``viewport''
  intA view(4);
  arr Kview(3, 3);
  glGetIntegerv(GL_VIEWPORT, view.p);
  Kview.setZero();
  Kview(0, 0) = .5*view(2); Kview(0, 2) = view(0)+.5*view(2);
  Kview(1, 1) = .5*view(3); Kview(1, 2) = view(1)+.5*view(3);
  Kview(2, 2) = 1.;  //OpenGL's w coordinate is the negative of what we want...
  //the projection matrix (without viewport-calibration) from OpenGL:
  P.resize(4, 4);
  glGetDoublev(GL_PROJECTION_MATRIX, P.p);
  //cout <<"OpenGL's glP=" <<P <<"\nK=" <<Kview <<endl;
  //double sca=P.elem(0);
  P = ~P;      //OpenGL uses transposed matrix storage convention
  P.delRows(2); //We're not interested in OpenGL's ``z-culling-coordinate'', only in the perspective coordinate (divisor) w
  //P[2]() *=-1.;
  //the full camera projection matrix:
  P = Kview*P;
  //cout <<"OpenGL's P=" <<P <<endl;

  /*
  double zn=camera.zNear, zf=camera.zFar, f=1./tan(MLR_PI/180.*camera.heightAngle/2.);
  arr Frust(4, 4); Frust.setZero();
  Frust(0, 0) = Frust(1, 1) = f;
  Frust(2, 2) = (zf+zn)/(zn-zf);
  Frust(3, 2) = -1.;
  Frust(2, 3) = 2.*zf*zn/(zn-zf);
  cout <<"OpenGL P=" <<P <<"K=" <<Kview <<"znear=" <<camera.zNear <<"zfar=" <<camera.zFar <<endl;
  cout <<"Frust=" <<Frust <<endl;;
  Frust.delRows(2); //We're not interested in OpenGL's ``z-coordinate'', only in the perspective coordinate (divisor) w
  cout <<"K=" <<Kview*Frust <<endl;
  */
#endif

  //draw focus?
  if(drawFocus && mode!=GL_SELECT) {
    glColor(1., .7, .3);
    double size = .005 * (camera.X.pos-camera.foc).length();
    glDrawDiamond(camera.foc.x, camera.foc.y, camera.foc.z, size, size, size);
  }
  /*if(topSelection && mode!=GL_SELECT){
    glColor(1., .7, .3);
    double size = .005 * (camera.X.pos-camera.foc).length();
    glDrawDiamond(topSelection->x, topSelection->y, topSelection->z, size, size, size);
  }*/

  //std color: black:
  glColor(.3, .3, .5);

  //draw central view
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  if(mode==GL_SELECT) glInitNames();
  for(uint i=0; i<drawers.N; i++) {
    if(mode==GL_SELECT) glLoadName(i);
    drawers(i)->glDraw(*this);
    glLoadIdentity();
  }

  //draw text
  if(text.N) {
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    if(clearR+clearG+clearB>1.) glColor(0.0, 0.0, 0.0, 1.0); else glColor(1.0, 1.0, 1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glOrtho(0., (double)w, (double)h, .0, -1., 1.);
    glDrawText(text, 10, 20, 0);
    glLoadIdentity();
  }

  //draw subviews
  for(uint v=0; v<views.N; v++) {
    GLView *vi=&views(v);
    glViewport(vi->le*w, vi->bo*h, (vi->ri-vi->le)*w, (vi->to-vi->bo)*h);
    //glMatrixMode(GL_MODELVIEW);
    //glLoadIdentity();
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    if(vi->img) {
      glDisable(GL_DEPTH_TEST);
      glRasterImage(-1., 1., *vi->img, backgroundZoom*(vi->ri-vi->le)*w/vi->img->d1);
      glEnable(GL_DEPTH_TEST);
    }
    vi->camera.glSetProjectionMatrix();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    if(drawFocus) {
      glColor(1., .7, .3);
      double size = .005 * (camera.X.pos-camera.foc).length();
      glDrawDiamond(vi->camera.foc.x, vi->camera.foc.y, vi->camera.foc.z, size, size, size);
    }
    for(uint i=0; i<vi->drawers.N; i++) vi->drawers(i)->glDraw(*this);
    if(vi->text.N) {
      glMatrixMode(GL_PROJECTION);
      glLoadIdentity();
      if(clearR+clearG+clearB>1.) glColor(0.0, 0.0, 0.0, 1.0); else glColor(1.0, 1.0, 1.0, 1.0);
      glMatrixMode(GL_MODELVIEW);
      glLoadIdentity();
      //glOrtho(0., (vi->ri-vi->le)*w, (vi->to-vi->bo)*h, .0, -1., 1.);
      glDrawText(vi->text, -.95, .85, 0.);
    }
  }

  //cout <<"UNLOCK draw" <<endl;

  if(captureImg){
    captureImage.resize(h, w, 3);
    glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, captureImage.p);
//    flip_image(captureImage);
    captureImg=false;
  }
  if(captureDep){
    captureDepth.resize(h, w);
    glReadPixels(0, 0, w, h, GL_DEPTH_COMPONENT, GL_FLOAT, captureDepth.p);
//    flip_image(captureDepth);
    captureDep=false;
  }

  //check matrix stack
  GLint s;
  glGetIntegerv(GL_MODELVIEW_STACK_DEPTH, &s);
  if(s!=1) MLR_MSG("OpenGL name stack has not depth 1 (pushs>pops) in DRAW mode:" <<s);
  //CHECK(s<=1, "OpenGL matrix stack has not depth 1 (pushs>pops)");

  if(!callerHasAlreadyLocked){
    dataLock.unlock(); //now de-accessing user data
    singleGLAccess.mutex.unlock();
  }
#endif
}

void OpenGL::Select(bool callerHasAlreadyLocked) {
  if(!callerHasAlreadyLocked){
    singleGLAccess.mutex.lock();
    dataLock.readLock();
  }

#ifdef MLR_GL
  uint i, j, k;

  s->beginGlContext();

  glSelectBuffer(1000, selectionBuffer);
  glRenderMode(GL_SELECT);

#if 1
  GLint w=width, h=height;

  //projection
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  if(mouseView==-1) {
    GLint viewport[4] = {0, 0, w, h};
    gluPickMatrix((GLdouble)mouseposx, (GLdouble)mouseposy, 2., 2., viewport);
    camera.glSetProjectionMatrix();
  } else {
    GLView *vi=&views(mouseView);
    GLint viewport[4] = { (GLint)vi->le*w, (GLint)vi->bo*h, (GLint)(vi->ri-vi->le)*w, (GLint)(vi->to-vi->bo)*h};
    gluPickMatrix((GLdouble)mouseposx, (GLdouble)mouseposy, 2., 2., viewport);
    vi->camera.glSetProjectionMatrix();
  }

  //draw objects
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glInitNames();
  if(mouseView==-1) {
    for(i=0; i<drawers.N; i++) {
      glLoadName(i);
      drawers(i)->glDraw(*this);
      GLint s;
      glGetIntegerv(GL_NAME_STACK_DEPTH, &s);
      if(s!=0) MLR_MSG("OpenGL name stack has not depth 1 (pushs>pops) in SELECT mode:" <<s);
    }
  } else {
    GLView *vi=&views(mouseView);
    for(i=0; i<vi->drawers.N; i++) { glLoadName(i); vi->drawers(i)->glDraw(*this); }
  }

#else
  Draw(width, height, NULL, true);
#endif
  glLoadIdentity();

  GLint n;
  n=glRenderMode(GL_RENDER);
  selection.resize(n);

  GLuint *obj, maxD=(GLuint)(-1);
  topSelection=NULL;
  for(j=0, i=0; i<(uint)n; i++) {
    obj=selectionBuffer+j;
    j+=3+obj[0];

    //get name as superposition of all names
    selection(i).name = 0;
    for(k=0; k<obj[0]; k++) selection(i).name |= obj[3+k];

    //get dim and dmax
    selection(i).dmin=(double)obj[1]/maxD;  //camera.glConvertToTrueDepth(selection(i).dmin);
    selection(i).dmax=(double)obj[2]/maxD;  //camera.glConvertToTrueDepth(selection(i).dmax);

    //get top-most selection
    if(!topSelection || selection(i).dmin < topSelection->dmin) topSelection = &selection(i);
  }

  if(topSelection) {
    topSelection->x=0; topSelection->y=0; topSelection->z=topSelection->dmin;
    unproject(topSelection->x, topSelection->y, topSelection->z);
  }

  if(reportSelects) reportSelection();

  s->endGlContext();
#endif
  if(!callerHasAlreadyLocked){
    dataLock.unlock();
    singleGLAccess.mutex.unlock();
  }
}

/** @brief watch in interactive mode and wait for an exiting event
  (key pressed or right mouse) */
int OpenGL::watch(const char *txt) {
  update(STRING(txt<<" - press ENTER to continue"));
  if(mlr::getInteractivity()){
    watching.setStatus(1);
    watching.waitForStatusEq(0);
//    while(watching.getStatus()!=0){
//      processEvents();
//      sleepForEvents();
//    }
  }else{
    mlr::wait(.5);
  }
  return pressedkey;
}

/// update the view (in Qt: also starts displaying the window)
int OpenGL::update(const char *txt, bool _captureImg, bool _captureDep, bool waitForCompletedDraw) {
  openWindow();
  captureImg |= _captureImg;
  captureDep |= _captureDep;
  if(txt) text.clear() <<txt;
  isUpdating.waitForStatusEq(0);
  isUpdating.setStatus(1);
  postRedrawEvent(false);
  if(captureImg || captureDep || waitForCompletedDraw){ isUpdating.waitForStatusEq(0); }//{ mlr::wait(.01); processEvents(); mlr::wait(.01); }
  return pressedkey;
}

/// waits some msecons before updating
int OpenGL::timedupdate(double sec) {
  static double lasttime=-1;
  double now;
  now=mlr::realTime();
  if(lasttime>0. && now-lasttime<sec) mlr::wait(lasttime+sec-now);
  lasttime=now;
  return update();
#if 0//def MLR_QTGL
  int i;
  quitLoopOnTimer=true;
  i=startTimer(msec);
  Loop();
  killTimer(i);
  return update();
#endif
}

/// set the four clear colors
void OpenGL::setClearColors(float r, float g, float b, float a) {
  clearR=r; clearG=g; clearB=b; clearA=a;
}

/** @brief inverse projection: given a 2D+depth coordinates in the
  camera view (e.g. as a result of selection) computes the world 3D
  coordinates */
void OpenGL::unproject(double &x, double &y, double &z,bool resetCamera) {
#ifdef MLR_GL
  double _x, _y, _z;
  GLdouble modelMatrix[16], projMatrix[16];
  GLint viewPort[4];
  if(resetCamera) {
    GLint viewport[4] = {0, 0, (GLint)width, (GLint)height};
    glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    camera.glSetProjectionMatrix();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
  }
  glGetDoublev(GL_MODELVIEW_MATRIX, modelMatrix);
  glGetDoublev(GL_PROJECTION_MATRIX, projMatrix);
  glGetIntegerv(GL_VIEWPORT, viewPort);
  gluUnProject(x, y, z, modelMatrix, projMatrix, viewPort, &_x, &_y, &_z);
  x=_x; y=_y; z=_z;
#else
  NICO
#endif
}

/// print some info on the selection buffer
void OpenGL::reportSelection() {
  uint i;
  std::cout <<"selection report: mouse=" <<mouseposx <<" " <<mouseposy <<" -> #selections=" <<selection.N <<std::endl;
  for(i=0; i<selection.N; i++) {
    if(topSelection == &selection(i)) std::cout <<"  TOP: "; else std::cout <<"       ";
    std::cout
        <<"name = 0x" <<std::hex <<selection(i).name <<std::dec
        <<" min-depth:" <<selection(i).dmin <<" max-depth:" <<selection(i).dmax
        <<" 3D: (" <<selection(i).x <<',' <<selection(i).y <<',' <<selection(i).z <<')'
        <<endl;
  }
}

#ifdef MLR_GL2PS
/** @brief generates a ps from the current OpenGL display, using gl2ps */
void OpenGL::saveEPS(const char *filename) {
  FILE *fp = fopen(filename, "wb");
  GLint buffsize = 0, state = GL2PS_OVERFLOW;
  GLint viewport[4];
  glGetIntegerv(GL_VIEWPORT, viewport);
  while(state==GL2PS_OVERFLOW) {
    buffsize+=1024*1024;
    gl2psBeginPage("Marc Toussaint", "MT", viewport,
                   GL2PS_EPS, GL2PS_BSP_SORT, GL2PS_SILENT |
                   GL2PS_SIMPLE_LINE_OFFSET | GL2PS_NO_BLENDING |
                   GL2PS_OCCLUSION_CULL | GL2PS_BEST_ROOT,
                   GL_RGBA, 0, NULL, 0, 0, 0, buffsize,
                   fp, filename);
    Draw(width, height);
    state = gl2psEndPage();
  }
  fclose(fp);
}
#else
void OpenGL::saveEPS(const char*) {
  MLR_MSG("WARNING: OpenGL::saveEPS was called without MLR_GL2PS configured!");
}
#endif

#ifndef MLR_QTGL
/** @brief report on the OpenGL capabilities (the QGLFormat) */
void OpenGL::about(std::ostream& os) { MLR_MSG("NICO"); }
#endif


//===========================================================================
//
// callbacks
//

#if 1
#  define CALLBACK_DEBUG(x) if(reportEvents) { cout <<MLR_HERE <<s <<':'; x; }
#else
#  define CALLBACK_DEBUG(x)
#endif

void getSphereVector(mlr::Vector& vec, int _x, int _y, int le, int ri, int bo, int to) {
  int w=ri-le, h=to-bo;
  int minwh = w<h?w:h;
  double x, y;
  x=(double)_x;  x=x-le-.5*w;   x*= 2./minwh;
  y=(double)_y;  y=y-bo-.5*h; y*= 2./minwh;
  vec.x=x;
  vec.y=y;
  vec.z=.5-(x*x+y*y);
  if(vec.z<0.) vec.z=0.;
}

void OpenGL::Reshape(int _width, int _height) {
  dataLock.writeLock();
  CALLBACK_DEBUG(printf("Window %d Reshape Callback:  %d %d\n", 0, _width, _height));
  width=_width;
  height=_height;
  if(width%4) width = 4*(width/4);
  if(height%2) height = 2*(height/2);
  camera.setWHRatio((double)width/height);
  for(uint v=0; v<views.N; v++) views(v).camera.setWHRatio((views(v).ri-views(v).le)*width/((views(v).to-views(v).bo)*height));
  dataLock.unlock();
}

void OpenGL::Key(unsigned char key, int _x, int _y) {
  dataLock.writeLock();
  _y = height-_y;
  CALLBACK_DEBUG(printf("Window %d Keyboard Callback:  %d (`%c') %d %d\n", 0, key, (char)key, _x, _y));
  mouseposx=_x; mouseposy=_y;
  pressedkey=key;

  bool cont=true;
  for(uint i=0; i<keyCalls.N; i++) cont=cont && keyCalls(i)->keyCallback(*this);

  if(key==13 || key==27 || key=='q' || mlr::contains(exitkeys, key)) watching.setStatus(0);
  dataLock.unlock();
}

void OpenGL::Mouse(int button, int downPressed, int _x, int _y) {
  dataLock.writeLock();
  int w=width, h=height;
  _y = h-_y;
  CALLBACK_DEBUG(printf("Window %d Mouse Click Callback:  %d %d %d %d\n", 0, button, downPressed, _x, _y));
  mouse_button=1+button;
  if(downPressed) mouse_button=-1-mouse_button;
  mouseposx=_x; mouseposy=_y;
  lastEvent.set(mouse_button, -1, _x, _y, 0., 0.);

  GLView *v;
  mlr::Camera *cam=&camera;
  mlr::Vector vec;
  for(mouseView=views.N; mouseView--;) {
    v=&views(mouseView);
    if(_x<v->ri*w && _x>v->le*w && _y<v->to*h && _y>v->bo*h) {
      getSphereVector(vec, _x, _y, v->le*w, v->ri*w, v->bo*h, v->to*h);
      cam=&views(mouseView).camera;
      break;
    }
  }
  if(mouseView==-1) getSphereVector(vec, _x, _y, 0, w, 0, h);
  CALLBACK_DEBUG(cout <<"associated to view " <<mouseView <<" x=" <<vec.x <<" y=" <<vec.y <<endl);

  if(!downPressed) {  //down press
    if(mouseIsDown){ dataLock.unlock(); return; } //the button is already down (another button was pressed...)
    //CHECK(!mouseIsDown, "I thought the mouse is up...");
    mouseIsDown=true;
    drawFocus=true;
  } else {
    if(!mouseIsDown){ dataLock.unlock(); return; } //the button is already up (another button was pressed...)
    //CHECK(mouseIsDown, "mouse-up event although the mouse is not down???");
    mouseIsDown=false;
    drawFocus=false;
  }
  //store where you've clicked
  s->downVec=vec;
  s->downRot=cam->X.rot;
  s->downPos=cam->X.pos;
  s->downFoc=cam->foc;

  //check object clicked on
  if(!downPressed) {
    if(reportSelects){
      auto sgl = singleGLAccess();
      Select(true);
//      singleGLAccess()->unlock();
    }
  }

  //mouse scroll wheel:
  if(mouse_button==4 && !downPressed) cam->X.pos += s->downRot*Vector_z * (.1 * (s->downPos-s->downFoc).length());
  if(mouse_button==5 && !downPressed) cam->X.pos -= s->downRot*Vector_z * (.1 * (s->downPos-s->downFoc).length());

  if(mouse_button==3) {  //selection
    {
      auto sgl = singleGLAccess();
      Select(true);
    }
//    singleGLAccess()->unlock();
    if(topSelection){
      cam->focus(topSelection->x, topSelection->y, topSelection->z);
//      uint name=topSelection->name;
//      cout <<"RIGHT CLICK call: id = 0x" <<std::hex <<topSelection->name <<endl;
    }
  }

  //step through all callbacks
  bool cont=true;
  if(!downPressed) {
    for(uint i=0; i<clickCalls.N; i++) cont=cont && clickCalls(i)->clickCallback(*this);
  }
  if(!cont) { postRedrawEvent(true); dataLock.unlock(); return; }

  postRedrawEvent(true);
  dataLock.unlock();
}

void OpenGL::MouseWheel(int wheel, int direction, int x, int y) {
  dataLock.writeLock();
  CALLBACK_DEBUG(printf("Window %d Mouse Wheel Callback:  %d %d %d %d\n", 0, wheel, direction, x, y));
  if(direction>0) camera.X.pos += camera.X.rot*Vector_z * (.1 * (camera.X.pos-camera.foc).length());
  else            camera.X.pos -= camera.X.rot*Vector_z * (.1 * (camera.X.pos-camera.foc).length());
  postRedrawEvent(true);
  dataLock.unlock();
}


void OpenGL::Motion(int _x, int _y) {
#ifdef MLR_GL
  dataLock.writeLock();
  int w=width, h=height;
  _y = h-_y;
  CALLBACK_DEBUG(printf("Window %d Mouse Motion Callback:  %d %d\n", 0, _x, _y));
  mouseposx=_x; mouseposy=_y;
  mlr::Camera *cam;
  mlr::Vector vec;
  if(mouseView==-1) {
    cam=&camera;
    getSphereVector(vec, _x, _y, 0, w, 0, h);
  } else {
    cam=&views(mouseView).camera;
    getSphereVector(vec, _x, _y, views(mouseView).le*w, views(mouseView).ri*w, views(mouseView).bo*h, views(mouseView).to*h);
  }
  CALLBACK_DEBUG(cout <<"associated to view " <<mouseView <<" x=" <<vec.x <<" y=" <<vec.y <<endl);
  lastEvent.set(mouse_button, -1, _x, _y, vec.x-s->downVec.x, vec.y-s->downVec.y);
  if(!mouseIsDown) {  //passive motion -> hover callbacks
    mouseposx=_x; mouseposy=_y;
    bool ud=false;
    for(uint i=0; i<hoverCalls.N; i++) ud=ud || hoverCalls(i)->hoverCallback(*this);
    if(ud) postRedrawEvent(true);
    dataLock.unlock();
    return;
  }
  if(mouse_button==1) {  //rotation // && !(modifiers&GLUT_ACTIVE_SHIFT) && !(modifiers&GLUT_ACTIVE_CTRL)){
    mlr::Quaternion rot;
    if(s->downVec.z<.1) {
      rot.setDiff(vec, s->downVec);  //consider imagined sphere rotation of mouse-move
    } else {
      rot.setVec((vec-s->downVec) ^ Vector_z); //consider only xy-mouse-move
    }
#if 0 //rotate about origin
    cam->X.rot = s->downRot * rot;   //rotate camera's direction
    rot = s->downRot * rot / s->downRot; //interpret rotation relative to current viewing
    cam->X.pos = rot * s->downPos;   //rotate camera's position
#else //rotate about focus
    cam->X.rot = s->downRot * rot;   //rotate camera's direction
    rot = s->downRot * rot / s->downRot; //interpret rotation relative to current viewing
    cam->X.pos = s->downFoc + rot * (s->downPos - s->downFoc);   //rotate camera's position
    //cam->focus();
#endif
    postRedrawEvent(true);
    if(immediateExitLoop) watching.setStatus(0);
  }
  if(mouse_button==3) {  //translation || (mouse_button==1 && (modifiers&GLUT_ACTIVE_SHIFT) && !(modifiers&GLUT_ACTIVE_CTRL))){
    /*    mlr::Vector trans = s->downVec - vec;
        trans.z=0.;
        trans = s->downRot*trans;
        cam->X.pos = s->downPos + trans;
        postRedrawEvent(true);*/
  }
  if(mouse_button==2) {  //zooming || (mouse_button==1 && !(modifiers&GLUT_ACTIVE_SHIFT) && (modifiers&GLUT_ACTIVE_CTRL))){
    double dy = s->downVec.y - vec.y;
    if(dy<-.99) dy = -.99;
    cam->X.pos = s->downPos + s->downRot*Vector_z * dy * s->downPos.length();
    postRedrawEvent(true);
  }
  dataLock.unlock();
#else
  NICO
#endif
}


//===========================================================================
//
// offscreen/background rendering
//

struct XBackgroundContext{
#ifdef MLR_GL
  typedef Bool (*glXMakeContextCurrentARBProc)(Display*, GLXDrawable, GLXDrawable, GLXContext);
  typedef GLXContext (*glXCreateContextAttribsARBProc)(Display*, GLXFBConfig, GLXContext, Bool, const int*);

  glXCreateContextAttribsARBProc glXCreateContextAttribsARB;
  glXMakeContextCurrentARBProc glXMakeContextCurrentARB;
  Display* dpy;
  int fbcount;
  GLXFBConfig* fbc;
  GLXContext ctx;
  GLXPbuffer pbuf;

  XBackgroundContext()
    : glXCreateContextAttribsARB(0), glXMakeContextCurrentARB(0){
    static int visual_attribs[] = { None };
    int context_attribs[] = { GLX_CONTEXT_MAJOR_VERSION_ARB, 3, GLX_CONTEXT_MINOR_VERSION_ARB, 0, None };

    dpy = NULL; //XOpenDisplay(0);
    fbcount = 0;
    fbc = NULL;

    /* open display */
    if(!(dpy = XOpenDisplay(0))) HALT("Failed to open display");

    /* get framebuffer configs, any is usable (might want to add proper attribs) */
    if(!(fbc = glXChooseFBConfig(dpy, DefaultScreen(dpy), visual_attribs, &fbcount)))
      HALT("Failed to get FBConfig");

    /* get the required extensions */
    glXCreateContextAttribsARB = (glXCreateContextAttribsARBProc)glXGetProcAddressARB( (const GLubyte *) "glXCreateContextAttribsARB");
    glXMakeContextCurrentARB = (glXMakeContextCurrentARBProc)glXGetProcAddressARB( (const GLubyte *) "glXMakeContextCurrent");
    if ( !(glXCreateContextAttribsARB && glXMakeContextCurrentARB) ){
      XFree(fbc);
      HALT("missing support for GLX_ARB_create_context");
    }

    /* create a context using glXCreateContextAttribsARB */
    if ( !( ctx = glXCreateContextAttribsARB(dpy, fbc[0], 0, True, context_attribs)) ){
      XFree(fbc);
      HALT("Failed to create opengl context");
    }

    /* create temporary pbuffer */
    int pbuffer_attribs[] = { GLX_PBUFFER_WIDTH, 800, GLX_PBUFFER_HEIGHT, 600, None };
    pbuf = glXCreatePbuffer(dpy, fbc[0], pbuffer_attribs);

    XFree(fbc);
    XSync(dpy, False);

    makeCurrent();
  }

  ~XBackgroundContext(){
    XFree(ctx);
    XFree(dpy);
  }

  void makeCurrent(){
    /* try to make it the current context */
    if ( !glXMakeContextCurrent(dpy, pbuf, pbuf, ctx) ){
      /* some drivers do not support context without default framebuffer, so fallback on
             * using the default window.
             */
      if ( !glXMakeContextCurrent(dpy, DefaultRootWindow(dpy), DefaultRootWindow(dpy), ctx) ){
        fprintf(stderr, "failed to make current");
        exit(1);
      }
    }
  }
#endif
};

Singleton<XBackgroundContext> xBackgroundContext;

void OpenGL::renderInBack(bool _captureImg, bool _captureDep, int w, int h){
#ifdef MLR_GL
  if(w<0) w=width;
  if(h<0) h=height;

  auto mut=singleGLAccess();
  dataLock.readLock();
  xBackgroundContext()->makeCurrent();

  CHECK_EQ(w%4,0,"should be devidable by 4!!");

  isUpdating.waitForStatusEq(0);
  isUpdating.setStatus(1);

  s->beginGlContext();

  if(!rboColor || !rboDepth){ //need to initialize
    glewInit();
    glGenRenderbuffers(1, &rboColor);  // Create a new renderbuffer unique name.
    glBindRenderbuffer(GL_RENDERBUFFER, rboColor);  // Set it as the current.
    glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, w, h); // Sets storage type for currently bound renderbuffer.
    // Depth renderbuffer.
    glGenRenderbuffers(1, &rboDepth);
    glBindRenderbuffer(GL_RENDERBUFFER, rboDepth);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, w, h);
    // Framebuffer.
    // Create a framebuffer and a renderbuffer object.
    // You need to delete them when program exits.
    glGenFramebuffers(1, &fboId);
    glBindFramebuffer(GL_FRAMEBUFFER, fboId);
    //from now on, operate on the given framebuffer
    //GL_FRAMEBUFFER        read write
    //GL_READ_FRAMEBUFFER   read
    //GL_FRAMEBUFFER        write

    // Adds color renderbuffer to currently bound framebuffer.
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, rboColor);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,  GL_RENDERBUFFER, rboDepth);

    glReadBuffer(GL_COLOR_ATTACHMENT0);
    //glReadBuffer(GL_BACK);

    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if (status != GL_FRAMEBUFFER_COMPLETE) {
      cout << "framebuffer error:" << endl;
      switch (status) {
        case GL_FRAMEBUFFER_UNDEFINED: {
          cout << "GL_FRAMEBUFFER_UNDEFINED" << endl;
          break;
        }
        case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT: {
          cout << "GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT" << endl;
          break;
        }
        case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT: {
          cout << "GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT" << endl;
          break;
        }
        case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER: {
          cout << "GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER" << endl;
          break;
        }
        case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER: {
          cout << "GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER" << endl;
          break;
        }
        case GL_FRAMEBUFFER_UNSUPPORTED: {
          cout << "GL_FRAMEBUFFER_UNSUPPORTED" << endl;
          break;
        }
        case GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE: {
          cout << "GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE" << endl;
          break;
        }
        case GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS: {
          cout << "GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS" << endl;
          break;
        }
        case 0: {
          cout << "0" << endl;
          break;
        }
      }
      exit(EXIT_FAILURE);
    }
  }

  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fboId);

  //-- draw!
  Draw(w, h, NULL, true);
  glFlush();

  //-- read
  if(_captureImg){
    captureImage.resize(h, w, 3);
    //  glReadBuffer(GL_BACK);
    glReadBuffer(GL_COLOR_ATTACHMENT0);
    glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, captureImage.p);
//    flip_image(captureImage);
  }
  if(_captureDep){
    captureDepth.resize(h, w);
    glReadBuffer(GL_DEPTH_ATTACHMENT);
    glReadPixels(0, 0, w, h, GL_DEPTH_COMPONENT, GL_FLOAT, captureDepth.p);
//    flip_image(captureDepth);
  }

  // Return to onscreen rendering:
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);

  isUpdating.setStatus(0);

  dataLock.unlock();
//  singleGLAccess()->unlock();
#endif
}

//===========================================================================
//
// GUI implementation
//

void glUI::addButton(uint x, uint y, const char *name, const char *img1, const char *img2) {
  Button &b = buttons.append();
  byteA img;
  b.hover=false;
  b.x=x; b.y=y; b.name=name;
  //read_png(img, tex1);
  if(img1) {
    read_ppm(img, img1, true);
  } else {
    img.resize(18, strlen(name)*9+10, 3);
    img=255;
  }
  b.w=img.d1; b.h=img.d0;
  b.img1=img;    add_alpha_channel(b.img1, 100);
  if(img2) {
    read_ppm(img, img1, true);
    CHECK(img.d1==b.w && img.d0==b.h, "mismatched size");
  }
  b.img2=img;    add_alpha_channel(b.img2, 200);
}

void glUI::glDraw() {
#ifdef MLR_GL
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  GLint viewPort[4];
  glGetIntegerv(GL_VIEWPORT, viewPort);
  glOrtho(0., viewPort[2], viewPort[3], .0, -1., 1.);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  Button *b;
  float x, y, h;
  for(uint i=0; i<buttons.N; i++) {
    b = &buttons(i);
    x=b->x-b->w/2.;
    y=b->y-b->h/2.;
    //w=b->w;
    h=b->h;
    glColor(0, 0, 0, 1);
    glDrawText(b->name, x+5, y+h-5, 0.);
    if((int)i==top) glRasterImage((int)x, (int)y, b->img2);
    else      glRasterImage((int)x, (int)y, b->img1);
  }
#else
  NICO
#endif
}

bool glUI::checkMouse(int _x, int _y) {
  float x, y, w, h;
  Button *b;
  int otop=top;
  top=-1;
  for(uint i=0; i<buttons.N; i++) {
    b = &buttons(i);
    x=b->x-b->w/2.;
    y=b->y-b->h/2.;
    w=b->w;
    h=b->h;
    if(_x>=x && _x <=x+w && _y>=y && _y<=y+h) top = i;
  }
  if(otop==top) return false;
  //postRedrawEvent(false);
  return true;
}

#ifdef MLR_QTGL
#if   defined MLR_MSVC
#  include"opengl_MSVC.moccpp"
#elif defined MLR_SunOS
#  include"opengl_SunOS.moccpp"
#elif defined MLR_Linux
#  include"opengl_qt_moc.cxx"
#elif defined MLR_Cygwin
#  include"opengl_Cygwin.moccpp"
#endif
#endif


/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */

#include <Core/util.tpp>
#include <Core/array.tpp>
#include <Core/thread.h>
#include "gtk.h"
#include <sys/syscall.h>

#ifdef MLR_GTK

#include <gtk/gtk.h>
#include <gtk/gtkgl.h>
#include <GL/glut.h>
#include <X11/Xlib.h>

struct GtkThread:Thread {
  GtkThread():Thread("GTK thread", .0) {
    int argc=1;
    char **argv = new char*[1];
    argv[0] = (char*)"x.exe";

    XInitThreads();
//      g_thread_init(NULL);
    gdk_threads_init();
    gdk_threads_enter();
    gtk_init(&argc, &argv);
    gtk_gl_init(&argc, &argv);

    threadLoop();
  }
  ~GtkThread(){
    gtk_main_quit();
    threadClose();
    gdk_threads_leave();
//    g_main_context_unref(g_main_context_default ());
    cout <<"STOPPING GTK" <<endl;
  }

  virtual void open() {}
  virtual void step() { gtk_main(); }
  virtual void close() {  }
};

Singleton<GtkThread> global_gtkThread;
Mutex callbackMutex;

void gtkEnterCallback() {  callbackMutex.lock();  }
void gtkLeaveCallback() {  callbackMutex.unlock();  }

void gtkLock(bool checkInitialized) {
  if(checkInitialized) gtkCheckInitialized();
  if(callbackMutex.state && callbackMutex.state==syscall(SYS_gettid)) return;
  gdk_threads_enter();
}

void gtkUnlock() {
  if(callbackMutex.state && callbackMutex.state==syscall(SYS_gettid)) return;
  gdk_threads_leave();
}


void gtkCheckInitialized() {
#if 0
  static bool isInitialized=false;
  if(!isInitialized) {
    isInitialized=true;
    int argc=1;
    char **argv = new char*[1];
    argv[0] = (char*)"x.exe";
    
    XInitThreads();
    g_thread_init(NULL); 1
    gdk_threads_init();
//    gdk_threads_enter();
    gtk_init(&argc, &argv);
    gtk_gl_init(&argc, &argv);
    //glutInit(&argc, argv);
  }
#else
  global_gtkThread();
#endif
}

void gtkProcessEvents(){
  gtkLock();
  while (gtk_events_pending())  gtk_main_iteration();
  gtkUnlock();
}

Signaler menuChoice(-1);
static void menuitem_response(int choice) { menuChoice.setStatus(choice); }

int gtkPopupMenuChoice(StringL& choices) {
  //create menu
  GtkWidget *menu = gtk_menu_new();
  gtk_menu_popup(GTK_MENU(menu), NULL, NULL, NULL, NULL, 0, gtk_get_current_event_time());
  for_list(mlr::String,  s,  choices) {
    GtkWidget *item = gtk_menu_item_new_with_label(s->p);
    gtk_container_add(GTK_CONTAINER(menu), item);
    gtk_signal_connect_object(GTK_OBJECT(item), "activate",
                              GTK_SIGNAL_FUNC(menuitem_response), (gpointer)(ulong)s_COUNT);
  }
  menuChoice.setStatus(-1);
  gtk_widget_show_all(menu);
  gtk_menu_shell_select_first(GTK_MENU_SHELL(menu), false);
  menuChoice.waitForStatusNotEq(-1);
  gtk_widget_destroy(menu);
  int choice = menuChoice.getStatus();
  return choice>=0?choice:0;
}

GtkWidget *gtkTopWindow(const char* name) {
  gtkCheckInitialized();
  gtkLock();
  GtkWidget *win = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title(GTK_WINDOW(win), name);
  gtk_window_set_default_size(GTK_WINDOW(win), 300, 300);
  //gtk_container_set_reallocate_redraws(GTK_CONTAINER(container), TRUE);
  gtkUnlock();
  return win;
}

#else //MLR_GTK
#endif

#include "GlobalIterativeNewton.h"

bool useNewton=true;

GlobalIterativeNewton::GlobalIterativeNewton(const ScalarFunction& f, const arr& bounds_lo, const arr& bounds_hi, OptOptions opt)
  : newton(x, f, opt),
    grad(x, f, opt),
    bounds_lo(bounds_lo), bounds_hi(bounds_hi),
    best(NULL) {
  newton.bound_lo = bounds_lo;
  newton.bound_hi = bounds_hi;
  newton.o.verbose = 0;
}

GlobalIterativeNewton::~GlobalIterativeNewton(){
}

void addRun(GlobalIterativeNewton& gin, const arr& x, double fx, double tol){
  GlobalIterativeNewton::LocalMinimum *found=NULL;
  for(GlobalIterativeNewton::LocalMinimum& m:gin.localMinima){
    double d = euclideanDistance(x, m.x);
    if(euclideanDistance(x,m.x)<tol){
      if(!found) found = &m;
      else if(d<euclideanDistance(x,found->x)) found = &m;
    }
  }

  if(found){
    found->hits++;
    if(fx<found->fx){
      found->x = x;
      found->fx = fx;
    }
  }else{
    gin.localMinima.append( {x, fx, 1} );
    found = &gin.localMinima.last();
    gin.best = NULL;
  }

  if(!gin.best){
    gin.best = &gin.localMinima.first();
    for(GlobalIterativeNewton::LocalMinimum& m:gin.localMinima) if(m.fx < gin.best->fx) gin.best = &m;
  }
  if(found->fx<gin.best->fx) gin.best=found;
  gin.newton.x = gin.best->x;
  gin.newton.fx = gin.best->fx;
  if(gin.newton.o.verbose>1) cout <<"***** optGlobalIterativeNewton: local minimum: " <<found->hits <<' ' <<found->fx <<' ' <<found->x <<endl;
}

void addRunFrom(GlobalIterativeNewton& gin, const arr& x){
  if(useNewton){
    gin.newton.reinit(x);
    gin.newton.run();
    addRun(gin, gin.newton.x, gin.newton.fx, 3.*gin.newton.o.stopTolerance);
  }else{
    gin.grad.reinit(x);
    gin.grad.run();
    addRun(gin, gin.grad.x, gin.grad.fx, 3.*gin.grad.o.stopTolerance);
  }
}

void GlobalIterativeNewton::step(){
  arr x = bounds_lo + (bounds_hi-bounds_lo) % rand(bounds_lo.N);
  if(newton.o.verbose>1) cout <<"***** optGlobalIterativeNewton: new iteration from x=" <<x <<endl;
  addRunFrom(*this, x);
}

void GlobalIterativeNewton::run(uint maxIt){
  for(uint i=0;i<maxIt;i++){
    step();
  }
}

void GlobalIterativeNewton::report(){
  cout <<"# local minima = " <<localMinima.N <<endl;
  uint i=0;
  for(LocalMinimum& m:localMinima){
    cout <<i++ <<' ' <<m.hits <<' ' <<m.fx <<" \t" <<m.x <<endl;
  }
}

void GlobalIterativeNewton::reOptimizeAllPoints(){
  if(!localMinima.N) return;
  arr X;
  for(LocalMinimum& m:localMinima) X.append(m.x);
  X.reshape(localMinima.N, X.N/localMinima.N);
  localMinima.clear();
  for(uint i=0;i<X.d0;i++) addRunFrom(*this, X[i]);
}
/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */

#include "lagrangian.h"
#include "newton.h"

//==============================================================================
//
// LagrangianProblem
//

double I_lambda_x(uint i, arr& lambda, arr& g){
  if(g(i)>0. || (lambda.N && lambda(i)>0.)) return 1.;
  return 0.;
}

//==============================================================================

LagrangianProblem::LagrangianProblem(ConstrainedProblem& P, OptOptions opt, arr& lambdaInit)
  : P(P), muLB(0.), mu(0.), nu(0.) {
  ScalarFunction::operator=( [this](arr& dL, arr& HL, const arr& x) -> double {
    return this->lagrangian(dL, HL, x);
  } );

  //switch on penalty terms
  nu=opt.muInit;
  switch(opt.constrainedMethod){
    case squaredPenalty: mu=opt.muInit;  break;
    case augmentedLag:   mu=opt.muInit;  break;
    case anyTimeAula:    mu=opt.muInit;  break;
    case logBarrier:     muLB=opt.muLBInit;  break;
    case squaredPenaltyFixed: mu=opt.muInit;  break;
    case noMethod: HALT("need to set method before");  break;
  }

  if(&lambdaInit) lambda = lambdaInit;
}

double LagrangianProblem::lagrangian(arr& dL, arr& HL, const arr& _x){
  //-- evaluate constrained problem and buffer
  if(_x!=x){
    x=_x;
    P.phi(phi_x, J_x, H_x, tt_x, x);
  }else{ //we evaluated this before - use buffered values; the meta F is still recomputed as (dual) parameters might have changed
  }
  CHECK_EQ(phi_x.N, J_x.d0, "Jacobian size inconsistent");
  CHECK_EQ(phi_x.N, tt_x.N, "termType array size inconsistent");

  //-- construct unconstrained problem
  //precompute I_lambda_x
  boolA I_lambda_x(phi_x.N);
  if(phi_x.N) I_lambda_x = false;
  if(mu)       for(uint i=0;i<phi_x.N;i++) if(tt_x.p[i]==OT_ineq) I_lambda_x.p[i] = (phi_x.p[i]>0. || (lambda.N && lambda.p[i]>0.));

  double L=0.; //L value
  for(uint i=0;i<phi_x.N;i++){
    if(            tt_x.p[i]==OT_f                    ) L += phi_x.p[i];                // direct cost term
    if(            tt_x.p[i]==OT_sumOfSqr             ) L += mlr::sqr(phi_x.p[i]);       // sumOfSqr term
    if(muLB     && tt_x.p[i]==OT_ineq                 ){ if(phi_x.p[i]>0.) return NAN;  L -= muLB * ::log(-phi_x.p[i]); } //log barrier, check feasibility
    if(mu       && tt_x.p[i]==OT_ineq && I_lambda_x.p[i]) L += mu * mlr::sqr(phi_x.p[i]);  //g-penalty
    if(lambda.N && tt_x.p[i]==OT_ineq && lambda.p[i]>0. ) L += lambda.p[i] * phi_x.p[i];    //g-lagrange terms
    if(nu       && tt_x.p[i]==OT_eq                   ) L += nu * mlr::sqr(phi_x.p[i]);  //h-penalty
    if(lambda.N && tt_x.p[i]==OT_eq                   ) L += lambda.p[i] * phi_x.p[i];    //h-lagrange terms
  }

  if(&dL){ //L gradient
    arr coeff=zeros(phi_x.N);
    for(uint i=0;i<phi_x.N;i++){
      if(            tt_x.p[i]==OT_f                    ) coeff.p[i] += 1.;              // direct cost term
      if(            tt_x.p[i]==OT_sumOfSqr             ) coeff.p[i] += 2.* phi_x.p[i];    // sumOfSqr terms
      if(muLB     && tt_x.p[i]==OT_ineq                 ) coeff.p[i] -= (muLB/phi_x.p[i]); //log barrier, check feasibility
      if(mu       && tt_x.p[i]==OT_ineq && I_lambda_x.p[i]) coeff.p[i] += 2.*mu*phi_x.p[i];  //g-penalty
      if(lambda.N && tt_x.p[i]==OT_ineq && lambda.p[i]>0. ) coeff.p[i] += lambda.p[i];       //g-lagrange terms
      if(nu       && tt_x.p[i]==OT_eq                   ) coeff.p[i] += 2.*nu*phi_x.p[i];  //h-penalty
      if(lambda.N && tt_x.p[i]==OT_eq                   ) coeff.p[i] += lambda.p[i];       //h-lagrange terms
    }
    dL = comp_At_x(J_x, coeff);
    dL.reshape(x.N);
  }

  if(&HL){ //L hessian: Most terms are of the form   "J^T  diag(coeffs)  J"
    arr coeff=zeros(phi_x.N);
    int fterm=-1;
    for(uint i=0;i<phi_x.N;i++){
      if(            tt_x.p[i]==OT_f){ if(fterm!=-1) HALT("There must only be 1 f-term (in the current implementation)");  fterm=i; }
      if(            tt_x.p[i]==OT_sumOfSqr             ) coeff.p[i] += 2.;      // sumOfSqr terms
      if(muLB     && tt_x.p[i]==OT_ineq                 ) coeff.p[i] += (muLB/mlr::sqr(phi_x.p[i]));  //log barrier, check feasibility
      if(mu       && tt_x.p[i]==OT_ineq && I_lambda_x.p[i]) coeff.p[i] += 2.*mu;   //g-penalty
      if(nu       && tt_x.p[i]==OT_eq                   ) coeff.p[i] += 2.*nu;   //h-penalty
    }
    arr tmp = J_x;
    for(uint i=0;i<phi_x.N;i++) tmp[i]() *= sqrt(coeff.p[i]);
#if 1
    HL = comp_At_A(tmp); //Gauss-Newton type!
#else
    arr tmpt = comp_At(tmp);
    HL = comp_A_At(tmpt); //Gauss-Newton type!
#endif

    if(fterm!=-1){ //For f-terms, the Hessian must be given explicitly, and is not \propto J^T J
      HL += H_x;
    }

    if(!HL.special) HL.reshape(x.N,x.N);
  }

  return L;
}

double LagrangianProblem::get_costs(){
  double S=0.;
  for(uint i=0;i<phi_x.N;i++){
    if(tt_x(i)==OT_f) S += phi_x(i);
    if(tt_x(i)==OT_sumOfSqr) S += mlr::sqr(phi_x(i));
  }
  return S;
}

double LagrangianProblem::get_sumOfGviolations(){
  double S=0.;
  for(uint i=0;i<phi_x.N;i++){
    if(tt_x(i)==OT_ineq && phi_x(i)>0.) S += phi_x(i);
  }
  return S;
}

double LagrangianProblem::get_sumOfHviolations(){
  double S=0.;
  for(uint i=0;i<phi_x.N;i++){
    if(tt_x(i)==OT_eq) S += fabs(phi_x(i));
  }
  return S;
}

uint LagrangianProblem::get_dimOfType(const ObjectiveType& tt){
  uint d=0;
  for(uint i=0;i<tt_x.N;i++) if(tt_x(i)==tt) d++;
  return d;
}

void LagrangianProblem::aulaUpdate(bool anyTimeVariant, double lambdaStepsize, double muInc, double *L_x, arr& dL_x, arr& HL_x){
  if(!lambda.N) lambda=zeros(phi_x.N);

  //-- lambda update
  for(uint i=0;i<lambda.N;i++){
    if(tt_x(i)==OT_eq  )  lambda(i) += (lambdaStepsize * 2.*nu)*phi_x(i);
    if(tt_x(i)==OT_ineq)  lambda(i) += (lambdaStepsize * 2.*mu)*phi_x(i);
    if(tt_x(i)==OT_ineq && lambda(i)<0.) lambda(i)=0.;  //bound clipping
  }

  if(anyTimeVariant){
    //collect gradients of active constraints
    arr A;
    RowShifted *Aaux=NULL, *Jaux=NULL;
    if(isRowShifted(J_x)){
      Aaux = makeRowShifted(A, 0, J_x.d1, x.N);
      Jaux = castRowShifted(J_x);
    }
    //append rows of J_x to A if constraint is active
    for(uint i=0;i<lambda.N;i++){
      if( (tt_x(i)==OT_eq) ||
          (tt_x(i)==OT_ineq && (phi_x(i)>0. || lambda(i)>0.)) ){
        A.append(J_x[i]);
        A.reshape(A.N/J_x.d1,J_x.d1);
        if(isRowShifted(J_x))
          Aaux->rowShift.append(Jaux->rowShift(i));
      }
    }
    if(A.d0>0){
      arr tmp = comp_A_At(A);
      addDiag(tmp, 1e-6);
      //    if(isRowShifted(J_x)){
      //      CHECK_EQ(castRowShifted(tmp)->symmetric, true, "");
      //      for(uint i=0;i<tmp.d0;i++) tmp(i,0) += 1e-6;
      //    }else{
      //      for(uint i=0;i<tmp.d0;i++) tmp(i,i) += 1e-6;
      //    }

      arr AdL = comp_A_x(A, dL_x);
      arr beta;
      bool worked=true;
      try {
        beta = lapack_Ainv_b_sym(tmp, AdL);
      }catch(...){
        arr sig, eig;
        lapack_EigenDecomp(tmp, sig, eig);
        cout <<endl <<"** hessian inversion failed AulaAnyTimeUpdate: " <<sig <<" -- revert to standard update" <<endl;
        worked=false;
      }

      if(worked){
        //reinsert zero rows
        for(uint i=0;i<lambda.N;i++){
          if(! ( (tt_x(i)==OT_eq) ||
                 (tt_x(i)==OT_ineq && (phi_x(i)>0. || lambda(i)>0.)) ) ){
            beta.insert(i,0.);
          }
        }
        lambda -= lambdaStepsize * beta;
        //bound clipping
        for(uint i=0;i<lambda.N;i++) if(lambda(i)<0.) lambda(i)=0.;
      }
    }
  }

  //-- adapt mu as well?
  if(muInc>1. && mu<1e6) mu *= muInc;
  if(muInc>1. && nu<1e6) nu *= muInc;

  //-- recompute the Lagrangian with the new parameters (its current value, gradient & hessian)
  if(L_x || &dL_x || &HL_x){
    double L = lagrangian(dL_x, HL_x, x); //reevaluate gradients and hessian (using buffered info)
    if(L_x) *L_x = L;
  }
}

//==============================================================================
//
// PhaseOneProblem
//


void PhaseOneProblem::phi(arr& meta_phi, arr& meta_J, arr& meta_H, ObjectiveTypeA& tt, const arr& x){
  NIY;
  arr g, Jg;
//  f_orig(NoArr, NoArr, g, (&meta_Jg?Jg:NoArr), x.sub(0,-2)); //the underlying problem only receives a x.N-1 dimensional x

  // meta_g.resize(g.N+1);
  // meta_g(0) = x.last();                                       //cost
  // for(uint i=0;i<g.N;i++) meta_g(i) = g(i)-x.last();  //slack constraints
  // meta_g.last() = -x.last();                                  //last constraint

  // if(&meta_Jg){
  //   meta_Jg.resize(meta_g.N, x.N);  meta_Jg.setZero();
  //   meta_Jg(0,x.N-1) = 1.; //cost
  //   for(uint i=0;i<g.N;i++) for(uint j=0;j<x.N-1;j++) meta_Jg(i,j) = Jg(i,j);
  //   for(uint i=0;i<g.N;i++) meta_Jg(i,x.N-1) = -1.;
  //   meta_Jg(g.N, x.N-1) = -1.;
  // }
}


//==============================================================================
//
// Solvers
//

const char* MethodName[]={ "NoMethod", "SquaredPenalty", "AugmentedLagrangian", "LogBarrier", "AnyTimeAugmentedLagrangian", "SquaredPenaltyFixed"};


//==============================================================================

OptConstrained::OptConstrained(arr& x, arr &dual, ConstrainedProblem& P, OptOptions opt)
  : UCP(P, opt, dual), newton(x, UCP, opt), dual(dual), opt(opt), its(0), earlyPhase(false){

  fil.open(STRING("z."<<MethodName[opt.constrainedMethod]));

  if(opt.verbose>0) cout <<"***** optConstrained: method=" <<MethodName[opt.constrainedMethod] <<endl;
}

bool OptConstrained::step(){
  fil <<its <<' ' <<newton.evals <<' ' <<UCP.get_costs() <<' ' <<UCP.get_sumOfGviolations() <<' ' <<UCP.get_sumOfHviolations() <<endl;

  if(opt.verbose>0){
    cout <<"** optConstr. it=" <<its
         <<(earlyPhase?'e':'l')
         <<" mu=" <<UCP.mu <<" nu=" <<UCP.nu <<" muLB=" <<UCP.muLB;
    if(newton.x.N<5) cout <<" \tlambda=" <<UCP.lambda;
    cout <<endl;
  }

  arr x_old = newton.x;

  //run newton on the Lagrangian problem
  if(opt.constrainedMethod==squaredPenaltyFixed){
    newton.run();
  }else{
    double stopTol = newton.o.stopTolerance;
    newton.o.stopTolerance *= (earlyPhase?10.:2.);
    if(opt.constrainedMethod==anyTimeAula)  newton.run(20);
    else                                    newton.run();
    newton.o.stopTolerance = stopTol;
  }

  if(opt.verbose>0){
    cout <<"** optConstr. it=" <<its
         <<(earlyPhase?'e':'l')
         <<' ' <<newton.evals
         <<" f(x)=" <<UCP.get_costs()
         <<" \tg_compl=" <<UCP.get_sumOfGviolations()
         <<" \th_compl=" <<UCP.get_sumOfHviolations()
         <<" \t|x-x'|=" <<absMax(x_old-newton.x);
    if(newton.x.N<5) cout <<" \tx=" <<newton.x;
    cout <<endl;
  }

  //check for squaredPenaltyFixed method
  if(opt.constrainedMethod==squaredPenaltyFixed){
    if(opt.verbose>0) cout <<"** optConstr. squaredPenaltyFixed stops after one outer iteration" <<endl;
    return true;
  }

  //check for no constraints
  if(UCP.get_dimOfType(OT_ineq) + UCP.get_dimOfType(OT_eq) == 0){
    if(opt.verbose>0) cout <<"** optConstr. NO CONSTRAINTS -> run Newton againg and stop" <<endl;
    newton.run();
    return true;
  }

  //stopping criteron
  if(its>=2 && absMax(x_old-newton.x) < (earlyPhase?5.:1.)*opt.stopTolerance){
    if(opt.verbose>0) cout <<"** optConstr. StoppingCriterion Delta<" <<opt.stopTolerance <<endl;
    if(earlyPhase) earlyPhase=false;
    else{
      if(opt.stopGTolerance<0.
         || UCP.get_sumOfGviolations() + UCP.get_sumOfHviolations() < opt.stopGTolerance)
        return true;
     }
  }

  //upate Lagrange parameters
  switch(opt.constrainedMethod){
    case squaredPenalty: UCP.mu *= 10.;  break;
    case augmentedLag:   UCP.aulaUpdate(false, 1., opt.aulaMuInc, &newton.fx, newton.gx, newton.Hx);  break;
    case anyTimeAula:    UCP.aulaUpdate(true,  1., opt.aulaMuInc, &newton.fx, newton.gx, newton.Hx);  break;
    case logBarrier:     UCP.muLB /= 2.;  break;
    case squaredPenaltyFixed: HALT("you should not be here"); break;
    case noMethod: HALT("need to set method before");  break;
  }

  if(&dual) dual=UCP.lambda;

  its++;

  return false;
}

uint OptConstrained::run(){
//  earlyPhase=true;
  while(!step());
  return newton.evals;
}

OptConstrained::~OptConstrained(){
}


/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#include "KOMO_Problem.h"
#include "Graph_Problem.h"

bool KOMO_Problem::checkStructure(const arr& x){
  arr y;
  arrA J, H;
  ObjectiveTypeA tt, featureTypes;
//  uint T=get_T();
  uint k=get_k();
  uintA variableDimensions, featureTimes;
  getStructure(variableDimensions, featureTimes, featureTypes);
  uintA varDimIntegral = integral(variableDimensions);

  phi(y, J, H, tt, x);

  CHECK_EQ(tt, featureTypes,"");
  CHECK_EQ(sum(variableDimensions), x.N, "variable dimensions don't match");
  uint m=y.N;
  CHECK_EQ(featureTimes.N, m, "");
  CHECK_EQ(J.N, m, "");
  CHECK_EQ(tt.N, m, "");

  for(uint i=0;i<m;i++){
    uint t=featureTimes(i);
    uint d=varDimIntegral(t) - (t>k? varDimIntegral(t-k-1) : 0);
    CHECK_EQ(J(i).N, d, i<<"th Jacobian has wrong dim");
  }
  return true;
}

void KOMO_Problem::report(const arr& phi){
  ObjectiveTypeA featureTypes;
  uint k=get_k();
  uintA variableDimensions, featureTimes;
  getStructure(variableDimensions, featureTimes, featureTypes);

  cout <<"KOMO Problem report:  k=" <<k <<"  Features:" <<endl;
  for(uint i=0;i<featureTimes.N;i++){
    cout <<i <<" t=" <<featureTimes(i) <<" vardim=" <<variableDimensions(featureTimes(i)) <<" type=" <<featureTypes(i);
    if(&phi) cout <<" phi=" <<phi(i) <<" phi^2=" <<mlr::sqr(phi(i));
    cout <<endl;
  }
}


void KOMO_GraphProblem::getStructure(uintA& variableDimensions, uintAA& featureVariables, ObjectiveTypeA& featureTypes){
  uintA featureTimes;
  KOMO.getStructure(variableDimensions, featureTimes, featureTypes);

  uint k=KOMO.get_k();
  uint m=featureTypes.N;
  CHECK_EQ(featureTimes.N, m, "");

  //-- tuples (x_{t-k:t})
  featureVariables.resize(m);
  for(uint i=0;i<m;i++){
    int first = featureTimes(i) - k;
    if(first<0) first=0;
    featureVariables(i).clear();
    for(uint j=first;j<=featureTimes(i);j++) featureVariables(i).append(j);
  }
}

void KOMO_GraphProblem::phi(arr& phi, arrA& J, arrA& H, const arr& x){
  ObjectiveTypeA featureTypes; //TODO: redundant -> remove
  KOMO.phi(phi, J, H, featureTypes, x);
}


Conv_KOMO_ConstrainedProblem::Conv_KOMO_ConstrainedProblem(KOMO_Problem& P) : KOMO(P){
  KOMO.getStructure(variableDimensions, featureTimes, featureTypes);
  varDimIntegral = integral(variableDimensions);
}

void Conv_KOMO_ConstrainedProblem::phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& tt, const arr& x){
  KOMO.phi(phi, (&J?J_KOMO:NoArrA), (&H?H_KOMO:NoArrA), tt, x);

  //-- construct a row-shifed J from the array of featureJs
  if(&J){
    uint k=KOMO.get_k();
    uint dim_xmax = max(variableDimensions);
    RowShifted *Jaux = makeRowShifted(J, phi.N, (k+1)*dim_xmax, x.N);
    J.setZero();

    //loop over features
    for(uint i=0; i<phi.N; i++) {
      arr& Ji = J_KOMO(i);
      CHECK(Ji.N<=J.d1,"");
      //        J({i, 0, J_KOMO(i}).N-1) = J_KOMO(i);
      memmove(&J(i,0), Ji.p, Ji.sizeT*Ji.N);
      uint t=featureTimes(i);
      if(t<=k) Jaux->rowShift(i) = 0;
      else Jaux->rowShift(i) =  varDimIntegral(t-k-1);
    }

    Jaux->reshift();
    Jaux->computeColPatches(true);
  }

  if(&H){
    bool hasFterm = false;
    if(&tt) hasFterm = (tt.findValue(OT_f) != -1);
    if(hasFterm){
      CHECK(H_KOMO.N, "this problem has f-terms -- I need a Hessian!");
      NIY
    }else{
      H.clear();
    }
  }
}
/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */

#include <iomanip>

#include "newton.h"

bool sanityCheck=false; //true;

/** @brief Minimizes \f$f(x) = A(x)^T x A^T(x) - 2 a(x)^T x + c(x)\f$. The optional _user arguments specify,
 * if f has already been evaluated at x (another initial evaluation is then omitted
 * to increase performance) and the evaluation of the returned x is also returned */
int optNewton(arr& x, const ScalarFunction& f,  OptOptions o) {
  return OptNewton(x, f, o).run();
}

//===========================================================================

OptNewton::OptNewton(arr& _x, const ScalarFunction& _f,  OptOptions _o):
  x(_x), f(_f), o(_o), it(0), evals(0), numTinySteps(0){
  alpha = o.initStep;
  beta = o.damping;
  additionalRegularizer=NULL;
  if(f) reinit(_x);
}

void OptNewton::reinit(const arr& _x){
  if(&x!=&_x) x = _x;
  fx = f(gx, Hx, x);  evals++;
  if(additionalRegularizer)  fx += scalarProduct(x,(*additionalRegularizer)*vectorShaped(x));

  //startup verbose
  if(o.verbose>1) cout <<"*** optNewton: starting point f(x)=" <<fx <<" alpha=" <<alpha <<" beta=" <<beta <<endl;
  if(o.verbose>2) cout <<"\nx=" <<x <<endl;
  if(o.verbose>0) fil.open("z.opt");
  if(o.verbose>0) fil <<0 <<' ' <<eval_cost <<' ' <<fx <<' ' <<alpha;
  if(o.verbose>2) fil <<' ' <<x;
  if(o.verbose>0) fil <<endl;
}

//===========================================================================

OptNewton::StopCriterion OptNewton::step(){
  double fy;
  arr y, gy, Hy, Delta;
  bool betaChanged=false;

  it++;
  if(o.verbose>1) cout <<"optNewton it=" <<std::setw(4) <<it << " \tbeta=" <<std::setw(8) <<beta <<flush;

  if(!(fx==fx)) HALT("you're calling a newton step with initial function value = NAN");

  //compute Delta
  arr R=Hx;
  if(beta) { //Levenberg Marquardt damping
    if(isRowShifted(R)) for(uint i=0; i<R.d0; i++) R(i,0) += beta; //(R(i,0) is the diagonal in the packed matrix!!)
    else for(uint i=0; i<R.d0; i++) R(i,i) += beta;
  }
  if(additionalRegularizer) { //obsolete -> retire
    if(isRowShifted(R)) R = unpack(R);
    Delta = lapack_Ainv_b_sym(R + (*additionalRegularizer), -(gx+(*additionalRegularizer)*vectorShaped(x)));
  } else {
    bool inversionFailed=false;
    try {
      Delta = lapack_Ainv_b_sym(R, -gx);
    }catch(...){
      inversionFailed=true;
    }
    if(inversionFailed){
      if(false){ //increase beta
        arr sig = lapack_kSmallestEigenValues_sym(R, 3);
        if(o.verbose>0){
          cout <<"** hessian inversion failed ... increasing damping **\neigenvalues=" <<sig <<endl;
        }
        double sigmin = sig.min();
        if(sigmin>0.) THROW("Hessian inversion failed, but eigenvalues are positive???");
        beta = 2.*beta - sigmin;
        betaChanged=true;
        return stopCriterion=stopNone;
      }else{ //use gradient
        if(o.verbose>0){
          cout <<"** hessian inversion failed ... using gradient descent direction" <<endl;
        }
        Delta = -gx / length(gx) * o.maxStep;
//        alpha = 1.;
      }
    }
  }

  //restrict stepsize
  double maxDelta = absMax(Delta);
  if(o.maxStep>0. && maxDelta>o.maxStep){  Delta *= o.maxStep/maxDelta; maxDelta = o.maxStep; }
  double alphaLimit = o.maxStep/maxDelta;

  //...due to bounds
  if(bound_lo.N && bound_hi.N){
    double a=1.;
    for(uint i=0;i<x.N;i++){
      if(x(i)+a*Delta(i)>bound_hi(i)) a = (bound_hi(i)-x(i))/Delta(i);
      if(x(i)+a*Delta(i)<bound_lo(i)) a = (bound_lo(i)-x(i))/Delta(i);
    }
    Delta *= a;
  }
  if(o.verbose>1) cout <<" \t|Delta|=" <<std::setw(11) <<maxDelta <<flush;

  //lazy stopping criterion: stop without any update
  if(absMax(Delta)<1e-1*o.stopTolerance){
    if(o.verbose>1) cout <<" \t - NO UPDATE" <<endl;
    return stopCriterion=stopCrit1;
  }

  for(;!betaChanged;) { //line search
    if(!o.allowOverstep) if(alpha>1.) alpha=1.;
    if(alphaLimit>0. && alpha>alphaLimit) alpha=alphaLimit;
    y = x + alpha*Delta;
    fy = f(gy, Hy, y);  evals++;
    if(additionalRegularizer) fy += scalarProduct(y,(*additionalRegularizer)*vectorShaped(y));
    if(o.verbose>2) cout <<" \tprobing y=" <<y;
    if(o.verbose>1) cout <<" \tevals=" <<std::setw(4) <<evals <<" \talpha=" <<std::setw(11) <<alpha <<" \tf(y)=" <<fy <<flush;
    bool wolfe = (fy <= fx + o.wolfe*alpha*scalarProduct(Delta,gx) );
    if(fy==fy && (wolfe || o.nonStrictSteps==-1 || o.nonStrictSteps>(int)it)) { //fy==fy is for NAN?
      //accept new point
      if(o.verbose>1) cout <<" - ACCEPT" <<endl;
      if(fx-fy<o.stopFTolerance) numTinySteps++; else numTinySteps=0;
      x = y;
      fx = fy;
      gx = gy;
      Hx = Hy;
      if(wolfe){
        if(alpha>.9 && beta>o.damping){
          beta *= o.dampingDec;
          if(alpha>1.) alpha=1.;
          betaChanged=true;
        }
        alpha *= o.stepInc;
      }else{
        //this is the nonStrict case... weird, but well
        if(alpha<.01){
          beta*=o.dampingInc;
          alpha*=o.dampingInc*o.dampingInc;
          betaChanged=true;
          if(o.verbose>1) cout <<"(line search stopped)" <<endl;
        }
        alpha *= o.stepDec;
      }
      break;
    } else {
      //reject new point
      if(o.verbose>1) cout <<" - reject" <<flush;
      if(evals>o.stopEvals){
        if(o.verbose>1) cout <<" (evals>stopEvals)" <<endl;
        break; //WARNING: this may lead to non-monotonicity -> make evals high!
      }
      if(alpha<.01){
        beta*=o.dampingInc;
        alpha*=o.dampingInc*o.dampingInc;
        betaChanged=true;
        if(o.verbose>1) cout <<", stop & betaInc" <<endl;
      }else{
        if(o.verbose>1) cout <<"\n\t\t\t\t\t(line search)\t" <<flush;
      }
      alpha *= o.stepDec;
    }
  }

  if(o.verbose>0) fil <<evals <<' ' <<eval_cost <<' ' <<fx <<' ' <<alpha;
  if(o.verbose>2) fil <<' ' <<x;
  if(o.verbose>0) fil <<endl;

  //stopping criteria
#define STOPIF(expr, code, ret) if(expr){ if(o.verbose>1) cout <<"\t\t\t\t\t\t--- stopping criterion='" <<#expr <<"'" <<endl; code; return stopCriterion=ret; }
  STOPIF(absMax(Delta)<o.stopTolerance, , stopCrit1);
  STOPIF(numTinySteps>10, numTinySteps=0, stopCrit2);
//  STOPIF(alpha*absMax(Delta)<1e-3*o.stopTolerance, stopCrit2);
  STOPIF(evals>=o.stopEvals, , stopCritEvals);
  STOPIF(it>=o.stopIters, , stopCritEvals);
#undef STOPIF

  return stopCriterion=stopNone;
}


OptNewton::~OptNewton(){
  if(o.fmin_return) *o.fmin_return=fx;
  if(o.verbose>0) fil.close();
#ifndef MLR_MSVC
//  if(o.verbose>1) gnuplot("plot 'z.opt' us 1:3 w l", NULL, true);
#endif
  if(o.verbose>1) cout <<"--- optNewtonStop: f(x)=" <<fx <<endl;
}


OptNewton::StopCriterion OptNewton::run(uint maxIt){
  numTinySteps=0;
  for(uint i=0;i<maxIt;i++){
    step();
    if(stopCriterion==stopStepFailed) continue;
    if(stopCriterion>=stopCrit1) break;
  }
  return stopCriterion;
}
/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */

#include <iomanip>

#include "gradient.h"


//===========================================================================

OptGrad::OptGrad(arr& _x, const ScalarFunction& _f,  OptOptions _o):
  x(_x), f(_f), o(_o), it(0), evals(0), numTinySteps(0){
  alpha = o.initStep;
  if(f) reinit();
}

void OptGrad::reinit(const arr& _x){
  if(&_x && &_x!=&x) x=_x;
  fx = f(gx, NoArr, x);  evals++;

  //startup verbose
  if(o.verbose>1) cout <<"*** optGrad: starting point f(x)=" <<fx <<" alpha=" <<alpha <<endl;
  if(o.verbose>2) cout <<"             x=" <<x <<endl;
  if(o.verbose>0) fil.open("z.opt");
  if(o.verbose>0) fil <<0 <<' ' <<eval_cost <<' ' <<fx <<' ' <<alpha;
  if(o.verbose>2) fil <<' ' <<x;
  if(o.verbose>0) fil <<endl;
}

OptGrad::StopCriterion OptGrad::step(){
  double fy;
  arr y, gy, Delta;

  it++;
  if(o.verbose>1) cout <<"optGrad it=" <<std::setw(4) <<it <<flush;

  if(!(fx==fx)) HALT("you're calling a gradient step with initial function value = NAN");

  //compute Delta
  Delta = gx / (-length(gx));

  //line search
  uint lineSteps=0;
  for(;;lineSteps++) {
    y = x + alpha*Delta;
    fy = f(gy, NoArr, y);  evals++;
    if(o.verbose>2) cout <<" \tprobing y=" <<y;
    if(o.verbose>1) cout <<" \tevals=" <<std::setw(4) <<evals <<" \talpha=" <<std::setw(11) <<alpha <<" \tf(y)=" <<fy <<flush;
    bool wolfe = (fy <= fx + o.wolfe*alpha*scalarProduct(Delta,gx) );
    if(fy==fy && (wolfe || o.nonStrictSteps==-1 || o.nonStrictSteps>(int)it)) { //fy==fy is for NAN?
      //accept new point
      if(o.verbose>1) cout <<" - ACCEPT" <<endl;
      if(fx-fy<o.stopFTolerance || alpha<o.stopTolerance) numTinySteps++; else numTinySteps=0;
      x = y;
      fx = fy;
      gx = gy;
      if(wolfe){
        alpha *= o.stepInc;
      }else{
        alpha *= o.stepDec;
      }
      break;
    } else {
      //reject new point
      if(o.verbose>1) cout <<" - reject" <<flush;
      if(lineSteps>o.stopLineSteps) break;
      if(evals>o.stopEvals) break; //WARNING: this may lead to non-monotonicity -> make evals high!
      if(o.verbose>1) cout <<"\n  (line search)" <<flush;
      alpha *= o.stepDec;
    }
  }

  if(o.verbose>0) fil <<evals <<' ' <<eval_cost <<' ' <<fx <<' ' <<alpha;
  if(o.verbose>2) fil <<' ' <<x;
  if(o.verbose>0) fil <<endl;

  //stopping criteria
#define STOPIF(expr, code, ret) if(expr){ if(o.verbose>1) cout <<"\t\t\t\t\t\t--- stopping criterion='" <<#expr <<"'" <<endl; code; return stopCriterion=ret; }
  //  STOPIF(absMax(Delta)<o.stopTolerance, , stopCrit1);
  STOPIF(numTinySteps>o.stopTinySteps, numTinySteps=0, stopCrit2);
  //  STOPIF(alpha<1e-3*o.stopTolerance, stopCrit2);
  STOPIF(lineSteps>=o.stopLineSteps, , stopCritLineSteps);
  STOPIF(evals>=o.stopEvals, , stopCritEvals);
  STOPIF(it>=o.stopIters, , stopCritEvals);
#undef STOPIF

  return stopCriterion=stopNone;
}


OptGrad::~OptGrad(){
  if(o.fmin_return) *o.fmin_return=fx;
  if(o.verbose>0) fil.close();
#ifndef MLR_MSVC
//  if(o.verbose>1) gnuplot("plot 'z.opt' us 1:3 w l", NULL, true);
#endif
  if(o.verbose>1) cout <<"--- OptGradStop: f(x)=" <<fx <<endl;
}


OptGrad::StopCriterion OptGrad::run(uint maxIt){
  numTinySteps=0;
  for(uint i=0;i<maxIt;i++){
    step();
    if(stopCriterion==stopStepFailed) continue;
    if(stopCriterion==stopCritLineSteps){ reinit();   continue; }
    if(stopCriterion>=stopCrit1) break;
  }
//  if(o.verbose>1) gnuplot("plot 'z.opt' us 1:3 w l", NULL, false);
  if(o.fmin_return) *o.fmin_return= fx;
  return stopCriterion;
}


//===========================================================================
//
// Rprop
//

int _sgn(double x) { if(x > 0) return 1; if(x < 0) return -1; return 0; }
double _mymin(double x, double y) { return x < y ? x : y; }
double _mymax(double x, double y) { return x > y ? x : y; }

struct sRprop {
  double incr;
  double decr;
  double dMax;
  double dMin;
  double rMax;
  double delta0;
  arr lastGrad; // last error gradient
  arr stepSize; // last update
  bool step(arr& w, const arr& grad, uint *singleI);
};

Rprop::Rprop() {
  s = new sRprop;
  s->incr   = 1.2;
  s->decr   = .33;
  s->dMax = 50.;
  s->dMin = 1e-6;
  s->rMax = 0.;
  s->delta0 = 1.;
}

Rprop::~Rprop() {
  delete s;
}

void Rprop::init(double initialStepSize, double minStepSize, double maxStepSize) {
  s->stepSize.resize(0);
  s->lastGrad.resize(0);
  s->delta0 = initialStepSize;
  s->dMin = minStepSize;
  s->dMax = maxStepSize;
}

bool sRprop::step(arr& w, const arr& grad, uint *singleI) {
  if(!stepSize.N) { //initialize
    stepSize.resize(w.N);
    lastGrad.resize(w.N);
    lastGrad.setZero();
    stepSize = delta0;
  }
  CHECK_EQ(grad.N,stepSize.N, "Rprop: gradient dimensionality changed!");
  CHECK_EQ(w.N,stepSize.N   , "Rprop: parameter dimensionality changed!");

  uint i=0, I=w.N;
  if(singleI) { i=*(singleI); I=i+1; }
  for(; i<I; i++) {
    if(grad.elem(i) * lastGrad(i) > 0) { //same direction as last time
      if(rMax) dMax=fabs(rMax*w.elem(i));
      stepSize(i) = _mymin(dMax, incr * stepSize(i)); //increase step size
      w.elem(i) += stepSize(i) * -_sgn(grad.elem(i)); //step in right direction
      lastGrad(i) = grad.elem(i);                    //memorize gradient
    } else if(grad.elem(i) * lastGrad(i) < 0) { //change of direction
      stepSize(i) = _mymax(dMin, decr * stepSize(i)); //decrease step size
      w.elem(i) += stepSize(i) * -_sgn(grad.elem(i)); //step in right direction (undo half the step)
      lastGrad(i) = 0;                               //memorize to continue below next time
    } else {                              //after change of direcion
      w.elem(i) += stepSize(i) * -_sgn(grad.elem(i)); //step in right direction
      lastGrad(i) = grad.elem(i);                    //memorize gradient
    }
  }

  return stepSize.max() < incr*dMin;
}

bool Rprop::step(arr& x, const ScalarFunction& f) {
  arr grad;
  f(grad, NoArr, x);
  return s->step(x, grad, NULL);
}

//----- the rprop wrapped with stopping criteria
uint Rprop::loop(arr& _x,
                 const ScalarFunction& f,
                 double *fmin_return,
                 double stoppingTolerance,
                 double initialStepSize,
                 uint maxEvals,
                 uint verbose) {

  if(!s->stepSize.N) init(initialStepSize);
  arr x, J(_x.N), x_min, J_min;
  double fx, fx_min=0;
  uint rejects=0, small_steps=0;
  x=_x;

  if(verbose>1) cout <<"*** optRprop: starting point x=" <<x <<endl;
  ofstream fil;
  if(verbose>0) fil.open("z.opt");

  uint evals=0;
  double diff=0.;
  for(;;) {
    //checkGradient(p, x, stoppingTolerance);
    //compute value and gradient at x
    fx = f(J, NoArr, x);  evals++;

    if(verbose>0) fil <<evals <<' ' <<eval_cost <<' ' << fx <<' ' <<diff <<' ' <<x <<endl;
    if(verbose>1) cout <<"optRprop " <<evals <<' ' <<eval_cost <<" \tf(x)=" <<fx <<" \tdiff=" <<diff <<" \tx=" <<(x.N<20?x:arr()) <<endl;

    //infeasible point! undo the previous step
    if(fx!=fx) { //is NAN
      if(!evals) HALT("can't start Rprop with unfeasible point");
      s->stepSize*=(double).1;
      s->lastGrad=(double)0.;
      x=x_min;
      fx=fx_min;
      J=J_min;
      rejects=0;
    }

    //update best-so-far
    if(evals<=1) { fx_min= fx; x_min=x; }
    if(fx<=fx_min) {
      x_min=x;
      fx_min=fx;
      J_min=J;
      rejects=0;
    } else {
      rejects++;
      if(rejects>10) {
        s->stepSize*=(double).1;
        s->lastGrad=(double)0.;
        x=x_min;
        fx=fx_min;
        J=J_min;
        rejects=0;
      }
    }

    //update x
    s->step(x, J, NULL);

    //check stopping criterion based on step-length in x
    diff=maxDiff(x, x_min);

    if(diff<stoppingTolerance) { small_steps++; } else { small_steps=0; }
    if(small_steps>3)  break;
    if(evals>maxEvals) break;
  }
  if(verbose>0) fil.close();
//  if(verbose>1) gnuplot("plot 'z.opt' us 1:3 w l", NULL, true);
  if(fmin_return) *fmin_return= fx_min;
  _x=x_min;
  return evals;
}
#include "kOrderMarkov.h"

#define TT T //(T+1)
#define tlT (t<T) //(t<=T)

void conv_KOrderMarkovFunction_ConstrainedProblem(KOrderMarkovFunction& f, arr& phi, arr& J, arr& H, ObjectiveTypeA& tt, const arr& x) {
#if 1
  //set state
  f.set_x(x);

  uint T=f.get_T();
  uint k=f.get_k();
  uint dim_phi=0;
  for(uint t=0; tlT; t++) dim_phi += f.dim_phi(t);
  uint dim_xmax = 0;
  for(uint t=0; tlT; t++){ uint d=f.dim_x(t); if(d>dim_xmax) dim_xmax=d; }

  //resizing things:
  phi.resize(dim_phi).setZero();
  RowShifted *Jaux=NULL;
  if(&J){
    Jaux = makeRowShifted(J, dim_phi, (k+1)*dim_xmax, x.N);
    J.setZero();
  }
  if(&tt) tt.resize(dim_phi).setZero();

  //loop over time t
  uint Jshift=0;
  uint M=0;
  for(uint t=0; tlT; t++) {
    uint dimxbar = 0;
    for(int s=(int)t-k;s<=(int)t;s++) if(s>=0) dimxbar += f.dim_x(s);

    //query
    arr phi_t, J_t;
    ObjectiveTypeA tt_t;
    f.phi_t(phi_t, (&J?J_t:NoArr), tt_t, t);
    //    CHECK_EQ(phi_t.N, f.dim_phi(t), "");
    if(!phi_t.N) continue;
    phi.setVectorBlock(phi_t, M);
    if(&tt) tt.setVectorBlock(tt_t, M);
    if(&J) {
      CHECK(J_t.nd==2 && J_t.d0==phi_t.N && J_t.d1==dimxbar,"");
      if(t>=k) {
        J.setMatrixBlock(J_t, M, 0);
        for(uint i=0; i<phi_t.N; i++) Jaux->rowShift(M+i) = Jshift;
        Jshift += f.dim_x(t-k);
      } else { //cut away the Jacobian w.r.t. the prefix
//        J_t.delColumns(0,(k-t)*n); //nothing to cut
        J.setMatrixBlock(J_t, M, 0);
        for(uint i=0; i<phi_t.N; i++) Jaux->rowShift(M+i) = 0;
      }
    }
    M += phi_t.N;
  }

  CHECK_EQ(M, dim_phi,"");
  if(&J){
    Jaux->reshift();
    Jaux->computeColPatches(true);
  }

  if(&H) H.clear();
#else

  //probing dimensionality
  uint T=f.get_T();
  uint k=f.get_k();
  uint n=f.dim_x();
  arr x_pre=f.get_prefix();
  arr x_post=f.get_postfix();
  arr x;
  x.referTo(_x);
  x.reshape(T+1-x_post.d0, n);
  uint dim_phi=0;
  for(uint t=0; tlT; t++) dim_phi+=f.dim_phi(t);
  CHECK(x.nd==2 && x.d1==n && x.d0==T+1-x_post.d0,"");
  CHECK(x_pre.nd==2 && x_pre.d1==n && x_pre.d0==k,"prefix is of wrong dim");

  //resizing things:
  phi.resize(dim_phi).setZero();
  RowShifted *Jaux;
  if(&J){
    Jaux = makeRowShifted(J, dim_phi, (k+1)*n, _x.N);
    J.setZero();
  }
  if(&tt) tt.resize(dim_phi).setZero();

  //loop over time t
  uint M=0;
  for(uint t=0; tlT; t++) {
    uint dimphi_t = f.dim_phi(t);
//    uint dimg_t   = f.dim_g(t);
//    uint dimh_t   = f.dim_h(t);
//    uint dimf_t   = dimphi_t - dimg_t - dimh_t;
    if(!dimphi_t) continue;

    //construct x_bar
    arr x_bar;
    if(t>=k) {
      if(t>=x.d0) { //x_bar includes the postfix
        x_bar.resize(k+1,n);
        for(int i=t-k; i<=(int)t; i++) x_bar[i-t+k]() = (i>=(int)x.d0)? x_post[i-x.d0] : x[i];
      } else{
        x_bar.referToRange(x, t-k, t);
      }
    } else { //x_bar includes the prefix
      x_bar.resize(k+1,n);
      for(int i=t-k; i<=(int)t; i++) x_bar[i-t+k]() = (i<0)? x_pre[k+i] : x[i];
    }

    //query
    arr phi_t, J_t;
    ObjectiveTypeA tt_t;
    f.phi_t(phi_t, (&J?J_t:NoArr), tt_t, t, x_bar);
    CHECK_EQ(phi_t.N,dimphi_t,"");
    phi.setVectorBlock(phi_t, M);
    if(&tt) tt.setVectorBlock(tt_t, M);

    //if the jacobian is returned
    if(&J) {
      if(J_t.nd==3) J_t.reshape(J_t.d0,J_t.d1*J_t.d2);
      //insert J_t into the large J at index M
      CHECK(J_t.d0==dimphi_t && J_t.d1==(k+1)*n,"");
      if(t>=k) {
        J.setMatrixBlock(J_t, M, 0);
        for(uint i=0; i<dimphi_t; i++) Jaux->rowShift(M+i) = (t-k)*n;
      } else { //cut away the Jacobian w.r.t. the prefix
        J_t.delColumns(0,(k-t)*n);
        J.setMatrixBlock(J_t, M, 0);
        for(uint i=0; i<dimphi_t; i++) Jaux->rowShift(M+i) = 0;
      }
    }

    M += dimphi_t;
  }

  CHECK_EQ(M, dim_phi,"");
  if(&J){
    Jaux->computeColPatches(true);
  }

  if(&H) H.clear();
#endif
}
/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#include "Graph_Problem.h"

Conv_Graph_ConstrainedProblem::Conv_Graph_ConstrainedProblem(GraphProblem& _G) : G(_G){
  G.getStructure(variableDimensions, featureVariables, featureTypes);
  varDimIntegral = integral(variableDimensions);
}

void Conv_Graph_ConstrainedProblem::phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& tt, const arr& x) {
  G.phi(phi, J_G, H_G, x);

  if(&tt) tt = featureTypes;

  //-- construct a row-shifed J from the array of featureJs
  if(&J){
    J.resize(phi.N, x.N).setZero();

    for(uint i=0; i<phi.N; i++) { //loop over features
      arr& Ji = J_G(i);
      uint c=0;
      for(uint& j:featureVariables(i)){ //loop over variables of this features
        uint xjN = variableDimensions(j);
#if 0
        memmove(&J(i,(j?varDimIntegral(j-1):0)), Ji.p+c, xjN+Ji.sizeT);
        c+=xjN;
#else
        for(uint xi=0;xi<xjN;xi++){ //loop over variable dimension
          J(i, (j?varDimIntegral(j-1):0) + xi) = Ji.elem(c);
          c++;
        }
#endif
      }
      CHECK_EQ(c, Ji.N, "you didn't count through all indexes");
    }
  }
}
/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */

#include "convert.h"
#include "KOMO_Problem.h"

//the Convert is essentially only a ``garbage collector'', creating all the necessary conversion objects and then deleting them on destruction
Convert::Convert(const ScalarFunction& p) : cstyle_fs(NULL), cstyle_fv(NULL), data(NULL), cpm(NULL) { sf=p; }
Convert::Convert(const VectorFunction& p) : cstyle_fs(NULL), cstyle_fv(NULL), data(NULL), cpm(NULL) { vf=p; }
//Convert::Convert(KOrderMarkovFunction& p):kom(&p), cstyle_fs(NULL), cstyle_fv(NULL), data(NULL) { }
Convert::Convert(double(*fs)(arr*, const arr&, void*),void *data) : cstyle_fs(fs), cstyle_fv(NULL), data(data), cpm(NULL) {  }
Convert::Convert(void (*fv)(arr&, arr*, const arr&, void*),void *data) : cstyle_fs(NULL), cstyle_fv(fv), data(data), cpm(NULL) {  }

#ifndef libRoboticsCourse
//Convert::Convert(ControlledSystem& p) { cs=&p; }
#endif

Convert::~Convert() {
  if(cpm){ delete cpm; cpm=NULL; }
}

//void conv_KOrderMarkovFunction_ConstrainedProblem(KOrderMarkovFunction& f, arr& phi, arr& J, arr& H, ObjectiveTypeA& tt, const arr& x);
double conv_VectorFunction_ScalarFunction(VectorFunction f, arr& g, arr& H, const arr& x){
  arr y,J;
  f(y, (&g?J:NoArr), x);
  //  if(J.special==arr::RowShiftedST) J = unpack(J);
  if(&g){ g = comp_At_x(J, y); g *= 2.; }
  if(&H){ H = comp_At_A(J); H *= 2.; }
  return sumOfSqr(y);
}

//===========================================================================
//
// casting methods
//

Convert::operator ScalarFunction() {
  if(!sf) {
    if(cstyle_fs) sf = conv_cstylefs2ScalarFunction(cstyle_fs, data);
    else {
      if(!vf) vf = this->operator VectorFunction();
      if(vf)  sf = conv_VectorFunction2ScalarFunction(vf);
    }
  }
  if(!sf) HALT("");
  return sf;
}

Convert::operator VectorFunction() {
  if(!vf) {
    if(cstyle_fv)
      vf = conv_cstylefv2VectorFunction(cstyle_fv, data);
//    else {
//      if(kom) vf = conv_KOrderMarkovFunction2VectorFunction(*kom);
//    }
  }
  if(!vf) HALT("");
  return vf;
}

//Convert::operator KOrderMarkovFunction&() {
//  if(!kom) {
//// #ifndef libRoboticsCourse
////     if(cs) kom = new sConvert::ControlledSystem_2OrderMarkovFunction(*cs);
//// #endif
//  }
//  if(!kom) HALT("");
//  return *kom;
//}

//===========================================================================
//
// actual convertion routines
//

ScalarFunction conv_cstylefs2ScalarFunction(double(*fs)(arr*, const arr&, void*),void *data){
  return [&fs,data](arr& g, arr& H, const arr& x) -> double {
    if(&H) NIY;
    return fs(&g, x, data);
  };
}

VectorFunction conv_cstylefv2VectorFunction(void (*fv)(arr&, arr*, const arr&, void*),void *data){
  return [&fv,data](arr& y, arr& J, const arr& x) -> void {
    fv(y, &J, x, data);
  };
}

ScalarFunction conv_VectorFunction2ScalarFunction(const VectorFunction& f) {
  return [&f](arr& g, arr& H, const arr& x) -> double {
    arr y,J;
    f(y, (&g?J:NoArr), x);
    //  if(J.special==arr::RowShiftedST) J = unpack(J);
    if(&g){ g = comp_At_x(J, y); g *= 2.; }
    if(&H){ H = comp_At_A(J); H *= 2.; }
    return sumOfSqr(y);
  };
}

void Conv_linearlyReparameterize_ConstrainedProblem::phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& tt, const arr& z){
  arr x = B*z;
  P.phi(phi, J, H, tt, x);
  if(&J) J = comp_A_x(J,B);
  if(&H && H.N) NIY;
}


//===========================================================================

Convert::Convert(KOMO_Problem& p) : cstyle_fs(NULL), cstyle_fv(NULL), data(NULL), cpm(NULL) {
  cpm = new Conv_KOMO_ConstrainedProblem(p);
}

Convert::operator ConstrainedProblem&() {
  if(!cpm) HALT("");
  return *cpm;
}


//===========================================================================

RUN_ON_INIT_BEGIN()
mlr::Array<ObjectiveType>::memMove=true;
RUN_ON_INIT_END()
#include "BayesOpt.h"
//#include <Plot/plot.h>

#include <Algo/MLcourse.h>

BayesOpt::BayesOpt(const ScalarFunction& _f, const arr& bounds_lo, const arr& bounds_hi, double init_lengthScale, OptOptions o)
  : f(_f),
    bounds_lo(bounds_lo), bounds_hi(bounds_hi),
    f_now(NULL), f_smaller(NULL),
    alphaMinima_now(ScalarFunction(), bounds_lo, bounds_hi),
    alphaMinima_smaller(ScalarFunction(), bounds_lo, bounds_hi) {

  init_lengthScale *= sum(bounds_hi - bounds_lo)/bounds_lo.N;

  kernel_now = new DefaultKernelFunction();
  kernel_smaller = new DefaultKernelFunction();

  kernel_now->type = kernel_smaller->type = DefaultKernelFunction::Gauss; //TODO: ugly!!

  kernel_now->hyperParam1 = ARR(init_lengthScale);
  kernel_now->hyperParam2 = ARR(1.);
  kernel_smaller->hyperParam1 = kernel_now->hyperParam1;
  kernel_smaller->hyperParam1 /= 2.;
  kernel_smaller->hyperParam2 = kernel_now->hyperParam2;
}

BayesOpt::~BayesOpt(){
  delete kernel_now;
  delete kernel_smaller;
  delete f_now;
  delete f_smaller;

}

void BayesOpt::step(){
  arr x;
  if(!data_X.N){
    x = bounds_lo + (bounds_hi-bounds_lo) % rand(bounds_lo.N);
  }else{
    x = pickNextPoint();
  }

  double fx = f(NoArr, NoArr, x);
//  report();

  addDataPoint(x, fx);

  reOptimizeAlphaMinima();
}

void BayesOpt::run(uint maxIt){
  for(uint i=0;i<maxIt;i++) step();
}

void BayesOpt::report(bool display){
  if(!f_now) return;
  cout <<"mean=" <<f_now->mu <<" var=" <<kernel_now->hyperParam2.scalar() <<endl;

  arr X_grid, s_grid;
  X_grid.setGrid(data_X.d1, 0., 1., (data_X.d1==1?500:30));
  X_grid = X_grid % (bounds_hi-bounds_lo);
  X_grid += repmat(bounds_lo, X_grid.d0, 1);
  arr y_grid = f_now->evaluate(X_grid, s_grid);
  s_grid = sqrt(s_grid);

  arr s2_grid;
  arr y2_grid = f_smaller->evaluate(X_grid, s2_grid);
  s2_grid = sqrt(s2_grid);

  arr locmin_X(0u,data_X.d1), locmin_y;
  for(auto& l:alphaMinima_now.localMinima){
    locmin_X.append(l.x);
    locmin_y.append(l.fx);
  }
  arr locmin2_X(0u,data_X.d1), locmin2_y;
  for(auto& l:alphaMinima_smaller.localMinima){
    locmin2_X.append(l.x);
    locmin2_y.append(l.fx);
  }

//  plotGnuplot();
//  plotClear();
//  plotFunctionPrecision(X_grid, y_grid, y_grid+s_grid, y_grid-s_grid);
//  plotFunction(X_grid, y2_grid);
//  plotFunction(X_grid, y2_grid-s2_grid);
//  plotPoints(data_X, data_y);
//  plotPoints(locmin_X, locmin_y);
//  plotPoints(locmin2_X, locmin2_y);
//  plot(false);
}

void BayesOpt::addDataPoint(const arr& x, double y){
  if(f_now) delete f_now;
  if(f_smaller) delete f_smaller;

  data_X.append(x);  data_X.reshape(data_X.N/x.N, x.N);
  data_y.append(y);

  double fmean = sum(data_y)/data_y.N;
  if(data_y.N>2){
    kernel_now->hyperParam2 = 2.*var(data_y);
    kernel_smaller->hyperParam2 = kernel_now->hyperParam2;
  }

  f_now = new KernelRidgeRegression(data_X, data_y, *kernel_now, -1., fmean);
  f_smaller = new KernelRidgeRegression(data_X, data_y, *kernel_smaller, -1., fmean);
}

void BayesOpt::reOptimizeAlphaMinima(){
  alphaMinima_now.newton.f = f_now->getF(-1.);
  alphaMinima_smaller.newton.f = f_smaller->getF(-1.);

  alphaMinima_now.reOptimizeAllPoints();
  alphaMinima_now.run(20);
  alphaMinima_smaller.reOptimizeAllPoints();
  alphaMinima_smaller.run(20);
}

arr BayesOpt::pickNextPoint(){
  arr x_now = alphaMinima_now.best->x;
  arr x_sma = alphaMinima_smaller.best->x;

  double fx_0 = f_now->evaluate(x_now, NoArr, NoArr, -2., false);
  double fx_1 = f_smaller->evaluate(x_sma, NoArr, NoArr, -1., false);

  if(fx_1 < fx_0){
    reduceLengthScale();
    return x_sma;
  }

  return x_now;
}

void BayesOpt::reduceLengthScale(){
  cout <<"REDUCING LENGTH SCALE!!" <<endl;
  kernel_now->hyperParam1 = kernel_smaller->hyperParam1;
  kernel_smaller->hyperParam1 /= 2.;
}
/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#include "optimization.h"

uint eval_cost=0;
Singleton<OptOptions> globalOptOptions;
const char* ObjectiveTypeString[]={"OT_none", "OT_f", "OT_sumOfSqr", "OT_ineq", "OT_eq" };
ObjectiveTypeA& NoTermTypeA = *((ObjectiveTypeA*)NULL);

//===========================================================================
//
// checks and converters
//


bool checkJacobianCP(ConstrainedProblem &P, const arr& x, double tolerance){
  VectorFunction F = [&P](arr& phi, arr& J, const arr& x){
    return P.phi(phi, J, NoArr, NoTermTypeA, x);
  };
  return checkJacobian(F, x, tolerance);
}

bool checkHessianCP(ConstrainedProblem &P, const arr& x, double tolerance){
  uint i;
  arr phi, J;
  ObjectiveTypeA tt;
  P.phi(phi, NoArr, NoArr, tt, x); //TODO: only call getStructure
  for(i=0;i<tt.N;i++) if(tt(i)==OT_f) break;
  if(i==tt.N){
    MLR_MSG("no f-term in this KOM problem");
    return true;
  }
  ScalarFunction F = [&P,&phi,&J,i](arr& g, arr& H, const arr& x) -> double{
    P.phi(phi, J, H, NoTermTypeA, x);
    g = J[i];
    return phi(i);
  };
  return checkHessian(F, x, tolerance);
}

//===========================================================================
//
// optimization options
//

OptOptions::OptOptions() {
  verbose    = mlr::getParameter<uint>  ("opt/verbose", 1);
  fmin_return=NULL;
  stopTolerance= mlr::getParameter<double>("opt/stopTolerance", 1e-2);
  stopFTolerance= mlr::getParameter<double>("opt/stopFTolerance", 1e-1);
  stopGTolerance= mlr::getParameter<double>("opt/stopGTolerance", -1.);
  stopEvals = mlr::getParameter<uint>  ("opt/stopEvals", 1000);
  stopIters = mlr::getParameter<uint>  ("opt/stopIters", 1000);
  stopLineSteps = mlr::getParameter<uint>  ("opt/stopLineSteps", 10);
  stopTinySteps = mlr::getParameter<uint>  ("opt/stopTinySteps", 10);
  initStep  = mlr::getParameter<double>("opt/initStep", 1.);
  minStep   = mlr::getParameter<double>("opt/minStep", -1.);
  maxStep   = mlr::getParameter<double>("opt/maxStep", .2);
  damping   = mlr::getParameter<double>("opt/damping", .1);
  stepInc   = mlr::getParameter<double>("opt/stepInc", 2.);
  stepDec   = mlr::getParameter<double>("opt/stepDec", .1);
  dampingInc= mlr::getParameter<double>("opt/dampingInc", 2.);
  dampingDec= mlr::getParameter<double>("opt/dampingDec", .5);
  wolfe     = mlr::getParameter<double>("opt/wolfe", .01);
  nonStrictSteps= mlr::getParameter<uint>  ("opt/nonStrictSteps", 0);
  allowOverstep= mlr::getParameter<bool>  ("opt/allowOverstep", false);
  constrainedMethod = (ConstrainedMethodType)mlr::getParameter<int>("opt/constrainedMethod", augmentedLag);
  muInit = mlr::getParameter<double>("opt/muInit", 1.);
  muLBInit = mlr::getParameter<double>("opt/muLBInit", 1.);
  aulaMuInc = mlr::getParameter<double>("opt/aulaMuInc", 2.);
}

void OptOptions::write(std::ostream& os) const{
#define WRT(x) os <<#x <<" = " <<x <<endl;
  WRT(verbose);
//  double *fmin_return);
  WRT(stopTolerance);
  WRT(stopEvals);
  WRT(stopIters);
  WRT(initStep);
  WRT(minStep);
  WRT(maxStep);
  WRT(damping);
  WRT(stepInc);
  WRT(stepDec);
  WRT(dampingInc);
  WRT(dampingDec);
  WRT(nonStrictSteps);
  WRT(allowOverstep);
  WRT(constrainedMethod);
  WRT(aulaMuInc);
#undef WRT
}

//===========================================================================
//
// helpers
//

void displayFunction(const ScalarFunction &f, bool wait, double lo, double hi){
  arr X, Y;
  X.setGrid(2,lo,hi,100);
  Y.resize(X.d0);
  for(uint i=0;i<X.d0;i++){
    double fx=f(NoArr, NoArr, X[i]);
    Y(i) = ((fx==fx && fx<10.)? fx : 10.);
  }
  Y.reshape(101,101);
//  plotGnuplot();  plotSurface(Y);  plot(true);
  write(LIST<arr>(Y),"z.fct");
  gnuplot("reset; splot [-1:1][-1:1] 'z.fct' matrix us ($1/50-1):($2/50-1):3 w l", wait, true);
}



/// minimizes \f$f(x)\f$ using its gradient only
uint optGradDescent(arr& x, const ScalarFunction& f, OptOptions o) {
  uint evals=0;
  arr y, grad_x, grad_y;
  double fx, fy;
  double a=o.initStep;
  
  fx = f(grad_x, NoArr, x);  evals++;
  if(o.verbose>1) cout <<"*** optGradDescent: starting point x=" <<(x.N<20?x:arr()) <<" f(x)=" <<fx <<" a=" <<a <<endl;
  ofstream fil;
  if(o.verbose>0) fil.open("z.opt");
  if(o.verbose>0) fil <<0 <<' ' <<eval_cost <<' ' <<fx <<' ' <<a <<' ' <<x <<endl;
  
  grad_x /= length(grad_x);
  
  for(uint k=0;; k++) {
    y = x - a*grad_x;
    fy = f(grad_y, NoArr, y);  evals++;
    CHECK_EQ(fy,fy, "cost seems to be NAN: fy=" <<fy);
    if(o.verbose>1) cout <<"optGradDescent " <<evals <<' ' <<eval_cost <<" \tprobing y=" <<(y.N<20?y:arr()) <<" \tf(y)=" <<fy <<" \t|grad|=" <<length(grad_y) <<" \ta=" <<a;
    
    if(fy <= fx) {
      if(o.verbose>1) cout <<" - ACCEPT" <<endl;
      double step=length(x-y);
      x = y;
      fx = fy;
      grad_x = grad_y/length(grad_y);
      a *= 1.2;
      if(o.maxStep>0. && a>o.maxStep) a = o.maxStep;
      if(o.verbose>0) fil <<evals <<' ' <<eval_cost <<' ' <<fx <<' ' <<a <<' ' <<x <<endl;
      if(step<o.stopTolerance) break;
    } else {
      if(o.verbose>1) cout <<" - reject" <<endl;
      a *= .5;
    }
    if(evals>o.stopEvals) break; //WARNING: this may lead to non-monotonicity -> make evals high!
    if(k>o.stopIters) break;
  }
  if(o.verbose>0) fil.close();
  if(o.verbose>1) gnuplot("plot 'z.opt' us 1:3 w l",NULL,true);
  return evals;
}



RUN_ON_INIT_BEGIN(optimization)
ObjectiveTypeA::memMove=true;
RUN_ON_INIT_END(optimization)
/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */

#include "benchmarks.h"
//#include "functions.h"

//===========================================================================

double _RosenbrockFunction(arr& g, arr& H, const arr& x) {
  double f=0.;
  for(uint i=1; i<x.N; i++) f += mlr::sqr(x(i)-mlr::sqr(x(i-1))) + .01*mlr::sqr(1-10.*x(i-1));
  f = ::log(1.+f);
  if(&g) NIY;
  if(&H) NIY;
  return f;
};

ScalarFunction RosenbrockFunction(){ return _RosenbrockFunction; }

//===========================================================================

double _RastriginFunction(arr& g, arr& H, const arr& x) {
  double A=.5, f=A*x.N;
  for(uint i=0; i<x.N; i++) f += x(i)*x(i) - A*::cos(10.*x(i));
  if(&g) {
    g.resize(x.N);
    for(uint i=0; i<x.N; i++) g(i) = 2*x(i) + 10.*A*::sin(10.*x(i));
  }
  if(&H) {
    H.resize(x.N,x.N);  H.setZero();
    for(uint i=0; i<x.N; i++) H(i,i) = 2 + 100.*A*::cos(10.*x(i));
  }
  return f;
}

ScalarFunction RastriginFunction(){ return _RastriginFunction; }

//===========================================================================

double _SquareFunction(arr& g, arr& H, const arr& x) {
  if(&g) g=2.*x;
  if(&H) H.setDiag(2., x.N);
  return sumOfSqr(x);
}

ScalarFunction SquareFunction(){ return _SquareFunction; }

//===========================================================================

double _SumFunction(arr& g, arr& H, const arr& x) {
  if(&g) { g.resize(x.N); g=1.; }
  if(&H) { H.resize(x.N,x.N); H.setZero(); }
  return sum(x);
}

ScalarFunction SumFunction(){ return _SumFunction; }

//===========================================================================

double _HoleFunction(arr& g, arr& H, const arr& x) {
  double f=exp(-sumOfSqr(x));
  if(&g) g=2.*f*x;
  if(&H) { H.setDiag(2.*f, x.N); H -= 4.*f*(x^x); }
  f = 1.-f;
  return f;
}

ScalarFunction HoleFunction(){ return _HoleFunction; }

//===========================================================================

struct _ChoiceFunction : ScalarFunction {
  enum Which { none=0, sum, square, hole, rosenbrock, rastrigin } which;
  arr condition;
  _ChoiceFunction():which(none){
    ScalarFunction::operator=(
          [this](arr& g, arr& H, const arr& x) -> double { return this->fs(g, H, x); }
    );
  }

  double fs(arr& g, arr& H, const arr& x) {
    //initialize on first call
    if(which==none){
      which = (Which) mlr::getParameter<int>("fctChoice");
    }
    if(condition.N!=x.N){
      condition.resize(x.N);
      double cond = mlr::getParameter<double>("condition");
      if(x.N>1){
        for(uint i=0; i<x.N; i++) condition(i) = pow(cond,0.5*i/(x.N-1));
      }else{
        condition = cond;
      }
    }

    arr y = x;
    y *= condition; //elem-wise product
    double f;
    switch(which) {
      case sum: f = _SumFunction(g, H, y); break;
      case square: f = _SquareFunction(g, H, y); break;
      case hole: f = _HoleFunction(g, H, y); break;
      case rosenbrock: f = _RosenbrockFunction(g, H, y); break;
      case rastrigin: f = _RastriginFunction(g, H, y); break;
      default: NIY;
    }
    if(&g) g *= condition; //elem-wise product
    if(&H) H = condition%H%condition;
    return f;
  }

  //  ScalarFunction get_f(){
  //    return [this](arr& g, arr& H, const arr& x) -> double { return this->fs(g, H, x); };
  //  }

} choice;

ScalarFunction ChoiceFunction() { return (ScalarFunction&)choice; }

//===========================================================================

void generateConditionedRandomProjection(arr& M, uint n, double condition) {
  uint i,j;
  //let M be a ortho-normal matrix (=random rotation matrix)
  M.resize(n,n);
  rndUniform(M,-1.,1.,false);
  //orthogonalize
  for(i=0; i<n; i++) {
    for(j=0; j<i; j++) M[i]()-=scalarProduct(M[i],M[j])*M[j];
    M[i]()/=length(M[i]);
  }
  //we condition each column of M with powers of the condition
  for(i=0; i<n; i++) M[i]() *= pow(condition, double(i) / (2.*double(n - 1)));
}

//===========================================================================

SquaredCost::SquaredCost(uint _n, double condition) {
  initRandom(_n, condition);
}

void SquaredCost::initRandom(uint _n, double condition) {
  n=_n;
  generateConditionedRandomProjection(M, n, condition);
  //the metric is equal M^T*M
  //C=~M*M;
  //arr U,d,V;    svd(U, d, V, C);    cout <<U <<d <<V <<M <<C <<endl;
}

void SquaredCost::fv(arr& y, arr& J,const arr& x) {
  CHECK_EQ(x.N,n,"");
  y = M*x;
  if(&J) J=M;
}

//===========================================================================

NonlinearlyWarpedSquaredCost::NonlinearlyWarpedSquaredCost(uint _n, double condition):sq(_n,condition) {
  n=_n;
}

void NonlinearlyWarpedSquaredCost::initRandom(uint _n, double condition) {
  n=_n;
  sq.initRandom(n,condition);
}

void NonlinearlyWarpedSquaredCost::fv(arr& y, arr& J,const arr& x) {
  CHECK_EQ(x.N,n,"");
  arr xx=atan(x);
  y=sq.M*xx;
  if(&J) {
    arr gg(xx.N);
    for(uint i=0; i<gg.N; i++) gg(i) = 1./(1.+x(i)*x(i));
    J = sq.M*diag(gg);
  }
}

//===========================================================================

void ParticleAroundWalls2::getStructure(uintA& variableDimensions, uintA& featureTimes, ObjectiveTypeA& featureTypes){
  variableDimensions = consts<uint>(n,T);

  if(&featureTimes) featureTimes.clear();
  if(&featureTypes) featureTypes.clear();
  for(uint t=0;t<T;t++){
    if(&featureTimes) featureTimes.append( consts<uint>(t, n) );
    if(&featureTypes) featureTypes.append( consts(OT_sumOfSqr, n) );
    if(t==T/4 || t==T/2 || t==3*T/4 || t==T-1){
      if(&featureTimes) featureTimes.append( consts<uint>(t, n) );
      if(&featureTypes) featureTypes.append( consts(OT_ineq, n) );
    }
  }
}

void ParticleAroundWalls2::phi(arr& phi, arrA& J, arrA& H, ObjectiveTypeA& tt, const arr& x){

  uint M=x.N + 4*3;
  phi.resize(M);
  if(&J) J.resize(M);
  if(&tt) tt.resize(M);

  uint m=0;
  for(uint t=0;t<T;t++){
    //-- construct x_bar
    arr x_bar;
    if(t>=k) {
      x_bar.referToRange(x, t-k, t);
    } else { //x_bar includes the prefix
      x_bar.resize(k+1,n);
      for(int i=t-k; i<=(int)t; i++) x_bar[i-t+k]() = (i<0)? zeros(n) : x[i];
    }

    //-- assert some dimensions
    CHECK_EQ(x_bar.d0,k+1,"");
    CHECK_EQ(x_bar.d1,n,"");

    //-- transition costs
    for(uint i=0;i<n;i++){
      if(k==1){
        phi(m) = x_bar(1,i)-x_bar(0,i); //penalize velocity
        if(&J){ J(m).resize(k+1,n).setZero(); J(m)(1,i) = 1.;  J(m)(0,i) = -1.; }
      }
      if(k==2){
        phi(m) = x_bar(2,i)-2.*x_bar(1,i)+x_bar(0,i); //penalize acceleration
        if(&J){ J(m).resize(k+1,n).setZero(); J(m)(2,i) = 1.;  J(m)(1,i) = -2.;  J(m)(0,i) = 1.; }
      }
      if(k==3){
        phi(m) = x_bar(3,i)-3.*x_bar(2,i)+3.*x_bar(1,i)-x_bar(0,i); //penalize jerk
        if(&J){ J(m).resize(k+1,n).setZero(); J(m)(3,i) = 1.;  J(m)(2,i) = -3.;  J(m)(1,i) = +3.;  J(m)(0,i) = -1.; }
      }
      if(&J && t<k) J(m) = J(m).sub(k-t,-1,0,-1); //cut the prefix Jacobians
      if(&tt) tt(m) = OT_sumOfSqr;
      m++;
    }

    //-- wall constraints
    if(t==T/4 || t==T/2 || t==3*T/4 || t==T-1){
      for(uint i=0;i<n;i++){ //add barrier costs to each dimension
        if(t==T/4){
          phi(m) = (i+1.-x_bar(k,i));  //``greater than i+1''
          if(&J){ J(m).resize(k+1,n).setZero(); J(m)(k,i) = -1.; }
        }
        if(t==T/2){
          phi(m) = (x_bar(k,i)+i+1.);  //``lower than -i-1''
          if(&J){ J(m).resize(k+1,n).setZero(); J(m)(k,i) = +1.; }
        }
        if(t==3*T/4){
          phi(m) = (i+1.-x_bar(k,i));  //``greater than i+1''
          if(&J){ J(m).resize(k+1,n).setZero(); J(m)(k,i) = -1.; }
        }
        if(t==T-1){
          phi(m) = (x_bar(k,i)+i+1.);  //``lower than -i-1''
          if(&J){ J(m).resize(k+1,n).setZero(); J(m)(k,i) = +1.; }
        }
        if(&tt) tt(m) = OT_ineq;
        m++;
      }
    }
  }
  CHECK_EQ(m,M,"");
}
/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#include "plot.h"
#include <Core/array.tpp>
#ifdef MLR_GL
#include <Geo/geo.h>
#include <Geo/mesh.h>
#  include <Gui/opengl.h>
#  include <Gui/color.h>
#endif

//===========================================================================
//
// global structures
//

PlotModule plotModule;

struct sPlotModule {
  mlr::Array<arr> array;
  mlr::Array<arr> images;
  mlr::Array<arr> points;
  mlr::Array<arr> lines;
  mlr::Array<mlr::String> legend;
#ifdef MLR_geo_h
  mlr::Array<mlr::Vector> planes;
  mlr::Mesh mesh;
#endif
};

PlotModule::PlotModule() {
  s = new sPlotModule;
  mode=gnupl;
  gl=0;
  light=false;
  grid=false;
  colors=true;
  drawBox=false;
  drawDots=false;
  perspective=false;
  thickLines=0;
}

PlotModule::~PlotModule() {
#ifdef MLR_GL
  if(gl){ delete gl; gl=NULL; }
#endif
  delete s;
}

void plotDrawOpenGL(void* data);
void plotDrawGnuplot(void* data, bool pauseMouse);
void glDrawPlot(void *module) { plotDrawOpenGL(((PlotModule*)module)->s); }

//===========================================================================
//
// color class
//


//===========================================================================
//
// C interface implementations
//

#ifdef MLR_GL
void plotInitGL(double xl=-1., double xh=1., double yl=-1., double yh=1., double zl=-1., double zh=1., const char* name=0, uint width=600, uint height=600, int posx=0, int posy=0) {
  if(!plotModule.gl) {
    plotModule.gl=new OpenGL(name, width, height, posx, posy);
    plotModule.gl->add(glDrawPlot, &plotModule);
    plotModule.gl->setClearColors(1., 1., 1., 1.);
  }
  plotModule.gl->camera.setPosition(.5*(xh+xl), .5*(yh+yl), 5.);
  plotModule.gl->camera.focus(.5*(xh+xl), .5*(yh+yl), .0);
  plotModule.gl->camera.setWHRatio((xh-xl)/(yh-yl));
  if(plotModule.perspective) {
    plotModule.gl->camera.setHeightAngle(45.);
  } else {
    plotModule.gl->camera.setHeightAbs(1.2*(yh-yl));
  }
  plotModule.gl->update();
}

void plotCloseGL(){
  if(plotModule.gl){ delete plotModule.gl; plotModule.gl=NULL; }
}
#endif

void plot(bool wait, const char* txt) {
  if(!mlr::getInteractivity()){
    wait=false;
  }
  switch(plotModule.mode) {
    case gnupl:
      plotDrawGnuplot(plotModule.s, wait);
//      if(wait) mlr::wait();
      break;
#ifdef MLR_GL
    case opengl:
      if(txt) plotModule.gl->text = txt;
      //plotInitGL();
      if(wait) plotModule.gl->watch();
      else plotModule.gl->update();
      break;
#else
    case opengl:
      HALT("can't plot on OpenGL without MLR_GL flag");
      break;
#endif
    case xfig:
      NIY;
      break;
  }
}

void plotClose() {
#ifdef MLR_GL
  if(plotModule.mode==opengl) plotCloseGL();
#endif
}

void plotClear() {
  plotModule.s->array.clear();
  plotModule.s->points.clear();
  plotModule.s->lines.clear();
#ifdef MLR_GL
  plotModule.s->planes.clear();
#endif
}

void plotGnuplot() { plotModule.mode=gnupl; }

#ifdef MLR_GL
void plotOpengl() { plotModule.mode=opengl; plotInitGL(); }

void plotOpengl(bool perspective, double xl, double xh, double yl, double yh, double zl, double zh) {
  plotModule.mode=opengl;
  plotModule.perspective=perspective;
  if(!plotModule.gl) plotInitGL(xl, xh, yl, yh, zl, zh);
}
#else
void plotOpengl() { MLR_MSG("dummy routine - compile with MLR_FREEGLUT to use this!"); }
void plotOpengl(bool perspective, double xl, double xh, double yl, double yh, double zl, double zh) { NICO }
#endif

void plotImage(const arr& x) { plotModule.s->images.append(x); }

void plotFunction(const arr& f, double x0, double x1) {
  arr X;
  uint i, j;
  if(f.nd==2) {
    if(x0 || x1) {
      X.resize(f.d0, f.d1+1);
      for(i=0; i<f.d0; i++) { X(i, 0)=x0+(x1-x0)*i/(f.N-1); for(j=1; j<X.d1; j++) X(i, j)=f(i, j-1); }
    } else {
      X=f;
    }
  }
  if(f.nd==1) {
    if(x0 || x1) {
      X.resize(f.N, 2);
      for(i=0; i<f.d0; i++) { X(i, 0)=x0+(x1-x0)*i/(f.N-1); X(i, 1)=f(i); }
    } else {
      X=f;
      X.reshape(X.N, 1);
    }
  }
  plotModule.s->lines.append(X);
}

void plotFunctions(const arr& F, double x0, double x1) {
  CHECK_EQ(F.nd,2, "");
  arr tF;
  transpose(tF, F);
  for(uint j=0; j<tF.d0; j++) plotFunction(tF[j], x0, x1);
}

void plotFunctionPoints(const arr& x, const arr& f) {
  CHECK_EQ(x.d0,f.d0, "Domain and image of function have different size!")
  arr X(x.d0, x.d1+1);
  uint i, j;
  for(i=0; i<X.d0; i++) {
    for(j=0; j<x.d1; j++) X(i, j)=x(i, j);
    X(i, j)=f(i);
  }
  plotModule.s->points.append(X);
}

void plotFunction(const arr& x, const arr& f) {
  CHECK_EQ(x.d0,f.d0, "Domain and image of function have different size!")
  CHECK_EQ(f.nd,1, "Function image should be 1D")
  CHECK(x.d[x.nd-1]<3, "Can handle up to 2D domains")
  arr X(x.d0, x.d1+1);
  uint i, j;
  for(i=0; i<X.d0; i++) {
    for(j=0; j<x.d1; j++) X(i, j)=x(i, j);
    X(i, j)=f(i);
  }
  plotModule.s->lines.append(X);
}

void plotFunctionPrecision(const arr& x, const arr& f, const arr& h, const arr& l) {
  CHECK_EQ(x.d0,f.d0, "Domain and image of function have different size!")
  CHECK(f.nd==1&&h.nd==1&&l.nd==1, "Function image should be 1D")
  CHECK(x.d[x.nd-1]<2, "Can handle up to 1D domains")
  arr X(x.d0, x.d1+3);
  uint i, j;
  for(i=0; i<X.d0; i++) {
    for(j=0; j<x.d1; j++) X(i, j)=x(i, j);
    X(i, j)=f(i);
    X(i, j+1)=l(i);
    X(i, j+2)=h(i);
  }
  plotModule.s->lines.append(X);
}

void plotSurface(const arr& X) {
  plotModule.s->array.append(X);
#ifdef MLR_GL
  plotModule.s->mesh.clear();
  plotModule.s->mesh.V.resize(X.N, 3);
  plotModule.s->mesh.C.resize(X.N, 3);
  plotModule.s->mesh.setGrid(X.d1, X.d0);
  //plotModule.s->mesh.gridToStrips(X.d1, X.d0);
#endif
}

void plotPoint(double x, double y, double z) {
  arr p(1, 3); p(0, 0)=x; p(0, 1)=y; p(0, 2)=z;
  plotModule.s->points.append(p);
}

void plotPoint(const arr& x) {
  arr p; p.referTo(x); p.reshape(1, p.N);
  plotModule.s->points.append(p);
}

void plotPoints(const arr& X) {
  plotModule.s->points.append(X);
}

void plotClearPoints() {
  plotModule.s->points.clear();
}

void plotLine(const arr& X) {
  plotModule.s->lines.append(X);
}

void plotPoints(const arr& X, const arr& Y) {
  arr P;
  uint i, j;
  if(X.nd==2) {
    P.resize(X.d0, X.d1+1);
    for(i=0; i<P.d0; i++) {
      for(j=0; j<X.d1; j++) P(i, j)=X(i, j);
      P(i, j)=Y(i);
    }
  } else {
    P.resize(X.d0, 2);
    for(i=0; i<P.d0; i++) { P(i, 0)=X(i); P(i, 1)=Y(i); }
  }
  plotModule.s->points.append(P);
}

void plotCovariance(const arr& mean, const arr& cov) {
  if(mean.nd==2) {
    for(uint k=0; k<mean.d0; k++) plotCovariance(mean[k], cov[k]);
    return;
  }
  uint d=mean.N;
  if(d==1) {
    arr d(20, 2);
    uint i;
    for(i=0; i<d.d0; i++) { //standard Gaussian
      d(i, 0)=5. * ((i+.5)/d.d0 - .5);
      d(i, 1)=1./::sqrt(MLR_2PI)*::exp(-.5*d(i, 0)*d(i, 0));
    }
    for(i=0; i<d.d0; i++) { //standard Gaussian
      d(i, 0) = ::sqrt(cov(0, 0)) * d(i, 0) + mean(0);
      d(i, 1) *= 1./::sqrt(cov(0, 0));
    }
    plotFunction(d);
  }
  if(d==2) {
    arr d(101, 2), Cov, U, V, w;
    double phi;
    uint i;
    if(cov.d0>2) { Cov=cov.sub(0, 1, 0, 1); } else { Cov.referTo(cov); }
    for(i=0; i<d.d0; i++) { //standard circle
      phi=MLR_2PI*((double)i)/(d.d0-1);
      d(i, 0)=cos(phi); d(i, 1)=sin(phi);
    }
    svd(U, w, V, Cov);
    for(i=0; i<w.N; i++) w(i)=sqrt(w(i)); //trace of eig^2 becomes N!
    for(i=0; i<d.d0; i++) { d[i]()*=w; d[i]=V*d[i]; d(i, 0)+=mean(0); d(i, 1)+=mean(1); }
    
    plotModule.s->lines.append(d);
  }
  if(d==3) {
#if 1
    arr d(303, 3), Cov, U, V, w;
    double phi;
    uint i;
    for(i=0; i<101; i++) { //standard sphere
      phi=MLR_2PI*((double)i)/(101-1);
      d(i, 0)=cos(phi); d(i, 1)=sin(phi); d(i, 2)=0.;
    }
    for(i=0; i<101; i++) {
      phi=MLR_2PI*((double)i)/(101-1);
      d(101+i, 0)=cos(phi); d(101+i, 1)=0.; d(101+i, 2)=sin(phi);
    }
    for(i=0; i<101; i++) {
      phi=MLR_2PI*((double)i)/(101-1);
      d(202+i, 0)=0.; d(202+i, 1)=cos(phi); d(202+i, 2)=sin(phi);
    }
    CHECK_EQ(cov.d0,3, "");
    //lapack_cholesky(V, cov);
    svd(U, w, V, cov);
    for(i=0; i<w.N; i++) w(i)=sqrt(w(i)); //trace of eig^2 becomes N!
    for(i=0; i<d.d0; i++) { d[i]()*=w; d[i]=V*d[i]; d[i]()+=mean; }
    d.reshape(3, 101, 3);
    plotModule.s->lines.append(d[0]);
    plotModule.s->lines.append(d[1]);
    plotModule.s->lines.append(d[2]);
#else
    arr d(101, 2), dd(101, 3), Cov, U, V, w;
    double phi;
    uint i;
    //x-y
    Cov=cov.sub(0, 1, 0, 1);
    for(i=0; i<d.d0; i++) { //standard circle
      phi=MLR_2PI*((double)i)/(d.d0-1);
      d(i, 0)=cos(phi); d(i, 1)=sin(phi);
    }
    svd(U, w, V, Cov);
    for(i=0; i<w.N; i++) w(i)=sqrt(w(i)); //trace of eig^2 becomes N!
    for(i=0; i<d.d0; i++) { mult(d[i](), d[i], w); d[i]=V*d[i]; d(i, 0)+=mean(0); d(i, 1)+=mean(1); }
    for(i=0; i<d.d0; i++) { dd(i, 0)=d(i, 0); dd(i, 1)=d(i, 1); dd(i, 2)=mean(2); }
    plotModule.s->lines.append(dd);
    //y-z
    Cov=cov.sub(1, 2, 1, 2);
    for(i=0; i<d.d0; i++) { //standard circle
      phi=MLR_2PI*((double)i)/(d.d0-1);
      d(i, 0)=cos(phi); d(i, 1)=sin(phi);
    }
    svd(U, w, V, Cov);
    for(i=0; i<w.N; i++) w(i)=sqrt(w(i)); //trace of eig^2 becomes N!
    for(i=0; i<d.d0; i++) { mult(d[i](), d[i], w); d[i]=V*d[i]; d(i, 0)+=mean(1); d(i, 1)+=mean(2); }
    for(i=0; i<d.d0; i++) { dd(i, 0)=mean(0); dd(i, 1)=d(i, 0); dd(i, 2)=d(i, 1); }
    plotModule.s->lines.append(dd);
    //x-z
    Cov(0, 0)=cov(0, 0); Cov(1, 0)=cov(2, 0); Cov(0, 1)=cov(0, 2); Cov(1, 1)=cov(2, 2);
    for(i=0; i<d.d0; i++) { //standard circle
      phi=MLR_2PI*((double)i)/(d.d0-1);
      d(i, 0)=cos(phi); d(i, 1)=sin(phi);
    }
    svd(U, w, V, Cov);
    for(i=0; i<w.N; i++) w(i)=sqrt(w(i)); //trace of eig^2 becomes N!
    for(i=0; i<d.d0; i++) { mult(d[i](), d[i], w); d[i]=V*d[i]; d(i, 0)+=mean(0); d(i, 1)+=mean(2); }
    for(i=0; i<d.d0; i++) { dd(i, 0)=d(i, 0); dd(i, 1)=mean(1); dd(i, 2)=d(i, 1); }
    plotModule.s->lines.append(dd);
#endif
  }
}

void plotVectorField(const arr& X, const arr& dX) {
  CHECK(X.nd==2 && samedim(X, dX), "");
  uint i;
  arr l(2, X.d1);
  for(i=0; i<X.d0; i++) {
    l[0]() = X[i];
    l[1]() = X[i]+dX[i];
    plotModule.s->lines.append(l);
  }
}

void plotMatrixFlow(uintA& M, double len) {
  CHECK_EQ(M.nd,2, "");
  uint i, j;
  arr X, dX;
  X.resize(M.d0, M.d1, 2);
  for(i=0; i<X.d0; i++) for(j=0; j<X.d1; j++) {
      X(i, j, 0)=-1.+(2.*j+1.)/X.d1;
      X(i, j, 1)=-1.+(2.*i+1.)/X.d0;
    }
  X.reshape(M.d0*M.d1, 2);
  dX.resize(M.d0*M.d1, 2);
  for(i=0; i<X.d0; i++) {
    dX[i]() = X[M.elem(i)]-X[i];
  }
  dX *= len;
  plotVectorField(X, dX);
  plotPoints(X);
}

#ifdef MLR_gauss_h
void plotGaussians(const GaussianA& G) {
  for(uint k=0; k<G.N; k++) { G(k).makeC(); plotCovariance(G(k).c, G(k).C); }
}
void plotGaussians(const GaussianL& G) {
  for(uint k=0; k<G.N; k++) { G(k)->makeC(); plotCovariance(G(k)->c, G(k)->C); }
}
#endif

//===========================================================================
//
// OpenGL draw routine
//

void plotDrawOpenGL(void *_data) {
#ifdef MLR_GL
  sPlotModule& data=(*((sPlotModule*)_data));
  uint a, i, j;
  
  mlr::Color c;
  
  double x=0., y=0., z=0.;
  
  //light?
  if(plotModule.light) glStandardLight(NULL);
  
  if(plotModule.drawBox) {
    glColor3f(.7, .7, .7);
    glBegin(GL_LINE_LOOP);
    glVertex3f(-1, -1, -1);
    glVertex3f(-1, 1, -1);
    glVertex3f(1, 1, -1);
    glVertex3f(1, -1, -1);
    glEnd();
    glBegin(GL_LINE_LOOP);
    glVertex3f(-1, -1, 1);
    glVertex3f(-1, 1, 1);
    glVertex3f(1, 1, 1);
    glVertex3f(1, -1, 1);
    glEnd();
    glBegin(GL_LINES);
    glVertex3f(-1, -1, -1);
    glVertex3f(-1, -1, 1);
    glVertex3f(1, -1, -1);
    glVertex3f(1, -1, 1);
    glVertex3f(-1, -1, -1);
    glVertex3f(-1, -1, 1);
    glVertex3f(1, 1, -1);
    glVertex3f(1, 1, 1);
    glVertex3f(-1, 1, -1);
    glVertex3f(-1, 1, 1);
    glEnd();
  }
  
  //draw images
  for(a=0; a<data.images.N; a++) {
  }
  
  //draw arrays
  for(a=0; a<data.array.N; a++) {
    CHECK(data.array(a).nd<=2, "can't display 3(or higher)-dim arrays");
    if(data.array(a).nd==1 || (data.array(a).nd==2 && data.array(a).d1==1)) { //1D functions
      c.setIndex(a);
      glColor(c.r, c.g, c.b);
      
      for(i=1; i<data.array(a).N; i++) {
        glBegin(GL_LINES);
        glVertex3f(2.*(i-1)/(data.array(a).N-1)-1., data.array(a).elem(i-1), 0);
        glVertex3f(2.*(i)/(data.array(a).N-1)-1., data.array(a).elem(i), 0);
        glEnd();
      }
      glBegin(GL_LINE_LOOP);
      glColor3f(0., 0., 0.);
      glVertex3f(-1, -1, 0);
      glVertex3f(-1, 1, 0);
      glVertex3f(1, 1, 0);
      glVertex3f(1, -1, 0);
      glEnd();
    }
    if(data.array(a).nd==2 && data.array(a).d1==2) { //2D path
      c.setIndex(a);
      glColor(c.r, c.g, c.b);
      glBegin(GL_LINE_STRIP);
      for(i=0; i<data.array(a).d0; i++) {
        glVertex3f(2.*i/(data.array(a).d0-1)-1., data.array(a).operator()(i, 0), data.array(a).operator()(i, 1));
      }
      glEnd();
    }
    if(data.array(a).nd==2 && data.array(a).d1>2) { //2D landscapes
      uint i, j, X=data.array(a).d1, Y=data.array(a).d0;
      c.setIndex(a);
      if(!plotModule.grid) { //as a mesh
        c.whiten(.5);
        CHECK_EQ(Y*X,data.mesh.V.d0, "you must recall display(data.array) when dimensions changed");
        for(j=0; j<Y; j++) for(i=0; i<X; i++) {
            x= 2.*(double)i/(X-1.)-1.;
            y= 2.*(double)j/(Y-1.)-1.;
            z=data.array(a)(j, i);
            c.setTemp2(z);
            data.mesh.V(j*X+i, 0)=x;    data.mesh.V(j*X+i, 1)=y;    data.mesh.V(j*X+i, 2)=z;
            data.mesh.C(j*X+i, 0)=c.r;  data.mesh.C(j*X+i, 1)=c.g;  data.mesh.C(j*X+i, 2)=c.b;
          }
        data.mesh.computeNormals();
        glDisable(GL_CULL_FACE);
        data.mesh.glDraw(NoOpenGL);
        glEnable(GL_CULL_FACE);
      } else { //as a grid
        c.blacken(.5);
        for(j=0; j<Y; j++) { //along the x-axis
          glBegin(GL_LINE_STRIP);
          for(i=0; i<X; i++) {
            x= 2.*(double)i/(X-1.)-1.;
            y=-2.*(double)j/(Y-1.)+1.;
            z=data.array(a)(j, i);
            //c.setTemp2(z);
            glColor3f(c.r, c.g, c.b);
            glColor(c.r, c.g, c.b);
            glVertex3f(x, y, z);
          }
          glEnd();
        }
        for(i=0; i<X; i++) { //along the y-axis
          glBegin(GL_LINE_STRIP);
          for(j=0; j<Y; j++) {
            x= 2.*(double)i/(X-1.)-1.;
            y=-2.*(double)j/(Y-1.)+1.;
            z=data.array(a)(j, i);
            //c.setTemp2(z);
            glColor3f(c.r, c.g, c.b);
            glColor(c.r, c.g, c.b);
            glVertex3f(x, y, z);
          }
          glEnd();
        }
      }
    }
  }
  
  //draw points
  for(i=0; i<data.points.N; i++) {
    c.setIndex(i);
    glColor(c.r, c.g, c.b);
    //glBegin(GL_LINES);
    if(plotModule.drawDots) glBegin(GL_POINTS);
    if(data.points(i).nd==2) {
      for(j=0; j<data.points(i).d0; j++) {
        if(data.points(i).d1==1) { x=(double)j; y=data.points(i)(j, 0); z=0.; }
        if(data.points(i).d1==2) { x=data.points(i)(j, 0); y=data.points(i)(j, 1); z=1.; }
        if(data.points(i).d1>=3) { x=data.points(i)(j, 0); y=data.points(i)(j, 1); z=data.points(i)(j, 2); }
        if(!plotModule.drawDots) {
          glPushMatrix();
          glTranslatef(x, y, z);
          glDrawDiamond(.01, .01, .01);
          glPopMatrix();
        } else {
          glVertex3d(x, y, z);
        }
      }
    } else {
      if(data.points(i).d0==1) { x=data.points(i)(0); y=0.; z=0.; }
      if(data.points(i).d0==2) { x=data.points(i)(0); y=data.points(i)(1); z=0.; }
      if(data.points(i).d0>=3) { x=data.points(i)(0); y=data.points(i)(1); z=data.points(i)(2); }
      if(!plotModule.drawDots) {
        glPushMatrix();
        glTranslatef(x, y, z);
        glDrawDiamond(.02, .02, .02);
        glPopMatrix();
      } else {
        glVertex3d(x, y, z);
      }
    }
    if(plotModule.drawDots) glEnd();
  }
  
  //draw lines
  for(i=0; i<data.lines.N; i++) {
    if(plotModule.colors) c.setIndex(i); else c.setIndex(0);
    glColor(c.r, c.g, c.b);
    
    if(plotModule.thickLines) {
      glLineWidth(plotModule.thickLines);
    }
    
    glBegin(GL_LINE_STRIP);
    for(j=0; j<data.lines(i).d0; j++) {
      if(data.lines(i).d1==1) glVertex3d((double)j, data.lines(i)(j, 0), 0.);
      if(data.lines(i).d1==2) glVertex3d(data.lines(i)(j, 0), data.lines(i)(j, 1), 1.);
      if(data.lines(i).d1>=3) glVertex3d(data.lines(i)(j, 0), data.lines(i)(j, 1), data.lines(i)(j, 2));
    }
    glEnd();
  }
  
  //draw planes
  for(i=0; i<data.planes.N; i+=4) {
    c.setIndex(i/4+1);
    glColor(c.r, c.g, c.b);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glBegin(GL_POLYGON);
    glVertex3f(data.planes(i).x, data.planes(i).y, data.planes(i).z);
    glVertex3f(data.planes(i+1).x, data.planes(i+1).y, data.planes(i+1).z);
    glVertex3f(data.planes(i+2).x, data.planes(i+2).y, data.planes(i+2).z);
    glVertex3f(data.planes(i+3).x, data.planes(i+3).y, data.planes(i+3).z);
    glEnd();
  }
#else
  NIY;
#endif
}


//===========================================================================
//
// gnuplot draw routine
//

#define PLOTEVERY(block, with)  gnuplotcmd \
      <<"'z.plotdata' every :::" <<(block) <<"::" <<(block) <<(with);

void plotDrawGnuplot(void *_data, bool pauseMouse) {
  sPlotModule& data=(*((sPlotModule*)_data));
  uint i;
  
  //openfiles
  mlr::String gnuplotcmd;
  std::ofstream gnuplotdata;
  mlr::open(gnuplotdata, "z.plotdata");
  uint block=0;
  
  // include custom definition file if exists
  FILE *incf = fopen("z.plotcmd.inc", "r");
  if(incf) { fclose(incf);  gnuplotcmd <<"load 'z.plotcmd.inc'\n";}
  
  //gnuplotcmd <<"set size square\n";
  //if(wait) gnuplotcmd <<"set title 'CLICK LEFT TO CONTINUE'\n";
  
  if(data.lines.N+data.points.N) gnuplotcmd <<"\nplot \\\n";
  
  //pipe data
  bool ior=mlr::IOraw;
  mlr::IOraw=true;
  //lines
  for(i=0; i<data.lines.N; i++) {
    data.lines(i).write(gnuplotdata," ","\n","  ",false,false);
    gnuplotdata <<'\n' <<std::endl;
    if(block) gnuplotcmd <<", \\\n";
    if(data.lines(i).d1!=4) {
      PLOTEVERY(block, " with l notitle");
    } else { //with filled error curves
      PLOTEVERY(block,
                " using 1:2:3 with filledcurves fill solid 0.4 lc rgb 'yellow' notitle, \\\n ");
      PLOTEVERY(block,
                " using 1:2:4 with filledcurves fill solid 0.4 lc rgb 'yellow' notitle, \\\n ");
      PLOTEVERY(block, " using 1:2 with l lc rgb 'green' notitle");
    }
    block++;
  }

  //points
  for(i=0; i<data.points.N; i++) {
    data.points(i).write(gnuplotdata," ","\n","  ",false,false);
    gnuplotdata <<'\n' <<std::endl;
    if(block) gnuplotcmd <<", \\\n";
    mlr::String a=" with p pt 3";
    if(i<data.legend.N) a<< " title '" <<data.legend(i) <<"' ";
    PLOTEVERY(block, a);
    block++;
  }
  
  if(data.array.N) gnuplotcmd <<"\n\npause mouse\nset dgrid3d\n\nsplot \\\n";
  
  //surfaces
  for(i=0; i<data.array.N; i++) {
    uint j, k, X=data.array(i).d1, Y=data.array(i).d0;
    for(j=0; j<Y; j++){
      for(k=0; k<X; k++) {
        gnuplotdata <<2.*(double)k/(X-1.)-1. <<' ' <<-2.*(double)j/(Y-1.)+1. <<' ' <<data.array(i)(j, k) <<std::endl;
      }
    }
    gnuplotdata <<std::endl;
    if(i && block) gnuplotcmd <<", \\\n";
    PLOTEVERY(block, " with l notitle");
    block++;
  }
  mlr::IOraw=ior;
  gnuplotcmd <<endl;
  
  //close files
  gnuplotdata.close();
  
  //call gnuplot
  gnuplot(gnuplotcmd, pauseMouse, false, "z.pdf");
}



/*
double lo[3], hi[3];
sPlotModule(){
lo[0]=lo[1]=lo[2]= 0.;
hi[0]=hi[1]=hi[2]= 1.;
}
void setRange(double xl, double xh, double yl=-1., double yh=1., double zl=-1., double zh=1.){
lo[0]=xl; hi[0]=xh;
lo[1]=yl; hi[1]=yh;
lo[2]=zl; hi[2]=zh;
}

 void transBackPoint(double &x, double &y){
 x=(hi[0]-lo[0])*(x+1.)/2. + lo[0];
 y=(hi[1]-lo[1])*(y+1.)/2. + lo[1];
 }
*/
/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#include "eigenValues.h"

void ExtremeEigenValues::computeExact(){
  arr lambda, x;
  lapack_EigenDecomp(A, lambda, x);
  lambda_lo = lambda(0);      if(lambda_lo>1e-10) x_lo = x[0];
  lambda_hi = lambda.last();  if(lambda_hi>1e-10) x_hi = x[x.d0-1];
}


void ExtremeEigenValues::initPowerMethodRandom(){
  x_hi = 2.*rand(A.d0)-1.;  x_hi/=length(x_hi);
  x_lo = 2.*rand(A.d0)-1.;  x_lo/=length(x_lo);
}


void ExtremeEigenValues::stepPowerMethod(uint k){
  for(uint i=0;i<k;i++){
    x_hi = A*x_hi;
    lambda_hi=length(x_hi);
    x_hi /= lambda_hi;

    x_lo = (lambda_hi*eye(A.d0) - A) * x_lo;
    lambda_lo=length(x_lo);
    x_lo /= lambda_lo;
    lambda_lo = lambda_hi - lambda_lo;
  }
}
/*  ---------------------------------------------------------------------
    Copyright 2014 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */


#include "cma.h"
#include "algos.h"




CMA::CMA() {




}
CMA::~CMA() {

}

void CMA::run(){
    //initialize
    arr pc,ps;
    m.resize(dim);
    pc.resize(dim);
    ps.resize(dim);
    m.setZero();
    pc.setZero();
    ps.setZero();
    C = eye(dim);


}

/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */



/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#include "dataNeighbored.h"

void DataNeighbored::setData(const arr& pts){
  X = pts;
  valid.resize(X.d0);
  for(uint i=0;i<X.d0;i++) if(pts(i,2)>=0) valid(i)=true; else valid(i)=false;
  if(isModelledWeights.N!=X.d0){
    isModelledWeights.resize(X.d0);
    isModelledWeights.setZero();
  }
  N.clear();
  idx2pixel.setStraightPerm(X.d0);
}

void DataNeighbored::setCosts(const arr& _costs){
  costs = _costs;
  CHECK_EQ(costs.N, X.d0, "");
}

uint DataNeighbored::n() const{ return X.d0; }

uint DataNeighbored::d() const{ return X.d1; }

void DataNeighbored::setGridNeighborhood(uint height, uint width, bool excludeNonValids){
  CHECK_EQ(width*height, X.d0, "");
  N.resize(X.d0);
  for(uint y=0;y<height;y++) for(uint x=0;x<width;x++){
    uint i=y*width + x, j;
    if(excludeNonValids && !valid(i)) continue;
    if(y){          j=(y-1)*width+(x  ); if(!excludeNonValids || valid(j)) N(i).append(j); }
    if(x){          j=(y  )*width+(x-1); if(!excludeNonValids || valid(j)) N(i).append(j); }
    if(y<height-1){ j=(y+1)*width+(x  ); if(!excludeNonValids || valid(j)) N(i).append(j); }
    if(x<width-1){  j=(y  )*width+(x+1); if(!excludeNonValids || valid(j)) N(i).append(j); }
  }
}

void DataNeighbored::removeNonValid(){
  uintA index(X.d0);
  index = X.d0;
  int s=0;
  for(uint i=0;i<X.d0;i++) if(valid(i)){ index(i)=s; s++; } //assign new indeces to each point
  idx2pixel.resize(s);
  for(uint i=0;i<X.d0;i++) if(valid(i)){
    uintA& Ni = N(i);
    for(uint& j:Ni) j=index(j); //use new indices in neighborhoods
    Ni.sort();                  //sort neighborhoods
    while(Ni.N && Ni.last()==X.d0) Ni.resizeCopy(Ni.N-1); //remove those, pointing to !ok (==X.d0 index)
  }
  for(uint i=0;i<X.d0;i++) if(valid(i)){
    if(index(i)!=i){
      X[index(i)] = X[i];
      N(index(i)) = N(i);
      isModelledWeights(index(i)) = isModelledWeights(i);
      costs(index(i)) = costs(i);
    }
    idx2pixel(index(i)) = i;
  }
  X.resizeCopy(s,X.d1);
  N.resizeCopy(s);
  isModelledWeights.resizeCopy(s);
  costs.resizeCopy(s);
}

void DataNeighbored::initFringe(uintA& fringe, uintA& pts, boolA& included, uint i){
  CHECK(valid(i),"");
  fringe.clear();
  fringe.append(i);
  pts = fringe;
  included.resize(X.d0);
  included.setZero();
  included(i) = true;
}

void DataNeighbored::expandFringe(uintA& fringe, uintA& pts, boolA& included){
  uintA newfringe;
  for(uint i:fringe) for(uint j:N(i)){
    if(valid(j) && !included(j)){
      newfringe.append(j);
      pts.append(j);
      included(j)=true;
    }
  }
  fringe = newfringe;
}

uintA DataNeighbored::getKneighborhood(uint i, uint k){
  CHECK(valid(i),"");
  uintA fringe, pts;
  boolA included;
  initFringe(fringe, pts, included, i);
  uintA Nk;

  for(;fringe.N;){
    if(Nk.N+fringe.N<=k){
      Nk.append(fringe);
      if(Nk.N==k) return Nk;
    }else for(uint j:fringe){
      Nk.append(j);
      if(Nk.N==k) return Nk;
    }

    expandFringe(fringe, pts, included);
  }
  return Nk;
}
/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */

#include "gaussianProcess.h"
#include <Plot/plot.h>

void plotBelief(GaussianProcess& gp, double lo, double hi, bool pause){
  arr X, Y, Z, S;
  uint dim;
  //there should be at least 1 observation to guess the dimensionality from
  dim = gp.X.d1 ? gp.X.d1 : gp.dX.d1;
  CHECK(dim > 0, "still no data here. I have no clue about dimensionality!?!");
  
  X.setGrid(dim, lo, hi, 100);
  gp.evaluate(X, Y, S);
  plotClear();
  switch(dim){
    case 1:
      plotFunctionPrecision(X, Y, Y+S, Y-S);
      //plotFunction(X, Y);
      //plotFunction(X, Y+S);
      //plotFunction(X, Y-S);
      plotPoints(gp.X, gp.Y);
      plotPoints(gp.dX, gp.dY);
      break;
    case 2:
      //plotFunction(X, Y);
      //plotFunction(X, Y+S);
      //plotFunction(X, Y-S);
      plotPoints(gp.X, gp.Y);
      plotPoints(gp.dX, gp.dY);
      break;
    default :
      HALT("Space is either 0- or higher than 3-dimensional. Tell me how to plot that!")
      break;
  }
  plot(pause);
}

void plotKernel1D(GaussianProcess& gp, double lo, double hi, bool pause){
  arr X, K, KD1, KD2;
  X.setGrid(1, lo, hi, 600);
  K.resize(X.d0);
  KD1.resize(X.d0);
  KD2.resize(X.d0);
  arr null=ARR(0.);
  for(uint i=0; i<X.d0; i++){
    K(i) = gp.cov(gp.kernelP, null, X[i]);
    KD1(i) = gp.covF_D(0, gp.kernelP, null, X[i]);
    KD2(i) = gp.covDD_F(0, 0, gp.kernelP, X[i], null);
  }
  plotClear();
  plotFunction(X, K);
  plotFunction(X, KD1);
  plotFunction(X, KD2);
  plot(pause);
}

void plotKernel2D(GaussianProcess& gp, double lo, double hi, bool pause){
  arr X, K, KD1, KD2;
  X.setGrid(2, lo, hi, 1000);
  K.resize(X.d0, X.d1);
  KD1.resize(X.d0, X.d1);
  KD2.resize(X.d0, X.d1);
  arr null=ARR(0.);
  for(uint i=0; i<X.d0; i++){
    for(uint j=0; j<X.d1; j++){
      K(i, j) = gp.cov(gp.kernelP, null, X[i]);
      KD1(i, j) = gp.covF_D(0, gp.kernelP, null, X[i]);
      KD2(i, j) = gp.covDD_F(0, 0, gp.kernelP, X[i], null);
    }
  }
  plotClear();
  plotSurface(K);
  plotSurface(KD1);
  plotSurface(KD2);
  plot(pause);
}
/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */

#include "rrt.h"

#include <Algo/ann.h>

struct sRRT {
  ANN ann;
  uintA parent;
  double stepsize;
  uint nearest;
};

RRT::RRT(const arr& q0, double _stepsize) : s(new sRRT()){
  s->ann   .append(q0); //append q as the root of the tree
  s->parent.append(0);    //q has itself as parent
  s->stepsize = _stepsize;
}
double RRT::getProposalTowards(arr& proposal, const arr& q){
  //find NN
  s->nearest=s->ann.getNN(q);

  //compute little step
  arr d = q - s->ann.X[s->nearest]; //difference vector between q and nearest neighbor
  double dist = length(d);
  if (dist > s->stepsize)
    proposal = s->ann.X[s->nearest] + s->stepsize/dist * d;
  else
    proposal = q;
  return dist;
}
void RRT::add(const arr& q){
  s->ann.append(q);
  s->parent.append(s->nearest);
}

//some access routines
double RRT::getStepsize() { return s->stepsize; }
uint RRT::getNearest(){ return s->nearest; }
uint RRT::getParent(uint i){ return s->parent(i); }
uint RRT::getNumberNodes(){ return s->ann.X.d0; }
arr RRT::getNode(uint i){ return s->ann.X[i]; }
void RRT::getRandomNode(arr& q){ q = s->ann.X[rnd(s->ann.X.d0)]; }
arr RRT::getRandomNode() { return s->ann.X[rnd(s->ann.X.d0)]; }

/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */

#include <Core/util.h>
#include "MLcourse.h"

arr beta_true;

double NormalSdv(const double& a, const double& b, double sdv) {
  double d=(a-b)/sdv;
  double norm = 1./(::sqrt(MLR_2PI)*sdv);
  return norm*::exp(-.5*d*d);
}

//===========================================================================

arr ridgeRegression(const arr& X, const arr& y, double lambda, arr& bayesSigma, const arr &weighted, arr &zScores) {
  if(lambda<0.) lambda = mlr::getParameter<double>("lambda",1e-10);

  CHECK((y.nd==1 || y.nd==2) && X.nd==2 && y.d0==X.d0, "wrong dimensions");
  arr Xt = ~X;
  if(&weighted) Xt = Xt % weighted;
  arr XtX = Xt*X;
  for(uint i=1;i<XtX.d0;i++) XtX(i,i) += lambda;
  XtX(0, 0) += 1e-10; //don't regularize beta_0 !!
  arr beta = lapack_Ainv_b_sym(XtX, Xt*y);
  if(&bayesSigma){
    lapack_inverseSymPosDef(bayesSigma, XtX);
    double sigma2 = sqrt(sumOfSqr(X*beta-y)/(X.d0-1));
    bayesSigma *= sigma2;
  }
  if(&zScores) {
    zScores.resize(beta.N);
    double sigma = sumOfSqr(X*beta-y)/(y.N - X.d1 - 1.);
    arr XtXinv;
    lapack_inverseSymPosDef(XtXinv, XtX);
    for(uint i=0; i<beta.N; i++) {
      zScores(i) = fabs(beta(i)) / (sigma * sqrt(XtXinv(i, i)));
    }
  }
  return beta;
}

//===========================================================================

arr evaluateBayesianRidgeRegressionSigma(const arr& X, const arr& bayesSigma){
  arr s(X.d0);
  for(uint i=0;i<s.N;i++) s.elem(i) = (~X[i] * bayesSigma * X[i]).scalar();
  return s;
}

//===========================================================================

RidgeRegression::RidgeRegression(const arr& X, const arr& y, double lambda, const arr& weighted, int verbose){
  if(lambda<0.) lambda = mlr::getParameter<double>("lambda",1e-10);
  CHECK((y.nd==1 || y.nd==2) && X.nd==2 && y.d0==X.d0, "wrong dimensions");

  arr Xt = ~X;
  if(&weighted) Xt = Xt % weighted;
  XtX_I = Xt*X;
  for(uint i=1;i<XtX_I.d0;i++) XtX_I(i,i) += lambda;
  XtX_I(0, 0) += 1e-10; //don't regularize beta_0 !!
  beta = lapack_Ainv_b_sym(XtX_I, Xt*y);
  sigmaSqr = sumOfSqr(X*beta-y)/double(y.N/*-beta.N*/); //beta.N are the degrees of freedom that we substract (=1 for const model)
  if(verbose>0){
    cout <<"Ridge Regression: #data=" <<X.d0 <<" #features=" <<X.d1 <<" #outputs=" <<(y.nd==2?y.d1:1) <<endl;
    cout <<"   mean error (sdv)=" <<sqrt(sigmaSqr) <<endl;
    if(y.nd==2)
      cout <<"   multi-output mean errors (sdv)=" <<sqrt(getMultiOutputSquaredErrors(X, y)) <<endl;
  }
}

arr RidgeRegression::getBetaSigmaMatrix(){
  lapack_inverseSymPosDef(betaSigmaMatrix, XtX_I);
  betaSigmaMatrix *= sigmaSqr;
  return betaSigmaMatrix;
}

arr RidgeRegression::getBetaZscores() {
  arr zScores(beta.N);
  arr betaSigmaMatrix = getBetaSigmaMatrix();
  for(uint i=0; i<beta.N; i++) zScores(i) = fabs(beta(i)) / ::sqrt(betaSigmaMatrix(i,i));
  return zScores;
}

arr RidgeRegression::getMultiOutputSquaredErrors(const arr& X, const arr& y){
  arr err = X*beta-y;
  err *= err; //elem-wise
  return sum(err,0)/double(X.d0/*-X.d1*/); //beta.N are the degrees of freedom that we substract (=1 for const model)
}

arr RidgeRegression::evaluate(const arr& X, arr& bayesSigma2){
  if(&bayesSigma2){
    bayesSigma2.resize(X.d0);
    if(!betaSigmaMatrix.N) getBetaSigmaMatrix();
    for(uint i=0;i<X.d0;i++) bayesSigma2(i) = (~X[i] * betaSigmaMatrix * X[i]).scalar();
  }
  return X*beta;
}

//===========================================================================

double DefaultKernelFunction::k(const arr& x1, const arr& x2, arr& gx1, arr& Hx1){
  if(!type){
    type = (KernelType) mlr::getParameter<uint>("ML/KernelType",1);
    switch(type){
      case readFromCfg: HALT("???");  break;
      case Gauss: hyperParam1 = ARR( mlr::sqr(mlr::getParameter<double>("ML/KernelWidth")) );  break;
    }
  }
  double k = hyperParam2.scalar()*::exp(-sqrDistance(x1,x2)/hyperParam1.scalar());
  double a = -2.*k/hyperParam1.scalar();
  if(&gx1) gx1 = a * (x1-x2);
  if(&Hx1) Hx1 = (-2.*a/hyperParam1.scalar())*((x1-x2)^(x1-x2)) + a*eye(x1.N);
  return k;
};
DefaultKernelFunction defaultKernelFunction;

//===========================================================================

KernelRidgeRegression::KernelRidgeRegression(const arr& X, const arr& y, KernelFunction& kernel, double lambda, double mu)
  :X(X),mu(mu),kernel(kernel){
  if(lambda<0.) lambda = mlr::getParameter<double>("lambda",1e-10);
  uint n=X.d0;

  //-- compute kernel matrix
  arr kernelMatrix(n,n);
  for(uint i=0;i<n;i++) for(uint j=0;j<i;j++){
    kernelMatrix(i,j) = kernelMatrix(j,i) = kernel.k(X[i],X[j]);
  }
  for(uint i=0;i<n;i++) kernelMatrix(i,i) = kernel.k(X[i],X[i]);

  kernelMatrix_lambda=kernelMatrix;
  for(uint i=0;i<n;i++) kernelMatrix_lambda(i,i) += lambda;

  //-- compute alpha
  alpha = lapack_Ainv_b_sym(kernelMatrix_lambda, y-mu);

  sigmaSqr = sumOfSqr(kernelMatrix*alpha-y)/double(y.N/*-beta.N*/); //beta.N are the degrees of freedom that we substract (=1 for const model)
}

arr KernelRidgeRegression::evaluate(const arr& Z, arr& bayesSigma2){
  arr kappa(Z.d0,X.d0);
  for(uint i=0;i<Z.d0;i++) for(uint j=0;j<X.d0;j++) kappa(i,j) = kernel.k(Z[i],X[j]);
  if(&bayesSigma2){
    if(!invKernelMatrix_lambda.N) invKernelMatrix_lambda = inverse_SymPosDef(kernelMatrix_lambda);
    bayesSigma2.resize(Z.d0);
    for(uint i=0;i<Z.d0;i++){
      bayesSigma2(i) = kernel.k(Z[i],Z[i]);
      bayesSigma2(i) -= scalarProduct(kappa[i], invKernelMatrix_lambda*kappa[i]);
    }
  }
  return mu + kappa * alpha;
}

double KernelRidgeRegression::evaluate(const arr& x, arr& g, arr& H, double plusSigma, bool onlySigma){
  arr kappa(X.d0);
  arr Jkappa(X.d0, x.N);
  arr Hkappa(X.d0, x.N, x.N);
  for(uint i=0;i<X.d0;i++) kappa(i) = kernel.k(x, X[i], Jkappa[i](), Hkappa[i]());

  double fx = 0.;
  if(&g) g = zeros(x.N);
  if(&H) H = zeros(x.N, x.N);

  if(!onlySigma){
    fx += mu + scalarProduct(alpha, kappa);
    if(&g) g += ~alpha * Jkappa;
    if(&H) H += ~alpha * Hkappa;
  }

  if(plusSigma){
//    arr gx, Hx;
//    fx += plusSigma * ;
////    if(&g) g += plusSigma*(gx + g2);
////    if(&H) H += plusSigma*(gx + g2);

    if(!invKernelMatrix_lambda.N) invKernelMatrix_lambda = inverse_SymPosDef(kernelMatrix_lambda);

    arr Kinv_k = invKernelMatrix_lambda*kappa;
    arr J_Kinv_k = ~Jkappa*Kinv_k;
    double k_Kinv_k = kernel.k(x, x) - scalarProduct(kappa, Kinv_k);
    fx += plusSigma * ::sqrt(k_Kinv_k);
    if(&g) g -= (plusSigma/sqrt(k_Kinv_k)) * J_Kinv_k;
    if(&H) H -= (plusSigma/(k_Kinv_k*sqrt(k_Kinv_k))) * (J_Kinv_k^J_Kinv_k) + (plusSigma/sqrt(k_Kinv_k)) * (~Jkappa*invKernelMatrix_lambda*Jkappa + ~Kinv_k*Hkappa);
  }

  return fx;
}

ScalarFunction KernelRidgeRegression::getF(double plusSigma){
  return [this,plusSigma](arr& g, arr& H, const arr& x) -> double{
    return this->evaluate(x, g, H, plusSigma, false);
  };
}

//===========================================================================

KernelLogisticRegression::KernelLogisticRegression(const arr& X, const arr& y, KernelFunction& _kernel, double _lambda, double _mu)
  :X(X),lambda(_lambda),mu(_mu),kernel(_kernel){
  if(lambda<0.) lambda = mlr::getParameter<double>("lambda",1e-10);
  uint n=X.d0;

  //-- compute kernel matrix
  arr kernelMatrix(n,n);
  for(uint i=0;i<n;i++) for(uint j=0;j<i;j++){
    kernelMatrix(i,j) = kernelMatrix(j,i) = kernel.k(X[i],X[j]);
  }
  for(uint i=0;i<n;i++) kernelMatrix(i,i) = kernel.k(X[i],X[i]);

  //-- iterate Newton steps on training data
  arr f(n), p(n), Z(n), w(n);
  double logLike;
  f.setUni(mu);
  for(uint k=0; k<100; k++) {
    p = exp(f);
    Z = 1.+p;
    p /= Z;
    w = p % (1.-p);

    //compute logLikelihood
    logLike=0.;
    for(uint i=0; i<n; i++) logLike += mlr::indicate(y(i)==1.)*f(i) - log(Z(i));
    LOG(1) <<"log-likelihood = " <<logLike;

    kernelMatrix_lambda = kernelMatrix;
    for(uint i=0;i<n;i++) kernelMatrix_lambda(i,i) += 2.*lambda/w(i);

    //compute the update
    arr f_old=f;
    alpha = lapack_Ainv_b_sym(kernelMatrix_lambda, f - (p-y)/w - mu);
    f = mu + kernelMatrix * alpha;
    for(uint i=0; i<f.N; i++) clip(f.elem(i), -100., 100.);  //constrain the discriminative values to avoid NANs...

    if(maxDiff(f,f_old)<1e-5) break;
  }
}

arr KernelLogisticRegression::evaluateF(const arr& Z, arr& bayesSigma2){
  arr kappa(Z.d0,X.d0);
  for(uint i=0;i<Z.d0;i++) for(uint j=0;j<X.d0;j++) kappa(i,j) = kernel.k(Z[i],X[j]);
  if(&bayesSigma2){
    if(!invKernelMatrix_lambda.N) invKernelMatrix_lambda = inverse_SymPosDef(kernelMatrix_lambda);
    bayesSigma2.resize(Z.d0);
    for(uint i=0;i<Z.d0;i++){
      bayesSigma2(i) = kernel.k(Z[i],Z[i]);
      bayesSigma2(i) -= scalarProduct(kappa[i], invKernelMatrix_lambda*kappa[i]);
    }
  }
  return mu + kappa * alpha;
}

arr KernelLogisticRegression::evaluate(const arr& Z, arr& p_bayes, arr &p_hi, arr &p_lo){
  arr kappa(Z.d0,X.d0);
  for(uint i=0;i<Z.d0;i++) for(uint j=0;j<X.d0;j++) kappa(i,j) = kernel.k(Z[i],X[j]);
  arr f = mu + kappa * alpha;
  for(uint i=0; i<f.N; i++) clip(f.elem(i), -100., 100.);  //constrain the discriminative values to avoid NANs...
  arr p = exp(f); p/=1.+p;

  if(&p_bayes || &p_hi || &p_lo){ //take sigma of discriminative function to estimate p_bayes, p_up and p_lo
    if(!invKernelMatrix_lambda.N) invKernelMatrix_lambda = inverse_SymPosDef(kernelMatrix_lambda);
    arr s(Z.d0);
    for(uint i=0;i<Z.d0;i++){
      s(i) = kernel.k(Z[i],Z[i]);
      s(i) -= scalarProduct(kappa[i], invKernelMatrix_lambda*kappa[i]);
    }
    s /= 2.*lambda; //TODO: why?? why not for KRR?
    for(uint i=0; i<s.N; i++) clip(s.elem(i), -100., 100.);  //constrain the discriminative values to avoid NANs...
    if(&p_bayes){ p_bayes = exp(f/sqrt(1.+s*MLR_PI/8.)); p_bayes /= 1.+p_bayes; }
    s = sqrt(s);
    if(&p_hi){ p_hi = exp(f+s); p_hi /= 1.+p_hi; }
    if(&p_lo){ p_lo = exp(f-s); p_lo /= 1.+p_lo; }
  }
  return p;
}

//===========================================================================

arr logisticRegression2Class(const arr& X, const arr& y, double lambda, arr& bayesSigma2) {
  if(lambda<0.) lambda = mlr::getParameter<double>("lambda",1e-10);

  CHECK_EQ(y.nd,1, "");
  uint n=y.N, d=X.d1;
  arr Xt;
  transpose(Xt, X);
  
  arr I;
  I.setDiag(lambda, X.d1);
  //I(0, 0)=1e-10; on classification is makes sense to include the bias in regularization, I think... (rescaling one beta only changes the slope of the sigmoid, not the decision boundary)
  
  arr f(n), p(n), Z(n), w(n), beta_update;
  double logLike, lastLogLike=0., alpha=1.;
  arr beta(d);
  beta.setZero();
  for(uint k=0; k<100; k++) {
    f = X*beta;
    for(uint i=0; i<f.N; i++) clip(f.elem(i), -100., 100.);  //constrain the discriminative values to avoid NANs...
    p = exp(f);
    Z = 1.+p;
    for(uint i=0; i<n; i++) p(i) /= Z(i);
    w = p % (1.-p);

    //compute logLikelihood
    logLike=0.;
    for(uint i=0; i<n; i++) logLike += mlr::indicate(y(i)==1.)*f(i) - log(Z(i));
    LOG(1) <<"log-likelihood = " <<logLike;

    //optionally reject the update
    if(k && logLike<lastLogLike) {
      //cout <<"REJECT" <<endl;
      beta -= alpha*beta_update;
      alpha *= .1;
      beta += alpha*beta_update;
      if(alpha*absMax(beta_update)<1e-5) break;
      continue;
    } else {
      alpha = pow(alpha, .8);
    }
    lastLogLike=logLike;

    beta_update = lapack_Ainv_b_sym(Xt*(w%X) + 2.*I, Xt*(y-p) - 2.*I*beta);   //beta update equation
    beta += alpha*beta_update;
    
//    MLR_MSG("logReg iter= " <<k <<" negLogLike= " <<-logLike/n <<" beta_update= " <<absMax(beta_update) <<" alpha= " <<alpha);
    
    if(alpha*absMax(beta_update)<1e-5) break;
  }
  if(&bayesSigma2){
    lapack_inverseSymPosDef(bayesSigma2, Xt*(w%X) + 2.*I);
  }
  return beta;
}

arr logisticRegressionMultiClass(const arr& X, const arr& y, double lambda) {
  if(lambda<0.) lambda = mlr::getParameter<double>("lambda",1e-10);


  CHECK(y.nd==2 && y.d0==X.d0, "");
  uint n=y.d0, d=X.d1, M=y.d1;
  arr Xt;
  transpose(Xt, X);
  
  arr XtWX, I;
  I.setDiag(lambda, X.d1);
  I(0, 0)=1e-10;
  
  arr f(n, M), p(n, M), Z(n), w(n), beta_update;
  double logLike, lastLogLike=0., alpha=1.;
  arr beta(d, M);
  beta.setZero();
  for(uint k=0; k<100; k++) {
    f = X*beta;
    for(uint i=0; i<f.N; i++) clip(f.elem(i), -100., 100.);  //constrain the discriminative values to avoid NANs...
    p = exp(f);
    Z = sum(p,1);
    for(uint i=0; i<n; i++) p[i]() /= Z(i);
//    w = p % (1.-p);

    //compute logLikelihood
    logLike=0.;
    for(uint i=0; i<n; i++) logLike += scalarProduct(f[i],y[i]) - log(Z(i));


    logLike=0.;
    for(uint i=0; i<n; i++) {
      p[i]() /= sum(p[i]); //normalize the exp(f(x)) along each row
      for(uint c=0; c<M; c++) logLike += y(i, c)*log(p(i, c));
    }
    
    //optionally reject the update
    if(k && logLike<lastLogLike) {
      //cout <<"REJECT" <<endl;
      beta -= alpha*beta_update;
      alpha *= .1;
      beta += alpha*beta_update;
      if(alpha*absMax(beta_update)<1e-5) break;
      continue;
    } else {
      alpha = pow(alpha, .8);
    }
    lastLogLike=logLike;
    
    //construct the Hessian matrix as block matrix of size d*M-times-d*M (the beta is of size d*M)
    XtWX.resize(beta.N, beta.N);
    XtWX.setZero();
    for(uint c1=0; c1<M; c1++) for(uint c2=0; c2<M; c2++) {
        for(uint i=0; i<n; i++) w(i) = p(i, c1)*(mlr::indicate(c1==c2)-p(i, c2));
        XtWX.setMatrixBlock(Xt*diagProduct(w, X) + 2.*mlr::indicate(c1==c2)*I, c1*d, c2*d);
      }
    //compute the beta update
    arr tmp = ~(Xt*(y-p) - 2.*I*beta); //the gradient as M-times-d matrix
    tmp.reshape(M*d);                  //... as one big vector
    beta_update = lapack_Ainv_b_sym(XtWX, tmp); //multiply the inv Hessian
    beta_update.reshape(M, d);         //... as M-times-d matrix
    beta_update = ~beta_update;        //... and back as d-times-M matrix
    
    beta += alpha*beta_update;
    
    cout <<"logReg iter= " <<k <<" logLike= " <<logLike/n <<" beta_update= " <<absMax(beta_update) <<" alpha= " <<alpha <<endl;
    if(alpha*absMax(beta_update)<1e-5) break;
  }
  return beta;
}

void CrossValidation::crossValidateSingleLambda(const arr& X, const arr& y, double lambda, uint k_fold, bool permute, arr* beta_k_fold, arr* beta_total, double *scoreMean, double *scoreSDV, double *scoreTrain) {
  arr Xtrain, Xtest, ytrain, ytest;
  uint n=X.d0;

  //permute data?
  arr X_perm, y_perm;
  if(permute){
    uintA perm;
    perm.setRandomPerm(X.d0);
    X_perm=X;  X_perm.permuteRows(perm);
    y_perm=y;  if(y.nd==2) y_perm.permuteRows(perm); else y_perm.permute(perm);
  }
  //initialize
  double cost, costM=0., costD=0.;
  arr beta;
  if(beta_k_fold) beta_k_fold->clear();
  
  //determine blocks
  CHECK(n>=k_fold,"we need at least as much data as k for k-fold CV");
  uintA blockStart(k_fold+1);
  for(uint k=0;k<=k_fold;k++) blockStart(k) = (k*n)/k_fold;
  
  //go
  for(uint k=0; k<k_fold; k++){
    if(!permute){
      Xtrain = X;  ytrain = y;
    } else {
      Xtrain = X_perm;  ytrain = y_perm;
    }
    Xtrain.delRows(blockStart(k), blockStart(k+1)-blockStart(k));
    if(ytrain.nd==2)
      ytrain.delRows(blockStart(k), blockStart(k+1)-blockStart(k));
    else
      ytrain.remove(blockStart(k), blockStart(k+1)-blockStart(k));
    Xtest.referToRange(X, blockStart(k), blockStart(k+1)-1);
    ytest.referToRange(y, blockStart(k), blockStart(k+1)-1);
    
    if(verbose) cout <<k <<": train:";
    train(Xtrain, ytrain, lambda, beta);
    if(beta_k_fold) beta_k_fold->append(beta);
    
    cost = test(Xtest, ytest, beta);
    costM += cost;
    costD += cost*cost;
    if(verbose) cout <<" test: " <<cost <<endl;
  }
  if(beta_k_fold) beta_k_fold->reshape(k_fold,beta.N);
  
  costM /= k_fold;
  costD /= k_fold;
  costD -= costM*costM;
  costD = sqrt(costD)/sqrt((double)k_fold); //sdv of the mean estimator
  
  //on full training data:
  if(verbose) cout <<"full: train:";
  train(X, y, lambda, beta);
  double costT = test(X, y, beta);
  if(beta_total) *beta_total = beta;
  if(verbose) cout <<" test: " <<costT <<endl;
  
  if(scoreMean)  *scoreMean =costM; else scoreMeans =ARR(costM);
  if(scoreSDV)   *scoreSDV  =costD; else scoreSDVs  =ARR(costD);
  if(scoreTrain) *scoreTrain=costT; else scoreTrains=ARR(costT);
  if(verbose) cout <<"CV: lambda=" <<lambda <<" \tmean-on-rest=" <<costM <<" \tsdv=" <<costD <<" \ttrain-on-full=" <<costT <<endl;
  if(verbose) cout <<"cross validation results:";
  if(verbose) if(lambda!=-1) cout <<"\n  lambda = " <<lambda;
  if(verbose) cout <<"\n  test-error  = " <<costM <<" (+- " <<costD <<", lower: " <<costM-costD <<")"<<"\n  train-error = " <<costT <<endl;
}

void CrossValidation::crossValidateMultipleLambdas(const arr& X, const arr& y, const arr& _lambdas, uint k_fold, bool permute) {
  lambdas=_lambdas;
  scoreMeans.resizeAs(lambdas);
  scoreSDVs.resizeAs(lambdas);
  scoreTrains.resizeAs(lambdas);
  for(uint i=0; i<lambdas.N; i++){
    crossValidateSingleLambda(X, y, lambdas(i), k_fold, permute, NULL, NULL, &scoreMeans(i), &scoreSDVs(i), &scoreTrains(i));
  }
}

void CrossValidation::plot() {
  FILE("z.cv") <<catCol(lambdas, scoreMeans, scoreSDVs, scoreTrains);
  gnuplot("set log x; set xlabel 'lambda'; set ylabel 'mean squared error'; plot 'z.cv' us 1:2:3 w errorlines title 'cv error','z.cv' us 1:4 w l title 'training error'", "z.pdf", true);
  
}

void linearFeatures(arr& Z, const arr& X) {
  Z.setBlockMatrix(ones(X.d0, 1), X);
}

void quadraticFeatures(arr& Z, const arr& X) {
  uint n=X.d0, d=X.d1;
  Z.resize(n, 1 + d + d*(d+1)/2);
  uint i, j, k, l;
  for(i=0; i<n; i++) {
    arr x=X[i];
    arr z=Z[i];
    l=0;
    z(l++)=1.;
    for(j=0; j<d; j++) z(l++) = x(j);
    for(j=0; j<d; j++) for(k=0; k<=j; k++) z(l++) = x(j)*x(k);
  }
}

void cubicFeatures(arr& Z, const arr& X) {
  uint n=X.d0, d=X.d1;
  Z.resize(n, 1 + d + d*(d+1)/2 + d*(d+1)*(d+2)/6);
  uint i, j, k, l, m;
  for(i=0; i<n; i++) {
    arr x=X[i];
    arr z=Z[i];
    l=0;
    z(l++)=1.;
    for(j=0; j<d; j++) z(l++) = x(j);
    for(j=0; j<d; j++) for(k=0; k<=j; k++) z(l++) = x(j)*x(k);
    for(j=0; j<d; j++) for(k=0; k<=j; k++) for(m=0; m<=k; m++) z(l++) = x(j)*x(k)*x(m);
  }
}

void piecewiseConstantFeatures(arr& Z, const arr& X) {
  uint n=X.d0, d=X.d1;
  if(d!=1) HALT("only for 1D data");
  Z.resize(n, 6);
  Z.setZero();
  for(uint i=0; i<n; i++) {
    double x=X(i, 0);
    arr z=Z[i];
    if(x<-2.5) x=-2.5; if(x>2.5) x=2.5;
    z(floor(x+3.))=1.;
  }
}

void piecewiseLinearFeatures(arr& Z, const arr& X) {
  uint n=X.d0, d=X.d1;
  if(d!=1) HALT("only for 1D data");
  Z.resize(n, 7);
  Z.setZero();
  for(uint i=0; i<n; i++) {
    double x=X(i, 0);
    arr z=Z[i];
    z(0) = 1.; //constant
    z(1) = x; //linear
    for(int j=-2; j<=2; j++) z(j+4) = x<j?0:x-j;
  }
}


void rbfFeatures(arr& Z, const arr& X, const arr& Xtrain) {
  uint rbfBias = mlr::getParameter<uint>("rbfBias", 1);
  double rbfWidth = mlr::sqr(mlr::getParameter<double>("rbfWidth", .2));
  Z.resize(X.d0, Xtrain.d0+rbfBias);
  for(uint i=0; i<Z.d0; i++) {
    if(rbfBias) Z(i, 0) = 1.; //bias feature also for rbfs?
    for(uint j=0; j<Xtrain.d0; j++) {
      Z(i, j+rbfBias) = ::exp(-sqrDistance(X[i], Xtrain[j])/rbfWidth);
    }
  }
}

arr makeFeatures(const arr& X, FeatureType featureType, const arr& rbfCenters) {
  if(featureType==readFromCfgFileFT) featureType = (FeatureType)mlr::getParameter<uint>("modelFeatureType", 1);
  arr Z;
  switch(featureType) {
    case constFT:     Z = consts<double>(1., X.d0, 1);  break;
    case linearFT:    linearFeatures(Z, X);  break;
    case quadraticFT: quadraticFeatures(Z, X);  break;
    case cubicFT:     cubicFeatures(Z, X);  break;
    case rbfFT:       if(&rbfCenters) rbfFeatures(Z, X, rbfCenters); else rbfFeatures(Z, X, X);  break;
    case piecewiseConstantFT:  piecewiseConstantFeatures(Z, X);  break;
    case piecewiseLinearFT:    piecewiseLinearFeatures(Z, X);  break;
    default: HALT("");
  }
  return Z;
}

void artificialData(arr& X, arr& y, ArtificialDataType dataType) {
  uint n = mlr::getParameter<uint>("n", 100);
  uint d = mlr::getParameter<uint>("d", 1);
  double sigma = mlr::getParameter<double>("sigma", 1.); // observation noise
  
  if(dataType==readFromCfgFileDT) dataType = (ArtificialDataType)mlr::getParameter<uint>("dataType", 1);
  switch(dataType) {
    case linearRedundantData:
    case linearData: {
      X = randn(n, d);
      arr Z = makeFeatures(X, (FeatureType)mlr::getParameter<uint>("dataFeatureType", 1));
      arr beta;
      beta = randn(Z.d1, 1).reshape(Z.d1);
      if(dataType==linearRedundantData){
	double pr = mlr::getParameter<double>("d_p_redundant", .5);
	for(uint j=1;j<beta.N;j++) if(rnd.uni()<pr) beta(j)=0.;
      }
      y = Z*beta;
      y = y + sigma*randn(size(y));
      beta_true = beta;
      break;
    }
    case sinusData: {
      X.setGrid(1, -3, 3, n-1);
      y.resize(X.d0);
      for(uint i=0; i<X.d0; i++) y(i) = sin(X(i, 0));
      y += sigma*randn(size(y));
      break;
    }
    case linearOutlier: {
      double rate = mlr::getParameter<double>("outlierRate", .1);
      X = randn(n, d);
      arr Z = makeFeatures(X, (FeatureType)mlr::getParameter<uint>("dataFeatureType", 1));
      arr beta;
      beta = randn(Z.d1, 1).reshape(Z.d1);
      y = Z*beta;
      for(uint i=0; i<y.N; i++)  if(rnd.uni()<rate) {
          y(i) += mlr::getParameter<double>("outlierSigma", 10.)*rnd.gauss();
        } else {
          y(i) += sigma*rnd.gauss();
        }
      beta_true = beta;
      break;
    }
    default: HALT("");
  }
  cout <<"correct beta=" <<beta_true <<endl;
}

void artificialData_Hasties2Class(arr& X, arr& y) {
  uint n = mlr::getParameter<uint>("n", 100);
  uint d = mlr::getParameter<uint>("d", 2);

  arr means0(10, d), means1(10, d), x(d), bias0(d), bias1(d);

  bias0.setZero(); bias0(0) = 1.;
  bias1.setZero(); if(d>1) bias1(1) = 1.;

  rndGauss(means0);  means0 += ones(10,1)*~bias0;
  rndGauss(means1);  means1 += ones(10,1)*~bias1;

  X.clear();
  y.clear();
  for(uint i=0; i<n; i++) {
    rndGauss(x, .2);  x += means0[rnd(10)];
    X.append(~x);
    y.append(0);

    rndGauss(x, .2);  x += means1[rnd(10)];
    X.append(~x);
    y.append(1);
  }
}

void artificialData_HastiesMultiClass(arr& X, arr& y) {
  uint n = mlr::getParameter<uint>("n", 100);
  uint d = mlr::getParameter<uint>("d", 2);
  uint M = mlr::getParameter<uint>("M", 3);

  arr means(M, 10, d), x(d);
  
  rndGauss(means);
  for(uint c=0; c<M; c++)  means[c]() += ones(10,1)*~consts((double)c,d);
  
  X.resize(M*n, d);
  y.resize(M*n, M);
  y.setZero();
  for(uint i=0; i<n; i++) {
    for(uint c=0; c<M; c++) {
      arr x=X[i*M+c];  rndGauss(x, .2);  x += means[c][rnd(10)];
      y(i*M+c, c)=1.;
    }
  }
}

void artificialData_GaussianMixture(arr& X, arr& y) {
  uint n = mlr::getParameter<uint>("n", 100);
  uint M = mlr::getParameter<uint>("M", 3);
  double sig = mlr::getParameter<double>("sigma", .2);
  
  arr means(M, 2), V(M, 2, 2), x(2);
  
  rndGauss(means);
  rndGauss(V);
  //means.setZero();
  //for(uint c=0;c<M;c++)  means[c]() += ARR(c, c);
  
  X.resize(M*n, 2);
  y.resize(M*n, M);
  y.setZero();
  for(uint i=0; i<n; i++) {
    for(uint c=0; c<M; c++) {
      arr x=X[i*M+c];  rndGauss(x, sig);  x = V[c]*x;  x += means[c];
      y(i*M+c, c)=1.;
    }
  }
}


void load_data(arr& X, const char* filename, bool whiten) {
  ifstream is;
  mlr::open(is, filename);
  mlr::Array<mlr::String> strs;
  if(!mlr::contains("0123456789.-+", mlr::peerNextChar(is))) {
    //read line of strings
    mlr::String str;
    for(;;) {
      str.read(is, " \"\t\r", " \"\t\r\n", false);
      if(!str.N) break;
      strs.append(str);
    }
    cout <<"header: " <<strs <<endl;
  }
  X.clear();
  X.read(is);
  cout <<"data stats:"
       <<"\n  data entries    n=" <<X.d0
       <<"\n  entry dimension d=" <<X.d1
       <<"\n  stats: [# 'name' mean sdv]" <<endl;
  arr mean = sum(X, 0);  mean /= (double)X.d0;
  arr var = ~X*X;       var /= (double)X.d0;
  var -= mean^mean;
  for(uint j=0; j<X.d1; j++) {
    cout <<j <<' ';
    if(strs.N) cout <<strs(j) <<' ';
    cout <<mean(j) <<' ' <<sqrt(var(j, j)) <<endl;
  }
  
  //-- whiten the data
  if(whiten) {
    for(uint i=0; i<X.d0; i++) for(uint j=0; j<X.d1; j++) {
        X(i, j) /= sqrt(var(j, j));
      }
  }
}


/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#include "ann.h"
#include "algos.h"

#ifdef MLR_ANN

#include <ANN/ANN.h>

struct sANN {
  ANNkd_tree *tree;
  //PartialLeastSquares pls;
  mlr::Array<double*> cpointers;
  uint treeSize;   //for how many entries in X have we build the tree?
  void clear() { if(tree) delete tree;   tree=NULL;  cpointers.clear();  treeSize=0; }
};

ANN::ANN() {
  bufferSize = 1 <<10;
  s = new sANN;
  s->tree = NULL;
  s->treeSize = 0;
}

ANN::~ANN() {
  s->clear();
  delete s;
  //annClose(); //mt09-07-29 this would close down the ANN lib completely
}

void ANN::clear() {
  s->clear();
  X.clear();
}

void ANN::setX(const arr& _XX) {
  s->clear();
  X=_XX;
}

void ANN::append(const arr& x) {
  double *p=X.p;
  X.append(x);
  if(X.N==x.d0) X.reshape(1, x.d0);
  if(X.p!=p) s->clear(); //when the memory location changed clear the tree! (implies recomputation)
}

void ANN::calculate() {
  if(s->treeSize == X.d0) return;
  s->clear();
  X.getCarray(s->cpointers);
  s->tree = new ANNkd_tree(s->cpointers.p, X.d0, X.d1);
  s->treeSize = X.d0;
}

void ANN::getkNN(arr& dists, intA& idx, const arr& x, uint k, double eps, bool verbose) {
  CHECK(X.d0>=k, "data has less (" <<X.d0 <<") than k=" <<k <<" points");
  CHECK_EQ(x.N,X.d1, "query point has wrong dimension. x.N=" << x.N << ", X.d1=" << X.d1);
  
  if(X.d0-s->treeSize>bufferSize) {
    if(verbose) std::cout <<"ANN recomputing: X.d0=" <<X.d0 <<" treeSize=" <<s->treeSize <<std::endl;
    calculate();
  }
  uint restStartsAt;
  if(s->treeSize>=k) {
    dists.resize(k);
    idx.resize(k);
    s->tree->annkSearch(x.p, k, idx.p, dists.p, eps);
    restStartsAt=s->treeSize;
  } else {
    dists.clear();
    idx.clear();
    restStartsAt=0;
  }
  
  //now check if in the rest of X there are even nearer points
  for(uint i=restStartsAt; i<X.d0; i++) {
    for(uint j=0; j<=idx.N && j<k; j++) {
      double d=sqrDistance(X[i], x);
      if(j==idx.N || d < dists(j)) {
        idx.insert(j, i);
        dists.insert(j, d);
        break;
      }
    }
  }
  if(idx.N>k) {
    idx.resizeCopy(k);
    dists.resizeCopy(k);
  }
  
  if(verbose) {
    std::cout
        <<"ANN query:"
        <<"\n data size = " <<X.d0 <<"  data dim = " <<X.d1 <<"  treeSize = " <<s->treeSize
        <<"\n query point " <<x
        <<"\n found neighbors:\n";
    for(uint i=0; i<idx.N; i++) {
      std::cout <<' '
                <<i <<' '
                <<idx(i) <<'\t'
                <<sqrt(dists(i)) <<'\t'
                <<X[idx(i)] <<std::endl;
    }
  }
}

uint ANN::getNN(const arr& x, double eps, bool verbose) {
  intA idx;
  arr dists;
  getkNN(dists, idx, x, 1, eps, verbose);
  return idx(0);
}

void ANN::getkNN(intA& idx, const arr& x, uint k, double eps, bool verbose) {
  arr dists;
  getkNN(dists, idx, x, k, eps, verbose);
}

void ANN::getkNN(arr& xx             , const arr& x, uint k, double eps, bool verbose) {
  intA idx;
  arr dists;
  getkNN(dists, idx, x, k, eps, verbose);
  xx.resize(idx.N, X.d1);
  for(uint i=0; i<idx.N; i++) xx[i]=X[idx(i)];
}

#else //MLR_ANN

ANN::ANN() { NICO }
ANN::~ANN() { NICO }
void ANN::setX(const arr& _XX) { NICO }
void ANN::append(const arr& x) { NICO }
uint ANN::getNN(const arr& x, double eps, bool verbose) { NICO }
void ANN::getkNN(intA& idx, const arr& x, uint k, double eps, bool verbose) { NICO }
void ANN::getkNN(arr& sqrDists, intA& idx, const arr& x, uint k, double eps, bool verbose) { NICO }
void ANN::getkNN(arr& X, const arr& x, uint k, double eps, bool verbose) { NICO }

#endif
/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


/* Note (mt): The Hungarian does a /perfect/ weighted bi-partite graph matching.
 * Perfect means that the mathching has full caridinality, i.e., all vertices have
 * a match (in the other partite). Non-perfect minimum weight matching can be reduced
 * to perfect matching by just expanding the graph (having O+P vertices); see also
 * (section 1.5.1 of Guido Schäfer's Master's thesis
 * http://homepages.cwi.nl/~schaefer/ftp/pdf/masters-thesis.pdf
 *
 * The Hungarian is not really efficient. More efficient implementations of graph matching are found here:
 * http://pub.ist.ac.at/~vnk/software.html#BLOSSOM5
 * and here
 * https://www.cs.purdue.edu/homes/apothen/software.html
 *
 * General info:
 * https://en.wikipedia.org/wiki/Matching_(graph_theory)
 * https://en.wikipedia.org/wiki/Blossom_algorithm
 *
 * If it is not an efficiency bottleneck, we stick with Hungarian first.
 */

#include "hungarian.h"

Hungarian::Hungarian(const arr& cost_matrix)
{
  costs = cost_matrix;
  dim = costs.dim(0);
  starred = zeros(dim, dim);
  primed = starred;
  covered_rows = zeros(dim);
  covered_cols = covered_rows;
  minimize();
}

Hungarian::~Hungarian(){}

void Hungarian::minimize()
{
  covered_rows = covered_cols = zeros(dim);
  starred = primed = zeros(dim, dim);
  for (uint i = 0; i < dim; i++ )
  {
    double minRow = costs[i]().minIndex();
    costs[i]() -= costs(i, minRow);
  }
  costs = ~costs;

  for (uint i = 0; i < dim; i++ )
  {
    double minRow = costs[i]().minIndex();
    costs[i]() -= costs(i, minRow);
  }
  costs = ~costs;
  starZeros();
}

void Hungarian::starZeros()
{
  for (uint i = 0; i < dim; i++ )
  {
    if (covered_rows(i))
      continue;

    for (uint j = 0; j < dim; j++ )
    {
      if (covered_cols(j))
        continue;

      if (costs(i,j) == 0)
      {
        starred(i,j) = 1;
        covered_rows(i) = 1;
        covered_cols(j) = 1;
        break;
      }
    }
  }

  covered_rows = zeros(dim);
  covered_cols = covered_rows;
  coverColumns();
}

void Hungarian::coverColumns()
{
  uint count = 0;
  starred = ~starred;
  for (uint i = 0; i < dim; i++ )
  {
    if (sum(starred[i]()) > 0)
    {
      covered_cols(i) = 1;
      count++;
    }
  }
  starred = ~starred;

  if (count == dim)
    return;

  prime();
}

void Hungarian::prime()
{
  // Find an uncovered zero.
  for (uint i = 0; i < dim; i++ )
  {
    if (covered_rows(i))
      continue;

    for (uint j = 0; j < dim; j++ )
    {
      if (covered_cols(j))
        continue;

      if (costs(i,j) == 0)
      {
        primed(i,j) = 1;
        // Check to see if there is a starred zero in this row.

        if (sum(starred[i]()) == 0)
        {
          path_row.clear();
          path_row.push_back(i);
          path_col.clear();
          path_col.push_back(j);
          makePath();
          return;
        }
        else
        {
          // Cover this row
          covered_rows(i) = 1;
          // Uncover columns containing star
          uint maxIndex = starred[i]().maxIndex();
          covered_cols(maxIndex) = 0;
          prime();
          return;
        }
      }
    }
  }
  modifyCost();
  return;
}

void Hungarian::makePath()
{
  uint count = 0;

  while (1)
  {
    starred = ~starred;
    // find the star in the column
    int row = starred[path_col.at(count)]().maxIndex();

    starred = ~starred;
    if (starred(row, path_col.at(count)) == 0)
      break;

    count++;
    path_row.push_back(row);
    path_col.push_back(path_col.at(count - 1));

    // find the prime in this row
    int col = primed[row]().maxIndex();
    count++;
    path_row.push_back(path_row.at(count - 1));
    path_col.push_back(col);
  }

  // Modify it.
  for (uint i = 0; i <= count; i++ )
  {
    uint row = path_row.at(i);
    uint col = path_col.at(i);

    if (starred(row,col))
    {
      starred(row,col) = 0;
    }
    else
    {
      starred(row,col) = 1;
    }
  }

  // Clear covers and primes, call cover columns.
  covered_rows = covered_cols = zeros(dim);
  primed = zeros(dim, dim);
  coverColumns();
}

void Hungarian::modifyCost()
{
  auto minCost = max(costs);
  for (uint i = 0; i < dim; i++ )
  {
    if (covered_rows(i))
      continue;

    for (uint j = 0; j < dim; j++)
    {
      if (covered_cols(j))
        continue;

      if (costs(i,j) < minCost)
        minCost = costs(i,j);
    }
  }
  // Modify the costs
  for (uint i = 0; i < dim; i++ )
  {
    for (uint j = 0; j < dim; j++)
    {
      if (covered_rows(i))
      {
        costs(i,j) += minCost;
      }
      else if (!covered_cols(j))
      {
        costs(i,j) -= minCost;
      }
    }
  }

  prime();
}
/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#include "algos.h"


namespace mlr{

void rk4(arr& x1, const arr& x0,
         const VectorFunction& f,
         double dt) {
  arr k1,k2,k3,k4;
  f(k1, NoArr, x0);
  f(k2, NoArr, x0 + 0.5*dt*k1);
  f(k3, NoArr, x0 + 0.5*dt*k2);
  f(k4, NoArr, x0 +     dt*k3);
  
  if(&x1!=&x0) x1 = x0;
  x1 += (dt/6.)*(k1 + 2.*k2 + 2.*k3 + k4);
}

void rk4_2ndOrder(arr& x, const arr& x0, const VectorFunction& f, double dt){
  CHECK(x0.nd==2 && x0.d0==2,"need a 2-times-n array   rk4_2ndOrder input");
  struct F2:VectorFunction{
    const VectorFunction& f;
    F2(const VectorFunction& _f):f(_f){
      VectorFunction::operator= ( [this](arr& y, arr& J, const arr& x) -> void {
        fv(y, J, x);
      } );
    }
    void fv(arr& y, arr& J, const arr& x){
      CHECK(x.nd==2 && x.d0==2,"");
      y.resizeAs(x);
      y[0]=x[1];
      f(y[1](), NoArr, x);
    }
  } f2(f);
  rk4(x, x0, f2, dt);
}

#if 0
bool rk4_switch(arr& x1, arr& s1, const arr& x0, const arr& s0,
                    void (*df)(arr& xd, const arr& x),
                    void (*sf)(arr& s, const arr& x),
                    double& dt, double tol) {
  uint i, sn;
  arr sa=s0, sb, sm, xa=x0, xb, xm; //states at times a, m, t
  rk4(xb, x0, df, dt);
  sf(sb, xb);
  //CHECK_EQ(sa.N,sb.N, "inconsistent state indicators");
  bool change=false;
  sn=sa.N<sb.N?sa.N:sb.N;
  for(i=0; i<sn; i++) if(s0(i)*sb(i)<0.) {
      change=true;
      break;
    }
  if(!change) { x1=xb; s1=sb; return false; } //no problems: no switch
  
  //we have a switch - so we must find it precisely!
  double a=0., b=dt; //time interval [a, b]
  double m, min_m;   //where to cut the interval (determined by linear interpolation)
  
  cout <<"entering zero-crossing detection loop" <<endl;
  for(; fabs(b-a)>tol;) {
    //compute new m
    min_m=m=b;
    sn=sa.N<sb.N?sa.N:sb.N;
    for(i=0; i<sn; i++) if(sa(i)*sb(i)<0.) {
        m = b - sb(i) * (b-a)/(sb(i)-sa(i));
        if(m<min_m) min_m=m;
      }
    min_m=m;
    if(m-a<.1*tol) m+=.1*tol; //really close already
    if(b-m<.1*tol) m-=.1*tol; //really close already
    rk4(xm, x0, df, m);
    sf(sm, xm);
    change=false;
    sn=s0.N<sm.N?s0.N:sm.N;
    for(i=0; i<sn; i++) if(s0(i)*sm(i)<0.) { change=true; break; }
    
    //cout <<"a=" <<a <<" b=" <<b <<" m=" <<m <<" sa=" <<sa <<" sb=" <<sb <<" sm=" <<sm <<endl;
    cout <<" sm=" <<sm <<endl;
    if(!change) {
      a=m;
      sa=sm;
      xa=xm;
    } else {
      b=m;
      sb=sm;
      xb=xm;
    }
  }
  
  //take right limit of time interval
  dt=b;
  x1=xb;
  s1=sb;
  cout <<"DONE" <<endl <<"dt=" <<dt <<" s1=" <<s1 <<endl;
  return true;
}

bool rk4dd_switch(arr& x1, arr& v1, arr& s1, const arr& x0, const arr& v0, const arr& s0,
                      void (*ddf)(arr& xdd, const arr& x, const arr& v),
                      void (*sf)(arr& s, const arr& x, const arr& v),
                      double& dt, double tol) {
                      
  global_ddf = ddf;
  global_sf  = sf;
  
  uint n=x0.N;
  
  arr X(2, n), Y(2*n);
  X[0]=x0;
  X[1]=v0;
  X.reshape(2*n);
  
  bool change=rk4_switch(Y, s1, X, s0, rk_df, rk_sf, dt, tol);
  
  Y.reshape(2, n);
  x1=Y[0];
  v1=Y[1];
  return change;
}
#endif


} //end namespace
/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#include "gaussianProcess.h"

#define MLR_GP_DEBUG 0

/** prior of 0 */
double const_0(const arr &x, const void *p){return 0.;}

GaussianProcess::GaussianProcess() :
  mu(0.),
  mu_func(const_0),
  priorP(NULL),
  obsVar(.1),
  kernelP(NULL)
{}

/** set Gauss cov function, its parameters, and GP prior
 */
void GaussianProcess::setGaussKernelGP(
    void *_kernelP,
    double _mu){
  mu = _mu;
  mu_func = const_0;
  priorP = NULL;
  kernelP=_kernelP;
  cov=GaussKernel;
  dcov=dGaussKernel;
  covF_D=GaussKernelF_D;
  covD_D=GaussKernelD_D;
  covDD_F=GaussKernelDD_F;
  covDD_D=GaussKernelDD_D;
}

/** set Gauss cov function, its parameters, and GP prior
 */
void GaussianProcess::setGaussKernelGP(
    void *_kernelP,
    double(*_mu)(const arr&, const void*),
    void *_priorP){
  mu_func = _mu;
  priorP = _priorP;
  mu = 0;
  kernelP=_kernelP;
  cov=GaussKernel;
  dcov=dGaussKernel;
  covF_D=GaussKernelF_D;
  covD_D=GaussKernelD_D;
  covDD_F=GaussKernelDD_F;
  covDD_D=GaussKernelDD_D;
}


void GaussianProcess::recompute(const arr& _XX, const arr& _YY){
  X.referTo(_XX);
  Y.referTo(_YY);
  recompute();
}

void GaussianProcess::recompute(){
  uint i, j, N=Y.N, dN=dY.N;
  arr gram, xi, xj, Mu_func;
  gram.resize(N+dN, N+dN);
  if(!gram.N) return;
  for(i=0; i<N; i++){
    xi.referToDim(X, i);
    gram(i, i) = cov(kernelP, xi, xi);
    Mu_func.append(mu_func(xi, priorP));
  }
  for(i=1; i<N; i++){
    xi.referToDim(X, i);
    for(j=0; j<i; j++){
      xj.referToDim(X, j);
      gram(i, j) = gram(j, i) = cov(kernelP, xi, xj);
    }
  }
  if(dN){ //derivative observations
    for(i=0; i<dN; i++){ xi.referToDim(dX, i); gram(N+i, N+i) = covD_D(dI(i), dI(i), kernelP, xi, xi); }
    for(i=0; i<dN; i++){
      xi.referToDim(dX, i);
      for(j=0; j<N; j++){
        xj.referToDim(X, j);
        gram(N+i, j) = gram(j, N+i) = covF_D(dI(i), kernelP, xj, xi);
      }
      for(j=0; j<i; j++){
        xj.referToDim(dX, j);
        gram(N+i, N+j) = gram(N+j, N+i) = covD_D(dI(i), dI(j), kernelP, xi, xj);
      }
    }
  }
  gram = gram + obsVar * eye(gram.d0);
  inverse_SymPosDef(Ginv, gram);
  if(!dN){
    if(N) GinvY = Ginv * (Y-Mu_func-mu); else GinvY.clear();
  }else{
    arr Yfull; Yfull.append(Y-Mu_func-mu); Yfull.append(dY);
    GinvY = Ginv * Yfull;
  }
  //cout <<"gram=" <<gram <<" Ginv=" <<Ginv <<endl;
}

void GaussianProcess::appendObservation(const arr& x, double y){
  uint N=X.d0;
  X.append(x); //append it to the data
  Y.append(y);
  X.reshape(N+1, x.N);
  Y.reshape(N+1);
#if MLR_GP_DEBUG
  arr iG=Ginv;
  recompute();
  double err=maxDiff(iG, Ginv);
  CHECK(err<1e-6, "mis-updated inverse Gram matrix" <<err <<endl <<iG <<Ginv);
#endif
}

void GaussianProcess::appendDerivativeObservation(const arr& x, double y, uint i){
  uint N=dX.d0;
  dX.append(x); //append it to the data
  dY.append(y);
  dI.append(i);
  dX.reshape(N+1, x.N);
  dY.reshape(N+1);
  dI.reshape(N+1);
}

void GaussianProcess::appendGradientObservation(const arr& x, const arr& nabla){
  for(uint i=0; i<nabla.N; i++) appendDerivativeObservation(x, nabla(i), i);
}

double GaussianProcess::max_var(){
  return cov(kernelP, ARR(0.), ARR(0.));
}

void GaussianProcess::evaluate(const arr& x, double& y, double& sig, bool calcSig){
  uint i, N=Y.N, dN=dY.N;
  /*static*/ arr k, xi, Ginvk; //danny: why was there a static
  if(N+dN==0){ //no data
    y = mu_func(x, priorP) + mu;
    sig=::sqrt(cov(kernelP, x, x));
    return;
  }
  if(k.N!=N+dN) k.resize(N+dN);
  for(i=0; i<N; i++){ xi.referToDim(X, i); k(i)=cov(kernelP, x, xi); }
  //derivative observations
  for(i=0; i<dN; i++){ xi.referToDim(dX, i); k(N+i)=covF_D(dI(i), kernelP, x, xi); }
  
  y = scalarProduct(k, GinvY) + mu_func(x, priorP) + mu;
  if(calcSig) {
    innerProduct(Ginvk, Ginv, k);
    sig = cov(kernelP, x, x) - scalarProduct(k, Ginvk);
    //if(sig<=10e-10) {
    //cout << "---" << endl;
    //cout << "x==" << x << endl;
    //cout << "k==" << k << endl;
    //cout << "Ginv==" << Ginv << endl;
    //cout << "kGinvk==" << scalarProduct(k, Ginvk) << endl;
    //sig=::sqrt(sig);
    //cout << "sig==" << sig << endl;
    //}
    //else
    sig = ::sqrt(sig);
  }
}

double GaussianProcess::log_likelihood() {
  arr gram;
  inverse_SymPosDef(gram, Ginv);
  return (-.5*~Y*GinvY - .5*log(length(gram)) - (X.N+dX.N)/2 * log(2*MLR_PI))(0); // actually a degenerated array of size 1x1
}

/** vector of covariances between test point and N+dN observation points */
void GaussianProcess::k_star(const arr& x, arr& k){
  uint i, N=Y.N, dN=dY.N;
  arr xi;
  
  if(k.N!=N+dN) k.resize(N+dN);
  for(i=0; i<N; i++){ xi.referToDim(X, i); k(i)=cov(kernelP, x, xi); }
  for(i=0; i<dN; i++){ xi.referToDim(dX, i); k(N+i)=covF_D(dI(i), kernelP, x, xi); }
}

/** vector of covariances between test point and N+dN  observation points */
void GaussianProcess::dk_star(const arr& x, arr& k){
  uint i, j, N=Y.N, dN=dY.N, d=x.N;
  arr xi;
  
  if(k.N!=N+dN) k.resize(N+dN, d);
  for(j=0; j<d; ++j){
    for(i=0; i<N; i++){
      xi.referToDim(X, i);
      k(i, j)=covD_F(j, kernelP, x, xi);
    }
    for(i=0; i<dN; i++){
      xi.referToDim(dX, i);
      k(N+i, j)=covD_D(j, dI(i), kernelP, x, xi);
    }
  }
}

/*****
 *
\[
\nabla f(x) (i) =  \frac{
     \partial (  \vec{k}  K^{-1} Y_{all} ) }{
     \partial x_i }
\]

which translates to
\[
 \frac{
   \partial k(x, \vec{X_j}) }{
   \partial x_i }
   K^{-1} Y_{all},
    for j \in {1...N}            % function value observations
\]
and
\[
 \frac{
   \partial^2 k(x, \vec{X_j}) }{
   \partial x_i  \partial x_l }
   K^{-1} Y_{all},
   j \in {N...dN}           % derivative observations
\]

where
<li>
\(k\) is the covariance function,
<li>
\(k\) is a vector \(1..N+dN\) with \(\vec{k}_j=k(x, \vec{X}j)\),
<li>
\(K\) is the Gram matrix \((\vec{X^TX}-\sigma^2\vec{I})^{-1}\)(augmented with noise),
<li>
\(Y_{all}\) is the vector of all observed responses,
<li>
\(\vec{X_j}\) is the vector of coordinates of the j-th data point,
<li>
\(x_l\) is the the same component in which the derivative in \(X_j\) has been
   observed
<li>
\(K^{-1} Y_{all}\) is a column vector;
<li>
the resulting \(\nabla f(x) \) is vector of the dimensionality of the GP
*
*/
void GaussianProcess::gradient(arr& grad, const arr& x){
  CHECK(X.N || dX.N , "can't recompute gradient without data");
  CHECK((X.N && x.N==X.d1) || (dX.N && x.N==dX.d1), "dimensions don't match!");
  uint i, d, N=Y.N, dN=dY.N, dim;
  dim = X.d1?X.d1:dX.d1;
  arr dk(dim);
  /*static*/ arr xi, dxi; //danny: why was there a static?
  grad.resize(x.N);
  grad.setZero();
  // take the gradient in the function value observations
  for(i=0; i<N; i++){
    xi.referToDim(X, i);
    dcov(dk, kernelP, x, xi);
    grad += GinvY(i) * dk;
    //cout << dk << endl;
  }
  // derivative observations
  for(i=0; i<dN; i++){
    dxi.referToDim(dX, i);
    dk.setZero();
    for(d=0; d<dim; ++d){
      dk(d) = covD_D(d, dI(i), kernelP, x, dxi);
    }
    grad += GinvY(i+N) * dk;
  }
}

/*****
 * The hessian (of the posterior mean \(f\) ) is a \(d\times d\) matrix of
\[
\frac {
\partial^2 f(\vec{x}) }{
\partial x_i, x_j, x_k} =
\partial^2 ( \vec{k}  K^{-1} Y_{all} ) }{
\partial x_i, x_j, x_k} =
\]

and translates to
\[
 \frac{
   \partial^2 k(x, \vec{X_n}) }{
   \partial x_i, x_j }
   K^{-1} Y_{all},
    n \in {1...N}            % function value observations
\]
and
\[
 \frac{
   \partial^3 k(x, \vec{X_n}) }{
   \partial x_l \partial x_i \partial x_j }
   K^{-1} Y_{all},
   n \in {N...dN}           % derivative observations
\]

where
<li>
\(k\) is the covariance function,
<li>
\(k\) is a vector \(1..N+dN\) with \(\vec{k}_j=k(x, \vec{X}j)\),
<li>
\(K\) is the Gram matrix \((\vec{X^TX}-\sigma^2\vec{I})^{-1}\)(augmented with noise),
<li>
\(Y_{all}\) is the vector of all observed responses,
<li>
\(\vec{X_n}\) is the vector of coordinates of the n-th data point,
<li>
\(x_l\) is the the same component in which the derivative in \(X_n\) has been
   observed
<li>
\(K^{-1} Y_{all}\) is a column vector;
<li>
the resulting \(\nabla f(x) \) is vector of the dimensionality of the GP

see also gradient()

pseudocode:
H:=H(3x3xd)
d:=GP dimension
for n \in \{1..N\}
  for i, j \in \{ 1..d\}
    H_{i, j, n} = covD_D(i, j, ..., \vec{x}, \vec{X_n})
  end
end
for n \in \{N+1..N+dN\}
  for i, j \in \{ 1..d\}
    H_{i, j, n} = covDD_D(i, j, dI(n), ..., \vec{x}, \vec{X_n})
  end
end
*
*/
void GaussianProcess::hessianPos (arr& hess, const arr& x){
  //Danny: I think that this is wrong.. Or at least numerical Hessian checking fails
  CHECK(X.N || dX.N , "can't recompute Hessian without data");
  CHECK((X.N && x.N==X.d1) || (dX.N && x.N==dX.d1), "dimensions don't match!");
  uint i, j, n, N=Y.N, dN=dY.N, dim;
  dim = X.d1?X.d1:dX.d1;
  arr d2k(N+dN, dim, dim);
  /*static*/ arr xn, dxn; //danny: why was there a static
  d2k.setZero();
  hess.resize(dim, dim);
  hess.setZero();
  // function value observations
  for(n=0; n<N; n++){
    xn.referToDim(X, n);
    for(i=0; i<dim; i++){
      for(j=0; j<dim; j++){
        d2k(n, i, j)=covD_D(i, j, kernelP, x, xn);
      }
    }
    //TODO: add inv gram
    hess += GinvY(n) * d2k[n];
  }
  // derivative observations
  for(n=0; n<dN; n++){
    dxn.referToDim(dX, n);
    for(i=0; i<dim; i++){
      for(j=0; j<dim; j++){
        d2k(n, i, j)=covDD_D(i, j, dI(n), kernelP, x, dxn);
      }
    }
    //TODO: add inv gram
    hess += GinvY(n+N) * d2k[n];
  }
}

void GaussianProcess::gradientV(arr& grad, const arr& x) {
  arr k, dk;
  k_star(x, k);
  dk_star(x, dk);
  grad = -2.0*~k*Ginv*dk;
}

void GaussianProcess::evaluate(const arr& X, arr& Y, arr& S){
  uint i;
  arr xi;
  Y.resize(X.d0); S.resize(X.d0);
  for(i=0; i<X.d0; i++){ xi.referToDim(X, i); evaluate(xi, Y(i), S(i)); }
}
/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */

#include "spline.h"
#include "plot.h"

//==============================================================================
//
// Spline
//


namespace mlr{

Spline::Spline(uint degree) : degree(degree){}

Spline::Spline(uint T, const arr& X, uint degree) : points(X){
  CHECK(points.nd==2,"");
  setUniformNonperiodicBasis(T, points.d0-1, degree);
}

void Spline::plotBasis() {
  plotClear();
  arr b_sum(basis.d0);
  tensorMarginal(b_sum, basis_trans, TUP(1u));
  plotFunction(b_sum, -1, 1);
  for(uint i=0; i<points.d0; i++) plotFunction(basis_trans[i], -1, 1);
  plot();
}



arr Spline::getCoeffs(double t, uint K, uint derivative) const {
  arr b(K+1), b_0(K+1), db(K+1), db_0(K+1), ddb(K+1), ddb_0(K+1);
  for(uint p=0; p<=degree; p++) {
    b_0=b; b.setZero();
    db_0=db; db.setZero();
    ddb_0=ddb; ddb.setZero();
    for(uint k=0; k<=K; k++) {
      if(!p) {
        if(times(k)<=t && t<times(k+1)) b(k)=1.;
        if(k==K && t>=times(k+1)) b(k)=1.;
      } else {
        double xnom = t - times(k);
        double xden = times(k+p) - times(k);
        double x = DIV(xnom, xden, true);
        b(k) = x * b_0(k);
        db(k) = DIV(1., xden, true) * b_0(k) + x * db_0(k);
        ddb(k) = DIV(2., xden, true) * db_0(k) + x * ddb_0(k);
        if(k<K) {
          double ynom = times(k+p+1) - t;
          double yden = times(k+p+1) - times(k+1);
          double y = DIV(ynom, yden, true);
          b(k) += y * b_0(k+1);
          db(k) += DIV(-1., yden, true) * b_0(k+1) + y * db_0(k+1);
          ddb(k) += DIV(-2., yden, true) * db_0(k+1) + y * ddb_0(k+1);
        }
      }
    }
  }
  switch(derivative) {
    case 0:
      return b;
    case 1:
      return db;
    case 2:
      return ddb;
  }
  HALT("Derivate of order " << derivative << " not yet implemented.");
}

void Spline::setBasis(uint T, uint K) {
  CHECK_EQ(times.N-1,K+1+degree, "wrong number of time knots");
  basis.resize(T+1, K+1);
  for(uint t=0; t<=T; t++) basis[t] = getCoeffs((double)t/T, K);
  transpose(basis_trans, basis);
}

void Spline::setBasisAndTimeGradient(uint T, uint K) {
  uint i, j, t, p, m=times.N-1;
  double time, x, xx, y, yy;
  CHECK_EQ(m,K+1+degree, "wrong number of time knots");
  arr b(K+1, T+1), b_0(K+1, T+1), dbt(m+1, K+1, T+1), dbt_0(m+1, K+1, T+1);
  for(p=0; p<=degree; p++) {
    if(p>0) { b_0=b; dbt_0=dbt; }
    for(i=0; i<=K; i++) for(t=0; t<=T; t++) {
        time = (double)t/(double)T;
        if(!p) {
          b(i, t) = 0.;
          if(times(i)<=time && time<times(i+1)) b(i, t)=1.;
          if(t==T && i==K && time==times(i+1)) b(i, t)=1.;
          for(j=0; j<=m; j++) dbt(j, i, t)=0.;
        } else {
          xx=times(i+p)-times(i);
          x=DIV(time-times(i), xx, true);
          if(i<K) {
            yy=times(i+p+1)-times(i+1);
            y=DIV(times(i+p+1)-time, yy, true);
          } else {
            yy=1.;
            y=0.;
          }
          b(i, t) = x * b_0(i, t);
          if(i<K) b(i, t) += y * b_0(i+1, t);
          for(j=0; j<=m; j++) {
            dbt(j, i, t) = x * dbt_0(j, i, t);
            if(i<K) dbt(j, i, t) += y * dbt_0(j, i+1, t);
            if(j==i)            dbt(j, i, t) += DIV((x-1), xx, true) * b_0(i, t);
            if(j==i+p)          dbt(j, i, t) -= DIV(x , xx, true) * b_0(i, t);
            if(i<K && j==i+1)   dbt(j, i, t) += DIV(y , yy, true) * b_0(i+1, t);
            if(i<K && j==i+p+1) dbt(j, i, t) -= DIV((y-1), yy, true) * b_0(i+1, t);
          }
        }
      }
  }
  basis_trans=b;
  transpose(basis, b);
  basis_timeGradient=dbt;
}

void Spline::setUniformNonperiodicBasis() {
  setUniformNonperiodicBasis(0, points.d0-1, degree);
}

void Spline::setUniformNonperiodicBasis(uint T, uint K, uint _degree) {
  degree=_degree;
  uint i, m;
  m=K+1+degree;
  times.resize(m+1);
  for(i=0; i<=m; i++) {
    if(i<=degree) times(i)=.0;
    else if(i>=m-degree) times(i)=1.;
    else times(i) = double(i-degree)/double(m-2*degree);
  }
  if(T) setBasis(T, K);
//  setBasisAndTimeGradient();
}

arr Spline::eval(double t, uint derivative) const {
  uint K = points.d0-1;
  return (~getCoeffs(t, K, derivative) * points).reshape(points.d1);
}

arr Spline::eval(uint t) const { return (~basis[t]*points).reshape(points.d1); }

arr Spline::eval() const { return basis*points; }

arr Spline::smooth(double lambda) const {
  CHECK(lambda >= 0, "Lambda must be non-negative");
  uint T = basis.d0 - 1;
  uint K = basis.d1 - 1;
  arr ddbasis(T+1, K+1);
  for(uint t=0; t<=T; t++)
    ddbasis[t] = getCoeffs((double)t/K, K, 2);
  
  arr A = ~ddbasis * ddbasis / (double)T;
  return basis*inverse(eye(K+1) + lambda*A)*points;
}

void Spline::partial(arr& grad_points, const arr& grad_path) const {
  CHECK(grad_path.d1==points.d1, "");
  grad_points = basis_trans * grad_path;
}

void Spline::partial(arr& dCdx, arr& dCdt, const arr& dCdf, bool constrain) const {
  uint K=points.d0-1, T=basis.d0-1;
  CHECK(dCdf.d0==T+1 && dCdf.d1==points.d1, "");
  CHECK(basis_timeGradient.N, "");
  uint n=dCdf.d1, m=K+1+degree, j;
  dCdx = basis_trans * dCdf;
  arr X;
  X.referTo(points);
  X.reshape((K+1)*n);
  arr B;
  B.referTo(basis_timeGradient);
  B.reshape((m+1)*(K+1), T+1);
  arr Z = B * dCdf; Z.reshape(m+1, (K+1)*n);
  dCdt = Z*X;
  if(constrain) {
    for(j=0; j<=degree; j++) dCdt(j)=0.;
    for(j=m-degree; j<=m; j++) dCdt(j)=0.;
  }
  dCdt(0)=dCdt(m)=0.;
}

//==============================================================================

arr Path::getPosition(double t) const{
  return Spline::eval(t);
}

arr Path::getVelocity(double t) const{
  return Spline::eval(t, true);
}

void Path::transform_CurrentBecomes_EndFixed(const arr& current, double t){
  arr delta = current - eval(t);
  for(uint i=0;i<points.d0;i++){
    double ti = double(i)/double(points.d0-1);
    double a = (1.-ti)/(1.-t);
    points[i]() += a*delta;
  }
}

void Path::transform_CurrentFixed_EndBecomes(const arr& end, double t){
  arr delta = end - eval(1.);
  for(uint i=0;i<points.d0;i++){
    double ti = double(i)/double(points.d0-1);
    double a = (ti-t)/(1.-t);
    points[i]() += a*delta;
  }
}

void Path::transform_CurrentBecomes_AllFollow(const arr& current, double t){
  arr delta = current - eval(t);
  for(uint i=0;i<points.d0;i++) points[i]() += delta;
}

} //namespace mlr
#include "gpOp.h"

GaussianProcessOptimized::GaussianProcessOptimized() :
  m(0.) {}

GaussianProcessOptimized::GaussianProcessOptimized(const GaussianProcessOptimized& copy) {
  X = copy.X;
  Y = copy.Y;
  L = copy.L;
  GinvY = copy.GinvY;
  m = copy.m;
  obsVar = copy.obsVar;
  kernel = copy.kernel->clone();
}

GaussianProcessOptimized::~GaussianProcessOptimized() {
  delete kernel;
}

void GaussianProcessOptimized::setKernel(GaussianProcessKernel* k) {
  kernel = k->clone();
  delete k;
}

void GaussianProcessOptimized::appendObsRecompute(const arr& x, const double& y) {
  if(X.d0) {
    X.append(~x);
    Y.append(y);
    double k = kernel->k(x, x) + obsVar;
    arr kappa = zeros(Y.N-1);
    for(uint i = 0; i < kappa.d0; i++) {
      kappa(i) = kernel->k(X[i], x);
    }
    arr l = lapack_Ainv_b_triangular(L, kappa);
    double lDiag = sqrt(k-sumOfSqr(l));

    arr temp;
    temp.resize(L.d0+1, L.d1+1);
    temp.setMatrixBlock(L, 0, 0);
    temp.setMatrixBlock(l, 0, L.d1);
    temp.setMatrixBlock(zeros(1,L.d1), L.d0, 0);
    temp(L.d0, L.d1) = lDiag;
    L = temp;
    GinvY = lapack_Ainv_b_symPosDef_givenCholesky(L, Y-m);
  } else {
    X.clear();
    Y.clear();
    X.append(~x);
    Y.append(y);
    recompute();
  }
}


void GaussianProcessOptimized::recompute(){
  if(X.d0) {
    arr G;
    G.resize(X.d0, X.d0);
    for(uint i = 0; i < G.d0; i++) {
      for(uint j = i; j < G.d1; j++) {
        G(i,j) = kernel->k(X[i], X[j]); //fill only the upper triangle, because the lapack routine does not access the other one at all.
      }
      G(i,i) += obsVar;
    }
    lapack_cholesky(L, G);
    GinvY = lapack_Ainv_b_symPosDef_givenCholesky(L, Y-m);
  }
}

void GaussianProcessOptimized::clearData() {
  X.clear();
  Y.clear();
  L.clear();
  GinvY.clear();
}

double GaussianProcessOptimized::evaluate(const arr& x) {
  double y, nonsense;
  evaluate(x, y, true, nonsense, false);
  return y;
}

double GaussianProcessOptimized::evaluateVariance(const arr& x) {
  double sig, nonsense;
  evaluate(x, nonsense, false, sig, true);
  return sig;
}

void GaussianProcessOptimized::evaluate(const arr& x, double& y, double& sig) {
  evaluate(x, y, true, sig, true);
}

void GaussianProcessOptimized::evaluate(const arr& x, double& y, bool calcY, double& sig, bool calcSig) {
  arr kappa;
  kappa = kernel->kappa(x, X);
  /*kappa.resize(X.d0);
  for(uint i = 0; i < X.d0; i++) {
    kappa(i) = kernel->k(X[i], x);
  }*/
  if(calcY) {
    y = scalarProduct(kappa, GinvY) + m;
  }
  if(calcSig) {
    arr v = lapack_Ainv_b_triangular(L, kappa);
    sig = sqrt(kernel->k(x, x) - scalarProduct(v,v));
  }
}

arr GaussianProcessOptimized::gradient(const arr& x) {
  arr grad = zeros(x.d0);
  //kernel->dk_dx(x, X[0]) * GinvY(0);
  for(uint i = 0; i < X.d0; i++) {
    grad += kernel->dk_dx(x, X[i])*GinvY(i);
  }
  return grad;
}

arr GaussianProcessOptimized::gradientVariance(const arr& x) {
  arr kappa;
  kappa = kernel->kappa(x, X);
  /*kappa.resize(X.d0);
  for(uint i = 0; i < X.d0; i++) {
    kappa(i) = kernel->k(x, X[i]);
  }*/

  arr dKappa;
  dKappa = kernel->dKappa(x, X);
  /*
  dKappa.resize(X.d0, X.d1);
  for(uint i = 0; i < X.d0; i++) {
    dKappa[i] = kernel->dk_dx(x, X[i]);
  }*/
  //this can be optimized to n^2 complexity, if kappa*L^-1 is solved first (need different lapack/blas routine). Lapack offers no such routine, but blas does (not directly, needs two calls).
  //return kernel->dk_dx(x,x) - 2.0*(~kappa*lapack_Ainv_b_symPosDef_givenCholesky(L, dKappa)).reshapeFlat();

  //this is the optimized, n^2 version, was quite easy :-)
  return kernel->dk_dx(x,x) - 2.0*(~lapack_Ainv_b_symPosDef_givenCholesky(L, kappa)*dKappa).reshapeFlat();
}

arr GaussianProcessOptimized::hessian(const arr& x) {
  arr hessian = zeros(x.d0, x.d0);
  //arr hessian = kernel->d2k_dx2(x, X[0]) * GinvY(0);
  for(uint i = 0; i < X.d0; i++) {
    hessian += kernel->d2k_dx2(x, X[i]) * GinvY(i);
  }
  return hessian;
}
/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */

#include "kalman.h"

void Kalman::stepPredict(const arr& A, const arr& a, const arr& Q){
  b_mean = A*b_mean + a;
  b_var  = Q + A*b_var*~A;
}

void Kalman::stepObserve(const arr& y, const arr& C, const arr& c, const arr& W){
  arr Winv, Sinv, Ct;
  inverse_SymPosDef(Winv, W);
  inverse_SymPosDef(Sinv, b_var);
  transpose(Ct, C);
  b_var  = inverse_SymPosDef(Ct * Winv * C + Sinv);
  b_mean = b_var * (Ct * (Winv * (y-c)) + Sinv*b_mean);
}

void Kalman::step(const arr& A, const arr& a, const arr& Q, const arr& y, const arr& C, const arr& c, const arr& W){
  stepPredict(A, a, Q);
  stepObserve(y, C, c, W);
}

/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#include "phase_optimization.h"

PhaseOptimization::PhaseOptimization(arr &X, uint _kX, double _w){
  T = X.d0;
  p = new mlr::Spline(T,X,1);
  kX = _kX;
  k = 3;
  w = sqrt(_w);
}

arr PhaseOptimization::get_postfix() {
  arr x = ones(get_k(), dim_x());
  return x;
}

arr PhaseOptimization::getInitialization(){
  arr s0 = linspace(0,1,T-1);s0.reshapeFlat();
  s0 = s0.sub(1, s0.d0-2); // remove 0 and 1 from optimization variables
  return s0;
}

void PhaseOptimization::getSolution(arr &xOpt, arr &sOpt){
  xOpt.clear();
  sOpt.prepend(0.); sOpt.append(1.); // add 0 and 1 to optimization variables
  for (uint i=0;i<sOpt.d0;i++) {
    xOpt.append(~p->eval(sOpt(i)));
  }
}

void PhaseOptimization::phi_t(arr& phi, arr& J, ObjectiveTypeA& tt, uint t, const arr& x_bar){
  uint T=get_T(), n=dim_x(), k=get_k();

  //assert some dimensions
  CHECK_EQ(x_bar.d0,k+1,"");
  CHECK_EQ(x_bar.d1,n,"");
  CHECK(t<=T,"");

  //-- transition costs of trajectory: append to phi
  if(kX==1)  phi = p->eval(x_bar(1,0)) - p->eval(x_bar(0,0)); //penalize velocity
  if(kX==2)  phi = p->eval(x_bar(2,0)) - 2.*p->eval(x_bar(1,0)) + p->eval(x_bar(0,0)); //penalize acceleration
  if(kX==3)  phi = p->eval(x_bar(3,0)) - 3.*p->eval(x_bar(2,0)) + 3.*p->eval(x_bar(1,0)) - p->eval(x_bar(0,0)); //penalize jerk

  //-- transition costs of phase: append to phi
  if (t<T) { phi.append( (x_bar(2,0) - 2.*x_bar(1,0) + x_bar(0,0))*w ); }

  if(&tt) tt.append(OT_sumOfSqr, phi.N);

  uint m=phi.N;
  CHECK_EQ(m,dim_phi(t),"");
  if(&tt) CHECK_EQ(m,tt.N,"");

  if(&J){ //we also need to return the Jacobian
    J.resize(m,k+1,n).setZero();
    //-- transition costs of trajectory
    for(uint j=0;j<m-1;j++){
      if(kX==1){ J(j,1,0) = p->eval(x_bar(1,0),true)(j);  J(j,0,0) = -p->eval(x_bar(0,0),true)(j);}
      if(kX==2){ J(j,2,0) = p->eval(x_bar(2,0),true)(j);  J(j,1,0) = -2.*p->eval(x_bar(1,0),true)(j);  J(j,0,0) = p->eval(x_bar(0,0),true)(j); }
      if(kX==3){ J(j,3,0) = p->eval(x_bar(3,0),true)(j);  J(j,2,0) = -3.*p->eval(x_bar(2,0),true)(j);  J(j,1,0) = +3.*p->eval(x_bar(1,0),true)(j);  J(j,0,0) = -1.*p->eval(x_bar(0,0),true)(j); }
    }
    //-- transition costs of phase
    for(uint j=m-1;j<m;j++){
      if (t<T) {J(j,2,0) = 1.*w;  J(j,1,0) = -2.*w;  J(j,0,0) = 1.*w;}
    }
  }
}
/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#include "komo.h"
#include <Algo/spline.h>
#include <iomanip>
#include <Kin/kin_swift.h>
#include <Kin/taskMaps.h>
#include <Gui/opengl.h>
#include <Kin/taskMap_FixSwitchedObjects.h>
#include <Kin/taskMap_QuaternionNorms.h>
#include <Kin/taskMap_AboveBox.h>
#include <Kin/taskMap_AlignStacking.h>
#include <Kin/taskMap_GJK.h>
#include <Optim/optimization.h>
#include <Optim/convert.h>

//===========================================================================

double height(mlr::Shape* s){
  CHECK(s,"");
  return s->size(2);// + s->size(3);
}

KOMO::KOMO() : T(0), tau(0.), k_order(2), useSwift(true), opt(NULL), gl(NULL), verbose(1), komo_problem(*this){
  verbose = mlr::getParameter<int>("KOMO/verbose",1);
}

KOMO::~KOMO(){
  if(gl) delete gl;
  if(opt) delete opt;
}

KOMO::KOMO(const Graph& specs) : KOMO(){
  init(specs);
//  reset();
//  CHECK(x.N,"");
}

void KOMO::init(const Graph& specs){
//  specs = _specs;

  Graph &glob = specs.get<Graph>("KOMO");
  stepsPerPhase=glob.get<double>("T");
  double duration=glob.get<double>("duration");
  maxPhase=glob.get<double>("phases", 1.);
  k_order=glob.get<double>("k_order", 2);

  if(glob["model"]){
    mlr::FileToken model = glob.get<mlr::FileToken>("model");
    world.read(model);
  }else{
    world.init(specs);
  }

  if(glob["meldFixedJoints"]){
    world.meldFixedJoints();
    world.removeUselessBodies();
  }

  if(glob["makeConvexHulls"])
    makeConvexHulls(world.shapes);

  if(glob["computeOptimalSSBoxes"]){
    NIY;
    //for(mlr::Shape *s: world.shapes) s->mesh.computeOptimalSSBox(s->mesh.V);
    world.gl().watch();
  }

  if(glob["activateAllContacts"])
    for(mlr::Shape *s:world.shapes) s->cont=true;

  world.swift().initActivations(world);
  FILE("z.komo.model") <<world;

//  if(MP) delete MP;
//  MP = new KOMO(world);
  if(stepsPerPhase>=0) setTiming(maxPhase, stepsPerPhase, duration);
//  MP->k_order=k_order;

  for(Node *n:specs) parseTask(n, stepsPerPhase);
}

void KOMO::setFact(const char* fact){
  Graph specs;
  specs.readNode(STRING(fact));
  parseTask(specs.last());
}

void KOMO::setModel(const mlr::KinematicWorld& K,
                    bool _useSwift,
                    bool meldFixedJoints, bool makeConvexHulls, bool computeOptimalSSBoxes, bool activateAllContacts){

  world.copy(K);

  useSwift = _useSwift;

  if(meldFixedJoints){
    world.meldFixedJoints();
    world.removeUselessBodies();
  }

  if(makeConvexHulls){
    ::makeConvexHulls(world.shapes);
  }
  computeMeshNormals(world.shapes);

  if(computeOptimalSSBoxes){
    NIY;
    //for(mlr::Shape *s: world.shapes) s->mesh.computeOptimalSSBox(s->mesh.V);
    world.gl().watch();
  }

  if(activateAllContacts){
    for(mlr::Shape *s:world.shapes) s->cont=true;
    world.swift().initActivations(world);
  }

  FILE("z.komo.model") <<world;
}

void KOMO::useJointGroups(const StringA& groupNames, bool OnlyTheseOrNotThese){
  for(mlr::Joint *j:world.joints){
    bool lock;
    if(OnlyTheseOrNotThese){ //only these
      lock=true;
      for(const mlr::String& s:groupNames) if(j->ats.getNode(s)){ lock=false; break; }
    }else{
      lock=false;
      for(const mlr::String& s:groupNames) if(j->ats.getNode(s)){ lock=true; break; }
    }
    if(lock) j->makeRigid();
//        j->type = mlr::JT_rigid;
  }
  world.qdim.clear();
  world.q.clear();
  world.qdot.clear();

  world.getJointState();

//  world.meldFixedJoints();
//  world.removeUselessBodies();

  FILE("z.komo.model") <<world;
}

void KOMO::setTiming(double _phases, uint _stepsPerPhase, double durationPerPhase, uint _k_order){
  maxPhase = _phases;
  stepsPerPhase = _stepsPerPhase;
  if(stepsPerPhase>=0){
    T = stepsPerPhase*maxPhase;
    CHECK(T, "using T=0 to indicate inverse kinematics is deprecated.");
    tau = durationPerPhase*maxPhase/T;
  }
//    setTiming(stepsPerPhase*maxPhase, durationPerPhase*maxPhase);
  k_order = _k_order;
}

void KOMO::activateCollisions(const char* s1, const char* s2){
  mlr::Shape *sh1 = world.getShapeByName(s1);
  mlr::Shape *sh2 = world.getShapeByName(s2);
  if(sh1 && sh2) world.swift().activate(sh1, sh2);
}

void KOMO::deactivateCollisions(const char* s1, const char* s2){
  mlr::Shape *sh1 = world.getShapeByName(s1);
  mlr::Shape *sh2 = world.getShapeByName(s2);
  if(sh1 && sh2) world.swift().deactivate(sh1, sh2);
}

//===========================================================================
//
// task specs
//

//#define STEP(t) (floor(t*double(stepsPerPhase) + .500001))-1

Task* KOMO::addTask(const char* name, TaskMap *m, const ObjectiveType& termType){
  Task *t = new Task(m, termType);
  t->name=name;
  tasks.append(t);
  return t;
}

bool KOMO::parseTask(const Node *n, int stepsPerPhase){
  if(stepsPerPhase==-1) stepsPerPhase=T;
  //-- task?
  Task *task = Task::newTask(n, world, stepsPerPhase, T);
  if(task){
    tasks.append(task);
    return true;
  }
  //-- switch?
  mlr::KinematicSwitch *sw = mlr::KinematicSwitch::newSwitch(n, world, stepsPerPhase, T);
  if(sw){
    switches.append(sw);
    return true;
  }
//  LOG(-1) <<"task spec '" <<*n <<"' could not be parsed";
  return false;
}

Task *KOMO::setTask(double startTime, double endTime, TaskMap *map, ObjectiveType type, const arr& target, double prec, uint order){
  CHECK(k_order>=order,"");
  map->order = order;
  Task *task = addTask(map->shortTag(world), map, type);
  task->setCostSpecs(startTime, endTime, stepsPerPhase, T, target, prec);
  return task;
}

void KOMO::setKinematicSwitch(double time, bool before, const char* type, const char* ref1, const char* ref2, const mlr::Transformation& jFrom, const mlr::Transformation& jTo){
  mlr::KinematicSwitch *sw = mlr::KinematicSwitch::newSwitch(type, ref1, ref2, world, 0/*STEP(time)+(before?0:1)*/, jFrom, jTo );
  sw->setTimeOfApplication(time, before, stepsPerPhase, T);
  switches.append(sw);
}

void KOMO::setKS_placeOn(double time, bool before, const char* obj, const char* table, bool actuated){
  //disconnect object from grasp ref
  setKinematicSwitch(time, before, "delete", NULL, obj);

  //connect object to table
  mlr::Transformation rel = 0;
  rel.addRelativeTranslation( 0., 0., .5*(height(world.getShapeByName(obj)) + height(world.getShapeByName(table))));
  if(!actuated)
    setKinematicSwitch(time, before, "transXYPhiZero", table, obj, rel );
  else
    setKinematicSwitch(time, before, "transXYPhiActuated", table, obj, rel );
}

void KOMO::setKS_slider(double time, bool before, const char* obj, const char* slider, const char* table){
  //disconnect object from grasp ref
  setKinematicSwitch(time, before, "delete", NULL, obj);

  //the two slider objects
  mlr::String slidera = STRING(slider <<'a');
  mlr::String sliderb = STRING(slider <<'b');

  //disconnect object from grasp ref
  setKinematicSwitch(time, before, "delete", NULL, slidera);

  mlr::Transformation rel = 0;
  rel.addRelativeTranslation( 0., 0., .5*(height(world.getShapeByName(obj)) + height(world.getShapeByName(table))));

  setKinematicSwitch(time, true, "transXYPhiZero", table, slidera, rel);
  setKinematicSwitch(time, true, "hingeZZero", sliderb, obj);

//  setKinematicSwitch(time, before, "sliderMechanism", table, obj, rel );

//  if(!actuated)
//    setKinematicSwitch(time, before, "hingeZZero", slider, obj, rel );
//  else
//    setKinematicSwitch(time, before, "transXActuated", slider, obj, rel );
}

void KOMO::setHoming(double startTime, double endTime, double prec){
  uintA bodies;
  for(mlr::Joint *j:world.joints) if(j->qDim()>0) bodies.append(j->to->index);
  setTask(startTime, endTime, new TaskMap_qItself(bodies, true), OT_sumOfSqr, NoArr, prec); //world.q, prec);
}

void KOMO::setSquaredQAccelerations(double startTime, double endTime, double prec){
  CHECK(k_order>=2,"");
  setTask(startTime, endTime, new TaskMap_Transition(world), OT_sumOfSqr, NoArr, prec, 2);
}

void KOMO::setSquaredQVelocities(double startTime, double endTime, double prec){
  auto *map = new TaskMap_Transition(world);
  map->velCoeff = 1.;
  map->accCoeff = 0.;
  setTask(startTime, endTime, map, OT_sumOfSqr, NoArr, prec, 1);
}

void KOMO::setFixEffectiveJoints(double startTime, double endTime, double prec){
  auto *map = new TaskMap_Transition(world, true);
  map->velCoeff = 1.;
  map->accCoeff = 0.;
  setTask(startTime, endTime, map, OT_eq, NoArr, prec, 1);
}

void KOMO::setFixSwitchedObjects(double startTime, double endTime, double prec){
    setTask(startTime, endTime, new TaskMap_FixSwichedObjects(), OT_eq, NoArr, prec, 1);
}

void KOMO::setSquaredQuaternionNorms(double startTime, double endTime, double prec){
    setTask(startTime, endTime, new TaskMap_QuaternionNorms(), OT_sumOfSqr, NoArr, prec);
}

void KOMO::setHoldStill(double startTime, double endTime, const char* shape, double prec){
  mlr::Shape *s = world.getShapeByName(shape);
  setTask(startTime, endTime, new TaskMap_qItself(TUP(s->body->index)), OT_sumOfSqr, NoArr, prec, 1);
}

void KOMO::setPosition(double startTime, double endTime, const char* shape, const char* shapeRel, ObjectiveType type, const arr& target, double prec){
  setTask(startTime, endTime, new TaskMap_Default(posTMT, world, shape, NoVector, shapeRel, NoVector), type, target, prec);
}

void KOMO::setVelocity(double startTime, double endTime, const char* shape, const char* shapeRel, ObjectiveType type, const arr& target, double prec){
  setTask(startTime, endTime, new TaskMap_Default(posTMT, world, shape, NoVector, shapeRel, NoVector), type, target, prec, 1);
}

void KOMO::setLastTaskToBeVelocity(){
  tasks.last()->map->order = 1; //set to be velocity!
}

/// a standard pick up: lower-attached-lift; centered, from top
void KOMO::setGrasp(double time, const char* endeffRef, const char* object, int verbose, double weightFromTop, double timeToLift){
  if(verbose>0) cout <<"KOMO_setGrasp t=" <<time <<" endeff=" <<endeffRef <<" obj=" <<object <<endl;
//  mlr::String& endeffRef = world.getShapeByName(graspRef)->body->inLinks.first()->from->shapes.first()->name;

  //-- position the hand & graspRef
  //hand upright
  setTask(time, time, new TaskMap_Default(vecTMT, world, endeffRef, Vector_z), OT_sumOfSqr, {0.,0.,1.}, weightFromTop);

  //hand center at object center (could be replaced by touch)
//  setTask(time, time, new TaskMap_Default(posDiffTMT, world, endeffRef, NoVector, object, NoVector), OT_eq, NoArr, 1e3);

  //hand grip axis orthogonal to object length axis
//  setTask(time, time, new TaskMap_Default(vecAlignTMT, world, endeffRef, Vector_x, object, Vector_x), OT_sumOfSqr, NoArr, 1e1);
  //hand grip axis orthogonal to object length axis
//  setTask(time, time, new TaskMap_Default(vecAlignTMT, world, endeffRef, Vector_y, object, Vector_x), OT_sumOfSqr, {-1.}, 1e1);

  //hand touches object
//  mlr::Shape *endeffShape = world.getShapeByName(endeffRef)->body->shapes.first();
//  setTask(time, time, new TaskMap_GJK(endeffShape, world.getShapeByName(object), false), OT_eq, NoArr, 1e3);


  //disconnect object from table
  setKinematicSwitch(time, true, "delete", NULL, object);
  //connect graspRef with object
#if 1
  setKinematicSwitch(time, true, "ballZero", endeffRef, object);
#else
  setKinematicSwitch(time, true, "freeZero", endeffRef, object);
  setTask(time, time, new TaskMap_Default(posDiffTMT, world, endeffRef, NoVector, object, NoVector), OT_eq, NoArr, 1e3);
#endif

  if(stepsPerPhase>2){ //velocities down and up
    setTask(time-timeToLift, time, new TaskMap_Default(posTMT, world, endeffRef), OT_sumOfSqr, {0.,0.,-.1}, 1e1, 1); //move down
    setTask(time, time+timeToLift, new TaskMap_Default(posTMT, world, object), OT_sumOfSqr, {0.,0.,.1}, 1e1, 1); // move up
  }
}

/// slide on table while grasping rigidly (kinematic loop)
void KOMO::setGraspSlide(double startTime, double endTime, const char* endeffRef, const char* object, const char* placeRef, int verbose, double weightFromTop){
  if(verbose>0) cout <<"KOMO_setGraspSlide t=" <<startTime <<" endeff=" <<endeffRef <<" obj=" <<object <<endl;

  //-- grasp part
  //hand upright
  setTask(startTime, startTime, new TaskMap_Default(vecTMT, world, endeffRef, Vector_z), OT_sumOfSqr, {0.,0.,1.}, weightFromTop);

  //disconnect object from table
  setKinematicSwitch(startTime, true, "delete", placeRef, object);
  //connect graspRef with object
  setKinematicSwitch(startTime, true, "ballZero", endeffRef, object);

  //-- place part
  //place inside box support
  setTask(endTime, endTime, new TaskMap_AboveBox(world, object, placeRef), OT_ineq, NoArr, 1e2);

  //disconnect object from grasp ref
  setKinematicSwitch(endTime, true, "delete", endeffRef, object);

  //connect object to table
  mlr::Transformation rel = 0;
  rel.pos.set(0,0, .5*(height(world.getShapeByName(object)) + height(world.getShapeByName(placeRef))));
  setKinematicSwitch(endTime, true, "transXYPhiZero", placeRef, object, rel );

  //-- slide constraints!
  setTask(startTime, endTime,
          new TaskMap_LinTrans(new TaskMap_Default(posDiffTMT, world, object, NoVector, placeRef), ~ARR(0,0,1), ARR(0)),
                               OT_sumOfSqr, ARR(rel.pos.z), 1e2);

  if(stepsPerPhase>2){ //velocities down and up
    setTask(startTime-.15, startTime, new TaskMap_Default(posTMT, world, endeffRef), OT_sumOfSqr, {0.,0.,-.1}, 1e1, 1); //move down
    setTask(endTime, endTime+.15, new TaskMap_Default(posTMT, world, endeffRef), OT_sumOfSqr, {0.,0.,.1}, 1e1, 1); // move up
  }
}

/// standard place on a table
void KOMO::setPlace(double time, const char* endeffRef, const char* object, const char* placeRef, int verbose){
  if(verbose>0) cout <<"KOMO_setPlace t=" <<time <<" endeff=" <<endeffRef <<" obj=" <<object <<" place=" <<placeRef <<endl;

  if(stepsPerPhase>2){ //velocities down and up
    setTask(time-.15, time, new TaskMap_Default(posTMT, world, object), OT_sumOfSqr, {0.,0.,-.1}, 1e1, 1); //move down
    setTask(time, time+.15, new TaskMap_Default(posTMT, world, endeffRef), OT_sumOfSqr, {0.,0.,.1}, 1e1, 1); // move up
  }

  //place roughly at center ;-(
//  setTask(time, time, new TaskMap_Default(posDiffTMT, world, object, NoVector, placeRef, NoVector), OT_sumOfSqr, {0.,0.,.1}, 1e-1);

  //place upright
  setTask(time-.02, time, new TaskMap_Default(vecTMT, world, object, Vector_z), OT_sumOfSqr, {0.,0.,1.}, 1e2);

  //place inside box support
  setTask(time, time, new TaskMap_AboveBox(world, object, placeRef), OT_ineq, NoArr, 1e2);

  //disconnect object from grasp ref
  setKinematicSwitch(time, true, "delete", endeffRef, object);

  //connect object to table
  mlr::Transformation rel = 0;
  rel.pos.set(0,0, .5*(height(world.getShapeByName(object)) + height(world.getShapeByName(placeRef))));
  setKinematicSwitch(time, true, "transXYPhiZero", placeRef, object, rel );
}

/// place with a specific relative pose -> no effective DOFs!
void KOMO::setPlaceFixed(double time, const char* endeffRef, const char* object, const char* placeRef, const mlr::Transformation& relPose, int verbose){
  if(verbose>0) cout <<"KOMO_setPlace t=" <<time <<" endeff=" <<endeffRef <<" obj=" <<object <<" place=" <<placeRef <<endl;

  //disconnect object from grasp ref
  setKinematicSwitch(time, true, "delete", endeffRef, object);

  //connect object to table
  setKinematicSwitch(time, true, "rigidZero", placeRef, object, relPose );

  if(stepsPerPhase>2){ //velocities down and up
    setTask(time-.15, time, new TaskMap_Default(posTMT, world, object), OT_sumOfSqr, {0.,0.,-.1}, 1e1, 1); //move down
    setTask(time, time+.15, new TaskMap_Default(posTMT, world, endeffRef), OT_sumOfSqr, {0.,0.,.1}, 1e1, 1); // move up
  }
}

/// switch attachemend (-> ball eDOF)
void KOMO::setHandover(double time, const char* oldHolder, const char* object, const char* newHolder, int verbose){
  if(verbose>0) cout <<"KOMO_setHandover t=" <<time <<" oldHolder=" <<oldHolder <<" obj=" <<object <<" newHolder=" <<newHolder <<endl;

  //hand center at object center (could be replaced by touch)

  //disconnect object from table
  setKinematicSwitch(time, true, "delete", oldHolder, object);
  //connect graspRef with object
#if 1
  setKinematicSwitch(time, true, "ballZero", newHolder, object); //why does this sometimes lead to worse motions?
#else
  setKinematicSwitch(time, true, "freeZero", newHolder, object);
  setTask(time, time, new TaskMap_Default(posDiffTMT, world, newHolder, NoVector, object, NoVector), OT_eq, NoArr, 1e3);
#endif

  if(stepsPerPhase>2){ //velocities: no motion
    setTask(time-.15, time+.15, new TaskMap_Default(posTMT, world, object), OT_sumOfSqr, {0.,0.,0.}, 1e1, 1); // no motion
  }

}

void KOMO::setPush(double startTime, double endTime, const char* stick, const char* object, const char* table, int verbose){
  if(verbose>0) cout <<"KOMO_setPush t=" <<startTime <<" stick=" <<stick <<" object=" <<object <<" table=" <<table <<endl;

  setTask(startTime, startTime+1., new TaskMap_Default(vecAlignTMT, world, stick, -Vector_y, "slider1b", Vector_x), OT_sumOfSqr, {1.}, 1e2);
  setTask(startTime, startTime+1., new TaskMap_Default(vecAlignTMT, world, stick, Vector_z, NULL, Vector_z), OT_sumOfSqr, {1.}, 1e2);
  setTask(startTime, startTime+1., new TaskMap_Default(posDiffTMT, world, stick, NoVector, "slider1b", {.12, .0, .0}), OT_sumOfSqr, {}, 1e2);

  setKS_slider(startTime, true, object, "slider1", table);

  if(stepsPerPhase>2){ //velocities down and up
    setTask(startTime-.3, startTime, new TaskMap_Default(posTMT, world, stick), OT_sumOfSqr, {0.,0.,-.1}, 1e2, 1); //move down
//    setTask(time, time+.15, new TaskMap_Default(posTMT, world, endeffRef), OT_sumOfSqr, {0.,0.,.1}, 1e1, 1); // move up
  }
}


void KOMO::setAttach(double time, const char* endeff, const char* object1, const char* object2, mlr::Transformation& rel, int verbose){
  if(verbose>0) cout <<"KOMO_setAttach t=" <<time <<" endeff=" <<endeff <<" obj1=" <<object1 <<" obj2=" <<object2 <<endl;

  //hand center at object center (could be replaced by touch)
//  setTask(time, time, new TaskMap_Default(posTMT, world, object2, NoVector, object1, NoVector), OT_sumOfSqr, rel.pos.getArr(), 1e3);
//  setTask(time, time, new TaskMap_Default(quatDiffTMT, world, object2, NoVector, object1, NoVector), OT_sumOfSqr, conv_quat2arr(rel.rot), 1e3);

//  setTask(time, time, new TaskMap_Default(vecAlignTMT, world, newHolder, Vector_y, object, Vector_x), OT_sumOfSqr, {-1.}, 1e1);

  //disconnect object from grasp ref
  setKinematicSwitch(time, true, "delete", endeff, object2);

//  mlr::Transformation rel = 0;
//  rel.addRelativeTranslation( 0., 0., .5*(height(world.getShapeByName(object)) + height(world.getShapeByName(placeRef))));
  setKinematicSwitch(time, true, "rigidZero", object1, object2, rel );

}

void KOMO::setSlowAround(double time, double delta, double prec){
  if(stepsPerPhase>2) //otherwise: no velocities
    setTask(time-delta, time+delta, new TaskMap_qItself(), OT_sumOfSqr, NoArr, prec, 1);
  //#    _MinSumOfSqr_qItself_vel(MinSumOfSqr qItself){ order=1 time=[0.98 1] scale=1e1 } #slow down
}

void KOMO::setFine_grasp(double time, const char* endeff, const char* object, double above, double gripSize, const char* gripper, const char* gripper2){
  double t1=-.25; //time when gripper is positined above
  double t2=-.1;  //time when gripper is lowered
  double t3=-.05; //time when gripper is closed

  //position above
  setTask(time+t1, 1., new TaskMap_Default(vecTMT, world, endeff, Vector_z), OT_sumOfSqr, {0.,0.,1.}, 1e0);
  setTask(time+t1, t1, new TaskMap_Default(posDiffTMT, world, endeff, NoVector, object, NoVector), OT_sumOfSqr, {0.,0.,above+.1}, 1e3);
  setTask(time+t1, 1., new TaskMap_Default(vecAlignTMT, world, endeff, Vector_x, object, Vector_y), OT_sumOfSqr, NoArr, 1e1);
  setTask(time+t1, 1., new TaskMap_Default(vecAlignTMT, world, endeff, Vector_x, object, Vector_z), OT_sumOfSqr, NoArr, 1e1);
  //open gripper
  if(gripper)  setTask(time+t1, .85, new TaskMap_qItself(QIP_byJointNames, {gripper}, world), OT_sumOfSqr, {gripSize + .05});
  if(gripper2) setTask(time+t1, .85, new TaskMap_qItself(QIP_byJointNames, {gripper2}, world), OT_sumOfSqr, {::asin((gripSize + .05)/(2.*.10))});
  //lower
  setTask(time+t2, 1., new TaskMap_Default(posDiffTMT, world, endeff, NoVector, object, NoVector), OT_sumOfSqr, {0.,0.,above}, 1e3);
  //close gripper
  if(gripper)  setTask(time+t3, 1., new TaskMap_qItself(QIP_byJointNames, {gripper}, world), OT_sumOfSqr, {gripSize});
  if(gripper2) setTask(time+t3, 1., new TaskMap_qItself(QIP_byJointNames, {gripper2}, world), OT_sumOfSqr, {::asin((gripSize)/(2.*.10))});
  setSlowAround(time, .05, 1e3);
}

/// translate a list of facts (typically facts in a FOL state) to LGP tasks
void KOMO::setAbstractTask(double phase, const Graph& facts, int verbose){
//  CHECK(phase<=maxPhase,"");
//  listWrite(facts, cout,"\n");  cout <<endl;
  for(Node *n:facts){
    if(!n->parents.N) continue;
    StringL symbols;
    for(Node *p:n->parents) symbols.append(&p->keys.last());
    if(n->keys.N && n->keys.last().startsWith("komo")){
      double time=n->get<double>(); //komo tag needs to be double valued!
      if(n->keys.last()=="komoGrasp")         setGrasp(phase+time, *symbols(0), *symbols(1), verbose);
      else if(n->keys.last()=="komoPlace")    setPlace(phase+time, *symbols(0), *symbols(1), *symbols(2), verbose);
      else if(n->keys.last()=="komoHandover") setHandover(phase+time, *symbols(0), *symbols(1), *symbols(2), verbose);
      else if(n->keys.last()=="komoPush")     setPush(phase+time, phase+time+1., *symbols(0), *symbols(1), *symbols(2), verbose); //TODO: the +1. assumes pushes always have duration 1
      else if(n->keys.last()=="komoAttach"){
        Node *attachableSymbol = facts.getNode("attachable");
        CHECK(attachableSymbol!=NULL,"");
        Node *attachableFact = facts.getEdge({attachableSymbol, n->parents(1), n->parents(2)});
        mlr::Transformation rel = attachableFact->get<mlr::Transformation>();
        setAttach(phase+time, *symbols(0), *symbols(1), *symbols(2), rel, verbose);
      }else HALT("UNKNOWN komo TAG: '" <<n->keys.last() <<"'");
    }
  }
}

void KOMO::setAlign(double startTime, double endTime, const char* shape, const arr& whichAxis, const char* shapeRel, const arr& whichAxisRel, ObjectiveType type, const arr& target, double prec){
#if 0
  mlr::String map;
  map <<"map=vecAlign ref1="<<shape;
  if(whichAxis) map <<" vec1=[" <<whichAxis <<']';
  if(shapeRel) map <<" ref2=" <<shapeRel <<" vec2=" <<;
  if(whichAxisRel) map <<" vec2=[" <<whichAxisRel <<']';
  setTask(startTime, endTime, map, type, target, prec);
#else
  setTask(startTime, endTime, new TaskMap_Default(vecAlignTMT, world, shape, mlr::Vector(whichAxis), shapeRel, mlr::Vector(whichAxisRel)), type, target, prec);
#endif

}

void KOMO::setTouch(double startTime, double endTime, const char* shape1, const char* shape2, ObjectiveType type, const arr& target, double prec){
  setTask(startTime, endTime, new TaskMap_GJK(world, shape1, shape2, true), type, target, prec);
}

void KOMO::setAlignedStacking(double time, const char* object, ObjectiveType type, double prec){
  setTask(time, time, new TaskMap_AlignStacking(world, object), type, NoArr, prec);
}

void KOMO::setCollisions(bool hardConstraint, double margin, double prec){
  if(hardConstraint){ //interpreted as hard constraint (default)
    setTask(0., -1., new CollisionConstraint(margin), OT_ineq, NoArr, prec);
  }else{ //cost term
    setTask(0., -1., new TaskMap_Proxy(allPTMT, {0u}, margin), OT_sumOfSqr, NoArr, prec);
  }
}

void KOMO::setLimits(bool hardConstraint, double margin, double prec){
  if(hardConstraint){ //interpreted as hard constraint (default)
    setTask(0., -1., new LimitsConstraint(margin), OT_ineq, NoArr, prec);
  }else{ //cost term
    NIY;
//    setTask(0., -1., new TaskMap_Proxy(allPTMT, {0u}, margin), OT_sumOfSqr, NoArr, prec);
  }
}

//===========================================================================
//
// config
//


void KOMO::setConfigFromFile(){
//  Graph model;
//  FILE(mlr::getParameter<mlr::String>("KOMO/modelfile")) >>model;
  mlr::KinematicWorld W(mlr::getParameter<mlr::String>("KOMO/modelfile"));
  setModel(
        W,
        mlr::getParameter<bool>("KOMO/useSwift", true),
        mlr::getParameter<bool>("KOMO/meldFixedJoints", false),
        mlr::getParameter<bool>("KOMO/makeConvexHulls", true),
        mlr::getParameter<bool>("KOMO/computeOptimalSSBoxes", false),
        mlr::getParameter<bool>("KOMO/activateAllContact", false)
        );
  setTiming(
        mlr::getParameter<uint>("KOMO/phases"),
        mlr::getParameter<uint>("KOMO/stepsPerPhase"),
        mlr::getParameter<double>("KOMO/durationPerPhase", 5.),
        mlr::getParameter<uint>("KOMO/k_order", 2)
        );
}

void KOMO::setPoseOpt(){
  setTiming(1., 2, 5., 1);
  setFixEffectiveJoints();
  setFixSwitchedObjects();
  setSquaredQVelocities();
}

void KOMO::setSequenceOpt(double _phases){
  setTiming(_phases, 2, 5., 1);
  setFixEffectiveJoints();
  setFixSwitchedObjects();
  setSquaredQVelocities();
}

void KOMO::setPathOpt(double _phases, uint stepsPerPhase, double timePerPhase){
  setTiming(_phases, stepsPerPhase, timePerPhase, 2);
  setFixEffectiveJoints();
  setFixSwitchedObjects();
  setSquaredQAccelerations();
}

void setTasks(KOMO& MP,
              mlr::Shape &endeff,
              mlr::Shape& target,
              byte whichAxesToAlign,
              uint iterate,
              int timeSteps,
              double duration){

  //-- parameters
  double posPrec = mlr::getParameter<double>("KOMO/moveTo/precision", 1e3);
  double colPrec = mlr::getParameter<double>("KOMO/moveTo/collisionPrecision", -1e0);
  double margin = mlr::getParameter<double>("KOMO/moveTo/collisionMargin", .1);
  double zeroVelPrec = mlr::getParameter<double>("KOMO/moveTo/finalVelocityZeroPrecision", 1e1);
  double alignPrec = mlr::getParameter<double>("KOMO/moveTo/alignPrecision", 1e3);

  //-- set up the KOMO
  target.cont=false; //turn off contact penalization with the target

//  MP.world.swift().initActivations(MP.world);
  //MP.world.watch(false);

  MP.setTiming(1., mlr::getParameter<uint>("timeSteps", 50), mlr::getParameter<double>("duration", 5.));
  if(timeSteps>=0) MP.setTiming(1., timeSteps, duration);
  if(timeSteps==0) MP.k_order=1;

  Task *t;

  t = MP.addTask("transitions", new TaskMap_Transition(MP.world), OT_sumOfSqr);
  if(timeSteps!=0){
    t->map->order=2; //make this an acceleration task!
  }else{
    t->map->order=1; //make this a velocity task!
  }
  t->setCostSpecs(0, MP.T-1, {0.}, 1e0);

  if(timeSteps!=0){
    t = MP.addTask("final_vel", new TaskMap_qItself(), OT_sumOfSqr);
    t->map->order=1; //make this a velocity task!
    t->setCostSpecs(MP.T-4, MP.T-1, {0.}, zeroVelPrec);
  }

  if(colPrec<0){ //interpreted as hard constraint (default)
    t = MP.addTask("collisionConstraints", new CollisionConstraint(margin), OT_ineq);
    t->setCostSpecs(0, MP.T-1, {0.}, 1.);
  }else{ //cost term
    t = MP.addTask("collision", new TaskMap_Proxy(allPTMT, {0u}, margin), OT_sumOfSqr);
    t->setCostSpecs(0, MP.T-1, {0.}, colPrec);
  }

  t = MP.addTask("endeff_pos", new TaskMap_Default(posTMT, endeff.index, NoVector, target.index, NoVector), OT_sumOfSqr);
  t->setCostSpecs(MP.T-1, MP.T-1, {0.}, posPrec);


  for(uint i=0;i<3;i++) if(whichAxesToAlign&(1<<i)){
    mlr::Vector axis;
    axis.setZero();
    axis(i)=1.;
    t = MP.addTask(STRING("endeff_align_"<<i),
                   new TaskMap_Default(vecAlignTMT, endeff.index, axis, target.index, axis),
                   OT_sumOfSqr);
    t->setCostSpecs(MP.T-1, MP.T-1, {1.}, alignPrec);
  }
}

void KOMO::setMoveTo(mlr::KinematicWorld& world, mlr::Shape& endeff, mlr::Shape& target, byte whichAxesToAlign){
//  if(MP) delete MP;
//  MP = new KOMO(world);
  setModel(world);

  setTasks(*this, endeff, target, whichAxesToAlign, 1, -1, -1.);
  reset();
}

void KOMO::setSpline(uint splineT){
  mlr::Spline S;
  S.setUniformNonperiodicBasis(T-1, splineT, 2);
  uint n=dim_x(0);
  splineB = zeros(S.basis.d0*n, S.basis.d1*n);
  for(uint i=0;i<S.basis.d0;i++) for(uint j=0;j<S.basis.d1;j++)
    splineB.setMatrixBlock(S.basis(i,j)*eye(n,n), i*n, j*n);
  z = pseudoInverse(splineB) * x;
}

void KOMO::reset(){
  x = getInitialization();
  rndGauss(x,.01,true); //don't initialize at a singular config
  if(splineB.N){
    z = pseudoInverse(splineB) * x;
  }
}

void KOMO::run(){
  mlr::KinematicWorld::setJointStateCount=0;
  mlr::timerStart();
  CHECK(T,"");
  if(opt) delete opt;
  if(!splineB.N){
    Convert C(komo_problem);
    opt = new OptConstrained(x, dual, C);
    opt->run();
  }else{
    arr a,b,c,d,e;
    Conv_KOMO_ConstrainedProblem P0(komo_problem);
    Conv_linearlyReparameterize_ConstrainedProblem P(P0, splineB);
    opt = new OptConstrained(z, dual, P);
    opt->run();
  }
  if(verbose>0){
    cout <<"** optimization time=" <<mlr::timerRead()
      <<" setJointStateCount=" <<mlr::KinematicWorld::setJointStateCount <<endl;
  }
  if(verbose>1) cout <<getReport(false);
}


void KOMO::checkGradients(){
  CHECK(T,"");
  if(!splineB.N)
    checkJacobianCP(Convert(komo_problem), x, 1e-4);
  else{
    Conv_KOMO_ConstrainedProblem P0(komo_problem);
    Conv_linearlyReparameterize_ConstrainedProblem P1(P0, splineB);
    checkJacobianCP(P1, z, 1e-4);
  }
}

bool KOMO::displayTrajectory(double delay, bool watch){
//  return displayTrajectory(watch?-1:1, "KOMO planned trajectory", delay);
  const char* tag = "KOMO planned trajectory";
  if(!gl){
    gl = new OpenGL ("KOMO display");
    gl->camera.setDefault();
  }

  for(uint t=0; t<T; t++) {
    gl->clear();
    gl->add(glStandardScene, 0);
    gl->addDrawer(configurations(t+k_order));
    if(delay<0.){
      if(delay<-10.) FILE("z.graph") <<*configurations(t+k_order);
      gl->watch(STRING(tag <<" (time " <<std::setw(3) <<t <<'/' <<T <<')').p);
    }else{
      gl->update(STRING(tag <<" (time " <<std::setw(3) <<t <<'/' <<T <<')').p);
      if(delay) mlr::wait(delay);
    }
  }
  if(watch){
    int key = gl->watch(STRING(tag <<" (time " <<std::setw(3) <<T-1 <<'/' <<T <<')').p);
    return !(key==27 || key=='q');
  }else
    return false;
}


mlr::Camera& KOMO::displayCamera(){
  if(!gl){
    gl = new OpenGL ("KOMO display");
    gl->camera.setDefault();
  }
  return gl->camera;
}

//===========================================================================

#define KOMO KOMO

void KOMO::setupConfigurations(){

  //IMPORTANT: The configurations need to include the k prefix configurations!
  //Therefore configurations(0) is for time=-k and configurations(k+t) is for time=t
  CHECK(!configurations.N,"why setup again?");
//    listDelete(configurations);

  if(useSwift) {
    makeConvexHulls(world.shapes);
    world.swift().setCutoff(2.*mlr::getParameter<double>("swiftCutoff", 0.11));
  }
  computeMeshNormals(world.shapes);

  configurations.append(new mlr::KinematicWorld())->copy(world, true);
  for(uint s=1;s<k_order+T;s++){
    configurations.append(new mlr::KinematicWorld())->copy(*configurations(s-1), true);
    CHECK(configurations(s)==configurations.last(), "");
    //apply potential graph switches
    for(mlr::KinematicSwitch *sw:switches){
      if(sw->timeOfApplication+k_order==s){
        sw->apply(*configurations(s));
        //          if(MP.useSwift) configurations(t)->swift().initActivations(*configurations(t));
      }
    }
    configurations(s)->jointSort();
    configurations(s)->calc_q_from_Q();
    //configurations.last()->checkConsistency();
  }
}

void KOMO::set_x(const arr& x){
  if(!configurations.N) setupConfigurations();
  CHECK_EQ(configurations.N, k_order+T, "configurations are not setup yet");

  //-- set the configurations' states
  uint x_count=0;
  for(uint t=0;t<T;t++){
    uint s = t+k_order;
    uint x_dim = dim_x(t); //configurations(s)->getJointStateDimension();
    if(x_dim){
      if(x.nd==1) configurations(s)->setJointState(x({x_count, x_count+x_dim-1}));
      else        configurations(s)->setJointState(x[t]);
      if(useSwift) configurations(s)->stepSwift();
      x_count += x_dim;
    }
  }
  CHECK_EQ(x_count, x.N, "");
}

void KOMO::reportProxies(std::ostream& os){
    int t=0;
    for(auto &K:configurations){
        os <<" **** KOMO PROXY REPORT t=" <<t-k_order <<endl;
        K->reportProxies(os);
        t++;
    }
}


Graph KOMO::getReport(bool gnuplt, int reportFeatures, std::ostream& featuresOs) {
  if(featureValues.N>1){ //old optimizer -> remove some time..
    arr tmp;
    for(auto& p:featureValues) tmp.append(p);
    featureValues = ARRAY<arr>(tmp);

    ObjectiveTypeA ttmp;
    for(auto& p:featureTypes) ttmp.append(p);
    featureTypes = ARRAY<ObjectiveTypeA>(ttmp);
  }

  bool wasRun = featureValues.N!=0;

  arr phi;
  ObjectiveTypeA tt;
  if(wasRun){
      phi.referTo( featureValues.scalar() );
      tt.referTo( featureTypes.scalar() );
  }

  //-- collect all task costs and constraints
  StringA name; name.resize(tasks.N);
  arr err=zeros(T,tasks.N);
  arr taskC=zeros(tasks.N);
  arr taskG=zeros(tasks.N);
  uint M=0;
  for(uint t=0; t<T; t++){
    for(uint i=0; i<tasks.N; i++) {
      Task *task = tasks(i);
      if(task->prec.N>t && task->prec(t)){
          uint d=0;
          if(wasRun){
              d=task->map->dim_phi(configurations({t,t+k_order}), t);
              for(uint j=0;j<d;j++) CHECK(tt(M+j)==task->type,"");
              if(d){
                  if(task->type==OT_sumOfSqr){
                      for(uint j=0;j<d;j++) err(t,i) += mlr::sqr(phi(M+j)); //sumOfSqr(phi.sub(M,M+d-1));
                      taskC(i) += err(t,i);
                  }
                  if(task->type==OT_ineq){
                      for(uint j=0;j<d;j++) err(t,i) += mlr::MAX(0., phi(M+j));
                      taskG(i) += err(t,i);
                  }
                  if(task->type==OT_eq){
                      for(uint j=0;j<d;j++) err(t,i) += fabs(phi(M+j));
                      taskG(i) += err(t,i);
                  }
                  M += d;
              }
          }
          if(reportFeatures==1){
            featuresOs <<std::setw(4) <<t <<' ' <<std::setw(2) <<i <<' ' <<std::setw(2) <<d
                <<' ' <<std::setw(40) <<task->name
               <<" k=" <<task->map->order <<" ot=" <<task->type <<" prec=" <<std::setw(4) <<task->prec(t);
            if(task->target.N<5) featuresOs <<" y*=[" <<task->target <<']'; else featuresOs<<"y*=[..]";
            featuresOs <<" y^2=" <<err(t,i) <<endl;
        }
      }
    }
  }
  CHECK_EQ(M , phi.N, "");


  //-- generate a report graph
  Graph report;
  double totalC=0., totalG=0.;
  for(uint i=0; i<tasks.N; i++) {
    Task *c = tasks(i);
    Graph *g = &report.newSubgraph({c->name}, {})->value;
    g->newNode<double>({"order"}, {}, c->map->order);
    g->newNode<mlr::String>({"type"}, {}, STRING(ObjectiveTypeString[c->type]));
    g->newNode<double>({"sqrCosts"}, {}, taskC(i));
    g->newNode<double>({"constraints"}, {}, taskG(i));
    totalC += taskC(i);
    totalG += taskG(i);
  }
  report.newNode<double>({"total","sqrCosts"}, {}, totalC);
  report.newNode<double>({"total","constraints"}, {}, totalG);

  //-- write a nice gnuplot file
  ofstream fil("z.costReport");
  //first line: legend
  for(auto c:tasks) fil <<c->name <<' ';
  for(auto c:tasks) if(c->type==OT_ineq && dualSolution.N) fil <<c->name <<"_dual ";
  fil <<endl;

  //rest: just the matrix
  if(!dualSolution.N){
    err.write(fil,NULL,NULL,"  ");
  }else{
    dualSolution.reshape(T, dualSolution.N/(T));
    catCol(err, dualSolution).write(fil,NULL,NULL,"  ");
  }
  fil.close();

  ofstream fil2("z.costReport.plt");
  fil2 <<"set key autotitle columnheader" <<endl;
  fil2 <<"set title 'costReport ( plotting sqrt(costs) )'" <<endl;
  fil2 <<"plot 'z.costReport' \\" <<endl;
  for(uint i=1;i<=tasks.N;i++) fil2 <<(i>1?"  ,''":"     ") <<" u 0:"<<i<<" w l lw 3 lc " <<i <<" lt " <<1-((i/10)%2) <<" \\" <<endl;
  if(dualSolution.N) for(uint i=0;i<tasks.N;i++) fil2 <<"  ,'' u 0:"<<1+tasks.N+i<<" w l \\" <<endl;
  fil2 <<endl;
  fil2.close();

  if(gnuplt){
    cout <<"KOMO Report\n" <<report <<endl;
    gnuplot("load 'z.costReport.plt'");
  }

  return report;
}

arr KOMO::getInitialization(){
  if(!configurations.N) setupConfigurations();
  CHECK_EQ(configurations.N, k_order+T, "configurations are not setup yet");
  arr x;
  for(uint t=0;t<T;t++) x.append(configurations(t+k_order)->getJointState());
  return x;
}

void KOMO::Conv_MotionProblem_KOMO_Problem::getStructure(uintA& variableDimensions, uintA& featureTimes, ObjectiveTypeA& featureTypes){
  variableDimensions.resize(MP.T);
  for(uint t=0;t<MP.T;t++) variableDimensions(t) = MP.configurations(t+MP.k_order)->getJointStateDimension();

  featureTimes.clear();
  featureTypes.clear();
  for(uint t=0;t<MP.T;t++){
    for(Task *task: MP.tasks) if(task->prec.N>t && task->prec(t)){
//      CHECK(task->prec.N<=MP.T,"");
      uint m = task->map->dim_phi(MP.configurations({t,t+MP.k_order}), t); //dimensionality of this task
      featureTimes.append(consts<uint>(t, m));
      featureTypes.append(consts<ObjectiveType>(task->type, m));
    }
  }
  dimPhi = featureTimes.N;
}

void KOMO::Conv_MotionProblem_KOMO_Problem::phi(arr& phi, arrA& J, arrA& H, ObjectiveTypeA& tt, const arr& x){
  //-- set the trajectory
  MP.set_x(x);


  CHECK(dimPhi,"getStructure must be called first");
  phi.resize(dimPhi);
  if(&tt) tt.resize(dimPhi);
  if(&J) J.resize(dimPhi);

  arr y, Jy;
  uint M=0;
  for(uint t=0;t<MP.T;t++){
    for(Task *task: MP.tasks) if(task->prec.N>t && task->prec(t)){
        //TODO: sightly more efficient: pass only the configurations that correspond to the map->order
      task->map->phi(y, (&J?Jy:NoArr), MP.configurations({t,t+MP.k_order}), MP.tau, t);
      if(!y.N) continue;
      if(absMax(y)>1e10) MLR_MSG("WARNING y=" <<y);

      //linear transform (target shift)
      if(task->target.N==1) y -= task->target.elem(0);
      else if(task->target.nd==1) y -= task->target;
      else if(task->target.nd==2) y -= task->target[t];
      y *= sqrt(task->prec(t));

      //write into phi and J
      phi.setVectorBlock(y, M);
      if(&J){
        Jy *= sqrt(task->prec(t));
        if(t<MP.k_order) Jy.delColumns(0,(MP.k_order-t)*MP.configurations(0)->q.N); //delete the columns that correspond to the prefix!!
        for(uint i=0;i<y.N;i++) J(M+i) = Jy[i]; //copy it to J(M+i); which is the Jacobian of the M+i'th feature w.r.t. its variables
      }
      if(&tt) for(uint i=0;i<y.N;i++) tt(M+i) = task->type;

      //counter for features phi
      M += y.N;
    }
  }

  CHECK_EQ(M, dimPhi, "");
  MP.featureValues = ARRAY<arr>(phi);
  if(&tt) MP.featureTypes = ARRAY<ObjectiveTypeA>(tt);
}
#include "task.h"

//===========================================================================

void Task::setCostSpecs(int fromTime,
                        int toTime,
                        const arr& _target,
                        double _prec){
  if(&_target) target = _target; else target = {0.};
  if(fromTime<0) fromTime=0;
  CHECK(toTime>=fromTime,"");
  prec.resize(toTime+1).setZero();
  for(uint t=fromTime;t<=(uint)toTime;t++) prec(t) = _prec;
}

#define STEP(t) (floor(t*double(stepsPerPhase) + .500001))-1

void Task::setCostSpecs(double fromTime, double toTime, int stepsPerPhase, uint T, const arr& _target, double _prec){
  if(stepsPerPhase<0) stepsPerPhase=T;
  if(STEP(toTime)>T-1) LOG(-1) <<"beyond the time!: endTime=" <<toTime <<" phases=" <<double(T)/stepsPerPhase;
  int tFrom = (fromTime<0.?0:STEP(fromTime)+map->order);
  int tTo = (toTime<0.?T-1:STEP(toTime));
  if(tTo<0) tTo=0;
  if(tFrom>tTo && tFrom-tTo<=(int)map->order) tFrom=tTo;

  setCostSpecs(tFrom, tTo, _target, _prec);
}

//===========================================================================

Task* Task::newTask(const Node* specs, const mlr::KinematicWorld& world, int stepsPerPhase, uint T){
  if(specs->parents.N<2) return NULL; //these are not task specs

  //-- check the term type first
  ObjectiveType termType;
  mlr::String& tt=specs->parents(0)->keys.last();
  if(tt=="MinSumOfSqr") termType=OT_sumOfSqr;
  else if(tt=="LowerEqualZero") termType=OT_ineq;
  else if(tt=="EqualZero") termType=OT_eq;
  else return NULL;

  //-- try to crate a map
  TaskMap *map = TaskMap::newTaskMap(specs, world);
  if(!map) return NULL;

  //-- create a task
  Task *task = new Task(map, termType);

  if(specs->keys.N) task->name=specs->keys.last();
  else{
    task->name = map->shortTag(world);
//    for(Node *p:specs->parents) task->name <<'_' <<p->keys.last();
    task ->name<<"_o" <<task->map->order;
  }

  //-- check for additional continuous parameters
  if(specs->isGraph()){
    const Graph& params = specs->graph();
    arr time = params.get<arr>("time",{0.,1.});
    task->setCostSpecs(time(0), time(1), stepsPerPhase, T, params.get<arr>("target", {}), params.get<double>("scale", {1.}));
  }else{
    task->setCostSpecs(0, T-1, {}, 1.);
  }
  return task;
}

//===========================================================================
