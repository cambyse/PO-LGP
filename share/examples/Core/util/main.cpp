#include <Core/util_t.h>
#include <math.h>

void TEST(String){
  //-- basic IO
  String s("4.123, ");                     // create the string
  String t1,t2;
  t1 <<s <<"length=" <<s.N <<'|' <<endl;   // piping into a string (including the endl '\n')
  t2 <<s <<"length=7|" <<endl;
  cout <<t1 <<t2;                          // outputting a string
  CHECK(t1==t2,"");

  //-- parsing from a string
  MT::String tmp;
  double a,b;
  s="a=1.2, b=3.4, blabla";
  cout <<s <<'|' <<endl;
  s >>(const char*)"a=" >>a >>(const char*)", b=" >>b;  // read things from string
  s >>tmp;      // read string from string (starting at current scan position)
  CHECK_ZERO(a-1.2, 1e-10, "");
  CHECK_ZERO(b-3.4, 1e-10, "");
  CHECK(tmp==", blabla", "");
  cout <<"a=" <<a <<", b=" <<b <<tmp <<'|' <<endl;

  s.resetIstream();   // reset the istream pointer of source string before...
  s >>tmp;            // ...reading string from string (starting at beginning of string)
  CHECK(tmp=="a=1.2, b=3.4, blabla", "");
  cout <<tmp <<'|' <<endl;

  s.resetIstream();   // reset the istream pointer of source string before...
  tmp.read(s,"", " ,;:\n\r"); //stop symbols
  CHECK(tmp=="a=1.2", "");
  cout <<tmp <<'|' <<endl;
}

void TEST(Parameter){
  String p1;
  MT::getParameter(p1, "par", String("default1"));
  CHECK(p1=="default1","");

  String p2;
  MT::getParameter(p2, "h", String("def2"));
  CHECK(p2=="def2","");

  cout <<p1 <<endl <<p2 <<endl;
}

void TEST(Timer){
  MT::timerStart();
  for(uint i=0;i<4;i++){
    cout <<"i=" <<i <<flush;
    for(uint j=0;j<10000000;j++){ j+=10; j-=10; } //do something stupid
    MT::wait(.5);
    cout <<" cpu timer reads " <<MT::timerRead(false) <<"sec" <<endl;
    if(i==1){ MT::timerPause(); cout <<"timer paused" <<endl; }
    if(i==2){ MT::timerResume(); cout <<"timer resumed" <<endl; }
  }
  double t=MT::timerRead();
  CHECK_ZERO(MT::realTime()-2., .5, "wait failed");
  CHECK(t>0. && t<1.,"no cpu time measured");
}

int MAIN(int argn,char** argv){
  MT::initCmdLine(argn,argv);

  uint double_size=sizeof(double);
  uint long_int_size=sizeof(long);
  CHECK(double_size==8,"oh oh");
  CHECK(long_int_size==4,"oh oh");
  cout <<"double size: " <<double_size <<"\nlong int size: " <<long_int_size <<endl;
  
  testString();
  testParameter();
  testTimer();

  return 0;
}
