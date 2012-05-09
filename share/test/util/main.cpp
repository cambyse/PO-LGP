#define MT_IMPLEMENT_TEMPLATES
#include<MT/util.h>

void testRnd(){
  std::ofstream os("z");
  double x;
  for(x=-5.;x<5.;x+=.003) os <<x <<' ' <<MT::gaussInt(x) <<std::endl;
}

void testString(){
  String s("4.123, ");                   // create the string
  cout <<s <<"length=" <<s.N <<'|' <<endl;   // output the string
  cout <<s <<"length=7|" <<endl;

  String tmp;
  tmp <<s <<"length=" <<s.N;
  //tmp=STRING(s <<"length=" <<s.N); // pipe into temporary string String()
  cout <<tmp <<'|' <<endl;

  double a,b;
  s="a=1.2, b=3.4, blabla";
  cout <<s <<'|' <<endl;

  s >>(const char*)"a=" >>a >>(const char*)", b=" >>b;  // read things from string
  s >>tmp;      // read string from string (starting at current scan position)
  cout <<"a=" <<a <<", b=" <<b <<tmp <<'|' <<endl;

  s.resetIstream();   // reset the istream pointer of source string before...
  s >>tmp;      // ...reading string from string (starting at beginning of string)
  cout <<tmp <<'|' <<endl;

  s.resetIstream();   // reset the istream pointer of source string before...
  tmp.read(s," ,;:\n\r");
  cout <<tmp <<'|' <<endl;
}

#if 0
void testBag(){
  MT::Bag bag;

  String str; str <<"this is a string";
  double a[3]={1,2,3};

  bag.add("book",&str,0);
  bag.write(cout);
  bag.add("numbers",a,3);
  bag.add("book",&str,0);
  bag.write(cout);
  bag.add("book",&str,0);
  bag.write(cout);
  MT::Bag::writeExistingItemTypes(cout);

  bag.clear();
  bag.write(cout);
  MT::Bag::writeExistingItemTypes(cout);
}
#endif

#include <MT/array.h>
void testAny(){
  AnyList bag;
  bag.append(anyNew<double>("bla",.125));
  bag.append(anyNew<const char*>("bla","blublu"));
  bag.append(anyNew<double>("bla",ARR(.1,.2,3.,.4).p,4,'['));

  listWrite(bag,cout); cout <<endl;
  AnyList B;
  listClone(B,bag);
  listWrite(B,cout); cout <<endl;
  listDelete(B);

  MT::String buf;
  listWrite(bag,buf);
  cout <<buf <<endl;
  anyListRead(B,buf);
  listWrite(B,cout); cout <<endl;
  
  listDelete(bag);
  listDelete(B);
}

void testParameter(){
  String p1;
  MT::getParameter(p1,"par",String("default1"));

  String p2;
  MT::getParameter(p2,"h",String("def2"));

  cout <<p1 <<endl <<p2 <<endl;
}

void testTimer(){
  MT::timerStart();
  for(uint i=0;i<4;i++){
    cout <<i <<endl;
    MT::wait(1);
    cout <<"timer reads " <<MT::timerRead() <<"sec" <<endl;
    if(i==1){ MT::timerPause(); cout <<"timer paused" <<endl; }
    if(i==2){ MT::timerResume(); cout <<"timer resumed" <<endl; }
  }
}

int main(int argn,char** argv){
  MT::initCmdLine(argn,argv);

  cout <<"double size: " <<sizeof(double) <<"\nlong int size: " <<sizeof(long) <<endl;
  
  testRnd();
  testString();
  testParameter();
  testTimer();
  testAny();

  return 0;
}
