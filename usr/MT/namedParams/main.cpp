#include <iostream>
using namespace std;

struct Options{
  double length;
  int bla;
};
ostream& operator<<(ostream& os, const Options& o){ os <<o.length <<' ' <<o.bla <<endl; return os; }

//#define ADD()

#define _NUM_ARGS2(X,X64,X63,X62,X61,X60,X59,X58,X57,X56,X55,X54,X53,X52,X51,X50,X49,X48,X47,X46,X45,X44,X43,X42,X41,X40,X39,X38,X37,X36,X35,X34,X33,X32,X31,X30,X29,X28,X27,X26,X25,X24,X23,X22,X21,X20,X19,X18,X17,X16,X15,X14,X13,X12,X11,X10,X9,X8,X7,X6,X5,X4,X3,X2,X1,N,...) N
#define _NUM_ARGS(...) _NUM_ARGS2(0,1, __VA_ARGS__ ,64,63,62,61,60,59,58,57,56,55,54,53,52,51,50,49,48,47,46,45,44,43,42,41,40,39,38,37,36,35,34,33,32,31,30,29,28,27,26,25,24,23,22,21,20,19,18,17,16,15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0)

#define _OPT_1(obj)         obj
#define _OPT_2(obj, assign) obj.assign
#define _OPT_3(obj, assign, ...) obj.assign, _OPT_2(obj,__VA_ARGS__)
#define _OPT_4(obj, assign, ...) obj.assign, _OPT_3(obj,__VA_ARGS__)
#define _OPT_N2(obj, N, ...) _OPT_ ## N(obj, __VA_ARGS__)
#define _OPT_N1(obj, N, ...) _OPT_N2(obj, N, __VA_ARGS__) //this forces that _NUM_ARGS is expanded to a number
#define OPT(...)     (_OPT_N1(global_options, _NUM_ARGS(__VA_ARGS__), __VA_ARGS__) , global_options)


Options global_options;


int main (int argc, char *argv[]){

  cout <<OPT(bla=7);
  cout <<OPT(bla=7, bla=5, length=3.);

  return 0;
}
