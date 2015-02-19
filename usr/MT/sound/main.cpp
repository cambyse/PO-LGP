#include <Core/util.h>
#include <Core/array.h>
#include <Media/audio.h>


uintA DUR = {0,2,4,5,7,9,11};

void basic(SineSound& S, Audio& audio){
  S.addNote(440., .1, .0);
  MT::wait(3.);
}

void complex(SineSound& S, Audio& audio){
  uint mode=0;
  for(uint k=0;k<200;k++){
    MT::wait(.1);
    switch(mode){
      case 0:  S.addNote(880.*pow(2.,double(rnd.uni(0,36))/12.));  break;
      case 1:{
        double range = 31. - 30.*fabs((double(k)-100.))/100.;
        S.addNote(880.*pow(2.,double(floor(rnd(0,range)-range/2.))/12.));
      }break;
      case 2:
        S.addNote(880.*pow(2.,double(rnd(2)*12 + DUR(rnd(7)))/12.));
        S.addNote(880.*pow(2.,double(rnd(2)*12 + DUR(rnd(7)))/12.));
        break;
      case 3:
        S.addNote(880.*pow(2.,double(rnd(2)*12 + DUR(rnd(7)))/12.));
        S.addNote(880.*pow(2.,double(((k/7)*12)%24 + DUR(k%7))/12.));
        S.addNote(880.*pow(2.,double((((k+2)/7)*12)%24 + DUR((k+2)%7))/12.));
        break;
    }
  }
}

/*******************************************************************/
int main(void){
  //paTestData data;

  SineSound S;
  Audio audio;

  audio.open(S);

  basic(S, audio);

  audio.close();

  return 0;
}
