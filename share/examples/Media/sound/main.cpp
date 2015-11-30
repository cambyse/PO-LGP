#include <Core/util.h>
#include <Core/array.h>
#include <Media/audio.h>

/*******************************************************************/

uintA DUR = {0u,2,4,5,7,9,11};

void basic(SineSound& S){
  S.addNote(12);
  mlr::wait(5.);
}

void complex(SineSound& S){
  uint mode=1;
  for(uint k=0;k<210;k++){
    mlr::wait(.1);
    switch(mode){
      case 0:  S.addNote(12 + rnd.uni(0,36));  break;
      case 1:{
        double range = 31. - 30.*fabs((double(k)-100.))/100.;
	if(range<0.) range=0.;
        S.addNote(12 + floor(rnd(0,range)-range/2.));
      }break;
      case 2:
        S.addNote(12 + rnd(2)*12 + DUR(rnd(7)));
        S.addNote(12 + rnd(2)*12 + DUR(rnd(7)));
        break;
      case 3:
        S.addNote(12 + rnd(2)*12 + DUR(rnd(7)));
        S.addNote(12 + ((k/7)*12)%24 + DUR(k%7));
        S.addNote(12 + (((k+2)/7)*12)%24 + DUR((k+2)%7));
        break;
    }
  }
}

/*******************************************************************/

int main(void){

  Sound S;


//  basic(S);
  complex(S);


  return 0;
}
