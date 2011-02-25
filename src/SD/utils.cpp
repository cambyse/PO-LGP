
#include "utils.h"

/* least greater than A power of 2 */
unsigned long
lgp2(unsigned long a){
  unsigned long r=1;
  do r<<=1; while( (a>>=1) );
  return r;
}

/* least greater than A or equal power of 2 */
unsigned long
lgep2(unsigned long a){
  return lgp2(a-1);
}
