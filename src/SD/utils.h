#ifndef SD_utils_h
#define SD_utils_h

/* verbosity level macros
 */
#ifndef VERBOSITY 
# define VERBOSITY 5
#endif
#define SD_CMNT(stream) {if(VERBOSITY > 9){ cout<<"comment "<<stream<<endl;}}
#define SD_DBG3(stream) {if(VERBOSITY > 7){ cout<<"dbg "<<stream<<" @"<<__FILE__<<":"<<__LINE__<<endl;}}
#define SD_DBG2(stream) {if(VERBOSITY > 6){ cout<<"dbg "<<stream<<" @"<<__FILE__<<":"<<__LINE__<<endl;}}
#define SD_DBG1(stream) {if(VERBOSITY > 5){ cout<<"dbg "<<stream<<" @"<<__FILE__<<":"<<__LINE__<<endl;}}
#define SD_DBG(stream)  {if(VERBOSITY > 4){ cout<<"dbg "<<stream<<" @"<<__FILE__<<":"<<__LINE__<<endl;}}
#define SD_INF(stream)  {if(VERBOSITY > 3){ cout<<"inf "<<stream<<endl;}}
#define SD_ERR(stream)  {if(VERBOSITY > 2){ cout<<"err! "<<stream<<endl;}}


// use to get the num elements of a C array: T a[4]; CARRAYLEN(a)==4;
#define CARRAYLEN(a) (sizeof(a)/sizeof(*(a)))

// loop over NULL terminated C array
#define FORCstrArr(i,a) for(i=0; (a)[i] != NULL; ++i)
// loop over MT::Array<char *> lists
#define FORStringList(i,l) for(i=0; i < l.N; ++i)

#define SD_MAX(a,b) ((a)<(b)?(b):(a))
#define SD_MIN(a,b) ((a)<(b)?(a):(b))


/* least greater than A power of 2 */
unsigned long lgp2(unsigned long a);
/* least greater than A or equal power of 2 */
unsigned long lgep2(unsigned long a);

#ifdef MT_IMPLEMENTATION
#include "utils.cpp"
#endif


#endif// header ifdef
