#define MT_IMPLEMENTATION
#include <MT/ors.h>
#include <MT/ors_opengl.h>
#include <MT/ors_swift.h>
#include <MT/opengl.h>
#include <MT/soc.h>

MT::Array<ors::Proxy*> d1;
template double MT::getParameter<double>(const char*);
template void MT::Array<double>::writeTagged(std::ofstream& os,const char* tag,bool binary) const;

