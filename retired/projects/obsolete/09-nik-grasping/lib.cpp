#define MLR_IMPLEMENTATION
#include <MT/kin.h>
#include <MT/kin_opengl.h>
#include <MT/kin_swift.h>
#include <MT/opengl.h>
#include <MT/soc.h>

mlr::Array<mlr::Proxy*> d1;
template double mlr::getParameter<double>(const char*);
template void mlr::Array<double>::writeTagged(std::ofstream& os,const char* tag,bool binary) const;

