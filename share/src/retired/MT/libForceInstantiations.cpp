/*  ---------------------------------------------------------------------
    Copyright 2012 Marc Toussaint
    email: mtoussai@cs.tu-berlin.de
    
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

//#define MLR_IMPLEMENTATION
#ifndef MLR_IMPLEMENT_TEMPLATES
#  define MLR_IMPLEMENT_TEMPLATES
#endif

//the above two flags cause the following headers to
//include their implementations...
#include <Core/array.h>
#include <Core/util.h>

//explicit template instantiations
//-- from util.h
template void mlr::getParameter(double&, const char*);
template float mlr::getParameter<float>(const char*);
template void mlr::getParameter(uint&, const char*);
template int mlr::getParameter<int>(const char*);
template void mlr::getParameter(int&, const char*);
template void mlr::getParameter(int&, const char*, const int&);
template void mlr::getParameter(bool&, const char*, const bool&);
template int  mlr::getParameter(const char*, const int&);
template uint mlr::getParameter(const char*, const uint&);
template bool mlr::getParameter(const char*, const bool&);
template double mlr::getParameter(const char*, const double&);
template long mlr::getParameter(const char*);
template mlr::String mlr::getParameter(const char*);
template mlr::String mlr::getParameter(const char*, const mlr::String&);
template bool mlr::checkParameter<uint>(const char*);

template void mlr::Parameter<mlr::String>::initialize();
template void mlr::Parameter<bool>::initialize();
template void mlr::Parameter<double>::initialize();
template void mlr::Parameter<int>::initialize();

template std::map<std::string,int> mlr::ParameterMap<int>::m;
template std::map<std::string,double> mlr::ParameterMap<double>::m;
template std::map<std::string,unsigned int> mlr::ParameterMap<unsigned int>::m;
template std::map<std::string,float> mlr::ParameterMap<float>::m;
template std::map<std::string,bool> mlr::ParameterMap<bool>::m;
template std::map<std::string,long> mlr::ParameterMap<long>::m;
template std::map<std::string,mlr::String> mlr::ParameterMap<mlr::String>::m;
template std::map<std::string,std::string> mlr::ParameterMap<std::string>::m;

//-- from array.h
//full classes
//template class mlr::Array<uint>;
//template class mlr::Array<int>;
//template class mlr::Array<mlr::Array<uint> >;
//full classes & numerical routines
#define T double
#  include <Core/array_instantiate.cxx>
#undef T
#define NOFLOAT
#define T float
#  include <Core/array_instantiate.cxx>
#undef T
#define T uint
#  include <Core/array_instantiate.cxx>
#undef T
#define T uint16_t
#  include <Core/array_instantiate.cxx>
#undef T
#define T int
#  include <Core/array_instantiate.cxx>
#undef T
#define T long
#  include <Core/array_instantiate.cxx>
#undef T
#define T byte
#  include <Core/array_instantiate.cxx>
#undef T
#undef NOFLOAT

//-- from ors.h
//template mlr::Array<mlr::Transformation*>::Array();
//template mlr::Array<mlr::Transformation*>::~Array();

template mlr::Array<mlr::String>::Array();
template mlr::Array<mlr::String>::~Array();

template mlr::Array<arr*>::Array();
template mlr::Array<arr*>::Array(uint);
template mlr::Array<arr*>::~Array();

template mlr::Array<mlr::Array<uint> >::~Array();

template mlr::Array<char const*>::Array();
template mlr::Array<char const*>::Array(uint);
template mlr::Array<char const*>::~Array();

//template mlr::Array<glUI::Button>::Array();
//template mlr::Array<glUI::Button>::~Array();
