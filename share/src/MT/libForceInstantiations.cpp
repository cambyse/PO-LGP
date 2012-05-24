/*  Copyright 2009 Marc Toussaint
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
    along with this program. If not, see <http://www.gnu.org/licenses/> */

//#define MT_IMPLEMENTATION
#ifndef MT_IMPLEMENT_TEMPLATES
#  define MT_IMPLEMENT_TEMPLATES
#endif

//the above two flags cause the following headers to
//include their implementations...
#include "util.h"
//#include <MT/opengl.h>
#include "array.h"
//#include "ors.h"
//#include <MT/algos.h>
//explicit template instantiations

//-- from util.h
template void MT::getParameter(double&, const char*);
template float MT::getParameter<float>(const char*);
template void MT::getParameter(uint&, const char*);
template int MT::getParameter<int>(const char*);
template void MT::getParameter(int&, const char*);
template void MT::getParameter(int&, const char*, const int&);
template void MT::getParameter(bool&, const char*, const bool&);
template int  MT::getParameter(const char*, const int&);
template uint MT::getParameter(const char*, const uint&);
template bool MT::getParameter(const char*, const bool&);
template double MT::getParameter(const char*, const double&);
template long MT::getParameter(const char*);
template MT::String MT::getParameter(const char*);
template MT::String MT::getParameter(const char*, const MT::String&);
template void MT::save<uintA>(const uintA&, const char*);
template bool MT::checkParameter<uint>(const char*);

template void MT::Parameter<MT::String>::initialize();
template void MT::Parameter<bool>::initialize();
template void MT::Parameter<double>::initialize();
template void MT::Parameter<int>::initialize();
template void MT::load<arr>(arr&,const char*, bool);

template std::map<std::string,int> MT::ParameterMap<int>::m;
template std::map<std::string,double> MT::ParameterMap<double>::m;
template std::map<std::string,unsigned int> MT::ParameterMap<unsigned int>::m;
template std::map<std::string,float> MT::ParameterMap<float>::m;
template std::map<std::string,bool> MT::ParameterMap<bool>::m;
template std::map<std::string,long> MT::ParameterMap<long>::m;
template std::map<std::string,MT::String> MT::ParameterMap<MT::String>::m;
template std::map<std::string,std::string> MT::ParameterMap<std::string>::m;

//-- from array.h
//full classes
//template class MT::Array<uint>;
//template class MT::Array<int>;
//template class MT::Array<MT::Array<uint> >;
//full classes & numerical routines
#define T double
#  include "array_instantiate.cxx"
#undef T
#define NOFLOAT
#define T float
#  include "array_instantiate.cxx"
#undef T
#define T uint
#  include "array_instantiate.cxx"
#undef T
#define T uint16
#  include "array_instantiate.cxx"
#undef T
#define T int
#  include "array_instantiate.cxx"
#undef T
#define T byte
#  include "array_instantiate.cxx"
#undef T
#undef NOFLOAT

//-- from ors.h
//template MT::Array<ors::Transformation*>::Array();
//template MT::Array<ors::Transformation*>::~Array();

template MT::Array<MT::String>::Array();

template MT::Array<arr*>::Array();
template MT::Array<arr*>::Array(uint);
template MT::Array<arr*>::~Array();

template MT::Array<MT::Array<uint> >::~Array();

template MT::Array<Any*>::Array();
template MT::Array<Any*>::~Array();

template MT::Array<char const*>::Array();
template MT::Array<char const*>::Array(uint);
template MT::Array<char const*>::~Array();

//template MT::Array<glUI::Button>::Array();
//template MT::Array<glUI::Button>::~Array();
