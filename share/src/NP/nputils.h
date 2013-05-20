/*  Copyright 2009 Nils Plath
    email: nilsp@cs.tu-berlin.de

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

/** \file nputils.h
    \brief Set of small commonly used routines */

#ifndef _NPUTILS_H
#define _NPUTILS_H

#include <iostream>
#include <exception>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <cstdlib>

#include <MT/array.h>

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define HERE __FILE__ ":" TOSTRING(__LINE__)

typedef MT::Array<std::string> stringA;

const double cpi = 3.1415926535897932384626433832795;

namespace np {
inline void msg_missing_implementation(const char* pos, const char* msg = NULL)
{
  std::cout << "Missing implementation: " << pos << ", "
            << (msg != NULL ? msg : "") << std::endl;
};

inline void msg_warning(const char* pos, const char* msg = NULL)
{
  std::cerr << "WARNING: " << pos << ", "
            << (msg != NULL ? msg : "") << std::endl;
};

inline void msg_error(const char* pos, const char* msg = NULL, bool throw_exception = true)
{
  std::cerr << "ERROR: " << pos << ", "
            << (msg != NULL ? msg : "") << std::endl;
  if (throw_exception)
    throw msg;
};

template <class T, class S>
void array2array(T& output, const S& input)
{
  if (input.nd > 3)
    msg_error(HERE, "input.nd > 3");

   int height = 1;
   int width  = 1;
   int depth  = 1;

   switch (input.nd)
   {
      case 0:
         msg_error(HERE, "input.nd == 0");
         break;
      case 1:
         height = input.d0;
         output.resize(height);
         break;
      case 2:
         height = input.d0;
         width = input.d1;
         output.resize(height, width);
         break;
      case 3:
         height = input.d0;
         width = input.d1;
         depth = input.d2;
         output.resize(height, width, depth);
         break;
      default:
         msg_error(HERE, "input.nd > 3");
         break;
   }

   int size = height * width * depth;
   for (int i = 0; i < size; i++)
      output.p[i] = input.p[i];
};

template <class T>
void load_array(MT::Array<T>& array, const char* filename, const char* tag)
{
  std::ifstream is;
  is.open(filename);
  array.readTagged(is, tag);
  is.close();
};

/** \brief Break a string or char array into tokens
 *
 *  @param tokens     string array with tokens
 *  @param str        the input string
 *  @param delimiter  denote the break points of the string
 *
 *  @return cf. parameter tokens
 */
void tokenize(stringA& tokens, const std::string& str, char delimiter);

/** \brief Break a global path into 3 tokens: path, file name, and file extension
 *
 *  @param tokens         three strings: path, file name, and file extension
 *  @param path2filename  global path and filename, e.g. "/home/foo/bar.c"
 *
 *  @return cf. parameter tokens
 */
void tokenize_filename(stringA& tokens, const std::string& path2filename);

charA datetime(bool separate = false);
void datetime
(
 int *year = NULL,
 int *month = NULL,
 int *day = NULL,
 int *hour = NULL,
 int *min = NULL,
 int *sec = NULL
);

double atod(const char* c);

template<class T>
void permute_rows(MT::Array<T>& matrix, uintA& permutation)
{
  MT::Array<T> matrix_temp = matrix;
  if (permutation.N == 0)
  {
    permutation.resize(matrix.d0);
    for (uint i = 0; i < matrix.d0; i++)
      permutation(i) = i;
    permutation.permuteRandomly();
  }

  if (matrix.nd == 2)
    for (uint i = 0; i < matrix.d0; i++)
      matrix[i] = matrix_temp[permutation(i)];
  else
    for (uint i = 0; i < matrix.d0; i++)
      matrix(i) = matrix_temp(permutation(i));
}

template <class S, class T>
class SimpleBinning
{
public:
  SimpleBinning(S low, S high, uint num_bins) : low_(low), high_(high), num_bins_(num_bins)
  {
    S step = (high-low)/ (S) (num_bins_-1);
    for (uint i=0; i<num_bins_; i++)
      markers_.append(low+i*step);
    markers_.p[markers_.N-1]=high;
  };
  void bin(MT::Array<T>& bins, const MT::Array<S>& values) const
  {
    uint bin_idx=0;
    bins.resize(num_bins_);
    bins = (T) 0;
    for (uint i=0; i<values.N; i++)
    {
//       std::cout << "values.p["<<i<<"] = " << values.p[i] << std::endl;
      bin_idx=0;
      if (values.p[i] <= markers_.p[0])
        bin_idx=0;
      else if(values.p[i] >= markers_.p[markers_.N-1])
        bin_idx=markers_.N-1;
      else
        for (uint j=1; j<markers_.N; j++)
        {
          if (values.p[i] <= markers_.p[j])
          {
            bin_idx=j-1;
            break;
          }
        }
//       std::cout << "values.p["<<i<<"] = " << values.p[i] << ", markers_.p["<<bin_idx<<"] = " << markers_.p[bin_idx] << std::endl;
      bins.p[bin_idx] += (T) 1;
//       std::cout << "bins = " << bins << std::endl;
    }
  };
  uint bin(S value) const
  {
    uint bin_idx=0;
    if (value <= markers_.p[0])
      bin_idx=0;
    else if(value >= markers_.p[markers_.N-1])
      bin_idx=markers_.N-1;
    else
      for (uint j=1; j<markers_.N; j++)
      {
        if (value <= markers_.p[j])
        {
          bin_idx=j-1;
          break;
        }
      }
    return bin_idx;
  };
protected:
  S low_, high_;
  uint num_bins_;
  MT::Array<S> markers_;
};

} // namespace np
#endif
