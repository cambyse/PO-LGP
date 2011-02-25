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

#include <MT/array.h>
#include <MT/util.h>
#include <iostream>
#include <iomanip>
#include <cstring>
#include <cmath>
#include <vector>
#include <cstdio>
#include <ctime>
#include <string>
#include "nputils.h"

/*! \brief Break a string or char array into tokens
 *
 *  @param tokens     string array with tokens
 *  @param str        the input string
 *  @param delimiter  denote the break points of the string
 *
 *  @return cf. parameter tokens
 */
void np::tokenize(stringA& tokens, const std::string& str, char delimiter)
{
    // Skip delimiters at beginning.
   std::string::size_type lastPos = str.find_first_not_of(delimiter, 0);
    // Find first "non-delimiter".
   std::string::size_type pos     = str.find_first_of(delimiter, lastPos);

   while (std::string::npos != pos || std::string::npos != lastPos)
   {
     // Found a token, add it to the vector.
     tokens.append(str.substr(lastPos, pos - lastPos));
     // Skip delimiters.  Note the "not_of"
     lastPos = str.find_first_not_of(delimiter, pos);
     // Find next "non-delimiter"
     pos = str.find_first_of(delimiter, lastPos);
   }
}

/*! \brief Break a global path into path, file name, and file extension
 *
 *  @param tokens         three strings: path, file name, and file extension
 *  @param path2filename  global path and filename, e.g. "/home/foo/bar.c"
 *
 *  @return cf. parameter tokens
 */
void np::tokenize_filename(stringA& tokens, const std::string& path2filename)
{
  // make sure output has the right size
  tokens.resize(3);

  stringA tokens_temp;
  std::string token;
  std::ostringstream output_file;

  // 0. split string into "/path/to/filename" and "ext"
  tokenize(tokens_temp, path2filename, '.');

  // 1. determine file name and path by going backwards in token
  // "/path/to/filename" until first '/' is found, then substring it
  int pos = tokens_temp(0).find_last_of("/");
  tokens(0) = tokens_temp(0).substr(0,pos+1);
  tokens(1) = tokens_temp(0).substr(pos+1);

  // 2. determine file extension
  tokens(2) = tokens_temp(tokens_temp.N-1);
}

charA np::datetime(bool separate)
{
  charA dt;
  std::ostringstream oss;
  int year, month, day, hour, minute, second;
  dt.clear();

  datetime(&year, &month, &day, &hour, &minute, &second);
  oss.str("");
  if (separate)
    dt.resize(20);
  else
    dt.resize(15);

  oss << year << (separate ? "-" : "")\
      << std::setw(2) << std::setfill('0') << month << (separate ? "-" : "") \
      << std::setw(2) << std::setfill('0') << day << (separate ? "_" : "") \
      << std::setw(2) << std::setfill('0') << hour << (separate ? ":" : "") \
      << std::setw(2) << std::setfill('0') << minute << (separate ? ":" : "") \
      << std::setw(2) << std::setfill('0') << second;

  dt = '\0';
  memcpy(dt.p, oss.str().c_str(), dt.N);

  return dt;
}

void np::datetime
(
 int *year,
 int *month,
 int *day,
 int *hour,
 int *min,
 int *sec
)
{
  std::time_t seconds= std::time(0);
  struct tm *ptm= std::localtime(&seconds);
  if (year != NULL)  *year  = (int)ptm->tm_year+1900;
  if (month != NULL) *month = (int)ptm->tm_mon+1;
  if (day != NULL)   *day   = (int)ptm->tm_mday;
  if (hour != NULL)  *hour  = (int)ptm->tm_hour;
  if (min != NULL)   *min   = (int)ptm->tm_min;
  if (sec != NULL)   *sec   = (int)ptm->tm_sec;
}


double np::atod(const char* a)
{
  double d = 0, exp = 0;
  char c[2]; c[1] = '\0';
  int dec_point = 0;
  bool is_negative = (a[0] == '-');
  uint start = (is_negative ? 1 : 0);
  uint len_a = strlen(a);
  for (uint i = start; i < len_a; i++)
    if (a[i] == '.')
    {
      dec_point = i-1-start;
      break;
    }


  exp = pow(10, dec_point);
  for (uint i = start; i < len_a; i++)
    if (a[i] != '.')
    {
      c[0] = a[i];
      d += exp * (double) atoi(c);
      exp *= 0.1;
    }

  if (is_negative)
    d *= -1;

  return d;
}
