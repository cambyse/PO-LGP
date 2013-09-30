/*  Copyright 2010 Nils Plath
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

#include "cvwindow.h"
#include "cvutils.h"
#include "nputils.h"
#include "opencv_helper.h"

np::CvWindow::CvWindow(const char* name, uint w, uint h, uint posx, uint posy)
  : name_(name)
{
  cvNamedWindow(name, CV_WINDOW_AUTOSIZE);
  cvMoveWindow(name, posx, posy);
};

np::CvWindow::~CvWindow()
{cvDestroyWindow(name_.c_str());};

void np::CvWindow::draw_image(const byteA& img)
{
  if (img.d0==0 || img.d1==0)
    return;

  if(img.nd!=3)
  {
    if (img.d0*img.d1*3)
      buffer_.resize(img.d0,img.d1,3);
    cvCvtColor(buffer_, img, CV_GRAY2RGB);
  }
  else
    array2array(buffer_,img);
};

void np::CvWindow::draw_caption(const std::string& caption)
{
  if (buffer_.d0==0 || buffer_.d1==0)
    return;

  int width=(int)buffer_.d1;//, height=(int)buffer_.d0;
  CvPoint pts1; pts1.x=10; pts1.y=10;
  CvPoint pts2; pts2.x=width-10; pts2.y=30;
  cvRectangle(buffer_, pts1, pts2, cvScalar(0,0,255), 1, 8, 0);

  CvPoint pts3; pts3.x=10+5; pts3.y=10+15;
  CvFont font;
  cvInitFont(&font,CV_FONT_HERSHEY_PLAIN, 1.0,1.0,0.,1,CV_AA);
  cvPutText(buffer_, caption.c_str(), pts3, &font, cvScalar(0,0,255));
};

void np::CvWindow::update(double scale)
{
  if (buffer_.d0==0 || buffer_.d1==0)
    return;

  if (scale==1.0)
    cvShowImage(name_.c_str(), buffer_);
  else
  {
    byteA buffer_rescaled(buffer_.d0*scale,buffer_.d1*scale,3);
    cvResize(buffer_rescaled, buffer_);
    cvShowImage(name_.c_str(), buffer_rescaled);
  };
};

