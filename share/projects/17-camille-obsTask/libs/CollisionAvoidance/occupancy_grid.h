/*  ------------------------------------------------------------------
    Copyright 2016 Camille Phiquepal
    email: camille.phiquepal@gmail.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */

#pragma once

#include <algorithm>
#include <math_utility.h>

#include <Kin/taskMap.h>
#include <Kin/taskMaps.h>


#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


struct CircularCage:TaskMap{

  CircularCage( const std::string & object, const arr & center, double radius )
    : object_( object )
    , center_( center )
    , max_radius_( radius )
    , safety_distance_(0.1)
  {

  }

  virtual mlr::String shortTag(const mlr::KinematicWorld& G)
  {
    return mlr::String("CircularCage");
  }

  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=-1) override
  {
    mlr::Frame *object = G.getFrameByName(object_.c_str());

    arr pos, Jpos;
    G.kinematicsPos(pos, Jpos, object);

    y.resize(1);//zeros(dim_phi(Gs, t));

    //y(0) = pos(0) - 1;
    const auto radius = sqrt(pow(pos(0)-center_(0), 2.0) + pow(pos(1)-center_(1), 2.0));
    y(0) = safety_distance_ + radius - max_radius_;

    //std::cout << "y(0)" << y(0) << std::endl;

    if(&J)
    {
      J = zeros(dim_, Jpos.d1);
      auto theta = std::atan2(pos(1)-center_(1), pos(0)-center_(0));

      J(0, 0) = cos(theta) * Jpos(0, 0);
      J(0, 1) = sin(theta) * Jpos(1, 1);
    }
  }

  virtual uint dim_phi(const mlr::KinematicWorld& K) override
  {
    return dim_;
  }

private:
  static const uint dim_ = 1;
  std::string object_;
  double max_radius_;
  arr center_;
  double safety_distance_;
};

void show_img(const std::string & window_name, const cv::Mat & mat)
{
  cv::Mat normalized;
  cv::normalize(mat, normalized, 0, 1.0, cv::NORM_MINMAX);
  cv::flip(normalized, normalized, 0);
  imshow(window_name, normalized);
}

struct OccupancyGrid:TaskMap{

  OccupancyGrid( const std::string & object )
    : object_( object )
    , cell_size_(0.01)
    , safety_distance_(0.1)
  {

  }

  double cell_size() const { return cell_size_; }

  void setDataFromFile(const std::string & filename)
  {
    const int delta = 0;
    const int ddepth = -1;

    cv::Mat sensor_map = cv::imread(filename.c_str());
    cv::flip(sensor_map, sensor_map, 0);
    cv::cvtColor(sensor_map, sensor_map_bw_, cv::COLOR_BGR2GRAY);

    cv::threshold(sensor_map_bw_, sensor_map_bw_,     127, 255, cv::THRESH_BINARY);
    cv::threshold(sensor_map_bw_, sensor_map_bw_inv_, 127, 255, cv::THRESH_BINARY_INV);

    cv::distanceTransform(sensor_map_bw_, dist_, CV_DIST_L2, 3);//, 2, 3);
    cv::Sobel(dist_, grad_x_, ddepth, 1, 0, 3, 1.0 / 8, delta, cv::BORDER_DEFAULT);
    cv::Sobel(dist_, grad_y_, ddepth, 0, 1, 3, 1.0 / 8, delta, cv::BORDER_DEFAULT);

    cv::distanceTransform(sensor_map_bw_inv_, dist_inv_, CV_DIST_L2, 3);//, 2, 3);
    cv::Sobel(dist_inv_, grad_x_inv_, ddepth, 1, 0, 3, 1.0 / 8, delta, cv::BORDER_DEFAULT);
    cv::Sobel(dist_inv_, grad_y_inv_, ddepth, 0, 1, 3, 1.0 / 8, delta, cv::BORDER_DEFAULT);

    if(0)
    {
      std::cout << dist_.type() << " val:" << dist_inv_.at<float>(150,147) << std::endl;
      std::cout << dist_.type() << " val:" << dist_inv_.at<float>(149,147) << std::endl;
      auto dx = dist_inv_.at<float>(150,147) - dist_inv_.at<float>(149,147);
      std::cout << "dx:" << dx << std::endl;
      std::cout << grad_x_.type() << " grad_x:" << " " << grad_x_inv_.at<float>(150,147) << std::endl;
      std::cout << grad_y_.type() << " grad_y:" << " " << grad_y_inv_.at<float>(149,147) << std::endl;

//      show_img("Normal", sensor_map_bw_);
//      show_img("Inverted", sensor_map_bw_inv_);
//      show_img("Distance Transform Image", dist_);
//      show_img("Gradient x Image", grad_x_);
//      show_img("Gradient y Image", grad_y_);

//      show_img("Distance Transform Image Inv", dist_inv_);
//      show_img("Gradient x Image Inv", grad_x_inv_);
//      show_img("Gradient y Image Inv", grad_y_inv_);

      cv::waitKey();
    }
  }

  virtual mlr::String shortTag(const mlr::KinematicWorld& G)
  {
    return mlr::String("OccupancyGrid");
  }

  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=-1) override
  {
    mlr::Frame *object = G.getFrameByName(object_.c_str());

    arr pos, Jpos;
    G.kinematicsPos(pos, Jpos, object);

    y.resize(1);//zeros(dim_phi(Gs, t));

    auto dist_info = get_distance_info(pos(0), pos(1));

    y(0) = safety_distance_ - dist_info[0];

    if(&J)
    {
      J = zeros(dim_, Jpos.d1);

      J(0, 0) = -dist_info[1] * Jpos(0, 0);
      J(0, 1) = -dist_info[2] * Jpos(1, 1); //sin(theta) * Jpos(1, 1);
    }
  }

  virtual uint dim_phi(const mlr::KinematicWorld& K) override
  {
    return dim_;
  }

  std::vector<double> get_distance_info(double x, double y) const
  {
    uint i, j;
    convert(x, y, i, j);

    double dist = 0;
    double gx = 0;
    double gy = 0;

    if( on_map(i, j) )
    {
      auto d = dist_.at<float>(i, j);
      auto d_inv = dist_inv_.at<float>(i, j);

      auto grad_x = grad_x_.at<float>(i, j);
      auto grad_x_inv = grad_x_inv_.at<float>(i, j);

      auto grad_y = grad_y_.at<float>(i, j);
      auto grad_y_inv = grad_y_inv_.at<float>(i, j);

      dist = cell_size_ * (d - d_inv);
      gx = grad_x - grad_x_inv;
      gy = grad_y - grad_y_inv;
    }
    else // not fully exact here, just tries to go back on closest point on map
    {
      double dx = 0;
      double dy = 0;
      if(i < 0)
      {
        dy = cell_size_ * i;
        //gy = 1;
      }
      else if( i > dist_.rows )
      {
        dy = -cell_size_ * (i - dist_.rows);
        gy = -1;
      }

      if(j < 0)
      {
        dx = cell_size_ * j;
        gx = 1;
      }
      else if( j > dist_.cols )
      {
        dx = -cell_size_ * (j - dist_.cols);
        gx = -1;
      }

      auto theta = std::atan2(-dy, -dx);
      dist = -sqrt(dx*dx+dy*dy);
      gx *= cos(theta);
      gy *= sin(theta);
    }

    return {dist, gx, gy};
  }

private:

  bool on_map(int i, int j) const
  {
    return i >= 0 && i < dist_.rows && j >= 0 && j < dist_.cols;
  }

  void convert(double x, double y, uint & i, uint & j) const
  {
    i = 0.5 * dist_.rows + y / cell_size_;
    j = 0.5 * dist_.cols + x / cell_size_;
  }

private:
  static const uint dim_ = 1;
  const std::string object_;
  const double cell_size_;
  const double safety_distance_;

  cv::Mat sensor_map_bw_, sensor_map_bw_inv_, dist_, dist_inv_, grad_x_, grad_y_, grad_x_inv_, grad_y_inv_;
};

