/**
 * @file footprint.cpp
 * @author Juan Sandino (juan.sandino@hdr.qut.edu.au)
 * @brief
 * @version 0.1.0
 * @date 2020-06-15
 *
 * @copyright Copyright (c) 2020 Juan Sandino, Data61 CSIRO
 *
 */
#include "camera/footprint.hpp"

Footprint::Footprint()
{
  footprint_img_ = cv::Mat();
}

Footprint::Footprint(const double &min_x, const double &max_x, const double &min_y, const double &max_y, const int &res)
  : min_x_(min_x), max_x_(max_x), min_y_(min_y), max_y_(max_y), res_(res)
{
  len_x_ = max_x_ - min_x_;
  len_y_ = max_y_ - min_y_;
  footprint_img_ = cv::Mat::zeros((int)(len_x_ * res_), (int)(len_y_ * res_), CV_8UC1);
}

Footprint::~Footprint() = default;

void Footprint::addFOVFootprint(const std::vector<geometry_msgs::Point> &points)
{
  cv::Point rec_vertices[4];
  for (size_t i = 0; i < 4; i++)
  {
    fromCartesianToImage(points[i], rec_vertices[i]);
  }
  cv::Scalar color = cv::Scalar(255.0);  // white
  cv::fillConvexPoly(footprint_img_, rec_vertices, 4, color);
#ifndef NDEBUG
  cv::imwrite(debug_path + "visionSensor_Footprint.jpg", footprint_img_);  // for debugging purposes.
#endif
}

bool Footprint::isFOVinFootprint(const std::vector<geometry_msgs::Point> &points, const unsigned short &min_points = 1)
{
  unsigned short counter = 0;
  for (auto point : points)
  {
    cv::Point conv_point;
    fromCartesianToImage(point, conv_point);

    if ((int)footprint_img_.at<unsigned char>(conv_point) > 0)
    {
      counter++;
    }
  }
  return (4 - counter < min_points);
}

double Footprint::fovOverlapInFootprint(const std::vector<geometry_msgs::Point> &points)
{
  std::vector<cv::Point> fov;
  for (auto point : points)
  {
    cv::Point conv_point;
    fromCartesianToImage(point, conv_point);
    fov.push_back(conv_point);
  }
  cv::Rect r = cv::boundingRect(fov);
  cv::Mat roi(footprint_img_, r);
  cv::Scalar intensity = cv::mean(roi);
  return (intensity[0] / 255);
}

cv::Mat Footprint::getFootprintImg() const
{
  return footprint_img_;
}

std::vector<uchar> Footprint::getFootprintVector()
{
  cv::Mat flat = footprint_img_.reshape(1, (int)footprint_img_.total() * footprint_img_.channels());
  return footprint_img_.isContinuous() ? flat : flat.clone();
}

void Footprint::fromCartesianToImage(const geometry_msgs::Point &fov_point, cv::Point &img_coord) const
{
  double rot_x = -fov_point.x;  // fov_point.x * cos(M_PI) - fov_point.y * sin(M_PI);
  double offset = min_x_ + (0.5 * len_x_);
  double tf_x = rot_x + (0.5 * len_x_) + offset;
  img_coord.y = (int)(tf_x * res_);
  if (img_coord.y >= footprint_img_.rows)
  {
    img_coord.y = (int)(footprint_img_.rows - 1);
  }
  if (img_coord.y < 0)
  {
    img_coord.y = 0;
  }

  double rot_y = -fov_point.y;  // fov_point.x * sin(M_PI) + fov_point.y * cos(M_PI);
  offset = min_y_ + (0.5 * len_y_);
  double tf_y = rot_y + (0.5 * len_y_) + offset;
  img_coord.x = (int)(tf_y * res_);
  if (img_coord.x >= footprint_img_.cols)
  {
    img_coord.x = (int)(footprint_img_.cols - 1);
  }
  if (img_coord.x < 0)
  {
    img_coord.x = 0;
  }
}