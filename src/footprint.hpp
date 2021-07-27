#ifndef _FOOTPRINT_HPP_
#define _FOOTPRINT_HPP_
/**
 * @file footprint.hpp
 * @author Juan Sandino (juan.sandino@hdr.qut.edu.au)
 * @brief
 * @version 0.1.0
 * @date 2020-10-18
 *
 * @copyright Copyright (c) 2020 Juan Sandino, Data61 CSIRO
 *
 */
#include <geometry_msgs/Point.h>
#include <ros/package.h>
#include <algorithm>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <vector>

class Footprint
{
public:
  /**
   * @brief Construct a new Footprint object
   *
   */
  Footprint();
  /**
   * @brief Construct a new Footprint object
   *
   * @param min_x Minimum value X axis in metres.
   * @param max_x Maximum value X axis in metres.
   * @param min_y Minimum value Y axis in metres.
   * @param max_y Maximum value Y axis in metres.
   * @param res Footprint resolution (pixels per metre).
   */
  Footprint(const double &min_x, const double &max_x, const double &min_y, const double &max_y, const int &res);
  /**
   * @brief Destroy the Footprint object
   *
   */
  ~Footprint();
  /**
   * @brief Add new points to the footprint contour. The method sorts new points automatically
   *
   * @param points The new FOV corner points to append
   */
  void addFOVFootprint(const std::vector<geometry_msgs::Point> &points);
  /**
   * @brief
   *
   * @param points
   * @return false If at least one point is not inside the footprint.
   * @return true Otherwise.
   */

  /**
   * @brief Check whether the FOV point corners are inside the path footprint.
   *
   * @param points FOV points.
   * @param min_points number of points to evaluate (threshold)
   * @return true If at least the min_points is/are not inside the footprint.
   * @return false Otherwise
   */
  bool isFOVinFootprint(const std::vector<geometry_msgs::Point> &points, const unsigned short &min_points);
  /**
   * @brief Calculates the overlapping of the FOV projection inside the recorded footprint.
   *
   * @param points FOV corner points.
   * @return double The relative overlap proportion [0 - 1].
   */
  double fovOverlapInFootprint(const std::vector<geometry_msgs::Point> &points);
  /**
   * @brief Get the Footprint Points object
   *
   * @return cv::Mat
   */
  cv::Mat getFootprintImg() const;
  /**
   * @brief Get the Footprint Vector object
   *
   * @return std::vector<uchar>
   */
  std::vector<uchar> getFootprintVector();

private:
  double min_x_ = 0.0;
  double max_x_ = 0.0;
  double min_y_ = 0.0;
  double max_y_ = 0.0;
  double len_x_ = 0.0;
  double len_y_ = 0.0;
  size_t res_ = 0;
  /* Footprint represented as a 2D array (Image). */
  cv::Mat footprint_img_;
#ifndef NDEBUG
  std::string debug_path = ros::package::getPath("scouter_ros") + "/debug/";
#endif

  void fromCartesianToImage(const geometry_msgs::Point &fov_point, cv::Point &img_coord) const;
};

#endif /* _FOOTPRINT_HPP_ */