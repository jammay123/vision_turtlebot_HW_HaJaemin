#ifndef TURTLEBOT3_SIMULATION_INTERN_HPP_
#define TURTLEBOT3_SIMULATION_INTERN_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <cmath>

class TurtlebotAutoRace : public rclcpp::Node
{
public:
  TurtlebotAutoRace();

private:
  void imgCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void getBirdView(cv::Point2f* src_vertices, cv::Point2f* dst_vertices, const cv::Mat& src, cv::Mat& dst);
  void directCallback(const std_msgs::msg::String::SharedPtr msg);

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;     //퍼블리셔 선언
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;   //이미지 수신 선언
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr direct_sub_; //교차로 수신

  double linear_velocity_;
  double angular_velocity_;

  int mode = 1;
  int spd_mode = 0;
  int delay_time = 0;

  cv::Mat src_img;
  cv::Mat roi_img;
  cv::Mat hsv_img;

  cv::Mat yellow_mask;
  cv::Mat white_mask;
  cv::Mat green_mask;
  cv::Mat blue_mask;
  cv::Mat red_mask;

  cv::Mat blur_img;
  cv::Mat blur_yellow;
  cv::Mat blur_white;
  cv::Mat blur_green;
  cv::Mat blur_blue;
  cv::Mat blur_red;
  cv::Mat edges_yellow;
  cv::Mat edges_white;
};

#endif  // TURTLEBOT3_SIMULATION_INTERN_HPP_
