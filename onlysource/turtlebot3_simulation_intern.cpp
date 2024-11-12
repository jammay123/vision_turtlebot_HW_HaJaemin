#include "turtlebot3_simulation_intern/turtlebot3_simulation_intern.hpp"

TurtlebotAutoRace::TurtlebotAutoRace() : Node("turtlebot3_simulation_intern_node"), angular_velocity_(0.0), linear_velocity_(0.07), spd_mode(0)
{
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  img_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/image_raw", 10, std::bind(&TurtlebotAutoRace::imgCallback, this, std::placeholders::_1));
  direct_sub_ = this->create_subscription<std_msgs::msg::String>("/turtlebot3_status_manager/direction", 10, std::bind(&TurtlebotAutoRace::directCallback, this, std::placeholders::_1));
}

// Bird's-eye view 변환 함수
void TurtlebotAutoRace::getBirdView(cv::Point2f* src_vertices, cv::Point2f* dst_vertices, const cv::Mat& src, cv::Mat& dst) {
  cv::Mat warpMatrix = cv::getPerspectiveTransform(src_vertices, dst_vertices);
  cv::warpPerspective(src, dst, warpMatrix, cv::Size(src.cols, src.rows), cv::INTER_LINEAR, cv::BORDER_CONSTANT);
}

void TurtlebotAutoRace::imgCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
  try {
    geometry_msgs::msg::Twist cmd_msg;
    // OpenCV 이미지로 변환
    src_img = cv_bridge::toCvCopy(msg, "bgr8")->image;

    int img_width = src_img.cols;
    int img_height = src_img.rows;

    cv::Point2f src_vertices[4] = {
      cv::Point2f(280, img_height),
      cv::Point2f(1680, img_height),
      cv::Point2f(1500, 800),
      cv::Point2f(450, 800)
    };

    cv::Point2f dst_vertices[4] = {
      cv::Point2f(0, img_height),
      cv::Point2f(img_width, img_height),
      cv::Point2f(img_width, 0),
      cv::Point2f(0, 0)
    };

    cv::Mat bird_eye_view;
    getBirdView(src_vertices, dst_vertices, src_img, bird_eye_view);

    cv::Rect roi(0, 0, bird_eye_view.cols, bird_eye_view.rows - 10);
    cv::Mat roi_bird_eye_view = bird_eye_view(roi);

    cv::Mat hsv_img, yellow_mask, white_mask;
    cv::cvtColor(roi_bird_eye_view, hsv_img, cv::COLOR_BGR2HSV);

    cv::Scalar lower_yellow(20, 100, 100);
    cv::Scalar upper_yellow(30, 255, 255);
    cv::inRange(hsv_img, lower_yellow, upper_yellow, yellow_mask);

    cv::Scalar lower_white(0, 0, 100);
    cv::Scalar upper_white(180, 25, 255);
    cv::inRange(hsv_img, lower_white, upper_white, white_mask);

    cv::Scalar lower_green = cv::Scalar(40,50,50);
    cv::Scalar upper_green = cv::Scalar(80,255,255);
    cv::inRange(hsv_img, lower_green, upper_green, green_mask);

    cv::Scalar lower_blue = cv::Scalar(100,50,50);
    cv::Scalar upper_blue = cv::Scalar(130,255,255);
    cv::inRange(hsv_img, lower_blue, upper_blue, blue_mask);

    cv::Scalar lower_red = cv::Scalar(0,50,50);
    cv::Scalar upper_red = cv::Scalar(20,255,255);
    cv::inRange(hsv_img, lower_red, upper_red, red_mask);

    cv::Mat blur_yellow, blur_white, blur_green, blur_blue;
    cv::GaussianBlur(yellow_mask, blur_yellow, cv::Size(5, 5), 0);
    cv::GaussianBlur(white_mask, blur_white, cv::Size(5, 5), 0);
    cv::GaussianBlur(green_mask, blur_green, cv::Size(5, 5), 0);
    cv::GaussianBlur(blue_mask, blur_blue, cv::Size(5, 5), 0);
    cv::GaussianBlur(red_mask, blur_red, cv::Size(5, 5), 0);

    cv::Mat edges_yellow, edges_white;
    cv::Canny(blur_yellow, edges_yellow, 50, 150);
    cv::Canny(blur_white, edges_white, 50, 150);

    std::vector<cv::Vec4i> lines_yellow, lines_white;
    cv::HoughLinesP (edges_yellow, lines_yellow, 1, CV_PI / 180, 50, 50, 10);
    cv::HoughLinesP (edges_white, lines_white, 1, CV_PI / 180, 50, 50, 10);

    bool yellow_detected = !lines_yellow.empty();
    bool white_detected = !lines_white.empty();
    bool green_detected = cv::countNonZero(green_mask) > 0;
    bool blue_detected = cv::countNonZero(blue_mask) > 0;
    bool red_detected = cv::countNonZero(red_mask) > 0;

    if(red_detected){
      angular_velocity_ = 0.0;
      rclcpp::sleep_for(std::chrono::milliseconds(2500));
      spd_mode = 3;
    }
    else if (!yellow_detected && !white_detected) {
      if (mode == 1) {
        angular_velocity_ = -0.6;
      }
      else if (mode == 2) {
        angular_velocity_ = +0.6;
        cmd_msg.linear.x = linear_velocity_;
        cmd_msg.angular.z = angular_velocity_;
        cmd_vel_pub_ -> publish(cmd_msg);
        rclcpp::sleep_for(std::chrono::milliseconds(1500));
      }
    }
    else if (yellow_detected && white_detected) {
      angular_velocity_ = 0.0;
    }
    else if (!white_detected && yellow_detected) {
      for (const auto& l : lines_yellow) {
        if (l[0] < img_width * 4 / 5 || l[2] < img_width * 4 / 5) {
            angular_velocity_ = +0.7;  // 더 강하게 회전
            break;
        }
        else if (l[0] < img_width * 3 / 5 || l[2] < img_width * 3 / 5) {
            angular_velocity_ = +0.5;  // 중간 강도의 회전
            break;
        }
        else if (l[0] < img_width * 2 / 5 || l[2] < img_width * 2 / 5) {
            angular_velocity_ = +0.3;  // 약간의 회전
            break;
        }
        else {
            angular_velocity_ = +0.05;  // 매우 약한 회전
            break;
        }
      }
    }
    else if (!yellow_detected && white_detected) {
      for (const auto& l : lines_white) {
        if (l[0] > img_width * 4 / 5 || l[2] > img_width * 4 / 5) {
            angular_velocity_ = -0.8;  // 더 강하게 회전
            break;
        }
        else if (l[0] > img_width * 3 / 5 || l[2] > img_width * 3 / 5) {
            angular_velocity_ = -0.6;  // 중간 강도의 회전
            break;
        }
        else if (l[0] > img_width * 2 / 5 || l[2] > img_width * 2 / 5) {
            angular_velocity_ = -0.35;  // 약간의 회전
            break;
        }
        else {
            angular_velocity_ = -0.05;  // 매우 약한 회전
            break;
        }
      }
    }

    if (green_detected) {
      spd_mode = 1;
    }

    if (blue_detected) {
      spd_mode = 2;
    }
    switch (spd_mode) {
      case 0:
        linear_velocity_ = 0.07;
        break;
      case 1:
        linear_velocity_ = 0.07 * 1.5;
        break;
      case 2:
        linear_velocity_ = 0.07;
        break;
      case 3:
        linear_velocity_ = 0.0;
        break;
    }

    cmd_msg.linear.x = linear_velocity_;
    cmd_msg.angular.z = angular_velocity_;
    cmd_vel_pub_ -> publish(cmd_msg);
    cv::imshow("Bird's Eye View with Lines", roi_bird_eye_view);
    //cv::imshow("Canny Edges - Yellow", edges_yellow);
    //cv::imshow("Canny Edges - White", edges_white);
    cv::waitKey(1);
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error converting image: %s", e.what());
  }
}

void TurtlebotAutoRace::directCallback(const std_msgs::msg::String::SharedPtr msg) {
  //RCLCPP_INFO(this->get_logger(), "Received direction: %s", msg->data.c_str());
  if(msg->data == "right")
  mode = 1;
  else if(msg->data == "left")
  mode = 2;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtlebotAutoRace>());
  rclcpp::shutdown();
  return 0;
}
