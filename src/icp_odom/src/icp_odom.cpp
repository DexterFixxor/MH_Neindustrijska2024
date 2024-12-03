#include <cstdio>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>

class ICPOdom : public rclcpp::Node
{
  public:
    ICPOdom()
    : Node("icp_odom")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, std::bind(&ICPOdom::topic_callback, this, _1));
    }

  private:

    void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
      P.clear();
        float theta = 0;
        for (auto range : scan->ranges)
        {
          float x = cos(theta) * range;
          float y = sin(theta) * range;

          theta += scan->angle_increment;
          P.push_back(std::tuple(x, y));
      }
      if (prev_scan) {
        // centroid
        //https://eigen.tuxfamily.org/dox/group__TutorialMatrixArithmetic.html
        // TODO: covariance
        // TODO: SVD
        // TODO: find Rot and Trans
        
        //publish TF on topic /tf
      }
      
      prev_scan = scan;
      Q = P;
    }


    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;

    std::shared_ptr<sensor_msgs::msg::LaserScan> prev_scan = NULL;

    std::vector<std::tuple<float, float>> Q;
    std::vector<std::tuple<float, float>> P;

};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::shared_ptr<ICPOdom>();
  rclcpp::spin(node);
  rclcpp::shutdown();
}
