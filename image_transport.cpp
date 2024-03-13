#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
using std::placeholders::_1;

class ImageConverter : public rclcpp::Node
{
  public:
    ImageConverter()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/cam0/image_raw", 10, std::bind(&ImageConverter::topic_callback, this, _1));
    }

  private:
    void topic_callback(const sensor_msgs::msg::Image & msg) const
    {
      std::cout << "EEEEEEE" << std::endl;
    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageConverter>());
  rclcpp::shutdown();
  return 0;
}