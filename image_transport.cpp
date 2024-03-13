#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"

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
        try
            {
                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                // Now you have the image in cv::Mat format
                cv::Mat image = cv_ptr->image;

                // Use the image as needed
                cv::imshow("Received Image", image);
                cv::waitKey(1);
            }
        catch (cv_bridge::Exception& e)
            {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }
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
