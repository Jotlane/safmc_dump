#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
using std::placeholders::_1;
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include "aruco_samples_utility.hpp"


#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"

using namespace std;
using namespace cv;
class ImageConverter : public rclcpp::Node
{
  public:
    ImageConverter()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/cam0/image_raw", 10, bind(&ImageConverter::topic_callback, this, _1));
    }

  private:
    void topic_callback(const sensor_msgs::msg::Image & msg) const
    {
        try
            {
                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                // Now you have the image in cv::Mat format
                Mat image = cv_ptr->image;


                vector< int > ids;
                vector< vector< Point2f > > corners, rejected;

                // detect markers and estimate pose
                detector.detectMarkers(image, corners, ids, rejected);

                size_t  nMarkers = corners.size();
                vector<Vec3d> rvecs(nMarkers), tvecs(nMarkers);

                if(estimatePose && !ids.empty()) {
                    // Calculate pose for each marker
                    for (size_t  i = 0; i < nMarkers; i++) {
                        solvePnP(objPoints, corners.at(i), camMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
                        node->publishMarkersPose(rvecs, tvecs);
                    }
                }


                // draw results
                image.copyTo(imageCopy);
                if(!ids.empty()) {
                    aruco::drawDetectedMarkers(imageCopy, corners, ids);

                    if(estimatePose) {
                        for(unsigned int i = 0; i < ids.size(); i++)
                        {
                            cv::drawFrameAxes(imageCopy, camMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 1.5f, 2);
                            cout << "R: " << rvecs[i] << "T " << tvecs[i] << endl;
                            }
                    }
                }

                if(showRejected && !rejected.empty())
                    aruco::drawDetectedMarkers(imageCopy, rejected, noArray(), Scalar(100, 0, 255));

                imshow("out", imageCopy);
                char key = (char)waitKey(waitTime);
                if(key == 27) break;



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
  rclcpp::spin(make_shared<ImageConverter>());
  rclcpp::shutdown();
  return 0;
}
