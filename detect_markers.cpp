#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
using std::placeholders::_1;
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include "aruco_samples_utility.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"

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
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/coordinates", 10);
    }

  private:
    void topic_callback(const sensor_msgs::msg::Image & msg) const
    {
        try
            {
                bool estimatePose = true;
                bool showRejected = false;
                float markerLength = 0.07;
                aruco::DetectorParameters detectorParams;
                aruco::Dictionary dictionary = aruco::getPredefinedDictionary(0);
                int dictionaryId = 16;
                dictionary = aruco::getPredefinedDictionary(aruco::PredefinedDictionaryType(dictionaryId));
                Mat camMatrix, distCoeffs;
                bool readOk = readCameraParameters("caliboutput.yaml", camMatrix, distCoeffs);
                aruco::ArucoDetector detector(dictionary, detectorParams);
                cv::Mat objPoints(4, 1, CV_32FC3);
                objPoints.ptr<Vec3f>(0)[0] = Vec3f(-markerLength/2.f, markerLength/2.f, 0);
                objPoints.ptr<Vec3f>(0)[1] = Vec3f(markerLength/2.f, markerLength/2.f, 0);
                objPoints.ptr<Vec3f>(0)[2] = Vec3f(markerLength/2.f, -markerLength/2.f, 0);
                objPoints.ptr<Vec3f>(0)[3] = Vec3f(-markerLength/2.f, -markerLength/2.f, 0);
                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                // Now you have the image in cv::Mat format
                Mat image = cv_ptr->image;
                Mat imageCopy;

                vector< int > ids;
                vector< vector< Point2f > > corners, rejected;

                // detect markers and estimate pose
                detector.detectMarkers(image, corners, ids, rejected);

                size_t  nMarkers = corners.size();
                vector<Vec3d> rvecs(nMarkers), tvecs(nMarkers);

                if(estimatePose && !ids.empty()) {
                    // Calculate pose for each marker
                    for (size_t  i = 0; i < nMarkers; i++) {
                        if (ids[i] == 1)
                        {
                            solvePnP(objPoints, corners.at(i), camMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
                        }
                    }
                }


                // draw results
                image.copyTo(imageCopy);
                if(!ids.empty()) {
                    aruco::drawDetectedMarkers(imageCopy, corners, ids);

                    if(estimatePose) {
                        for(unsigned int i = 0; i < ids.size(); i++)
                        {
                            if (ids[i] == 1)
                            {
                                cv::drawFrameAxes(imageCopy, camMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 1.5f, 2);
                                cout << ids[i] << endl;
                                cout << "R: " << rvecs[i] << endl;
                                cout << "T " << tvecs[i] << endl;
                                geometry_msgs::msg::Twist twist_msg;

                                // Set the linear velocity (assuming tvecs contains translation vectors)
                                twist_msg.linear.x = tvecs[1][0]; // Adjust as necessary
                                twist_msg.linear.y = tvecs[1][1]; // Adjust as necessary
                                twist_msg.linear.z = tvecs[1][2]; // Adjust as necessary

                                // Set the angular velocity (assuming rvecs contains rotation vectors)
                                twist_msg.angular.x = rvecs[1][0]; // Adjust as necessary
                                twist_msg.angular.y = rvecs[1][1]; // Adjust as necessary
                                twist_msg.angular.z = rvecs[1][2]; // Adjust as necessary

                                // Publish the twist message
                                publisher_->publish(twist_msg);
                            }

                        }
                    }
                }

                if(showRejected && !rejected.empty())
                    aruco::drawDetectedMarkers(imageCopy, rejected, noArray(), Scalar(100, 0, 255));

                imshow("out", imageCopy);
                waitKey(1);



            }
        catch (cv_bridge::Exception& e)
            {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }
    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageConverter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
