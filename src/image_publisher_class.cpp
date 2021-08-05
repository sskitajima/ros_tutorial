#include <iostream>
#include <string>

#include <ros/ros.h>
#include <image_transport.h>
#include <cv_bridge.h>

#include <opencv2/highgui.hpp>

// クラスLaserSampleNodeの定義
class ImagePublisher
{
private:
  ros::NodeHandle nh_;

  // http://wiki.ros.org/image_transport
  image_transport::ImageTransport it_;
  image_transport::Publisher image_publisher_;
  image_transport::Publisher depth_image_publisher_;

  const int loop_rate_ = 1;
  const int buf_size_ = 10;

  const char *topic_name_ = "/ros_lecture/iamge";
  const char *depth_topic_name_ = "/ros_lecture/depth_image";

  const char *img_path_ = "/path_to_image";
  const char *depth_img_path_ = "/home/sskitajima/catkin_ws/src/ros_tutorial/image/1622618594.152518.png";

public:
  ImagePublisher()
      : it_(nh_)
  {
    image_publisher_ = it_.advertise(topic_name_, buf_size_);
    depth_image_publisher_ = it_.advertise(depth_topic_name_, buf_size_);
  }

  void mainloop()
  {
    ros::Rate r(loop_rate_);
    int cnt = 0;

    std_msgs::Header header;
    header.stamp = ros::Time::now();

    // http://wiki.ros.org/image_transport/Tutorials/PublishingImages
    cv::Mat image = cv::imread(img_path_, cv::IMREAD_UNCHANGED);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();

    cv::Mat depth_image = cv::imread(depth_img_path_, cv::IMREAD_UNCHANGED);
    sensor_msgs::ImagePtr depth_msg = cv_bridge::CvImage(header, "mono16", depth_image).toImageMsg();

    while (ros::ok())
    {
      //publish
      image_publisher_.publish(msg);
      depth_image_publisher_.publish(depth_msg);

      ROS_INFO("publish cnt: %d", cnt);

      // 指定したループ周期になるように調整
      r.sleep();
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_tutorial_image_publisher_class");
  ROS_INFO("ros_tutorial_image_publisher_class node start");

  // クラスLaserSampleNodeの実体となる変数nodeの定義
  ImagePublisher node = ImagePublisher();

  // メインループを回す
  node.mainloop();

  return 0;
}