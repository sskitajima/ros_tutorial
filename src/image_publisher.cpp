#include <iostream>
#include <string>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_tutorial_image_publisher");
    ROS_INFO("ros_tutorial_image_publisher node start");

    const int loop_rate = 1;
    const char *topic_name = "/ros_lecture/image";
    const char *depth_topic_name = "/ros_lecture/depth_image";
    const char *img_path = "/home/sskitajima/catkin_ws/src/ros_tutorial/image/1622618594.196132.png";
    const char *depth_img_path = "/home/sskitajima/catkin_ws/src/ros_tutorial/image/1622618594.152518.png";
    const int buf_size = 10;
    
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher image_publisher = it.advertise(topic_name, buf_size);
    image_transport::Publisher depth_image_publisher = it.advertise(depth_topic_name, buf_size);

    ros::Rate r(loop_rate);
    int cnt = 0;

    std_msgs::Header header;
    header.stamp = ros::Time::now();

    // http://wiki.ros.org/image_transport/Tutorials/PublishingImages
    cv::Mat image = cv::imread(img_path, cv::IMREAD_UNCHANGED);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();

    cv::Mat depth_image = cv::imread(depth_img_path, cv::IMREAD_UNCHANGED);
    sensor_msgs::ImagePtr depth_msg = cv_bridge::CvImage(header, "mono16", depth_image).toImageMsg();

    while (ros::ok())
    {
        //publish
        image_publisher.publish(msg);
        depth_image_publisher.publish(depth_msg);

        ROS_INFO("publish cnt: %d", ++cnt);

        // 指定したループ周期になるように調整
        r.sleep();
    }

    return 0;
}