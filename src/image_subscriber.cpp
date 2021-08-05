#include <iostream>
#include <string>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui.hpp>

const char* topoic_name = "/ros_lecture/image";
const char* path_to_save_image = "/home/sskitajima/catkin_ws/src/ros_tutorial/image/save_image.png";

void ImageCallback(const sensor_msgs::ImageConstPtr &img_msg)
{
    // subした画像の処理
    cv::Mat raw_img;

    const char* encoding = img_msg->encoding.c_str();

    ROS_INFO("topic %s, encoding %s", topoic_name, encoding);

    // 2つの文字列が等しいときに 0 を返す関数 strcmp
    if(!strcmp(encoding, "bgr8"))
    {
        raw_img = cv_bridge::toCvShare(img_msg, "8UC3")->image;
        cv::imwrite(path_to_save_image, raw_img);
        std::cout << "save image " << std::endl;
    }
}


int main(int argc, char **argv)
{
    // if(argc != 2)
    // {
    //     std::cout << "usage: rosrun sandbox check_depth_type_node [topic_name]" << std::endl;
    //     return -1;
    // }
    // topoic_name = argv[1];

    if(argc == 2)
    {
        std::cout << "argument: " << argv[1] << std::endl; 
    }
    else
    {
        std::cout << "num arg " << argc <<  std::endl;
    }

    ros::init(argc, argv, "ros_tutorial_image_subscriber");
    ROS_INFO("ros_tutorial_image_subscriber node start");

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(topoic_name, 1, ImageCallback);

    ros::spin();


    return 0;
}