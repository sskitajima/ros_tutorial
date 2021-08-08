#include <iostream>
#include <string>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui.hpp>

// global変数として定義（大きなプログラムではあまり良い書き方ではない）
const char* topic_name = "/ros_lecture/image";
const char* path_to_save_image = "/home/sskitajima/catkin_ws/src/ros_tutorial/image/save_image.png";

// メッセージを受け取った（subscribe）したときに実行される関数
void ImageCallback(const sensor_msgs::ImageConstPtr &img_msg)
{
    // opencvで画像を示す型を用意
    cv::Mat raw_img;

    const char* encoding = img_msg->encoding.c_str();

    ROS_INFO("topic %s, encoding %s", topic_name, encoding);

    // 2つの文字列が等しいときに 0 を返す関数 strcmp
    // もし8but rgbの画像だったとき、処理を行う
    if(!strcmp(encoding, "bgr8"))
    {
        // 画像を取り出す
        raw_img = cv_bridge::toCvShare(img_msg, "8UC3")->image;

        // 画像を指定されたパスに保存
        cv::imwrite(path_to_save_image, raw_img);

        std::cout << "save image " << std::endl;
    }
}

// http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_tutorial_image_subscriber");
    ROS_INFO("ros_tutorial_image_subscriber node start");

    // ノードハンドラ
    ros::NodeHandle nh;

    // サブスクライバの定義
    // サブスクライブしたときに実行される、関数を登録しておく。
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(topic_name, 1, ImageCallback);

    // 処理をこの行でブロックし、サブスクライブされるのを待ち続ける
    ros::spin();


    return 0;
}