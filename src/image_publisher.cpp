// C++標準ライブラリのヘッダ
#include <iostream>
#include <string>

// ROSのヘッダ
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// opencvのヘッダ
#include <opencv2/highgui.hpp>

// http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
int main(int argc, char **argv)
{
    // rosノードとして起動
    ros::init(argc, argv, "ros_tutorial_image_publisher");

    // ros時間とともに標準出力を行うマクロ
    ROS_INFO("ros_tutorial_image_publisher node start");

    // 定数の定義
    const int loop_rate = 1;
    const char *topic_name = "/ros_lecture/image";
    const char *depth_topic_name = "/ros_lecture/depth_image";
    const char *img_path = "/home/sskitajima/catkin_ws/src/ros_tutorial/image/1622618594.196132.png";
    const char *depth_img_path = "/home/sskitajima/catkin_ws/src/ros_tutorial/image/1622618594.152518.png";
    const int buf_size = 10;
    
    // ノードハンドラ
    // マスタからパラメータを取得したり、publisherの登録などに使う
    ros::NodeHandle nh;

    // 画像をpublish、subscribeするための変数の定義
    // 通常はr ros::Publisher ros::Subscriber 型を用いるが、
    // 画像の場合はファイルの大きさから、image_transportという専用の型を用いる
    image_transport::ImageTransport it(nh);
    image_transport::Publisher image_publisher = it.advertise(topic_name, buf_size);
    image_transport::Publisher depth_image_publisher = it.advertise(depth_topic_name, buf_size);

    // ループの周期を決める 1s に loop_rate だけ実行される
    ros::Rate r(loop_rate);
    int cnt = 0;

    // 画像の内容と、ヘッダも一緒に送るので、ヘッダを定義
    // 現在時刻をタイムスタンプとしておく
    std_msgs::Header header;
    header.stamp = ros::Time::now();

    // 画像をopencvを用いて読み込み、それをROSの方に変換する
    // 画像の場合は cv_bridgeという専用のパッケージを用いて変換する
    // http://wiki.ros.org/image_transport/Tutorials/PublishingImages
    // http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
    cv::Mat image = cv::imread(img_path, cv::IMREAD_UNCHANGED);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();

    cv::Mat depth_image = cv::imread(depth_img_path, cv::IMREAD_UNCHANGED);
    sensor_msgs::ImagePtr depth_msg = cv_bridge::CvImage(header, "mono16", depth_image).toImageMsg();

    // ctrl+C などでノードが停止するまでループ
    while (ros::ok())
    {
        // publish
        image_publisher.publish(msg);
        depth_image_publisher.publish(depth_msg);

        ROS_INFO("publish cnt: %d", ++cnt);

        // 指定したループ周期になるように調整
        r.sleep();
    }

    return 0;
}