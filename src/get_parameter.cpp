#include <ros/ros.h>

int main(int argc, char* argv[])
{

    ros::init(argc, argv, "get_parameter_sample_node");

    // nodehandle
    ros::NodeHandle nh;

    std::string param1, param2;
    
    nh.param("param1", param1, std::string("get_parameter_sample_node"));
    nh.param("/ros_tutorial_image_subscriber/node_param", param2, std::string("get_paramter_sample_node"));
    
    std::cout << "[parameter]" << std::endl;
    std::cout << "param1 " << param1 << std::endl;
    std::cout << "param2 " << param2 << std::endl;


    ros::spin();
    

    return 0;
}