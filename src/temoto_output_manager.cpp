#include "ros/ros.h"
#include "std_msgs/String.h"
#include "temoto_output_manager/rviz_manager/rviz_manager.h"

#include <sstream>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "temoto_output_manager");
    ros::NodeHandle n;

    // Create instance of rviz manager
    temoto_output_manager::RvizManager rviz_manager;

    ros::Rate loop_rate(10);
    ros::spin();
    return 0;
}
