#include "ros/ros.h"
#include "temoto_visualization_manager/visualization_manager_interface.h"

void resourceFailureCallback(temoto_visualization_manager::LoadRvizPlugin load_resource_msg, temoto_resource_registrar::Status status_msg)
{
  ROS_WARN_STREAM("The following resource stopped unexpectedly\n" << load_resource_msg.request);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_er_client_node");
  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();
  ros::Duration(3).sleep();

  /*
   * Create Visualization Manager Interface object that provides a simplified
   * API for communicating with the Visualization Manager. The boolean "true", that's passed
   * to the constructor of the interface tells it whether it should be initialised immediately,
   * or that's done later by the user.
   */
  temoto_visualization_manager::VisualizationManagerInterface vmi(true);

  /*
   * You can register a custom routine (not required) where resource failures are reported.
   */
  vmi.registerUpdateCallback(resourceFailureCallback);

  /*
   * Load a "sensor_msgs/Image" plugin and ask it to subscribe to images on "my/stream/topic_1" topic
   */
  std::string topic_name_1 = "my/stream/topic_1";
  ROS_INFO_STREAM("Asking the visualization manager to show camera stream on topic '" << topic_name_1 << "'");
  auto plugin_handle_1 = vmi.loadRvizPlugin("image", topic_name_1);
  ros::Duration(6).sleep();

  /*
   * Load a "sensor_msgs/LaserScan" plugin and ask it to subscribe to images on "my/scan/topic_1" topic
   */
  std::string topic_name_2 = "my/scan/topic_1";
  ROS_INFO_STREAM("Asking the visualization manager to show LIDAR stream on topic '" << topic_name_2 << "'");
  auto plugin_handle_2 = vmi.loadRvizPlugin("laser_scan", topic_name_2);
  ros::Duration(6).sleep();

  /*
   * Unload the a "sensor_msgs/Image" plugin
   */
  ROS_INFO("Unloading the camera stream visualizer");
  vmi.unloadRvizPlugin(plugin_handle_1);
  ros::Duration(5).sleep();
  
  /*
   * Note that this time the "unloadRvizPlugin" was not invoked, as the destructor of "vmi" automatically
   * unloads all loaded resources.
   */ 
  return 0;
}