#include "ros/ros.h"
#include "temoto_visualization_manager/visualization_manager.h"
#include "temoto_resource_registrar/temoto_logging.h"

int main(int argc, char **argv)
{
  TEMOTO_LOG_ATTR.initialize("temoto_visualization_manager");
  ros::init(argc, argv, TEMOTO_LOG_ATTR.getSubsystemName());
  
  // Create instance of rviz manager
  temoto_visualization_manager::VisualizationManager visualization_manager;
  
  ros::spin();
  return 0;
}
