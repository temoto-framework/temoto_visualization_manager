#include "ros/ros.h"
#include "temoto_visualization_manager/visualization_manager.h"
#include "temoto_resource_registrar/temoto_logging.h"
#include <boost/program_options.hpp>

int main(int argc, char **argv)
{
  namespace po = boost::program_options;
  po::variables_map vm;
  po::options_description desc("Allowed options");
  desc.add_options()
    ("config-base-path", po::value<std::string>(), "Base path to plugin_description.yaml config file.");

  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  // Get the config path
  std::string config_base_path;
  if (vm.count("config-base-path"))
  {
    config_base_path = vm["config-base-path"].as<std::string>();    
  }
  else
  {
    std::cout << "Missing plugin description config base path\n" << desc;
    return 1;
  }


  TEMOTO_LOG_ATTR.initialize("temoto_visualization_manager");
  ros::init(argc, argv, TEMOTO_LOG_ATTR.getSubsystemName());
  
  // Create instance of rviz manager
  temoto_visualization_manager::VisualizationManager visualization_manager(config_base_path);
  
  ros::spin();
  return 0;
}
