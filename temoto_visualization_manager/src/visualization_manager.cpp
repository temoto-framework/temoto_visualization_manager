#include "temoto_visualization_manager/visualization_manager.h"
#include <yaml-cpp/yaml.h>
#include <fstream>

namespace temoto_visualization_manager
{
VisualizationManager::VisualizationManager(const std::string& config_base_path)
: resource_registrar_(srv_name::RVIZ_MANAGER)
{
  /*
   * Configure the RR catalog backup routine
   */
  std::string home_path = std::getenv("HOME");
  std::string rr_catalog_backup_path = home_path + "/.temoto/" + srv_name::RVIZ_MANAGER + ".rrcat";
  rr_catalog_config_.setName(srv_name::RVIZ_MANAGER);
  rr_catalog_config_.setLocation(rr_catalog_backup_path);
  rr_catalog_config_.setSaveOnModify(true);
  rr_catalog_config_.setEraseOnDestruct(true);
  resource_registrar_.updateConfiguration(rr_catalog_config_);

  /*
   * Add the LoadRvizPlugin server to the resource registrar
   */
  auto server = std::make_unique<Ros1Server<LoadRvizPlugin>>(srv_name::RVIZ_SERVER
  , std::bind(&VisualizationManager::loadRvizPluginCb, this, std::placeholders::_1, std::placeholders::_2)
  , std::bind(&VisualizationManager::unloadRvizPluginCb, this, std::placeholders::_1, std::placeholders::_2));
  resource_registrar_.registerServer(std::move(server));
  resource_registrar_.init();

  /*
   * Load rviz_plugin_manager service clients
   */
  load_plugin_client_ = nh_.serviceClient<rviz_plugin_manager::PluginLoad>("rviz_plugin_load");
  unload_plugin_client_ = nh_.serviceClient<rviz_plugin_manager::PluginUnload>("rviz_plugin_unload");
  get_plugin_config_client_ = nh_.serviceClient<rviz_plugin_manager::PluginGetConfig>("rviz_plugin_get_config");
  set_plugin_config_client_ = nh_.serviceClient<rviz_plugin_manager::PluginSetConfig>("rviz_plugin_set_config");

  /*
   * Add some plugin entries to the "plugin_info_handler_". 
   * TODO: This should be done via external xml file or a service request
   */

  
  findPluginDescriptionFiles(config_base_path);

  // plugin_info_handler_.plugins_.emplace_back("marker", "rviz/Marker");
  // plugin_info_handler_.plugins_.emplace_back("interactive_marker", "rviz/InteractiveMarkers", "Temoto marker");
  // plugin_info_handler_.plugins_.emplace_back("camera", "rviz_textured_sphere/SphereDisplay", "Temoto camera");
  // plugin_info_handler_.plugins_.emplace_back("image", "rviz/Image", "Temoto Image", "sensor_msgs/Image");
  // plugin_info_handler_.plugins_.emplace_back("compressed_image", "rviz/Image", "Temoto Compressed Image", "sensor_msgs/CompressedImage");
  // plugin_info_handler_.plugins_.emplace_back("depth image", "rviz/PointCloud2", "Temoto Pointcloud", "sensor_msgs/PointCloud2");
  // plugin_info_handler_.plugins_.emplace_back("laser_scan", "rviz/LaserScan", "Temoto Laser Scan", "sensor_msgs/LaserScan");
  // plugin_info_handler_.plugins_.emplace_back("path", "rviz/Path", "Path plugin", "");
  // plugin_info_handler_.plugins_.emplace_back("robot_model", "rviz/RobotModel", "Robot model plugin", "");
  // plugin_info_handler_.plugins_.emplace_back("manipulation", "moveit_rviz_plugin/MotionPlanning", "Moveit Motion Planning", "");
  // plugin_info_handler_.plugins_.emplace_back("map", "rviz/Map", "Map");
  // plugin_info_handler_.plugins_.emplace_back("DepthCloud", "rviz/DepthCloud", "DepthCloud");
  

  TEMOTO_INFO_STREAM_("Visualization Manager is good to go.");
}

/* * * * * * * * * * * * * * * * *
 *  runRviz
 * * * * * * * * * * * * * * * * */
void VisualizationManager::runRviz()
try
{
  // Create the message and fill out the request part
  temoto_er_manager::LoadExtResource load_er_msg;
  load_er_msg.request.action = temoto_er_manager::action::ROS_EXECUTE;
  load_er_msg.request.package_name = "rviz_plugin_manager";
  load_er_msg.request.executable = "rviz_plugin_manager.launch";

  TEMOTO_DEBUG_("Requesting to launch rviz ...");

  resource_registrar_.call<temoto_er_manager::LoadExtResource>(temoto_er_manager::srv_name::MANAGER
  , temoto_er_manager::srv_name::SERVER
  , load_er_msg
  , std::bind(&VisualizationManager::erStatusCb, this, std::placeholders::_1, std::placeholders::_2));

  TEMOTO_DEBUG_("Rviz launched");

  // Wait until rviz_plugin_manager clients become active or throw an error on timeout
  ros::Time timeout = ros::Time::now() + ros::Duration(10);
  while ((!load_plugin_client_.exists() || !unload_plugin_client_.exists() ||
          !set_plugin_config_client_.exists() || !get_plugin_config_client_.exists()) &&
          ros::Time::now() < timeout)
  {
    ros::Duration diff = timeout - ros::Time::now();
    TEMOTO_DEBUG_("Waiting for rviz to start (timeout in %.1f sec).", diff.toSec());
    ros::Duration(1).sleep();
  }

  if (ros::Time::now() >= timeout)
  {
    throw TEMOTO_ERRSTACK("Failed to launch rviz plugin manager: Timeout reached.");
  }
  TEMOTO_DEBUG_("All rviz_plugin_manager services connected.");
}
catch (...)
{
  throw TEMOTO_ERRSTACK("Failed to start RViz");
}


/* * * * * * * * * * * * * * * * *
 *  Load process status callback
 * * * * * * * * * * * * * * * * */
void VisualizationManager::erStatusCb(temoto_er_manager::LoadExtResource srv_msg
, temoto_resource_registrar::Status status_msg)
{
  TEMOTO_WARN_STREAM_("Received a resource status message: " << status_msg.message_);
}

/* * * * * * * * * * * * * * * * *
 *  loadPluginRequest
 * * * * * * * * * * * * * * * * */
bool VisualizationManager::loadPluginRequest(rviz_plugin_manager::PluginLoad& load_plugin_srv)
{
  // Send the plugin request
  if (load_plugin_client_.call(load_plugin_srv))
  {
    if (load_plugin_srv.response.code == 0)
    {
      TEMOTO_INFO_("Request successful: %s", load_plugin_srv.response.message.c_str());
      return true;
    }
    else
    {
      throw TEMOTO_ERRSTACK("Failed to load rviz plugin: " + load_plugin_srv.response.message);
    }
  }
  else
  {
    throw TEMOTO_ERRSTACK("Failed to call service /rviz_plugin_load");
  }
}

/* * * * * * * * * * * * * * * * *
 *  unloadPluginRequest
 * * * * * * * * * * * * * * * * */

bool VisualizationManager::unloadPluginRequest(rviz_plugin_manager::PluginUnload& unload_plugin_srv)
{
  // Send the plugin request
  if (unload_plugin_client_.call(unload_plugin_srv))
  {
    if (unload_plugin_srv.response.code == 0)
    {
      TEMOTO_INFO_("Request successful: %s", unload_plugin_srv.response.message.c_str());
      return true;
    }
    else
    {
      throw TEMOTO_ERRSTACK("Failed to unload rviz plugin: " + unload_plugin_srv.response.message);
    }
  }
  else
  {
    throw TEMOTO_ERRSTACK("Failed to call service /rviz_plugin_unload");
  }
}

/* * * * * * * * * * * * * * * * *
 *  getPluginConfigRequest
 * * * * * * * * * * * * * * * * */

bool VisualizationManager::getPluginConfigRequest(
  rviz_plugin_manager::PluginGetConfig& get_plugin_config_srv)
{
  // Send the plugin request
  if (get_plugin_config_client_.call(get_plugin_config_srv))
  {
    if (get_plugin_config_srv.response.code == 0)
    {
      TEMOTO_INFO_("Request successful: %s", get_plugin_config_srv.response.message.c_str());
      return true;
    }
    else
    {
      throw TEMOTO_ERRSTACK("Failed to get rviz plugin config: " + get_plugin_config_srv.response.message);
    }
  }
  else
  {
    throw TEMOTO_ERRSTACK("Failed to call service /rviz_plugin_get_config");
  }
}

/* * * * * * * * * * * * * * * * *
 *  setPluginConfigRequest
 * * * * * * * * * * * * * * * * */

bool VisualizationManager::setPluginConfigRequest(
    rviz_plugin_manager::PluginSetConfig& set_plugin_config_srv)
{
  // Send the plugin request
  if (set_plugin_config_client_.call(set_plugin_config_srv))
  {
    if (set_plugin_config_srv.response.code == 0)
    {
      TEMOTO_INFO_("Request successful: %s", set_plugin_config_srv.response.message.c_str());
      return true;
    }
    else
    {
      throw TEMOTO_ERRSTACK("Failed to set rviz plugin config: " + set_plugin_config_srv.response.message);
    }
  }
  else
  {
    throw TEMOTO_ERRSTACK("Failed to call service /rviz_plugin_set_config");
  }
}

/* * * * * * * * * * * * * * * * *
 *  loadPluginCb
 * * * * * * * * * * * * * * * * */

void VisualizationManager::loadRvizPluginCb(temoto_visualization_manager::LoadRvizPlugin::Request& req,
                                   temoto_visualization_manager::LoadRvizPlugin::Response& res)
try
{ START_SPAN
  TEMOTO_INFO_STREAM_("Received a request to load a RViz plugin: " << req);
  runRviz();

  // Check the type of the requested display plugin and run if found
  PluginInfo plugin_info;

  // Create the message and fill out the request part
  if (plugin_info_handler_.findPlugin(req.type, plugin_info))
  {
    rviz_plugin_manager::PluginLoad load_plugin_srv;
    load_plugin_srv.request.plugin_class = plugin_info.getClassName();
    load_plugin_srv.request.plugin_topic = req.topic;
    load_plugin_srv.request.plugin_config = req.config;
    load_plugin_srv.request.plugin_data_type = plugin_info.getDataType();
    load_plugin_srv.request.plugin_name = plugin_info.getRvizName();

    loadPluginRequest(load_plugin_srv);

    // Add the request and ID into the active_requests_
    active_requests_.emplace(res.temoto_metadata.request_id, load_plugin_srv.response.plugin_uid);
  }
  else
  {
    throw TEMOTO_ERRSTACK("Did not find any appropriate display plugins");
  }
}
catch(resource_registrar::TemotoErrorStack e)
{
  throw FWD_TEMOTO_ERRSTACK(e);
}

/* * * * * * * * * * * * * * * * *
 *  unloadRvizPluginCb
 * * * * * * * * * * * * * * * * */

void VisualizationManager::unloadRvizPluginCb(temoto_visualization_manager::LoadRvizPlugin::Request& req,
                                     temoto_visualization_manager::LoadRvizPlugin::Response& res)
try
{
  TEMOTO_INFO_STREAM_("Unloading RViz display: " << req);

  // Go through the  and "active_requests_" list, look for the plugin_uid
  auto it = active_requests_.find(res.temoto_metadata.request_id);
  if (it != active_requests_.end())
  {
    rviz_plugin_manager::PluginUnload plugin_unload_srv;
    plugin_unload_srv.request.plugin_uid = it->second;

    unloadPluginRequest(plugin_unload_srv);
    active_requests_.erase(it);
  }
  else
  {
    throw TEMOTO_ERRSTACK("No allocated resources were found");
  }
}
catch (resource_registrar::TemotoErrorStack e)
{
  throw FWD_TEMOTO_ERRSTACK(e);
}

void VisualizationManager::findPluginDescriptionFiles(boost::filesystem::path current_dir)
{ 
  boost::filesystem::directory_iterator end_itr;
  for ( boost::filesystem::directory_iterator itr( current_dir ); itr != end_itr; ++itr )
  {
    if (boost::filesystem::is_regular_file(*itr) && (itr->path().filename() == description_file_))
    {
      TEMOTO_INFO_STREAM_("NEW PLUGIN FILE FOUND");
      TEMOTO_INFO_STREAM_(itr->path().string());   
      readPluginDescription(itr->path().string());
    }
    else if ( boost::filesystem::is_directory(*itr) )
    {
      findPluginDescriptionFiles(*itr);
    }
  }
}

void VisualizationManager::readPluginDescription(const std::string& path_to_plugin_description)
{
  std::ifstream in(path_to_plugin_description);
  YAML::Node yaml_config = YAML::Load(in);  
 
  if (yaml_config["Plugins"])
  {
    YAML::Node plugins_config = yaml_config["Plugins"];

    if (!plugins_config.IsSequence())
    {
      TEMOTO_WARN_("The given config does not contain sequence of plugins.");   
    }

    for (YAML::const_iterator node_it = plugins_config.begin(); node_it != plugins_config.end(); ++node_it)
    {
      if (!node_it->IsMap())
      {
        TEMOTO_ERROR_("Unable to parse the plugins config. Parameters in YAML have to be specified in "
                    "key-value pairs.");
        continue;
      }
      const YAML::Node& plugin = *node_it;      
      std::string rviz_name;
      std::string data_type;
      if (!plugin["type"] || !plugin["class_name"])
      {
        TEMOTO_WARN_("Invalid description of plugin, It does not contain a type or class_name");        
      }
      else 
      {
        if(plugin["rviz_name"])
        {
          rviz_name = plugin["rviz_name"].as<std::string>();
        }
        else
        {
          rviz_name = "temoto_" + plugin["type"].as<std::string>();
        }
        if(plugin["data_type"])
        {
          data_type = plugin["data_type"].as<std::string>();
        }
        else
        {
          data_type = "";
        }
        plugin_info_handler_.plugins_.emplace_back(plugin["type"].as<std::string>(),
                                                  plugin["class_name"].as<std::string>(),
                                                  rviz_name, data_type);
      }
    }
  }
  else
  {
    TEMOTO_INFO_STREAM_("No plugings defined");
  }
}

}  // namespace visualization_manager
