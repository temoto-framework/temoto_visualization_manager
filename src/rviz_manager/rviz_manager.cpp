#include "temoto_output_manager/rviz_manager/rviz_manager.h"

namespace temoto_output_manager
{
RvizManager::RvizManager()
  : temoto_core::BaseSubsystem("temoto_output_manager", temoto_core::error::Subsystem::OUTPUT_MANAGER, __func__)
  , resource_registrar_(srv_name::RVIZ_MANAGER, this)
{
  class_name_ = __func__;
  subsystem_name_ = "rviz_manager";
  subsystem_code_ = temoto_core::error::Subsystem::SENSOR_MANAGER;
  log_group_ = "rviz_manager";
  error_handler_ = temoto_core::error::ErrorHandler(subsystem_code_, log_group_);

  // Set up server for loading rviz plugins
  resource_registrar_.addServer<temoto_output_manager::LoadRvizPlugin>(srv_name::RVIZ_SERVER,
                                                        &RvizManager::LoadRvizPluginCb,
                                                        &RvizManager::unloadRvizPluginCb);

  // Load rviz_plugin_manager service clients
  load_plugin_client_ = nh_.serviceClient<rviz_plugin_manager::PluginLoad>("rviz_plugin_load");
  unload_plugin_client_ =
      nh_.serviceClient<rviz_plugin_manager::PluginUnload>("rviz_plugin_unload");
  get_plugin_config_client_ =
      nh_.serviceClient<rviz_plugin_manager::PluginGetConfig>("rviz_plugin_get_"
                                                              "config");
  set_plugin_config_client_ =
      nh_.serviceClient<rviz_plugin_manager::PluginSetConfig>("rviz_plugin_set_"
                                                              "config");

  /*
   * Add some plugin entries to the "plugin_info_handler_". This should be done
   * via
   * external xml file or a service request
   */
  // plugin_info_handler_.plugins_.emplace_back( "marker", "rviz/Marker",
  // "Temoto Marker" );
  plugin_info_handler_.plugins_.emplace_back("marker", "rviz/Marker");
  plugin_info_handler_.plugins_.emplace_back("camera", "rviz_textured_sphere/SphereDisplay",
                                             "Temoto camera");
  plugin_info_handler_.plugins_.emplace_back("image", "rviz/Image", "Temoto Image", "sensor_msgs/Image");
  plugin_info_handler_.plugins_.emplace_back("depth image", "rviz/PointCloud2", "Temoto Pointcloud", "sensor_msgs/PointCloud2");
  plugin_info_handler_.plugins_.emplace_back("laser scan", "rviz/LaserScan", "Temoto Laser Scan", "sensor_msgs/LaserScan");
  plugin_info_handler_.plugins_.emplace_back("path", "rviz/Path", "Path plugin", "");
  plugin_info_handler_.plugins_.emplace_back("robot_model", "rviz/RobotModel", "Robot model plugin", "");
  plugin_info_handler_.plugins_.emplace_back("manipulation", "moveit_rviz_plugin/MotionPlanning", "Moveit Motion Planning", "");

  TEMOTO_INFO_STREAM("RViz Manager is good to go.");
}

/* * * * * * * * * * * * * * * * *
 *  runRviz
 * * * * * * * * * * * * * * * * */
void RvizManager::runRviz()
{
  // Create the message and fill out the request part
  temoto_er_manager::LoadExtResource msg;
  msg.request.action = temoto_er_manager::action::ROS_EXECUTE;
  msg.request.package_name = "rviz_plugin_manager";
  msg.request.executable = "rviz_plugin_manager.launch";

  TEMOTO_INFO("Requesting to launch rviz ...");

  try
  {
    // Ask process manager to start rviz
    resource_registrar_.call<temoto_er_manager::LoadExtResource>(temoto_er_manager::srv_name::MANAGER,
                                                  temoto_er_manager::srv_name::SERVER, msg);
    if (msg.response.trr.code == 0)
    {
      TEMOTO_INFO("Rviz launched succesfully: %s",
                  msg.response.trr.message.c_str());

      // Wait until rviz_plugin_manager clients become active or throw an error on timeout
      ros::Time timeout = ros::Time::now() + ros::Duration(10);
      while ((!load_plugin_client_.exists() || !unload_plugin_client_.exists() ||
              !set_plugin_config_client_.exists() || !get_plugin_config_client_.exists()) &&
             ros::Time::now() < timeout)
      {
        ros::Duration diff = timeout - ros::Time::now();
        TEMOTO_DEBUG("Waiting for rviz to start (timeout in %.1f sec).", diff.toSec());
        ros::Duration(1).sleep();
      }

      if (ros::Time::now() >= timeout)
      {
        throw CREATE_ERROR(temoto_core::error::Code::RVIZ_OPEN_FAIL, "Failed to launch rviz plugin manager: Timeout reached.");
      }
      TEMOTO_DEBUG("All rviz_plugin_manager services connected.");
    }
    else
    {
      throw CREATE_ERROR(temoto_core::error::Code::RVIZ_OPEN_FAIL, "Failed to launch rviz: ");
    }
  }
  catch (...)
  {
    throw CREATE_ERROR(temoto_core::error::Code::SERVICE_REQ_FAIL, "Failed to start RViz");
  }

}

/* * * * * * * * * * * * * * * * *
 *  loadPluginRequest
 * * * * * * * * * * * * * * * * */

bool RvizManager::loadPluginRequest(rviz_plugin_manager::PluginLoad& load_plugin_srv)
{
  // Send the plugin request
  if (load_plugin_client_.call(load_plugin_srv))
  {
    if (load_plugin_srv.response.code == 0)
    {
      TEMOTO_INFO("Request successful: %s", load_plugin_srv.response.message.c_str());
      return true;
    }
    else
    {
      throw CREATE_ERROR(temoto_core::error::Code::PLUGIN_LOAD_FAIL, "Failed to load rviz plugin: " + load_plugin_srv.response.message);
    }
  }
  else
  {
    throw CREATE_ERROR(temoto_core::error::Code::SERVICE_REQ_FAIL, "Failed to call service /rviz_plugin_load");
  }
}

/* * * * * * * * * * * * * * * * *
 *  unloadPluginRequest
 * * * * * * * * * * * * * * * * */

bool RvizManager::unloadPluginRequest(rviz_plugin_manager::PluginUnload& unload_plugin_srv)
{
  // Send the plugin request
  if (unload_plugin_client_.call(unload_plugin_srv))
  {
    if (unload_plugin_srv.response.code == 0)
    {
      TEMOTO_INFO("Request successful: %s", unload_plugin_srv.response.message.c_str());
      return true;
    }
    else
    {
      throw CREATE_ERROR(temoto_core::error::Code::PLUGIN_UNLOAD_FAIL, "Failed to unload rviz plugin: " + unload_plugin_srv.response.message);
    }
  }
  else
  {
    throw CREATE_ERROR(temoto_core::error::Code::SERVICE_REQ_FAIL, "Failed to call service /rviz_plugin_unload");
  }
}

/* * * * * * * * * * * * * * * * *
 *  getPluginConfigRequest
 * * * * * * * * * * * * * * * * */

bool RvizManager::getPluginConfigRequest(
    rviz_plugin_manager::PluginGetConfig& get_plugin_config_srv)
{
  // Send the plugin request
  if (get_plugin_config_client_.call(get_plugin_config_srv))
  {
    if (get_plugin_config_srv.response.code == 0)
    {
      TEMOTO_INFO("Request successful: %s", get_plugin_config_srv.response.message.c_str());
      return true;
    }
    else
    {
      throw CREATE_ERROR(temoto_core::error::Code::PLUGIN_GET_CONFIG_FAIL,
                         "Failed to get rviz plugin config: " +
                             get_plugin_config_srv.response.message);
    }
  }
  else
  {
    throw CREATE_ERROR(temoto_core::error::Code::SERVICE_REQ_FAIL, "Failed to call service /rviz_plugin_get_config");
  }
}

/* * * * * * * * * * * * * * * * *
 *  setPluginConfigRequest
 * * * * * * * * * * * * * * * * */

bool RvizManager::setPluginConfigRequest(
    rviz_plugin_manager::PluginSetConfig& set_plugin_config_srv)
{
  // Send the plugin request
  if (set_plugin_config_client_.call(set_plugin_config_srv))
  {
    if (set_plugin_config_srv.response.code == 0)
    {
      TEMOTO_INFO("Request successful: %s", set_plugin_config_srv.response.message.c_str());
      return true;
    }
    else
    {
      throw CREATE_ERROR(temoto_core::error::Code::PLUGIN_SET_CONFIG_FAIL,
                         "Failed to set rviz plugin config: " +
                             set_plugin_config_srv.response.message);
    }
  }
  else
  {
    throw CREATE_ERROR(temoto_core::error::Code::SERVICE_REQ_FAIL, "Failed to call service /rviz_plugin_set_config");
  }
}

/* * * * * * * * * * * * * * * * *
 *  loadPluginCb
 * * * * * * * * * * * * * * * * */

void RvizManager::LoadRvizPluginCb(temoto_output_manager::LoadRvizPlugin::Request& req,
                                   temoto_output_manager::LoadRvizPlugin::Response& res)
{
  TEMOTO_INFO_STREAM("Received a new %s request: " << srv_name::RVIZ_SERVER);
  TEMOTO_INFO_STREAM(req);

  try
  {
    runRviz();
  }
  catch (temoto_core::error::ErrorStack& error_stack)
  {
    FORWARD_ERROR(error_stack);
  }

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

    try
    {
      loadPluginRequest(load_plugin_srv);

      // Fill out the response message
      // res.plugin_uid = load_plugin_srv.response.plugin_uid;
      // store plugin uid
      res.trr.code = load_plugin_srv.response.code;
      res.trr.message = load_plugin_srv.response.message;

      // Add the request and ID into the active_requests_
      active_requests_.emplace(res.trr.resource_id, load_plugin_srv.response.plugin_uid);

      // \TODO this is now deprecated, clean up!
      // In presence of config, send it to rviz_plugin_manager
     // if (req.config != "")
     // {
     //   //ros::Duration(10).sleep();
     //   rviz_plugin_manager::PluginSetConfig set_plugin_config_srv;
     //   set_plugin_config_srv.request.plugin_uid = load_plugin_srv.response.plugin_uid;
     //   set_plugin_config_srv.request.config = req.config;
     //   setPluginConfigRequest(set_plugin_config_srv);
     // }
    }
    catch (temoto_core::error::ErrorStack& error_stack)
    {
      FORWARD_ERROR(error_stack);
    }
  }
  else
  {
    throw CREATE_ERROR(temoto_core::error::Code::SERVICE_REQ_FAIL, "Did not find any appropriate display plugins");
  }
}

/* * * * * * * * * * * * * * * * *
 *  unloadRvizPluginCb
 * * * * * * * * * * * * * * * * */

void RvizManager::unloadRvizPluginCb(temoto_output_manager::LoadRvizPlugin::Request& req,
                                     temoto_output_manager::LoadRvizPlugin::Response& res)
{
  TEMOTO_INFO("Received an 'unload' request to ID: '%ld'.", res.trr.resource_id);

  // Default the response code to 0
  res.trr.code = 0;

  // Go through the  and "active_requests_" list, look for the plugin_uid

  auto it = active_requests_.find(res.trr.resource_id);
  if (it != active_requests_.end())
  {
    rviz_plugin_manager::PluginUnload plugin_unload_srv;
    plugin_unload_srv.request.plugin_uid = it->second;

    // Call the service
    try
    {
      unloadPluginRequest(plugin_unload_srv);
      active_requests_.erase(it);
      res.trr.code = 0;
      res.trr.message = "Request satisfied";
    }
    catch (temoto_core::error::ErrorStack& error_stack)
    {
      FORWARD_ERROR(error_stack);
    }
  }
  else
  {
    throw CREATE_ERROR(temoto_core::error::Code::PLUGIN_UNLOAD_FAIL, "No allocated resources were found");
  }
}

}  // namespace rviz_manager
