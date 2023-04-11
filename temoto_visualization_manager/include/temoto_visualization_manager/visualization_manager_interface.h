/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Copyright 2019 TeMoto Framework
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#ifndef TEMOTO_VISUALIZATION_MANAGER__VISUALIZATION_MANAGER_INTERFACE_H
#define TEMOTO_VISUALIZATION_MANAGER__VISUALIZATION_MANAGER_INTERFACE_H

#include "rr/ros1_resource_registrar.h"
#include "temoto_visualization_manager/visualization_manager_services.h"
#include "temoto_visualization_manager/LoadRvizPlugin.h"
#include <sstream>
#include <fstream>
#include <memory>
#include <vector>
#include <functional>
#include <yaml-cpp/yaml.h>

namespace temoto_visualization_manager
{

class VisualizationManagerInterface
{
public:

  VisualizationManagerInterface(bool initialize_interface = false)
  : unique_suffix_(std::to_string(createID()))
  , initialized_(false)
  {
    if (initialize_interface)
    {
      initialize();
    }
  }

  void initialize()
  {
    if (!initialized_)
    {
      rr_name_ = TEMOTO_LOG_ATTR.getSubsystemNameWithSlash() + GET_CLASS_NAME + "_" + unique_suffix_;
      resource_registrar_ = std::make_unique<temoto_resource_registrar::ResourceRegistrarRos1>(rr_name_);
      resource_registrar_->init();
      initialized_ = true;
    }
    else
    {
      TEMOTO_WARN_STREAM_("The Visualization Manager interface is already initialized");
    }
  }

  unsigned int createID()
  {
    std::srand(std::time(nullptr));
    return std::rand();
  }

  LoadRvizPlugin loadRvizPlugin(std::string display_class
  , std::string topic = ""
  , std::string display_config = ""
  , std::string displayed_name = ""
  , std::string temoto_namespace = "")
  try
  {
    validateInterface();
    LoadRvizPlugin load_resource_msg;
    load_resource_msg.request.class_name = display_class;
    load_resource_msg.request.topic = topic;
    load_resource_msg.request.config = display_config;
    load_resource_msg.request.displayed_name = displayed_name;

    resource_registrar_->call<LoadRvizPlugin>(srv_name::RVIZ_MANAGER
    , srv_name::RVIZ_SERVER
    , load_resource_msg
    , std::bind(&VisualizationManagerInterface::statusInfoCb, this, std::placeholders::_1, std::placeholders::_2));

    loaded_plugins_.insert({load_resource_msg.response.temoto_metadata.request_id, load_resource_msg});
    return load_resource_msg;
  }
  catch(resource_registrar::TemotoErrorStack e)
  {
    throw FWD_TEMOTO_ERRSTACK(e);
  }

  void unloadRvizPlugin(const LoadRvizPlugin& load_resource_msg)
  try
  {
    validateInterface();
    const auto& loaded_plugin_it = loaded_plugins_.find(load_resource_msg.response.temoto_metadata.request_id);
    if (loaded_plugin_it != loaded_plugins_.end())
    {
      resource_registrar_->unload(srv_name::RVIZ_MANAGER, loaded_plugin_it->first);
      loaded_plugins_.erase(loaded_plugin_it);
    }
    else
    {
      throw TEMOTO_ERRSTACK("Cannot unload plugin " + load_resource_msg.request.class_name + " because it is not loaded yet");
    }
  }
  catch (resource_registrar::TemotoErrorStack e)
  {
    throw FWD_TEMOTO_ERRSTACK(e);
  }

  void unloadRvizPlugin(std::string display_class
  , std::string topic = ""
  , std::string display_config = "")
  try
  {
    validateInterface();
    temoto_visualization_manager::LoadRvizPlugin::Request req;
    req.class_name = display_class;
    req.topic = topic;
    req.config = display_config;

    // Iterate over each plugin that has been loaded

    const auto loaded_plugin_it = std::find_if(loaded_plugins_.begin()
    , loaded_plugins_.end()
    , [&req](const auto& loaded_plugin) { return loaded_plugin.second.request == req;});

    if (loaded_plugin_it != loaded_plugins_.end())
    {
      resource_registrar_->unload(srv_name::RVIZ_MANAGER, loaded_plugin_it->first);
      loaded_plugins_.erase(loaded_plugin_it);
    }
    else
    {
      throw TEMOTO_ERRSTACK("Cannot unload plugin " + req.class_name + " because it is not loaded yet");
    }
  }
  catch (resource_registrar::TemotoErrorStack e)
  {
    throw FWD_TEMOTO_ERRSTACK(e);
  }

  void statusInfoCb(LoadRvizPlugin srv_msg, temoto_resource_registrar::Status status_msg)
  try
  {
    validateInterface();
    TEMOTO_ERROR_STREAM_("status info was received ...");
    
    /*
     * Check if the owner has a status routine defined
     */
    if (user_status_callback_)
    {
      TEMOTO_DEBUG_STREAM_("Invoking user-registered status callback");
      user_status_callback_(srv_msg, status_msg);
      return;
    }
  }
  catch (resource_registrar::TemotoErrorStack e)
  {
    throw FWD_TEMOTO_ERRSTACK(e);
  }

  /**
   * @brief registerUpdateCallback
   */
  void registerUpdateCallback( std::function<void(LoadRvizPlugin, temoto_resource_registrar::Status)> user_status_callback)
  {
    user_status_callback_ = user_status_callback;
  }

  // void showRobot(const std::set<std::string>& visualization_options)
  // {
  //   showRobot("", visualization_options);
  // }

  // void showRobot(const std::string& robot_name,
  //                const std::set<std::string>& visualization_options)
  // {
  //   try
  //   {
  //     YAML::Node info = YAML::Load(getRobotInfo(robot_name));
  //     YAML::Node rviz_node = info["RViz"];

  //     if(!rviz_node.IsMap())
  //     {
  //       throw CREATE_ERROR(temoto_core::error::Code::ROBOT_VIZ_NOT_FOUND, "RViz visualization options are "
  //                                                            "missing.");
  //     }

  //     // Show robot model
  //     if (visualization_options.find("robot_model") != visualization_options.end())
  //     {
  //       showRobotModel(rviz_node);
  //     }

  //     // Show manipulation
  //     if (visualization_options.find("manipulation") != visualization_options.end())
  //     {
  //       showManipulation(rviz_node);
  //     }
  //   }
  //   catch(temoto_core::error::ErrorStack& error_stack)
  //   {
  //     throw FORWARD_ERROR(error_stack);
  //   }
  //   catch(std::exception& e) // capture and wrap possible YAML failures
  //   {
  //     throw CREATE_ERROR(temoto_core::error::Code::UNHANDLED_EXCEPTION, "Unhandled exception: " + std::string(e.what()));
  //   }
  // }

  // void showRobotModel(YAML::Node& rviz_node)
  // {
  //   YAML::Node urdf_node = rviz_node["urdf"];
  //   if (urdf_node.IsMap())
  //   {
  //     std::string robot_desc_param = urdf_node["robot_description"].as<std::string>();
  //     std::string conf = "{Robot Description: " + robot_desc_param + "}";
  //     showInRviz("robot_model", "", conf);
  //   }
  //   else
  //   {
  //     throw CREATE_ERROR(temoto_core::error::Code::ROBOT_FEATURE_NOT_FOUND, "Robot does not have an urdf "
  //                                                              "capability, which is requred to "
  //                                                              "show the robot model.");
  //   }
  // }

  // std::string showManipulation(YAML::Node& rviz_node)
  // {
  //   YAML::Node urdf_node = rviz_node["urdf"];
  //   if (!urdf_node.IsMap())
  //   {
  //     throw CREATE_ERROR(temoto_core::error::Code::ROBOT_FEATURE_NOT_FOUND, "Robot does not have an urdf "
  //                                                              "capability, which is requred to "
  //                                                              "show the manipulation.");
  //   }

  //   YAML::Node manipulation_node = rviz_node["manipulation"];
  //   if (!manipulation_node.IsMap())
  //   {
  //     throw CREATE_ERROR(temoto_core::error::Code::ROBOT_FEATURE_NOT_FOUND, "Robot does not have a manipulation "
  //                                                              "capability, which is requred to "
  //                                                              "show the manipulation.");
  //   }

  //   std::string robot_desc_param = urdf_node["robot_description"].as<std::string>();
  //   std::string move_group_ns = manipulation_node["move_group_ns"].as<std::string>();
  //   std::string active_planning_group = manipulation_node["active_planning_group"].as<std::string>();

  //   std::string conf = "{Robot Description: " + robot_desc_param +
  //                      ", Move Group Namespace: " + move_group_ns + 
  //                      ", Planning Scene Topic: " + move_group_ns + "/move_group/monitored_planning_scene"
  //                      ", Planning Request: {Planning Group: " + active_planning_group + ", Interactive Marker Size: 0.2}"
  //                      ", Planned Path: {Trajectory Topic: " + move_group_ns + "/move_group/display_planned_path}}";
  //   showInRviz("manipulation", "", conf);
  // }

  // // TODO: UNCOMMENT THIS SECTION WHEN ROBOT MANAGER IS CONTAINED WITHIN ITS OWN PACKAGE
  // std::string getRobotInfo(const std::string& robot_name)
  // {
  //   std::string info;
  //   ros::ServiceClient rm_client =
  //       nh_.serviceClient<temoto_robot_manager::RobotGetVizInfo>(robot_manager::srv_name::SERVER_GET_VIZ_INFO);
  //   temoto_robot_manager::RobotGetVizInfo info_srvc;
  //   info_srvc.request.robot_name = robot_name;
  //   if (rm_client.call(info_srvc))
  //   {
  //     TEMOTO_DEBUG(" GET ROBOT INFO SUCESSFUL. Response:");
  //     TEMOTO_DEBUG_STREAM(info_srvc.response);
  //     info = info_srvc.response.info;
  //   }
  //   else
  //   {
  //     throw CREATE_ERROR(temoto_core::error::Code::SERVICE_REQ_FAIL, "Failed to obtain visualization info from robot manager.");
  //   }
  //   return info;
  // }

  /*
   * @brief displayConfigFromFile
   * @param config_path
   * @return
   */
  std::string displayConfigFromFile(std::string config_path)
  {
    // Create filestream object and configure exceptions
    std::ifstream config_file;
    config_file.exceptions(std::ifstream::failbit | std::ifstream::badbit);

    try
    {
      // Open the file stream
      config_file.open(config_path);

      // Stream the file into a stringstream
      std::stringstream sstr;
      while (config_file >> sstr.rdbuf());
      return sstr.str();
    }
    catch (std::ifstream::failure e)
    {
      // Rethrow the exception
      throw TEMOTO_ERRSTACK("Failed to open the display config file");
    }
  }

  const std::string& getName() const
  {
    return rr_name_;
  }

private:

  void validateInterface()
  {
    if (!initialized_)
    {
      throw TEMOTO_ERRSTACK("The Visualization Manager interface is not initialized");
    }
  }

  std::string rr_name_;
  std::string unique_suffix_;
  bool initialized_;
  std::unique_ptr<temoto_resource_registrar::ResourceRegistrarRos1> resource_registrar_;
  std::map<std::string, LoadRvizPlugin> loaded_plugins_;
  std::function<void (LoadRvizPlugin, temoto_resource_registrar::Status)> user_status_callback_ = NULL;
};

} // namespace

#endif