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

#ifndef TEMOTO_VISUALIZATION_MANAGER__RVIZ_MANAGER_H
#define TEMOTO_VISUALIZATION_MANAGER__RVIZ_MANAGER_H

#include "rr/ros1_resource_registrar.h"
#include "temoto_visualization_manager/LoadRvizPlugin.h"
#include "temoto_visualization_manager/plugin_info.h"
#include "temoto_visualization_manager/visualization_manager_services.h"
#include "rviz_plugin_manager/PluginLoad.h"
#include "rviz_plugin_manager/PluginUnload.h"
#include "rviz_plugin_manager/PluginGetConfig.h"
#include "rviz_plugin_manager/PluginSetConfig.h"
#include "temoto_er_manager/temoto_er_manager_services.h"

namespace temoto_visualization_manager
{

class VisualizationManager
{
public:
  VisualizationManager();

  // VisualizationManager(std::string path_to_default_conf);
  VisualizationManager(const std::string& config_base_path);

private:

  void runRviz();

  bool loadPluginRequest(rviz_plugin_manager::PluginLoad& load_plugin_srv);

  bool unloadPluginRequest(rviz_plugin_manager::PluginUnload& unload_plugin_srv);

  bool getPluginConfigRequest(rviz_plugin_manager::PluginGetConfig& get_plugin_config_srv);

  bool setPluginConfigRequest(rviz_plugin_manager::PluginSetConfig& set_plugin_config_srv);

  void loadRvizPluginCb(temoto_visualization_manager::LoadRvizPlugin::Request& req,
                        temoto_visualization_manager::LoadRvizPlugin::Response& res);

  void unloadRvizPluginCb(temoto_visualization_manager::LoadRvizPlugin::Request& req,
                          temoto_visualization_manager::LoadRvizPlugin::Response& res);

  void erStatusCb(temoto_er_manager::LoadExtResource srv_msg
  , temoto_resource_registrar::Status status_msg);

  void findPluginDescriptionFiles(const std::string& current_dir);

  void readPluginDescription(const std::string& path_to_plugin_description);

  PluginInfo findPlugin(std::string plugin_type);

  std::map<std::string, int> active_requests_;

  temoto_resource_registrar::ResourceRegistrarRos1 resource_registrar_;
  temoto_resource_registrar::Configuration rr_catalog_config_;

  ros::NodeHandle nh_;

  ros::ServiceClient load_plugin_client_;

  ros::ServiceClient unload_plugin_client_;

  ros::ServiceClient set_plugin_config_client_;

  ros::ServiceClient get_plugin_config_client_;

  PluginInfoHandler plugin_info_handler_;

  /// Name of the plugin description file
  std::string description_file_= "plugin_description.yaml";
};

}  // namespace visualization_manager

#endif
