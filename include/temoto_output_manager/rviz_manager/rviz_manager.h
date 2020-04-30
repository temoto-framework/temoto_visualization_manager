/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Copyright 2019 TeMoto Telerobotics
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

/* Author: Veiko Vunder */
/* Author: Robert Valner */

#ifndef TEMOTO_OUTPUT_MANAGER__RVIZ_MANAGER_H
#define TEMOTO_OUTPUT_MANAGER__RVIZ_MANAGER_H

#include "temoto_core/common/request_container.h"
#include "temoto_core/common/temoto_id.h"
#include "temoto_core/common/base_subsystem.h"
#include "temoto_output_manager/LoadRvizPlugin.h"
#include "temoto_output_manager/rviz_manager/plugin_info.h"
#include "temoto_output_manager/temoto_output_manager_services.h"
#include "temoto_er_manager/temoto_er_manager_services.h"
#include "temoto_core/trr/resource_registrar.h"
#include "rviz_plugin_manager/PluginLoad.h"
#include "rviz_plugin_manager/PluginUnload.h"
#include "rviz_plugin_manager/PluginGetConfig.h"
#include "rviz_plugin_manager/PluginSetConfig.h"
//#include "temoto_output_manager/stopAllocatedServices.h"

namespace temoto_output_manager
{

class RvizManager : public temoto_core::BaseSubsystem
{
public:
  RvizManager();

  RvizManager(std::string path_to_default_conf);

  const std::string& getName() const
  {
    return subsystem_name_;
  }

private:

  void runRviz();

  bool loadPluginRequest(rviz_plugin_manager::PluginLoad& load_plugin_srv);

  bool unloadPluginRequest(rviz_plugin_manager::PluginUnload& unload_plugin_srv);

  bool getPluginConfigRequest(rviz_plugin_manager::PluginGetConfig& get_plugin_config_srv);

  bool setPluginConfigRequest(rviz_plugin_manager::PluginSetConfig& set_plugin_config_srv);

  void LoadRvizPluginCb(temoto_output_manager::LoadRvizPlugin::Request& req,
                        temoto_output_manager::LoadRvizPlugin::Response& res);

  void unloadRvizPluginCb(temoto_output_manager::LoadRvizPlugin::Request& req,
                          temoto_output_manager::LoadRvizPlugin::Response& res);

  PluginInfo findPlugin(std::string plugin_type);

  std::map<long, temoto_core::temoto_id::ID> active_requests_;

  temoto_core::trr::ResourceRegistrar<RvizManager> resource_registrar_;

  ros::NodeHandle nh_;

  ros::ServiceClient load_plugin_client_;

  ros::ServiceClient unload_plugin_client_;

  ros::ServiceClient set_plugin_config_client_;

  ros::ServiceClient get_plugin_config_client_;

  PluginInfoHandler plugin_info_handler_;
};

}  // namespace rviz_manager

#endif
