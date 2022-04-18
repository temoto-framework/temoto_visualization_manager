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

#ifndef TEMOTO_VISUALIZATION_MANAGER__PLUGIN_INFO_H
#define TEMOTO_VISUALIZATION_MANAGER__PLUGIN_INFO_H

#include <string>
#include <vector>
#include <map>
#include <ctype.h>

class PluginInfo
{
public:

    /**
     * @brief plugin_info
     */
    PluginInfo ();

//    PluginInfo (std::string type, std::string class_name);

 //   PluginInfo (std::string type, std::string class_name, std::string rviz_name);

    PluginInfo (std::string class_name,
                std::string rviz_name = "",
                std::string data_type = "");

    void setDescription (std::string description);

    void setRvizName (std::string rviz_name);

    std::string getClassName ();

    std::string getDataType ();

    std::string getRvizName ();

    std::string getDescription ();

private:

    /**
     * @brief Name of the plugin
     */
    std::string class_name_;

    /**
     * @brief A data type of the plugin, e.g., std_msgs/Image.
     */
    std::string data_type_;

    /**
     * @brief Plugin name that appears in rviz plugin list, e.g., 'My Marker'.
     */
    std::string rviz_name_;

    /**
     * @brief A description of the plugin
     */
    std::string description_;
};


class PluginInfoHandler
{
public:

    /**
     * @brief Returns the first plugin that satisfies the "plugin type" condition
     * @param plugin_class
     * @param plugin_info
     * @return
     */
    bool findPlugin ( std::string plugin_class, PluginInfo& plugin_info);

    /**
     * @brief Returns a vector of plugins if the "plugin type" condition is satisfied
     * @param plugin_type
     * @return
     */
    std::vector <PluginInfo> findPlugins ( std::string plugin_class );

    std::vector<PluginInfo> plugins_;
};

#endif
