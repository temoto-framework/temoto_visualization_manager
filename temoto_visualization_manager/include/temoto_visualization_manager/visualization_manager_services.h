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

#ifndef TEMOTO_VISUALIZATION_MANAGER__VISUALIZATION_MANAGER_SERVICES_H
#define TEMOTO_VISUALIZATION_MANAGER__VISUALIZATION_MANAGER_SERVICES_H

#include <string>
#include "temoto_visualization_manager/LoadRvizPlugin.h"

namespace temoto_visualization_manager
{
	namespace srv_name
	{
		//const std::string MANAGER = "temoto_visualization_manager";
		const std::string RVIZ_MANAGER = "temoto_visualization_manager";
		const std::string RVIZ_SERVER = "load_rviz_plugin";
	}
}

static bool operator==(const temoto_visualization_manager::LoadRvizPlugin::Request& r1,
		const temoto_visualization_manager::LoadRvizPlugin::Request& r2)
{
	return(
			r1.class_name == r2.class_name &&
			r1.topic == r2.topic &&
			r1.config == r2.config
		  );
}
#endif
