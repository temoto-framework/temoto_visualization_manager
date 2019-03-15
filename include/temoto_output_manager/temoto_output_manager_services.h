#ifndef OUTPUT_MANAGER_SERVICES_H
#define OUTPUT_MANAGER_SERVICES_H

#include <string>
#include "temoto_core/rmp/resource_manager_services.h"
#include "temoto_output_manager/LoadRvizPlugin.h"

namespace temoto_output_manager
{
	namespace srv_name
	{
		//const std::string MANAGER = "temoto_output_manager";
		const std::string RVIZ_MANAGER = "rviz_manager";
		const std::string RVIZ_SERVER = "load_rviz_plugin";
	}
}

static bool operator==(const temoto_output_manager::LoadRvizPlugin::Request& r1,
		const temoto_output_manager::LoadRvizPlugin::Request& r2)
{
	return(
			r1.type == r2.type &&
			r1.name == r2.name &&
			r1.topic == r2.topic &&
			r1.config == r2.config
		  );
}
#endif
