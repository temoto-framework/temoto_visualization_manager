#include "temoto_core/common/temoto_id.h"
#include "temoto_core/common/base_subsystem.h"
#include "temoto_core/rmp/resource_manager.h"
#include "temoto_nlp/base_task/base_task.h"
#include "temoto_robot_manager/robot_manager_services.h"
#include "temoto_output_manager/temoto_output_manager_services.h"
#include "temoto_output_manager/LoadRvizPlugin.h"
#include <sstream>
#include <fstream>
#include <memory>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace temoto_output_manager
{

namespace generic_topics
{
  const std::string MARKER = "marker_topic";
}

/**
 * @brief The OutputManagerInterface class
 */
template <class OwnerTask>
class OutputManagerInterface : public temoto_core::BaseSubsystem
{
public:
  /**
   * @brief OutputManagerInterface
   */
  OutputManagerInterface()
  {
    class_name_ = __func__;
  }

  /**
   * @brief initialize
   */
  void initialize(temoto_nlp::BaseTask* task)
  {
    initializeBase(task);
    log_group_ = "interfaces." + task->getPackageName();

    name_ = task->getName() + "/temoto_output_manager_interface";
    resource_manager_ = std::unique_ptr<temoto_core::rmp::ResourceManager<OutputManagerInterface>>(
        new temoto_core::rmp::ResourceManager<OutputManagerInterface>(name_, this));
    //    resource_manager_->registerStatusCb(&OutputManagerInterface::statusInfoCb);
  }

  /**
   * @brief showInRviz
   * @param display_type
   * @param topic
   * @param display_config
   */
  void showInRviz(std::string display_type, std::string topic = "", std::string display_config = "")
  {
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = temoto_core::common::generateLogPrefix(log_subsys_, log_class_, __func__);
    validateInterface(prefix);

    temoto_output_manager::LoadRvizPlugin load_srv;
    load_srv.request.type = display_type;
    load_srv.request.topic = topic;
    load_srv.request.config = display_config;

    // Call the server
    try
    {
      resource_manager_->template call<temoto_output_manager::LoadRvizPlugin>(
          srv_name::RVIZ_MANAGER, srv_name::RVIZ_SERVER, load_srv);
    }
    catch(temoto_core::error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }
    plugins_.push_back(load_srv);
  }

  /**
   * @brief hideInRviz
   * @param display_type
   * @param topic
   * @param display_config
   */
  void hideInRviz(std::string display_type, std::string topic = "", std::string display_config = "")
  {
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = temoto_core::common::generateLogPrefix(log_subsys_, log_class_, __func__);
    validateInterface(prefix);

    temoto_output_manager::LoadRvizPlugin::Request req;
    req.type = display_type;
    req.topic = topic;
    req.config = display_config;

    bool plugin_unloaded = false;

    // Iterate over each plugin that has been loaded
    for (auto cur_plugin_it = plugins_.begin(); cur_plugin_it != plugins_.end(); /* empty */)
    {
      // The == operator used in the lambda function is defined in temoto_output_manager_services.h
      if ( !(cur_plugin_it->request == req) )
      {
        cur_plugin_it++;
        continue;
      }

      // do the unloading
      resource_manager_->unloadClientResource(cur_plugin_it->response.rmp.resource_id);
      plugins_.erase(cur_plugin_it);
      plugin_unloaded = true;
    }

    if (!plugin_unloaded)
    {
      throw CREATE_ERROR(temoto_core::error::Code::RESOURCE_UNLOAD_FAIL, "Unable to unload resource that is not loaded.");
    }
  }

  void showRobot(const std::set<std::string>& visualization_options)
  {
    showRobot("", visualization_options);
  }

  void showRobot(const std::string& robot_name,
                 const std::set<std::string>& visualization_options)
  {
    try
    {
      YAML::Node info = YAML::Load(getRobotInfo(robot_name));
      YAML::Node rviz_node = info["RViz"];

      if(!rviz_node.IsMap())
      {
        throw CREATE_ERROR(temoto_core::error::Code::ROBOT_VIZ_NOT_FOUND, "RViz visualization options are "
                                                             "missing.");
      }

      // Show robot model
      if (visualization_options.find("robot_model") != visualization_options.end())
      {
        showRobotModel(rviz_node);
      }

      // Show manipulation
      if (visualization_options.find("manipulation") != visualization_options.end())
      {
        showManipulation(rviz_node);
      }
    }
    catch(temoto_core::error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }
    catch(std::exception& e) // capture and wrap possible YAML failures
    {
      throw CREATE_ERROR(temoto_core::error::Code::UNHANDLED_EXCEPTION, "Unhandled exception: " + std::string(e.what()));
    }
  }

  void showRobotModel(YAML::Node& rviz_node)
  {
    YAML::Node urdf_node = rviz_node["urdf"];
    if (urdf_node.IsMap())
    {
      std::string robot_desc_param = urdf_node["robot_description"].as<std::string>();
      std::string conf = "{Robot Description: " + robot_desc_param + "}";
      showInRviz("robot_model", "", conf);
    }
    else
    {
      throw CREATE_ERROR(temoto_core::error::Code::ROBOT_FEATURE_NOT_FOUND, "Robot does not have an urdf "
                                                               "capability, which is requred to "
                                                               "show the robot model.");
    }
  }

  std::string showManipulation(YAML::Node& rviz_node)
  {
    YAML::Node urdf_node = rviz_node["urdf"];
    if (!urdf_node.IsMap())
    {
      throw CREATE_ERROR(temoto_core::error::Code::ROBOT_FEATURE_NOT_FOUND, "Robot does not have an urdf "
                                                               "capability, which is requred to "
                                                               "show the manipulation.");
    }

    YAML::Node manipulation_node = rviz_node["manipulation"];
    if (!manipulation_node.IsMap())
    {
      throw CREATE_ERROR(temoto_core::error::Code::ROBOT_FEATURE_NOT_FOUND, "Robot does not have a manipulation "
                                                               "capability, which is requred to "
                                                               "show the manipulation.");
    }

    std::string robot_desc_param = urdf_node["robot_description"].as<std::string>();
    std::string move_group_ns = manipulation_node["move_group_ns"].as<std::string>();
    std::string active_planning_group = manipulation_node["active_planning_group"].as<std::string>();

    std::string conf = "{Robot Description: " + robot_desc_param +
                       ", Move Group Namespace: " + move_group_ns + 
                       ", Planning Scene Topic: " + move_group_ns + "/move_group/monitored_planning_scene"
                       ", Planning Request: {Planning Group: " + active_planning_group + ", Interactive Marker Size: 0.2}"
                       ", Planned Path: {Trajectory Topic: " + move_group_ns + "/move_group/display_planned_path}}";
    showInRviz("manipulation", "", conf);
  }

  // TODO: UNCOMMENT THIS SECTION WHEN ROBOT MANAGER IS CONTAINED WITHIN ITS OWN PACKAGE
  std::string getRobotInfo(const std::string& robot_name)
  {
    std::string info;
    ros::ServiceClient rm_client =
        nh_.serviceClient<temoto_robot_manager::RobotGetVizInfo>(robot_manager::srv_name::SERVER_GET_VIZ_INFO);
    temoto_robot_manager::RobotGetVizInfo info_srvc;
    info_srvc.request.robot_name = robot_name;
    if (rm_client.call(info_srvc))
    {
      TEMOTO_DEBUG(" GET ROBOT INFO SUCESSFUL. Response:");
      TEMOTO_DEBUG_STREAM(info_srvc.response);
      info = info_srvc.response.info;
    }
    else
    {
      throw CREATE_ERROR(temoto_core::error::Code::SERVICE_REQ_FAIL, "Failed to obtain visualization info from robot manager.");
    }
    return info;
  }

  /*
   * @brief displayConfigFromFile
   * @param config_path
   * @return
   */
  std::string displayConfigFromFile(std::string config_path)
  {
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = temoto_core::common::generateLogPrefix(log_subsys_, log_class_, __func__);
    validateInterface(prefix);

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
      throw CREATE_ERROR(temoto_core::error::Code::CONFIG_OPEN_FAIL, "Failed to open the display config file");
    }
  }

  //  void statusInfoCb(temoto_core::ResourceStatus& srv)
  //  {
  //    std::string prefix = temoto_core::common::generateLogPrefix(log_subsys_, log_class_, __func__);
  //    validateInterface(prefix);
  //
  //    TEMOTO_DEBUG("%s status info was received", prefix.c_str());
  //    TEMOTO_DEBUG_STREAM(srv.request);
  //    // if any resource should fail, just unload it and try again
  //    // there is a chance that sensor manager gives us better sensor this time
  //    if (srv.request.status_code == temoto_core::rmp::status_codes::FAILED)
  //    {
  //      TEMOTO_WARN("Output manager interface detected a sensor failure. Unloading and "
  //                                "trying again");
  //      auto sens_it = std::find_if(allocated_sensors_.begin(), allocated_sensors_.end(),
  //                                  [&](const temoto_2::LoadSensor& sens) -> bool {
  //                                    return sens.response.rmp.resource_id ==
  //                                    srv.request.resource_id;
  //                                  });
  //      if (sens_it != allocated_sensors_.end())
  //      {
  //        TEMOTO_DEBUG("Unloading");
  //        resource_manager_->unloadClientResource(sens_it->response.rmp.resource_id);
  //        TEMOTO_DEBUG("Asking the same sensor again");
  //        if (!resource_manager_->template call<temoto_2::LoadSensor>(
  //                sensor_manager::srv_name::MANAGER, sensor_manager::srv_name::SERVER, *sens_it))
  //        {
  //          throw CREATE_ERROR(temoto_core::error::Code::SERVICE_REQ_FAIL, "Failed to call service");
  //        }
  //
  //        // If the request was fulfilled, then add the srv to the list of allocated sensors
  //        if (sens_it->response.rmp.code == 0)
  //        {
  //          // @TODO: send somehow topic to whoever is using this thing
  //          // or do topic remapping
  //        }
  //        else
  //        {
  //          throw CREATE_ERROR(temoto_core::error::Code::SERVICE_REQ_FAIL, "Unsuccessful call to sensor manager: ");
  //        }
  //      }
  //      else
  //      {
  //      }
  //    }
  //  }

  ~OutputManagerInterface()
  {
    // Name of the method, used for making debugging a bit simpler
    std::string prefix = temoto_core::common::generateLogPrefix(log_subsys_, log_class_, __func__);

    TEMOTO_DEBUG("OutputManagerInterface destroyed.");
  }

  const std::string& getName() const
  {
    return log_class_;
  }

private:
  std::string name_;

  std::string log_class_, log_subsys_, log_group_;

  temoto_core::temoto_id::ID id_ = temoto_core::temoto_id::UNASSIGNED_ID;

  std::unique_ptr<temoto_core::rmp::ResourceManager<OutputManagerInterface>> resource_manager_;

  std::vector<temoto_output_manager::LoadRvizPlugin> plugins_;

  ros::NodeHandle nh_;

  /**
   * @brief validateInterface()
   * @param sensor_type
   */
  void validateInterface(std::string& log_prefix)
  {
    if (!resource_manager_)
    {
      throw CREATE_ERROR(temoto_core::error::Code::UNINITIALIZED, "Interface is not initalized.");
    }
  }
};

} // namespace
