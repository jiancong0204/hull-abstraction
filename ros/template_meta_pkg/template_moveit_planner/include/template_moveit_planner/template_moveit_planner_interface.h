#ifndef TEMPLATE_MOVEIT_PLANNER_INTERFACE_H
#define TEMPLATE_MOVEIT_PLANNER_INTERFACE_H

#include <ros/ros.h>
#include <moveit/macros/class_forward.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/MoveItErrorCodes.h>

#include <template_moveit_planner/template_moveit_library/template_moveit_planner.h>
#include <template_moveit_planner/template_moveit_library/template_moveit_parameters.h>
#include <template_moveit_planner/template_moveit_library/template_moveit_datatypes.h>

namespace template_moveit_interface
{
  MOVEIT_CLASS_FORWARD(TEMPLATE_MOVEITInterface); // define pointer

  class TEMPLATE_MOVEITInterface
  {
  public:
    TEMPLATE_MOVEITInterface(const ros::NodeHandle& nh = ros::NodeHandle("~"));

    // Converts moveit objects to TEMPLATE_MOVEITlib entities and calls TEMPLATE_MOVEITlib solver
    bool solve(const planning_scene::PlanningSceneConstPtr& planning_scene, const moveit_msgs::MotionPlanRequest& req,
               moveit_msgs::MotionPlanDetailedResponse& res);


  protected:
    /** @brief Configure everything using the param server */
    void loadParams();

    ros::NodeHandle nh_;  /// The ROS node handle

    std::map<std::string, template_moveit_library::TEMPLATE_MOVEITParameterBasePtr> params_;

    private:
    template <typename T> void ReadParameter(std::string parameter_name, T* member);
    const std::string template_moveit_colored_string_start_ = "\033[34;41m"; // "\033[40;35m";
    const std::string template_moveit_colored_string_end_ = "\033[0m\n";

  };

} // namespace template_moveit_interface

#endif // TEMPLATE_MOVEIT_PLANNER_INTERFACE_H
