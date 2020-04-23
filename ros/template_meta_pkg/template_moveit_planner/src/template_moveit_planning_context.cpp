#include <template_moveit_planner/template_moveit_planning_context.h>

#include <moveit/robot_state/conversions.h>

namespace template_moveit_interface
{


  TEMPLATE_MOVEITPlanningContext::TEMPLATE_MOVEITPlanningContext(const std::string& name, const std::string& group, const robot_model::RobotModelConstPtr& model)
    : planning_interface::PlanningContext(name, group), robot_model_(model)
  {
    template_moveit_interface_ = TEMPLATE_MOVEITInterfacePtr(new TEMPLATE_MOVEITInterface());
  }

  TEMPLATE_MOVEITPlanningContext::~TEMPLATE_MOVEITPlanningContext() = default;

  bool TEMPLATE_MOVEITPlanningContext::solve(planning_interface::MotionPlanDetailedResponse& res)
  {
    moveit_msgs::MotionPlanDetailedResponse res_msg;
    if (template_moveit_interface_->solve(planning_scene_, request_, res_msg))
    {
      res.trajectory_.resize(1);
      res.trajectory_[0] =
          robot_trajectory::RobotTrajectoryPtr(new robot_trajectory::RobotTrajectory(robot_model_, getGroupName()));

      moveit::core::RobotState start_state(robot_model_);
      robot_state::robotStateMsgToRobotState(res_msg.trajectory_start, start_state);
      res.trajectory_[0]->setRobotTrajectoryMsg(start_state, res_msg.trajectory[0]);

      res.description_.push_back("plan");
      res.processing_time_ = res_msg.processing_time;
      res.error_code_ = res_msg.error_code;
      return true;
    }
    else
    {
      res.error_code_ = res_msg.error_code;
      return false;
    }
    return true;
  }

  //Converts Input from MotionPlanResponse to MotionPlanDetailedResponse
  bool TEMPLATE_MOVEITPlanningContext::solve(planning_interface::MotionPlanResponse& res)
  {
    planning_interface::MotionPlanDetailedResponse res_detailed;
    bool planning_success = solve(res_detailed);

    res.error_code_ = res_detailed.error_code_;

    if (planning_success)
    {
      res.trajectory_ = res_detailed.trajectory_[0];
      res.planning_time_ = res_detailed.processing_time_[0];
    }

    return planning_success;
  }

  bool TEMPLATE_MOVEITPlanningContext::terminate()
  {
    // TODO - make interruptible
    return true;
  }

  void TEMPLATE_MOVEITPlanningContext::clear()
  {

  }


}
