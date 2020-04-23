#include <template_moveit_planner/template_moveit_planner_interface.h>

namespace template_moveit_interface
{
  TEMPLATE_MOVEITInterface::TEMPLATE_MOVEITInterface(const ros::NodeHandle& nh) : nh_(nh)
  {
    loadParams();
  }

  void TEMPLATE_MOVEITInterface::loadParams()
  {
    std::cout << this->template_moveit_colored_string_start_ << "This is TEMPLATE_MOVEIT - TEMPLATE_MOVEITInterface - Load Parameters \n Load from namespace " << nh_.getNamespace() << std::endl;

    template_moveit_library::TEMPLATE_MOVEITParameterInterpPtr interp_param_ptr = template_moveit_library::TEMPLATE_MOVEITParameterInterpPtr(new template_moveit_library::TEMPLATE_MOVEITParameterInterp());

    ReadParameter("planner_configs/TEMPLATE_MOVEIT_Interp/N", &interp_param_ptr->N);
    ReadParameter("planner_configs/TEMPLATE_MOVEIT_Interp/trajectory_execution_time", &interp_param_ptr->trajectory_execution_time_);

    params_.insert({"TEMPLATE_MOVEIT_Interp", interp_param_ptr});

  }

  bool TEMPLATE_MOVEITInterface::solve(const planning_scene::PlanningSceneConstPtr& planning_scene, const moveit_msgs::MotionPlanRequest& req,
             moveit_msgs::MotionPlanDetailedResponse& res)
  {

    // Print request message
    // std::cout << req << "\n";

    // Convert Moveit Datatypes to TEMPLATE_MOVEIT Datatypes
    // Create and convert TEMPLATE_MOVEIT Plan Request
    template_moveit_library::TEMPLATE_MOVEITPlanRequestPtr template_moveit_req_ptr = template_moveit_library::TEMPLATE_MOVEITPlanRequestPtr(new template_moveit_library::TEMPLATE_MOVEITPlanRequest());
    template_moveit_req_ptr->joint_names_            = req.start_state.joint_state.name;
    template_moveit_req_ptr->joint_initial_position_ = req.start_state.joint_state.position;
    template_moveit_req_ptr->joint_initial_velocity_ = req.start_state.joint_state.velocity;
    while(template_moveit_req_ptr->joint_initial_position_.size() > template_moveit_req_ptr->joint_initial_velocity_.size())
    {
      template_moveit_req_ptr->joint_initial_velocity_.push_back(0);
    }

    if( req.goal_constraints.size() < 1)
    {
      ROS_ERROR("GOAL constraints size smaller than 1");
      return false;
    }
    template_moveit_req_ptr->joint_final_position_.clear();
    for( auto j_cons : req.goal_constraints[0].joint_constraints)
    {
      template_moveit_req_ptr->joint_final_position_.push_back(j_cons.position);
      template_moveit_req_ptr->joint_final_velocity_.push_back(0); //TODO Where is the velocity in goal state defined?
    }

    // Create the solver object
    template_moveit_library::TEMPLATE_MOVEITPlannerBasePtr solver;
    if(req.planner_id == "TEMPLATE_MOVEIT_Interp")
      solver = template_moveit_library::TEMPLATE_MOVEITPlannerInterpPtr(new template_moveit_library::TEMPLATE_MOVEITPlannerInterp());

    // Create new TEMPLATE_MOVEIT Plan Response
    template_moveit_library::TEMPLATE_MOVEITPlanRespondPtr template_moveit_res_ptr = template_moveit_library::TEMPLATE_MOVEITPlanRespondPtr(new template_moveit_library::TEMPLATE_MOVEITPlanRespond());

    //////////////////////////
    ///////// SOLVE //////////
    ros::Time begin = ros::Time::now();
    bool success = solver->solve(params_.at(req.planner_id), template_moveit_req_ptr, template_moveit_res_ptr);
    ros::Time end = ros::Time::now();
    res.processing_time.push_back((end - begin).toSec());
    //////////////////////////

    //Convert from TEMPLATE_MOVEIT Response to Moveit Response
    res.trajectory_start = req.start_state; //TODO necesarry?
    res.group_name = req.group_name;
    if(success)
      res.error_code.val = res.error_code.SUCCESS;
    else
      res.error_code.val = res.error_code.FAILURE;
    res.description.push_back(this->template_moveit_colored_string_start_ + " TEMPLATE_MOVEIT TRAJECTORY DESCRIPTION " + this->template_moveit_colored_string_end_ + "\n");

    moveit_msgs::RobotTrajectory rt;  // This is a single trajectory for all joints
    rt.joint_trajectory.joint_names = template_moveit_res_ptr->joint_names_;

    // Iterate through each point in the trajectory and copy the attributes
    for (template_moveit_library::TEMPLATE_MOVEITJointTrajectoryPoint traj_point : *template_moveit_res_ptr->joint_trajectory_)
    {
      trajectory_msgs::JointTrajectoryPoint jtp;
      jtp.positions = traj_point.positions_;
      jtp.velocities = traj_point.velocities_;
      jtp.accelerations = traj_point.accelerations_;
      jtp.effort = traj_point.effort_;
      jtp.time_from_start = ros::Duration(traj_point.time_from_start_); // Seconds
      rt.joint_trajectory.points.push_back(jtp);
    }

    res.trajectory.push_back(rt);

    return true;
  }

  template <typename T> void TEMPLATE_MOVEITInterface::ReadParameter(std::string parameter_name, T* member)
  {
    // Load parameters from parameter server
    if (nh_.getParam(parameter_name, *member))
      ROS_INFO("Successfully loaded parameter: %s", parameter_name.c_str());
    else
    {
      std::string error_msg =  "No parameter " + parameter_name + " specified with namespace" + nh_.getNamespace();
      ROS_WARN_STREAM(error_msg);
      throw error_msg;
    }
  }
}
