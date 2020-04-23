#include <template_moveit_planner/template_moveit_library/template_moveit_planner.h>

namespace template_moveit_library
{

  TEMPLATE_MOVEITPlannerInterp::TEMPLATE_MOVEITPlannerInterp()
  {
     std::cout << this->template_moveit_colored_string_start_ << "This is TEMPLATE_MOVEIT - TEMPLATE_MOVEITPlannerInterp Constructor "  << this->template_moveit_colored_string_end_<< std::endl;
  }

  bool TEMPLATE_MOVEITPlannerInterp::solve(TEMPLATE_MOVEITParameterBasePtr& params, TEMPLATE_MOVEITPlanRequestPtr template_moveit_req_ptr,  TEMPLATE_MOVEITPlanRespondPtr template_moveit_res_ptr)
  {
    std::cout << this->template_moveit_colored_string_start_ << "This is TEMPLATE_MOVEIT - TEMPLATE_MOVEITPlannerInterp solve " << std::endl;

    // Typecast the parameters to PlannerID
    this->params_ = std::dynamic_pointer_cast<TEMPLATE_MOVEITParameterInterp>(params);

    template_moveit_res_ptr->joint_names_ = template_moveit_req_ptr->joint_names_;

    // Create a new empty trajectory
    TEMPLATE_MOVEITJointTrajectoryPtr joint_traj = TEMPLATE_MOVEITJointTrajectoryPtr(new TEMPLATE_MOVEITJointTrajectory());

    // Iterate through each trajectory point | (N+1) points!!!
    for (int n=0; n < this->params_->N+1; n++)
    {
      // Create new trajectory point
      TEMPLATE_MOVEITJointTrajectoryPointPtr joint_traj_point = TEMPLATE_MOVEITJointTrajectoryPointPtr(new TEMPLATE_MOVEITJointTrajectoryPoint());

      // Fill the trajectory point with a state for each joint
      // Using simple interpolation between start and end point
      for( std::vector<std::string>::size_type joint_i = 0; joint_i < template_moveit_req_ptr->joint_names_.size() ; joint_i ++)
      {
        double pos = static_cast<double>(n) / static_cast<double>(this->params_->N) * template_moveit_req_ptr->joint_final_position_[joint_i] +
           static_cast<double>(this->params_->N-n) / static_cast<double>(this->params_->N) * template_moveit_req_ptr->joint_initial_position_[joint_i];

        double vel = static_cast<double>(n) / static_cast<double>(this->params_->N) * template_moveit_req_ptr->joint_final_velocity_[joint_i] +
           static_cast<double>(this->params_->N-n) / static_cast<double>(this->params_->N) * template_moveit_req_ptr->joint_initial_velocity_[joint_i];

        joint_traj_point->add_joint_state(pos, vel, 0, 0);
      }

      // Add the time from start
      joint_traj_point->time_from_start_ = static_cast<double>(n) / static_cast<double>(this->params_->N) * this->params_->trajectory_execution_time_; //sec

      // Add point to trajectory
      joint_traj->push_back(*joint_traj_point);
    }

    template_moveit_res_ptr->joint_trajectory_ = joint_traj;

    std::cout << "Successfully solved" << this->template_moveit_colored_string_end_ << std::endl;
    return true;
  }



}


