#include <ros/ros.h>
#include <pluginlib/class_list_macros.hpp>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/collision_distance_field/collision_detector_allocator_hybrid.h>


#include <template_moveit_planner/template_moveit_planner_manager.h>



namespace template_moveit_interface
{

    bool TEMPLATE_MOVEITPlannerManager::initialize(const robot_model::RobotModelConstPtr& model, const std::string& ns)
    {
      // Create a context for each planning group ( same as joint model group )
      for (const std::string& group : model->getJointModelGroupNames())
      {
        this->planning_contexts_[group] =
            TEMPLATE_MOVEITPlanningContextPtr(new TEMPLATE_MOVEITPlanningContext("template_moveit_planning_context", group, model));
      }
      return true;
    }

    std::string TEMPLATE_MOVEITPlannerManager::getDescription() const
    {
      return std::string("TEMPLATE_MOVEIT Planner - The Optimal Control Trajectory Optimization Planner");
    }

    void TEMPLATE_MOVEITPlannerManager::getPlanningAlgorithms(std::vector<std::string>& algs) const
    {
      algs.push_back( "TEMPLATE_MOVEIT_Interp");
    }

    planning_interface::PlanningContextPtr TEMPLATE_MOVEITPlannerManager::getPlanningContext(
                                                  const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                  const planning_interface::MotionPlanRequest& req,
                                                  moveit_msgs::MoveItErrorCodes& error_code) const
    {
      // If everything is okay, this is the returned message
      error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

      // Throw error if the request group name is not specified
      if (req.group_name.empty())
      {
        ROS_ERROR("No group specified to plan for");
        error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
        return planning_interface::PlanningContextPtr();
      }

      // Throw error if planning scene is not supplied
      if (!planning_scene)
      {
        ROS_ERROR("No planning scene supplied as input");
        error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
        return planning_interface::PlanningContextPtr();
      }


      // create PlanningScene using hybrid collision detector
      planning_scene::PlanningScenePtr ps = planning_scene->diff(); // Make a copy
      ps->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorHybrid::create(), true);

      // retrieve and configure existing (group-specific) context
      const TEMPLATE_MOVEITPlanningContextPtr& context = planning_contexts_.at(req.group_name);
      context->setPlanningScene(ps);
      context->setMotionPlanRequest(req);
      error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
      return context;

    }

    bool TEMPLATE_MOVEITPlannerManager::canServiceRequest(const planning_interface::MotionPlanRequest& req) const
    {
         return true;
    }


    void TEMPLATE_MOVEITPlannerManager::setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& pcs)
    {
      this->config_settings_ = pcs;
      std::cout << "PlannerConfigurationMap:" << std::endl;
      for (planning_interface::PlannerConfigurationMap::const_iterator itr = pcs.begin(); itr != pcs.end(); ++itr)
      {
        std::cout << "PlannerConfigurationSettings.name= " << itr->first
                  << "setting.name" << itr->second.name   << "\n"
                  << "setting.group" << itr->second.group << "\n";
        for (std::map<std::string, std::string>::const_iterator jtr = itr->second.config.begin(); jtr != itr->second.config.end(); ++jtr)
        {
          std::cout << '\t' << jtr->first << '\t' << jtr->second << '\n';
        }
      }
      std::cout  << std::endl;
    }

}

PLUGINLIB_EXPORT_CLASS(template_moveit_interface::TEMPLATE_MOVEITPlannerManager, planning_interface::PlannerManager);
