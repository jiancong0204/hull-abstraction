#ifndef TEMPLATE_MOVEIT_PLANNER_MANAGER_H_
#define TEMPLATE_MOVEIT_PLANNER_MANAGER_H_

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>

#include <template_moveit_planner/template_moveit_planning_context.h>

namespace template_moveit_interface
{
  class TEMPLATE_MOVEITPlannerManager : public planning_interface::PlannerManager
  {
    public:
      bool initialize(const robot_model::RobotModelConstPtr& model, const std::string& ns) override;
      std::string getDescription() const override;
      void getPlanningAlgorithms(std::vector<std::string>& algs) const override;

      planning_interface::PlanningContextPtr getPlanningContext(
                                                    const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                    const planning_interface::MotionPlanRequest& req,
                                                    moveit_msgs::MoveItErrorCodes& error_code) const override;

      bool canServiceRequest(const planning_interface::MotionPlanRequest& req) const override;

      void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& pcs) override;

  private:

      std::map<std::string, TEMPLATE_MOVEITPlanningContextPtr> planning_contexts_;

  };

}

#endif // TEMPLATE_MOVEIT_PLANNER_MANAGER_H_
