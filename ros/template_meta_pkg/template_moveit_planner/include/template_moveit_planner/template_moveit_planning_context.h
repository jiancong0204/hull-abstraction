#ifndef TEMPLATE_MOVEIT_PLANNING_INTERFACE_H_
#define TEMPLATE_MOVEIT_PLANNING_INTERFACE_H_

#include <moveit/planning_interface/planning_interface.h>
#include <template_moveit_planner/template_moveit_planner_interface.h>
//#include <moveit/planning_scene/planning_scene.h>


namespace template_moveit_interface
{
  MOVEIT_CLASS_FORWARD(TEMPLATE_MOVEITPlanningContext); //Defines Ptr

  class TEMPLATE_MOVEITPlanningContext : public planning_interface::PlanningContext
  {
  public:

    TEMPLATE_MOVEITPlanningContext(const std::string& name, const std::string& group, const robot_model::RobotModelConstPtr& model);

    ~TEMPLATE_MOVEITPlanningContext() override;

    void initialize();

    bool solve(planning_interface::MotionPlanResponse& res) override;
    bool solve(planning_interface::MotionPlanDetailedResponse& res) override;

    void clear() override;
    bool terminate() override;


  private:
    TEMPLATE_MOVEITInterfacePtr template_moveit_interface_;

    moveit::core::RobotModelConstPtr robot_model_;
  };

} // namespace template_moveit_interface


#endif /* TEMPLATE_MOVEIT_PLANNING_INTERFACE_H_ */
