#ifndef TEMPLATE_MOVEIT_DATATYPES_H
#define TEMPLATE_MOVEIT_DATATYPES_H

#include <iostream>
#include <memory>
#include <vector>
#include <sstream>
namespace template_moveit_library {

/// Create shared Ptr, ConstPtr, WeakPtr and ConstWeakPtr for each TEMPLATE_MOVEIT library class
#define TEMPLATE_MOVEIT_DECLARE_PTR(Name, Type)                                                                                   \
  typedef std::shared_ptr<Type> Name##Ptr;                                                                             \
  typedef std::shared_ptr<const Type> Name##ConstPtr;                                                                  \

////////////// Template_moveit Base Class ////////////////////////
/// \brief The TEMPLATE_MOVEITBase class
class TEMPLATE_MOVEITBase
{
  public:
  virtual ~TEMPLATE_MOVEITBase(){}

};
TEMPLATE_MOVEIT_DECLARE_PTR(TEMPLATE_MOVEITBase, TEMPLATE_MOVEITBase)


//////////////// TEMPLATE_MOVEITJointTrajectoryPoint ////////////////////////
/// \brief The TEMPLATE_MOVEITJointTrajectoryPoint class
/// One Trajectory Point with positions, velocities, accs and effs FOR EACH joint
class TEMPLATE_MOVEITJointTrajectoryPoint : public TEMPLATE_MOVEITBase
{
  public:

  /// \brief add_joint_state Adds the state of a joint to this particular joint trajectory point
  bool add_joint_state(double pos, double vel, double acc, double eff)
  {
    positions_.push_back(pos);
    velocities_.push_back(vel);
    accelerations_.push_back(acc);
    effort_.push_back(eff);
  }

  std::vector<double> positions_;
  std::vector<double> velocities_;
  std::vector<double> accelerations_;
  std::vector<double> effort_;

  double time_from_start_; // [s] seconds
};
TEMPLATE_MOVEIT_DECLARE_PTR(TEMPLATE_MOVEITJointTrajectoryPoint,TEMPLATE_MOVEITJointTrajectoryPoint)


//////////////// TEMPLATE_MOVEITJointTrajectory ////////////////////////
typedef  std::vector<TEMPLATE_MOVEITJointTrajectoryPoint> TEMPLATE_MOVEITJointTrajectory;
TEMPLATE_MOVEIT_DECLARE_PTR(TEMPLATE_MOVEITJointTrajectory, TEMPLATE_MOVEITJointTrajectory)



////////////// Template_moveit Plan request ////////////////////////
class TEMPLATE_MOVEITPlanRequest : public TEMPLATE_MOVEITBase
{
  public:

  std::vector<std::string> joint_names_;
  std::vector<double> joint_initial_position_;
  std::vector<double> joint_initial_velocity_;

  std::vector<double> joint_final_position_;
  std::vector<double> joint_final_velocity_;



};
TEMPLATE_MOVEIT_DECLARE_PTR(TEMPLATE_MOVEITPlanRequest, TEMPLATE_MOVEITPlanRequest)



////////////// Template_moveit Plan respond ////////////////////////
class TEMPLATE_MOVEITPlanRespond : public TEMPLATE_MOVEITBase
{
  public:

  std::vector<std::string> joint_names_;
  TEMPLATE_MOVEITJointTrajectoryPtr joint_trajectory_;

};
TEMPLATE_MOVEIT_DECLARE_PTR(TEMPLATE_MOVEITPlanRespond, TEMPLATE_MOVEITPlanRespond)




}
#endif // TEMPLATE_MOVEIT_DATATYPES_H
