#ifndef TEMPLATE_MOVEIT_PARAMETERS_H
#define TEMPLATE_MOVEIT_PARAMETERS_H

#include "template_moveit_datatypes.h"

namespace template_moveit_library{

  class TEMPLATE_MOVEITParameterBase : public TEMPLATE_MOVEITBase
  {
  public:
    TEMPLATE_MOVEITParameterBase(){}
    virtual std::ostringstream print() const = 0;

  };
  TEMPLATE_MOVEIT_DECLARE_PTR(TEMPLATE_MOVEITParameterBase, TEMPLATE_MOVEITParameterBase)



  class TEMPLATE_MOVEITParameterInterp : public TEMPLATE_MOVEITParameterBase
  {
  public:
    TEMPLATE_MOVEITParameterInterp(){}
    std::ostringstream print() const
    {
      std::cout << "Parameters TEMPLATE_MOVEITParameterInterp:" << std::endl;
    }

    int N;                             /**< Number of interpolation points **/
    double trajectory_execution_time_; /**< Time duration of the trajectory execution */

  };
  TEMPLATE_MOVEIT_DECLARE_PTR(TEMPLATE_MOVEITParameterInterp, TEMPLATE_MOVEITParameterInterp)

}

#endif // TEMPLATE_MOVEIT_PARAMETERS_H
