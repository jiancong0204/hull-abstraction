#ifndef TEMPLATE_MOVEIT_PLANNER_H_
#define TEMPLATE_MOVEIT_PLANNER_H_

#include "template_moveit_parameters.h"
#include "template_moveit_datatypes.h"

namespace template_moveit_library
{

  class TEMPLATE_MOVEITPlannerBase : TEMPLATE_MOVEITBase
  {
    public:
    TEMPLATE_MOVEITPlannerBase(){}
      virtual bool solve(TEMPLATE_MOVEITParameterBasePtr& params, TEMPLATE_MOVEITPlanRequestPtr template_moveit_req_ptr,  TEMPLATE_MOVEITPlanRespondPtr template_moveit_res_ptr) = 0;
    protected:
      const std::string template_moveit_colored_string_start_ = "\033[31;42m"; // "\033[40;35m";
      const std::string template_moveit_colored_string_end_ = "\033[0m\n";
  };
  TEMPLATE_MOVEIT_DECLARE_PTR(TEMPLATE_MOVEITPlannerBase,TEMPLATE_MOVEITPlannerBase);

  class TEMPLATE_MOVEITPlannerInterp : public TEMPLATE_MOVEITPlannerBase
  {
    public:
      TEMPLATE_MOVEITPlannerInterp();
      bool solve(TEMPLATE_MOVEITParameterBasePtr& params, TEMPLATE_MOVEITPlanRequestPtr template_moveit_req_ptr,  TEMPLATE_MOVEITPlanRespondPtr template_moveit_res_ptr);
    private:
      TEMPLATE_MOVEITParameterInterpPtr params_;
  };
  TEMPLATE_MOVEIT_DECLARE_PTR(TEMPLATE_MOVEITPlannerInterp, TEMPLATE_MOVEITPlannerInterp);


} // namespace template_moveit_library


#endif // TEMPLATE_MOVEIT_PLANNER_H_
