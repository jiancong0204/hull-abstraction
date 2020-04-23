#include "do_something_big.h"


namespace template_nodes
{
    void DoSomethingBig::dynReconfCallback(reconf_config_t & config, uint32_t level)
    {
        /** @bug Do not use config.group.publisher.TEXT! Refer to members directly, otherwise they dont get a default
         *  value! See: https://answers.ros.org/question/213305/dynamic_reconfigure-group-defaults/
         */
        ROS_INFO_STREAM("Reconfigure Request: " << config.text);
        text_ = config.text;
    }
} // namespace template_nodes
