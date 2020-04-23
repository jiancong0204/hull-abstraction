#include "do_something.h"


namespace template_nodes
{
    DoSomething::DoSomething()
        : NodeBase(10.0)
    {
        // Load parameters from parameter server
        // dynamic_reconfigure parameters will be set automatically
        getPrivateParam("loop_rate", &loop_rate_, 10.0);

        // Set up dynamic reconfigure
        reconf_server_t::CallbackType dyn_reconf_callback;
        dyn_reconf_callback = boost::bind(&DoSomething::dynReconfCallback, this, _1, _2);

        reconf_server_.setCallback(dyn_reconf_callback);
    }


    void DoSomething::run()
    {
        while (ros::ok())
        {
            ROS_INFO_STREAM("Current text: " << text_);

            ros::spinOnce();
            keepLoopRate();
        }
    }

    void DoSomething::dynReconfCallback(reconf_config_t & config, uint32_t level)
    {
        /** @bug Do not use config.group.publisher.TEXT! Refer to members directly, otherwise they dont get a default
         *  value! See: https://answers.ros.org/question/213305/dynamic_reconfigure-group-defaults/
         */
        ROS_INFO_STREAM("Reconfigure Request: " << config.text);
        text_ = config.text;
    }
} // namespace template_nodes
