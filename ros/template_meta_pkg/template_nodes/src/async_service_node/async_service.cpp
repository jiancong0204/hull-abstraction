#include "async_service.h"


namespace template_nodes
{
    AsyncService::AsyncService()
        : NodeBase(10.0), callback_queue_(), spinner_(2, &callback_queue_)
    {
        // Set up service server
        ros::AdvertiseServiceOptions opts;
        opts.init<service_t>("do", boost::bind(&AsyncService::serviceCallback, this, _1, _2));
        opts.callback_queue = &callback_queue_;

        service_server_ = pnh_.advertiseService(opts);
    }


    void AsyncService::run()
    {
        spinner_.start();

        while (ros::ok())
        {
            ROS_INFO("Loop");

            // No spin is needed here for an async spinner
            keepLoopRate();
        }
    }


    bool AsyncService::serviceCallback(service_t::Request & req, service_t::Response & res)
    {
        // This service simply sleeps away 5 seconds
        ROS_WARN("Service [start]");
        ros::Duration(5.0).sleep();
        ROS_WARN("Service [end]");
        
        return true;
    }
} // namespace template_nodes
