#include "exampleserver.h"

#include <boost/thread.hpp>


namespace template_nodes
{
    ExampleServer::ExampleServer()
        : nh_()
    {}

    ExampleServer::ExampleServer(const ExampleServer & es)
    {
        nh_ = es.nh_;
    }

    bool ExampleServer::serviceCallback(template_srvs::Addition::Request & req, template_srvs::Addition::Response & res)
    {
        ROS_INFO("Server got rerquest but waits for one second before sending response.");
        ros::Duration(1).sleep();

        res.sum = req.a + req.b;

        return true;
    }

    void ExampleServer::startServer()
    {
        ros::Rate loop_rate(5.0);

        service_server_ = nh_.advertiseService("add", &ExampleServer::serviceCallback, this);

        ROS_INFO("Service server was initialized");

        while (nh_.ok())
        {
            ros::spinOnce();
            loop_rate.sleep();

            // This line allows a call to boost::thread.interrupt() to stop this thread
            boost::this_thread::interruption_point();
        }
    }
} // namespace template_nodes
