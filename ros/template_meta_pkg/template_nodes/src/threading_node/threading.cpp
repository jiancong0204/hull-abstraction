#include "threading.h"


namespace template_nodes
{
    Threading::Threading()
        : NodeBase()
    {
        // Start everything which is running on main thread
        service_client_ = nh_.serviceClient<template_srvs::Addition>("add");

        // Start threads
        thread_waiter_ = boost::thread(&WaitThread::startThread, waiter_);
        thread_server_ = boost::thread(&ExampleServer::startServer, server_);
    }


    void Threading::run()
    {
        // Make sure service is available
        service_client_.waitForExistence();

        sendRequest();

        // Tell server thread to exit
        thread_server_.interrupt();

        // Join threads before exiting
        thread_waiter_.join();
        thread_server_.join();
    }


    void Threading::sendRequest()
    {
        // Example request
        template_srvs::Addition::Request req;
        template_srvs::Addition::Response res;
        req.a = 3;
        req.b = 6;

        // Call service
        if (service_client_.call(req, res))
        {
            ROS_INFO_STREAM("The client got response: " << res.sum);
        }
        else
        {
            ROS_INFO("Service call failed");
        }
    }
} // namespace template_nodes
