#include "do_something_big.h"


namespace template_nodes
{
    DoSomethingBig::DoSomethingBig()
        : NodeBase(10.0), action_server_(pnh_, "do_something_action", boost::bind(&DoSomethingBig::actionExecuteCallback, this, _1), false)
    {
        // Load parameters from parameter server
        // dynamic_reconfigure parameters will be set automatically
        getPrivateParam("loop_rate", &loop_rate_, 10.0);

        // Set up dynamic reconfigure
        reconf_server_t::CallbackType dyn_reconf_callback;
        dyn_reconf_callback = boost::bind(&DoSomethingBig::dynReconfCallback, this, _1, _2);

        reconf_server_.setCallback(dyn_reconf_callback);

        // Start action server
        action_server_.start();

        // Set up publishers (latched)
        pub_ = pnh_.advertise<template_msgs::DoSomething>("pub_topic", 10, true);

        // Set up subscribers
        sub_ = pnh_.subscribe("sub_topic", 100, &DoSomethingBig::subCallback, this);

        // Set up services
        service_server_ = pnh_.advertiseService("do_something_service", &DoSomethingBig::serviceCallback, this);
    }


    void DoSomethingBig::run()
    {
        while (ros::ok())
        {
            publish();

            ros::spinOnce();
            keepLoopRate();
        }
    }


    void DoSomethingBig::doCalculations()
    {
        result_ = 1 + 1;
    }
} // namespace template_nodes
