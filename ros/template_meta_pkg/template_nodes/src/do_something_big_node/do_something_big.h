/**
 * @file
 * @brief This file contains the declaration of the DoSomethingBig node class
 *
 * @ros_node This node does something big.
 *
 * This file contains the declaration of the DoSomethingBig node class.
 *
 * @author Henrik Stickann
 */

#ifndef TEMPLATE_NODES_DO_SOMETHING_BIG_NODE_DO_SOMETHING_BIG_H
#define TEMPLATE_NODES_DO_SOMETHING_BIG_NODE_DO_SOMETHING_BIG_H

// IGMR package headers
#include <igmr_node_interface/node_base.h>
#include <template_nodes/DoSomethingConfig.h>
#include <template_msgs/DoSomething.h>
#include <template_srvs/DoSomething.h>
#include <template_actions/DoSomethingAction.h>

// ROS headers
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <actionlib/server/simple_action_server.h>

// Standard library headers
#include <string>


namespace template_nodes
{
    /**
     * @brief A simple example C++ node class
     *
     * This class description is free to copy for everyone who is writing
     * breathtaking good source code for the COAR of the IGMR.
     */
    class DoSomethingBig : public igmr_node_interface::NodeBase
    {
    public:
        /**
         * @brief Constructor
         */
        DoSomethingBig();
        virtual ~DoSomethingBig() = default;

        /**
         * @brief Execute node logic
         */
        virtual void run() override;


    private:
        using reconf_config_t = template_nodes::DoSomethingConfig;            /**< Dynamic Reconfigure config type used by this node */
        using reconf_server_t = dynamic_reconfigure::Server<reconf_config_t>; /**< Dynamic Reconfigure server type used by this node */

        using action_t        = template_actions::DoSomethingAction;          /**< Action type used by this node */
        using action_server_t = actionlib::SimpleActionServer<action_t>;      /**< Action server type used by this node */

    private:
        /**
         * @brief Dynamic Reconfigure Callback function to change member objects during runtime
         * @param config new configuration
         * @param level result of ORing
         */
        void dynReconfCallback(reconf_config_t & config, uint32_t level);

        /**
         * @brief Do some calculations
         */
        void doCalculations();

        /**
         * @brief Publishes a message to the advertised topic
         */
        void publish();

        /**
         * @brief Callback for the subscriber
         * @param msg message received on the subscribed topic
         */
        void subCallback(const template_msgs::DoSomething::ConstPtr & msg);

        /**
         * @brief Callback for the service provided by this node
         * @param req request message for the service
         * @param res result message returned by the service
         */
        bool serviceCallback(template_srvs::DoSomething::Request & req, template_srvs::DoSomething::Response & res);

        /**
         * @brief Callback for the action provided by this node
         * @param goal goal of the action
         */
        void actionExecuteCallback(const action_server_t::GoalConstPtr & goal);

    private:
        reconf_server_t     reconf_server_;  /**< Dynamic reconfigure server */
        action_server_t     action_server_;  /**< Action server */
        ros::Publisher      pub_;            /**< Publisher */
        ros::Subscriber     sub_;            /**< Subscriber */
        ros::ServiceServer  service_server_; /**< Service server */

        std::string text_;   /**< Some text variable */
        int         result_; /**< Some results */
    };
} // namespace template_nodes

#endif // TEMPLATE_NODES_DO_SOMETHING_BIG_NODE_DO_SOMETHING_BIG_H
