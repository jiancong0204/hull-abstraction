/**
 * @file
 * @brief This file contains the declaration of the DoSomething node class
 *
 * @ros_node This node does something.
 *
 * This file contains the declaration of the DoSomething node class
 *
 * @author Henrik Stickann
 */

#ifndef TEMPLATE_NODES_DO_SOMETHING_NODE_DO_SOMETHING_H
#define TEMPLATE_NODES_DO_SOMETHING_NODE_DO_SOMETHING_H

// IGMR package headers
#include <igmr_node_interface/node_base.h>
#include <template_nodes/DoSomethingConfig.h>

// ROS headers
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

// Standard library headers
#include <string>


namespace template_nodes
{
    /**
     * @brief A simple example C++ node class
     *
     * This class description is free to copy for everyone who is writing
     * breathtaking good source code for the COAR of the IGMR
     */
    class DoSomething : public igmr_node_interface::NodeBase
    {
    public:
        /**
         * @brief Constructor
         */
        DoSomething();
        virtual ~DoSomething() = default;

        /**
         * @brief Execute node logic
         */
        virtual void run() override;

    private:
        using reconf_config_t = template_nodes::DoSomethingConfig;              /**< Dynamic Reconfigure config type used by this node */
        using reconf_server_t = dynamic_reconfigure::Server<reconf_config_t>;   /**< Dynamic Reconfigure server type used by this node */

        /**
         * @brief Dynamic Reconfigure Callback function to change member objects during runtime
         * @param config new configuration
         * @param level result of ORing
         */
        void dynReconfCallback(reconf_config_t & config, uint32_t level);

    private:
        reconf_server_t reconf_server_; /**< Dynamic Reconfigure server */
        std::string     text_;          /**< Some text variable */
    };
} // namespace template_nodes

#endif // TEMPLATE_NODES_DO_SOMETHING_NODE_DO_SOMETHING_H
