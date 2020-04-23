/**
 * @file
 * @brief This file contains the declaration of the AsyncService node class
 *
 * @ros_node This node provides an asynchronous service.
 *
 * @author Henrik Stickann
 */

#ifndef TEMPLATE_NODES_ASYNC_SERVICE_NODE_ASYNC_SERVICE_H
#define TEMPLATE_NODES_ASYNC_SERVICE_NODE_ASYNC_SERVICE_H

// IGMR package headers
#include <igmr_node_interface/node_base.h>

// ROS headers
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_srvs/Empty.h>


namespace template_nodes
{
    /**
     * @brief A simple example for asynchronous callbacks
     *
     * This node uses an AsyncSpinner and it's own CallbackQueue
     * to allow parallel execution of the service callback and main loop.
     */
    class AsyncService : public igmr_node_interface::NodeBase
    {
    public:
        /**
         * @brief Constructor
         */
        AsyncService();
        virtual ~AsyncService() = default;

        /**
         * @brief Execute node logic
         */
        virtual void run() override;


    private:
        using service_t = std_srvs::Empty; /**< Service type used by this node */

    private:
        /**
         * @brief Service callback function to implement the service
         * @param req Request sent to the service
         * @param res Response sent by the service
         */
        bool serviceCallback(service_t::Request & req, service_t::Response & res);

    private:
        ros::CallbackQueue  callback_queue_; /**< Callback queue for service */
        ros::AsyncSpinner   spinner_;        /**< Asynchronous spinner */
        ros::ServiceServer  service_server_; /**< Service server */
    };
} // namespace template_nodes


#endif // TEMPLATE_NODES_ASYNC_SERVICE_NODE_ASYNC_SERVICE_H
