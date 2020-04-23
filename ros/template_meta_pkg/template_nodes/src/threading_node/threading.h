/**
 * @file
 * @brief This file contains the declaration of the Threading node class
 *
 * @ros_node This node uses multithreading.
 *
 * This file contains the declaration of the Threading node class
 *
 * @author Johanna
 */

#ifndef TEMPLATE_NODES_THREADING_NODE_THREADING_H
#define TEMPLATE_NODES_THREADING_NODE_THREADING_H

// IGMR package headers
#include <igmr_node_interface/node_base.h>
#include <template_srvs/Addition.h>
#include <template_nodes/threading_library/waitthread.h>

#include "server/exampleserver.h"

// ROS headers
#include <ros/ros.h>

// Boost headers
#include <boost/thread.hpp>


namespace template_nodes
{
    /**
     * @brief An object of this class is to be contructed in main-function. The class contains everything, to make the node work.
     * It runs three threads. The main thread, on which a service server is running, one side thread with a client to the server and another side thread with a blocking class
     */
    class Threading : public igmr_node_interface::NodeBase
    {
    public:
        /**
         * @brief Constructor
         */
        Threading();
        virtual ~Threading() = default;

        /**
         * @brief run
         */
        virtual void run() override;

    private:
        /**
         * @brief Waits for the server to be connected and sends a request using service "thread_srv"
         */
        void sendRequest();

    private:
        ros::ServiceClient  service_client_;  /**< client to topic "add" */

        WaitThread    waiter_;  /**< WaitThread, which will be started in a seperate thread */
        ExampleServer server_;  /**< Server will be started in a seperate thread */

        boost::thread thread_waiter_; /**< thread of waiter */
        boost::thread thread_server_; /**< thread of server */
    };
} // namespace template_nodes


#endif // TEMPLATE_NODES_THREADING_NODE_THREADING_H
