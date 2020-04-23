/**
 * @file
 * @brief This file contains the declaration of the ExampleServer class
 *
 * This file contains the declaration of the ExampleServer class
 *
 * @author Johanna
 */

#ifndef TEMPLATE_NODES_THREADING_NODE_SERVER_EXAMPLESERVER_H
#define TEMPLATE_NODES_THREADING_NODE_SERVER_EXAMPLESERVER_H

// IGMR package headers
#include <template_srvs/Addition.h>

// ROS headers
#include <ros/ros.h>
#include <ros/callback_queue.h>


namespace template_nodes
{
    /**
     * @brief This class shows a simple service server, which uses service "thread_srv"
     */
    class ExampleServer
    {
    public:
        /**
         * @brief Contructor
         */
        ExampleServer();

        /**
         * @brief Copy constructor
         * @param es ExampleServer which is copied
         */
        ExampleServer(const ExampleServer & es);

        /**
         * @brief Starts the ExampleServer by advertising service to "threaded_example_srv", using callbackFunction
         */
        void startServer();

    private:
        /**
         * @brief Callback function of server, adds the two integers send as request and responses the answer
         * @param req request data
         * @param res response data
         * @return true, in case the service was successfull
         */
        bool serviceCallback(template_srvs::Addition::Request & req, template_srvs::Addition::Response & res);

    private:
        ros::NodeHandle     nh_;              /**< node handle of server */
        ros::ServiceServer  service_server_;  /**< server of topic "threaded_example_srv" */
    };
} // namespace template_nodes


#endif // TEMPLATE_NODES_THREADING_NODE_SERVER_EXAMPLESERVER_H
