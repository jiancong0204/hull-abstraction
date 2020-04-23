/** 
 * @file
 * @brief This file contains the declaration of the DoSomethingAction node class.
 *
 * @ros_node This node provides some action.
 *
 * This file contains the declaration of the DoSomethingAction node class.
 *
 * @author Henrik Stickann
 */

#ifndef TEMPLATE_NODES_DO_SOMETHING_ACTION_NODE_DO_SOMETHING_ACTION_H
#define TEMPLATE_NODES_DO_SOMETHING_ACTION_NODE_DO_SOMETHING_ACTION_H

// IGMR package headers
#include <igmr_node_interface/node_base.h>
#include <template_actions/DoSomethingAction.h>

// ROS headers
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>


namespace template_nodes
{
    /**
     * @brief A simple example of an action client node.
     *
     * This class description is free to copy for everyone who is writing
     * breathtaking good source code for the COAR of the IGM.
     **/
    class DoSomethingAction : public igmr_node_interface::NodeBase
    {
    public:
        /**
         * @brief Constructor
         */
        DoSomethingAction();
        virtual ~DoSomethingAction() = default;

        /**
         * @brief Execute node logic.
         */
        virtual void run() override;

    private:
        using action_t        = template_actions::DoSomethingAction;              /**< Action type used by this node */
        using goal_t          = action_t::_action_goal_type::_goal_type;          /**< Goal type since action clients are stupid and made it private */
        using result_t        = action_t::_action_result_type::_result_type;      /**< Result type since action clients are stupid and made it private */
        using feedback_t      = action_t::_action_feedback_type::_feedback_type;  /**< Feedback type since action clients are stupid and made it private */
        using action_client_t = actionlib::SimpleActionClient<action_t>;          /**< Action client type used by this node */

    private:
        action_client_t action_client_;  /**< Action client */
    };
} // namespace template_nodes

#endif // TEMPLATE_NODES_DO_SOMETHING_ACTION_NODE_DO_SOMETHING_ACTION_H
