#include "do_something_big.h"


namespace template_nodes
{
    void DoSomethingBig::actionExecuteCallback(const action_server_t::GoalConstPtr & goal)
    {
        // Initialize feedback data
        action_server_t::Feedback feedback;
        feedback.percent_complete = 0.0;

        // Initialize result data
        action_server_t::Result result;
        result.total_dishes_cleaned = 0;

        bool success = true;
        float percent_per_interation = 100.0 / goal->dishes_to_clean;

        for (int i = 0; i < goal->dishes_to_clean; ++i)
        {
            // Handle abortion of action
            if (action_server_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("DoSomethingAction: Preempted");
                action_server_.setPreempted();
                success = false;
                break;
            }

            // Update feedback and result
            feedback.percent_complete += percent_per_interation;
            ++result.total_dishes_cleaned;
            // Publish the feedback
            action_server_.publishFeedback(feedback);

            keepLoopRate();
        }

        if (success)
        {
            ROS_INFO("DoSomethingAction: Succeeded");
            // Set the action state to succeeded
            action_server_.setSucceeded(result);
        }
    }
} // namespace template_nodes
