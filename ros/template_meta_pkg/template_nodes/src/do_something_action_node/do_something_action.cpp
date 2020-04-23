#include "do_something_action.h"


namespace template_nodes
{
    DoSomethingAction::DoSomethingAction()
        : NodeBase(10.0), action_client_(pnh_, "do_something_action", true)
    {}


    void DoSomethingAction::run()
    {
        // Wait for action client to connect to server
        action_client_.waitForServer();

        // Set goal parameters
        goal_t goal;
        goal.dishes_to_clean = 20;

        // Send goal to server
        action_client_.sendGoal(goal);

        // Wait for server to finish action
        action_client_.waitForResult(ros::Duration(20.0));
        if (action_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO_STREAM("Yay! " << action_client_.getResult()->total_dishes_cleaned << " dishes are now clean");

        ROS_INFO_STREAM("Current state: " << action_client_.getState().toString());
    }
} // namespace template nodes
