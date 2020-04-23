#include "do_something_big.h"


namespace template_nodes
{
    void DoSomethingBig::subCallback(const template_msgs::DoSomething::ConstPtr & msg)
    {
        ROS_INFO("Recieved data: [%s]", msg->data.c_str());
    }
} // namespace template_nodes
