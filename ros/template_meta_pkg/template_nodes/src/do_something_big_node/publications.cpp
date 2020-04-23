#include "do_something_big.h"


namespace template_nodes
{
    void DoSomethingBig::publish()
    {
        template_msgs::DoSomething msg;

        std::stringstream ss;
        ss << text_ << " " << result_ << std::endl;
        
        msg.data = ss.str();

        pub_.publish(msg);
    }
} // namespace template_nodes
