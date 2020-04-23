#include "do_something_big.h"


namespace template_nodes
{
    bool DoSomethingBig::serviceCallback(template_srvs::DoSomething::Request & req, template_srvs::DoSomething::Response & res)
    {
        int input = req.input_data;

        int output = input + 1;

        res.return_data = output;

        return true;
    }
} // namespace template_nodes
