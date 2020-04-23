#include <template_nodes/example_library/example_class.h>


namespace template_nodes
{
    ExampleClass::ExampleClass()
    {}

    void ExampleClass::conductHeavyComputation()
    {
        result_ = input_ + input_;
    }

    void ExampleClass::setInput(int input)
    {
        input_ = input;
    }

    int ExampleClass::getResult()
    {
        return result_;
    }
} // namespace template_nodes
