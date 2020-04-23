/**
 * @file
 * @brief This file contains the definitions for the example_templates library templates
 *
 * @author Henrik Stickann
 * @version 1.0
 * @date 01.12.2017
 */

#ifndef EXAMPLEBOT_CONTROL_EXAMPLE_TEMPLATES_EXAMPLE_TEMPLATES_TPP
#define EXAMPLEBOT_CONTROL_EXAMPLE_TEMPLATES_EXAMPLE_TEMPLATES_TPP


namespace template_nodes
{
    template <typename T>
    T TemplateClass<T>::add(T a, T b)
    {
        return a + b;
    }


    template <typename T>
    T * createNew()
    {
        return new T();
    }
} // namespace template_nodes


#endif // EXAMPLEBOT_CONTROL_EXAMPLE_TEMPLATES_EXAMPLE_TEMPLATES_TPP
