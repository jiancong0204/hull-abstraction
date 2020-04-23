/**
 * @file
 * @brief This file contains the declarations for the example_templates library
 *
 * @author Henrik Stickann
 * @version 1.0
 * @date 01.12.2017
 */

#ifndef TEMPLATE_NODES_EXAMPLE_TEMPLATES_EXAMPLE_TEMPLATES_H
#define TEMPLATE_NODES_EXAMPLE_TEMPLATES_EXAMPLE_TEMPLATES_H


namespace template_nodes
{
    /**
     * @brief The TemplateClass class
     *
     * This class is an example class how to integrate
     * breathtaking good source code for the COAR of the IGMR
     */
    template <typename T>
    class TemplateClass
    {
    public:
        T add(T a, T b);
    };

    /**
     * @brief Constructs object of type T
     */
    template <typename T>
    T * createNew();

    /**
     * @brief Prints a greeting to stdout
     */
    void sayHello();
} // namespace template_nodes


// This is a very important line! It includes the definitions for the template class and function
#include "example_templates.tpp"


#endif // TEMPLATE_NODES_EXAMPLE_TEMPLATES_EXAMPLE_TEMPLATES_H
