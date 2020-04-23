/**
 * @file
 * @brief This file contains the declaration of the ExampleClass class
 *
 * @author Marius Gürtler
 * @version 1.0
 * @date 31.10.2017
 */

#ifndef TEMPLATE_NODES_EXAMPLE_LIBRARY_EXAMPLE_CLASS_H
#define TEMPLATE_NODES_EXAMPLE_LIBRARY_EXAMPLE_CLASS_H


namespace template_nodes
{
    /**
     * @brief A simple example C++ lib class
     *
     * This class description is free to copy for everyone who is writing
     * breathtaking good source code for the COAR of the IGMR
     */
    class ExampleClass
    {
    public:
        /**
         * @brief Constructor
         */
        ExampleClass();

        /**
         * @brief Conducts a heavy computation
         */
        void conductHeavyComputation();

        /**
         * @brief Sets the input
         * @param input The new input value
         */
        void setInput(int input);

        /**
         * @brief Returns the result
         */
        int getResult();

    private:
        int input_;  /**< Some input */
        int result_; /**< Some result */
    };
} // namespace template_nodes

#endif // TEMPLATE_NODES_EXAMPLE_LIBRARY_EXAMPLE_CLASS_H
