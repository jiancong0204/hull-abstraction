/**
 * @file
 * @brief This file contains the declaration of the WaitThread class
 *
 * This file contains the declaration of the WaitThread class
 *
 * @author Johanna
 */

#ifndef TEMPLATE_NODES_THREADING_LIBRARY_WAITTHREAD_H
#define TEMPLATE_NODES_THREADING_LIBRARY_WAITTHREAD_H


namespace template_nodes
{
    /**
     * @brief This class is to be started in a Thread for demonstrating,
     * that the other threads are running, even though this thread is blocked.
     */
    class WaitThread
    {
    public:
        /**
         * @brief Runs the function "wait()", symbolizes a function, in which everything will be started and run, which is to be done on an extra thread
         */
        void startThread();

    private:
        /**
         * @brief Blocks the thread, for five seconds.
         */
        void wait();
    };
} // namespace template_nodes


#endif // TEMPLATE_NODES_THREADING_LIBRARY_WAITTHREAD_H
