#include <template_nodes/threading_library/waitthread.h>

#include <boost/thread.hpp>
#include <iostream>


namespace template_nodes
{
    void WaitThread::wait()
    {
        try
        {
            std::cout << "WaitThread: This thread will be blocked for five seconds" << std::endl;
            boost::this_thread::sleep_for(boost::chrono::seconds(5));
            std::cout << "WaitThread: Blocking time is over" << std::endl;
        }
        catch (const boost::thread_interrupted & e)
        {
            // Handle call to boost::thread.interrupt()
            std::cout << "WaitThread: Interrupted before sleep finished" << std::endl;
            // Rethrow exception to exit thread
            throw;
        }
    }

    void WaitThread::startThread()
    {
        wait();
    }
} // namespace template_nodes
