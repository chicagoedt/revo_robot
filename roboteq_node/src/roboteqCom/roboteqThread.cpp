#include "roboteqThread.h"
#include "serialException.h"
#include <stdexcept>

namespace oxoocoffee
{

void* ThreadWrapper(void *ptr)
{
    if( ptr != 0L )
        ((IRunnable*)ptr)->Run();

    return 0;
}

RoboteqThread::RoboteqThread( IRunnable& runnable )
 : _runnable(runnable), _running(false)
{
}

RoboteqThread::~RoboteqThread(void)
{
}

void    RoboteqThread::Start(void)
{
    if ( _running )
        throw std::runtime_error("Cant start thread. It is already started");
    else
    {
        if( pthread_create(&_thread, NULL, ThreadWrapper, (void*)&_runnable) != 0 )
            THROW_RUNTIME_ERROR("Couldn't start thread");

        _running = true;
    }
}

void    RoboteqThread::Join(void)
{
    if( _running )
    {
        if( pthread_join(_thread, NULL) != 0 )
            THROW_RUNTIME_ERROR("Couldn't join thread");

        _running = false;
    }
}

} // End of namespace oxoocoffee

