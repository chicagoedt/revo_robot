#include "roboteqLogger.h"
 
bool    RoboteqLogger::Open(const string& filePath, bool threded)
{
    _mx       = PTHREAD_MUTEX_INITIALIZER;
    _threaded = threded;

    Close();

    _file.open( filePath.c_str(), ios_base::out | ios_base::app );

    if( _file.is_open() )
    {
        _file << "+++++++++ Opened ++++++++" << endl;
        return true;
    }
    else
        return false;
}

void     RoboteqLogger::Close(void)
{
    if( _file.is_open() )
    {
        _file << "--------- Closed --------" << endl;
        _file.close();
    }
}

void    RoboteqLogger::LogLine(const char* pBuffer, unsigned int len)
{
    if( _file.is_open() )
    {
        if( _threaded )
            pthread_mutex_lock(&_mx);

        _file.write(pBuffer, len) << std::endl;

        if( _threaded )
            pthread_mutex_unlock(&_mx);
    }
}

void    RoboteqLogger::LogLine(const std::string& message)
{
    if( _file.is_open() )
    {
        if( _threaded )
            pthread_mutex_lock(&_mx);

        _file << message << std::endl;

        if( _threaded )
            pthread_mutex_unlock(&_mx);
    }
}

// DO NOT Write new line at end
void    RoboteqLogger::Log(const char* pBuffer, unsigned int len)
{
    if( _file.is_open() )
    {
        if( _threaded )
            pthread_mutex_lock(&_mx);

        _file.write(pBuffer, len);
    
        if( _threaded )
            pthread_mutex_unlock(&_mx);
    }
}

void    RoboteqLogger::Log(const std::string& message)
{
    if( _file.is_open() )
    {
         if( _threaded )
            pthread_mutex_lock(&_mx);

        _file << message;

        if( _threaded )
            pthread_mutex_unlock(&_mx);
    }
}

