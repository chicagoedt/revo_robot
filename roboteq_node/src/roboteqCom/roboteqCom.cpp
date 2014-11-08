#include "roboteqCom.h"
#include <unistd.h>
#include <string.h> // For strtok

namespace oxoocoffee
{

#define     ROBO_SND_TERMINATOR		"\r"
#define     ROBO_RCV_TERMINATOR		'\n'	
#define	    ROBO_MSG_MAX		    1024

RoboteqCom::RoboteqCom(SerialLogger& log)
 : _port(log), _event(_dummyEvent), _thread(*this)
{
    // This is just to shut up compiler warning
    // of _dummyEvent not used
    IEventArgs dummy("");
    _dummyEvent.OnMsgEvent( dummy );

    CTorInit();
}

RoboteqCom::RoboteqCom(SerialLogger& log, IRoboteqEvent& event)
 : _port(log), _event(event), _thread(*this)
{
    CTorInit();
}

void    RoboteqCom::CTorInit(void)
{
    _port.canonical(SerialPort::eCanonical_Enable);
    _port.baud(115200);
    _port.dateSize(SerialPort::eDataSize_8Bit);
    _port.stopBit(SerialPort::eStopBit_1);
    _port.parity(SerialPort::eParity_None);
    _port.flowControl(SerialPort::eFlow_None);
}

void    RoboteqCom::Open(const string& device)
{
    _port.logLine("RoboteqCom - connecting");

    _port.connect( device );

    _port.logLine("RoboteqCom - connected ");

    if( IssueCommand("#") <= 0 )
    {
         _port.log("RoboteqCom - Clears out auto message responce FAILED ");
         throw std::runtime_error("RoboteqCom - Clears out auto message responce FAILED ");
    }

    if( IssueCommand("# C") <= 0 )
    {
         _port.log("RoboteqCom - Clears out telemetry strings FAILED ");
         throw std::runtime_error("RoboteqCom - Clears out telemetry strings FAILED ");
    }

    if( IssueCommand("^ECHOF 0") <= 0)
    {
	_port.log("RoboteqCom - ECHO OFF send FAILED ");
         throw std::runtime_error("RoboteqCom - ECHO OFF Send FAILED ");
    }


    // Eat all messages untill we get '+' from "^ECHOF 0". We know we are not in sync.
    while(ReadReply( _version ) > 0 && _version[0] != '+' )
	;

    if( IssueCommand("?$1E") > 0 )
    {
        _port.log("RoboteqCom - ver: ");
    
        if( ReadReply( _version ) > 0 )
        {
            string::size_type Idx = _version.find_first_of("=");

            if( Idx != string::npos)
                _version = _version.substr( Idx + 1, _version.size() - (Idx + 2));  // Strip '\r'

            _port.logLine(_version);

            if( IssueCommand("?$1F") > 0 )
            {
                _port.log("RoboteqCom - mod: ");
    
                if( ReadReply( _model ) > 0 )
                {
                    string::size_type Idx = _model.find_first_of(":");
                    
                    if( Idx != string::npos)
                        _model = _model.substr( Idx + 1, _model.size() - (Idx + 2));    // Strip '\r'

                    _port.logLine(_model);
                }
                else
                {
                    ostringstream i2a; i2a << "RoboteqCom - ERROR Model: errno " << errno;
                    _port.logLine(i2a.str());
                }
            }
            else
            {
                _port.log("RoboteqCom - checking model FAILED ");
                throw std::runtime_error("RoboteqCom - checking model FAILED ");
            }
        }
        else
        {
            ostringstream i2a; i2a << "RoboteqCom - ERROR Version: errno " << errno;
            _port.logLine(i2a.str());
        }

        _port.logLine("RoboteqCom - login ok");
    }
    else
    {
        _port.log("RoboteqCom - checking version FAILED ");
        throw std::runtime_error("RoboteqCom - checking version FAILED ");
    }

    if( _event.Type() == IRoboteqEvent::eReal )
    {
        // Running in threading mode
        _thread.Start();
        _port.logLine("RoboteqCom - reader started");
    }
}

void    RoboteqCom::Close(void)
{
    _mtx.Lock();
    if( _port.isOpen() )
        _port.disconnect();
    _mtx.UnLock();

    if( _event.Type() == IRoboteqEvent::eReal )
        _thread.Join();
}

int     RoboteqCom::IssueCommand(const char* buffer, int size)
{
    return IssueCommand( string(buffer, size) );
}

int     RoboteqCom::IssueCommand(const string&  command,
                                 const string&  args)
{
    if( _thread.IsRunning() )
    {
	RoboScopedMutex lock(_mtx);
	
        if(args == "")
            return _port.write(command + ROBO_SND_TERMINATOR);
        else
            return _port.write(command + " " + args + ROBO_SND_TERMINATOR);
    }
    else
    {
        if(args == "")
            return _port.write(command + ROBO_SND_TERMINATOR);
        else
            return _port.write(command + " " + args + ROBO_SND_TERMINATOR);
    }
}

int    RoboteqCom::ReadReply(string& reply)
{
    if( _thread.IsRunning() )
        throw std::runtime_error("RoboteqCom - ReadReply() not allowed when running is threaded mode");
 
    reply.clear();

    char buf[ROBO_MSG_MAX + 1];
    int  countRcv;
    
    while(true)
    {
    	if( (countRcv = _port.read(buf, ROBO_MSG_MAX)) <= 0 )
	    break;

        reply.append(buf, countRcv);

        if( buf[countRcv-1] == ROBO_RCV_TERMINATOR )
            return reply.length();
    }

    return countRcv;
}


// This methods runs on seperate thread
void    RoboteqCom::Run(void)
{
    int  countRcv;
    char buf[ROBO_MSG_MAX + 1];

    while( _port.isOpen() )
    {
        _mtx.Lock();
       	countRcv = _port.read(buf, ROBO_MSG_MAX); // ROBO_MSG_MAX - totalRcv);
	_mtx.UnLock();

	        if( countRcv > 0 )
            {
	    if( countRcv > 1 && buf[countRcv-1] == ROBO_RCV_TERMINATOR )
            	{
                if(buf[0] != '+')
                    {
                    IEventArgs evt(string(buf, countRcv-1));

                        _event.OnMsgEvent( evt );
                    }
            	}
	        }
	        else 
            {
		        _mtx.Lock();
		        _port.disconnect();
		        _mtx.UnLock();
		        break;
            }
        }
    }

}   // End of oxoocoffee namespace


