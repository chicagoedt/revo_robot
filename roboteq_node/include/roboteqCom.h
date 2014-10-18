#ifndef __ROBOTEQ_COM_H__
#define __ROBOTEQ_COM_H__

// Roboteq Communicator Class
// Robert J. Gebis (oxoocoffee) <rjgebis@yahoo.com>
// EDT Chicago (UIC) 2014
//
// Version 1.0
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation; either version 2 of
// the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Public License for more details at
// http://www.gnu.org/copyleft/gpl.html

#include "serialPort.h"
#include "roboteqComEvent.h"
#include "roboteqComEventArgs.h"
#include "roboteqThread.h"

namespace oxoocoffee
{

class RoboteqCom : public IRunnable
{
    typedef IEventListener<IEventArgs>  IRoboteqEvent;
    typedef IDummyListener<IEventArgs>  IDummyEvent;

    public:
        // Non threaded version
        RoboteqCom(SerialLogger& log);

        // Threaded version. We are using this one!!
        RoboteqCom(SerialLogger& log, IRoboteqEvent& event);
    
        void    Open(const string& device);
        void    Close(void);

        int     IssueCommand(const char* buffer, int size);
        int     IssueCommand(const string&  command,
                             const string&  args = "");

        int     ReadReply(string& reply);

        inline       bool    IsThreadRunning(void) const { return _thread.IsRunning(); }
        inline       bool    IsThreaded(void)      const { return _event.Type() == IRoboteqEvent::eReal; }
        inline const string& Version(void)         const { return _version; }
        inline const string& Model(void)           const { return _model; }

    protected:
        // IRunnable override
        // Run is run the thread
        // Join blocks for the thread to finish
        virtual void Run(void);

    private:
        void    CTorInit(void);

    private:
        string          _device;
        string          _version;
        string          _model;
        SerialPort      _port;
        IRoboteqEvent&  _event;
        IDummyEvent     _dummyEvent; // do not use it. Only used to init _event reference
        RoboteqThread   _thread;        
	RoboMutex	_mtx;
};

}   // End of amespace oxoocoffee

#endif // __ROBOTEQ_COM_H__

