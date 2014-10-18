#ifndef __SERIAL_PORT_H__
#define __SERIAL_PORT_H__

#include "serialLogger.h"
#include "serialException.h"
#include <list>
#include <termios.h>

// Serial Device Class
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
 
// The default serial settings:
// 9600 bits/s
// 8-bit data
// 1 Start bit
// 1 Stop bit
// No Parity
// No Flow Control

// EDIT BEHAVIOR STUFF HERE

namespace oxoocoffee
{
    using namespace std;

    class SerialPort
    {
        const int   INVALID_FD;

        public:
            enum eParity
            {
                eParity_None,
                eParity_Even,
                eParity_Odd
            };

            enum eDataSize
            {
                eDataSize_5Bit,
                eDataSize_6Bit,
                eDataSize_7Bit,
                eDataSize_8Bit
            };

            enum eStopBit
            {
                eStopBit_1,
                eStopBit_2
            };

            enum eFlow
            {
                eFlow_None,
                eFlow_Hardware,
                eFlow_Software
            };

            enum eCanonical
            {
                eCanonical_Disable, // Raw
                eCanonical_Enable   // Line mode
            };

            typedef list<string>   TList;

                     SerialPort(SerialLogger& log);
            virtual ~SerialPort(void);

                            // connect will block if not device connected and running!!!!
            virtual void    connect(const string& device);
            virtual void    disconnect(bool echo = true);

            inline  bool    isOpen(void) const { return _fd != INVALID_FD; }
                            // Canonical mode does not work on Roboteq Device
                    void    canonical(const eCanonical mode);
                    void    baud(const unsigned int& baud);
                    void    dateSize(const eDataSize size);
                    void    stopBit(const eStopBit stop);
                    void    parity(const eParity parity);
                    void    flowControl(const eFlow flow);
            
                    int     write(const string& mseeage);
                    int     write(const char* pBuffer, const unsigned int numBytes);
                    int     read(char* pBuffer, const unsigned int numBytes);

                    void    log(const string& msg);
                    void    logLine(const string& msg);

            static  void    enumeratePorts(TList& lst, const string& path = "/dev/");
            static  void    printPorts(void);

        private:
                    void    applySettings(void);

        private:
            SerialLogger&   _logger;
            int             _fd;
            speed_t         _baud;
            eCanonical      _canonical;
            eParity         _parity;
            eDataSize       _dataSize;
            eStopBit        _stopBit;
            eFlow           _flow;
    };
}   // End of namespace oxoocoffee

#endif // __SERIAL_PORT_H__

