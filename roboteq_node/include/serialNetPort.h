#ifndef __SERIAL_NETWORK_PORT_H__
#define __SERIAL_NETWORK_PORT_H__ 

#include "serialLogger.h"
#include "serialException.h"

// Serial Network Class
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
 
namespace oxoocoffee
{
    using namespace std;

    class SerialNetPort 
    {
        public:
                     SerialNetPort(SerialLogger& log);
            virtual ~SerialNetPort(void);

                    bool isOpen(void) const;
            virtual void connect(const string& host, unsigned short port);
            virtual void disconnect(void);
            
        private:
            SerialLogger&    _logger;
    };
}

#endif // __SERIAL_NETWORK_PORT_H__


