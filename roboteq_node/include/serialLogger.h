#ifndef __SERIAL_LOGGER_H__
#define __SERIAL_LOGGER_H__

#include <string>

// Serial Logger Interface
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

class SerialLogger
{
    public:
        SerialLogger(void) {}

        virtual bool    IsLogOpen(void) const = 0;

		// Write new line at end
        virtual void    LogLine(const char* pBuffer, unsigned int len) {}
        virtual void    LogLine(const std::string& message) {}
		
		// DO NOT Write new line at end
        virtual void    Log(const char* pBuffer, unsigned int len) {}
        virtual void    Log(const std::string& message) {}
};

}   // End of namespace oxoocoffee

#endif // __SERIAL_LOGGER_H__

