#ifndef __SERIAL_EXCEPTION_H__
#define __SERIAL_EXCEPTION_H__

#include <stdexcept>
#include <sstream>

// Serial Exception macros
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

#define THROW_RUNTIME_ERROR(msg)                \
{                                               \
    std::ostringstream errMsg;                  \
    errMsg << msg;                              \
    errMsg << ". File: " << __FILE__;           \
    errMsg << " : "  << __LINE__;               \
    throw std::runtime_error(errMsg.str());     \
}

#define THROW_INVALID_ARG(msg)                  \
{                                               \
    std::ostringstream errMsg;                  \
    errMsg << msg;                              \
    errMsg << ". File: " << __FILE__;           \
    errMsg << " : "  << __LINE__;               \
    throw std::invalid_argument(errMsg.str());  \
}


#endif // __SERIAL_EXCEPTION_H__

