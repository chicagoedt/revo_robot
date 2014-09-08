#ifndef __ROBOTEQ_COM_EVENT_ARGS_H__
#define __ROBOTEQ_COM_EVENT_ARGS_H__

#include <string>

// Event Handler Class
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

    class IEventArgs
    {
        public:
            IEventArgs(const string& reply)
              : _reply(reply)
	    {

		    if( isalpha( _reply[0] ) == 0 )
			    _reply = _reply.erase(0, 1);
        }

        inline const string& Reply(void)   const { return _reply;   }

        private:
            string _reply;
    };
}

#endif // __ROBOTEQ_COM_EVENT_ARGS__H__


