#ifndef __ROBOTEW_MUTEX_H__
#define __ROBOTEW_MUTEX_H__

#include <pthread.h>
#include <errno.h>

// Robo Mutex and ScopedMutex
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
    class RoboMutex
    {
        public:
            RoboMutex(void) 
            {
		pthread_mutex_init(&_mx, NULL);
               // _mx = PTHREAD_MUTEX_INITIALIZER;
            }

           ~RoboMutex(void)
            {
                // Check if can lock it. We do not want
                // to use seperate bool value for that
                // If it can it wss not in locked state so unlock it
                // If it was already locked (EBUSY) unluck it.
                int ret = pthread_mutex_trylock(&_mx);

                if( ret == 0 || ret == EBUSY )
                    pthread_mutex_unlock( &_mx);
            }

            inline void    Lock(void)
                                { pthread_mutex_lock( &_mx); }
            inline void    UnLock(void)
                                { pthread_mutex_unlock( &_mx); }
                                

        private:
            pthread_mutex_t _mx;
    };

    class RoboScopedMutex
    {
        public:
            RoboScopedMutex(RoboMutex& mtx) : _mtx(mtx)
            {
	        _mtx.Lock();
            }

           ~RoboScopedMutex(void)
            {
            	_mtx.UnLock();
            }

        private:
            RoboMutex&  _mtx;
    };
}

#endif // __ROBOTEW_MUTEX_H__

