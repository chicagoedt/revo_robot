#ifndef __ROBO_THREAD_H__
#define __ROBO_THREAD_H__

#include "roboteqMutex.h"

// Simple threading wraper
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
    class IRunnable
    {
        friend void* ThreadWrapper(void *ptr);

        protected:
            virtual void Run(void)   = 0;
    };

    class RoboteqThread
    {
        public:
                     RoboteqThread(IRunnable& r);
            virtual ~RoboteqThread(void);

            inline bool IsRunning(void) const { return _running; }

            void        Start(void);
            void        Join(void);

        private:
            IRunnable&      _runnable;
            pthread_t       _thread;
            bool            _running;
    };
} // End of namespace oxoocoffee

#endif
