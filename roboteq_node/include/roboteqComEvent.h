#ifndef __ROBOTEQ_COM_EVENT_H__
#define __ROBOTEQ_COM_EVENT_H__

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
    template< typename T >
    class IEventListener
    {
        public:
            enum eType
            {
                eDummy,
                eReal
            };

            virtual void       OnMsgEvent(T& evt) = 0; 
            inline const eType Type(void) const { return _type; }

        private:
            eType _type;

        protected:
            IEventListener(eType type = eReal) : _type(type) {}
    };

    template< typename T >
    class IDummyListener : public IEventListener<T>
    {
        public:
            IDummyListener(void) : IEventListener<T>(IEventListener<T>::eDummy) {}

            virtual void OnMsgEvent(T& evt) {};
    };
}

#endif // __ROBOTEQ_COM_EVENT_H__


