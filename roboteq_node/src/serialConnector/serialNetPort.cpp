#include "serialNetPort.h"
#include <sstream>

namespace oxoocoffee
{

SerialNetPort::SerialNetPort(SerialLogger& log)
 : _logger(log)
{
}

SerialNetPort::~SerialNetPort(void)
{
}

bool    SerialNetPort::isOpen(void) const
{
    return false;
}

void    SerialNetPort::connect(const string& host, unsigned short port)
{
    ostringstream msg;
    msg << "SerialNetPort::connect " << host << ":" << port;

    _logger.Log( msg.str() );
}

void    SerialNetPort::disconnect(void)
{
    _logger.Log("SerialNetPort::disconnect");
}

} // end of namespace oxoocoffee

