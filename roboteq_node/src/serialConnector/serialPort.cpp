#include "serialPort.h"
#include <errno.h>
#include <string.h> // For strcmp on ubuntu
#include <stdlib.h> // For free() on ubuntu
#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <iostream>

int     fileFilter(const struct dirent* pEntry);

namespace oxoocoffee
{

// -1 means invalid file 
SerialPort::SerialPort(SerialLogger& log) 
 : INVALID_FD(-1), _logger(log), _fd(INVALID_FD)
{
    baud(9600);
    dateSize(eDataSize_8Bit);
    stopBit(eStopBit_1);
    parity(eParity_None);
    flowControl(eFlow_None);
}

SerialPort::~SerialPort(void)
{
}
        // device is /dev/tty???
void    SerialPort::connect(const string& device)
{
    if( device.empty() )
        THROW_INVALID_ARG("SerialPort - invalid device path")

    if( isOpen() )
        disconnect();

    if( _logger.IsLogOpen() )
        _logger.LogLine("SerialPort - opening " + device );

    _fd = open(device.c_str(), O_RDWR | O_NOCTTY); // | O_NDELAY); 

    if( isOpen() == false )
        THROW_RUNTIME_ERROR("SerialPort - failed to open device");

    if( ! ::isatty(_fd) )
    {
        disconnect();
        THROW_RUNTIME_ERROR("SerialPort - invalid device");
    }

    applySettings();

    if( _logger.IsLogOpen() )
        _logger.LogLine("SerialPort - connected " + device );
}

void    SerialPort::disconnect(bool echo)
{
    if( isOpen() )
    {
        if( echo && _logger.IsLogOpen() )
            _logger.LogLine("SerialPort - disconnect");

        ::close(_fd);
    }

    _fd = INVALID_FD;
}

void    SerialPort::canonical(const eCanonical mode)
{
    _canonical = mode;

    if( _logger.IsLogOpen() )
    {
        if( _canonical == eCanonical_Disable )
            _logger.LogLine("SerialPort - setting up raw mode");
        else
            _logger.LogLine("SerialPort - setting up line mode");
    }

    applySettings(); 
}

void    SerialPort::baud(const unsigned int& baud)
{
    if( _logger.IsLogOpen() )
    {
        ostringstream msg; msg << "SerialPort - setting baud to " << baud;
        _logger.LogLine( msg.str() );
    }

    switch (baud) 
    {
        case 50:        _baud = B50;     break;
        case 75:        _baud = B75;     break;
        case 110:       _baud = B110;    break;
        case 134:       _baud = B134;    break;
        case 150:       _baud = B150;    break;
        case 200:       _baud = B200;    break;
        case 300:       _baud = B300;    break;
        case 600:       _baud = B600;    break;
        case 1200:      _baud = B1200;   break;
        case 1800:      _baud = B1800;   break;
        case 4800:      _baud = B4800;   break;
        case 9600:      _baud = B9600;   break;
        case 19200:     _baud = B19200;  break;
        case 38400:     _baud = B38400;  break;
        case 57600:     _baud = B57600;  break;
        case 115200:    _baud = B115200; break;
        case 230400:    _baud = B230400; break;
        default:
            disconnect();
            THROW_INVALID_ARG("SerialPort - invalid port boud rate set");
            break;
    }

    applySettings(); 
}

void    SerialPort::dateSize(const eDataSize size)
{
    _dataSize = size;

    switch( size )
    {
        case eDataSize_5Bit:
            if( _logger.IsLogOpen() )
                _logger.LogLine("SerialPort - setting up data size to 5 bits"); break;
        case eDataSize_6Bit:
            if( _logger.IsLogOpen() )
                _logger.LogLine("SerialPort - setting up data size to 6 bits"); break;
        case eDataSize_7Bit:
            if( _logger.IsLogOpen() )
                _logger.LogLine("SerialPort - setting up data size to 7 bits"); break;
        case eDataSize_8Bit:
            if( _logger.IsLogOpen() )
                _logger.LogLine("SerialPort - setting up data size to 8 bits"); break;
        default:
            THROW_INVALID_ARG("SerialPort - invalid date size"); break;
    }

    applySettings(); 
}

void    SerialPort::parity(const eParity parity)
{
    _parity = parity;

    if( _logger.IsLogOpen() )
    {
        if( _parity == eParity_None )
            _logger.LogLine("SerialPort - disable parity");
        else if( _parity == eParity_Even )
            _logger.LogLine("SerialPort - enable odd parity");
        else
            _logger.LogLine("SerialPort - enable odd parity");
    }

    applySettings();
}

void    SerialPort::stopBit(const eStopBit stop)
{
    _stopBit = stop;

    if( _logger.IsLogOpen() )
    {
        if( stop == eStopBit_1 )
            _logger.LogLine("SerialPort - setting up 1 stop bit");
        else
            _logger.LogLine("SerialPort - setting up 2 stop bit");
    }

    applySettings(); 
}

void    SerialPort::flowControl(const eFlow flow)
{
    _flow = flow;

    if( _logger.IsLogOpen() )
    {
        if( _flow == eFlow_None )
            _logger.LogLine("SerialPort - disable flow control");
        else
            _logger.LogLine("SerialPort - enable flow control");
    }

    applySettings(); 
}

int     SerialPort::write(const string& mseeage)
{
    return SerialPort::write(mseeage.c_str(), mseeage.size() );
}

int     SerialPort::write(const char* pBuffer, const unsigned int numBytes)
{
    if( _fd == INVALID_FD )
        THROW_RUNTIME_ERROR("SerialPort - trying to write on closed device")
    else if( pBuffer == 0L )
        THROW_RUNTIME_ERROR("SerialPort - trying to write from null pointer")

    return ::write(_fd, pBuffer, numBytes);
}

int     SerialPort::read(char* pBuffer, const unsigned int numBytes)
{
    if( _fd == INVALID_FD )
	return _fd;
    else if( pBuffer == 0L )
        THROW_RUNTIME_ERROR("SerialPort - trying to read to null pointer")

        return ::read(_fd, pBuffer, numBytes);
}

void    SerialPort::enumeratePorts(SerialPort::TList& lst, const string& path)
{
    lst.clear();

    struct dirent** pNameList(0L);

    int items = scandir(path.c_str(), &pNameList, fileFilter, 0L);

    if (items < 0)
        THROW_INVALID_ARG("SerialPort - invalid device path")
    else
    {
        while ( items-- )
        {
            if (strcmp(pNameList[ items ]->d_name, "..") && strcmp(pNameList[ items ]->d_name, "."))
            {
                // Construct full absolute file path
                string devicePath(path + pNameList[items]->d_name);

                struct stat st;

                if ( lstat(devicePath.c_str(), &st) == 0 && !S_ISLNK(st.st_mode))
                    lst.push_back(devicePath);
            }

            free(pNameList[items]);
        }
    }

    if( pNameList != 0L )
        free(pNameList);
}

void    SerialPort::printPorts(void)
{
    TList lst;

    enumeratePorts(lst);  

    cout << "SerialPort - found " << lst.size() << " ports" << endl;

    TList::const_iterator iter = lst.begin();

    while( iter != lst.end() )
    {
        cout << "\t" << *iter << endl;
        ++iter;
    }
}

void    SerialPort::log(const string& msg)
{
    if( _logger.IsLogOpen() )
        _logger.Log(msg);
}

void    SerialPort::logLine(const string& msg)
{
    if( _logger.IsLogOpen() )
        _logger.LogLine(msg);
}

void    SerialPort::applySettings(void)
{
    if( isOpen() )
    {
        termios newtio;

	bzero(&newtio, sizeof(newtio));

	newtio.c_cflag = _baud | CRTSCTS | CS8 | CLOCAL | CREAD;
	newtio.c_iflag = IGNPAR | ICRNL;
	newtio.c_oflag = 0;
	newtio.c_lflag = ICANON;

	newtio.c_cc[VINTR]    = 0;     // Ctrl-c 
        newtio.c_cc[VQUIT]    = 0;     /* Ctrl-\ */
        newtio.c_cc[VERASE]   = 0;     // del
        newtio.c_cc[VKILL]    = 0;     // @
        newtio.c_cc[VEOF]     = 4;     // Ctrl-d
        newtio.c_cc[VTIME]    = 0;     // inter-character timer unused
        newtio.c_cc[VMIN]     = 1;     // blocking read until 1 character arrives
        newtio.c_cc[VSWTC]    = 0;     // '\0'
        newtio.c_cc[VSTART]   = 0;     // Ctrl-q 
        newtio.c_cc[VSTOP]    = 0;     // Ctrl-s
        newtio.c_cc[VSUSP]    = 0;     // Ctrl-z
        newtio.c_cc[VEOL]     = 0;     // '\0'
        newtio.c_cc[VREPRINT] = 0;     // Ctrl-r
        newtio.c_cc[VDISCARD] = 0;     // Ctrl-u
        newtio.c_cc[VWERASE]  = 0;     // Ctrl-w
        newtio.c_cc[VLNEXT]   = 0;     // Ctrl-v
        newtio.c_cc[VEOL2]    = 0;     // '\0' 

	tcflush(_fd, TCIFLUSH);

        if(tcsetattr(_fd, TCSANOW, &newtio)!= 0)
        {
            ostringstream err; err << "SerialPort - failed to apply changes. errno: " << errno;
            disconnect();
            THROW_RUNTIME_ERROR(err.str());
        }

/*
        termios options;

	bzero(&options, sizeof(options));

        if( tcgetattr(_fd, &options) != 0)
        {
            ostringstream err; err << "SerialPort - failed to probe device. errno: " << errno;
            disconnect();
            THROW_RUNTIME_ERROR(err.str());
        }

        // Set the read and write speed 
        if( ::cfsetispeed(&options, _baud) != 0)
            THROW_RUNTIME_ERROR("SerialPort - failed to set input baud speed");

        if( ::cfsetospeed(&options, _baud) != 0)
            THROW_RUNTIME_ERROR("SerialPort - failed to set output baud speed");

        // Enable the receiver and set local mode
        options.c_cflag |= (CLOCAL | CREAD);

        switch( _dataSize )
        {
            case eDataSize_5Bit:
                options.c_cflag = (options.c_cflag & ~CSIZE) | CS5; break;
            case eDataSize_6Bit:
                options.c_cflag = (options.c_cflag & ~CSIZE) | CS6; break;
            case eDataSize_7Bit:
                options.c_cflag = (options.c_cflag & ~CSIZE) | CS7; break;
            case eDataSize_8Bit:
                options.c_cflag = (options.c_cflag & ~CSIZE) | CS8; break;
            default:
                THROW_INVALID_ARG("SerialPort - invalid date size"); break;
        }

        options.c_cflag &= ~(PARENB | PARODD); 

        if( _parity == eParity_Even )
            options.c_cflag |= PARENB;
        else if( _parity == eParity_Odd )
            options.c_cflag |= (PARENB | PARODD);

        if( _stopBit == eStopBit_1 )
            options.c_cflag &= ~CSTOPB;
        else
            options.c_cflag |= CSTOPB;

        // Clear Software and Hardware Flow
        options.c_cflag &= ~CRTSCTS;                // Hardware 
        options.c_iflag &= ~(IXON | IXOFF | IXANY); // Software

        if( _flow == eFlow_Hardware )
            options.c_cflag |= CRTSCTS;
        else if( _flow == eFlow_Software )
            options.c_iflag |= (IXON | IXOFF); 

        // Line Mode (Canonical) or Raw Mode
        if( _canonical == eCanonical_Enable ) 
        {
            options.c_oflag = 0; //|= OPOST;               // Convert newlines into CR and LFs
            options.c_lflag = 0; // |= ICANON;
        }
        else
        { 
            //options.c_oflag &= ~OPOST;              // Do NOT convert newlines into CR and LFs 
            options.c_lflag &= ~(ICANON | ISIG);
            options.c_cc[VMIN]  = 1;                // 1 second timeout
            options.c_cc[VTIME] = 5;
        }

        tcflush(_fd, TCIFLUSH);
        tcflush(_fd, TCOFLUSH);

        if(tcsetattr(_fd, TCSANOW, &options)!= 0)
        {
            ostringstream err; err << "SerialPort - failed to apply changes. errno: " << errno;
            disconnect();
            THROW_RUNTIME_ERROR(err.str());
        }
*/
    }
}

} // end of namespace oxoocoffee

int fileFilter(const struct dirent* pEntry)
{
    return strstr(pEntry->d_name, "tty.") != 0L;
}

