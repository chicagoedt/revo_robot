#include <iostream>
#include <fstream>
#include <vector>
#include <iterator>
#include <unistd.h>
#include "serialPort.h"
#include "serialNetPort.h"

using namespace oxoocoffee;

class ScreenLogger : public SerialLogger
{
    public:
        virtual bool    IsLogOpen(void) const { return true; }
        virtual void    LogLine(const char* pBuffer, unsigned int len)
        {
            std::cout.write(pBuffer, len) << std::endl;
        }

        virtual void    LogLine(const std::string& message)
        {
            std::cout << message << std::endl;
        }
		
		// DO NOT Write new line at end
        virtual void    Log(const char* pBuffer, unsigned int len)
		{
			std::cout.write(pBuffer, len);
		}
		
        virtual void    Log(const std::string& message)
		{
			std::cout << message;
		}
};

typedef vector<string>              TStrParam;
typedef istream_iterator<string>    TStrIter;

class StringUtil
{
    public:
        static TStrParam    split(const string& str, char ch = ' ');
        static std::string  trimRight(const string& str, char ch = ' ');
        static std::string  trimLeft(const string& str, char ch = ' ');
        static std::string  trimBoth(const string& str, char ch = ' ');
};

void    ProcessSerialPort(const TStrParam& param);
void    ProcessSerialNetPort(const TStrParam& param);
void    PrintHelp(string progName);

enum eConnType
{
    eConnType_None,
    eConnType_Serial,
    eConnType_Network
};

int main(int argc, char* argv[])
{
    if( argc < 2 )
    {
        PrintHelp(argv[0]);
        return 0;
    }

    TStrParam params;

    for(int Idx(1); Idx < argc; Idx++)
    {
        if( string(argv[Idx]) == "-m")
            Idx++;
        else
            params.push_back( argv[Idx] );
    }
 
    try
    {
        bool        bLSwitch(false);
        bool        bTSwitch(false);
        bool        bFSwitch(false);
        bool        bPSwitch(false);
        eConnType   type(eConnType_None);
        int c;

        while( (c = getopt( argc, argv, "m:t:p:f:s:elh")) != -1 )
        {
            switch( c )
            {
                case 'm':
                    switch( optarg[0] )
                    {
                        case 's':
                            type = eConnType_Serial;
                            break;

                        case 'n':
                            type = eConnType_Network;
                            break;

                        default:
                            THROW_INVALID_ARG("ERROR : Invalid -m switch");
                    }
                    break;

                case 'h':
                    PrintHelp(argv[0]);
                    return 0;
                    break;

                case 'f':
                    if( std::string(optarg).empty() )
                        THROW_INVALID_ARG("ERROR : invalid file name -f");

                    bFSwitch = true;
                    break;

                case 'p':
                    if( std::string(optarg).empty() )
                        THROW_INVALID_ARG("ERROR : invalid file name -p");

                    bPSwitch = true;
                    break;

                case 'l':
                    bLSwitch = true;
                case 's':
                case 'e':
                    // Just to handle this as valid parameters
                    break;

                case 't':
                    switch( optarg[0] )
                    {
                        case 'r':
                        case 's':
                            bTSwitch = true;
                            break;

                        default:
                            THROW_INVALID_ARG("ERROR : Invalid -e switch");
                    }
                    break;

                default:
                    THROW_INVALID_ARG("ERROR : Invalid switch");
                    return -1;
            }
        }

        if( type == eConnType_None )
            THROW_INVALID_ARG("ERROR : missing -m switch");

        if( bLSwitch == false )
        {
            if( bTSwitch == false )
                THROW_INVALID_ARG("ERROR : missing -t switch");

            if( bFSwitch == false )
                THROW_INVALID_ARG("ERROR : missing -f switch");

            if( bPSwitch == false )
                THROW_INVALID_ARG("ERROR : missing -p switch");
        }

        if( params.empty() )
           THROW_INVALID_ARG("ERROR : mising parameters");

        if( type == eConnType_Serial )
            ProcessSerialPort(params);
        else if( type == eConnType_Network )
            ProcessSerialNetPort(params);
        else
            THROW_RUNTIME_ERROR("ERROR : invalid mode"); 
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        return -2;
    }
    catch (...)
    {
        std::cerr << "ERROR : general exception" << std::endl;
        return -3;
    }

    return 0;
}

void    ProcessSerialPort(const TStrParam& params)
{
    unsigned int             baud(115200);
    SerialPort::eCanonical   mode(SerialPort::eCanonical_Enable);
    SerialPort::eDataSize    dataSize(SerialPort::eDataSize_8Bit);
    SerialPort::eStopBit     stopBit(SerialPort::eStopBit_1);
    SerialPort::eParity      parity(SerialPort::eParity_None);
    SerialPort::eFlow        flow(SerialPort::eFlow_None);

    bool                       bReader(false);
    bool                       bEcho(false);
    std::string                filePath;
    string                     device;
    ScreenLogger               log;
    istringstream              a2i;
    TStrParam::const_iterator  iter = params.begin();

    log.LogLine("----------------------------------------------");

    while( iter != params.end() )
    {
        a2i.clear();

        if( *iter == "-l")
        {
            SerialPort::printPorts();
            return;
        } 
        else if( *iter == "-t")
        {
            ++iter;

            if( *iter == "r" )
            {
                log.LogLine("SerialPort - modei  x: receiving"); 
                bReader = true;
            }
            else
                log.LogLine("SerialPort - mode   : sending"); 
        }
        else if( *iter == "-f")
        {
            ++iter;
            filePath = *iter;
            log.LogLine(string("SerialPort - file   : ") + filePath); 
        }
        else if( *iter == "-p")
        {
            ++iter;
            device = *iter;
            log.LogLine(string("SerialPort - device : ") + device); 
        }
        else if( *iter == "-e" )
        {
            bEcho = true;
            log.LogLine("SerialPort - echo   : on"); 
        }
        else if( *iter == "-s" )
        {
            ++iter;
            TStrParam parameters = StringUtil::split(*iter, ':');
    
            int paramIdx(0);
            int size(0);

            TStrParam::const_iterator paramIter = parameters.begin();

            while( paramIter != parameters.end() )
            {
                a2i.clear();

                string val = StringUtil::trimBoth(*paramIter);

                if( val.length() )
                {
                    switch( paramIdx )
                    {
                        case 0:             // mode
                            if( val == "l" )
                                mode = SerialPort::eCanonical_Enable;
                            else if( val == "r" )
                                mode = SerialPort::eCanonical_Disable; 
                            else
                                THROW_INVALID_ARG("SerialPort : invalid line mode <l|r> : " + val);
                            break;

                        case 1:             // baud
                            a2i.str( val );
                            if( ! ( a2i >> baud ) )
                                THROW_INVALID_ARG("SerialPort : invalid speed <baud> : " + val ); 
                            break;

                        case 2:             // datasize 
                            size = 0;

                            a2i.str( val );
                            if( ! ( a2i >> size ) )
                                THROW_INVALID_ARG("SerialPort : invalid datasize <5|6|7|8> : " + val ); 

                            if( size == 5 )
                                dataSize = SerialPort::eDataSize_5Bit;
                            else if( size == 6 )
                                dataSize = SerialPort::eDataSize_6Bit;
                            else if( size == 7 )
                                dataSize = SerialPort::eDataSize_7Bit;
                            else if( size == 8 )
                                dataSize = SerialPort::eDataSize_8Bit;
                            else
                                THROW_INVALID_ARG("SerialPort : invalid datasize <5|6|7|8> : " + val );
                            break;

                        case 3:             // parity
                            if( val == "N" )
                                parity = SerialPort::eParity_None;
                            else if( val == "E" )
                                parity = SerialPort::eParity_Even;
                            else if( val == "O" )
                                parity = SerialPort::eParity_Odd;
                            else
                                THROW_INVALID_ARG("SerialPort : invalid parity <E|O|D> : " + val ); 
                            break;

                        case 4:             // bits
                             size = 0;
 
                             a2i.str( val );
                             if( ! ( a2i >> size ) )
                                 THROW_INVALID_ARG("SerialPort : invalid stopbit <1|2> : " + val );
                            if( size == 1 )
                                stopBit = SerialPort::eStopBit_1;
                            else if( size == 2 )
                                stopBit = SerialPort::eStopBit_2;
                            else
                                THROW_INVALID_ARG("SerialPort : invalid stopbit <1|2> : " + val );
                            break;

                        case 5:             // flow
                            if( val == "N" )
                                flow = SerialPort::eFlow_None;
                            else if( val == "S" )
                                flow = SerialPort::eFlow_Software;
                            else if( val == "H" )
                                flow = SerialPort::eFlow_Hardware;
                            else
                                THROW_INVALID_ARG("SerialPort : invalid flow control <S|H|D> : " + val ); 
                            break;

                    }
                }


                ++paramIdx;
                ++paramIter;
            }
        }

        ++iter;
    }

    log.LogLine("----------------------------------------------");

    SerialPort   port(log);

    try
    {
        port.canonical(mode);
        port.baud(baud);
        port.dateSize(dataSize);
        port.stopBit(stopBit);
        port.parity(parity);
        port.flowControl(flow);

        port.connect(device);

        char         buffer[512];
        unsigned int msgSize(1);

        if( bReader == false )
        {
            // Read from Serial Port
            ifstream file(filePath.c_str());

            if( file.is_open() == false )
                THROW_RUNTIME_ERROR("SerialPort - failed to open output file");

            log.LogLine(string("SerialPort - opened ") + filePath);
            log.Log("SerialPort > ");

            while( ! file.eof() )
            {
                file.read(buffer, msgSize);

                if( bEcho )
                    log.Log(buffer, msgSize);

                port.write(buffer, msgSize); 
            }
        }
        else
        {
            // Send to serial port
            ofstream file(filePath.c_str());

            if( file.is_open() == false )
                THROW_RUNTIME_ERROR("SerialPort - failed to open input file");

            log.LogLine(string("SerialPort - opened ") + filePath);
            log.Log("SerialPort < ");

            while( port.read(buffer, msgSize) > 0 )
            {
                if( bEcho )
                    log.Log(buffer, msgSize);

                file.write(buffer, msgSize);
            }
        }

        port.disconnect();
    }
    catch(...)
    {
        port.disconnect();
        throw;
    }
}

void    ProcessSerialNetPort(const TStrParam& params)
{
    ScreenLogger   log;
    SerialNetPort  port(log);

    try
    {
    }
    catch(...)
    {
        port.disconnect();
        throw;
    }
}

TStrParam    StringUtil::split(const string& str, char ch)
{
    TStrParam vec;

    if( str.empty() )
        return vec;

    string::size_type startIdx(0);

    while(true)
    {
        string::size_type startIdy = str.find_first_of(ch, startIdx);

        if( startIdy == string::npos )
        {
            vec.push_back( str.substr( startIdx ) );
            break;
        }
        else
        {
            vec.push_back( str.substr( startIdx, startIdy -  startIdx) );
            startIdx = startIdy+1;
        }
    }

    return vec;
}

std::string     StringUtil::trimRight(const string& str, char ch)
{
    string::size_type Idx = str.find_last_not_of(ch);

    if( Idx != string::npos )
        return str.substr(0, Idx+1);
    else if( str.length() > 0 && str[0] == ch)
        return "";
    else return str;
}

std::string     StringUtil::trimLeft(const string& str, char ch)
{
    string::size_type Idx = str.find_first_not_of(ch);

    if( Idx != string::npos )
        return str.substr(Idx);
    else if( str.length() > 0 && str[str.length()-1] == ch)
        return "";
    else return str;
}

std::string     StringUtil::trimBoth(const string& str, char ch)
{
    return trimRight( trimLeft( str, ch), ch);
}

void    PrintHelp(string progName)
{
    string::size_type Idx = progName.find_last_of("\\/");

    if( Idx != string::npos )
        progName = progName.substr( Idx + 1 );

    cout << endl;
    cout << "Usage: " << progName << " [options]" << endl;
    cout << "   -m s|n      - (s)erial or (n)network" << endl;
    cout << "   -t r|s      - serial (r)ecive or (s)sender" << endl;
    cout << "   -p dev|net  - tty path or host:port" << endl;
    cout << "   -s settings - tty or network" << endl;
    cout << "                 tty - > mode:speed:datasize:parity:bits:flow" << endl;
    cout << "                 ex. l|r:9600:5|6|7|8:E|O|N:1|2:H|S|N" << endl;
    cout << "                 default values r:115200:8:N:1:N" << endl; 
    cout << "   -h          - this information" << endl;
    cout << "   -e          - echo to screen" << endl;
    cout << "               - serial parameters section " << endl;
    cout << "    -f file    - file to transfer/save" << endl;
    cout << "     -l        - com tty devices found on system" << endl;
    cout << "               - network parameters section " << endl;
}

