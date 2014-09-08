#include <iostream>
#include <vector>
#include <unistd.h>
#include "roboteqCom.h"

using namespace oxoocoffee;

class RoboteqLogger : public SerialLogger
{
    public:
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

void    PrintHelp(string progName);

int main(int argc, char* argv[])
{
    if( argc < 2 )
    {
        PrintHelp(argv[0]);
        return 0;
    }

    try
    {
        int c;

        while( (c = getopt( argc, argv, "h")) != -1 )
        {
            switch( c )
            {
                case 'h':
                    PrintHelp(argv[0]);
                    return 0;
                    break;

                default:
                    THROW_INVALID_ARG("ERROR : Invalid switch");
                    return -1;
            }
        }

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

void    PrintHelp(string progName)
{
    string::size_type Idx = progName.find_last_of("\\/");

    if( Idx != string::npos )
        progName = progName.substr( Idx + 1 );

    cout << endl;
    cout << "Usage: " << progName << " [options]" << endl;
    cout << "   -h          - this information" << endl;
}


