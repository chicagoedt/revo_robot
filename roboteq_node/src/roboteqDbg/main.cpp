#include <signal.h>
#include <iostream>
#include "mainWindow.h"

MainWindow* app(0L);

static void PrintHelp(std::string progName)
{
    string::size_type Idx = progName.find_last_of("\\/");

    if( Idx != string::npos )
        progName = progName.substr( Idx + 1 );

    cout << endl;
    cout << "Usage: " << progName << " -p /dev/tty.???" << endl;
    cout << "   -p /dev/tty.???    - path to device" << endl;
    cout << "   -l count           - number of restarts" << endl;
    cout << "   -d ms              - ms delay between connect/disconnect" << endl;
    cout << "   -f cmdFile.robo    - comands to run on each connect" << endl;
    cout << endl;
}

static void SigInt(int sig)
{
    if( app != 0L )
        app->Shutdown();
}

static void SigResize(int sig)
{
    if( app != 0L )
        app->ResizeNotify();
}

int main(int argc, char* argv[])
{
    if( argc == 1 )
    {
        PrintHelp(argv[0]);
        return 0;
    }

    app = new MainWindow();

    setlocale(LC_CTYPE, "");

    signal(SIGINT,   SigInt);
    signal(SIGWINCH, SigResize);

    app->Title("RoboteqDbg");

    try
    {
        if( app->Initialize(argc, argv) )
            app->Run();

        refresh();
    }
    catch(std::exception& ex)
    {
        printw("Exception: %s\n\n", ex.what() );
        printw("< Press any key to exit...>" );
        refresh();
        getch();
    }
    catch(...)
    {
        printw("Exception: General");
        printw("< Press any key to exit...>" );
        refresh();
        getch();
    }

    delete app;

    return 0;
}

