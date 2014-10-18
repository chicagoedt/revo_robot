#include "mainWindow.h"
#include <string.h>
#include <unistd.h>
#include <fstream>
#include <iostream>

#define COLOR_TOP_BKG       1
#define COLOR_BOTTOM_BKG    2 
#define COLOR_MSG_IN        3
#define COLOR_MSG_OUT       4
#define COLOR_MSG_RED       5

#define LABEL_LEFT          0
#define LABEL_CENTER        1
#define LABEL_RIGHT         2

#define KEY_F1              1
#define KEY_F2              2
#define KEY_F3              3
#define KEY_F4              4
#define KEY_F5              5
#define KEY_F6              6
#define KEY_F7              7
#define KEY_F8              8
#define KEY_F9              9
#define KEY_F10             10
#define KEY_F11             11
#define KEY_F12             12

#define LOG_FILE_NAME       "roboteqDbg.log"

std::string     trimRight(const string& str, char ch)
{
    string::size_type Idx = str.find_last_not_of(ch);

    if( Idx != string::npos )
        return str.substr(0, Idx+1);
    else if( str.length() > 0 && str[0] == ch)
        return "";
    else return str;
}

std::string     trimLeft(const string& str, char ch)
{
    string::size_type Idx = str.find_first_not_of(ch);

    if( Idx != string::npos )
        return str.substr(Idx);
    else if( str.length() > 0 && str[str.length()-1] == ch)
        return "";
    else return str;
}

std::string     trimBoth(const string& str, char ch)
{
    return trimRight( trimLeft( str, ch), ch);
}


MainWindow::MainWindow(void)
 : _comunicator(_logger, *this), _looped(0), _delay(0)
{
    CTorInit();
}

MainWindow::MainWindow(const string& title)
 : _comunicator(_logger, *this), _looped(0), _delay(0)
{
    _title = title;
    CTorInit();
}

MainWindow::~MainWindow(void)
{
    Close();
    endwin();
}

void    MainWindow::Title(const string& title)
{
    _title = title;
    RepaintTopPanel();
}

bool    MainWindow::Initialize(int argc, char* argv[])
{
    if( argc == 1 )
        return false;

    int         c;
    string      filePath;

    while( (c = getopt( argc, argv, "p:d:f:l:")) != -1 )
    {
        switch( c )
        {
            case 'p':
                _device = optarg;
                break;

            case 'd':
                if( optarg != 0L)
                {
                    istringstream a2i( optarg );

                    if( ! (a2i >> _delay ) || (_delay < 100 || _delay > 5000) )
                    {
                        cerr << "Invalid delay value -d" << endl; 
                        return false;
                    }
                }
                break;

            case 'f':
                filePath = optarg;
                break;

            case 'l':
                if( optarg != 0L)
                {
                    istringstream a2i( optarg );

                    if( ! (a2i >> _looped) || _looped > 100000 )
                    {
                        cerr << "Invalid restart value -l" << endl; 
                        return false;
                    }
                }
                break;

            default:
                break;
        }
    }

    if( _device.empty() )
    {
        cout << "Error: missing device path" << endl;
        return false;
    }

    if( filePath.length() )
        LoadCommands(filePath);

    if( has_colors() == false )
    {
        addstr("Terminal CANNOT do colors");
        refresh();
        napms(1000); 
        clear();       
    }
    else
        start_color();

    getmaxyx(stdscr, _parentY, _parentX); 

    init_pair(COLOR_TOP_BKG,    COLOR_WHITE,COLOR_BLUE);
    init_pair(COLOR_BOTTOM_BKG, COLOR_WHITE,COLOR_GREEN);
    init_pair(COLOR_MSG_IN,     COLOR_CYAN, COLOR_BLACK);
    init_pair(COLOR_MSG_OUT,    COLOR_RED,  COLOR_BLACK);
    init_pair(COLOR_MSG_RED,    COLOR_RED,  COLOR_BLUE);

    CreateTopWindow();
    CreateMiddleWindow();

    if( _commands.size() == 0 )
        CreateBottomWindow();

    if( _logger.Open(LOG_FILE_NAME, _comunicator.IsThreaded() ) == false )
        THROW_RUNTIME_ERROR(string("Failed to open ") + LOG_FILE_NAME);

    _comunicator.Open( _device );

    // Done in roboteqCom class
    _comunicator.IssueCommand("?S");    // Query for speed and enters this speed
                                        // request into telemetry system
    _comunicator.IssueCommand("# 200"); // auto message responce is 200ms

    return true;
}

void    MainWindow::Run(void)
{
    RepaintAllPanels();

    if( _commands.size() )
        EnterAutoCommandMode();
    else
        EnterInterectiveMode();
}

void    MainWindow::EnterAutoCommandMode(void)
{
    try
    {
        do
        {
            TVec::const_iterator iter = _commands.begin();

            while( iter != _commands.end() && _keepRunning )
            {
                if( (*iter)[0] != '*')
                    _comunicator.IssueCommand(*iter);
                else
                {
                    istringstream   a2i( (iter->c_str() + 1) );
                    int             delay;

                    a2i >> delay;
            
                    napms(delay);             
                }

                iter++;
            }
   
            _comunicator.Close();

            if( _keepRunning == false )
                break;

            if( _looped > 0 )
            {
                napms(_delay);             
                _comunicator.Open( _device );

                _comunicator.IssueCommand("# C");   // Clears out telemetry strings
                _comunicator.IssueCommand("?S");    // Query for speed and enters this speed
                                        // request into telemetry system
                _comunicator.IssueCommand("# 200"); // auto message responce is 200ms
            }
        }
        while( _looped-- );
    }
    catch(std::exception& ex)
    {
        _logger.LogLine(string("Exception: ") + ex.what() ); 
    }
}

void    MainWindow::EnterInterectiveMode(void)
{
    nodelay(_bottom, TRUE);
    keypad(_bottom, TRUE);

    while( _keepRunning )
    {
        //CheckForResize();

        int ch = wgetch(_bottom );

        if( ch != ERR )
        {
            switch( ch )
            {
                case '\n':
                    if( _inputLen > 0 )
                        ProcessUserRequest(); 
                    break;

                case KEY_F( KEY_F1 ):
    
                    break;

                case KEY_F( KEY_F2 ):
                    if( _logger.IsLogOpen() )
                    {
                        slk_set(KEY_F2, "F2 Log N",   LABEL_CENTER);
                        _logger.Close();
                    }
                    else
                    {
                        slk_set(KEY_F2, "F2 Log Y",   LABEL_CENTER);

                        if( _logger.Open(LOG_FILE_NAME, _comunicator.IsThreaded() ) == false )
                            THROW_RUNTIME_ERROR(string("Failed to open ") + LOG_FILE_NAME);
                    }
                    slk_refresh();
                    break;

                case KEY_F( KEY_F3 ):
                    wclear(_middle); 
                    wrefresh(_middle);
                    _lines = 0;
                    break;

                case KEY_F( KEY_F4 ):
                    Shutdown();
                    break;

                case KEY_F( KEY_F5 ):
                case KEY_F( KEY_F6 ):
                case KEY_F( KEY_F7 ):
                case KEY_F( KEY_F8 ):
                case KEY_F( KEY_F9 ):
                case KEY_F( KEY_F10 ):
                case KEY_F( KEY_F11 ):
                case KEY_F( KEY_F12 ):
                    break;  // Do nothing
    
                default:
                    if( isprint( ch ) && _inputLen < MaxBufferSize )
                    {
                        _inputBuffer[_inputLen] = (char)ch;

                        if( _inputLen == 0 )
                            mvwaddnstr(_bottom, 1, 0, _inputBuffer, ++_inputLen);                
                        else
                        {
                            waddnstr(_bottom, _inputBuffer + _inputLen, 1);
                            ++_inputLen;
                        }
                    }
                    else if( ch == 127 )
                    {
                        if( _inputLen > 0 )
                        {
                            _inputLen--;
                            _inputBuffer[_inputLen] = '\0';;
                            mvwdelch(_bottom, 1, _inputLen);
                        }
                    }

                    break;
            }

        }
        else
            napms(150);
    }

    _comunicator.Close();
    _logger.Close();
}

void    MainWindow::Shutdown(void)
{
    _keepRunning = false;
}

void    MainWindow::Close(void)
{
    if( _top != 0L )
    {
        delwin(_top);
        _top = 0L;
    }

    if( _bottom != 0L )
    {
        delwin(_bottom);
        _bottom = 0L;
    }
    
    _inputLen = 0;
}

void    MainWindow::CheckForResize(void)
{
   if( _winSizeChanged == false )
       return;

    _winSizeChanged = false;

    getmaxyx(stdscr, _parentX, _parentY);

    wresize(_top, TopSize, _parentX);
    wresize(_middle, _parentY - (TopSize + BottomSize), _parentX);

    if( _bottom != 0L )
        wresize(_bottom, BottomSize, _parentX);

    mvwin(_top,    0, 0);
    mvwin(_middle, TopSize, 0);

    if( _bottom != 0L )
        mvwin(_bottom, _parentY - BottomSize, 0);

    wclear(_top);
    wclear(_middle);
    
    if( _bottom != 0L )
        wclear(_bottom);
    DrawAllBorders();
    //RepaintAllPanels();
}

void    MainWindow::CTorInit(void)
{
    _top            = 0L;
    _middle         = 0L;
    _bottom         = 0L;
    _parentX        = 0;
    _parentY        = 0;
    _inputLen       = 0;
    _lines          = 0;
    _keepRunning    = true;
    _winSizeChanged = false;

    memset(_inputBuffer, 0, MaxBufferSize+1);

    slk_init(1);
    initscr();

    slk_set(KEY_F1, "F1 Reset", LABEL_CENTER);
    slk_set(KEY_F2, "F2 Log Y", LABEL_CENTER);
    slk_set(KEY_F3, "F3 Clear", LABEL_CENTER);
    slk_set(KEY_F4, "F4 Quit",  LABEL_CENTER);

    cbreak();
    noecho();
    curs_set(FALSE);

    //keypad(_bottom, TRUE);
    //intrflush(_bottom, FALSE);
    slk_refresh();
}

void    MainWindow::CreateTopWindow(void)
{
    if( _top != 0L )
        return;

    _top = newwin(TopSize, _parentX, 0, 0);
    DrawTopBorders();
} 

void    MainWindow::CreateMiddleWindow(void)
{
    if( _middle != 0L )
        return;

    _middle = newwin(_parentY - (TopSize + BottomSize), _parentX, TopSize, 0);

    scrollok(_middle, TRUE);

    DrawMiddleBorders();
}

void    MainWindow::CreateBottomWindow(void)
{
    if( _bottom != 0L )
        return;

    _bottom = newwin(BottomSize, _parentX, _parentY - BottomSize, 0);
    DrawBottomBorders();
}

void    MainWindow::DrawAllBorders(void)
{
    DrawTopBorders();
    DrawMiddleBorders();
    DrawBottomBorders();
}

void    MainWindow::DrawTopBorders(void)
{
    if( _top != 0L )
    {
        //wborder(_top, ' ', ' ', ' ', '-', ' ', ' ', '-', '-' );
        wbkgd(_top, COLOR_PAIR(1));
    }
}

void    MainWindow::DrawMiddleBorders(void)
{
    // if( _middle != 0L )
    //   wborder(_bottom, ' ', ' ', '-', ' ', '-', '-', ' ', ' ' );
}

void    MainWindow::DrawBottomBorders(void)
{
    if( _bottom != 0L )
    {
        wclear(_bottom);
        wborder(_bottom, ' ', ' ', '_', ' ', '_', '_', ' ', ' ' );
    }
}

void    MainWindow::RepaintAllPanels(void)
{
    RepaintTopPanel();
    RepaintMiddlePanel();
    RepaintBottomPanel();
}

void    MainWindow::RepaintTopPanel(void)
{
    if( _top == 0L )
        return;

    wbkgd(_top, COLOR_PAIR(COLOR_TOP_BKG));

    ostringstream i2a;

    if( _title.length() )
        mvwaddstr(_top, 0, 0, _title.c_str());
    else
        wmove(_top, 0, 0);
        
    i2a << " " << _parentX << "x" << _parentY; 
    wattrset(_top, COLOR_PAIR( COLOR_MSG_RED ));
    waddstr(_top, i2a.str().c_str());
    wattroff(_top, COLOR_PAIR( COLOR_MSG_RED ));

    i2a.clear();

    if( _comunicator.Model().length() && _comunicator.Version().length() )
    {
        waddstr(_top, " Model: "); 
        wattrset(_top, COLOR_PAIR( COLOR_MSG_RED ));
        waddstr(_top, _comunicator.Model().c_str() ); 
        wattroff(_top, COLOR_PAIR( COLOR_MSG_RED ));

        waddstr(_top, "  Version: "); 
        wattrset(_top, COLOR_PAIR( COLOR_MSG_RED ));
        waddstr(_top, _comunicator.Version().c_str() ); 
        wattroff(_top, COLOR_PAIR( COLOR_MSG_RED ));
    }

    // refresh each window
    wrefresh( _top );
}

void    MainWindow::RepaintMiddlePanel(void)
{
    // refresh each window
    wrefresh( _middle );
}

void    MainWindow::RepaintBottomPanel(void)
{
    //wbkgd(_bottom, COLOR_PAIR(COLOR_BOTTOM_BKG));
    // refresh each window
    if( _bottom != 0L )
        wrefresh( _bottom );
}

void    MainWindow::ProcessUserRequest(void)
{
    _inputBuffer[_inputLen] = '\0';

    AppendText( _middle, eMsgDir_OUT, _inputBuffer );

    _comunicator.IssueCommand(_inputBuffer, _inputLen);

    if( _bottom != 0L )
    {
        wmove(_bottom, 1, 0);
        wclrtoeol(_bottom);
    }

    _inputLen = 0;
}

bool    MainWindow::LoadCommands(const string& filePath)
{
    _commands.clear();

    ifstream stream;

    stream.open( filePath.c_str() );

    if( stream.is_open() == false )
        return false;

    string command;

    while(std::getline(stream, command))
    {
        command = trimBoth( command, ' ' );

        if( command.length() )
            _commands.push_back( command );
    }

    return true;
}

void    MainWindow::AppendText(WINDOW* win, eMsgDir dir, const char* msg)
{
    RoboScopedMutex scopedMtx(_mutex);
    
    if( _lines == (_parentY - (TopSize + BottomSize) ) )
    {
        wscrl(_middle, 1);
        --_lines;
    }

    if( dir == eMsgDir_IN )
    {
        wattrset(_middle, COLOR_PAIR( COLOR_MSG_IN ));
        mvwaddnstr(_middle, _lines, 0, "> : ", sizeof("> : "));
        wattroff(_middle, COLOR_PAIR( COLOR_MSG_IN )); 

        if( _logger.IsLogOpen() )
        {
            string  str("RoboteqCom > ");
                    str.append(msg);

            _logger.LogLine(str);
        } 
    
    }
    else
    {
        wattrset(_middle, COLOR_PAIR( COLOR_MSG_OUT ));
        mvwaddnstr(_middle, _lines, 0, "< : ", sizeof("< : "));
        wattroff(_middle, COLOR_PAIR( COLOR_MSG_OUT )); 

	if( _logger.IsLogOpen() )
	{
        string str("RoboteqCom < ");
		str.append(msg);
		_logger.LogLine(str);
	}
    }

    waddstr(_middle, msg);

    ++_lines;

    wrefresh( _middle );
}

void    MainWindow::OnMsgEvent(IEventArgs& evt)
{
    AppendText(_middle, eMsgDir_IN, evt.Reply().c_str() ); 

    switch( evt.Reply()[0] )
    {
            case 'S':
                    Process_S( evt );
            break;

            case 'N':
                    Process_N( evt );
            break;

            default:
            break;
    }
}

void    MainWindow::Process_S(const IEventArgs& evt)
{
        string::size_type idx = evt.Reply().find_first_of('=');

        if( idx != string::npos )
        {
                string::size_type idy = evt.Reply().find_first_of(':', idx);

                if( idy != string::npos )
                {
                        //char* pVal1 = (char*)(evt.Request().c_str() + idx + 1);
                        //char* pVal2 = (char*)(evt.Request().c_str() + idy);

                        //*pVal2 = 0L;
                        //pVal2++;

                        //int firstVal  = atoi( pVal1 );
                        //int secondVal = atoi( pVal2 );
                }
        }
}

void    MainWindow::Process_N(const IEventArgs& evt)
{

}


