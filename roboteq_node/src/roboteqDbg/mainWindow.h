#ifndef __MAIN_WINDOW_H__
#define __MAIN_WINDOW_H__

#include <ncurses.h>
#include <vector>
#include "roboteqCom.h"
#include "roboteqLogger.h"

// ncurses roboteq Dbg 
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

using namespace std;
using namespace oxoocoffee;

class MainWindow : public IEventListener<IEventArgs>
{
        typedef vector<string>  TVec;

        enum eMsgDir
        {
            eMsgDir_IN,
            eMsgDir_OUT
        };

    public:
        static const int TopSize       = 1;
        static const int BottomSize    = 2;
        static const int MaxBufferSize = 128;

    public:
                 MainWindow(void);
                 MainWindow(const string& title);
        virtual ~MainWindow(void);

        void    Title(const string& title);
        bool    Initialize(int argc, char* argv[]);
        void    Run(void);
        void    Shutdown(void);
        void    ResizeNotify(void) { _winSizeChanged = true; }

    private:
        void    AppendText(WINDOW* win, eMsgDir dir, const char* msg);
        void    CTorInit(void);
        void    CreateTopWindow(void);
        void    CreateMiddleWindow(void);
        void    CreateBottomWindow(void);
        void    DrawAllBorders(void);
        void    DrawTopBorders(void);
        void    DrawMiddleBorders(void);
        void    DrawBottomBorders(void);

        void    Close(void);
        bool    LoadCommands(const string& filePath);
        void    EnterAutoCommandMode(void);
        void    EnterInterectiveMode(void);
        void    CheckForResize(void);

        void    RepaintAllPanels(void);
        void    RepaintTopPanel(void);
        void    RepaintMiddlePanel(void);
        void    RepaintBottomPanel(void);

        void    ProcessUserRequest(void);

        // RoboteqCom Events
        virtual void OnMsgEvent(IEventArgs& evt);

        void    Process_S(const IEventArgs& evt);
        void    Process_N(const IEventArgs& evt);

    private:
        RoboteqLogger   _logger;
        RoboteqCom      _comunicator;
        RoboMutex       _mutex;            // Optionally used if RoboteqCom setup in threaded mode
        string          _title;
        WINDOW*         _top;
        WINDOW*         _middle;
        WINDOW*         _bottom;
        int             _lines;
        int             _parentX;
        int             _parentY;
        bool            _keepRunning;
        bool            _winSizeChanged;
        char            _inputBuffer[MaxBufferSize+1];
        int             _inputLen;

        string          _device;
        unsigned int    _looped;    // number ot restarts
        unsigned int    _delay;     // 100ms minimum
        TVec            _commands;        
};

#endif // __MAIN_WINDOW_H__


