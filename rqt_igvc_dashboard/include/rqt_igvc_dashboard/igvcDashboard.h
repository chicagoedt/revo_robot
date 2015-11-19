#ifndef rqt_igvc_dashboardPlugin_H
#define rqt_igvc_dashboardPlugin_H

// ROS/RQT Plugin
// Developed by Krystian R. Gebis (krgebis at gmail dot com)

#include <QtCore/QObject>
#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <QWidget>
#include <fstream>

// Generated into ./build/WORK_SPACE/PACKAGE by Catkin
#include <ui_igvcDashboard.h>

namespace rqt_igvc_dashboard
{
class IgvcDashboard : public rqt_gui_cpp::Plugin
{
    Q_OBJECT

    public:
        IgvcDashboard();
            virtual void initPlugin(qt_gui_cpp::PluginContext& context);
            virtual void shutdownPlugin();
            virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings,
                                      qt_gui_cpp::Settings& instance_settings) const;
            virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                                         const qt_gui_cpp::Settings& instance_settings);
    private:
            void setupWidgets();
            void setupCallbacks();

    signals:

    protected slots:

    private:
        QWidget*               _parentWidget; 
        Ui::IgvcDashboardWidget   _ui;
        //ros::Subscriber      _fixSub;
};

} // namespace

#endif // rqt_igvc_dashboardPlugin_H

