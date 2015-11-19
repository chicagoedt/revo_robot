#include "rqt_igvc_dashboard/igvcDashboard.h"
#include <pluginlib/class_list_macros.h>
#include <ros/package.h>
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include <QStringList>
#include <QDateTime>

namespace rqt_igvc_dashboard
{
IgvcDashboard::IgvcDashboard()
 : rqt_gui_cpp::Plugin(), _parentWidget( new QWidget() )
{
    // Constructor is called first before initPlugin function, needless to say.

    // give QObjects reasonable names
    rqt_gui_cpp::Plugin::setObjectName("IgvcDashboard");
}

void IgvcDashboard::initPlugin(qt_gui_cpp::PluginContext& context)
{
    // access standalone command line arguments
    QStringList argv = context.argv();

    if (context.serialNumber() > 1)
        _parentWidget->setWindowTitle(_parentWidget->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");

    // extend the widget with all attributes and children from UI file
    _ui.setupUi(_parentWidget);

    setupWidgets();

    // add widget to the user interface
    context.addWidget( _parentWidget);

    setupCallbacks();
}

void IgvcDashboard::shutdownPlugin()
{
}

void IgvcDashboard::saveSettings(qt_gui_cpp::Settings& plugin_settings,
                              qt_gui_cpp::Settings& instance_settings) const
{
}

void IgvcDashboard::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                                 const qt_gui_cpp::Settings& instance_settings)
{
}

void IgvcDashboard::setupWidgets()
{
    // Setup your QT Widgets and their signal/slot connections
}

void IgvcDashboard::setupCallbacks()
{
    // Setup your ROS NodeHandlers here
}

} // end of namespace 

PLUGINLIB_EXPORT_CLASS(rqt_igvc_dashboard::IgvcDashboard, rqt_gui_cpp::Plugin)



