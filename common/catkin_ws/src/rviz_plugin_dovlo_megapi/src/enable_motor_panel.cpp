#include <stdio.h>

#include <QPushButton>
#include <QHBoxLayout>
#include <QLabel>

#include <std_srvs/SetBool.h>

#include "enable_motor_panel.h"

namespace rviz_plugin_dovlo_megapi {

EnableMotorsPanel::EnableMotorsPanel( QWidget* parent )
  : rviz::Panel( parent )
{
  // Next we lay out the "output topic" text entry field using a
  // QLabel and a QLineEdit in a QHBoxLayout.
  QHBoxLayout* layout = new QHBoxLayout;
  layout->addWidget( new QLabel( "Motors:" ));
  QPushButton* enableButton = new QPushButton;
  enableButton->setCheckable(true);
  layout->addWidget( enableButton );

  setLayout( layout );

  connect( enableButton, SIGNAL( toggled(bool) ), this, SLOT( enableMotor(bool) ));

}

void EnableMotorsPanel::enableMotor( bool enable ) {
    ROS_INFO("Test");
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void EnableMotorsPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
}

void EnableMotorsPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
}

} // end namespace

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin_dovlo_megapi::EnableMotorsPanel,rviz::Panel )
// END_TUTORIAL
