#include <ros/ros.h>

#include <rviz/panel.h>

#include <QPushButton>

namespace  rviz_plugin_dovlo_megapi {

class EnableMotorsPanel: public rviz::Panel {
// This class uses Qt slots and is a subclass of QObject, so it needs
// the Q_OBJECT macro.
Q_OBJECT
public:
    // QWidget subclass constructors usually take a parent widget
    // parameter (which usually defaults to 0).  At the same time,
    // pluginlib::ClassLoader creates instances by calling the default
    // constructor (with no arguments).  Taking the parameter and giving
    // a default of 0 lets the default constructor work and also lets
    // someone using the class for something else to pass in a parent
    // widget as they normally would with Qt.
    EnableMotorsPanel( QWidget* parent = 0 );

    // Now we declare overrides of rviz::Panel functions for saving and
    // loading data from the config file.
    virtual void load( const rviz::Config& config );
    virtual void save( rviz::Config config ) const;

  // Next come a couple of public Qt slots.
public Q_SLOTS:
    
  void enableMotor( bool enable );

  // Then we finish up with protected member variables.
protected:

  QPushButton* enableButton;

  // The ROS node handle.
  ros::NodeHandle nh_;

};

} // end namespace
