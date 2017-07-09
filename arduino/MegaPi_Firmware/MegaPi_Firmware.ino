//
// MegaPi configuration:
//    . Motors with encoder on slot 1/2
//
// Features
//  . ROS parameter server for :
//     . motor timeout
//     . base_width, wheel_radius
//     . motor speed PID coefficients
//  . ROS 'cmd_vel' Twist message to drive motor
//  . ROS 'enableMotors' service server to activate motors

// TODO
// Use personal PID, speedPID only do P.


#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>

#include <MeMegaPi.h>

#define PI 3.1415

MeEncoderOnBoard Encoder1(SLOT1);
MeEncoderOnBoard Encoder2(SLOT2);

void interruptEncoder1(void) {
    if (digitalRead(Encoder1.getPortB())==0)
        Encoder1.pulsePosPlus();
    else
        Encoder1.pulsePosMinus();
}
void interruptEncoder2(void) {
    if (digitalRead(Encoder2.getPortB())==0)
        Encoder2.pulsePosPlus();
    else
        Encoder2.pulsePosMinus();
}

class NewHardware : public ArduinoHardware
{
    public:
        NewHardware():ArduinoHardware(&Serial2, 57600){};
};
ros::NodeHandle_<NewHardware>  nh;

bool   motors_enabled;  // When false cmd_vel message have no effect
long   motors_timer;    // Handles stopping motors after certain time without message
int    motors_timeout;  // Times in millisecond that the motors will be driven after a new cmd_vel message

float  baseWidth;       // Robot
float  wheelRadius;

char msg[128]; // Use to create debug message

// ROS service server 'enableMotors' that control if the motors are effectively driven or not
void enableMotorsCb(const std_srvs::SetBoolRequest & req, std_srvs::SetBoolResponse & res) {
    motors_enabled = req.data;
    if (!motors_enabled) {
        nh.loginfo("Motors have been stopped by service");
        Encoder1.runSpeed(0);
        Encoder2.runSpeed(0);
        motors_timer = 0;
    } else {
        nh.loginfo("Motors have been enabled by service");
    }
    res.success = true;
}
ros::ServiceServer<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse> enableMotorsServer("enableMotors",&enableMotorsCb);

// ROS subscriber to 'cmd_vel' that converts Twist message to left and right motor speeds
// No action if motors are not enabled
void cmdVelCb( const geometry_msgs::Twist& cmd_vel_msg){
    //nh.loginfo("Motor cmd_vel received");
    if (motors_enabled) {
        float v = cmd_vel_msg.linear.x;
        float w = cmd_vel_msg.angular.z;
        float vL = v-(w*baseWidth)/2;
        float vR = v+(w*baseWidth)/2;
        float vLrpm = vL*30/(PI*wheelRadius);
        float vRrpm = vR*30/(PI*wheelRadius);
        //sprintf(msg,"Current speed 1: %d vs %d 2: %d vs %d ; v:%d w:%d",
        //    int(1000*Encoder1.getCurrentSpeed())
        //,   int(1000*-vRrpm)
        //,   int(1000*Encoder2.getCurrentSpeed())
        //,   int(1000*vLrpm)
        //,   int(1000*v)
        //,   int(1000*w)
        //);
        //nh.loginfo(msg);
        Encoder1.runSpeed(-vRrpm);
        Encoder2.runSpeed(vLrpm);
        motors_timer = millis()+motors_timeout;
    }
}
ros::Subscriber<geometry_msgs::Twist> cmdVelSub("cmd_vel", &cmdVelCb);


void waitRosConnection() {

    float pidConstants[3];

    // Get Node parameters
    while (!nh.connected()) { nh.spinOnce(); };

    if (!nh.getParam("~motors_timeout", &motors_timeout)) { 
       nh.loginfo("Using default values for motors_timeout");
       motors_timeout = 1000;
    }
    if (!nh.getParam("~pid", pidConstants, 3)) { 
       nh.loginfo("Using default values for pid");
       pidConstants[0] = 0.18;
       pidConstants[1] = 0;
       pidConstants[2] = 0;
    }

    if (!nh.getParam("~baseWidth", &baseWidth)) { 
       baseWidth = 0.17;
    }
    if (!nh.getParam("~wheelRadius", &wheelRadius)) { 
       wheelRadius = 0.032;
    }

    Encoder1.setSpeedPid(pidConstants[0],pidConstants[1],pidConstants[2]);
    Encoder2.setSpeedPid(pidConstants[0],pidConstants[1],pidConstants[2]);

    Encoder1.setPulsePos(0);
    Encoder2.setPulsePos(0);

    motors_timer   = 0;
    motors_enabled = 0;
}

void setup() {

    nh.initNode();
    nh.subscribe(cmdVelSub);
    nh.advertiseService(enableMotorsServer);

    attachInterrupt(Encoder1.getIntNum(), interruptEncoder1, RISING);
    attachInterrupt(Encoder2.getIntNum(), interruptEncoder2, RISING);
}

void loop() {

    if (!nh.connected())
        waitRosConnection();

    if (motors_timer!=0 && millis()>motors_timer) {
        Encoder1.runSpeed(0);
        Encoder2.runSpeed(0);
        motors_timer = 0;
        nh.loginfo("Motors have been stopped after timeout");
    }

    Encoder1.loop();
    Encoder2.loop();
    nh.spinOnce();
}
