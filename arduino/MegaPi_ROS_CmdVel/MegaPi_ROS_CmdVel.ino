//
// Setup encoder for slot 1/2
// Subscribe to cmd_vel s a Twist message
// Support parameters for robot base

// TODO
// Use personal PID, speedPID only do P.


#include <ros.h>
#include <geometry_msgs/Twist.h>

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
        NewHardware():ArduinoHardware(&Serial2, 115200){};
};
ros::NodeHandle_<NewHardware>  nh;

long   motors_timer;    // Handles stopping motors after certain time without message
int    motors_timeout;
float  baseWidth;
float  wheelRadius;

char msg[128];

void cmdVelCb( const geometry_msgs::Twist& cmd_vel_msg){
    motors_timer = millis() + motors_timeout;
    nh.loginfo("Motor cmd_vel received");
    float v = cmd_vel_msg.linear.x;
    float w = cmd_vel_msg.angular.z;
    float vL = v+(w*baseWidth)/2;
    float vR = v-(w*baseWidth)/2;
    float vLrpm = vL*30/(PI*wheelRadius);
    float vRrpm = vR*30/(PI*wheelRadius);
    sprintf(msg,"Current speed 1: %d vs %d 2: %d vs %d ; v:%d w:%d",
        int(1000*Encoder1.getCurrentSpeed())
    ,   int(1000*-vRrpm)
    ,   int(1000*Encoder2.getCurrentSpeed())
    ,   int(1000*vLrpm)
    ,   int(1000*v)
    ,   int(1000*w)
    );
    nh.loginfo(msg);
    Encoder1.runSpeed(-vRrpm);
    Encoder2.runSpeed(vLrpm);
    motors_timer = millis()+motors_timeout;
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

    motors_timer = 0;
}

void setup() {

    nh.initNode();
    nh.subscribe(cmdVelSub);

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
