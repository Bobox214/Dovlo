//
// Setup encoder for slot 1/2
// Subscribe to cmd_vel s a Twist message
// Support parameters for robot base


#include <ros.h>
#include <geometry_msgs/Twist.h>

#include <MeMegaPi.h>

MeEncoderOnBoard Encoder1(SLOT1);
MeEncoderOnBoard Encoder2(SLOT2);

void interruptEncoder1(void) {
    if (digitalRead(Encoder1.getPortB())==0)
        Encoder1.pulsePosMinus();
    else
        Encoder1.pulsePosPlus();
}
void interruptEncoder2(void) {
    if (digitalRead(Encoder2.getPortB())==0)
        Encoder2.pulsePosMinus();
    else
        Encoder2.pulsePosPlus();
}

class NewHardware : public ArduinoHardware
{
    public:
        NewHardware():ArduinoHardware(&Serial2, 57600){};
};
ros::NodeHandle_<NewHardware>  nh;

long motors_timer;    // Handles stopping motors after certain time without message
int  motors_timeout;

void cmdVelCb( const geometry_msgs::Twist& cmd_vel_msg){
    digitalWrite(13, HIGH-digitalRead(13));   // blink the led
    nh.loginfo("Motors command received");
    motors_timer = millis() + motors_timeout;
}

void setup()
{
    nh.initNode();

    attachInterrupt(Encoder1.getIntNum(), interruptEncoder1, RISING);
    attachInterrupt(Encoder2.getIntNum(), interruptEncoder2, RISING);
    
    Encoder1.setMotorPwm(0);
    Encoder1.setPulsePos(0);
    Encoder1.setSpeedPid(0.18,0,0);
    Encoder2.setMotorPwm(0);
    Encoder2.setPulsePos(0);
    Encoder2.setSpeedPid(0.18,0,0);

    motors_timer = 0;

    //while (!nh.connected()) { nh.spinOnce(); };
    //if (!nh.getParam("~motors_timeout", &motors_timeout)) { 
    //   nh.loginfo("Using default_values for motors_timeout");
    //   motors_timeout = 1000;
    //}
    motors_timeout = 1000;
}

void loop() {

    if (motors_timer!=0 && millis()>motors_timer) {
        Encoder1.setMotorPwm(0);
        Encoder1.runSpeed(0);
        Encoder2.setMotorPwm(0);
        motors_timer = 0;
        nh.logdebug("Motors have been stopped after timeout");
    }
    if (millis()%10000==0) {
        Encoder1.runSpeed(100.0);
        motors_timer = millis() + motors_timeout;
    }

    Encoder1.loop();
    Encoder2.loop();
    nh.spinOnce();
}
