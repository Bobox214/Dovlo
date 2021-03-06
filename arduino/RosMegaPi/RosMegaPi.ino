//
// Setup encoder for slot 1/2
// Publish encoder states 10Hz
// Subscribe to motorPwm to change motorPwm from -255 to 255

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/UInt32MultiArray.h>
#include <std_msgs/Int16MultiArray.h>

#include <MeMegaPi.h>

MeEncoderOnBoard Encoder1(SLOT1);
MeEncoderOnBoard Encoder2(SLOT2);

class NewHardware : public ArduinoHardware
{
    public:
        NewHardware():ArduinoHardware(&Serial2, 57600){};
};

ros::NodeHandle_<NewHardware>  nh;

std_msgs::UInt32MultiArray encoders_msg;

long publisher_timer; // To handle a 10Hz publishing
long motors_timer;    // To handle stopping motors after 1s without message

void motorsMessageCb( const std_msgs::Int16MultiArray& motors_msg){
    digitalWrite(13, HIGH-digitalRead(13));   // blink the led
    if (motors_msg.data_length==2) {
        Encoder1.setMotorPwm(motors_msg.data[0]);
        Encoder2.setMotorPwm(motors_msg.data[1]);
        motors_timer = millis() + 1000;
        nh.logdebug("motors_pwm applied");
    } else {
        nh.logerror("motors_pwm message received but without correct length");
    }
}

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

ros::Publisher motorsEncoderPub("motors_encoder", &encoders_msg);
ros::Subscriber<std_msgs::Int16MultiArray> motorsPwmSub("motors_pwm", &motorsMessageCb );

void setup()
{
    nh.initNode();
    nh.advertise(motorsEncoderPub);
    nh.subscribe(motorsPwmSub);
    nh.loginfo("Node setup done");

    encoders_msg.data_length = 2;
    encoders_msg.data  = (uint32_t*)malloc( 2 * sizeof(uint32_t));
    publisher_timer = 0;
    motors_timer    = 0;

    attachInterrupt(Encoder1.getIntNum(), interruptEncoder1, RISING);
    attachInterrupt(Encoder2.getIntNum(), interruptEncoder2, RISING);
    
    Encoder1.setMotorPwm(0);
    Encoder1.setPulsePos(0);
    Encoder2.setMotorPwm(0);
    Encoder2.setPulsePos(0);
}

void loop() {
    if (millis()>publisher_timer) {
        encoders_msg.data[0]  = Encoder1.getPulsePos();
        encoders_msg.data[1]  = Encoder2.getPulsePos();
        motorsEncoderPub.publish( &encoders_msg );
        publisher_timer = millis() + 100;
    }
    if (motors_timer!=0 && millis()>motors_timer) {
        Encoder1.setMotorPwm(0);
        Encoder2.setMotorPwm(0);
        motors_timer = 0;
        nh.logdebug("Motors have been stopped after 1s timeout");
    }
    Encoder1.loop();
    Encoder2.loop();
    nh.spinOnce();
}
