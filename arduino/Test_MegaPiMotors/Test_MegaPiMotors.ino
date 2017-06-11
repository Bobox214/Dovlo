#include <MeMegaPi.h>

MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
MeEncoderOnBoard Encoder_3(SLOT3);
MeEncoderOnBoard Encoder_4(SLOT4);
long motorNb = 1;
long motorPwm;

void printState() {
	Serial3.print("Testing Motor ");
	Serial3.print(motorNb);
	Serial3.print(" Target Pwm : ");
	Serial3.print(motorPwm);
	Serial3.print(" Pwm : ");
	if (motorNb==1) Serial3.print(Encoder_1.getCurPwm());
	if (motorNb==2) Serial3.print(Encoder_2.getCurPwm());
	if (motorNb==3) Serial3.print(Encoder_3.getCurPwm());
	if (motorNb==4) Serial3.print(Encoder_4.getCurPwm());
	Serial3.print(" Pos : ");
	if (motorNb==1) Serial3.print(Encoder_1.getPulsePos());
	if (motorNb==2) Serial3.print(Encoder_2.getPulsePos());
	if (motorNb==3) Serial3.print(Encoder_3.getPulsePos());
	if (motorNb==4) Serial3.print(Encoder_4.getPulsePos());
	Serial3.print(" Speed : ");
	if (motorNb==1) Serial3.print(Encoder_1.getCurrentSpeed());
	if (motorNb==2) Serial3.print(Encoder_2.getCurrentSpeed());
	if (motorNb==3) Serial3.print(Encoder_3.getCurrentSpeed());
	if (motorNb==4) Serial3.print(Encoder_4.getCurrentSpeed());
	Serial3.println();
}

void reset() {
	motorPwm = 0;
	Encoder_1.setMotorPwm(0);
    Encoder_1.setPulsePos(0);
	Encoder_2.setMotorPwm(0);
    Encoder_2.setPulsePos(0);
	Encoder_3.setMotorPwm(0);
    Encoder_3.setPulsePos(0);
	Encoder_4.setMotorPwm(0);
    Encoder_4.setPulsePos(0);
}

void printHelp() {
    Serial3.println("Help.");
    Serial3.println("Following commands:");
    Serial3.println("  1,2,3,4 : Choose motor port.");
    Serial3.println("  z : Increase speed.");
    Serial3.println("  s : Decrease speed.");
    Serial3.println("  d : Stop motor, and reset encoder pulse count.");
}

void bluetoothLoop() {
	if (Serial3.available()) {
		int v = Serial3.read();
        if (v=='h')
            printHelp();
		else if (v=='z')
			motorPwm += 5;
		else if (v=='s')
			motorPwm -= 5;
		else if (v=='d')
			motorPwm = 0;
		else if (v=='1') {
			reset();
			motorNb = 1;
		} else if (v=='2') {
			reset();
			motorNb = 2;
		} else if (v=='3') {
			reset();
			motorNb = 3;
		} else if (v=='4') {
			reset();
			motorNb = 4;
		}
		printState();
	}
}

void interruptEncoder1(void) {
    if (digitalRead(Encoder_1.getPortB())==0)
        Encoder_1.pulsePosMinus();
    else
        Encoder_1.pulsePosPlus();
}
void interruptEncoder2(void) {
    if (digitalRead(Encoder_2.getPortB())==0)
        Encoder_2.pulsePosMinus();
    else
        Encoder_2.pulsePosPlus();
}
void interruptEncoder3(void) {
    if (digitalRead(Encoder_3.getPortB())==0)
        Encoder_3.pulsePosMinus();
    else
        Encoder_3.pulsePosPlus();
}
void interruptEncoder4(void) {
    if (digitalRead(Encoder_4.getPortB())==0)
        Encoder_4.pulsePosMinus();
    else
        Encoder_4.pulsePosPlus();
}

void setup() {
    Serial.begin(115200);
    Serial3.begin(115200);
    attachInterrupt(Encoder_1.getIntNum(), interruptEncoder1, RISING);
    attachInterrupt(Encoder_2.getIntNum(), interruptEncoder2, RISING);
    attachInterrupt(Encoder_3.getIntNum(), interruptEncoder3, RISING);
    attachInterrupt(Encoder_4.getIntNum(), interruptEncoder4, RISING);
    reset();
    printState();
}

void loop()
{
	bluetoothLoop();
	if (motorNb==1) Encoder_1.setMotorPwm(motorPwm);
	if (motorNb==2) Encoder_2.setMotorPwm(motorPwm);
	if (motorNb==3) Encoder_3.setMotorPwm(motorPwm);
	if (motorNb==4) Encoder_4.setMotorPwm(motorPwm);
    Encoder_1.loop();
    Encoder_2.loop();
    Encoder_3.loop();
    Encoder_4.loop();
	delay(1);
}
