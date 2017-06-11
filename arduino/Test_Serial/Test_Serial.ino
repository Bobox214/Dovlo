// Used to check that serials are working or to find which serial is connected where
// Serial3 usually is bluetooth
// Serial2 usually is Raspberry pi /dev/serial0

void setup() {
    Serial.begin(115200);
    Serial2.begin(115200);
    Serial3.begin(115200);
}

void loop() {
    Serial.println("Serial");
    Serial2.println("Serial2");
    Serial3.println("Serial3");
    delay(1000);
}
