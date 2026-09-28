#include <Arduino.h>

void setup() {
    Serial.begin(115200);
}

void loop() {
    Serial.println("ATCS_ESP32S3_ONLINE");
    delay(1000);
}
