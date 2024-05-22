void setup() {
   Serial.begin(9600);
}

void loop() {
   uint8_t buf[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 0};
   uint16_t checksum = calculateChecksum(buf, 10);
   Serial.write(buf, 10);
   Serial.write((uint8_t)(checksum & 0xFF)); // Lower byte of checksum
   Serial.write((uint8_t)(checksum >> 8));   // Upper byte of checksum
   delay(1000); // Adjust delay as needed
}

uint16_t calculateChecksum(uint8_t *data, size_t length) {
    uint16_t sum1 = 0;
    uint16_t sum2 = 0;
    for (size_t i = 0; i < length; ++i) {
        sum1 = (sum1 + data[i]) % 255;
        sum2 = (sum2 + sum1) % 255;
    }
    return (sum2 << 8) | sum1;
}
