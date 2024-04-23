float StartTime = 0;         // Timer
long loopCounter = 0;        // Keeps track of the number of iterations
uint16_t testNumber = 0;     // A 16-bit number as an example
byte testbuffer[2];          // Buffer that stores 2x8=16 bits (2 bytes)

void setup() {
  Serial.begin(2000000);
  delay(1000);  // Wait 1 s
}

void loop() {
  if (Serial.available() > 0) {
    char commandCharacter = Serial.read();  // Read the command character
    switch (commandCharacter) {
      case 'b':                                          // Binary
        testNumber = 0;                                  // Reset test number
        loopCounter = 0;                                 // Reset loop counter
        StartTime = micros();                            // Start the timer
        while (loopCounter < 50000000) {                 // 5M points for a 10 kHz signal
          float t = loopCounter * 1e-5;                  // Time in seconds
          float sinValue = 127.5 * (1 + sin(20000 * PI * t));  // Generate sine wave
          testbuffer[0] = (uint8_t)sinValue;            // First 8 bits
          testbuffer[1] = (uint8_t)sinValue;            // 8-15 bits (same as first 8 bits)
          Serial.write(testbuffer, sizeof(testbuffer));  // Send the buffer
          loopCounter++;                                 // Increase the loop counter
          if (micros() - StartTime >= 50000000) {         // 1 s timeout
            break;                                       // Exit the loop
          }
        }
        break;
    }
  }
}
