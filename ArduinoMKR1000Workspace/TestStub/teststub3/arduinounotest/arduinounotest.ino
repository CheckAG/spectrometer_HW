float StartTime = 0;         // Timer
long loopCounter = 0;        // Keeps track of the number of iterations
uint16_t testNumber = 0;     // A 16-bit number as an example
byte testbuffer[2];          // Buffer that stores 2x8=16 bits (2 bytes)
const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to

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
        while (loopCounter < 1000000000) {               // 100M points
          testbuffer[0] = testNumber & 255;              // First 8 bits
          testbuffer[1] = (testNumber >> 8) & 255;       // 8-15 bits
          Serial.write(testbuffer, sizeof(testbuffer));  // Send the buffer
          loopCounter++;                                 // Increase the loop counter
          testNumber = analogRead(analogInPin);          // Update the test number
          if (micros() - StartTime >= 100000000) {       // 100 s timeout
            break;                                       // Exit the loop
          }
        }
        break;
      case 'd':                                   // Decimal
        loopCounter = 0;                          // Reset loop counter
        StartTime = micros();                     // Start the timer
        while (loopCounter < 1000000) {           // 1M points
          Serial.println(loopCounter);            // Send the number
          loopCounter++;                          // Increase the loop counter
          if (micros() - StartTime >= 5000000) {  // 5 s timeout
            Serial.println("Time is over");
            break;  // Exit the loop
          }
        }
        break;
    }
  }
}
