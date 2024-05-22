float StartTime = 0;      //Timer
long loopCounter = 0;     //keeps track of the number of iterations
uint32_t testNumber = 0;  //a 24-bit number as an example 14480912 - 00000000|11011100|11110110|00010000
byte testbuffer[3];       //buffer that stores 3x8=24 bits (3 bytes)

void setup() {
  Serial.begin(2000000);
  delay(1000);  //wait 1 s
  //Serial.println("2000000 test");
}

void loop() {
  if (Serial.available() > 0) {
    char commandCharacter = Serial.read();  //we use characters (letters) for controlling the switch-case
    switch (commandCharacter)               //based on the command character, we decide what to do
    {
      case 'b':                         //binary
        testNumber = 0;                 //test number
        loopCounter = 0;                //counts the iterations of the loop
        StartTime = micros();           //starts the timer
        while (loopCounter < 50000000)  //5M points
        {
          // uint32_t frequency = 500000;              // 500 kHz
          // uint32_t time = loopCounter / frequency;  // Calculate time
          // Calculate sine wave value
          // testNumber = (uint32_t)(sin(2 * PI * frequency * time));
          //-------------------------------------------------------------------
          testbuffer[0] = testNumber & 255;  //first 8 bits - bitwise AND keeps the first 8 digits
          //Serial.print("Buffer 0: ");
          //Serial.println(testbuffer[0]); //print the first part of the output
          //------------------------------------------------
          testbuffer[1] = (testNumber >> 8) & 255;  //8-15 bits
          //shifts the 8-15 bits to the first 8 places (>>8) and keeps only those (&255)
          //Serial.print("Buffer 1: ");
          //Serial.println(testbuffer[1]); //print the second part of the output
          //--------------------------------------------------
          testbuffer[2] = (testNumber >> 16) & 255;  //16-23 bits
          //shifts the 16-23 bits to the first 8 places (>>16) and keeps only those (&255)
          //Serial.print("Buffer 2: ");
          //Serial.println(testbuffer[2]); //print the third part of the output
          //-------------------------------------------
          Serial.write(testbuffer, sizeof(testbuffer));  //dump the buffer to the serial port (24 bit number in 3 bytes)
          loopCounter++;                                 //Increase the value of the counter (+1)
          testNumber = loopCounter;                      //this here might be inefficient, we could directly work with a single variable
          // testNumber = 1123;

          if (micros() - StartTime >= 5000000)  //5 s; If the timer ends before 1M points are transfered, we jump out
          {
            break;  //exit the whole thing
          }
        }
        break;
      //-----------------------------------------------------------------------------------------------------------------
      case 'd':                        //decimal
        loopCounter = 0;               //counts the iterations of the loop
        StartTime = micros();          //starts the timer
        while (loopCounter < 1000000)  //1M points
        {
          Serial.println(loopCounter);  //dump the number to the serial port with a linebreak (one at a time)
          loopCounter++;                //Increase the value of the counter

          if (micros() - StartTime >= 5000000)  //5 s; If the timer ends before 1M points are transfered, we jump out
          {
            Serial.println("Time is over");
            break;  //exit the whole thing
          }
        }
        break;
    }
  }
}