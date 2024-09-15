#include <Arduino.h>

const int DAC_PIN = A0;  // DAC output pin
const int SAMPLE_RATE = 100000;  // Sample rate in Hz
const float MY_PI = 3.14159265358979323846;

int frequency = 1000;  // Starting frequency in Hz
int samplesPerWave;  // Number of samples per sine wave cycle
unsigned long lastUpdateTime = 0;  // Time of the last frequency update
unsigned long updateInterval = 2000;  // Interval to update the frequency in milliseconds
int sampleIndex = 0;  // Index of the current sample

void setup() {
  analogWriteResolution(10);  // Set DAC resolution to 10 bits
  samplesPerWave = SAMPLE_RATE / frequency;
}

void loop() {
  unsigned long currentTime = millis();

  // Generate the next sample of the sine wave
  float angle = 2 * MY_PI * sampleIndex / samplesPerWave;
  int value = (sin(angle) + 1) * 511.5;  // Scale sine value to 0-1023 range
  analogWrite(DAC_PIN, value);

  // Move to the next sample
  sampleIndex++;
  if (sampleIndex >= samplesPerWave) {
    sampleIndex = 0;
  }

  // Check if it's time to update the frequency
  if (currentTime - lastUpdateTime >= updateInterval) {
    lastUpdateTime = currentTime;
    frequency += 1000;  // Increase frequency by 1 kHz
    samplesPerWave = SAMPLE_RATE / frequency;  // Update samples per wave cycle
  }

  // Calculate the delay to achieve the desired sample rate
  delayMicroseconds(1000000 / SAMPLE_RATE);
}
