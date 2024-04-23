float StartTime = 0;      // Timer
long loopCounter = 0;     // Keeps track of the number of iterations
uint16_t testNumber = 0;  // A 16-bit number as an example
byte testbuffer[2];       // Buffer that stores 2x8=16 bits (2 bytes)

const int16_t dSize = 1024;   // Number of samples
const byte gClk = 3;          // Generic clock used for ADC
const int cDiv = 2;           // Clock division factor for generic clock
const float period = 1.714;  // Sampling period for 350 kSPS
volatile int aDCVal[dSize];   // Array to hold ADC samples
volatile int count = 0;       // Tracks number of samples collected
bool done = false;            // Tracks whether data has been sent

void setup() {
  portSetup();
  genericClockSetup(gClk, cDiv);
  aDCSetup();
  setUpInterrupt(0);         // Set interrupt priority to 0
  aDCSWTrigger();             // Trigger ADC to start free run mode
  Serial.begin(2000000);     // Start serial communication at 2 Mbps
  delay(1000);               // Wait for serial communication to stabilize
}

void loop() {
  if (count == dSize && !done) {
    for (int y = 0; y < dSize; y++) {
      testbuffer[0] = aDCVal[y] & 255;              // First 8 bits
      testbuffer[1] = (aDCVal[y] >> 8) & 255;       // 8-15 bits
      Serial.write(testbuffer, sizeof(testbuffer)); // Send the buffer
    }
    done = true;
  }
}

void portSetup() {
  // Input pin for ADC Arduino A0/PA02
  REG_PORT_DIRCLR1 = PORT_PA02;

  // Enable multiplexing on PA02_AIN0 PA03/ADC_VREFA
  PORT->Group[0].PINCFG[2].bit.PMUXEN = 1;
  PORT->Group[0].PINCFG[3].bit.PMUXEN = 1;
  PORT->Group[0].PMUX[1].reg = PORT_PMUX_PMUXE_B | PORT_PMUX_PMUXO_B;
}

void genericClockSetup(int clk, int dFactor) {
  REG_PM_APBCMASK |= PM_APBCMASK_ADC;
  GCLK->GENDIV.reg |= GCLK_GENDIV_ID(clk) | GCLK_GENDIV_DIV(dFactor);
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
  GCLK->GENCTRL.reg |= GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_ID(clk);
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
  GCLK->CLKCTRL.reg |= GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(clk) | GCLK_CLKCTRL_ID(30);
  while (GCLK->STATUS.bit.SYNCBUSY);
}

void aDCSetup() {
  REG_ADC_REFCTRL = ADC_REFCTRL_REFSEL_INTVCC1;
  REG_ADC_AVGCTRL |= ADC_AVGCTRL_SAMPLENUM_1;
  REG_ADC_SAMPCTRL = ADC_SAMPCTRL_SAMPLEN(0);
  REG_ADC_INPUTCTRL |= ADC_INPUTCTRL_GAIN_1X | ADC_INPUTCTRL_MUXNEG_GND | ADC_INPUTCTRL_MUXPOS_PIN0;
  while (REG_ADC_STATUS & ADC_STATUS_SYNCBUSY);
  ADC->CTRLB.reg |= ADC_CTRLB_RESSEL_10BIT | ADC_CTRLB_PRESCALER_DIV4 | ADC_CTRLB_FREERUN;
  while (REG_ADC_STATUS & ADC_STATUS_SYNCBUSY);
  ADC->WINCTRL.reg = ADC_WINCTRL_WINMODE_DISABLE;
  while(ADC->STATUS.bit.SYNCBUSY);
  ADC->EVCTRL.reg |= ADC_EVCTRL_STARTEI;
  while (ADC->STATUS.bit.SYNCBUSY);
  ADC->CTRLA.reg |= ADC_CTRLA_ENABLE;
  while (ADC->STATUS.bit.SYNCBUSY);
}

void setUpInterrupt(byte priority) {
  ADC->INTENSET.reg |= ADC_INTENSET_RESRDY;
  while (ADC->STATUS.bit.SYNCBUSY);
  NVIC_EnableIRQ(ADC_IRQn);
  NVIC_SetPriority(ADC_IRQn, priority);
}

void aDCSWTrigger() {
  ADC->SWTRIG.reg |= ADC_SWTRIG_START;
}

void ADC_Handler() {
  if (count < dSize) {
    aDCVal[count] = REG_ADC_RESULT;
    count++;
  }
  ADC->INTFLAG.reg = ADC_INTENSET_RESRDY;
}
