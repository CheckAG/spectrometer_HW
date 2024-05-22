#define ICG 0  // D0
#define SH 2   // D2
#define MCLK 8 // D8
#define OS 15
#define CLK_ID 4
#define CLK_DIV 1
#define CLK_ID_ADC 3
#define CLK_DIV_ADC 1


int t1 = 0;
int t2 = 0;
int t3 = 0;
int t4 = 0;
volatile bool icgSignalRisingEdge = false;
volatile unsigned long masterClockCycleCount = 0;
volatile int count = 0;
const int16_t NUMBER_OF_PIXEL = 1546;
volatile int aDCVal[NUMBER_OF_PIXEL];

void set_pwm_clk()
{
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(CLK_DIV) |          // Divide the 48MHz clock source by divisor 1: 48MHz/1=48MHz
                    GCLK_GENDIV_ID(CLK_ID);            // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(CLK_ID);          // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

}

void set_adc_clk() {
  // Enable the APBC clock for the ADC
  REG_PM_APBCMASK |= PM_APBCMASK_ADC;
  
  //This allows you to setup a div factor for the selected clock certain clocks allow certain division factors: Generic clock generators 3 - 8 8 division factor bits - DIV[7:0]
  GCLK->GENDIV.reg |= GCLK_GENDIV_ID(CLK_ID_ADC)| GCLK_GENDIV_DIV(CLK_DIV_ADC);
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);  

  //configure the generator of the generic clock with 48MHz clock
  GCLK->GENCTRL.reg |= GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_ID(CLK_ID_ADC); // GCLK_GENCTRL_DIVSEL don't need this, it makes divide based on power of two
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
  
  //enable clock, set gen clock number, and ID to where the clock goes (30 is ADC)
  GCLK->CLKCTRL.reg |= GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(CLK_ID_ADC) | GCLK_CLKCTRL_ID(30);
  while (GCLK->STATUS.bit.SYNCBUSY);
}

void pmuxing()
{
  // Enable the port multiplexer for the TCC0 PWM channel 0 (digital pin D0), SAMD21 pin PA22
  PORT->Group[g_APinDescription[ICG].ulPort].PINCFG[g_APinDescription[ICG].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[ICG].ulPort].PMUX[g_APinDescription[ICG].ulPin >> 1].reg |= /*PORT_PMUX_PMUXO_F |*/ PORT_PMUX_PMUXE_F;

  // Enable the port multiplexer for the TCC0 PWM channel 0 (digital pin D1), SAMD21 pin PA10
  PORT->Group[g_APinDescription[SH].ulPort].PINCFG[g_APinDescription[SH].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[SH].ulPort].PMUX[g_APinDescription[SH].ulPin >> 1].reg |= PORT_PMUX_PMUXE_E | PORT_PMUX_PMUXO_E;

  // Enable the port multiplexer for the TCC0 PWM channel 0 (digital pin D2), SAMD21 pin PA16
  PORT->Group[g_APinDescription[MCLK].ulPort].PINCFG[g_APinDescription[MCLK].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[MCLK].ulPort].PMUX[g_APinDescription[MCLK].ulPin >> 1].reg |= PORT_PMUX_PMUXE_E /*| PORT_PMUX_PMUXE_F */;

}

void set_tcc0_tcc1()
{
  // Feed GCLK4 to TCC0 and TCC1
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC0 and TCC1
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TCC0_TCC1;   // Feed GCLK4 to TCC0 and TCC1
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
  
  // Normal (single slope) PWM operation: timer countinuouslys count up to PER register value and then is reset to 0
  REG_TCC0_WAVE |= TCC_WAVE_POL1 | TCC_WAVE_WAVEGEN_NPWM;         // Setup single slope PWM on TCC0
  // REG_TCC0_WAVE |= TCC_WAVE_WAVEGEN_NPWM;         // Setup single slope PWM on TCC0
  while (TCC0->SYNCBUSY.bit.WAVE);                // Wait for synchronization

  REG_TCC1_WAVE |= TCC_WAVE_POL1 | TCC_WAVE_WAVEGEN_NPWM;
  while (TCC1->SYNCBUSY.bit.WAVE);

}

void set_tcc2()
{
  // Feed GCLK4 to TCC0 and TCC1
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC0 and TCC1
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TCC2_TC3;   // Feed GCLK4 to TCC0 and TCC1
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
  
  // Normal (single slope) PWM operation: timer countinuouslys count up to PER register value and then is reset to 0
  // REG_TCC2_WAVE |= TCC_WAVE_POL1 | TCC_WAVE_WAVEGEN_NPWM;         // Setup single slope PWM on TCC0
  REG_TCC2_WAVE |= TCC_WAVE_WAVEGEN_NPWM;         // Setup single slope PWM on TCC0
  while (TCC2->SYNCBUSY.bit.WAVE);                // Wait for synchronization
}

void aDCSetup() {
  // Select reference
  REG_ADC_REFCTRL = ADC_REFCTRL_REFSEL_INTVCC1; //set vref for ADC to VCC

  // Average control 1 sample, no right-shift
  REG_ADC_AVGCTRL |= ADC_AVGCTRL_SAMPLENUM_1;

  // Sampling time, no extra sampling half clock-cycles
  REG_ADC_SAMPCTRL = ADC_SAMPCTRL_SAMPLEN(0);
  
  // Input control and input scan
  REG_ADC_INPUTCTRL |= ADC_INPUTCTRL_GAIN_1X | ADC_INPUTCTRL_MUXNEG_GND | ADC_INPUTCTRL_MUXPOS_PIN0;
  // Wait for synchronization
  while (REG_ADC_STATUS & ADC_STATUS_SYNCBUSY);

  ADC->CTRLB.reg |= ADC_CTRLB_RESSEL_12BIT | ADC_CTRLB_PRESCALER_DIV32 | ADC_CTRLB_FREERUN; //This is where you set the divide factor, note that the divide call has no effect until you change Arduino wire.c
  //Wait for synchronization
  while (REG_ADC_STATUS & ADC_STATUS_SYNCBUSY);

  ADC->WINCTRL.reg = ADC_WINCTRL_WINMODE_DISABLE; // Disable window monitor mode
  while(ADC->STATUS.bit.SYNCBUSY);

  ADC->EVCTRL.reg |= ADC_EVCTRL_STARTEI; //start ADC when event occurs
  while (ADC->STATUS.bit.SYNCBUSY);

  ADC->CTRLA.reg |= ADC_CTRLA_ENABLE; //set ADC to run in standby
  while (ADC->STATUS.bit.SYNCBUSY);

  // Input pin for ADC Arduino A0/PA02, OS pin
  REG_PORT_DIRCLR1 = PORT_PA02;

  // Enable multiplexing on PA02_AIN0 PA03/ADC_VREFA
  PORT->Group[0].PINCFG[2].bit.PMUXEN = 1;
  PORT->Group[0].PINCFG[3].bit.PMUXEN = 1;
  PORT->Group[0].PMUX[1].reg = PORT_PMUX_PMUXE_B | PORT_PMUX_PMUXO_B;
}

void adcInterrupt(byte priority) {
  
  ADC->INTENSET.reg |= ADC_INTENSET_RESRDY; // enable ADC ready interrupt
  while (ADC->STATUS.bit.SYNCBUSY);

  NVIC_EnableIRQ(ADC_IRQn); // enable ADC interrupts
  NVIC_SetPriority(ADC_IRQn, priority); //set priority of the interrupt
}

//software trigger to start ADC in free run
//in future could use this to set various ADC triggers
void aDCSWTrigger() {
  ADC->SWTRIG.reg |= ADC_SWTRIG_START;
}

// matser clock trigger handler to read pixel on every 2 periods
void TCC2_Handler()
{
  masterClockCycleCount++;
  // Trigger ADC conversion
  if (masterClockCycleCount % 2 == 0){
        ADC->SWTRIG.reg |= ADC_SWTRIG_START;
        // Clear TCC2 interrupt flag
        REG_TCC2_INTENSET |= TCC_INTENCLR_MC0;
  }
  
}

//This ISR is called each time ADC makes a reading
void ADC_Handler() {
    if(count<NUMBER_OF_PIXEL) {
      aDCVal[count] = REG_ADC_RESULT;
      //aDCVal[count] = count;
      count++;
    }
    ADC->INTFLAG.reg = ADC_INTENSET_RESRDY; //Need to reset interrupt
}

// event handler for ICG trigger to SH 
void EVSYS_Handler() {
  if (EVSYS->INTFLAG.bit.EVD0) {
    icgSignalRisingEdge = true;
    if (icgSignalRisingEdge) {
    // Trigger SH signal
    REG_TCC1_COUNT = REG_TCC1_PER;

    icgSignalRisingEdge = false;  // Reset the flag
  }
  EVSYS->INTFLAG.reg = EVSYS_INTENCLR_EVD0;  // Clear the interrupt flag
  }
}

void setup() {
  // put your setup code here, to run once:
  set_pwm_clk();
  set_adc_clk();
  pmuxing();
  set_tcc0_tcc1();
  set_tcc2();
  aDCSetup();
  adcInterrupt(0); //sets up interrupt for ADC and argument assigns priority
  aDCSWTrigger(); //trigger ADC to start free run mode


///////////////////////////////////////////////// setup ICG SH MCLK ////////////////////////////////////////////////////////////////////
// ICG setup
  REG_TCC0_PER = 317255-1;      // Set the frequency of the PWM on TCC0 to 25kHz
  while(TCC0->SYNCBUSY.bit.PER);

  REG_TCC0_CCB0 = 456-1;       // TCC0 CCB0 - 50% duty cycle on D0
  while(TCC0->SYNCBUSY.bit.CCB0);

// MCLK setup
  REG_TCC2_PER = 96-1;
  while(TCC2->SYNCBUSY.bit.PER);

  REG_TCC2_CCB0 = REG_TCC2_PER/2;       // TCC0 CCB0 - 50% duty cycle on D1
  while(TCC2->SYNCBUSY.bit.CCB0);

// SH setup
  // REG_TCC1_PER = 317255-1;   // without shutter
  REG_TCC1_PER = 500-1;         // with shutter
  while(TCC1->SYNCBUSY.bit.PER);

  REG_TCC1_CCB0 = 192-1;       // TCC0 CCB0 - 50% duty cycle on D2
  while(TCC1->SYNCBUSY.bit.CCB0);

  // dead time insertion
  // REG_TCC1_COUNT = REG_TCC1_PER-12;
  // while(TCC1->SYNCBUSY.bit.COUNT);

  // invert SH signal
  TCC1->DRVCTRL.reg |= TCC_DRVCTRL_INVEN0;
  while (TCC1->SYNCBUSY.bit.ENABLE); 

////////////////////////////////////////////////////////////////// event trigger for SH and ADC ///////////////////////////////////////////////////////////////
  // Configure EVSYS for ICG signal event generation to trigger the SH 
  EVSYS->USER.reg = EVSYS_USER_CHANNEL(0x01) | EVSYS_USER_USER(EVSYS_ID_USER_TCC0_EV_0);
  EVSYS->CHANNEL.reg = EVSYS_CHANNEL_EDGSEL_RISING_EDGE | EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_TCC0_OVF) | EVSYS_CHANNEL_PATH_SYNCHRONOUS;
  EVSYS->INTENSET.reg = EVSYS_INTENSET_EVD0;
  // Configure NVIC for EVSYS interrupt
  NVIC_SetPriority(EVSYS_IRQn, 1);  // Set interrupt priority
  NVIC_EnableIRQ(EVSYS_IRQn);       // Enable EVSYS interrupt

  // ADC trigger based on master clock
  REG_TCC2_INTENSET |= TCC_INTENSET_MC0;
  // Enable TCC2 interrupt
  NVIC_EnableIRQ(TCC2_IRQn);

//////////////////////////////////////////////////////////////// enable TCC0, TCC1, TCC2 ////////////////////////////////////////////////////////////////////////////
  // Divide the 48MHz signal by 1 giving 48MHz (20.8ns) TCC0 timer tick and enable the outputs
  REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV1 |    // Divide GCLK4 by 1
                    TCC_CTRLA_ENABLE;             // Enable the TCC0 output

    // Divide the 48MHz signal by 1 giving 48MHz (20.8ns) TCC0 timer tick and enable the outputs
  REG_TCC1_CTRLA |= TCC_CTRLA_PRESCALER_DIV1 |    // Divide GCLK4 by 1
                    TCC_CTRLA_ENABLE;             // Enable the TCC0 output
  while (TCC1->SYNCBUSY.bit.ENABLE);              // Wait for synchronization
  while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization

  REG_TCC2_CTRLA |= TCC_CTRLA_PRESCALER_DIV1 |    // Divide GCLK4 by 1
                    TCC_CTRLA_ENABLE;             // Enable the TCC0 output
  while (TCC2->SYNCBUSY.bit.ENABLE);              // Wait for synchronization

}

void loop() {
  // put your main code here, to run repeatedly:

}
