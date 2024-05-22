void setup() {
  // put your setup code here, to run once:
 REG_GCLK_GENDIV = GCLK_GENDIV_DIV(1) |          // Divide the 48MHz clock source by divisor 1: 48MHz/1=48MHz
                    GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4);          // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Enable the port multiplexer for the TCC0 PWM channel 2 (digital pin D6), SAMD21 pin PA20
  PORT->Group[g_APinDescription[6].ulPort].PINCFG[g_APinDescription[6].ulPin].bit.PMUXEN = 1;

    // Enable the port multiplexer for the TCC0 PWM channel 0 (digital pin D0), SAMD21 pin PA22
  PORT->Group[g_APinDescription[0].ulPort].PINCFG[g_APinDescription[0].ulPin].bit.PMUXEN = 1;

      // Enable the port multiplexer for the TCC0 PWM channel 0 (digital pin D1), SAMD21 pin PA23
  PORT->Group[g_APinDescription[1].ulPort].PINCFG[g_APinDescription[1].ulPin].bit.PMUXEN = 1;
  
  // Connect the TCC0 timer to the port outputs - port pins are paired odd PMUO and even PMUXE
  // F & E specify the timers: TCC0, TCC1 and TCC2
  PORT->Group[g_APinDescription[6].ulPort].PMUX[g_APinDescription[6].ulPin >> 1].reg |= /*PORT_PMUX_PMUXO_F |*/ PORT_PMUX_PMUXE_F;

  PORT->Group[g_APinDescription[0].ulPort].PMUX[g_APinDescription[0].ulPin >> 1].reg |= /*PORT_PMUX_PMUXO_F |*/ PORT_PMUX_PMUXE_F;

  PORT->Group[g_APinDescription[1].ulPort].PMUX[g_APinDescription[1].ulPin >> 1].reg |= PORT_PMUX_PMUXO_F /*| PORT_PMUX_PMUXE_F */;

  // Feed GCLK4 to TCC0 and TCC1
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC0 and TCC1
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TCC0_TCC1;   // Feed GCLK4 to TCC0 and TCC1
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Normal (single slope) PWM operation: timer countinuouslys count up to PER register value and then is reset to 0
  REG_TCC0_WAVE |= TCC_WAVE_WAVEGEN_NPWM;         // Setup single slope PWM on TCC0
  while (TCC0->SYNCBUSY.bit.WAVE);                // Wait for synchronization

  REG_TCC1_WAVE |= TCC_WAVE_WAVEGEN_NPWM;
  while (TCC0->SYNCBUSY.bit.WAVE);  

  REG_TCC0_WAVE |= TCC_WAVE_POL1;         // Setup single slope PWM on TCC0
  while (TCC0->SYNCBUSY.bit.WAVE);                // Wait for synchronization
  // Each timer counts up to a maximum or TOP value set by the PER (period) register,
  // this determines the frequency of the PWM operation:
  // 1919 = 25kHz
  REG_TCC0_PER = 1919;      // Set the frequency of the PWM on TCC0 to 25kHz
  //REG_TCC0_PER = 24;      // Set the frequency of the PWM on TCC0 to 25kHz
  while(TCC0->SYNCBUSY.bit.PER);

  // The CCBx register value determines the duty cycle
  REG_TCC0_CCB2 = 959;       // TCC0 CCB2 - 50% duty cycle on D6
  //REG_TCC0_CCB2 = 12;       // TCC0 CCB2 - 50% duty cycle on D6
    while(TCC0->SYNCBUSY.bit.CCB2);

  REG_TCC0_CCB0 = 959/2;       // TCC0 CCB0 - 50% duty cycle on D0
  while(TCC0->SYNCBUSY.bit.CCB0);

  REG_TCC0_CCB1 = 959/2;       // TCC0 CCB0 - 50% duty cycle on D0
  while(TCC0->SYNCBUSY.bit.CCB1);

  
  //REG_TCC0_WEXCTRL |= TCC_WEXCTRL_DTLS(500) | TCC_WEXCTRL_DTIEN1;
  //REG_TCC0_WEXCTRL |= TCC_WEXCTRL_DTHS(500) | TCC_WEXCTRL_DTIEN1 ;
  REG_TCC0_WEXCTRL |= TCC_WEXCTRL_DTHS(500) | TCC_WEXCTRL_DTIEN1 | TCC_WEXCTRL_DTLS(500) | TCC_WEXCTRL_DTIEN0;
  //REG_TCC0_WEXCTRL |= TCC_WEXCTRL_DTLS(100) | TCC_WEXCTRL_DTIEN0;
  
  // Divide the 48MHz signal by 1 giving 48MHz (20.8ns) TCC0 timer tick and enable the outputs
  REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV1 |    // Divide GCLK4 by 1
                    TCC_CTRLA_ENABLE;             // Enable the TCC0 output
  while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization
}

void loop() {
  // put your main code here, to run repeatedly:

}
