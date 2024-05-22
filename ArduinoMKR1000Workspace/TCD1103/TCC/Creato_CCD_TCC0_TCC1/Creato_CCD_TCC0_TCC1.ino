#define ICG 0
#define SH 1
#define MCLK 2
#define OS 15
#define CLK_DIV 1
#define CLK_ID 4

int t1 = 0;
int t2 = 0;
int t3 = 0;
int t4 = 0;

void set_clk()
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

void pmuxing()
{
  // Enable the port multiplexer for the TCC0 PWM WO[4] (digital pin D0), SAMD21 pin PA22
  PORT->Group[g_APinDescription[ICG].ulPort].PINCFG[g_APinDescription[ICG].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[ICG].ulPort].PMUX[g_APinDescription[ICG].ulPin >> 1].reg |= /*PORT_PMUX_PMUXO_F |*/ PORT_PMUX_PMUXE_F;

      // Enable the port multiplexer for the TCC0 PWM WO[5] (digital pin D1), SAMD21 pin PA23
  PORT->Group[g_APinDescription[SH].ulPort].PINCFG[g_APinDescription[SH].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[SH].ulPort].PMUX[g_APinDescription[SH].ulPin >> 1].reg |= PORT_PMUX_PMUXO_F /*| PORT_PMUX_PMUXE_F */;

        // Enable the port multiplexer for the TCC0 PWM channel 0 (digital pin D2), SAMD21 pin PA10
  PORT->Group[g_APinDescription[MCLK].ulPort].PINCFG[g_APinDescription[MCLK].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[MCLK].ulPort].PMUX[g_APinDescription[MCLK].ulPin >> 1].reg |= PORT_PMUX_PMUXE_E /*| PORT_PMUX_PMUXE_F */;

}

void set_tcc()
{
  // Feed GCLK4 to TCC0 and TCC1
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC0 and TCC1
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TCC0_TCC1;   // Feed GCLK4 to TCC0 and TCC1
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
  
  // Normal (single slope) PWM operation: timer countinuouslys count up to PER register value and then is reset to 0
  REG_TCC0_WAVE |= TCC_WAVE_POL1 | TCC_WAVE_WAVEGEN_NPWM;         // Setup single slope PWM on TCC0
  //REG_TCC0_WAVE |=  TCC_WAVE_WAVEGEN_NPWM;         // Setup single slope PWM on TCC0
  // REG_TCC0_WAVE |= TCC_WAVE_WAVEGEN_NPWM;         // Setup single slope PWM on TCC0
  while (TCC0->SYNCBUSY.bit.WAVE);                // Wait for synchronization

  REG_TCC1_WAVE |= TCC_WAVE_WAVEGEN_NPWM;
  while (TCC1->SYNCBUSY.bit.WAVE);

}

void setup() {
  // put your setup code here, to run once:
  set_clk();
  pmuxing();
  set_tcc();

  REG_TCC0_PER = 317255-1;      // Set the frequency of the PWM on TCC0 to 25kHz
  while(TCC0->SYNCBUSY.bit.PER);

  REG_TCC0_CCB0 = 5064-1;       // TCC0 CCB0 - 50% duty cycle on D0
  //REG_TCC0_CCB0 = (5064*4)-1;       // TCC0 CCB0 - 50% duty cycle on D0
  while(TCC0->SYNCBUSY.bit.CCB0);

  //REG_TCC0_CCB1 = (4800*4)-1;       // TCC0 CCB0 - 50% duty cycle on D1
  REG_TCC0_CCB1 = 4800-1;       // TCC0 CCB0 - 50% duty cycle on D1
  //REG_TCC0_CCB1 = 5064-1;       // TCC0 CCB0 - 50% duty cycle on D1
  while(TCC0->SYNCBUSY.bit.CCB1);

  REG_TCC1_PER = 96-1;
  while(TCC1->SYNCBUSY.bit.PER);

  REG_TCC1_CCB0 = REG_TCC1_PER/2;       // TCC0 CCB0 - 50% duty cycle on D2
  while(TCC1->SYNCBUSY.bit.CCB0);

  // dead time insertion
  // dead time insertion for TCC0

  // REG_TCC0_WEXCTRL |= TCC_WEXCTRL_DTHS(1) | TCC_WEXCTRL_DTIEN0 | TCC_WEXCTRL_DTLS(120) | TCC_WEXCTRL_DTIEN0;
  //REG_TCC0_WEXCTRL |= TCC_WEXCTRL_DTHS(255) | TCC_WEXCTRL_DTLS(255) | TCC_WEXCTRL_DTIEN0 |TCC_WEXCTRL_DTIEN1 ;
  REG_TCC0_WEXCTRL |= TCC_WEXCTRL_DTHS(24) |  TCC_WEXCTRL_DTIEN1 ;
  //REG_TCC0_WEXCTRL |= TCC_WEXCTRL_DTLS(254) |  TCC_WEXCTRL_DTIEN0 ;

  // invert signal
  TCC0->DRVCTRL.reg |= TCC_DRVCTRL_INVEN5; 
  

  // Divide the 48MHz signal by 1 giving 48MHz (20.8ns) TCC0 timer tick and enable the outputs
  REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV1 |    // Divide GCLK4 by 1
                    TCC_CTRLA_ENABLE;             // Enable the TCC0 output
  while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization

    // Divide the 48MHz signal by 1 giving 48MHz (20.8ns) TCC0 timer tick and enable the outputs
  REG_TCC1_CTRLA |= TCC_CTRLA_PRESCALER_DIV1 |    // Divide GCLK4 by 1
                    TCC_CTRLA_ENABLE;             // Enable the TCC0 output
  while (TCC1->SYNCBUSY.bit.ENABLE);              // Wait for synchronization

}

void loop() {
  // put your main code here, to run repeatedly:

}
