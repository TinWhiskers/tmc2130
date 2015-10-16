//****************************************************************************
//* (c) 2015 Tin Whiskers Technology.  All rights reserved.
//****************************************************************************
#include <SPI.h>

#include "tmc2130.h"
//#define SIMULATION 1

//#define TMC_DEBUG



Tmc2130::Tmc2130(uint8_t chipSelectPin)
{
  csPin = chipSelectPin;
}

void Tmc2130::begin(boolean initSpi)
{
  pinMode(csPin,OUTPUT);
  digitalWrite(csPin,HIGH);
  if (initSpi)
  {
    SPI.begin();
    //CPOL=0, CPHA=0.  MSB first, per datasheet.  Assuming fsck = 8 MHz
    SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  }

  //default_config();
}

void Tmc2130::default_config()
{
  // GCONF 0x00
  // COMMENTARY: PWM_MODE seems to be better if set to TRUE than false.
  set_en_pwm_mode(true); //Disable en_pwm_mode (StealthChop)
  
  // IHOLD_RUN 0x10
  ihold_run_t ihold_run;
  ihold_run.ihold      = 11;
  ihold_run.irun       = 31;
  ihold_run.iholddelay = 8;
  set_ihold_irun(ihold_run);

  // TPOWERDOWN 0x11
  // COMMENTARY: Datasheet set to 10 (fraction of a second).  I set it to ~2-3 seconds.
  set_tpowerdown(128); // Delay before power down in stand still

  // TPWMTHRS 0x13
  //
  set_tpwmthrs(500); // yields a switching velocity about 35000 = ca. 30RPM
 
  // TCOOLTHRS 0x14
  set_tcoolthrs(0);
  
  // THIGH 0x15
  set_thigh(0);

  // CHOPCONF 0x6C
  // spi_write(CHOPCONF,   0x000100C5); // CHOPCONF: TOFF=5, HSTRT=4, HEND=1, TBL=2, CHM=0 (spreadCycle)
  //000100C5 //default
  //140080C4 //before, when it didn't work
  //140100C5 //now (16x microstepping, )
  //000100C5 // this works
  //040100C5 // This doesn't work.. 16x microstepping enabled
  chopconf_t chopconf;
  //chopconf.chm      = true;  // Chopper mode.  true=classic constant off time (don't use this!)
  chopconf.chm      = false;  // Chopper mode.  false=Standard mode (SpreadCycle)
  chopconf.toff     = 5;      // 0=Driver off. General enable for stepper motor driver.
  chopconf.hstrt    = 4;      // Default value is '4'
  chopconf.hend     = -2;     // Default value is '1' raw, which equals -2 
  chopconf.fd3      = false;  // TBD
  chopconf.disfdcc  = false;  // TBD
  chopconf.rndtf    = false;  // Chopper off time is fixed as set by TOFF
  chopconf.tbl      = TBL_36; // TBD.. TBL_24 or TBL_36.
  chopconf.vsense   = false;  // Low sensitivity, high sense resistor voltage
//  chopconf.vsense   = true;
  chopconf.vhighfs  = false;  // TBD
  chopconf.vhighchm = false;  // TBD
  chopconf.sync     = 0;      // TBD
  chopconf.mres     = MRES_256; //16x microstepping
  chopconf.intpol   = false;  // MicroPlyer interpolation on/off
  chopconf.dedge    = false;  // T=Enable step impulse at each step edge to reduce step frequency requirement.
  chopconf.diss2g   = false;  // TBD
  set_chopconf(chopconf);

  // COOLCONF 0x6D
  coolconf_t coolconf;
  coolconf.semin    = 0;      // 0=coolStep off.  Effectively disables everything below.
  coolconf.seup     = SEUP_1; // current up step width
  coolconf.semax    = 15;      // TBD 0..15.  stallGuard2 hysteresis value for smart current control
  coolconf.sedn     = SEDN_1; // current down step width
  coolconf.seimin   = false;  // TBD TRUE=1/4 of current setting (IRUN), FALSE=1/2 of current setting (IRUN)
  coolconf.sgt      = 0;      // TBD Signed, -64 to 63
  coolconf.sfilt    = false;  // TBD  TRUE = stallGuard2 filter enable
  set_coolconf(coolconf);

  // PWMCONF 0x70
  spi_write(PWMCONF,   0x000401C8); // PWM_CONF: AUTO=1, 1/1024 Fclk, Switch amplitude limit=200, Grad=1
  // pwmconf_t pwmconf;
  // pwmconf.pwm_ampl      = 200;   // User defined amplitude (offset)
  // pwmconf.pwm_grad      = 1;     // User defined amplitude (gradient) or regulation loop gradient
  // pwmconf.pwm_freq      = PWMFREQ_1_1024; // PWM frequency selection (410/512/683/1024)
  // pwmconf.pwm_autoscale = true;  // PWM automatic amplitude scaling
  // pwmconf.pwm_symmetric = false; // Force symmetric PWM
  // pwmconf.freewheel     = FREEWHEEL_NORMAL; // Allows different standstill modes
  // set_pwmconf(pwmconf);

  //This sets all of the sine lookup tables
  set_mslut();

#ifdef TMC_DEBUG
  ioin_t ioin;
  ioin = get_ioin();
  if (ioin.version != 0x11)
  {
    Serial.print("Error: Unexpected IOIN version#: Expected 0x11.  Received ");
    Serial.println(ioin.version);
  }
#endif
}

void Tmc2130::print_regs()
{ 
#ifdef TMC_DEBUG
  Serial.println("Reg  Name        Data      ");
  Serial.println("---------------------------");
  //              0x00 GCONF       0x00000000
  printReg(GCONF,     "GCONF",     spi_read(GCONF) );
  printReg(GSTAT,     "GSTAT",     spi_read(GCONF) );
  printReg(IOIN,      "IOIN",      spi_read(GCONF) );
  // printReg(IHOLD_RUN, "IHOLD_RUN", reg_ihold_run   );
  // printReg(TPOWERDOWN,"TPOWERDOWN",reg_tpowerdown );
  printReg(TSTEP,     "TSTEP",     spi_read(TSTEP) );
  // printReg(TPWMTHRS,  "TPWMTHRS",  reg_tpwmthrs );
  // printReg(TCOOLTHRS, "TCOOLTHRS", reg_tcoolthrs );
  // printReg(THIGH,     "THIGH",     reg_thigh );
  // printReg(VDCMIN,    "VDCMIN",    reg_vdcmin );
  // printReg(MSLUT0,    "MSLUT0",    reg_mslut0 );
  // printReg(MSLUT1,    "MSLUT1",    reg_mslut1 );
  // printReg(MSLUT2,    "MSLUT2",    reg_mslut2 );
  // printReg(MSLUT3,    "MSLUT3",    reg_mslut3 );
  // printReg(MSLUT4,    "MSLUT4",    reg_mslut4 );
  // printReg(MSLUT5,    "MSLUT5",    reg_mslut5 );
  // printReg(MSLUT6,    "MSLUT6",    reg_mslut6 );
  // printReg(MSLUT7,    "MSLUT7",    reg_mslut7 );
  // printReg(MSLUTSEL,  "MSLUTSEL",  reg_mslutsel );
  // printReg(MSLUTSTART,"MSLUTSTART",reg_mslutstart );
  printReg(MSCNT,     "MSCNT",     spi_read(MSCNT) );
  printReg(MSCURACT,  "MSCURACT",  spi_read(MSCURACT) );
  printReg(CHOPCONF,  "CHOPCONF",  spi_read(CHOPCONF) );
  // printReg(COOLCONF,  "COOLCONF",  reg_coolconf );
  // printReg(DCCTRL,    "DCCTRL",    reg_dcctrl );
  printReg(DRV_STATUS,"DRV_STATUS",spi_read(DRV_STATUS) );
  // printReg(PWMCONF,   "PWMCONF",   reg_pwmconf );
  printReg(PWM_SCALE, "PWM_SCALE", spi_read(PWM_SCALE) );
  printReg(LOST_STEPS,"LOST_STEPS",spi_read(LOST_STEPS) );
  Serial.println("");
#endif
}

void Tmc2130::printReg(uint8_t reg, char description[], uint32_t data)
{
#ifdef TMC_DEBUG
  char tmp[30]; // for sprintf()
  sprintf(tmp, "0x%.2X %-11s 0x%.8lX", reg, description, data);
  Serial.println(tmp);
#endif
}

//**************************************************************************
// GENERAL CONFIGURATION REGISTERS (0x00..0x0F)
//**************************************************************************

void Tmc2130::set_small_hysteresis(boolean sh)
{
  uint32_t data;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 33222222222211111111110000000000
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 10987654321098765432109876543210
  data |= (((uint32_t)sh                       << 0)  & 0b00000000000000000100000000000000); // 0..3
  data |= (((uint32_t)en_pwm_mode              << 8)  & 0b00000000000000000000000000000100); // 8..12
  spi_write(GCONF, data);

  small_hysteresis = sh;
}


void Tmc2130::set_en_pwm_mode(boolean stealthchopEn)
{
  uint32_t data;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 33222222222211111111110000000000
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 10987654321098765432109876543210
  data |= (((uint32_t)small_hysteresis         << 0)  & 0b00000000000000000100000000000000); // 0..3
  data |= (((uint32_t)stealthChopEn            << 8)  & 0b00000000000000000000000000000100); // 8..12
  spi_write(GCONF, data);

  en_pwm_mode = stealthChopEn;
}

uint32_t Tmc2130::get_gstat_raw()
{
  uint32_t response = spi_read(GSTAT);
  return response;
}

boolean Tmc2130::get_gstat_uv_cp() // GSTAT NOTE: Already get [0] and [1] in the spi status byte.. no need to get them twice!
{
  uint32_t response = spi_read(GSTAT);
  return (boolean)((response >> 2) & 0x01);
}

ioin_t Tmc2130::get_ioin()
{
  uint32_t response = spi_read(IOIN);
  ioin_t ioin;
  
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 33222222222211111111110000000000
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 10987654321098765432109876543210
  ioin.step             = (boolean) ((response      ) & 0b00000000000000000000000000000001); // 
  ioin.direct_mode      = (boolean) ((response >>  1) & 0b00000000000000000000000000000001); // 
  ioin.dcen_cfg4        = (boolean) ((response >>  2) & 0b00000000000000000000000000000001); // 
  ioin.dcin_cfg5        = (boolean) ((response >>  3) & 0b00000000000000000000000000000001); // 
  ioin.drv_enn_cfg6     = (boolean) ((response >>  4) & 0b00000000000000000000000000000001); // 
  ioin.dco              = (boolean) ((response >>  5) & 0b00000000000000000000000000000001); // 
  ioin.version          = (uint8_t) ((response >> 24) & 0b00000000000000000000000011111111); // 
  return ioin;
}

uint8_t Tmc2130::get_version()
{
  uint32_t response = spi_read(IOIN);
  
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 33222222222211111111110000000000
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 10987654321098765432109876543210
  return                  (uint8_t) ((response >> 24) & 0b00000000000000000000000011111111); // 
}


//**************************************************************************
// VELOCITY DEPENDENT DRIVER FEATURE CONTROL REGISTER SET (0x10..0x1F)
//**************************************************************************

void Tmc2130::set_ihold_irun(ihold_run_t iholdrun)
{
  uint32_t data = 0;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 33222222222211111111110000000000
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 10987654321098765432109876543210
  data |= (((uint32_t)ihold_run.ihold          << 0)  & 0b00000000000000000000000000001111); // 0..3
  data |= (((uint32_t)ihold_run.irun           << 8)  & 0b00000000000000000001111100000000); // 8..12
  data |= (((uint32_t)ihold_run.iholddelay     << 16) & 0b00000000000011110000000000000000); // 16..19
  spi_write(IHOLD_RUN, data);
  ihold_run = iholdrun; //Whenever this function is called, update the private var
}

void Tmc2130::set_ihold(uint8_t ihold)
{
  ihold_run.ihold = ihold;
  set_ihold_irun(ihold_run);
}
void Tmc2130::set_irun(uint8_t irun)
{
  ihold_run.irun = irun;
  set_ihold_irun(ihold_run);
}

void Tmc2130::set_iholddelay(  uint8_t iholddelay)
{
  ihold_run.iholddelay = iholddelay;
  set_ihold_irun(ihold_run);
}

void Tmc2130::set_tpowerdown(uint8_t tpowerdown)
{
  uint32_t data;
  data = tpowerdown & 0b11111111;              // 8-bit field
  spi_write(TPOWERDOWN, data);
  //reg_tpowerdown = data;
}

void Tmc2130::set_tpwmthrs(uint32_t tpwmthrs)
{
  uint32_t data;
  data = tpwmthrs & 0b11111111111111111111;    // 20-bit field
  spi_write(TPWMTHRS, data);
  //reg_tpwmthrs = data;
}

void Tmc2130::set_tcoolthrs(uint32_t tcoolthrs)
{
  uint32_t data;
  data = tcoolthrs & 0b11111111111111111111;    // 20-bit field
  spi_write(TCOOLTHRS, data);
  //reg_tcoolthrs = data;
}

void Tmc2130::set_thigh(uint32_t thigh)
{
  uint32_t data;
  data = thigh & 0b11111111111111111111;    // 20-bit field
  spi_write(THIGH, data);
  //reg_thigh = data;
}

uint32_t Tmc2130::get_tstep()      // TSTEP Actual measured time between two 1/256 microsteps derived from the step input frequency in units of 1/fCLK.
{
  uint32_t response = spi_read(TSTEP);
  return (response & 0b11111111111111111111);
}

//**************************************************************************
// MICROSTEPPING CONTROL REGISTER SET (0x60..0x6B)
//**************************************************************************
void Tmc2130::set_mslut()
{
  // NOTE: Currently this just sets the registers up for default values.  At some point these might be made 
  // configurable if there is a need to do so, but for now the default values should work just fine.
  spi_write(MSLUT0,     0xAAAAB554); // 0b10101010101010101011010101010100 = 0xAAAAB554 
  spi_write(MSLUT1,     0x4A9554AA); // 0b01001010100101010101010010101010 = 0x4A9554AA 
  spi_write(MSLUT2,     0x24492929); // 0b00100100010010010010100100101001 = 0x24492929 
  spi_write(MSLUT3,     0x10104222); // 0b00010000000100000100001000100010 = 0x10104222 
  spi_write(MSLUT4,     0xFBFFFFFF); // 0b11111011111111111111111111111111 = 0xFBFFFFFF 
  spi_write(MSLUT5,     0xB5BB777D); // 0b10110101101110110111011101111101 = 0xB5BB777D 
  spi_write(MSLUT6,     0x49295556); // 0b01001001001010010101010101010110 = 0x49295556 
  spi_write(MSLUT7,     0x00404222); // 0b00000000010000000100001000100010 = 0x00404222 

  //These given values work pretty well so we will use them.
//  spi_write(MSLUTSEL,   0x80FFFF56); // 0b11111111111111111000000001010110 = 0xFFFF8056: 
  spi_write(MSLUTSEL,   0xFFFF8056); // 0b11111111111111111000000001010110 = 0xFFFF8056: 
  spi_write(MSLUTSTART, 0x00F70000); // 0b00000000111101110000000000000000 = 0x00F70000: 
}

void Tmc2130::set_mslutsel(mslutsel_t mslutsel)
{
  uint32_t data = 0;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 33222222222211111111110000000000
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 10987654321098765432109876543210
  data |= (((uint32_t)mslutsel.w0              << 0)  & 0b00000000000000000000000000000011); // 0..1
  data |= (((uint32_t)mslutsel.w1              << 2)  & 0b00000000000000000000000000001100); // 2..3
  data |= (((uint32_t)mslutsel.w2              << 4)  & 0b00000000000000000000000000110000); // 4..5
  data |= (((uint32_t)mslutsel.w3              << 6)  & 0b00000000000000000000000011000000); // 6..7
  data |= (((uint32_t)mslutsel.x1              << 8)  & 0b00000000000000001111111100000000); // 8..15
  data |= (((uint32_t)mslutsel.x2              << 16) & 0b00000000111111110000000000000000); // 16..23
  data |= (((uint32_t)mslutsel.x3              << 24) & 0b11111111000000000000000000000000); // 24..31
  spi_write(MSLUTSEL, data);
  //reg_mslutsel = data;
}

void Tmc2130::set_mslutstart(mslutstart_t mslutstart)
{
  uint32_t data = 0;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 33222222222211111111110000000000
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 10987654321098765432109876543210
  data |= (((uint32_t)mslutstart.start_sin     << 0)  & 0b00000000000000000000000011111111); // 0..7
  data |= (((uint32_t)mslutstart.start_sin90   << 16) & 0b00000000111111110000000000000000); // 23..16
  spi_write(MSLUTSTART, data);
  //reg_mslutstart = data;
}

uint16_t Tmc2130::get_mscnt()      // MSCNT register.  
{
  uint32_t response = spi_read(MSCNT);
  return (response & 0b1111111111);
}

mscuract_t Tmc2130::get_mscuract() // actual current_a and current_b read from MSLUT, not scaled
{
  mscuract_t mscuract;
  uint32_t response = spi_read(MSCURACT);
  mscuract.cur_a = (((uint8_t)response      ) & 0b111111111);
  mscuract.cur_b = (((uint8_t)response >> 16) & 0b111111111);
  return mscuract;
}

uint32_t Tmc2130::get_mscuract_raw()
{
  uint32_t response = spi_read(MSCURACT);
  return response;
}

//**************************************************************************
// DRIVER REGISTER SET (0X6C…0X7F)
//**************************************************************************

void Tmc2130::set_chopconf(chopconf_t chopconf)
{
  uint32_t data = 0;
  chopconf.hend += 3; //Change from {-3,-2,-1,0,1,...,12} to {0..15}


  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 33222222222211111111110000000000
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 10987654321098765432109876543210
  data |= (((uint32_t)chopconf.toff            << 0)  & 0b00000000000000000000000000001111); // 0..3
  data |= (((uint32_t)chopconf.hstrt           << 4)  & 0b00000000000000000000000001110000); // 4..6
  data |= (((uint32_t)chopconf.hend            << 7)  & 0b00000000000000000000011110000000); // 7..10
  data |= (((uint32_t)chopconf.fd3             << 11) & 0b00000000000000000000100000000000); // 11
  data |= (((uint32_t)chopconf.disfdcc         << 12) & 0b00000000000000000001000000000000); // 12
  data |= (((uint32_t)chopconf.rndtf           << 13) & 0b00000000000000000010000000000000); // 13
  data |= (((uint32_t)chopconf.chm             << 14) & 0b00000000000000000100000000000000); // 14
  data |= (((uint32_t)chopconf.tbl             << 15) & 0b00000000000000011000000000000000); // 15..16
  data |= (((uint32_t)chopconf.vsense          << 17) & 0b00000000000000100000000000000000); // 17
  data |= (((uint32_t)chopconf.vhighfs         << 18) & 0b00000000000001000000000000000000); // 18
  data |= (((uint32_t)chopconf.vhighchm        << 19) & 0b00000000000010000000000000000000); // 19
  data |= (((uint32_t)chopconf.sync            << 20) & 0b00000000111100000000000000000000); // 20..23
  data |= (((uint32_t)chopconf.mres            << 24) & 0b00001111000000000000000000000000); // 24..27
  data |= (((uint32_t)chopconf.intpol          << 28) & 0b00010000000000000000000000000000); // 28
  data |= (((uint32_t)chopconf.dedge           << 29) & 0b00100000000000000000000000000000); // 29
  data |= (((uint32_t)chopconf.diss2g          << 30) & 0b01000000000000000000000000000000); // 30
  spi_write(CHOPCONF, data);
  // Serial.print("CHOPCONF: ");
  // Serial.println(data,HEX);
}

void Tmc2130::set_chopconf_raw(uint32_t raw_word)
{
  spi_write(CHOPCONF, raw_word);
}


void Tmc2130::set_coolconf(coolconf_t coolconf)
{
  uint32_t data = 0;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 33222222222211111111110000000000
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 10987654321098765432109876543210
  data |= (((uint32_t)coolconf.semin           <<  0) & 0b00000000000000000000000000001111); // 0..3
  data |= (((uint32_t)coolconf.seup            <<  5) & 0b00000000000000000000000001100000); // 5..6
  data |= (((uint32_t)coolconf.semax           <<  8) & 0b00000000000000000000111100000000); // 8..11
  data |= (((uint32_t)coolconf.sedn            << 13) & 0b00000000000000000110000000000000); // 13..14
  data |= (((uint32_t)coolconf.seimin          << 15) & 0b00000000000000001000000000000000); // 15
  data |= (((uint32_t)coolconf.sgt             << 16) & 0b00000000011111110000000000000000); // 16..22
  data |= (((uint32_t)coolconf.sfilt           << 24) & 0b00000000000000000000000000000000); // 24
  spi_write(COOLCONF, data);
  //reg_coolconf = data;
}

void Tmc2130::set_dcctrl(uint8_t dc_time, uint8_t dc_sg)
{
  uint32_t data = 0;
  data |= (((uint32_t)dc_time)     & 0b1111111111);
  data |= (((uint32_t)dc_sg << 16) & 0b11111111);
  spi_write(DCCTRL, data);
  //reg_dcctrl = data;
}

void Tmc2130::set_pwmconf(pwmconf_t pwmconf)
{
  uint32_t data = 0;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 33222222222211111111110000000000
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 10987654321098765432109876543210
  data |= (((uint32_t)pwmconf.pwm_ampl         << 0)  & 0b00000000000000000000000011111111); // 0..7
  data |= (((uint32_t)pwmconf.pwm_grad         << 8)  & 0b00000000000000001111111100000000); // 8..15
  data |= (((uint32_t)pwmconf.pwm_freq         << 16) & 0b00000000000000110000000000000000); // 16..17
  data |= (((uint32_t)pwmconf.pwm_autoscale    << 18) & 0b00000000000001000000000000000000); // 18
  data |= (((uint32_t)pwmconf.pwm_symmetric    << 19) & 0b00000000000010000000000000000000); // 19
  data |= (((uint32_t)pwmconf.freewheel        << 20) & 0b00000000001100000000000000000000); // 20..21
  spi_write(PWMCONF, data);
  //reg_pwmconf = data;
}

void Tmc2130::set_pwmconf_raw(uint32_t raw_word)
{
  spi_write(PWMCONF, raw_word);
}

// DRV_STATUS : 0x6F : stallGuard2 Value and Driver Error Flags
// Datasheet page 37
drvst_t  Tmc2130::get_drv_status() // stallGuard2 value and driver error flags
{
  uint32_t response = spi_read(DRV_STATUS);
  drvst_t drv_status;
  
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 33222222222211111111110000000000
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 10987654321098765432109876543210
  drv_status.sg_result  = (uint16_t)((response      ) & 0b00000000000000000000000111111111); // 
  drv_status.fsactive   = (boolean) ((response >> 15) & 0b00000000000000000000000000000001); // 
  drv_status.cs_actual  = (uint8_t) ((response >> 16) & 0b00000000000000000000000000011111); // 
  drv_status.stallguard = (boolean) ((response >> 24) & 0b00000000000000000000000000000001); // 
  drv_status.ot         = (boolean) ((response >> 25) & 0b00000000000000000000000000000001); // 
  drv_status.otpw       = (boolean) ((response >> 26) & 0b00000000000000000000000000000001); // 
  drv_status.s2ga       = (boolean) ((response >> 27) & 0b00000000000000000000000000000001); // 
  drv_status.s2gb       = (boolean) ((response >> 28) & 0b00000000000000000000000000000001); // 
  drv_status.ola        = (boolean) ((response >> 29) & 0b00000000000000000000000000000001); // 
  drv_status.olb        = (boolean) ((response >> 30) & 0b00000000000000000000000000000001); // 
  drv_status.stst       = (boolean) ((response >> 31) & 0b00000000000000000000000000000001); // 

  return drv_status;
}

uint32_t Tmc2130::get_drv_status_raw()
{
  uint32_t response = spi_read(DRV_STATUS);
  return response;


void Tmc2130::print_drv_status()
{
#ifdef TMC_DEBUG
  drvst_t drv_status = get_drv_status();
  Serial.print("drv_status.sg_result: "); Serial.println(drv_status.sg_result,HEX);
  Serial.print("drv_status.fsactive:  "); Serial.println(drv_status.fsactive,HEX);
  Serial.print("drv_status.cs_actual: "); Serial.println(drv_status.cs_actual,HEX);
  Serial.print("drv_status.stallguard:"); Serial.println(drv_status.stallguard,HEX);
  Serial.print("drv_status.ot:        "); Serial.println(drv_status.ot,HEX);
  Serial.print("drv_status.otpw:      "); Serial.println(drv_status.otpw,HEX);
  Serial.print("drv_status.s2ga:      "); Serial.println(drv_status.s2ga,HEX);
  Serial.print("drv_status.s2gb:      "); Serial.println(drv_status.s2gb,HEX);
  Serial.print("drv_status.ola:       "); Serial.println(drv_status.ola,HEX);
  Serial.print("drv_status.olb:       "); Serial.println(drv_status.olb,HEX);
  Serial.print("drv_status.stst:      "); Serial.println(drv_status.stst,HEX);
#endif
}

// NOTE: This function is very similar to the above function, but is optimized for when only the
//    sg_result field is needed.  It uses less memory and less CPU cycles
uint16_t Tmc2130::get_loadmeas()
{
  uint32_t response = spi_read(DRV_STATUS);
  return  (uint16_t)(response & 0b00000000000000000000000111111111);
}

uint8_t  Tmc2130::get_pwm_scale()  // Actual PWM amplitude scaler (255=max. Voltage)
{
  uint32_t response = spi_read(PWM_SCALE);
  return (uint8_t)(response & 0xFF);
}


//**************************************************************************
// LOW-LEVEL SPI FUNCTIONS
//**************************************************************************

//          w | 38..32  |  31..24  |  23..16  |  15..8   |   7..0   |
//         ----------------------------------------------------------
// master   1 | aaaaaaa | dddddddd | dddddddd | dddddddd | dddddddd | 
// response 1 | status  | data ??                                   |
void Tmc2130::spi_write(uint8_t reg, uint32_t data)
{
 #ifndef SIMULATION
  digitalWrite(csPin, LOW);
  SPI.transfer( reg | 0x80 ); //Send register location.  Bit 39 of SPI datagram should be '1' for write access
  SPI.transfer( (uint8_t)( (data>>24) & 0xFF) );  //31..24
  SPI.transfer( (uint8_t)( (data>>16) & 0xFF) );  //23..16
  SPI.transfer( (uint8_t)( (data>> 8) & 0xFF) );  //15..8
  SPI.transfer( (uint8_t)( (data    ) & 0xFF) );  //7..0
  digitalWrite(csPin, HIGH);
#else
#ifdef TMC_DEBUG
  Serial.print("Reg: 0x");
  Serial.print(reg,HEX);
  Serial.print(" : Data: 0b");
  Serial.println(data,HEX);
#endif // TMC_DEBUG
#endif
}

//          w | 38..32  |  31..24  |  23..16  |  15..8   |   7..0   |
//         ----------------------------------------------------------
// master   0 | address | 00000000 | 00000000 | 00000000 | 00000000 | (dummy data)
// response 0 | status  | MSB -- data --                        lsb |
uint32_t Tmc2130::spi_read(uint8_t reg)
{
  uint8_t status = 0;
  uint32_t data  = 0;

#ifndef SIMULATION
  //Bit 39 of SPI datagram should be '0' for read access
  digitalWrite(csPin, LOW);
  data = SPI.transfer(reg & 0x7F); //Send register location
  data = SPI.transfer(0x00);  //31..24
  data = SPI.transfer(0x00);  //23..16
  data = SPI.transfer(0x00);  //15..8
  data = SPI.transfer(0x00);  //7..0
  digitalWrite(csPin, HIGH);
  delay(10);

  //Ask the chip a second time, now the data will be available
  digitalWrite(csPin, LOW);
  status = SPI.transfer(reg); //Send register location
  data   = SPI.transfer(0x00);  //31..24
  data   = data << 8;
  data  |= SPI.transfer(0x00);  //23..16
  data   = data << 8;
  data  |= SPI.transfer(0x00);  //15..8
  data   = data << 8;
  data  |= SPI.transfer(0x00);  //7..0
  digitalWrite(csPin, HIGH);
#else
#ifdef TMC_DEBUG
  Serial.print("Reg: 0x");
  Serial.print(reg,HEX);
  Serial.print(" : Data: 0b");
  Serial.println(data,BIN);
#endif //TMC_DEBUG
#endif

  standstill   = (status >> 3) & 0x01;
  stallGuard2  = (status >> 2) & 0x01;
  driver_error = (status >> 1) & 0x01;
  reset_flag   = (status >> 0) & 0x01;

  return data;
}

void Tmc2130::print_status()
{
#ifdef TMC_DEBUG
  Serial.print("standstill  : "); Serial.println(standstill);
  Serial.print("stallGuard2 : "); Serial.println(stallGuard2);
  Serial.print("driver_error: "); Serial.println(driver_error);
  Serial.print("reset_flag  : "); Serial.println(reset_flag);
#endif  
}

void Tmc2130::print_ioin()
{
#ifdef TMC_DEBUG
  ioin_t ioin;
  ioin = get_ioin();

  Serial.println("IOIN : 0x04 : ");
  Serial.println("---------------------------");

  Serial.print("step:         "); Serial.println(ioin.step?        "HIGH":"LOW");
  Serial.print("direct_mode:  "); Serial.println(ioin.direct_mode? "HIGH":"LOW");
  Serial.print("dcen_cfg4:    "); Serial.println(ioin.dcen_cfg4?   "HIGH":"LOW");
  Serial.print("dcin_cfg5:    "); Serial.println(ioin.dcin_cfg5?   "HIGH":"LOW");
  Serial.print("drv_enn_cfg6: "); Serial.println(ioin.drv_enn_cfg6?"HIGH":"LOW");
  Serial.print("dco:          "); Serial.println(ioin.dco?         "HIGH":"LOW");
  Serial.print("version:      "); Serial.println(ioin.version,HEX);
  Serial.println();
#endif
}
