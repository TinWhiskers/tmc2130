//****************************************************************************
//* (c) 2015 Tin Whiskers Technology.  All rights reserved.
//****************************************************************************
#include <SPI.h>

#include "tmc2130.h"
//#define SIMULATION 1

tmc2130::tmc2130(uint8_t chipSelectPin)
{
  csPin = chipSelectPin;
}

void tmc2130::begin(boolean initSpi)
{
  pinMode(csPin,OUTPUT);
  digitalWrite(csPin,HIGH);
  if (initSpi)
  {
    SPI.begin();
    //CPOL=0, CPHA=0.  MSB first, per datasheet.  Assuming fsck = 8 MHz
    SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  }

  //TODO: Try 'true' to test 256 MicroPlyer interpolation.

  //spi_write(CHOPCONF,   0x000100C5); // CHOPCONF: TOFF=5, HSTRT=4, HEND=1, TBL=2, CHM=0 (spreadCycle)
  //spi_write(IHOLD_IRUN, 0x00061F0A); // IHOLD_IRUN: IHOLD=10, IRUN=31 (max. current), IHOLDDELAY=6
  //spi_write(TPOWERDOWN, 0x0000000A); // TPOWERDOWN=10: Delay before power down in stand still
  //spi_write(GCONF,      0x00000004); // EN_PWM_MODE=1 enables stealthChop (with default PWM_CONF)
  //spi_write(TPWMTHRS,   0x000001F4); // TPWMTHRS=500 
  //spi_write(PWMCONF,    0x000401C8); // PWMCONF: AUTO=1, 1/1024 Fclk, Switch amplitude limit=200, Grad=1
  //spi_write(MSLUTSEL,   0xFFFF8056); // 0b11111111111111111000000001010110 = 0xFFFF8056: 
  //spi_write(MSLUTSTART, 0x00F70000); // 0b00000000111101110000000000000000 = 0x00F70000: 


  // GCONF 0x00
  set_en_pwm_mode(false); //Disable en_pwm_mode (StealthChop)
  
  // IHOLD_RUN 0x10
  ihold_run_t ihold_run;
  ihold_run.ihold      = 10;
  ihold_run.irun       = 31;
  ihold_run.iholddelay = 6;
  set_ihold_irun(ihold_run);

  // TPOWERDOWN 0x11
  set_tpowerdown(10); // Delay before power down in stand still
  
  // TPWMTHRS 0x13
  set_tpwmthrs(500); // yields a switching velocity about 35000 = ca. 30RPM
 
  // TCOOLTHRS 0x14
  set_tcoolthrs(0);
  
  // THIGH 0x15
  set_thigh(0);

  // CHOPCONF 0x6C
  chopconf_t chopconf;
  chopconf.chm      = false;  // Chopper mode.  Standard mode (SpreadCycle)
  chopconf.toff     = 5;      // 0=Driver off. General enable for stepper motor driver.
  chopconf.hstrt    = 4;      // TBD
  chopconf.hend     = 1;      // TBD
  chopconf.fd3      = false;  // TBD
  chopconf.disfdcc  = false;  // TBD
  chopconf.rndtf    = false;  // TBD
  chopconf.tbl      = TBL_36; // TBD.. TBL_24 or TBL_36.
  chopconf.vsense   = false;
  chopconf.vhighfs  = false;  // TBD
  chopconf.vhighchm = false;  // TBD
  chopconf.sync     = 0;      // TBD
  chopconf.mres     = MRES_16;
  chopconf.intpol   = false;  // MicroPlyer interpolation on/off
  chopconf.dedge    = false;  // TBD
  chopconf.diss2g   = false;  // TBD
  set_chopconf(chopconf);

  // COOLCONF 0x6D
  coolconf_t coolconf;
  coolconf.semin    = 0;      // TBD 0..15.  minimum stallGuard2 value for smart current control and smart current enable
  coolconf.seup     = SEUP_1; //TBD 1/2/4/8 Current up step width
  coolconf.semax    = 0;      // TBD 0..15.  stallGuard2 hysteresis value for smart current control
  coolconf.sedn     = SEDN_1; // TBD Current down step speed 1/2/8/32
  coolconf.seimin   = false;  // TBD TRUE=1/4 of current setting (IRUN), FALSE=1/2 of current setting (IRUN)
  coolconf.sgt      = 0;      // TBD Signed, -64 to 63
  coolconf.sfilt    = false;  // TBD  TRUE = stallGuard2 filter enable
  set_coolconf(coolconf);

  //DCSTEP ... Not using this crap
  // set_dcctrl(uint8_t dc_time, uint8_t dc_sg);
  //set_vdcmin(0);

  // PWMCONF 0x70
  pwmconf_t pwmconf;
  pwmconf.pwm_ampl      = 0; // User defined amplitude (offset)
  pwmconf.pwm_grad      = 0; // User defined amplitude (gradient) or regulation loop gradient
  pwmconf.pwm_freq      = PWMFREQ_1_683; // PWM frequency selection (410/512/683/1024)
  pwmconf.pwm_autoscale = false; // PWM automatic amplitude scaling
  pwmconf.pwm_symmetric = false; // Force symmetric PWM
  pwmconf.freewheel     = FREEWHEEL_NORMAL; // Allows different standstill modes
  set_pwmconf(pwmconf);

  // MSLUTSEL 0x68
  // Configure Microstepping Control Register Set (values from datasheet)
  mslutsel_t mslutsel;
  mslutsel.w0 = 2;   // LUT width select from ofs00 to ofs(X1-1)
  mslutsel.w1 = 1;   // LUT width select from ofs(X1) to ofs(X2-1)
  mslutsel.w2 = 1;   // LUT width select from ofs(X2) to ofs(X3-1)
  mslutsel.w3 = 1;   // LUT width select from ofs(X3) to ofs255
  mslutsel.x1 = 11;  // LUT segment 1 start
  mslutsel.x2 = 255; // LUT segment 2 start
  mslutsel.x3 = 255; // LUT segment 3 start
  set_mslutsel(mslutsel);

  // MSLUTSTART 0x69
  mslutstart_t mslutstart;
  mslutstart.start_sin   = 0;
  mslutstart.start_sin90 = 0;
  set_mslutstart(mslutstart);
}

//**************************************************************************
// GENERAL CONFIGURATION REGISTERS (0x00..0x0F)
//**************************************************************************

void tmc2130::set_en_pwm_mode(boolean stealthchopEn)
{
  // Flip bit 2 of GCONF
  // NOTE: Assumes all other bits of GCONF will always be '0'!
  spi_write(GCONF, stealthchopEn?0x00000004:0x00000000);
}

boolean tmc2130::get_gstat_uv_cp() // GSTAT NOTE: Already get [0] and [1] in the spi status byte.. no need to get them twice!
{
  uint32_t response = spi_read(GSTAT);
  return (boolean)((response >> 2) & 0x01);
}

ioin_t tmc2130::get_ioin()
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
  //                                                      000100010000000000000000
  return ioin;
}

//**************************************************************************
// VELOCITY DEPENDENT DRIVER FEATURE CONTROL REGISTER SET (0x10..0x1F)
//**************************************************************************

void tmc2130::set_ihold_irun(ihold_run_t ihold_run)
{
  uint32_t data = 0;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 33222222222211111111110000000000
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 10987654321098765432109876543210
  data |= (((uint32_t)ihold_run.ihold          << 0)  & 0b00000000000000000000000000001111); // 0..3
  data |= (((uint32_t)ihold_run.irun           << 8)  & 0b00000000000000000001111100000000); // 8..12
  data |= (((uint32_t)ihold_run.iholddelay     << 16) & 0b00000000000011110000000000000000); // 16..19
  spi_write(IHOLD_RUN, data);
}

void tmc2130::set_tpowerdown(uint8_t tpowerdown)
{
  tpowerdown &= 0b11111111;              // 8-bit field
  spi_write(TPOWERDOWN, tpowerdown);
}

void tmc2130::set_tpwmthrs(uint16_t tpwmthrs)
{
  tpwmthrs &= 0b11111111111111111111;    // 20-bit field
  spi_write(TPWMTHRS, tpwmthrs);
}

void tmc2130::set_tcoolthrs(uint16_t tcoolthrs)
{
  tcoolthrs &= 0b11111111111111111111;    // 20-bit field
  spi_write(TCOOLTHRS, tcoolthrs);
}

void tmc2130::set_thigh(uint16_t thigh)
{
  thigh     &= 0b11111111111111111111;    // 20-bit field
  spi_write(THIGH, thigh);
}

uint32_t tmc2130::get_tstep()      // TSTEP Actual measured time between two 1/256 microsteps derived from the step input frequency in units of 1/fCLK.
{
  uint32_t response = spi_read(TSTEP);
  return (response & 0b11111111111111111111);
}

//**************************************************************************
// DCSTEP REGISTERS
//**************************************************************************

void tmc2130::set_vdcmin(uint32_t vdcmin)
{
  vdcmin    &= 0b11111111111111111111111; // 23-bit field
  spi_write(VDCMIN, vdcmin);
}

uint32_t tmc2130::get_lost_steps() // Number of input steps skipped due to higher load in dcStep operation, if step input does not stop when DC_OUT is low.
{
  uint32_t response = spi_read(LOST_STEPS);
  return (response & 0b11111111111111111111);
}

//**************************************************************************
// MICROSTEPPING CONTROL REGISTER SET (0x60..0x6B)
//**************************************************************************
void tmc2130::set_mslut()
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

}

void tmc2130::set_mslutsel(mslutsel_t mslutsel)
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
}

void tmc2130::set_mslutstart(mslutstart_t mslutstart)
{
  uint32_t data = 0;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 33222222222211111111110000000000
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 10987654321098765432109876543210
  data |= (((uint32_t)mslutstart.start_sin     << 0)  & 0b00000000000000000000000011111111); // 0..7
  data |= (((uint32_t)mslutstart.start_sin90   << 16) & 0b00000000111111110000000000000000); // 23..16
  spi_write(MSLUTSTART, data);
}

uint16_t tmc2130::get_mscnt()      // MSCNT register.  
{
  uint32_t response = spi_read(MSCNT);
  return (response & 0b1111111111);
}

mscuract_t tmc2130::get_mscuract() // actual current_a and current_b read from MSLUT, not scaled
{
  mscuract_t mscuract;
  uint32_t response = spi_read(MSCURACT);
  mscuract.cur_a = (((uint8_t)response      ) & 0b111111111);
  mscuract.cur_b = (((uint8_t)response >> 16) & 0b111111111);
  return mscuract;
}

//**************************************************************************
// DRIVER REGISTER SET (0X6C…0X7F)
//**************************************************************************

void tmc2130::set_chopconf(chopconf_t chopconf)
{
  uint32_t data = 0;
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
}

void tmc2130::set_coolconf(coolconf_t coolconf)
{
  uint32_t data = 0;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 33222222222211111111110000000000
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 10987654321098765432109876543210
  data |= (((uint32_t)coolconf.semin           <<  0) & 0b00000000000000000000000000001111); // 0..3
  data |= (((uint32_t)coolconf.seup            <<  5) & 0b00000000000000000000000001100000); // 5..6
  data |= (((uint32_t)coolconf.semax           <<  8) & 0b00000000000000000000111100000000); // 8..11
  data |= (((uint32_t)coolconf.sedn            << 13) & 0b00000000000000000000000000000000); // 13..14
  data |= (((uint32_t)coolconf.seimin          << 15) & 0b00000000000000000010000000000000); // 15
  data |= (((uint32_t)coolconf.sgt             << 16) & 0b00000000000111111100000000000000); // 16..22
  data |= (((uint32_t)coolconf.sfilt           << 24) & 0b00000000010000000000000000000000); // 24
  spi_write(CHOPCONF, data);
}

void tmc2130::set_dcctrl(uint8_t dc_time, uint8_t dc_sg)
{
  uint32_t data = 0;
  data |= (((uint32_t)dc_time)     & 0b1111111111);
  data |= (((uint32_t)dc_sg << 16) & 0b11111111);
  spi_write(IHOLD_RUN, data);
}

void tmc2130::set_pwmconf(pwmconf_t pwmconf)
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
  spi_write(CHOPCONF, data);
}

// DRV_STATUS : 0x6F : stallGuard2 Value and Driver Error Flags
// Datasheet page 37
drvst_t  tmc2130::get_drv_status() // stallGuard2 value and driver error flags
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

// NOTE: This function is very similar to the above function, but is optimized for when only the
//    sg_result field is needed.  It uses less memory and less CPU cycles
uint16_t tmc2130::get_loadmeas()
{
  uint32_t response = spi_read(DRV_STATUS);
  return  (uint16_t)(response & 0b00000000000000000000000111111111);
}

uint8_t  tmc2130::get_pwm_scale()  // Actual PWM amplitude scaler (255=max. Voltage)
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
void tmc2130::spi_write(uint8_t reg, uint32_t data)
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
  Serial.print("Reg: 0x");
  Serial.print(reg,HEX);
  Serial.print(" : Data: 0b");
  Serial.println(data,BIN);
#endif
}

//          w | 38..32  |  31..24  |  23..16  |  15..8   |   7..0   |
//         ----------------------------------------------------------
// master   0 | address | 00000000 | 00000000 | 00000000 | 00000000 | (dummy data)
// response 0 | status  | MSB -- data --                        lsb |
uint32_t tmc2130::spi_read(uint8_t reg)
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
  Serial.print("Reg: 0x");
  Serial.print(reg,HEX);
  Serial.print(" : Data: 0b");
  Serial.println(data,BIN);
#endif

  standstill   = (status >> 3) & 0x01;
  stallGuard2  = (status >> 2) & 0x01;
  driver_error = (status >> 1) & 0x01;
  reset_flag   = (status >> 0) & 0x01;

  return data;
}
