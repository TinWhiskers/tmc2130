//****************************************************************************
//* (c) 2015 Tin Whiskers Technology.  All rights reserved.
//****************************************************************************

#ifndef tmc2130_H
#define tmc2130_H

#include <Arduino.h>

// GENERAL CONFIGURATION REGISTERS (0x00..0x0F)
#define GCONF      0x00 // RW   GCONF – Global configuration flags
#define GSTAT      0x01 // R+C  GSTAT – Global status flags
#define IOIN       0x04 // R    Reads the state of all input pins available

// VELOCITY DEPENDENT DRIVER FEATURE CONTROL REGISTER SET (0x10..0x1F)
#define IHOLD_RUN  0x10 //  W   IHOLD_IRUN – Driver current control
#define TPOWERDOWN 0x11 //  W   TPOWERDOWN sets the delay time after stand still (stst) of the motor to motor current power down. 
#define TSTEP      0x12 // R    Actual measured time between two 1/256 microsteps derived from the step input frequency in units of 1/fCLK.
#define TPWMTHRS   0x13 //  W   This is the upper velocity for stealthChop voltage PWM mode.
#define TCOOLTHRS  0x14 //  W   This is the lower threshold velocity for switching on smart energy coolStep and stallGuard feature.
#define THIGH      0x15 //  W   This velocity setting allows velocity dependent switching into a different chopper mode and fullstepping to maximize torque.

// DCSTEP MINIMUM VELOCITY REGISTER (0x33)
#define VDCMIN     0x33 //  W   dcStep minimum velocity register.

// MICROSTEPPING CONTROL REGISTER SET (0x60..0x6B)
#define MSLUT      0x60 //  W   Microstepping lookup table base address.  0x60 thru 0x67.
#define MSLUT0     0x60 //  W   MSLUT[0]
#define MSLUT1     0x61 //  W   MSLUT[1]
#define MSLUT2     0x62 //  W   MSLUT[2]
#define MSLUT3     0x63 //  W   MSLUT[3]
#define MSLUT4     0x64 //  W   MSLUT[4]
#define MSLUT5     0x65 //  W   MSLUT[5]
#define MSLUT6     0x66 //  W   MSLUT[6]
#define MSLUT7     0x67 //  W   MSLUT[7]
#define MSLUTSEL   0x68 //  W   This register defines four segments within each quarter MSLUT wave.
#define MSLUTSTART 0x69 //  W   START_SIN and START_SIN90
#define MSCNT      0x6A // R    Microstep counter.  Indicates actual position in the microstep table for CUR_A. CUR_B uses an offset of 256 (2 phase motor).
#define MSCURACT   0x6B // R    Actual microstep current for motor phases A & B

// DRIVER REGISTER SET (0X6C…0X7F)
#define CHOPCONF   0x6C // RW   Chopper and driver configuration
#define COOLCONF   0x6D //  W   coolStep smart current control register and stallGuard2 configuration
#define DCCTRL     0x6E //  W   dcStep (DC) automatic commutation configuration register (enable via pin DCEN or via VDCMIN):
#define DRV_STATUS 0x6F // R    stallGuard2 value and driver error flags
#define PWMCONF    0x70 //  W   Voltage PWM mode chopper configuration
#define PWM_SCALE  0x71 // R    Actual PWM amplitude scaler.  In voltage mode PWM, this value allows to detect a motor stall.
#define LOST_STEPS 0x73 // R    Number of input steps skipped due to higher load in dcStep operation

// MSCURACT register
typedef struct
{
  uint8_t cur_a;      // Actual microstep current for motor phase A as read from MSLUT (not scaled by current)
  uint8_t cur_b;      // Actual microstep current for motor phase B as read from MSLUT (not scaled by current)
} mscuract_t;

typedef struct
{
  uint8_t ihold;
  uint8_t irun;
  uint8_t iholddelay;
} ihold_run_t;


// DRV_STATUS register - stallGuard2 Value and Driver Error Flags
typedef struct
{
  uint16_t sg_result; // Mechanical load measurement.  
  boolean fsactive;   // TRUE = Driver has switched to fullstep as defined by chopper mode settings and velocity thresholds.
  uint8_t cs_actual;  // Actual motor current / smart energy current
  boolean stallguard; // TRUE = StallGuard - motor stall detected or dcStep stall in dcStep mode.
  boolean ot;         // TRUE = overtemp & drivers disabled.
  boolean otpw;       // TRUE = overtemp pre-warning threshold exceeded.
  boolean s2ga;       // TRUE = short to GND detected on phasa A
  boolean s2gb;       // TRUE = short to GND detected on phasa B
  boolean ola;        // TRUE = Open-load indicator on phase A
  boolean olb;        // TRUE = Open-load indicator on phase B
  boolean stst;       // TRUE = stand-still (2^20 clocks after last step pulse)
} drvst_t;

// CHOPCONF -> MRES - Micro-step resolution
enum mres_t
{
  MRES_256 = 0b0000,
  MRES_128 = 0b0001,
  MRES_64  = 0b0010,
  MRES_32  = 0b0011,
  MRES_16  = 0b0100,
  MRES_8   = 0b0101,
  MRES_4   = 0b0110,
  MRES_2   = 0b0111,
  MRES_1   = 0b1000
};

// CHOPCONF -> TBL - TBL blank time select.  16, 24, 36, or 54 clock cycles.
enum tbl_t
{
  TBL_16 = 0b00,
  TBL_24 = 0b01,      // Recommended for most applications.
  TBL_36 = 0b10,      // Recommended for most applications.
  TBL_54 = 0b11
};

// CHOPCONF
typedef struct
{
  boolean chm;        // Chopper mode.  TRUE = Constant off time with fast decay time.  FALSE=Standard mode (spreadCycle)
  uint8_t toff;       // TOFF off time & driver enable.  0=Driver disable.  Off time setting controls duration of slow decay phase NCLK= 12 + 32*TOFF
  uint8_t hstrt;      // 
  uint8_t hend;       // 
  boolean fd3;        // TRUE = 
  boolean disfdcc;    // Fast decay mode.  TRUE = disfdcc=1 disables current comparator usage for termination of the fast decay cycle
  boolean rndtf;      // Random TOFF time.  TRUE = Random mode, TOFF is random modulated by dNCLK = -12 ... +3 clocks.
  tbl_t   tbl;        // TBL blank time select.
  boolean vsense;     // TRUE = High sensitivity, low sense resistor voltage.  FALSE = Low sensitivity, high sense resistor voltage.
  boolean vhighfs;    // TRUE = Enables switching to fullstep, when VHIGH is exceeded. 
  boolean vhighchm;   // TRUE = TOFF setting automatically becomes doubled during high velocity operation in order to avoid doubling of the chopper frequency.
  uint8_t sync;       // SYNC PWM synchronization clock. 0=ChopSync off.  0001-1111: Sync with fsync=fck/(sync*64)
  mres_t  mres;       // Micro-step resolution.  
  boolean intpol;     // TRUE = The actual microstep resolution (MRES) becomes extrapolated to 256 microsteps for smoothest motor operation
  boolean dedge;      // TRUE = Enable step impulse at each step edge to reduce step frequency requirement.
  boolean diss2g;     // TRUE = Short to GND protection is disabled
} chopconf_t;

enum seup_t
{
  SEUP_1 = 0b00, // Current increment steps per measured stallGuard2 value = 1
  SEUP_2 = 0b01, // Current increment steps per measured stallGuard2 value = 2
  SEUP_4 = 0b10, // Current increment steps per measured stallGuard2 value = 4
  SEUP_8 = 0b11  // Current increment steps per measured stallGuard2 value = 8
};

enum sedn_t
{
  SEDN_32 = 0b00, // %00: For each 32 stallGuard2 values decrease by one
  SEDN_8  = 0b01, // %01: For each 8 stallGuard2 values decrease by one
  SEDN_2  = 0b10, // %10: For each 2 stallGuard2 values decrease by one
  SEDN_1  = 0b11  // %11: For each stallGuard2 value decrease by one
};

typedef struct
{
  uint8_t semin;  // 0..15.  minimum stallGuard2 value for smart current control and smart current enable
  seup_t  seup;   // current up step width
  uint8_t semax;  // 0..15.  stallGuard2 hysteresis value for smart current control
  sedn_t  sedn;   // Current down step speed
  boolean seimin; // TRUE=1/4 of current setting (IRUN), FALSE=1/2 of current setting (IRUN)
  int8_t  sgt;    // Signed, -64 to 63
  boolean sfilt;  // TRUE = stallGuard2 filter enable
} coolconf_t;

enum pwm_freq_t
{
  PWMFREQ_1_1024 = 0b00, // %00: f PWM =1/1024 f CLK
  PWMFREQ_1_683  = 0b01, // %01: f PWM =1/683 f CLK
  PWMFREQ_1_512  = 0b10, // %10: f PWM =1/512 f CLK
  PWMFREQ_1_410  = 0b11  // %11: f PWM =1/410 f CLK
};

enum freewheel_t
{
  FREEWHEEL_NORMAL    = 0b00, // %00: Normal operation
  FREEWHEEL_FREEWHEEL = 0b01, // %01: Freewheeling
  FREEWHEEL_SHORT_LS  = 0b10, // %10: Coil shorted using LS drivers
  FREEWHEEL_SHORT_HS  = 0b11  // %11: Coil shorted using HS drivers
};

typedef struct
{
  uint8_t     pwm_ampl;      // User defined amplitude (offset)
  uint8_t     pwm_grad;      // User defined amplitude (gradient) or regulation loop gradient
  pwm_freq_t  pwm_freq;      // PWM frequency selection
  boolean     pwm_autoscale; // PWM automatic amplitude scaling
  boolean     pwm_symmetric; // Force symmetric PWM
  freewheel_t freewheel;     // Allows different standstill modes
} pwmconf_t;

typedef struct
{
  uint8_t w0;
  uint8_t w1;
  uint8_t w2;
  uint8_t w3;
  uint8_t x1;
  uint8_t x2;
  uint8_t x3;
}mslutsel_t;

typedef struct
{
  uint8_t start_sin;
  uint8_t start_sin90;
} mslutstart_t;

typedef struct
{
  boolean step;
  boolean direct_mode;
  boolean dcen_cfg4;
  boolean dcin_cfg5;
  boolean drv_enn_cfg6;
  boolean dco;
  uint32_t version;
} ioin_t;

class tmc2130 {
public:
  tmc2130(uint8_t chipSelectPin);
  void begin(boolean initSpi);

  //**************************************************************************
  // SPI STATUS BITS
  //**************************************************************************
  // These bits are updated after every SPI read or write application. 
  boolean standstill;   // DRV_STATUS[31] – 1: Signals motor stand still
  boolean stallGuard2;  // DRV_STATUS[24] – 1: Signals stallguard flag active
  boolean driver_error; // GSTAT[1] – 1: Signals driver 1 driver error (clear by reading GSTAT)
  boolean reset_flag;   // GSTAT[0] – 1: Signals, that a reset has occurred (clear by reading GSTAT)

  //**************************************************************************
  // GENERAL CONFIGURATION REGISTERS (0x00..0x0F)
  //**************************************************************************
  //void set_gconf();
  void set_en_pwm_mode(boolean stealthchopEn); //GCONF bit 2
  boolean get_gstat_uv_cp(); // GSTAT NOTE: Already get [0] and [1] in the spi status byte.. no need to get them twice!
  
  ioin_t get_ioin();   //Version: 0x11=first version of the IC

  //**************************************************************************
  // VELOCITY DEPENDENT DRIVER FEATURE CONTROL REGISTER SET (0x10..0x1F)
  //**************************************************************************
  void set_ihold_irun(ihold_run_t ihold_run);
  void set_tpowerdown(uint8_t tpowerdown);
  void set_tpwmthrs(uint16_t tpwmthrs);
  void set_tcoolthrs(uint16_t tcoolthrs);
  void set_thigh(uint16_t thigh);

  uint32_t get_tstep();      // TSTEP Actual measured time between two 1/256 microsteps derived from the step input frequency in units of 1/fCLK.
  
  //**************************************************************************
  // DCSTEP REGISTERS
  //**************************************************************************
  void set_vdcmin(uint32_t vdcmin);
  uint32_t get_lost_steps(); // Number of input steps skipped due to higher load in dcStep operation, if step input does not stop when DC_OUT is low.

  //**************************************************************************
  // MICROSTEPPING CONTROL REGISTER SET (0x60..0x6B)
  //**************************************************************************
  // NOTE: Write functions TBD, currently done in the constructor by writing the default reg values
  void set_mslut(); //Set default Microstep lookup table.  
  void set_mslutsel(mslutsel_t mslutsel);
  void set_mslutstart(mslutstart_t mslutstart);
  //void set_mslut(void); //NOTE: No settings here, it is to be hard-coded.
  uint16_t get_mscnt();      // MSCNT register.  
  mscuract_t get_mscuract(); // actual current_a and current_b read from MSLUT, not scaled

  //**************************************************************************
  // DRIVER REGISTER SET (0X6C…0X7F)
  //**************************************************************************
  void set_chopconf(chopconf_t chopconf);
  void set_coolconf(coolconf_t coolconf);
  void set_dcctrl(uint8_t dc_time, uint8_t dc_sg);
  void set_pwmconf(pwmconf_t pwmconf);

  // DRV_STATUS : 0x6F : stallGuard2 Value and Driver Error Flags
  // stallGuard2 value and driver error flags
  drvst_t  get_drv_status(); 
  uint16_t get_loadmeas();  // Get back-EMF load measurement from stallGuard2.  This is more efficient than

  // PWM_SCALE : 0x71 : Actual PWM amplitude scaler (255=max. Voltage).  StealthChop / Voltage Mode.
  // Information about the motor state is available with automatic scaling by reading out PWM_SCALE. As
  // this parameter reflects the actual voltage required to drive the target current into the motor, it
  // depends on several factors: motor load, coil resistance, supply voltage, and current setting. Therefore,
  // an evaluation of the PWM_SCALE value allows seeing the motor load (similar to stallGuard2) and
  // finding out if the target current can be reached. It even gives an idea on the motor temperature
  // (evaluate at a well-known state of operation).
  uint8_t  get_pwm_scale();  


private:
  uint8_t csPin;

  //**************************************************************************
  // LOW-LEVEL SPI FUNCTIONS
  //**************************************************************************
  void     spi_write(uint8_t reg, uint32_t data);
  uint32_t spi_read (uint8_t reg);
  boolean en_pwm_mode;  // GCONF - stealthChop voltage PWM mode enabled (depending on velocity thresholds).
};

#endif //tmc2130_H
