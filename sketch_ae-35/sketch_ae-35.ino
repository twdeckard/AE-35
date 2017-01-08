#include "Arduino.h"
#include <Wire.h>
#include <TFT.h>
// requires fork of AFMotor to remap shift reg pins to slot 4 for DFRobot mega multi expansion shield
#include <AFMotor.h>
// requires fork of Time for sub second resolution
#include <Time.h>
#include <TimeLib.h>
#include <AccelStepper.h>
#include <LiquidCrystal.h>
// requires fork of PLAN13 for fractional seconds input for smooth'er motion, a bit of a clumsy hack
#include <Plan13.h>
// I2Cdev and MPU9150 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU9150.h"
#include "helper_3dmath.h"
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU9150 accelGyroMag;

/***********************************************************************************************************
 *
 * AE-35 Antenna Controller
 *
 * Satellite Tracking System
 *
 * Arduino Mega 2560 R3
 * SainSmart L293D Motor Drive
 * SainSmart LCD 1602 + Keypad
 * MPU-9150 accelerometer/gyro/compass
 *
 * Compute the location of one of a selection of orbiting satellites from an internal database
 * and steer an altitude/azimuth gimbal to maintain a track while object is above the horizon.
 *
 * On initialization the user inputs the GMT time and date, chooses from a list of orbiting objects 
 * and steers the antenna gimbal to north/horizon using manual inputs. 
 * 
 * Copyright 2015 Todd Deckard, All rights reserved. 
 * 
 * 
 * *******************************************************************************************************/
 
 
#define NEVER                                     0
#define ALWAYS                                    1

// only mode implemented is SAT_TRACKR 

#define SAT_TRACKR     0
#define GND_RPEATR     (SAT_TRACKR + 1)
#define SKY_SURVEY     (SAT_TRACKR + 2)
#define GND_SURVEY     (SAT_TRACKR + 3)
#define SKY_SEARCH     (SAT_TRACKR + 4)
#define SKY_TRACKR     (SAT_TRACKR + 5)
#define GND_SEARCH     (SAT_TRACKR + 6)
#define GND_TRACKR     (SAT_TRACKR + 7)
#define EASYCOMMII     (SAT_TRACKR + 8)

#define NUMBEROFMODES  (EASYCOMMII - SAT_TRACKR + 1)

#define INIT        0
#define CNFG        (INIT+1)
#define GPSL        (INIT+2)
#define DRIV        (INIT+3)
#define TRAC        (INIT+4)
#define HOME        (INIT+5)
#define SERL        (INIT+6)
#define STOP        (INIT+7)

#define  NUMBEROFSTATES     (STOP - INIT + 1)

// forward declarations make the compiler happy

void  satTrackLCD(), pollGPS(), currentMotorPos(), updateSat(), currentMotorPos(), 
      satTrackButtons(),  processSerial1(), stepsAzElToMotors(), cmdAzElToSteps(), 
      checkSerial1(), readBtn(), readSwtch(), MPU9150_setupCompass();

int   MPU9150_readSensor(int addrL, int addrH),MPU9150_writeSensor(int addr,int data);

// typedefs with arrays make the compiler sad

typedef void (*StateFunctionsType)(void);
typedef void (*ExecutiveFunctionsArrayType[5][NUMBEROFSTATES])();
typedef short int StateTransitionArrayType[NUMBEROFSTATES][NUMBEROFSTATES];

unsigned short int           _state;
unsigned short int           _stateTransitionFlag;

time_t                       _time;
unsigned long int            _ms;
unsigned long int            _lastms = 0;
unsigned long int            _fastTimer;
unsigned long int            _mediumTimer;
unsigned long int            _slowTimer;

// future, single object for Az and El to provide bounds checks and coordinate transformations between antenna and frame
typedef struct { 
  double theta;
  double x;
  double y;
  double z; 
} axisAngle; 

typedef struct {
  double rho; 
  double theta;
} coord; 

class gimbal 
{
    private:
        double    Az; 
        double    El;
        boolean   homePositionSet = false;
        double    AzMotorStepsPerDegree; 
        double    ElMotorStepsPerDegree; 
        long int  AzMotorUpperLimitSteps;
        long int  AzMotorLowerLimitSteps; 
        long int  ElMotorUpperLimitSteps;
        long int  ElMotorLowerLimitSteps; 
        double    AzAntLowerLimitDeg; 
        double    AzAntUpperLimitDeg; 
        double    ElAntLowerLimitDeg; 
        double    ElAntUpperLimitDeg; 
        unsigned long int    AzMotorIdleTimeout;
        unsigned long int    ElMotorIdleTimeout;  
        axisAngle IMU;
        axisAngle plumb; 

    public:
        long int boundAntAzSteps(long int steps) {
          if (!homePositionSet) return steps; 
          steps = min(steps,AzMotorUpperLimitSteps);
          steps = max(steps,AzMotorLowerLimitSteps);
          return steps; 
        }
        long int boundAntElSteps(long int steps) {
          if (!homePositionSet) return steps; 
          steps = min(steps,ElMotorUpperLimitSteps);
          steps = max(steps,ElMotorLowerLimitSteps);
          return steps; 
        }
        long int gimbalAzDegToSteps(double deg) {
          long int steps = (int)round(deg * AzMotorStepsPerDegree); 
          return steps;
        }
        long int gimbalElDegToSteps(double deg) { 
          long int steps = (int)round(deg * ElMotorStepsPerDegree); 
          return steps;
        }
        double gimbalAzStepsToDeg(long int steps) {
          double deg = (double)steps / AzMotorStepsPerDegree;
          return deg; 
        }
        double gimbalElStepsToDeg(long int steps) {
          double deg = (double)steps / ElMotorStepsPerDegree;
          return deg; 
        }
        double boundAntAzDeg(double deg) {   
          if (!homePositionSet) return deg; 
          // deg = (deg + 360) % 360;          // should really check for >360
          if      (deg > AzAntUpperLimitDeg)   deg = deg - 360.;      // simple cable wrap logic
          else if (deg < AzAntLowerLimitDeg)   deg = deg + 360.;
          // deg = min(deg,AzAntUpperLimitDeg);
          // deg = max(deg,AzAntLowerLimitDeg);
          return deg; 
        }
        double boundAntElDeg(double deg) {
          if (!homePositionSet) return deg; 
          deg = min(deg,ElAntUpperLimitDeg);
          deg = max(deg,ElAntLowerLimitDeg);
          return deg; 
        }
        double   AbsAzToAntAz(double AzDeg) {
          double rotate = AzDeg + plumb.theta;  // should rotate coordinates thru plumb vector, assume level and mag=true for now
          return rotate;                        // need to channel Olinde Rodrigues
        }
        double   AbsElToAntEl(double ElDeg) {   // should rotate coordinates thru plumb vector, assume level and mag=true for now
          double rotate = ElDeg;
          return rotate;
        }
        void ae35();
        // void ~ae35(); // either I don't understand c++ anymore or the arduino doesn't, no delete?

        void set_targetAz(double arg) { Az = boundAntAzDeg(AbsAzToAntAz(arg)); Serial.println("set_targetAz"); Serial.println(Az);};
        void set_targetEl(double arg) { El = boundAntElDeg(AbsElToAntEl(arg)); Serial.println("set_targetEl"); Serial.println(El);};
        void set_IMUSensorToEarth(double theta, double x, double y, double z) {   // relationship of sensor to earth
            IMU.theta = theta;
            IMU.x = x; 
            IMU.y = y;
            IMU.z = z;
        }
        void set_SensorToFrame(double theta, double x, double y, double z) {      // relationship of sensor to frame datum
            plumb.theta = theta; 
            plumb.x     = x; 
            plumb.y     = y;
            plumb.z     = z; 
        }
        void set_AzMotorUpperLimitSteps(long int steps) {
                AzMotorUpperLimitSteps = steps; 
        }                
        void set_AzMotorLowerLimitSteps(long int steps) {
                AzMotorLowerLimitSteps = steps; 
        }
        void set_ElMotorUpperLimitSteps(long int steps) {
                ElMotorUpperLimitSteps = steps; 
        }
        void set_ElMotorLowerLimitSteps(long int steps) {
                ElMotorLowerLimitSteps = steps; 
        }
        void set_AzAntLowerLimitDeg(double deg) {         // could also test for "reasonable" limits here
                AzAntLowerLimitDeg = deg; 
        }
        void set_AzAntUpperLimitDeg(double deg) {
                AzAntUpperLimitDeg = deg;  
        }
        void set_ElAntLowerLimitDeg(double deg) {
                ElAntLowerLimitDeg = deg; 
        }
        void set_ElAntUpperLimitDeg(double deg) {
                ElAntUpperLimitDeg = deg;  
        }
        void set_ElAntStepsPerDegree(double steps) {
                ElMotorStepsPerDegree = steps;
        }
        void set_AzAntStepsPerDegree(double steps) {
                AzMotorStepsPerDegree = steps; 
        }
        void AddStepsToElevation(long int steps) {
          // Serial.print("[AddStepsToElevation]"); Serial.print("<input>steps="); Serial.println(steps); 
          El = El + gimbalElStepsToDeg(steps); 
          if (homePositionSet) El = boundAntElDeg(El); 
        }
        void AddDegreesToElevation(double deg) {
          // Serial.print("[AddDegreesToElevation]"); Serial.print("<input>deg="); Serial.println(deg);
          El = El + deg;
          if (homePositionSet) El = boundAntElDeg(El);
        }
        void AddStepsToAzimuth(long int steps) {
          // Az = boundAntAzDeg(Az + gimbalAzStepsToDeg(steps));
          Az = Az + gimbalAzStepsToDeg(steps); 
          if (homePositionSet) Az = boundAntAzDeg(Az); 
        }
        void AddDegreesToAzimuth(double deg) {
          Az = Az + deg;
          if (homePositionSet) Az = boundAntAzDeg(Az); 
        }
        void setHome() { homePositionSet = true; };
        void clearHome() { homePositionSet = false; };
        int get_AzTargetMotorSteps()  { return (boundAntAzSteps(gimbalAzDegToSteps(AbsAzToAntAz(Az)))); };
        int get_ElTargetMotorSteps()  { return (boundAntElSteps(gimbalElDegToSteps(AbsElToAntEl(El)))); };
        bool get_TrueIfHomeSet() { return homePositionSet; }
        double get_AzTarget() { return (Az); };
        double get_ElTarget() { return (El); };
        void setAzIdleTimeout (unsigned long int timenow) { AzMotorIdleTimeout = timenow; };
        void setElIdleTimeout (unsigned long int timenow) { ElMotorIdleTimeout = timenow; };
        unsigned long int get_AzIdleTimeout() { return AzMotorIdleTimeout; };
        unsigned long int get_ElIdleTimeout() { return ElMotorIdleTimeout; };
      

        void debug() {
          Serial.print("Az=");
          Serial.println(Az); 
          Serial.print("El=");
          Serial.println(El); 
          Serial.print("AzMotorStepsPerDegree=");
          Serial.println(AzMotorStepsPerDegree);
          Serial.print("ElMotorStepsPerDegree=");
          Serial.println(ElMotorStepsPerDegree);
          Serial.print("AzMotorUpperLimitSteps=");
          Serial.println(AzMotorUpperLimitSteps);
          Serial.print("AzMotorLowerLimitSteps=");
          Serial.println(AzMotorLowerLimitSteps);
          Serial.print("ElMotorUpperLimitSteps=");
          Serial.println(ElMotorUpperLimitSteps);
          Serial.print("ElMotorLowerLimitSteps=");
          Serial.println(ElMotorLowerLimitSteps);
          Serial.print("AzAntLowerLimitDeg=");
          Serial.println(AzAntLowerLimitDeg);
          Serial.print("AzAntUpperLimitDeg=");
          Serial.println(AzAntUpperLimitDeg);
          Serial.print("ElAntLowerLimitDeg=");
          Serial.println(ElAntLowerLimitDeg);
          Serial.print("ElAntUpperLimitDeg=");
          Serial.println(ElAntUpperLimitDeg);
        };
} AE35; 

// for now, think globally but act locally

double                      _latitude;
double                      _longitude;

double                      _maxElSpeed; 
double                      _maxAzSpeed; 

String                      _target;
String                      _serialInput;       // repetitive string commmands may be perilous 


String                      _stateLabels[] = { "INIT", "CNFG", "GPSL", "DRIV", "TRAC", "HOME", "SERL", "STOP"};

unsigned short int          _lcd_button_now = 0;
unsigned short int          _lcd_button_prev = 0;
unsigned short int          _lcd_button_debounce = 0;



unsigned short int          _limitSwitch = 0;

unsigned short int          _adc_debug_temporary = 0;
unsigned short int          _gps_debug_temporary = 0;
unsigned short int          _accel_debug_temporary = 0;

boolean                     _flagTimeSetManually = 0;
boolean                     _flagDateSetManually = 0;
boolean                     _flagSatelliteSet = 0;

/* plumb bob values */ 
double _accX =0, _accY=0, _accZ=0; 
double _compX=0, _compY=0,_compZ=0;



double _roll;
double _pitch; 
double _heading;


Plan13              p13;
// temporarily hard code in TLEs
// http://www.amsat.org/amsat/ftp/keps/current/nasabare.txt
//
String _keps[] =
{
"2016-12-29",
"BY70-1",                  
"1 41909U 16083C   17007.00151180  .00394953  23696-5  76574-3 0  9998",
"2 41909  97.5837  85.2882 0200235 329.9655  28.9838 15.73432531  1539",
"SO-50",     
"1 27607U 02058C   17006.80708347  .00000132  00000-0  19126-4 0  9997",
"2 27607  64.5563 258.6893 0043453 165.5221 194.7626 14.75268359755300",
"AO-85",
"1 40967U 15058D   16364.30099515  .00000193  00000-0  41154-4 0 03533",
"2 40967 064.7775 014.1427 0180170 140.3304 221.1144 14.75282999052058",
"STARS-C",                 
"1 41895U 98067KR  16358.80991774  .00012447  00000-0  19209-3 0  9997",
"2 41895  51.6416 192.5419 0004954  32.0961 328.0333 15.54532956   654",
"ISS",          
"1 25544U 98067A   16353.17824509  .00001755  00000-0  34038-4 0  9994",
"2 25544  51.6443 220.6275 0006286 346.5926 159.5138 15.53908628 33607",
"FUNCUBE-3 (EO-79)",       
"1 40025U 14033R   16353.29594094  .00000348  00000-0  43766-4 0  9997",
"2 40025  97.9201 258.6026 0012596 173.5067 186.6315 14.88249904135583",
"HUBBLE",
"1 20580U 90037B   16337.33349726  .00000643  00000-0  28860-4 0  9992",
"2 20580  28.4735 228.1317 0002820 210.1105 149.9297 15.08610929260529",
"MOON2015_11",
"1 01511U 00000    15298.25194076  .00000000  00000-0  10000-3 0 00004",
"2 01511 018.2897 359.7740 0563000 005.5133 355.1249  0.03660099000003",
"END",
};

// http://www.embedded.com/design/mcus-processors-and-socs/4006438/Generate-stepper-motor-speed-profiles-in-real-time

#define AZIMUTH_MOTOR_STEPS_PER_REV         200
#define ELEVATION_MOTOR_STEPS_PER_REV       200
#define AZIMUTH_GIMBAL_STEPS_PER_DEGREE     ((1180.0) / 90.0)
#define ELEVATION_GIMBAL_STEPS_PER_DEGREE   ((2000.0) / 50.0)

#define AZIMUTH_MOTOR_MAX_STEPS             (3000)
#define AZIMUTH_MOTOR_MIN_STEPS             (-3000)
#define ELEVATION_MOTOR_MAX_STEPS           (2100)
#define ELEVATION_MOTOR_MIN_STEPS           (0)
#define ELEVATION_GIMBAL_MAX_ELEVATION      (55)

#define MAX_EL_SPEED_MANUAL                 (200)
#define MAX_EL_SPEED_TRACK                  (200)
#define MAX_EL_SPEED_HOME                   (75)
#define MAX_AZ_SPEED_MANUAL                 (200)
#define MAX_AZ_SPEED_TRACK                  (200)
#define MAX_AZ_SPEED_HOME                   (100)

#define MOTOR_MS_IDLE_BEFORE_RELEASE        (1500)

void readBtn();

/********************************************************************************************************
 *
 * Display and Inputs Functions
 *
 *******************************************************************************************************/

LiquidCrystal lcd(28,11,24,25,26,27);

#define LCD_BUTTON_ANALOG_INPUTS_TOLERANCE        3
#define LCD_BUTTON__RIGHT_ANALOG_VALUE            0
#define LCD_BUTTON__LEFT__ANALOG_VALUE            504
#define LCD_BUTTON__DOWN__ANALOG_VALUE            328
#define LCD_BUTTON___UP___ANALOG_VALUE            143
#define LCD_BUTTON_SELECT_ANALOG_VALUE            741
#define LCD_BUTTON__NONE__ANALOG_VALUE            1023

#define LIMIT_SWITCH_EL_MAX                       2

#define LCD_BUTTON_NONE                           2
#define LCD_BUTTON_RIGHT                          4
#define LCD_BUTTON_LEFT                           8
#define LCD_BUTTON_DOWN                           16
#define LCD_BUTTON_UP                             32
#define LCD_BUTTON_SELECT                         64
#define GPS_LOCK                                  128
#define HOME_POSITION_SET                         256
#define SERIAL_DATA                               1024



void readBtn()
{

#ifdef FUNCTIONTRACETOSERIAL
    Serial.println(__func__);
#endif

    unsigned int adc = analogRead(0);
    _adc_debug_temporary = adc;

    if (abs(adc - LCD_BUTTON__NONE__ANALOG_VALUE)<LCD_BUTTON_ANALOG_INPUTS_TOLERANCE) _lcd_button_now = LCD_BUTTON_NONE;
    else if (abs(adc - LCD_BUTTON__RIGHT_ANALOG_VALUE)<LCD_BUTTON_ANALOG_INPUTS_TOLERANCE) _lcd_button_now = LCD_BUTTON_RIGHT;
    else if (abs(adc - LCD_BUTTON__LEFT__ANALOG_VALUE)<LCD_BUTTON_ANALOG_INPUTS_TOLERANCE) _lcd_button_now = LCD_BUTTON_LEFT;
    else if (abs(adc - LCD_BUTTON___UP___ANALOG_VALUE)<LCD_BUTTON_ANALOG_INPUTS_TOLERANCE) _lcd_button_now = LCD_BUTTON_UP;
    else if (abs(adc - LCD_BUTTON__DOWN__ANALOG_VALUE)<LCD_BUTTON_ANALOG_INPUTS_TOLERANCE) _lcd_button_now = LCD_BUTTON_DOWN;
    else if (abs(adc - LCD_BUTTON_SELECT_ANALOG_VALUE)<LCD_BUTTON_ANALOG_INPUTS_TOLERANCE) _lcd_button_now = LCD_BUTTON_SELECT;

    if (_lcd_button_debounce != _lcd_button_now) {
      _stateTransitionFlag |= _lcd_button_now;
    }
    _lcd_button_debounce = _lcd_button_now; 

} ;

void readSwtch()
{
     if (digitalRead(31)==HIGH) _limitSwitch |= LIMIT_SWITCH_EL_MAX;   // no debounce, should really walk off the switch
     else                       _limitSwitch & !LIMIT_SWITCH_EL_MAX;                           
}



void pollGPS()                            // GPS shield is not integrated
{

#ifdef FUNCTIONTRACETOSERIAL
    Serial.println(__func__);
#endif

  double alpha = 0.8;
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  int16_t mx, my, mz;
  
  accelGyroMag.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  _compX = mx*10.*1229./4096.;   // Conversion from 1229 microTesla full scale (4096) to 12.29 Gauss full scale
  _compY = my*10.*1229./4096.;   // note: really need to calibrate for large ferrous soft steel bench ginder stand
  _compZ = mz*10.*1229./4096.;   // values are gibberish now, but hey at least they are in uTesla. 
  _accX  = ax*alpha + ((_accX)*(1.0-alpha));
  _accY  = ay*alpha + ((_accY)*(1.0-alpha));
  _accZ  = az*alpha + ((_accZ)*(1.0-alpha));
  
  _roll  = atan2(_accY, _accZ) * RAD_TO_DEG;
  _pitch = atan(-_accX / sqrt(_accY * _accY + _accZ * _accZ)) * RAD_TO_DEG;
  _heading = atan2(_compY,_compX) * RAD_TO_DEG; // note we compute the tilt and then disregard it for heading!
   
  AE35.set_IMUSensorToEarth(0. /* -_heading*/ , 0., 0., -1.);      // the compass is reading gibberish
  AE35.set_SensorToFrame(0., 1.,1.,1.);                 // the MPU9150 is tilted -17 degrees from vertical
  
    time_t t = now();
    _gps_debug_temporary += 1;            // 150 laps to simulate time to acquire time base and lat/lon 
    if (_gps_debug_temporary > 30)       // but we'll use the repetition to average the IMU readings
    {
        _latitude  =  44.884860;          // grid square EN34FV 
        _longitude = -93.551492;
        p13.setLocation(_longitude,_latitude,0.0);
        // p13.setLocation(-64.375, 45.8958,0.0); // Sackville, NB
        p13.setTime((int)year(t),(int)month(t),(int)day(t),(int)hour(t),(int)minute(t),(int)second(t));
        _stateTransitionFlag |= GPS_LOCK;
    }
}


void satTrackButtons()
{

#ifdef FUNCTIONTRACETOSERIAL
    Serial.println(__func__);
#endif

    if ((_lcd_button_debounce == LCD_BUTTON_NONE) || (_lcd_button_debounce == 0)) return;

    if (AE35.get_TrueIfHomeSet())
    {
        // user commands azimuth and angle in degrees

        switch(_lcd_button_debounce)
        {
        case LCD_BUTTON_UP:
            AE35.AddDegreesToElevation(+1.0);
            break;
        case LCD_BUTTON_DOWN:
            AE35.AddDegreesToElevation(-1.0);
            break;
        case LCD_BUTTON_RIGHT:
            AE35.AddDegreesToAzimuth(+1.0);
            break;
        case LCD_BUTTON_LEFT:
            AE35.AddDegreesToAzimuth(-1.0);
            break;
        };
    }
    else
    {

        switch(_lcd_button_debounce)
        {
        case LCD_BUTTON_UP:
            AE35.AddStepsToElevation(+10);
            break;
        case LCD_BUTTON_DOWN:
            AE35.AddStepsToElevation(-10);
            break;
        case LCD_BUTTON_LEFT:
            AE35.AddStepsToAzimuth(-10);
            break;
        case LCD_BUTTON_RIGHT:
            AE35.AddStepsToAzimuth(+10);
            break;
        };

        // _commandElevation = _commandAzimuth = 0;

    };
};



/*
 * Display:        STATE   HH:MM:SS
 *                 SAT AL:00 AZ:000
 */

/****
INIT->CNFG->DRIV->HOME->TRAC->STOP
                        STOP->TRAC
****/
void displayMagnetometer() {

   char buffer[256];
   lcd.setCursor(0,0);
   lcd.print("MAGNETOM");
   sprintf(buffer,"X:%3dY:%3dZ:%3d",_compX,_compY,_compZ);
   lcd.setCursor(0,1);
   lcd.print(buffer);
}


void displayAccelerometer() {

   char buffer[256];
   // lcd.setCursor(0,0);
   // lcd.print("ACCELEROM");
   sprintf(buffer,"X:%3dY:%3dZ:%3d",(int)_accX,(int)_accY,(int)_accZ);
   lcd.setCursor(0,1);
   lcd.print(buffer);
}
void displayHdgRollPitch() {
   char buffer[256];
   lcd.setCursor(0,0);
   lcd.print("MAG HEADING:");
   sprintf(buffer,"%3d",_heading);
   lcd.print(_heading); 
   lcd.setCursor(0,1);
   lcd.print("ROLL");
   sprintf(buffer,"%+02d",(int)_roll);
   lcd.print(buffer);
   lcd.print("PITCH");
   sprintf(buffer,"%+02d",(int)_pitch);
   lcd.print(buffer);
}

void displayTimeHHMMSST() {

   char buffer[256];

   time_t t = now();
   unsigned long int milliseconds = nowMillis();

   int tenths = round((double)milliseconds / 100.0)*1000;  
   lcd.setCursor(6,0);
   sprintf(buffer,"%02d:%02d:%02d.%1d",hour(t),minute(t),second(t),tenths);
   lcd.print(buffer);
}
void displayState() {
   lcd.setCursor(0,0);
   lcd.print(_stateLabels[_state]);
}

void displaySatellite() {

   lcd.setCursor(0,1);
   lcd.print(_target);
}

void displayTargetAzElDeg() {

   lcd.setCursor(4,1);
   lcd.print("A     E     ");
   lcd.setCursor(5,1);
   lcd.print(AE35.get_AzTarget(),1);
   lcd.setCursor(11,1);
   lcd.print(AE35.get_ElTarget(),1);
}

void displayTargetAzElSteps() {

   char buffer[256];

   sprintf(buffer,"A%05dE%05d",(int)AE35.get_AzTargetMotorSteps(),(int)AE35.get_ElTargetMotorSteps());
   lcd.setCursor(4,1);
   lcd.print(buffer);
}

void displaySatelliteAzElDeg() {
     lcd.setCursor(4,1);
   lcd.print("A     E     ");
   lcd.setCursor(5,1);
   lcd.print(p13.AZ,1);
   lcd.setCursor(11,1);
   lcd.print(p13.EL,1);
}



void satTrackLCD()
{
switch(_state) {
    case INIT:
    case CNFG:
        lcd.clear();
        displayState();
        displayTimeHHMMSST();
        break;
   case GPSL:
#if 0
        lcd.clear();
        displayState();
        displayTimeHHMMSST();
        displayAccelerometer();
#endif
        displayHdgRollPitch();
        break;
   case DRIV:
        lcd.clear();
        displayState();
        displayTimeHHMMSST();
        if (AE35.get_TrueIfHomeSet()) displayTargetAzElDeg(); else displayTargetAzElSteps();
        break;
   case TRAC:
   case HOME:
   case STOP:

        lcd.clear();
        displayState();
        displayTimeHHMMSST();
        displaySatellite();
        //if (AE35.get_TrueIfHomeSet()) displayTargetAzElDeg(); else displayTargetAzElSteps();
        if (AE35.get_TrueIfHomeSet()) displaySatelliteAzElDeg(); else displayTargetAzElSteps();
        break;

    case SERL:
        lcd.clear();
        displayState();
        lcd.setCursor(4,1);
        lcd.print("          ");
        displayTargetAzElDeg();
    };
}

/********************************************************************************************************
 *
 * Antenna Gimbal Motor Control
 *
 *******************************************************************************************************/

// Motor shield has two motor ports, now we'll wrap them in an AccelStepper object


// Define some steppers and the pins the will use

AF_Stepper motor1(ELEVATION_MOTOR_STEPS_PER_REV,1);
AF_Stepper motor2(AZIMUTH_MOTOR_STEPS_PER_REV,2);

void forwardstep_motor1()
{
    motor1.onestep(BACKWARD, SINGLE);     // note with free spinning exec, consider INTERLEAVE motor for elevation
}
void backwardstep_motor1()
{
    motor1.onestep(FORWARD, SINGLE);      // rewire the motor later
}

void forwardstep_motor2()
{
    motor2.onestep(FORWARD, /* SINGLE */ INTERLEAVE);    
}
void backwardstep_motor2()
{
    motor2.onestep(BACKWARD, /* SINGLE */ INTERLEAVE);
}


AccelStepper azimuthMotor(forwardstep_motor2, backwardstep_motor2);
AccelStepper elevationMotor(forwardstep_motor1, backwardstep_motor1);

void switchCmd()
{
      if ((_limitSwitch & LIMIT_SWITCH_EL_MAX) == LIMIT_SWITCH_EL_MAX)
          {
          elevationMotor.stop();        // all engines stop 
          azimuthMotor.stop();
          // Keep the L293D from frying while we debug, antenna position may drift
          motor2.release();
          motor1.release();

          // elevation is homed at maximum (level to horizon is zero)
          elevationMotor.setCurrentPosition(ELEVATION_MOTOR_MAX_STEPS);
          azimuthMotor.setCurrentPosition(0);

          AE35.setHome(); 
          _limitSwitch = _limitSwitch & !LIMIT_SWITCH_EL_MAX;       // fix this later, home operation is clumsy
          _stateTransitionFlag |= HOME_POSITION_SET;
          }
}

void movAzElMotors()
{
#ifdef FUNCTIONTRACETOSERIAL
    Serial.println(__func__);
#endif

// executive used to run every 20ms, 500 times a second maximum, now it spins freely 
//
    azimuthMotor.run();
    elevationMotor.run();
}

void movAzElMotorsSpeed() 
{
#ifdef FUNCTIONTRACETOSERIAL
    Serial.println(__func__);
#endif

// don't use acceleration function during satellite tracking  
// (experiment for smooth motion)
//
    azimuthMotor.runSpeed(); 
    elevationMotor.runSpeed();
}

void updateSat()
{
   double seconds;
#ifdef FUNCTIONTRACETOSERIAL
    Serial.println(__func__);
#endif

    time_t t = now();
    unsigned long int milliseconds = nowMillis();
   // millseconds = round((double)milliseconds / 100.0)*100.0; 
    seconds = (double)second(t) + ((double)(milliseconds)/1000.0); 
   
    // modified PLAN13 library, use floating point seconds to smooth antenna motion 
    p13.setPrecisionTime((int)year(t),(int)month(t),(int)day(t),(int)hour(t),(int)minute(t),seconds);
    // p13.setTime((int)year(t),(int)month(t),(int)day(t),(int)hour(t),(int)minute(t),(int)second(t));
    p13.satvec();
    p13.rangevec();
    AE35.set_targetAz(p13.AZ); 
    AE35.set_targetEl(p13.EL);

#ifdef DEBUGP13
    debugp13update();
#endif

}

void stepsAzElToMotors()
{
     
    azimuthMotor.moveTo(AE35.get_AzTargetMotorSteps());       // Note: original executive was throttled to a maximum of 50 steps/second
    azimuthMotor.setMaxSpeed(_maxAzSpeed);               // 50 steps/sec / (1180.0 / 90.0) = ~4deg/sec
    azimuthMotor.setAcceleration(20.0);         // "but they're so small they're evading our turbolasers."
    elevationMotor.moveTo(AE35.get_ElTargetMotorSteps());
    elevationMotor.setMaxSpeed(_maxElSpeed);             // 50 steps/second / (1760.0 / 45.0) = ~1.3deg/sec
    elevationMotor.setAcceleration(20.0);       //
}

void currentMotorPos()
{
#ifdef FUNCTIONTRACETOSERIAL
    Serial.println(__func__);
#endif

// current antenna mounting loads the gimbal too much and elevation can slide if motors off
// keep an eye on drive circuit. However, stacked L293D chips may fry so release motor if we are really stopped
// 
   if (azimuthMotor.distanceToGo() == 0) {
      if ((millis() - AE35.get_AzIdleTimeout()) > MOTOR_MS_IDLE_BEFORE_RELEASE) motor2.release();
   }
   else {
      AE35.setAzIdleTimeout(millis()); 
   }

   if (elevationMotor.distanceToGo() == 0) {
      if ((millis() - AE35.get_ElIdleTimeout()) > MOTOR_MS_IDLE_BEFORE_RELEASE) motor1.release();
   }
   else {
      AE35.setElIdleTimeout(millis()); 
   } 
};

void displaySel() { } ;
void menuSel()  { };

void checkSerial1() {
  if (Serial.available()>0) { 
      _stateTransitionFlag |= SERIAL_DATA; 
     }
  // once we transition to Serial we never go back to MASTER modes
}

//
// AZ    Azimuth     number - 1 decimal place
// EL    Elevation   number - 1 decimal place
// SA    Stop azimuth moving
// SE    Stop elevation moving
//
void processSerial1() {
    if (!(Serial.available() > 0)) return; 
    _target = "SLAVE";
    
    char c = (char)Serial.read();
    // add it to the inputString:
    _serialInput += c;

// Easycomm I
//
// The host PC issues a single line command as follows -:
// AZaaa.a ELeee.e UPuuuuuuuuu UUU DNddddddddd DDD
//
   if (c == '\n') { 
    AE35.set_targetAz(_serialInput.substring(_serialInput.indexOf("AZ")+2,_serialInput.indexOf(" ",_serialInput.indexOf("AZ"))).toFloat());
    AE35.set_targetEl(_serialInput.substring(_serialInput.indexOf("EL")+2,_serialInput.indexOf(" ",_serialInput.indexOf("AZ"))).toFloat());
   _serialInput = "";
   }

//
// Easycomm II
//   
#if 0
   if (((c == '\n') || (c == '\r') || (c == ' ')) && (_serialInput.length()>2 )) {
        _serialInput.toUpperCase();
        if (_serialInput.startsWith("AZ")) {
             _commandAzimuth = _targetAzimuth = _serialInput.substring(2,_serialInput.length()-2).toFloat(); 
             _commandAzSteps = round(_commandAzimuth * AZIMUTH_GIMBAL_STEPS_PER_DEGREE);  
             AE35.set_targetAz(_serialInput.substring(2,_serialInput.length()-2).toFloat());
             _serialInput = ""; 
        } else if (_serialInput.startsWith("EL")) {
             _commandElevation = _targetElevation = _serialInput.substring(2,_serialInput.length()-2).toFloat(); 
             _commandElSteps = round(_commandElevation * ELEVATION_GIMBAL_STEPS_PER_DEGREE); 
              AE35.set_targetEl(_serialInput.substring(2,_serialInput.length()-2).toFloat());
             _serialInput = ""; 
        } 
    }
#endif 
}
/********************************************************************************************************
 *
 * Define Cyclic Executive State Machine
 *
 *******************************************************************************************************/



#define UPDNRTLF    (LCD_BUTTON_UP | LCD_BUTTON_DOWN | LCD_BUTTON_LEFT | LCD_BUTTON_RIGHT)
#define SEL         (LCD_BUTTON_SELECT)
#define HOM         (HOME_POSITION_SET)
#define SER         (SERIAL_DATA)

unsigned short int transitions[NUMBEROFSTATES][NUMBEROFSTATES] =
{
    /*                   INIT        CNFG         GPSL       DRIV        TRAC        HOM       SERL     STOP  */
    /* INIT   */        NEVER,      ALWAYS,       NEVER,     NEVER,     NEVER,     NEVER,      NEVER,   NEVER,
    /* CNFG   */        NEVER,      NEVER,        ALWAYS,    NEVER,     NEVER,     NEVER,      NEVER,   NEVER,
    /* GPSL   */        NEVER,      NEVER,        NEVER,    GPS_LOCK,   NEVER,     NEVER,      NEVER,   NEVER,
    /* DRIV   */        NEVER,      NEVER,        NEVER,     NEVER,     NEVER,     SEL,        SER,     NEVER,
    /* TRAC   */        NEVER,      SEL,          NEVER,    UPDNRTLF,   NEVER,     NEVER,      SER,     NEVER,
    /* HOME   */        NEVER,      NEVER,        NEVER,     NEVER,     HOM,       NEVER,      NEVER,   NEVER,
    /* SERL   */        NEVER,      NEVER,        NEVER,     NEVER,     NEVER,     NEVER,      NEVER,   NEVER, 
    /* STOP   */        NEVER,      NEVER,        NEVER,     NEVER,     NEVER,     NEVER,      NEVER,   NEVER,
};

/*    INIT         CNFG        GPSL        DRIV          TRAC        HOME        SERL      STOP  */
void (*slow[5][NUMBEROFSTATES])()
    = {   noop,          noop,        satTrackLCD,  satTrackLCD,   satTrackLCD,   satTrackLCD, satTrackLCD,   noop,
          noop,          noop,          pollGPS,   currentMotorPos, updateSat,         noop,        noop,       noop,
          noop,          noop,           noop,        noop,       stepsAzElToMotors,   noop,        noop,       noop,
          noop,          noop,           noop,        noop,       currentMotorPos,     noop,        noop,       noop,
          noop,          noop,           noop,        noop,            noop,           noop,        noop,       noop
      };
      
void (*medium[5][NUMBEROFSTATES])()
    /*  INIT           CNFG            GPSL          DRIV           TRAC           HOM          SERL        STOP  */
    = {   noop,          noop,       noop,  satTrackButtons, satTrackButtons, stepsAzElToMotors, processSerial1,   noop,
          noop,          noop,       noop,  stepsAzElToMotors, checkSerial1,     noop,    stepsAzElToMotors,       noop,
          noop,          noop,       noop,         noop,          noop,           noop,        noop,               noop,
          noop,          noop,       noop,         noop,          noop,           noop,        noop,               noop,
          noop,          noop,       noop,         noop,          noop,           noop,        noop,               noop
      };

void (*fast[5][NUMBEROFSTATES])()
    = {  readBtn,       readBtn,        readBtn,     readBtn,       readBtn,      readBtn,       readBtn,    noop,
         noop,           noop,           noop,         noop,          noop,         noop,         noop,      noop,
         noop,           noop,           noop,         noop,          noop,         noop,         noop,      noop,
         noop,           noop,           noop,         noop,          noop,         noop,         noop,      noop,
         noop,           noop,           noop,         noop,          noop,         noop,         noop,      noop
      };

void (*always[5][NUMBEROFSTATES])()
    = {  noop,           noop,           noop,     movAzElMotors,  movAzElMotors, readSwtch,  movAzElMotors,    noop,
         noop,           noop,           noop,         noop,          noop,        switchCmd,     noop,         noop,
         noop,           noop,           noop,         noop,          noop,     movAzElMotors,    noop,         noop,
         noop,           noop,           noop,         noop,          noop,         noop,         noop,         noop,
         noop,           noop,           noop,         noop,          noop,         noop,         noop,         noop
      };


void manualSatelliteInput_blocking()
{
    int index = 1;
    unsigned short int initial_button_state;
    int debounce_counter = 4;
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(F("Set Satellite"));
    lcd.blink();

    while (!_flagSatelliteSet)
    {

        initial_button_state = _lcd_button_debounce;
        readBtn();
        
        if ((initial_button_state != _lcd_button_debounce) /* || (debounce_counter++ % 3)== 0) */)
        {

            if        (_lcd_button_debounce == LCD_BUTTON_UP)
            {
                index+=3;
            }
            else if (_lcd_button_debounce == LCD_BUTTON_DOWN)
            {
                index=max(index-3,1);
            }
            else if (_lcd_button_debounce == LCD_BUTTON_SELECT)
            {
                _flagSatelliteSet = true;
                _target = _keps[index];
            }
        delay(50);
        }

        if (_keps[index].equals("END")) index=1;

        lcd.setCursor(0,1);
        lcd.print(_keps[index]);
        lcd.print("   ");

    }
    lcd.noBlink();
    delay(50);
    _stateTransitionFlag = 0;
    _lcd_button_debounce = _lcd_button_now = 0;
}


//
// sloppy function to manually input date until GPS module integrated
//
void manualDateInput_blocking()
{
    int Month=1;
    int Day=5;
    int Year = 2017;
    int indexPos = 0;
    int cursorPos = 0;
    unsigned short int initial_button_state;
    int debounce_counter=4;

    char displayString[24];

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(F("Set Date  (GMT)"));
    lcd.blink();

    while (!_flagDateSetManually)
    {

        initial_button_state = _lcd_button_debounce;
        readBtn();
        if ((initial_button_state != _lcd_button_debounce) /* || ((debounce_counter++ % 3) == 0) */)
        {

            if (_lcd_button_debounce == LCD_BUTTON_UP)
            {

                switch (indexPos)
                {
                case 0:
                    Month = (Month+1)%13;
                    break;
                case 1:
                    Day  +=10;
                    break;
                case 2:
                    Day  +=1;
                    break;
                case 3:
                    Year +=1;
                    break;
                }

            }
            else if (_lcd_button_debounce == LCD_BUTTON_DOWN)
            {

                switch (indexPos)
                {
                case 0:
                    if (Month>1)   Month-=1;
                    break;
                case 1:
                    if (Day>10)    Day-=10;
                    break;
                case 2:
                    if (Day>2)     Day-=1;
                    break;
                case 3:
                    if (Year>2015) Year -=1;
                    break;
                }

            }
            else if (_lcd_button_debounce == LCD_BUTTON_LEFT)
            {

                indexPos = max(indexPos-1,0);

            }
            else if (_lcd_button_debounce == LCD_BUTTON_RIGHT)
            {

                indexPos = min(indexPos+1,3);

            }
            else if (_lcd_button_debounce == LCD_BUTTON_SELECT)
            {
                time_t t=now();
                setTime(hour(t),minute(t),second(t),Day,Month,Year);
                _flagDateSetManually = 1;
            }
        }
        switch (Month)
        {
        case 1:
        case 3:
        case 5:
        case 7:
        case 10:
        case 12:
            Day = Day % (31+1);
            break;
        case 2:
            Day = Day % (28+1);
            break;
        case 4:
        case 6:
        case 9:
        case 11:
            Day = Day % (30+1);
            break;
        }

        if (Day<1) Day=1;

        sprintf(displayString,"%s %02d %04d",monthShortStr(Month), Day, Year);
        lcd.setCursor(0,1);
        lcd.print(displayString);

        cursorPos = indexPos;
        if (indexPos > 0) cursorPos+=3;
        if (indexPos > 2) cursorPos+=4;

        lcd.setCursor(cursorPos,1);
        delay(50);

    }
    lcd.noBlink();
    delay(50);
    _stateTransitionFlag = 0;
    _lcd_button_debounce = _lcd_button_now = 0;
}

//
// sloppy function to manually input time until GPS module integrated
//
void manualTimeInput_blocking()
{

    unsigned short int initial_button_state;
    int cursorPos = 0;
    int indexPos = 0;
    int tens = 0;
    int digit = 0;
    int hours, minutes, seconds;
    int H_____;
    int _H____;
    int __M___;
    int ___M__;
    int ____S_;
    int _____S;
    char displayString[24];

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(F("Set Time (GMT)"));
    lcd.blink();

    time_t t = now();

    hours = 23; /* hour(t); */
    minutes = 25; /* minute(t); */
    seconds = second(t);

    while (!_flagTimeSetManually)
    {

        H_____ = hours/10;
        _H____ =hours % 10;
        __M___ = minutes/10;
        ___M__ =minutes % 10;
        ____S_ = seconds/10;
        _____S =seconds % 10;

        initial_button_state = _lcd_button_debounce;

        readBtn();
        if (initial_button_state != _lcd_button_debounce)
        {

            if (_lcd_button_debounce == LCD_BUTTON_UP)
            {

                switch (indexPos)
                {
                case 0:
                    H_____ =(H_____+1)%3;
                    break;
                case 1:
                    _H____ =(_H____+1)%9;
                    if (H_____>1) _H____ = max(_H____,4);
                    break;
                case 2:
                    __M___ =(__M___+1)%6;
                    break;
                case 3:
                    ___M__ =(___M__+1)%10;
                    break;
                case 4:
                    ____S_ =(____S_+1)%6;
                    break;
                case 5:
                    _____S =(_____S+1)%10;
                    break;
                }

            }
            else if (_lcd_button_debounce == LCD_BUTTON_DOWN)
            {

                switch (indexPos)
                {
                case 0:
                    H_____ -=1;
                    if (H_____<0) H_____ = 2;
                    break;
                case 1:
                    _H____ -=1;
                    if (_H____<0) _H____ = 9;
                    break;
                case 2:
                    __M___ -=1;
                    if (__M___<0) __M___ = 5;
                    break;
                case 3:
                    ___M__ -=1;
                    if (___M__<0) ___M__ = 9;
                    break;
                case 4:
                    ____S_ -=1;
                    if (____S_<0) ____S_ = 5;
                    break;
                case 5:
                    _____S -=1;
                    if (_____S<0) _____S = 9;
                    break;
                }

            }
            else if (_lcd_button_debounce == LCD_BUTTON_LEFT)
            {
                indexPos = max(indexPos-1,0);
            }
            else if (_lcd_button_debounce == LCD_BUTTON_RIGHT)
            {
                indexPos = min(indexPos+1,5);
            }
            else if (_lcd_button_debounce == LCD_BUTTON_SELECT)
            {
                hours   = H_____*10+_H____;
                minutes = __M___*10+___M__;
                seconds = ____S_*10+_____S;
                time_t d = now();
                setTime(hours,minutes,seconds,day(d),month(d),year(d));
                _flagTimeSetManually = 1;
            }
        }

        hours   = H_____*10+_H____;
        minutes = __M___*10+___M__;
        seconds = ____S_*10+_____S;

        sprintf(displayString,"%02d:%02d:%02d",hours, minutes, seconds);
        lcd.setCursor(0,1);
        lcd.print(displayString);

        cursorPos = indexPos;
        if (indexPos > 1) cursorPos++;
        if (indexPos > 3) cursorPos++;

        lcd.setCursor(cursorPos,1);
        delay(50);
    }
    lcd.noBlink();
    _stateTransitionFlag = 0;
    _lcd_button_debounce = _lcd_button_now = 0;
    delay(50);
}

void initAE35()
{
#ifdef FUNCTIONTRACETOSERIAL
    Serial.println(__func__);
#endif
    String temp; 
    _lastms = _fastTimer = _mediumTimer = _slowTimer = 0;
    AE35.set_targetAz(0.);
    AE35.set_targetEl(0.);

    _stateTransitionFlag = 0;

    pinMode(31,INPUT_PULLUP);         // elevation MAX limit switch breadboarded into digital 31
    setTime(23,30,00,11,10,15);
    Serial.begin(9600);
    Serial.print(F("Hello World!"));
    lcd.begin(16, 2);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(F("AE-35 Antenna"));
    lcd.setCursor(0,1);
    lcd.print(__DATE__);
    _serialInput = "";
    delay(3000);
    azimuthMotor.setCurrentPosition(0);
    elevationMotor.setCurrentPosition(0);
    // azimuthMotor.setMaxSpeed(MAX_AZ_SPEED_MANUAL);               // 50 steps/sec / (1180.0 / 90.0) = ~4deg/sec
    // azimuthMotor.setAcceleration(200.0);                         // "but they're so small they're evading our turbolasers."
    // elevationMotor.setMaxSpeed(MAX_EL_SPEED_MANUAL);             // 50 steps/second / (1760.0 / 45.0) = ~1.3deg/sec
    // elevationMotor.setAcceleration(200.0);       //
  
  Wire.begin();
  accelGyroMag.initialize();

  AE35.set_ElAntUpperLimitDeg(55.);                     
  AE35.set_ElAntLowerLimitDeg(5.);
  AE35.set_AzAntUpperLimitDeg(+190.);  
  AE35.set_AzAntLowerLimitDeg(-190.); 
  AE35.set_AzMotorUpperLimitSteps(AZIMUTH_MOTOR_MAX_STEPS);
  AE35.set_AzMotorLowerLimitSteps(AZIMUTH_MOTOR_MIN_STEPS);
  AE35.set_ElMotorUpperLimitSteps(ELEVATION_MOTOR_MAX_STEPS);
  AE35.set_ElMotorLowerLimitSteps(ELEVATION_MOTOR_MIN_STEPS);
  AE35.set_ElAntStepsPerDegree(ELEVATION_GIMBAL_STEPS_PER_DEGREE);
  AE35.set_AzAntStepsPerDegree(AZIMUTH_GIMBAL_STEPS_PER_DEGREE);
  AE35.debug();

// temporarily allow user to input an approximate time

    manualTimeInput_blocking();
    manualDateInput_blocking();


//
};

void configAE35()
{
#ifdef FUNCTIONTRACETOSERIAL
    Serial.println(__func__);
#endif
    int i = 1;
    boolean found = 0;

    double epochTime,inclination,eccentricity, rightAscension,argOfPerigee,
           meanAnomoly, meanMotion,decayRate, revolutionNumber, epochYear, orbitNumber;
    long   lineNo, elemSetNo;
    String catNo, satellite, designator;

    // sloppy function to input satellite choice
    _flagSatelliteSet = 0;
    manualSatelliteInput_blocking();
    
    while (!found)
    {
        if (_keps[i].equals(_target)) found=true;
        else i++;
    }

    epochTime =          _keps[i+1].substring(21-1,32).toFloat();
    // String decay = _keps[i+1].substring(54-1,54) + "0." + _keps[i+1].substring(55-1,59) + "E-" + _keps[i+1].substring(61-1,61);  
    double mant = _keps[i+1].substring(54-1,59).toFloat();
           mant =  mant / pow(10.0,(int)(log10(abs(mant))+1));
    double expo = _keps[i+1].substring(60-1,61).toFloat(); 
    decayRate = mant*pow(10.0,expo);
   
    epochYear =          _keps[i+1].substring(19-1,20).toFloat() + 2000.0;

    inclination =        _keps[i+2].substring(9-1,16).toFloat();
    rightAscension =     _keps[i+2].substring(18-1,25).toFloat();
    String eccen = "0." + _keps[i+2].substring(27-1,33);                                          // note the leading zero
    eccentricity = eccen.toFloat();
    argOfPerigee =       _keps[i+2].substring(35-1,42).toFloat();
    meanAnomoly =        _keps[i+2].substring(44-1,51).toFloat();
    meanMotion =         _keps[i+2].substring(53-1,63).toFloat();
    revolutionNumber =   _keps[i+2].substring(64-1,68).toFloat();

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(F("epochYear"));
    lcd.setCursor(0,1);
    lcd.print(epochYear);
    delay(200);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(F("epochTime"));
    lcd.setCursor(0,1);
    lcd.print(epochTime);
    delay(200);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(F("decayRate * 10^6"));
    lcd.setCursor(0,1);
    lcd.print((decayRate * 1000000.0));
    delay(200);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(F("inclination"));
    lcd.setCursor(0,1);
    lcd.print(inclination);
    delay(200);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(F("rightAscension"));
    lcd.setCursor(0,1);
    lcd.print(rightAscension); 
    delay(200); 
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(F("eccentr. * 10^6"));
    lcd.setCursor(0,1);
    lcd.print((eccentricity * 1000000.0)); 
    delay(200);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(F("argOfPerigee"));
    lcd.setCursor(0,1);
    lcd.print(argOfPerigee); 
    delay(200);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(F("meanAnomoly"));
    lcd.setCursor(0,1);
    lcd.print(meanAnomoly); 
    delay(200);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(F("meanMotion"));
    lcd.setCursor(0,1);
    lcd.print(meanMotion); 
    delay(200);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(F("revolutionNumber"));
    lcd.setCursor(0,1);
    lcd.print(revolutionNumber); 
    delay(200);
    

    time_t t = now();
    p13.setTime((int)year(t),(int)month(t),(int)day(t),(int)hour(t),(int)minute(t),(int)second(t));
    // p13.setTime(2009, 10, 1, 19, 5, 0); //Oct 1, 2009 19:05:00 UTC
    _latitude  =  44.884860;
    _longitude = -93.551492;
    p13.setLocation(_longitude,_latitude,0.0);  // just set a sensible lat/lon

    p13.setElements(           epochYear,
                               epochTime,
                               inclination,
                               rightAscension,
                               eccentricity,
                               argOfPerigee,
                               meanAnomoly,
                               meanMotion,
                               decayRate,
                               revolutionNumber,
                               180.0);

  // p13.setElements(2009, 232.55636497, 98.0531, 238.4104, 83652*1.0e-7, 290.6047,
  // 68.6188, 14.406497342, -0.00000001, 27022, 180.0); //fairly recent keps for AO-51 
    
    p13.initSat();

#ifdef DEBUGP13
    debugP13keps();
#endif
    AE35.debug(); 
};

void gpsLockAE35()
{
#ifdef FUNCTIONTRACETOSERIAL
    Serial.println(__func__);
#endif

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(F("AE-35 Antenna"));
    lcd.setCursor(0,1);
    lcd.print(F("configure GPS"));
};

void driveAE35()
{
#ifdef FUNCTIONTRACETOSERIAL
    Serial.println(__func__);
#endif
    _maxElSpeed = MAX_EL_SPEED_MANUAL;
    _maxAzSpeed = MAX_AZ_SPEED_MANUAL;
};

void trackAE35()
{
#ifdef FUNCTIONTRACETOSERIAL
    Serial.println(__func__);
#endif

    _maxElSpeed = MAX_EL_SPEED_TRACK;
    _maxAzSpeed = MAX_AZ_SPEED_TRACK;
    
    time_t t = now();
    p13.setTime(year(t),month(t),day(t),hour(t),minute(t),second(t));
    p13.satvec();
    p13.rangevec();

#ifdef DEBUGP13
    debugP13keps();
#endif
    AE35.set_targetAz(p13.AZ);
    AE35.set_targetEl(p13.EL);
};

void homeAE35()
{
#ifdef FUNCTIONTRACETOSERIAL
    Serial.println(__func__);
#endif
    
    if (AE35.get_TrueIfHomeSet()) {
      _stateTransitionFlag |= HOME_POSITION_SET;
      return;
    }

    _maxElSpeed = MAX_EL_SPEED_HOME;
    _maxAzSpeed = MAX_AZ_SPEED_HOME;

    azimuthMotor.setCurrentPosition(0);       // will re-zero when actually home
    elevationMotor.setCurrentPosition(0);
    AE35.set_targetEl(0.);
    AE35.set_targetAz(0.);
    AE35.AddStepsToElevation(ELEVATION_MOTOR_MAX_STEPS);
};

void stopAE35()
{
#ifdef FUNCTIONTRACETOSERIAL
    Serial.println(__func__);
#endif
};

void serialAE35() { };

void (*states[NUMBEROFSTATES])() = { initAE35, configAE35, gpsLockAE35, driveAE35, trackAE35, homeAE35, serialAE35, stopAE35 } ;

void updateState()
{

#ifdef FUNCTIONTRACETOSERIAL
    Serial.println(__func__);
#endif
    int i,j;
    unsigned short int flag;
    _stateTransitionFlag |= ALWAYS;

    for (i=0; i<NUMBEROFSTATES; i++)
    {
        if (transitions[_state][i] & _stateTransitionFlag)
        {
            _stateTransitionFlag &= !(transitions[_state][i] & _stateTransitionFlag);
            _state = i;
            // lcd.setCursor(0,1); lcd.print(i);
            (*states[_state])();
            break;
        }
    }
}


void bail() { }; // could put a unique function in last slot of each table so executive loop bails out, eliminates frivilous noops

void noop() { };

void setup()
{
#ifdef FUNCTIONTRACETOSERIAL
    Serial.println(__func__);
#endif

    _state = INIT;
    (*states[_state])();
}

// cyclic executive

void loop ()
{

    short int           i;
    unsigned long int   elapsed;
    time_t              t;
    t = now(); 

    updateState();

    _ms = millis();
    elapsed = _ms - _lastms;
 
    _fastTimer += elapsed;
    _mediumTimer += elapsed;
    _slowTimer += elapsed;
    _lastms = _ms;

    for (i=0; i<5; i++) (*always[i][_state])();
    
    if (_fastTimer >= 10)
    {
        for (i=0; i<5; i++) (*fast[i][_state])();
        _fastTimer = 0;
    }
    if (_mediumTimer >= 50)
    {
        for (i=0; i<5; i++) (*medium[i][_state])();
        _mediumTimer = 0;
    }
    if (_slowTimer >= 100)
    {
        for (i=0; i<5; i++) (*slow[i][_state])();
        _slowTimer = 0;
    }
    
    // NO CHECK FOR 1202 error
    // delay (max((long)(20 - (millis() - _ms)),0));  // original exec would sleep off the unused time every 20ms  
                                                      // now always try to advance the stepper motors if the executive is spinning 
                                                      // timing tests indicated 12/1/2015 version loops every 1-2ms
}


void debugP13keps()
{

    Serial.print(F("p13.LA = "));
    Serial.println(p13.LA,8);
    Serial.print(F("p13.LO = "));
    Serial.println(p13.LO,8);
    Serial.println("");
    Serial.print(F("p13.DN = "));
    Serial.println(p13.DN,8);
    Serial.print(F("p13.TN = "));
    Serial.println(p13.TN);
    Serial.println("");

    Serial.print(F("p13.YE = "));
    Serial.println(p13.YE,8);
    Serial.print(F("p13.TE = "));
    Serial.println(p13.TE,8);
    Serial.print(F("p13.IN = "));
    Serial.println(p13.IN,8);
    Serial.print(F("p13.RA = "));
    Serial.println(p13.RA,8);
    Serial.print(F("p13.EC = "));
    Serial.println(p13.EC,8);
    Serial.print(F("p13.WP = "));
    Serial.println(p13.WP,8);
    Serial.print(F("p13.MA = "));
    Serial.println(p13.MA,8);
    Serial.print(F("p13.MM = "));
    Serial.println(p13.MM,8);
    Serial.print(F("p13.M2 = "));
    Serial.println(p13.M2,8);
    Serial.print(F("p13.RV = "));
    Serial.println(p13.RV,8);
    Serial.print(F("p13.ALON="));
    Serial.println(p13.ALON,8);
    Serial.println("");

    Serial.print(F("p13.DE = "));
    Serial.println(p13.DE,8);
    Serial.print(F("p13.TE = "));
    Serial.println(p13.TE,8);
    Serial.println("");
}


void debugp13update()
{
#ifdef DEBUGP13
    p13.printdata();
    Serial.println("");
#endif 
}
// CELESTRAK TLEs from NORAD 12/18/2016

#if 0
OSCAR 7 (AO-7)          
1 07530U 74089B   16353.16578859 -.00000043  00000-0  52829-5 0  9999
2 07530 101.6041 319.6214 0011954 315.0561 113.1100 12.53624611926170
UOSAT 2 (UO-11)         
1 14781U 84021B   16352.71879893  .00000088  00000-0  17156-4 0  9995
2 14781  97.7248  38.8521 0009940  77.0385 283.1938 14.82838888763456
LUSAT (LO-19)           
1 20442U 90005G   16352.86320363  .00000019  00000-0  23210-4 0  9996
2 20442  98.5604 285.3733 0011209 328.5641  31.4872 14.32842726405653
EYESAT-1 (AO-27)        
1 22825U 93061C   16352.82971124 -.00000009  00000-0  14230-4 0  9993
2 22825  98.7865 310.4476 0009080 135.8166 224.3743 14.29980011211482
ITAMSAT (IO-26)         
1 22826U 93061D   16352.13280203 -.00000008  00000-0  14380-4 0  9994
2 22826  98.7806 309.8476 0009706 129.4320 230.7722 14.30306968211557
RADIO ROSTO (RS-15)     
1 23439U 94085A   16351.95792303 -.00000069  00000-0 -73674-3 0  9998
2 23439  64.8130 153.9114 0167486 276.7925 257.9672 11.27565212904866
JAS-2 (FO-29)           
1 24278U 96046B   16353.22141482 -.00000010  00000-0  22660-4 0  9992
2 24278  98.5836 254.3344 0351334  86.1335 277.9890 13.53072738  4403
TECHSAT 1B (GO-32)      
1 25397U 98043D   16352.79328526 -.00000007  00000-0  15780-4 0  9992
2 25397  98.5900 293.1536 0001823 130.3779 229.7560 14.23610874957987
ISS (ZARYA)             
1 25544U 98067A   16353.17824509  .00001755  00000-0  34038-4 0  9994
2 25544  51.6443 220.6275 0006286 346.5926 159.5138 15.53908628 33607
PCSAT (NO-44)           
1 26931U 01043C   16352.73824227 -.00000039  00000-0  16768-4 0  9996
2 26931  67.0498 199.3512 0005867 280.2751  79.7689 14.30439777794391
SAUDISAT 1C (SO-50)     
1 27607U 02058C   16353.21634419 -.00000019  00000-0  17909-4 0  9996
2 27607  64.5558 318.5513 0045871 171.6455 188.5422 14.75261646752418
CUTE-1 (CO-55)          
1 27844U 03031E   16352.96308422  .00000046  00000-0  40354-4 0  9993
2 27844  98.6915 358.6025 0010274 146.6998 213.4828 14.22002242698562
CUBESAT XI-IV (CO-57)   
1 27848U 03031J   16351.99908332  .00000024  00000-0  31029-4 0  9990
2 27848  98.7009 357.8311 0010024 157.7675 202.3939 14.21621699698313
MOZHAYETS 4 (RS-22)     
1 27939U 03042A   16353.23948638  .00000129  00000-0  31652-4 0  9993
2 27939  97.9122 138.1189 0014328  81.9932  14.8811 14.66500701706795
CUBESAT XI-V (CO-58)    
1 28895U 05043F   16353.17946338  .00000140  00000-0  35737-4 0  9997
2 28895  97.8438 158.0405 0016802   1.0861 359.0376 14.63314698594016
CUTE-1.7+APD II (CO-65) 
1 32785U 08021C   16352.52263135  .00000209  00000-0  28564-4 0  9990
2 32785  97.5842  18.9324 0014351  54.4218 305.8334 14.87754666467880
DELFI-C3 (DO-64)        
1 32789U 08021G   16353.19031694  .00001329  00000-0  99933-4 0  9993
2 32789  97.5835  49.0768 0012868   2.3806 357.7479 15.04182769469651
SEEDS II (CO-66)        
1 32791U 08021J   16353.14053698  .00000241  00000-0  30437-4 0  9995
2 32791  97.5836  23.3066 0014406  40.3943 319.8342 14.90032462468188
YUBILEINY (RS-30)       
1 32953U 08025A   16353.11182510  .00000010  00000-0  36040-5 0  9993
2 32953  82.5023 352.2674 0018270 324.0234  35.9615 12.43073887388921
PRISM (HITOMI)          
1 33493U 09002B   16353.17885858  .00000371  00000-0  39390-4 0  9990
2 33493  98.1902 221.7199 0015403 324.8134  35.2077 14.95358120428727
KKS-1 (KISEKI)          
1 33499U 09002H   16353.17345241  .00000140  00000-0  28839-4 0  9993
2 33499  98.3095 147.4946 0008333 225.1816 134.8715 14.75108313424676
SWISSCUBE               
1 35932U 09051B   16353.18869464  .00000134  00000-0  41442-4 0  9998
2 35932  98.4708 122.7496 0008820  89.3710 270.8498 14.55931384383989
BEESAT                  
1 35933U 09051C   16352.09095136  .00000115  00000-0  36865-4 0  9997
2 35933  98.4739 122.8347 0007013 103.4268 256.7711 14.56060282383925
ITUPSAT 1               
1 35935U 09051E   16352.11624129  .00000106  00000-0  35356-4 0  9996
2 35935  98.4849 123.1117 0009343  97.0916 263.1343 14.55261470383759
XIWANG-1 (HOPE-1)       
1 36122U 09072B   16353.21947918 -.00000056  00000-0 -36579-4 0  9998
2 36122 100.1216  21.4447 0006903 294.4994  65.5399 13.16337436336813
TISAT 1                 
1 36799U 10035E   16352.32109990  .00000430  00000-0  50442-4 0  9999
2 36799  98.0448 105.6363 0012159 198.5547 161.5228 14.90152289348696
JUGNU                   
1 37839U 11058B   16352.53333001  .00000282  00000-0  88353-5 0  9994
2 37839  19.9610 111.9097 0019085 341.3651 100.7999 14.12565633267985
SRMSAT                  
1 37841U 11058D   16352.54485157  .00000281  00000-0  91629-5 0  9997
2 37841  19.9706 149.7879 0011774 246.2916 174.4002 14.10560977267601
M-CUBED & EXP-1 PRIME   
1 37855U 11061F   16352.35232334  .00001185  00000-0  71228-4 0  9998
2 37855 101.7145 125.4442 0183168 258.1384  99.9234 15.02214242279488
HORYU 2                 
1 38340U 12025D   16352.04620538  .00000196  00000-0  37093-4 0  9995
2 38340  98.3692 343.6409 0012293 146.2028 213.9966 14.75209931246432
STRAND-1                
1 39090U 13009E   16353.14598101  .00000023  00000-0  23369-4 0  9993
2 39090  98.5665 194.1098 0010175 114.2372 245.9877 14.34938052199514
SOMP                    
1 39134U 13015E   16352.71206731  .00001095  00000-0  64860-4 0  9997
2 39134  64.8687  91.8343 0021070 306.2792  53.6387 15.17010215201949
BEESAT-2                
1 39136U 13015G   16351.76072614  .00001049  00000-0  66972-4 0  9999
2 39136  64.8701 103.1605 0022069 306.1343  53.7739 15.14459657201649
CUBEBUG-1 (CAPITAN BETO)
1 39153U 13018D   16353.18605300  .00000222  00000-0  36357-4 0  9999
2 39153  97.9930  80.4332 0015747 274.5715  85.3698 14.80055738196810
ZACUBE-1 (TSHEPISOSAT)  
1 39417U 13066B   16352.43865735  .00000227  00000-0  35245-4 0  9992
2 39417  97.6560  37.2484 0060440 143.2216 217.3168 14.80577781165884
TRITON-1                
1 39427U 13066M   16353.14820158  .00000158  00000-0  33608-4 0  9996
2 39427  97.6502  14.5428 0113923 207.9563 151.5494 14.67342067164494
GOMX 1                  
1 39430U 13066Q   16352.95622989  .00000142  00000-0  35280-4 0  9991
2 39430  97.6631 359.8654 0150804 255.2491 103.1958 14.58752790163509
HUMSAT-D                
1 39433U 13066T   16353.15989542  .00000303  00000-0  35755-4 0  9994
2 39433  97.6773  55.4250 0032849  98.6261 261.8681 14.91178023167037
EAGLE 2                 
1 39436U 13066W   16352.20136371  .00009073  00000-0  32206-3 0  9996
2 39436  97.7105  84.5583 0012845  10.9106 349.2410 15.29216257168778
CUBEBUG-2 (LO-74)       
1 39440U 13066AA  16353.16506374  .00000193  00000-0  34178-4 0  9996
2 39440  97.6512  28.5854 0081357 166.5531 193.7874 14.75327974165409
FUNCUBE-1 (AO-73)       
1 39444U 13066AE  16353.11434033  .00000263  00000-0  39200-4 0  9996
2 39444  97.6572  38.7193 0059718 138.5240 222.0528 14.81284604164804
UWE-3                   
1 39446U 13066AG  16353.10709578  .00000186  00000-0  31675-4 0  9998
2 39446  97.6525  32.9766 0071323 154.0931 206.3879 14.77857578164458
SPROUT                  
1 39770U 14029E   16353.17081411  .00000713  00000-0  88021-4 0  9997
2 39770  97.8734  88.4127 0008941 193.2274 166.8716 14.85873706139250
UNISAT-6                
1 40012U 14033C   16352.22036505  .00000168  00000-0  32806-4 0  9996
2 40012  97.8794 235.3956 0059134 157.8467 202.5317 14.73501447134145
DUCHIFAT-1              
1 40021U 14033M   16353.21962405  .00000456  00000-0  53658-4 0  9991
2 40021  97.9245 260.0755 0013119 171.2991 188.8456 14.89646147135655
FUNCUBE-3 (EO-79)       
1 40025U 14033R   16352.22019102  .00000287  00000-0  37091-4 0  9997
2 40025  97.9202 257.5278 0012507 177.0114 183.1180 14.88248934135422
CHUBUSAT-1              
1 40300U 14070C   16353.18962823  .00000760  00000-0  37834-4 0  9998
2 40300  97.4133  70.3034 0021695  19.4334  38.8709 15.20430824117283
NUDT-PHONESAT           
1 40900U 15049B   16353.19473626  .00001734  00000-0  87666-4 0  9990
2 40900  97.4442 357.8337 0015691 132.3045 227.9521 15.18288117 68982
ZDPS 2A                 
1 40901U 15049C   16353.11262386  .00000759  00000-0  43238-4 0  9995
2 40901  97.4585 357.5693 0015415 130.1570 230.1016 15.15701197 68908
ZDPS 2B                 
1 40902U 15049D   16353.22188262  .00000753  00000-0  43043-4 0  9996
2 40902  97.4447 356.7133 0015527 127.2831 232.9820 15.15562691 68915
XW-2A                   
1 40903U 15049E   16353.14667154  .00001564  00000-0  44364-4 0  9998
2 40903  97.4358  10.5933 0014914 189.8307 170.2648 15.37729363 69818
KAITUO 1A               
1 40904U 15049F   16353.18342417  .00000305  00000-0  20226-4 0  9998
2 40904  97.4530 356.5409 0016254 125.2590 288.6178 15.14203252 68853
2015-049G               
1 40905U 15049G   16353.16877750  .00000406  00000-0  26393-4 0  9993
2 40905  97.4490 355.6153 0017571 116.7355 243.5677 15.13273598 68822
XW-2C                   
1 40906U 15049H   16353.18834853  .00000584  00000-0  35514-4 0  9995
2 40906  97.4539 356.3702 0016877 118.4746 241.8188 15.14134330 68847
XW-2D                   
1 40907U 15049J   16353.18246499  .00000630  00000-0  37981-4 0  9991
2 40907  97.4507 356.1773 0016271 116.7858 243.5040 15.14210893 68847
LILACSAT 2              
1 40908U 15049K   16353.20715237  .00000305  00000-0  20955-4 0  9998
2 40908  97.4606 356.1270 0017781 113.9506 246.3590 15.12873746 68803
XW-2F                   
1 40910U 15049M   16353.21258261  .00000950  00000-0  54363-4 0  9997
2 40910  97.4532 356.6201 0017200 115.3431 244.9585 15.14905934 68740
XW-2B                   
1 40911U 15049N   16353.18463177  .00000618  00000-0  37372-4 0  9990
2 40911  97.4552 356.4592 0016166 117.0168 243.2716 15.14182627 68744
KAITUO 1B               
1 40912U 15049P   16352.98825329  .00001047  00000-0  56297-4 0  9993
2 40912  97.4520 357.7137 0017600 110.1559 333.7394 15.16935204 68815
TIANWANG 1C (TW-1C)     
1 40926U 15051B   16353.11428443  .00004540  00000-0  11624-3 0  9998
2 40926  97.2532  28.2132 0014344 103.0967 321.6498 15.39805822 69035
TIANWANG 1B (TW-1B)     
1 40927U 15051C   16353.18462111  .00004453  00000-0  11797-3 0  9990
2 40927  97.2624  28.3699 0014185 111.0149 289.8105 15.38770242 69015
TIANWANG 1A (TW-1A)     
1 40928U 15051D   16353.15603344  .00001755  00000-0  54358-4 0  9996
2 40928  97.2490  26.1763 0014614 123.5710 274.3206 15.34690958 68925
LAPAN-A2 (IO-86)        
1 40931U 15052B   16353.14995315  .00000673  00000-0  47765-5 0  9995
2 40931   6.0007  83.5180 0012975 204.1011 155.8518 14.76502591 66162
LQSAT                   
1 40958U 15057A   16353.20924398  .00000079  00000-0  20173-4 0  9992
2 40958  97.9980  65.5180 0017023 287.6051  72.3308 14.72931509 64472
CHUBUSAT-2              
1 41338U 16012B   16352.84533196  .00000299  00000-0  16878-4 0  9999
2 41338  31.0019 183.8490 0013809  64.2052 296.0006 14.99597302 45740
CHUBUSAT-3              
1 41339U 16012C   16352.93212804  .00000363  00000-0  21821-4 0  9996
2 41339  31.0118 182.7418 0013669  70.8864 289.3247 14.99814161 45733
OUFTI-1                 
1 41458U 16025C   16353.19465630  .00000802  00000-0  47823-4 0  9996
2 41458  98.2077  14.3274 0178142 173.2557 187.1121 15.02302399 35396
E-ST@R-II               
1 41459U 16025D   16353.13400908  .00001158  00000-0  66814-4 0  9994
2 41459  98.2087  14.2392 0179006 173.7913 186.5593 15.02212970 35389
AAUSAT 4                
1 41460U 16025E   16353.15233146  .00002778  00000-0  15541-3 0  9995
2 41460  98.1951  14.2943 0171270 173.1837 187.1779 15.02764059 35390
NUSAT 1 (LO-87)         
1 41557U 16033B   16353.17848862  .00001150  00000-0  47097-4 0  9995
2 41557  97.4815  66.6311 0013071 287.6499  72.3311 15.26311072 30815
BEESAT-4                
1 41619U 16040W   16353.13125969  .00001217  00000-0  59727-4 0  9991
2 41619  97.4883  51.1072 0011852 348.0108  12.0843 15.19968129 15082
PRATHAM                 
1 41783U 16059A   16353.16896815  .00000056  00000-0  20478-4 0  9992
2 41783  98.1938  50.1702 0033794  10.4777 349.7121 14.62789673 12137
ALSAT 1N                
1 41789U 16059G   16353.16971757  .00000145  00000-0  36933-4 0  9999
2 41789  98.1938  50.3272 0028566   6.2734 353.8829 14.63989308 12122
#endif 
