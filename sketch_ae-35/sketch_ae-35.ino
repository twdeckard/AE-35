
// requires fork of AFMotor to remap shift reg pins to slot 4 for DFRobot mega multi expansion shield
#include <AFMotor.h>
// requires fork of Time for sub second resolution
#include <Time.h>
#include <AccelStepper.h>
#include <LiquidCrystal.h>
#include <Wire.h>
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
"2015-11-22",
"HUBBLE",
"1 20580U 90037B   15308.45560487  .00002158  00000-0  12608-3 0  9996",
"2 20580  28.4708 308.8893 0002648 304.5112 184.0287 15.07840717201004",
"ISS",          
"1 25544U 98067A   15355.13133534  .00018357  00000-0  27491-3 0  9992",
"2 25544  51.6438 231.1133 0008100 305.0563 214.8814 15.54985591977120",
"MOON2015_11",
"1 01511U 00000    15298.25194076  .00000000  00000-0  10000-3 0 00004",
"2 01511 018.2897 359.7740 0563000 005.5133 355.1249  0.03660099000003",
"AO-73",
"1 39444U 13066AE  15322.87353904  .00001603  00000-0  20942-3 0  9992",
"2 39444 097.7123 019.8359 0059120 352.6005 007.4329 14.80630332105717",
"EO-79",
"1 40025U 14033R   15323.68957936  .00002024  00000-0  22884-3 0  9994",
"2 40025  97.9393 224.0784 0013846  39.8494 320.3736 14.87549197 76900",
"XW-2A",
"1 40903U 15049E   15323.81209640  .00008866  00000-0  27050-3 0  9994",
"2 40903  97.4535 331.6657 0016993 161.4437 315.3446 15.34028666  9271",
"XW-2C",
"1 40906U 15049H   15323.76045139  .00004684  00000-0  27165-3 0  9993",
"2 40906  97.4554 330.5189 0017561  48.0555  38.9239 15.12514737  9189",
"NO-44",
"1 26931U 01043C   15323.45331922  .00000099  00000-0  69935-4 0  9996",
"2 26931  67.0493 136.5664 0006697 276.9610  83.0729 14.30361042738003",
"SO-50",
"1 27607U 02058C   15322.84217690  .00001068  00000-0  17106-3 0  9990",
"2 27607  64.5550  86.4971 0082302 258.7912 100.3931 14.74931981694095",
"AO-51",
"1 28375U 04025K   15323.38078738  .00000170  00000-0  63277-4 0  9991",
"2 28375  98.3112 259.4755 0081976 220.4782 260.7991 14.41721314598851",
"NO-84",
"1 40654U 15025X   15336.15621742  .00014546  00000-0  35274-3 0 02007",
"2 40654 054.9965 209.1907 0224949 302.4861 055.4602 15.19293577029654",
"AO-85",
"1 40967U 15058D   15323.50196694  .00001732  00000-0  19539-3 0 00443",
"2 40967 064.7773 164.5226 0217735 269.2432 088.3721 14.74416702006068",
"END",
};

#define AZIMUTH_MOTOR_STEPS_PER_REV         200
#define ELEVATION_MOTOR_STEPS_PER_REV       200
#define AZIMUTH_GIMBAL_STEPS_PER_DEGREE     (1180.0 / 90.0)
#define ELEVATION_GIMBAL_STEPS_PER_DEGREE   ((2000.0) / 50.0)

#define AZIMUTH_MOTOR_MAX_STEPS             (3000)
#define AZIMUTH_MOTOR_MIN_STEPS             (-3000)
#define ELEVATION_MOTOR_MAX_STEPS           (2100)
#define ELEVATION_MOTOR_MIN_STEPS           (0)
#define ELEVATION_GIMBAL_MAX_ELEVATION      (50)

#define MAX_EL_SPEED_MANUAL                 (120)
#define MAX_EL_SPEED_TRACK                  (60)
#define MAX_EL_SPEED_HOME                   (30)
#define MAX_AZ_SPEED_MANUAL                 (120)
#define MAX_AZ_SPEED_TRACK                  (60)
#define MAX_AZ_SPEED_HOME                   (30)
   
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
    if (_gps_debug_temporary > 50)       // but we'll use the repetition to average the IMU readings
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
    motor2.onestep(FORWARD, INTERLEAVE);
}
void backwardstep_motor2()
{
    motor2.onestep(BACKWARD, INTERLEAVE);
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
    azimuthMotor.setAcceleration(200.0);         // "but they're so small they're evading our turbolasers."
    elevationMotor.moveTo(AE35.get_ElTargetMotorSteps());
    elevationMotor.setMaxSpeed(_maxElSpeed);             // 50 steps/second / (1760.0 / 45.0) = ~1.3deg/sec
    elevationMotor.setAcceleration(200.0);       //
}

void currentMotorPos()
{
#ifdef FUNCTIONTRACETOSERIAL
    Serial.println(__func__);
#endif

// Keep the L293D from frying while we debug, antenna position may drift
    if (azimuthMotor.distanceToGo() == 0) motor2.release();
    if (elevationMotor.distanceToGo() == 0) motor1.release();
}

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
          noop,          noop,          pollGPS,   currentMotorPos, updateSat,       noop,        noop,       noop,
          noop,          noop,           noop,        noop,       currentMotorPos,   noop,        noop,       noop,
          noop,          noop,           noop,        noop,            noop,         noop,        noop,       noop,
          noop,          noop,           noop,        noop,            noop,         noop,        noop,       noop
      };
      
void (*medium[5][NUMBEROFSTATES])()
    /*  INIT           CNFG            GPSL          DRIV           TRAC           HOM          SERL        STOP  */
    = {   noop,          noop,       noop,  satTrackButtons,   satTrackButtons, stepsAzElToMotors, processSerial1, noop,
          noop,          noop,       noop,  stepsAzElToMotors, stepsAzElToMotors,  noop,    stepsAzElToMotors,     noop,
          noop,          noop,       noop,         noop,        checkSerial1,     noop,        noop,               noop,
          noop,          noop,       noop,         noop,          noop,           noop,        noop,               noop,
          noop,          noop,       noop,         noop,          noop,           noop,        noop,               noop
      };

void (*fast[5][NUMBEROFSTATES])()
    = {  readBtn,       readBtn,        readBtn,     readBtn,       readBtn,      readBtn,       readBtn,    noop,
         noop,           noop,           noop,         noop,          noop,         noop,         noop,      noop,
         noop,           noop,           noop,         noop,          noop,         noop,         noop,      noop,
         noop,           noop,           noop,         noop,          noop,         noop,         noop,      noop,
         noop,           noop,           noop,         noop,          noop,         noop,         noop,      noop,
      };

void (*always[5][NUMBEROFSTATES])()
    = {  noop,           noop,           noop,     movAzElMotors, movAzElMotors,  readSwtch,    movAzElMotors, noop,
         noop,           noop,           noop,         noop,          noop,       switchCmd,      noop,        noop,
         noop,           noop,           noop,         noop,          noop,      movAzElMotors,   noop,        noop,
         noop,           noop,           noop,         noop,          noop,         noop,         noop,        noop,
         noop,           noop,           noop,         noop,          noop,         noop,         noop,        noop
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
    int Day=1;
    int Year = 2015;
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

    hours = hour(t);
    minutes = minute(t);
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

  Wire.begin();
  accelGyroMag.initialize();

  AE35.set_ElAntUpperLimitDeg(50.);                     
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
    time_t              t = now(); 

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

