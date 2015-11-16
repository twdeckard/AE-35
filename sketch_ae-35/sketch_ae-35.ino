// note requires a fork of AFMotor library
// to remap pins to slot 4 of DFRobot mega
// multi expansion shield
//
#include <AFMotor.h>
#include <Time.h>
#include <AccelStepper.h>
#include <LiquidCrystal.h>
#include <Plan13.h>


/***********************************************************************************************************
 *
 * AE-35 Antenna Controller
 *
 * Satellite Tracking System
 *
 * Arduino Mega 2560 R3
 * SainSmart L293D Motor Drive
 * SainSmart LCD 1602 + Keypad
 *
 * Compute the location of one of a selection of orbiting satellites from an internal database
 * and steer an altitude/azimuth gimbal to maintain a track while object is above the horizon.
 *
 * On initialization the user inputs the GMT time and date, chooses from a list of orbiting objects 
 * and steers the antenna gimbal to north/horizon using manual inputs. 
 * 
 * Copyright 2015 Todd Deckard, All rights reserved. 
 *
 ********************************************************************************************************/

#define NEVER                                     0
#define ALWAYS                                    1

#define INIT        0
#define CNFG        (INIT+1)
#define GPSL        (INIT+2)
#define DRIV        (INIT+3)
#define TRAC        (INIT+4)
#define HOME        (INIT+5)
#define STOP        (INIT+6)

unsigned short int           _state;
unsigned short int           _state_transition_flag;

time_t                       _time;
unsigned long int            _ms;
unsigned long int            _lastms;
unsigned long int            _fastTimer;
unsigned long int            _mediumTimer;
unsigned long int            _slowTimer;

double                      _currentAzimuth;
double                      _currentElevation;

long int                    _currentAzSteps;
long int                    _currentElSteps;

double                      _targetAzimuth;
double                      _targetElevation;

double                      _commandAzimuth;
double                      _commandElevation;

long int                    _commandAzSteps;
long int                    _commandElSteps;

double                      _latitude;
double                      _longitude;

String                      _target;

#define  NUMBEROFSTATES     7

String                      _stateLabels[] = { "INIT", "CNFG", "GPSL", "DRIV", "TRAC", "HOME", "STOP"};
unsigned short int          _lcd_button_now = 0;
unsigned short int          _lcd_button_prev = 0;
boolean                     _flagHomePositionSet = 0;

unsigned short int          _lcd_button_debounce = 0;
unsigned short int          _adc_debug_temporary = 0;
unsigned short int          _gps_debug_temporary = 0;
unsigned short int          _flagTimeSetManually = 0;
unsigned short int          _flagDateSetManually = 0;
unsigned short int          _flagSatelliteSet = 0;

#define DEBUGP13 1
Plan13              p13;
// temporarily hard code in TLEs
// http://www.amsat.org/amsat/ftp/keps/current/nasabare.txt
//
String _keps[] =
{
    "2014-12-28",
    "HUBBLE",
    "1 20580U 90037B   15308.45560487  .00002158  00000-0  12608-3 0  9996",
    "2 20580  28.4708 308.8893 0002648 304.5112 184.0287 15.07840717201004",
    "ISS",
    "1 25544U 98067A   15319.57746612  .00014090  00000-0  21211-3 0  9991",
    "2 25544  51.6461  48.5120 0006315 139.5641 326.7652 15.55086969971595",
    "MOON2015_11",
    "1 01511U 00000    15298.25194076  .00000000  00000-0  10000-3 0 00004",
    "2 01511 018.2897 359.7740 0563000 005.5133 355.1249  0.03660099000003",
    "AO-07",
    "1 07530U 74089B   15308.92431491 -.00000023  00000-0  13127-3 0  9996",
    "2 07530 101.5414 280.4840 0012327 030.1971 351.6256 12.53617504874668",
    "FO-29",
    "1 24278U 96046B   15309.47255762  .00000031  00000-0  65076-4 0  9996",
    "2 24278  98.5618 259.2809 0351321  64.6751  45.2683 13.53059038949132",
    "AO-73",
    "1 39444U 13066AE  15309.15464776  .00001507  00000-0  19724-3 0  9997",
    "2 39444  97.7143   6.6449 0060249  37.1727 323.3634 14.80585767104273",
    "XW-2A",
    "1 40903U 15049E   15309.85269854  .00007556  00000-0  23280-3 0  9991",
    "2 40903  97.4536 317.5796 0015707 213.3133 268.3547 15.33793567  7131",
    "XW-2C",
    "1 40906U 15049H   15309.86646862  .00003108  00000-0  18212-3 0  9991",
    "2 40906  97.4557 316.9462 0017867  91.3943 353.0725 15.12395146  7084",
    "LILACSAT-2",
    "1 40908U 15049K   15309.88333928  .00002284  00000-0  13702-3 0  9994",
    "2 40908  97.4627 316.9629 0018537  85.4385 359.3742 15.11801067  7089",
    "XW-2E",
    "1 40909U 15049L   15309.85726856  .00004706  00000-0  27109-3 0  9995",
    "2 40909  97.4561 316.9585 0017320  94.4786 349.3912 15.12760900  7081",
    "AO-27",
    "1 22825U 93061C   15307.93286803  .00000103  00000-0  57299-4 0  9993",
    "2 22825 098.7267 258.1938 0007315 259.0593 100.9766 14.29914069152636",
    "NO-44",
    "1 26931U 01043C   15309.04910312  .00000083  00000-0  63618-4 0  9996",
    "2 26931  67.0504 173.7260 0006523 277.1982  82.8377 14.30356000735948",
    "SO-50",
    "1 27607U 02058C   15308.94235219  .00000645  00000-0  11160-3 0  9996",
    "2 27607 064.5533 128.9524 0082601 261.4106 097.7631 14.74903884691915",
    "AO-51",
    "1 28375U 04025K   15307.88059361  .00000181  00000-0  66721-4 0  9998",
    "2 28375 098.3053 244.3918 0081442 267.7720 091.4142 14.41714835596555",
    "XW-2D",
    "1 40907U 15049J   15309.80212380  .00003171  00000-0  18597-3 0  9996",
    "2 40907  97.4527 316.8600 0017349  90.2694 355.8372 15.12348934  7071",
    "XW-2F",
    "1 40910U 15049M   15309.87165313  .00004847  00000-0  28247-3 0  9994",
    "2 40910  97.4564 316.9397 0018646  88.8776 356.6142 15.12306885  6976",
    "TEST",
    "1 25544U 98067A   08264.51782528 -.00002182  00000-0 -11606-4 0  2927",
    "2 25544  51.6416 247.4627 0006703 130.5360 325.0288 15.72125391563537",
    "END",
};

#define AZIMUTH_MOTOR_STEPS_PER_REV         200
#define ELEVATION_MOTOR_STEPS_PER_REV       200
#define AZIMUTH_GIMBAL_STEPS_PER_DEGREE     (1180.0 / 90.0)
#define ELEVATION_GIMBAL_STEPS_PER_DEGREE   (1760.0/ 45.0 )

void readbtns();

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

#define LCD_BUTTON_NONE                           2
#define LCD_BUTTON_RIGHT                          4
#define LCD_BUTTON_LEFT                           8
#define LCD_BUTTON_DOWN                           16
#define LCD_BUTTON_UP                             32
#define LCD_BUTTON_SELECT                         64
#define GPS_LOCK                                  128
#define HOME_POSITION_SET                         256

#ifdef DEBUGP13

void debugP13keps()
{

    Serial.print("p13.LA = ");
    Serial.println(p13.LA,8);
    Serial.print("p13.LO = ");
    Serial.println(p13.LO,8);
    Serial.println("");
    Serial.print("p13.DN = ");
    Serial.println(p13.DN,8);
    Serial.print("p13.TN = ");
    Serial.println(p13.TN);
    Serial.println("");

    Serial.print("p13.YE = ");
    Serial.println(p13.YE,8);
    Serial.print("p13.TE = ");
    Serial.println(p13.TE,8);
    Serial.print("p13.IN = ");
    Serial.println(p13.IN,8);
    Serial.print("p13.RA = ");
    Serial.println(p13.RA,8);
    Serial.print("p13.EC = ");
    Serial.println(p13.EC,8);
    Serial.print("p13.WP = ");
    Serial.println(p13.WP,8);
    Serial.print("p13.MA = ");
    Serial.println(p13.MA,8);
    Serial.print("p13.MM = ");
    Serial.println(p13.MM,8);
    Serial.print("p13.M2 = ");
    Serial.println(p13.M2,8);
    Serial.print("p13.RV = ");
    Serial.println(p13.RV,8);
    Serial.print("p13.ALON=");
    Serial.println(p13.ALON,8);
    Serial.println("");

    Serial.print("p13.DE = ");
    Serial.println(p13.DE,8);
    Serial.print("p13.TE = ");
    Serial.println(p13.TE,8);
    Serial.println("");
}

#endif

#ifdef DEBUGP13
void debugp13update()
{
    p13.printdata();
    Serial.println("");
}
#endif

void readbtns()
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

    _lcd_button_debounce = _lcd_button_now; // no debouncing
    /*
    _state_transition_flag &= !(LCD_BUTTON_NONE | LCD_BUTTON_RIGHT | LCD_BUTTON_LEFT | LCD_BUTTON_UP | LCD_BUTTON_DOWN |
                                LCD_BUTTON_SELECT);  // one button command at a time
    */
    _state_transition_flag |= _lcd_button_debounce;

} ;

void pollGPS()                            // GPS shield is not integrated
{

#ifdef FUNCTIONTRACETOSERIAL
    Serial.println(__func__);
#endif

    time_t t = now();
    _gps_debug_temporary += 1;
    if (_gps_debug_temporary > 50)
    {
        _latitude  =  44.884860;          // grid square EN34FV 
        _longitude = -93.551492;
        p13.setLocation(_longitude,_latitude,0);
        // p13.setTime(year(t),month(t),day(t),hour(t),minute(t),second(t));
        _state_transition_flag |= GPS_LOCK;
        lcd.clear();
    }
}


void buttonCommand()
{

#ifdef FUNCTIONTRACETOSERIAL
    Serial.println(__func__);
#endif

    if ((_lcd_button_debounce == LCD_BUTTON_NONE) || (_lcd_button_debounce == 0)) return;

    if (_flagHomePositionSet)
    {
        // user commands azimuth and angle in degrees

        switch(_lcd_button_debounce)
        {
        case LCD_BUTTON_UP:
            _commandElevation+=1.0;
            _commandElevation = min(_commandElevation,80.0);
            break;
        case LCD_BUTTON_DOWN:
            _commandElevation-=1.0;
            _commandElevation = max(_commandElevation,0.0);
            break;
        case LCD_BUTTON_RIGHT:
            _commandAzimuth+=1.0;
            _commandAzimuth  =  min(_commandAzimuth,350.0);
            break;
        case LCD_BUTTON_LEFT:
            _commandAzimuth-=1.0;
            _commandAzimuth   = max(_commandAzimuth,0.0);
            break;
        };

        _commandElSteps = _commandElevation / ELEVATION_GIMBAL_STEPS_PER_DEGREE;
        _commandAzSteps = _commandAzimuth / AZIMUTH_GIMBAL_STEPS_PER_DEGREE; 

    }
    else
    {

        switch(_lcd_button_debounce)
        {
        case LCD_BUTTON_UP:
            _commandElSteps+=10;
            break;
        case LCD_BUTTON_DOWN:
            _commandElSteps-=10;
            break;
        case LCD_BUTTON_LEFT:
            _commandAzSteps-=10;
            break;
        case LCD_BUTTON_RIGHT:
            _commandAzSteps+=10;
            break;
        };

        _commandElevation = _commandAzimuth = 0;

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

void updateLCD()
{

#ifdef FUNCTIONTRACETOSERIAL
    Serial.println(__func__);
#endif

    time_t t = now();
    char       buffer[256];
    lcd.setCursor(0,0);

    lcd.print(_stateLabels[_state]);
    lcd.print("    ");
    sprintf(buffer,"%02d:%02d:%02d",hour(t),minute(t),second(t));
    lcd.print(buffer);

    lcd.setCursor(0,1);
    lcd.print(_target);

    if (!_flagHomePositionSet)
    {
        sprintf(buffer," A%05dE%05d",(int)_commandAzSteps,(int)_commandElSteps);
        lcd.print(buffer);
    }
    else
    {
        lcd.setCursor(4,1);
        lcd.print("A     E     ");
        lcd.setCursor(5,1);
        lcd.print(_targetAzimuth,1);
        lcd.setCursor(11,1);
        lcd.print(_targetElevation,1);
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

void motorCommand()
{
#ifdef FUNCTIONTRACETOSERIAL
    Serial.println(__func__);
#endif

    azimuthMotor.moveTo(_commandAzSteps);       // Note: original executive was throttled to a maximum of 50 steps/second
    azimuthMotor.setMaxSpeed(50);               // 50 steps/sec / (1180.0 / 90.0) = ~4deg/sec
    azimuthMotor.setAcceleration(10.0);         // 
    elevationMotor.moveTo(_commandElSteps);
    elevationMotor.setMaxSpeed(50);             // 50 steps/second / (1760.0 / 45.0) = ~1.3deg/sec
    elevationMotor.setAcceleration(10.0);       //
}

void moveMotors()
{
#ifdef FUNCTIONTRACETOSERIAL
    Serial.println(__func__);
#endif

// executive runs every 20ms, 500 times a second maximum
//
    azimuthMotor.run();
    elevationMotor.run();
}

void updateSat()
{
#ifdef FUNCTIONTRACETOSERIAL
    Serial.println(__func__);
#endif

    time_t t = now();
    p13.setTime(year(t),month(t),day(t),hour(t),minute(t),second(t));
    p13.satvec();
    p13.rangevec();

#ifdef DEBUGP13
    debugp13update();
#endif

}

void trackCommand()
{
#ifdef FUNCTIONTRACETOSERIAL
    Serial.println(__func__);
#endif
    _targetElevation = p13.EL;
    _commandElevation = min(max(_targetElevation,5.0),80.0);

    _commandAzimuth = _targetAzimuth = min(p13.AZ,360.0);

    if (_targetAzimuth > 180.0) _commandAzimuth = - (360.0 - _commandAzimuth);

    // note elevation steps/degree presumes SINGLE step mode

    _commandElSteps = round(_commandElevation * ELEVATION_GIMBAL_STEPS_PER_DEGREE);
    _commandAzSteps = round(_commandAzimuth * AZIMUTH_GIMBAL_STEPS_PER_DEGREE);
}

void printMotorPos()
{
#ifdef DEBUGMOTOR
#ifdef FUNCTIONTRACETOSERIAL
    Serial.println(__func__);
#endif
#endif
    _currentAzSteps = azimuthMotor.currentPosition();
    _currentElSteps = elevationMotor.currentPosition();

// Keep the L293D from frying while we debug
    if (azimuthMotor.distanceToGo() == 0) motor2.release();
    if (elevationMotor.distanceToGo() == 0) motor1.release();
}
void displaySel() { } ;
void menuSel()  { };

/********************************************************************************************************
 *
 * Define Cyclic Executive State Machine
 *
 *******************************************************************************************************/



#define UPDNRTLF    (LCD_BUTTON_UP | LCD_BUTTON_DOWN | LCD_BUTTON_LEFT | LCD_BUTTON_RIGHT)
#define SEL         (LCD_BUTTON_SELECT)
#define SEL         (LCD_BUTTON_SELECT)
#define HOM         (HOME_POSITION_SET)

unsigned short int transitions[NUMBEROFSTATES][NUMBEROFSTATES] =
{
    /*                   INIT        CNFG         GPSL       DRIV        TRAC        HOM        STOP  */
    /* INIT   */        NEVER,      ALWAYS,       NEVER,     NEVER,     NEVER,     NEVER,      NEVER,
    /* CNFG   */        NEVER,      NEVER,        ALWAYS,    NEVER,     NEVER,     NEVER,      NEVER,
    /* GPSL   */        NEVER,      NEVER,        NEVER,    GPS_LOCK,   NEVER,     NEVER,      NEVER,
    /* DRIV   */        NEVER,      NEVER,        NEVER,     NEVER,     NEVER,     SEL,       NEVER,
    /* TRAC   */        NEVER,      NEVER,        NEVER,    UPDNRTLF,   NEVER,     NEVER,      NEVER,
    /* HOME   */        NEVER,      NEVER,        NEVER,     NEVER,     ALWAYS,    NEVER,      NEVER,
    /* STOP   */        NEVER,      NEVER,        NEVER,     NEVER,     NEVER,     NEVER,      NEVER
};

/*                            INIT         CNFG        GPSL        DRIV        TRAC        HOME         STOP  */
void (*slow[5][NUMBEROFSTATES])()
= {  noop,       displaySel, updateLCD,   updateLCD,   updateLCD,   updateLCD,   noop,
     noop,        noop,       pollGPS,  printMotorPos, updateSat,     noop,      noop,
     noop,        noop,        noop,        noop,    trackCommand,    noop,      noop,
     noop,        noop,        noop,        noop,    printMotorPos,   noop,      noop,
     noop,        noop,        noop,        noop,        noop,        noop,      noop
  };

/*                            INIT         CNFG        GPSL        DRIV         TRAC         HOME        STOP  */
void (*medium[5][NUMBEROFSTATES])()
= {  noop,      menuSel,       noop,   buttonCommand, motorCommand,        noop,      noop,
     noop,        noop,        noop,    motorCommand,    noop,        noop,      noop,
     noop,        noop,        noop,        noop,        noop,        noop,      noop,
     noop,        noop,        noop,        noop,        noop,        noop,      noop,
     noop,        noop,        noop,        noop,        noop,        noop,      noop
  };

/*                            INIT         CNFG        GPSL        DRIV         TRAC,        HOME        STOP  */
void (*fast[5][NUMBEROFSTATES])()
= { readbtns,  readbtns,    readbtns,    readbtns,     readbtns,    readbtns,    noop,
    noop,        noop,        noop,     moveMotors,  moveMotors,     noop,      noop,
    noop,        noop,        noop,        noop,        noop,        noop,      noop,
    noop,        noop,        noop,        noop,        noop,        noop,      noop,
    noop,        noop,        noop,        noop,        noop,        noop,      noop
  };

void manualSatelliteInput_blocking()
{
    int index = 1;
    unsigned short int initial_button_state;
    int debounce_counter = 4;
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Set Satellite");
    lcd.blink();

    while (!_flagSatelliteSet)
    {

        initial_button_state = _lcd_button_debounce;
        readbtns();
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
        }

        if (_keps[index].equals("END")) index=1;

        lcd.setCursor(0,1);
        lcd.print(_keps[index]);
        lcd.print("   ");
        delay(50);
    }
    lcd.noBlink();
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
    lcd.print("Set Date  (GMT)");
    lcd.blink();

    while (!_flagDateSetManually)
    {

        initial_button_state = _lcd_button_debounce;
        readbtns();
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
                setTime(hour(t),minute(t),second(t),Month,Day,Year);
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
    lcd.print("Set Time (GMT)");
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

        readbtns();

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
}

void initAE35()
{
#ifdef FUNCTIONTRACETOSERIAL
    Serial.println(__func__);
#endif
    String temp; 
    _lastms = _fastTimer = _mediumTimer = _slowTimer = 0;
    _commandAzimuth = _commandElevation = _targetAzimuth = _targetElevation = 0;
    _commandAzSteps = _commandElSteps = 0;
    _state_transition_flag = 0;
    _flagHomePositionSet = 0;

    setTime(23,30,00,11,10,15);
    Serial.begin(9600);
    Serial.print("Hello World!");
    lcd.begin(16, 2);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("AE-35 Antenna");
    lcd.setCursor(0,1);
    lcd.print(__FILE__);

    azimuthMotor.setCurrentPosition(0);
    elevationMotor.setCurrentPosition(0);
    delay(3000);

#ifdef DEBUGMOTOR
    Serial.println("forked AF_Motor library");
    Serial.print("MOTORLATCH=");
    Serial.println(MOTORLATCH);
    Serial.print("MOTORENABLE=");
    Serial.println(MOTORENABLE);
    Serial.print("MOTORCLK=");
    Serial.println(MOTORCLK);
    Serial.print("MOTORDATA=");
    Serial.println(MOTORDATA);
#endif

// temporarily allow user to input an approximate time

    manualTimeInput_blocking();
    manualDateInput_blocking();
    manualSatelliteInput_blocking();

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

    /*********************
       epochYear = 2015.0;
       epochTime = 262.54809569;
       inclination = 51.6451;
       rightAscension = 332.9346;
       eccentricity = 0.0005645;
       argOfPerigee = 294.622;
       meanAnomoly = 141.0489;
       meanMotion = 15.54107337;
       revolutionNumber = 96272;
    ***********************/

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("epochYear");
    Serial.println("epochYear");
    lcd.setCursor(0,1);
    lcd.print(epochYear);
    Serial.println(epochYear); 
    delay(500);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("epochTime");
    Serial.println("epochTime");
    lcd.setCursor(0,1);
    lcd.print(epochTime);
    Serial.println(epochTime);
    delay(500);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("inclination");
    Serial.println("inclination");
    lcd.setCursor(0,1);
    lcd.print(inclination);
    Serial.println(inclination);
    delay(500);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("rightAscension");
    Serial.println("rightAscension");
    lcd.setCursor(0,1);
    lcd.print(rightAscension); 
    Serial.println(rightAscension);  
    delay(500); 
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("eccentr. * 10^6");
    Serial.println("eccentr. * 10^6");
    lcd.setCursor(0,1);
    lcd.print((eccentricity * 1000000.0)); 
    Serial.println((eccentricity * 1000000.0));
    delay(1000);
    
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("argOfPerigee");
    Serial.print("argOfPerigee");
    lcd.setCursor(0,1);
    lcd.print(argOfPerigee);
    Serial.print(argOfPerigee);
    delay(500);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("meanAnomoly");
    Serial.print("meanAnomoly");
    lcd.setCursor(0,1);
    lcd.print(meanAnomoly);
    Serial.print(meanAnomoly);
    delay(500);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("decayRate * 10^6");
    Serial.print("decayRate * 10^6");
    lcd.setCursor(0,1);
    lcd.print((decayRate * 1000000.0));
    Serial.print((decayRate * 1000000.0));
    delay(1000);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("revolutionNumber");
    lcd.setCursor(0,1);
    lcd.print(revolutionNumber);
    delay(500);

    p13.setTime(2015, 9, 20, 12, 0, 0);   // just set a sensible time
    _latitude  =  44.884860;
    _longitude = -93.551492;
    p13.setLocation(_longitude,_latitude,0);  // just set a sensible lat/lon

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
    p13.initSat();

#ifdef DEBUGP13
    debugP13keps();
#endif
};

void gpsLockAE35()
{
#ifdef FUNCTIONTRACETOSERIAL
    Serial.println(__func__);
#endif

    // setTime(12,00,00,9,20,2015);

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("AE-35 Antenna");
    lcd.setCursor(0,1);
    lcd.print("configure GPS");
};

void driveAE35()
{
#ifdef FUNCTIONTRACETOSERIAL
    Serial.println(__func__);
#endif
};

void trackAE35()
{
#ifdef FUNCTIONTRACETOSERIAL
    Serial.println(__func__);
#endif

    time_t t = now();
    p13.setTime(year(t),month(t),day(t),hour(t),minute(t),second(t));
    p13.satvec();
    p13.rangevec();

#ifdef DEBUGP13
    debugP13keps();
#endif

    _commandAzimuth = _targetAzimuth = p13.AZ;
    _commandElevation = _targetElevation = p13.EL;
};

void homeAE35()
{
#ifdef FUNCTIONTRACETOSERIAL
    Serial.println(__func__);
#endif
    azimuthMotor.setCurrentPosition(0);
    elevationMotor.setCurrentPosition(0);

    if (!_flagHomePositionSet) { };
    _flagHomePositionSet = 1;
};

void stopAE35()
{
#ifdef FUNCTIONTRACETOSERIAL
    Serial.println(__func__);
#endif
};


void (*states[NUMBEROFSTATES])() = { initAE35, configAE35, gpsLockAE35, driveAE35, trackAE35, homeAE35, stopAE35 } ;

void updateState()
{

#ifdef FUNCTIONTRACETOSERIAL
    Serial.println(__func__);
#endif
    int i,j;
    unsigned short int flag;

    _state_transition_flag |= _flagHomePositionSet;
    _state_transition_flag |= ALWAYS;

    for (i=0; i<NUMBEROFSTATES; i++)
    {
        if (transitions[_state][i] & _state_transition_flag)
        {
            _state_transition_flag &= !(transitions[_state][i] & _state_transition_flag);
            _state = i;
            // lcd.setCursor(0,1); lcd.print(i);
            (*states[_state])();
            break;
        }
    }
}




void noop() { };
void readSerial() { };
void processSerial() { };
void readAbsolutePosition() {};
void updateTime() {};



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

    updateState();

    _ms = millis();
    elapsed = _ms - _lastms;
    _fastTimer += elapsed;
    _mediumTimer += elapsed;
    _slowTimer += elapsed;
    _lastms = _ms;

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
    azimuthMotor.run();                               // now always try to advance the stepper motors if the executive is spinning 
    elevationMotor.run();                             
}



