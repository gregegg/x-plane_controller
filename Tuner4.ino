#include <LiquidCrystal.h>
#include <Bounce2.h>
#include <Encoder.h>
#include <string.h>

char buffer[20];

const long pinRS = 2;
const long pinE = 3;
const long pinD4 = 4;
const long pinD5 = 5;
const long pinD6 = 7;
const long pinD7 = 27;
const long pinLCD = 26;

const long pinA1 = 20;
const long pinA2 = 39;
const long pinA3 = 40;
const long pinA4 = 44;
const long pinA5 = 17;
const long pinA6 = 14;
const long pinB1 = 24;
const long pinB2 = 43;
const long pinB3 = 42;
const long pinB4 = 16;
const long pinB5 = 15;
const long pinB6 = 25;
const long pinEnc1But = 41;
const long pinEnc2But = 45;

const long pinTog1A = 22;
const long pinTog1B = 23;
const long pinTog2A = 8;
const long pinTog2B = 9;
const long pinTog3A = 38;
const long pinTog3B = 21;
const long pinTog4A = 10;
const long pinTog4B = 11;
const long pinTog5A = 13;
const long pinTog6A = 12;

const long pinEnc1A = 18;
const long pinEnc1B = 19;
const long pinEnc2A = 0;
const long pinEnc2B = 1;

// define the screen
LiquidCrystal lcd(pinRS, pinE, pinD4, pinD5, pinD6, pinD7);
elapsedMillis lcd_refresh;
String sL1, sL2, sL3, sL4;

//define debouncers for pushbuttons
Bounce butA1 = Bounce();
Bounce butA2 = Bounce();
Bounce butA3 = Bounce();
Bounce butA4 = Bounce();
Bounce butA5 = Bounce();
Bounce butA6 = Bounce();
Bounce butB1 = Bounce();
Bounce butB2 = Bounce();
Bounce butB3 = Bounce();
Bounce butB4 = Bounce();
Bounce butB5 = Bounce();
Bounce butB6 = Bounce();
Bounce butEnc1But = Bounce();
Bounce butEnc2But = Bounce();

//define encoders

Encoder Enc1(pinEnc1B, pinEnc1A);
Encoder Enc2(pinEnc2B, pinEnc2A);
long oldEnc1;
long oldEnc2;

// Flight Sim Variables
// Toggles

FlightSimInteger HSI_Select; //sim/cockpit2/radios/actuators/HSI_source_select_pilot  //Tog 2 - HSI
FlightSimInteger APMode; //sim/cockpit2/autopilot/flight_director_mode  //Tog 3 - Autopilot
FlightSimInteger XPDRMode; //sim/cockpit2/radios/actuators/transponder_mode //Tog 4 - xpdr
FlightSimInteger GearIsDown; //sim/cockpit2/controls/gear_handle_down  // Tog 5 - Gear
FlightSimInteger DeIce; //sim/cockpit2/ice/ice_all_on //Tog 6 - DeIce



// Mode 1  (NAV)
// Row A  EFIS Mode	EFIS Zoom	Mach	Bank Angle   Land/Taxi	Panel/Instrument

long iLightMode; //1 = Land/Taxi, 2 = panel/instrument

FlightSimInteger  EFISMapMode; //sim/cockpit2/EFIS/map_mode
FlightSimInteger  EFISMapModeHSI;   //sim/cockpit2/EFIS/map_mode_is_HSI
FlightSimInteger  EFISMapRange;   //sim/cockpit2/EFIS/map_range
int iEFISDetail;  // a variable to loop through detail on EFIS map view
FlightSimInteger  EFIS_weather; // = XPlaneRef("sim/cockpit2/EFIS/EFIS_weather_on");
FlightSimInteger  EFIS_tcas; // = XPlaneRef("sim/cockpit2/EFIS/EFIS_tcas_on");
FlightSimInteger  EFIS_airport; // = XPlaneRef("sim/cockpit2/EFIS/EFIS_airport_on");
FlightSimInteger  EFIS_fix; // = XPlaneRef("sim/cockpit2/EFIS/EFIS_fix_on");
FlightSimInteger  EFIS_vor; // = XPlaneRef("sim/cockpit2/EFIS/EFIS_vor_on");
FlightSimInteger  EFIS_ndb; // = XPlaneRef("sim/cockpit2/EFIS/EFIS_ndb_on");
FlightSimInteger  AirspeedMach;  //sim/cockpit2/autopilot/airspeed_is_mach
FlightSimInteger  BankAngle; //sim/cockpit2/autopilot/bank_angle_mode
FlightSimInteger  LandingLightsOn;  //sim/cockpit/electrical/landing_lights_on
boolean bLandingLightsOn;
FlightSimInteger  TaxiLightsOn;    //sim/cockpit/electrical/taxi_light_on
boolean bTaxiLightsOn;
FlightSimFloat  PanelBright;    //sim/cockpit2/EFIS/panel_brightness_ratio 
FlightSimFloat  InstrumentBright;  //sim/cockpit2/EFIS/instrument_brightness_ratio


// Row B  NAV1	NAV2	COM	ADF	XPDR	IDENT

long iNavMode;  // 1 = NAV1, 2 = NAV2, 3 = COM1, 4 = ADF1, 5 = XPDR
long iNavRadio; //1 for NAV1, 2 for NAV2 - variable to hold selected NAV radio on LCD when switching from other iModes
FlightSimInteger AllInOneMode;  // "sim/cockpit2/radios/actuators/nav_com_adf_mode"  0->5 are Nav1, Nav2, Com1, Com2, ADF1, ADF2.
int iAllInOneMode;  
FlightSimInteger NAV1ActiveFrequency; 
FlightSimInteger NAV1StandbyFrequency;
FlightSimCommand NAV1Flip;
FlightSimInteger NAV2ActiveFrequency;
FlightSimInteger NAV2StandbyFrequency;
FlightSimCommand NAV2Flip;
FlightSimInteger COM1ActiveFrequency;
FlightSimInteger COM1StandbyFrequency;
FlightSimCommand COM1Flip;
FlightSimInteger ADF1ActiveFrequency;
FlightSimInteger ADF1StandbyFrequency;
FlightSimCommand ADF1Flip;
int ADF1_channel;  //a variable used to select which number in the code to adjust with the encoder
FlightSimInteger XpdrCode;
FlightSimInteger XpdrMode;
FlightSimCommand XpdrIdent;  // sim/transponder/transponder_ident  
int Xpdr_channel;  //a variable used to select which number in the code to adjust with the encoder
String sXpdrCode;

//Mode 2  (Autopilot)

// Row A  Functions (in order): GS/VNAV  LOC	HDG	ALT	VS	SPD
 
// annunciators for autopilot
FlightSimInteger GSstatus; // sim/cockpit2/autopilot/glideslope_status
FlightSimInteger LOCstatus; // sim/cockpit2/autopilot/nav_status
FlightSimInteger HDGstatus; // sim/cockpit2/autopilot/heading_status
FlightSimInteger ALTstatus; // sim/cockpit2/autopilot/altitude_hold_status
FlightSimInteger VSstatus; // sim/cockpit2/autopilot/vvi_status
FlightSimInteger SPDstatus; // sim/cockpit2/autopilot/speed_status

// - commands for autopilot
FlightSimCommand armGS; // sim/autopilot/glide_slope                          
FlightSimCommand armVNAV; //sim/autopilot/FMS        Autopilot FMS altitude. (VNAV??)
FlightSimCommand armLoc; //sim/autopilot/NAV         Autopilot VOR/LOC arm 
FlightSimCommand armHDG; //sim/autopilot/heading                 
FlightSimCommand armALT;  //sim/autopilot/altitude_hold     
FlightSimCommand armVS;  //sim/autopilot/vertical_speed
FlightSimInteger AutoThrottleOn;   //sim/cockpit2/autopilot/autothrottle_enabled


// Row B  OBS1	OBS2	HDG	ALT	VS	SPD
long iAPMode;

FlightSimFloat OBS1;  //sim/cockpit/radios/nav1_obs_degm
FlightSimFloat OBS2;  //sim/cockpit/radios/nav2_obs_degm
FlightSimFloat AP_Heading; // sim/cockpit2/autopilot/heading_dial_deg_mag_pilot
FlightSimFloat AP_Altitude; // sim/cockpit2/autopilot/altitude_dial_ft
FlightSimFloat AP_VVI; // sim/cockpit2/autopilot/vvi_dial_fpm
FlightSimFloat AP_Airspeed; // sim/cockpit2/autopilot/airspeed_dial_kts_mach


long iMode;
long iHSI;
long iAP;
long iXPDR;
long iGear;
long iDeice;


void setup() {
  
//initialize output hardware

Serial.begin(9600);

pinMode(pinLCD,OUTPUT);
digitalWrite(pinLCD,HIGH);

lcd.begin(20, 4);

sL1 = String ();
sL2 = String ();
sL3 = String ();
sL4 = String ();

//initialize input hardware

pinMode (pinA1, INPUT_PULLUP);
pinMode (pinA2, INPUT_PULLUP);
pinMode (pinA3, INPUT_PULLUP);
pinMode (pinA4, INPUT_PULLUP);
pinMode (pinA5, INPUT_PULLUP);
pinMode (pinA6, INPUT_PULLUP);
pinMode (pinB1, INPUT_PULLUP);
pinMode (pinB2, INPUT_PULLUP);
pinMode (pinB3, INPUT_PULLUP);
pinMode (pinB4, INPUT_PULLUP);
pinMode (pinB5, INPUT_PULLUP);
pinMode (pinB6, INPUT_PULLUP);
pinMode (pinEnc1But, INPUT_PULLUP);
pinMode (pinEnc2But, INPUT_PULLUP);

pinMode (pinTog1A, INPUT_PULLUP);
pinMode (pinTog1B, INPUT_PULLUP);
pinMode (pinTog2A, INPUT_PULLUP);
pinMode (pinTog2B, INPUT_PULLUP);
pinMode (pinTog3A, INPUT_PULLUP);
pinMode (pinTog3B, INPUT_PULLUP);
pinMode (pinTog4A, INPUT_PULLUP);
pinMode (pinTog4B, INPUT_PULLUP);
pinMode (pinTog5A, INPUT_PULLUP);
pinMode (pinTog6A, INPUT_PULLUP);

butA1.attach(pinA1);
butA2.attach(pinA2);
butA3.attach(pinA3);
butA4.attach(pinA4);
butA5.attach(pinA5);
butA6.attach(pinA6);
butB1.attach(pinB1);
butB2.attach(pinB2);
butB3.attach(pinB3);
butB4.attach(pinB4);
butB5.attach(pinB5);
butB6.attach(pinB6);
butEnc1But.attach(pinEnc1But);
butEnc2But.attach(pinEnc2But);

butA1.interval(50);
butA2.interval(50);
butA3.interval(50);
butA4.interval(50);
butA5.interval(50);
butA6.interval(50);
butB1.interval(50);
butB2.interval(50);
butB3.interval(50);
butB4.interval(50);
butB5.interval(50);
butB6.interval(50);
butEnc1But.interval(50);
butEnc2But.interval(50);


// initialize variables
iLightMode = 1; // 1 = landing/taxi, 2 = panel, 3 = instrument
iNavMode = 1; //  1 = NAV1, 2 = NAV2, 3 = COM1, 4 = ADF1, 5 = XPDR
iNavRadio = 1;
iAPMode = 1;  //  1 = OBS1, 2 = OBS2, 3 = HDG, 4 = ALT, 5 = VS, 6 = SPD
//iFMSMode = 1; //  1 = APT, 2 = VOR, 3 = NDB, 4 = FIX

//need to assign a dummy number to encoders values.
oldEnc1 = 0;
oldEnc2 = 0;
sL3 = "";

// X-Plane Variables and Commands

//Toggles
  HSI_Select = XPlaneRef("sim/cockpit2/radios/actuators/HSI_source_select_pilot");  //Tog 2 - HSI
  APMode = XPlaneRef("sim/cockpit2/autopilot/flight_director_mode");  //Tog 3 - Autopilot
  XPDRMode = XPlaneRef("sim/cockpit2/radios/actuators/transponder_mode"); //Tog 4 - xpdr
  GearIsDown = XPlaneRef("sim/cockpit2/controls/gear_handle_down");  // Tog 5 - Gear
  DeIce = XPlaneRef("sim/cockpit2/ice/ice_all_on"); //Tog 6 - DeIce
  
  //Mode 1  (NAV)
  //Row A

  EFISMapMode = XPlaneRef("sim/cockpit2/EFIS/map_mode");
  EFISMapModeHSI = XPlaneRef("sim/cockpit2/EFIS/map_mode_is_HSI");
  EFISMapRange = XPlaneRef("sim/cockpit2/EFIS/map_range");
  iEFISDetail = 1;
  EFIS_weather = XPlaneRef("sim/cockpit2/EFIS/EFIS_weather_on");
  EFIS_tcas = XPlaneRef("sim/cockpit2/EFIS/EFIS_tcas_on");
  EFIS_airport = XPlaneRef("sim/cockpit2/EFIS/EFIS_airport_on");
  EFIS_fix = XPlaneRef("sim/cockpit2/EFIS/EFIS_fix_on");
  EFIS_vor = XPlaneRef("sim/cockpit2/EFIS/EFIS_vor_on");
  EFIS_ndb = XPlaneRef("sim/cockpit2/EFIS/EFIS_ndb_on");
  AirspeedMach = XPlaneRef("sim/cockpit2/autopilot/airspeed_is_mach");
  BankAngle = XPlaneRef("sim/cockpit2/autopilot/bank_angle_mode");
  LandingLightsOn = XPlaneRef("sim/cockpit/electrical/landing_lights_on");
  TaxiLightsOn = XPlaneRef("sim/cockpit/electrical/taxi_light_on");
  PanelBright = XPlaneRef("sim/cockpit2/switches/panel_brightness_ratio");
  InstrumentBright = XPlaneRef("sim/cockpit2/switches/instrument_brightness_ratio");
  
  //Row B
  
  NAV1ActiveFrequency = XPlaneRef("sim/cockpit2/radios/actuators/nav1_frequency_hz");
  NAV1StandbyFrequency = XPlaneRef("sim/cockpit2/radios/actuators/nav1_standby_frequency_hz");
  NAV1Flip = XPlaneRef("sim/radios/nav1_standy_flip");
  NAV2ActiveFrequency = XPlaneRef("sim/cockpit2/radios/actuators/nav2_frequency_hz");
  NAV2StandbyFrequency = XPlaneRef("sim/cockpit2/radios/actuators/nav2_standby_frequency_hz");
  NAV2Flip = XPlaneRef("sim/radios/nav2_standy_flip");
  COM1ActiveFrequency = XPlaneRef("sim/cockpit2/radios/actuators/com1_frequency_hz");
  COM1StandbyFrequency = XPlaneRef("sim/cockpit2/radios/actuators/com1_standby_frequency_hz");  
  COM1Flip = XPlaneRef("sim/radios/com1_standy_flip");
  ADF1ActiveFrequency = XPlaneRef("sim/cockpit2/radios/actuators/adf1_frequency_hz");
  ADF1StandbyFrequency = XPlaneRef("sim/cockpit2/radios/actuators/adf1_standby_frequency_hz");  
  ADF1Flip = XPlaneRef("sim/radios/adf1_standy_flip");
  ADF1_channel = 1;
  XpdrCode = XPlaneRef("sim/cockpit2/radios/actuators/transponder_code");
  XpdrIdent = XPlaneRef("sim/transponder/transponder_ident");
  Xpdr_channel = 1;
  sXpdrCode = XpdrCode;
  AllInOneMode = XPlaneRef("sim/cockpit2/radios/actuators/nav_com_adf_mode");  //0->5 are Nav1, Nav2, Com1, Com2, ADF1, ADF2.
  
  //Mode 2 (AP)
  // Row A  Functions (in order): GS/VNAV  LOC	HDG	ALT	VS	SPD
 
  // annunciators for autopilot
  
  GSstatus = XPlaneRef("sim/cockpit2/autopilot/glideslope_status");
  LOCstatus = XPlaneRef("sim/cockpit2/autopilot/nav_status");
  HDGstatus = XPlaneRef("sim/cockpit2/autopilot/heading_status");
  ALTstatus = XPlaneRef("sim/cockpit2/autopilot/altitude_hold_status");
  VSstatus = XPlaneRef("sim/cockpit2/autopilot/vvi_status");
  SPDstatus = XPlaneRef("sim/cockpit2/autopilot/autothrottle_on");

  // commands for autopilot
  armGS = XPlaneRef("sim/autopilot/glide_slope");
  armVNAV = XPlaneRef("sim/autopilot/FMS");
  armLoc = XPlaneRef("sim/autopilot/NAV");
  armHDG = XPlaneRef("sim/autopilot/heading");
  armALT = XPlaneRef("sim/autopilot/altitude_arm");
  armVS = XPlaneRef("sim/autopilot/vertical_speed");
  AutoThrottleOn = XPlaneRef("sim/cockpit2/autopilot/autothrottle_enabled");
  
  //variables for autopilot
  OBS1 = XPlaneRef("sim/cockpit/radios/nav1_obs_degm");
  OBS2 = XPlaneRef("sim/cockpit/radios/nav2_obs_degm");
  AP_Heading = XPlaneRef("sim/cockpit2/autopilot/heading_dial_deg_mag_pilot");
  AP_Altitude = XPlaneRef("sim/cockpit2/autopilot/altitude_dial_ft");
  AP_VVI = XPlaneRef("sim/cockpit2/autopilot/vvi_dial_fpm");
  AP_Airspeed = XPlaneRef("sim/cockpit2/autopilot/airspeed_dial_kts_mach");

}

void loop() {

FlightSim.update();
  
  //update hardware
butA1.update();
butA2.update();
butA3.update();
butA4.update();
butA5.update();
butA6.update();
butB1.update();
butB2.update();
butB3.update();
butB4.update();
butB5.update();
butB6.update();
butEnc1But.update();
butEnc2But.update();  

long FineDiff = (Enc2.read() - oldEnc2)/4;
long CoarseDiff = (Enc1.read() - oldEnc1)/4;

if(FineDiff || CoarseDiff) 
{
  if (FineDiff > 0) {FineDiff = 1;}// sL3 = "FineDiff = +1";}
  else if (FineDiff < 0) {FineDiff = -1;}// sL3 = "FineDiff = -1";}
  if (CoarseDiff > 0) {CoarseDiff = 1;}// sL3 = "CoarseDiff = +1";}
  else if (CoarseDiff < 0) {CoarseDiff = -1;}// sL3 = "CoarseDiff = -1";}
  
  oldEnc1 = 0;
  oldEnc2 = 0;
  Enc1.write(0); 
  Enc2.write(0);
}

// update toggles if they've changed.
iMode = Tog1();  
iHSI = Tog2();
iAP = Tog3();
iXPDR = Tog4();
iGear = Tog5();
iDeice = Tog6();

if (HSI_Select != iHSI) {HSI_Select = iHSI;}
if (APMode != iAP) {APMode = iAP;}
if (XPDRMode != iXPDR) {XPDRMode = iXPDR;}
if (GearIsDown != iGear) {GearIsDown = iGear;}
if (DeIce != iDeice) {DeIce = iDeice;}

//----------------Main Code--------------------//

if (iMode == 2) // NAV Mode
{
 
  if( butA1.fell() ) {iNavMode = 1;}
  if( butA2.fell() ) {iNavMode = 2;}
  if( butA3.fell() ) {iNavMode = 3;}
  if( butA4.fell() ) {iNavMode = 4;}
  if( butA5.fell() ) {iNavMode = 5;}
  if( butA6.fell() ) {iNavMode = 6;}
    
  if( butB1.fell() ) {iNavMode = 7;}
  if( butB2.fell() ) {iNavMode = 8;}
  if( butB3.fell() ) {iNavMode = 9;}
  if( butB4.fell() ) {iNavMode = 10;}
  if( butB5.fell() ) {iNavMode = 11;}
  if( butB6.fell() ) {iNavMode = 12;}
 

 
  if (iNavMode == 1) // EFIS Mode
  {

    if (FineDiff || CoarseDiff)
    {
      EFISMapMode = EFISMapMode + CoarseDiff;
      if(EFISMapMode < 0) {EFISMapMode = 0;}
      if(EFISMapMode > 4) {EFISMapMode = 4;}
          
      EFISMapModeHSI = EFISMapModeHSI + FineDiff;
      if(EFISMapModeHSI < 0) {EFISMapModeHSI = 0;}
      if(EFISMapModeHSI > 1) {EFISMapModeHSI = 1;}
    }
  }
  else if (iNavMode == 2) // EFIS Zoom and Detail
  {
    
    if (FineDiff || CoarseDiff)
    {     
      EFISMapRange = EFISMapRange + CoarseDiff;
      if(EFISMapRange < 0) {EFISMapRange = 0;}
      if(EFISMapRange > 6) {EFISMapRange = 6;}
      
      iEFISDetail = iEFISDetail + FineDiff;
      if(iEFISDetail < 1) {iEFISDetail = 1;}
      if(iEFISDetail > 6) {iEFISDetail = 6;}
      

      
      switch (iEFISDetail)
      {
      case 1:   //full detail
        EFIS_weather = 1;
        EFIS_tcas = 1;
        EFIS_airport = 1;
        EFIS_fix = 1;
        EFIS_vor = 1;
        EFIS_ndb = 1;
        break;
      case 2:   //full detail - no fix
        EFIS_weather = 1;
        EFIS_tcas = 1;
        EFIS_airport = 1;
        EFIS_fix = 0;
        EFIS_vor = 1;
        EFIS_ndb = 1;     
        break;
      case 3:   //full detail - no fix, no airport
        EFIS_weather = 1;
        EFIS_tcas = 1;
        EFIS_airport = 0;
        EFIS_fix = 0;
        EFIS_vor = 1;
        EFIS_ndb = 1;          
        break;
      case 4:   //only ndb and vor - no weather
        EFIS_weather = 0;
        EFIS_tcas = 1;
        EFIS_airport = 0;
        EFIS_fix = 0;
        EFIS_vor = 1;
        EFIS_ndb = 1;                
        break;
        
      case 5:  //ndb vor airport
        EFIS_weather = 0;
        EFIS_tcas = 1;
        EFIS_airport = 1;
        EFIS_fix = 0;
        EFIS_vor = 1;
        EFIS_ndb = 1;          
        break;
      case 6:  //ndb vor airport Fix
        EFIS_weather = 0;
        EFIS_tcas = 1;
        EFIS_airport = 1;
        EFIS_fix = 1;
        EFIS_vor = 1;
        EFIS_ndb = 1;          
        break;
      }

    }
 
  }
  else if (iNavMode == 3) //Airspeed vs Mach
  {
    /*if (AirspeedMach == 0) {AirspeedMach = 1;}
    else if (AirspeedMach == 1) {AirspeedMach = 0;}
    iNavMode = 1;
    */
  }
  
  if (iNavMode == 4) // Bank Ang 
  {
    if (FineDiff || CoarseDiff)    
    {
      BankAngle = BankAngle + FineDiff + CoarseDiff;
      if (BankAngle < 0) {BankAngle = 0;}
      if (BankAngle > 6) {BankAngle = 6;}
    }     
   }
  else if (iNavMode == 5) // Land/Taxi
  {
      if (FineDiff > 0) {TaxiLightsOn = 1; bTaxiLightsOn = 1;}
      if (FineDiff < 0) {TaxiLightsOn = 0; bTaxiLightsOn = 0;}
      if (CoarseDiff > 0) {LandingLightsOn = 1; bLandingLightsOn = 1;}
      if (CoarseDiff < 0) {LandingLightsOn = 0; bLandingLightsOn = 0;}
  }
  

  
  else if (iNavMode == 6) //Instrument
  {
      if (FineDiff != 0) {PanelBright = PanelBright + FineDiff*0.1;}
      if (PanelBright < 0) {PanelBright = 0;}
      if (PanelBright > 1) {PanelBright = 1;}
      
      if (CoarseDiff != 0) {InstrumentBright = InstrumentBright + CoarseDiff*0.1;}
      if (InstrumentBright < 0) {InstrumentBright = 0;}
      if (InstrumentBright > 1) {InstrumentBright = 1;}
  }  
  
  if (iNavMode == 7) // NAV1
  {
      iNavRadio = 1; 
      iAllInOneMode = 0;
      if( butEnc2But.fell() ) {NAV1Flip = 1;}
      if (FineDiff || CoarseDiff)
      {
        NAV1StandbyFrequency = NAV1StandbyFrequency + (CoarseDiff)*100 + (FineDiff)*5;
        while (NAV1StandbyFrequency < 10800) NAV1StandbyFrequency = NAV1StandbyFrequency + 1000;
        while (NAV1StandbyFrequency >= 11800) NAV1StandbyFrequency = NAV1StandbyFrequency - 1000; 
      }

  }
  else if (iNavMode == 8) // NAV2
  {
     iNavRadio = 2;
     iAllInOneMode = 1;
     if( butEnc2But.fell() ) {NAV2Flip = 1;}
     if (FineDiff || CoarseDiff)
      {
        NAV2StandbyFrequency = NAV2StandbyFrequency + (CoarseDiff)*100 + (FineDiff)*5;
        while (NAV2StandbyFrequency < 10800) NAV2StandbyFrequency = NAV2StandbyFrequency + 1000;
        while (NAV2StandbyFrequency >= 11800) NAV2StandbyFrequency = NAV2StandbyFrequency - 1000;   
      }  
  }
  else if (iNavMode == 9) //COM1
  {
      iAllInOneMode = 2;
      if( butEnc2But.fell() ) {COM1Flip = 1;}
      if (FineDiff || CoarseDiff)
      {
        COM1StandbyFrequency = COM1StandbyFrequency + (CoarseDiff)*100 + (FineDiff)*3;
        //x-plane doesn't carry enough digits to increment from by .025 mhz, so the last digit needs to get adjusted... 
        if (COM1StandbyFrequency%10 == 3 || COM1StandbyFrequency%10 == 8) COM1StandbyFrequency = COM1StandbyFrequency - 1;
        if (COM1StandbyFrequency%10 == 4 || COM1StandbyFrequency%10 ==9) COM1StandbyFrequency = COM1StandbyFrequency + 1;    
        while (COM1StandbyFrequency < 11800) COM1StandbyFrequency = COM1StandbyFrequency + 1800;
        while (COM1StandbyFrequency >= 13600) COM1StandbyFrequency = COM1StandbyFrequency - 1800;       
      }
  }
  else if (iNavMode == 10) // ADF
  {
   iAllInOneMode = 4;
    if (FineDiff || CoarseDiff)
      {
        ADF1_channel = ADF1_channel + CoarseDiff;
        while (ADF1_channel < 1) ADF1_channel = ADF1_channel + 3;
        while (ADF1_channel > 3) ADF1_channel = ADF1_channel - 3;
      
        short ADF[3];
        int a = ADF1ActiveFrequency;
        ADF[3] = a % 10;
        a = a / 10;
        ADF[2] = a % 10; 
        a = a / 10;
        ADF[1] = a % 10;
      
        if (ADF1_channel == 1) 
        {
          ADF[1] = ADF[1] + FineDiff;
          while (ADF[1] < 0) ADF[1] = ADF[1] + 10;
          while (ADF[1] > 9) ADF[1] = ADF[1] - 10;
        } 
        else if (ADF1_channel == 2) 
        {
          ADF[2] = ADF[2] + FineDiff;
          while (ADF[2] < 0) ADF[2] = ADF[2] + 10;
          while (ADF[2] > 9) ADF[2] = ADF[2] - 10;
        } 
        else if (ADF1_channel == 3) 
        {
          ADF[3] = ADF[3] + FineDiff;
          while (ADF[3] < 0) ADF[3] = ADF[3] + 10;
          while (ADF[3] > 9) ADF[3] = ADF[3] - 10;
        }      
  
       ADF1ActiveFrequency = ADF[1] * 100 + ADF[2] * 10 + ADF[3] * 1; 
      }
      
  }
  else if (iNavMode == 11) // XPDR
  {
    if (FineDiff || CoarseDiff)
    {
      Xpdr_channel = Xpdr_channel + CoarseDiff;
      while (Xpdr_channel < 1) Xpdr_channel = Xpdr_channel + 5;
      while (Xpdr_channel > 5) Xpdr_channel = Xpdr_channel - 5;
          
      short Xpdr[4];
      int a = XpdrCode;
    
      Xpdr[4] = a % 10;
      a = a / 10;
      Xpdr[3] = a % 10; 
      a = a / 10;
      Xpdr[2] = a % 10;
      a = a / 10;
      Xpdr[1] = a % 10;    
    
      if (Xpdr_channel == 1) 
      {
        Xpdr[1] = Xpdr[1] + FineDiff;
        while (Xpdr[1] < 0) Xpdr[1] = Xpdr[1] + 8;
        while (Xpdr[1] > 7) Xpdr[1] = Xpdr[1] - 8;
      } 
      else if (Xpdr_channel == 2) 
      {
        Xpdr[2] = Xpdr[2] + FineDiff;
        while (Xpdr[2] < 0) Xpdr[2] = Xpdr[2] + 8;
        while (Xpdr[2] > 7) Xpdr[2] = Xpdr[2] - 8;
      }  
      else if (Xpdr_channel == 3) 
      {
        Xpdr[3] = Xpdr[3] + FineDiff;
        while (Xpdr[3] < 0) Xpdr[3] = Xpdr[3] + 8;
        while (Xpdr[3] > 7) Xpdr[3] = Xpdr[3] - 8;
      } 
      else if (Xpdr_channel == 4) 
      {
        Xpdr[4] = Xpdr[4] + FineDiff;
        while (Xpdr[4] < 0) Xpdr[4] = Xpdr[4] + 8;
        while (Xpdr[4] > 7) Xpdr[4] = Xpdr[4] - 8;
      } 
      else if (Xpdr_channel == 5) 
      {
        XpdrMode = XpdrMode + FineDiff;
        while (XpdrMode < 0) XpdrMode = XpdrMode + 4;
        while (XpdrMode > 3) XpdrMode = XpdrMode - 4;
      }     
     
      XpdrCode = Xpdr[1] * 1000 + Xpdr[2] * 100 + Xpdr[3] * 10 + Xpdr[4];
      sXpdrCode = "";
      for (int i = 1; i < 5; i++) 
      {
       sXpdrCode = sXpdrCode + Xpdr[i]; 
      }
    } 
  }
  else if (iNavMode == 12) // XPDR
  {
    XpdrIdent = 1;
    iNavMode = 11;
  }
    
if (iAllInOneMode != AllInOneMode) {AllInOneMode = iAllInOneMode;}

if (lcd_refresh > 25)
  {
    lcd.setCursor(0,0);   //  bLandingLightsOn  bTaxiLightsOn
    if (bLandingLightsOn) {lcd.print("LAND  ");} else {lcd.print("      ");}
    if (bTaxiLightsOn) {lcd.print("TAXI  ");} else {lcd.print("      ");}
    lcd.print("  BANK ");
    lcd.print(BankAngle);

    
    lcd.setCursor(0,1);
    if (iNavRadio == 1) 
    {
      lcd.print("NAV1 ");    
      lcd.print(NAV1ActiveFrequency/100);
      lcd.print(".");
      if (NAV1ActiveFrequency%100 < 10) {
        lcd.print("0");
        lcd.print(NAV1ActiveFrequency%100);
      }else {
        lcd.print(NAV1ActiveFrequency%100);
      }
      lcd.print("  ");
      if (iNavMode == 7) {lcd.print(">");} else {lcd.print(" ");}
      lcd.print(NAV1StandbyFrequency/100);
      lcd.print(".");
      lcd.print(NAV1StandbyFrequency%100);
    }
    else
    {
      lcd.setCursor(0,1);
      lcd.print("NAV2 ");
      lcd.print(NAV2ActiveFrequency/100);
      lcd.print(".");
      if (NAV2ActiveFrequency%100 < 10) {
        lcd.print("0");
        lcd.print(NAV2ActiveFrequency%100);
      }else {
        lcd.print(NAV2ActiveFrequency%100);
      }
      lcd.print("  ");
      if (iNavMode == 8) {lcd.print(">");} else {lcd.print(" ");}     
      lcd.print(NAV2StandbyFrequency/100);
      lcd.print(".");
      lcd.print(NAV2StandbyFrequency%100);    
    }
    
    
    lcd.setCursor(0,2);
    lcd.print("COM1 ");
    lcd.print(COM1ActiveFrequency/100);
    lcd.print(".");
    lcd.print(COM1ActiveFrequency%100);
    lcd.print("  ");
    if (iNavMode == 9) {lcd.print(">");} else {lcd.print(" ");}
    lcd.print(COM1StandbyFrequency/100);
    lcd.print(".");
    lcd.print(COM1StandbyFrequency%100);
    
    lcd.setCursor(0,3);
    lcd.print("ADF1 ");
    if (iNavMode == 10) {lcd.print(">");} else {lcd.print(" ");}
    if(ADF1ActiveFrequency < 100) {lcd.print("0");}
    if(ADF1ActiveFrequency < 10) {lcd.print("0");}
    lcd.print(ADF1ActiveFrequency);
    lcd.print(" XPDR ");

    if (iNavMode == 11) {lcd.print(">");} else {lcd.print(" ");}
    if (XpdrCode < 1000) {lcd.print("0");}
    if (XpdrCode < 100) {lcd.print("0");}
    if (XpdrCode < 10) {lcd.print("0");}
    lcd.print(XpdrCode);   
  
    lcd_refresh = 0;  
  }

}

if (iMode == 1) //AP Mode
{  
  if( butA1.fell() ) 
  {
    if(HSI_Select == 2) //HSI_Select = 2 -> FMS
    {
      armGS = 0;
      armVNAV = 1;
    }
    else 
    {
      armVNAV = 0;
      armGS = 1;
      
    }
  }
  if( butA2.fell() ) {armLoc = 1;}
  if( butA3.fell() ) {armHDG = 1;iAPMode = 3;}
  if( butA4.fell() ) {armALT = 1; iAPMode = 4;}
  if( butA5.fell() ) {armVS = 1; iAPMode = 5;}
  if( butA6.fell() ) {AutoThrottleOn = !AutoThrottleOn; iAPMode = 6;}
    
  if( butB1.fell() ) {iAPMode = 1;}  //OBS1
  if( butB2.fell() ) {iAPMode = 2;}  //OBS2
  if( butB3.fell() ) {iAPMode = 3;}  //HDG
  if( butB4.fell() ) {iAPMode = 4;}  //ALT
  if( butB5.fell() ) {iAPMode = 5;}  //VS
  if( butB6.fell() ) {iAPMode = 6;}  //SPD
  
  switch (iAPMode) 
  {
    case 1:  
      if (FineDiff || CoarseDiff)
        {
          OBS1 = OBS1 + 10*CoarseDiff + FineDiff;
          while (OBS1 < 0) OBS1 = OBS1 + 360;
          while (OBS1 > 360) OBS1 = OBS1 - 360;
        }
      break;
    case 2:
      if (FineDiff || CoarseDiff)
        {
          OBS2 = OBS2 + 10*CoarseDiff + FineDiff;
          while (OBS2 < 0) OBS2 = OBS2 + 360;
          while (OBS2 > 360) OBS2 = OBS2 - 360;
        }
      break;
    case 3:
      if (FineDiff || CoarseDiff)
        {
          AP_Heading = AP_Heading + 10*CoarseDiff + FineDiff;
          while (AP_Heading < 0) AP_Heading = AP_Heading + 360;
          while (AP_Heading > 360) AP_Heading = AP_Heading - 360;
        }
      break;    
    case 4:
      if (FineDiff || CoarseDiff)
        {
          AP_Altitude = AP_Altitude + 1000*CoarseDiff + 100*FineDiff;
          while (AP_Altitude < 0) AP_Altitude = 0;
          while (AP_Altitude > 99999) AP_Altitude = 99999;
        }
      break;
    case 5:
      if (FineDiff || CoarseDiff)
        {
          AP_VVI = AP_VVI + 1000*CoarseDiff + 100*FineDiff;
          while (AP_VVI < -10000) AP_VVI = -10000;
          while (AP_VVI > 10000) AP_VVI = 10000;
        }
      break;
    case 6:
      if (FineDiff || CoarseDiff)
        {
          if(AirspeedMach == 0)
          {
            AP_Airspeed = AP_Airspeed + 20*CoarseDiff + 1*FineDiff;
            while (AP_Airspeed < 0) AP_Airspeed = 0;
            while (AP_Airspeed > 1000) AP_Airspeed = 1000;
          }
          else
          {
            AP_Airspeed = AP_Airspeed + 0.1*CoarseDiff + .01*FineDiff;
            while (AP_Airspeed < 0) AP_Airspeed = 0;
            while (AP_Airspeed > 0.99) AP_Airspeed = .99; 
          }
        }
      break;
    }
    
  if (lcd_refresh > 25)
  {
   lcd.setCursor(0,0);
   if (GSstatus > 0) {lcd.print("GS ");} else {lcd.print("   ");} 
   if (LOCstatus > 0) {lcd.print("LOC ");} else {lcd.print("    ");}
   if (HDGstatus == 0) {lcd.print("LV ");} else {lcd.print("   ");} 
   if (HDGstatus > 0) {lcd.print("HDG ");} else {lcd.print("    ");} 
   if (VSstatus > 0) {lcd.print("VS ");} else {lcd.print("   ");} 
   if (SPDstatus > 0) {lcd.print("SPD");} else {lcd.print("   ");} 

   
   lcd.setCursor(0,1);
   lcd.print("OBS1 ");
   if (iAPMode == 1) {lcd.print(">");} else {lcd.print(" ");}
   if (OBS1 < 100) {lcd.print("0");}
   if (OBS1 < 10) {lcd.print("0");}
   lcd.print(round(OBS1));
   lcd.print(" OBS2 ");
   if (iAPMode == 2) {lcd.print(">");} else {lcd.print(" ");}
   if (OBS2 < 100) {lcd.print("0");}
   if (OBS2 < 10) {lcd.print("0");}
   lcd.print(round(OBS2));
   lcd.print(" ");
   
   lcd.setCursor(0,2);
   lcd.print("HDG ");
   if (iAPMode == 3) {lcd.print(">");} else {lcd.print(" ");}
   if (AP_Heading < 100) {lcd.print("0");}
   if (AP_Heading < 10) {lcd.print("0");}
   lcd.print(round(AP_Heading));
   lcd.print("  SPD ");
   if (iAPMode == 6) {lcd.print(">");} else {lcd.print(" ");}
   if (AP_Airspeed< 100) {lcd.print("0");}
   if (AP_Airspeed < 10) {lcd.print("0");}  
   lcd.print(round(AP_Airspeed)); 
   lcd.print("  ");
   
   
   lcd.setCursor(0,3);
   lcd.print("ALT ");
   if (iAPMode == 4) {lcd.print(">");} else {lcd.print(" ");}
   if (AP_Altitude < 10000) {lcd.print("0");}
   if (AP_Altitude < 1000) {lcd.print("0");}
   if (AP_Altitude < 100) {lcd.print("00");}
   lcd.print(round(AP_Altitude));
   lcd.print(" VS ");
   if (iAPMode == 5) {lcd.print(">");} else {lcd.print(" ");}
   if(AP_VVI > 0) {lcd.print("+");}
   if(AP_VVI == 0) {lcd.print(" ");}
   if(AP_VVI < 0) {lcd.print("-");}
   lcd.print(round(abs(AP_VVI)));
   if(AP_VVI == 0) {lcd.print ("00 ");}
   if(AP_VVI > -1000 || AP_VVI < 1000) {lcd.print(" ");}
 }
}

if (iMode == 0) //FMS Mode
{
  
  
sL1 = "We're in FMS Mode!";
}

  


}


long Tog1()
{
  boolean bTog1A = digitalRead(pinTog1A);
  boolean bTog1B = digitalRead(pinTog1B);
  long b;
  if (bTog1A == LOW) {b = 0;}
  else if (bTog1B == LOW) {b = 2;}
  else if (bTog1B == HIGH && bTog1A == HIGH) {b = 1;}
  
  return b;
}

long Tog2()
{
  boolean bTog2A = digitalRead(pinTog2A);
  boolean bTog2B = digitalRead(pinTog2B);
  long b;
  if (bTog2A == LOW) {b = 0;}
  else if (bTog2B == LOW) {b = 2;}
  else if (bTog2B == HIGH && bTog2A == HIGH) {b = 1;}
  
  return b;
}


long Tog3()
{
  boolean bTog3A = digitalRead(pinTog3A);
  boolean bTog3B = digitalRead(pinTog3B);
  long b;
  if (bTog3A == LOW) {b = 2;}  
  else if (bTog3B == LOW) {b = 0;}  
  else if (bTog3B == HIGH && bTog3A == HIGH) {b = 1;} 
  
  return b;
}
 
long Tog4()
{
  boolean bTog4A = digitalRead(pinTog4A);
  boolean bTog4B = digitalRead(pinTog4B);
  long b;
  if (bTog4A == LOW) {b = 0;} 
  else if (bTog4B == LOW) {b = 2;} 
  else if (bTog4B == HIGH && bTog4A == HIGH) {b = 1;} 
  
  return b;
}

long Tog5()
{
  boolean bTog5A = digitalRead(pinTog5A);

  long b;
  if (bTog5A == LOW) {b = 1;} 
  else if (bTog5A == HIGH) {b = 0;} 
  
  return b;
}

long Tog6()
{
  boolean bTog6A = digitalRead(pinTog6A);
  long b;
  if (bTog6A == LOW) {b = 0;} 
  else if (bTog6A == HIGH) {b = 1;} 
  
  return b;
}

char * floatToString(char * outstr, double val, byte precision, byte widthp){
 char temp[16];
 byte i;

 // compute the rounding factor and fractional multiplier
 double roundingFactor = 0.5;
 unsigned long mult = 1;
 for (i = 0; i < precision; i++)
 {
   roundingFactor /= 10.0;
   mult *= 10;
 }
 
 temp[0]='\0';
 outstr[0]='\0';

 if(val < 0.0){
   strcpy(outstr,"-\0");
   val = -val;
 }

 val += roundingFactor;

 strcat(outstr, itoa(int(val),temp,10));  //prints the int part
 if( precision > 0) {
   strcat(outstr, ".\0"); // print the decimal point
   unsigned long frac;
   unsigned long mult = 1;
   byte padding = precision -1;
   while(precision--)
     mult *=10;

   if(val >= 0)
     frac = (val - int(val)) * mult;
   else
     frac = (int(val)- val ) * mult;
   unsigned long frac1 = frac;

   while(frac1 /= 10)
     padding--;

   while(padding--)
     strcat(outstr,"0\0");

   strcat(outstr,itoa(frac,temp,10));
 }

 // generate space padding 
 if ((widthp != 0)&&(widthp >= strlen(outstr))){
   byte J=0;
   J = widthp - strlen(outstr);
   
   for (i=0; i< J; i++) {
     temp[i] = ' ';
   }

   temp[i++] = '\0';
   strcat(temp,outstr);
   strcpy(outstr,temp);
 }
 
 return outstr;
}
