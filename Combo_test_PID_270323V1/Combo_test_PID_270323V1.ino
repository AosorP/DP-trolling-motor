/*
  V1 28042022 1)dodana akceleracija stepper motora u proceduri Go_Waypoint

  V1 03052022 1)izmjenjen pin PWM sa D8 na D7, sheeld ploćica prebrikana sa pina D4 na D7, podešena frekvencija D7 na 30.64 Hz
              2)dodana mogučnost LongLeftTurn (LLT) i LongRightTurn (LRT), steper motor vrti konstantno dok se ne prekine s komandom STOP
              komande LLT i LRT se šalju dugim pritiskom tipki Left i Right u Android programu case 18 i 19

  V1 09062022 1) MPU9250 i Qaternion filter ispada iz igre, ulazi LMS303DLHC tilt compesated compass
              2) Izmjenjena logika Go_waypoint procedura (KUT, error, T i Tx)
              3) Usporili malo program sa delay 50 i 100 ms
              4) spustili brzinu I2C na 100 kilobita/s

  V1 12062022 1) Spremanje GPS toćke u Android MitApp aplikaciju i učitavanje iste u Arduino Megu iz Aplikacije
                 Rješenje u slučaju da se točka izgubi zbog gašenje i blokiranja rada Arduina!
                 Izmjenjena MitApp aplikacija
                 020722; M sa 50 na 40; T sa 20° na 40°; Distance_To_Home sa 1.00 na 1.50 m
                 230722; M sa 40 na 30; T vraćen sa 40°na 20°;
                 else if promjenjen u samo if // Go_Waypoint /red 87/ Stepper motor se okreće u smijeru suprotnom od kazaljke na satu

  V1 05092022 1) Dodano enable i desable funkcija drajvera A4988 na pin D4 što gasi i pali stepper driver, stepper ne radi kada nema okretanja
                 nakon eneblePin LOW (aktivacija drivera) pauza od 1 ms da se stigne aktivirati
              2) Kod go_waypoint procedure error se uzima prosječna vrijednost 200 očitanja (cca ??? s)
              3) Serial1 monitor se šalje svako 500 ms kako podatci ne bi bez veze letali po ekranu
              4) Odvojeni previousMillis i curentMillis za stepper i trolling u go waypoint proceduri
              5) Ubačeni faktori PWM 0,4/0,8/ ovisno o kutu odstupanja 0-30°= 100%; 30-50°=80%; 50-90°=40% PWM
                 241122; Distance_To_Home sa 1.50 na 2.00 m

  V1 02122022 1) Dodana PID regulacija približavanja spremljenoj poziciji kP=15; kI=0; kD=70000
                 I je limitiran na +-17300, Integral se racuna bez vremena (početno vrijeme dugo) i poništava se kada pozicija dođe unutar  od 1 m
                 Uvedena dodatna podjela od 1m do 2m od spremljene toćke stepper motor vrti u smjeru kompasa, trolling motor ugašen

  V2 02122022 1) PID regulacija približavanja spremljenoj poziciji kP=70; kI=0.4; kD=100
                 integralno pojačanje se aktivira ispod 4m od spremljene GPS točke, a poništava 2m od točke, limitirano je na 250 x 0,1 = 25
                 derivativno pojačanje se deaktivira kad se uđe u zonu 0,5m od spremljene GPS točke kako GPS drift nebi smetao
              2) trolling motor se ne gasi kad uđe u zonu, već ga PID regulacija gasi, samo se stepper motor gasi kad uđe u zonu radijusa 1,0 m, kako ne bi stalno šetao livo desno!
              3) eliminirano prvo očitanje vrijednosti PID vrijednosti sa funkcijom COUNT++  i if (count > 1)
              4) prosijecna vrijednost kompasa se uzima od 500 točaka umjesto dosadašnjih 100
              5) refrash rate NEO-6M GPS modula postavljen na 5 Hz (200ms), onemogučeno slanje nekih NEMA rečenica kako bi sprijećili nepotrebne podatke

  V2 15122022 1) PID regulacija približavanja spremljenoj poziciji kP=70; kI=0.5; kD=100
                 integralno pojačanje se aktivira ispod 4m od spremljene GPS točke, a poništava 0.2m od točke, limitirano je na 500 x 0,1 = 50
              2) ugašen delay(50) u void loop() i lsm_procedure()
              3) Tx = +-3° ukupno 6°

  V1 11012023 1) Dodan circle mode (kruženje u radijusu od 10-100 m od spermljene GPS točke), radijus se šalje preko bluetootha.
                 Check point histereza 2 m radius oko točke

  V1 14012023 1) Dodant tracking mode (učenje rute do 300 točaka) te vožnja u krug po točkama. Toćke su razmaka 10m,
                 check point histereza 3 m radius oko točke
              2) "CLEAR ALL" bluetooth 16 naredba gasi trolling motor kao i "STOP" bluetooth 5 NAREDBA! Mitt App modificirana da šalje ručno unesene koordinate.

  V1 27032023 1) Ugrađen JOG mode. Mit app šalje heading broda (kompas mobitela, mobitel fiksiran na stalak poravnat sa smijerom broda)preko MITapp aplikacije. Na osnovu headinga broda i GPS 
                 položaja, moguče je fino pomicanje broda za 2m ili koloko već zadamo u programu (jednim pritiskom na jog botune),ljevo, desno, naprijed, nazad.
              2) Spiral mode, slično kao i Circle mode, samo se kruženje odmata u spiralu s posmakom radijusa od 10 m
*/
//******************************************************************************************************
#include "Wire.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303.h>


#define I2Cclock 100000                                 // I2C clock is 100 kilobits/s (standard je 100 = 100000)
#define I2Cport Wire                                    // I2C using Wire library

//******************************************************************************************************
// GPS Variables & Setup

#include <TinyGPS++.h>
int GPS_Course;                                                    // variable to hold the gps's determined course to destination
int Number_of_SATS;                                                // variable to hold the number of satellites acquired
TinyGPSPlus gps;                                                   // gps = instance of TinyGPS

const unsigned char UBLOX_INIT[] PROGMEM = {
  // Rate (pick one)
  //  0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12, //(10Hz)
  0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A, //(5Hz)
  //  0xb5,0x62,0x06,0x08,0x06,0x00,0xFA,0x00,0x01,0x00,0x01,0x00,0x10,0x96, //(4Hz)
  //  0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39, //(1Hz)

  //  Disable specific NMEA sentences
  //  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x24, // GxGGA off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B, // GxGLL off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32, // GxGSA off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39, // GxGSV off
  //  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x04,0x00,0x00,0x00,0x00,0x00,0x01,0x04,0x40, // GxRMC off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x47, // GxVTG off
};

//*******************************************************************************************************
// Compass Variables & Setup

//int desired_heading;                                               // initialize variable - stores value for the new desired heading
int compass_heading;                                                // initialize variable - stores value calculated from compass readings
int boat_heading;
int Tx  = 3;                                                       // maksimalno dozvoljeno odstupanje kuta smjera pri kojem se pali troling motor
//int T = 30;
int error1;
int error;
//int Heading_A;                                                     // variable to store compass heading
//int Heading_B;                                                    // variable to store compass heading in Opposite direction
int pass = 0;                                                      // variable to store which pass the rover is on
int kut;                                                          // angle difference (compass "heading" -  GPS_Course)

// Magnetic declination
#define True_North true         // change this to "true" for True North                
float Declination = +4.32;    // substitute your magnetic declination SPLIT-CROATIA +4.32°

// Adafruit_LSM303
Adafruit_LSM303 lsm;

float Mx, My, Mz; // Raw Magnetometer measurements
float Mxc, Myc, Mzc; // Calibrated Magnetometer measurements
float Mxcn, Mycn, Mzcn; // Normalized Magnetometer measurements

float Ax, Ay, Az; // Raw Accelerometer measurments
float Axc, Ayc, Azc;  // Calibrated Accelerometer mesurements
float Axcn, Aycn, Azcn; // Normalized Accelerometer measurements

float norm_a, norm_m;

double pitch, roll;
double pitch_print, roll_print;
double tiltcnf;

double Xh, Yh;  // Tilt compensated values for heading calculation
float headingct; // Calibrated and tilt compensated heading


//calibrated_values[3] is the global array where the calibrated data will be placed
//calibrated_values[3]: [0]=Xc, [1]=Yc, [2]=Zc
float calibrated_mag_values[3];
float calibrated_acc_values[3];

float alpha = 0.25; // Low-Pass Filtercoefficient

// Filtered Magnetometer measurements
float Mxcnf = 0;
float Mycnf = 0;
float Mzcnf = 0;

// Filtered Accelerometer measurements
float Axcnf = 0;
float Aycnf = 0;
float Azcnf = 0;

//******************************************************************************************************
// Stepper motor

#define dirPin 2 //define D2 as the direction pin of stepper motor driver
#define stepPin 3 //define D3 as the stepp pin of stepper motor driver
#define enablePin 4 //define D4 pin for activation and deactivation of A4899 stepper motor driver
int stepAcc;
int Puls = 6000; //define the speed os stepper motor step "tone" (6000 for 16 microsteps)
int M = 1; //define millis in stepper motor acceleration procedure;  (lover number faster acceleration)

//********************************************************************
// Trolling motor

int runPin = 7; //define D7 as PWM outpot for 30A MOSFET troling motor driver
int pwm;
int a = 1;   //incremental value for PWM (acceleration of trolling motor)
int b = 5;   //incremental value for PWM (deacceleration of trolling motor)
int interval_a = 8; // accel.value for PWM acceleration
int interval_b = 4; // deaccel.value for PWM deacceleration
unsigned long previousMillis = 0;
unsigned long pMillis = 0;
unsigned long p1Millis = 0;
unsigned long pStLMillis = 0;
unsigned long pStRMillis = 0;
unsigned long pTrMillis = 0;
unsigned long pTime = 0;

//*****************************************************************************************************
//PID troling motora

double kp = 70;
double ki = 0.4;
double kd = 100;
double PID_p = 0;
double PID_i = 0;
double PID_d = 0;
static int count = 0;
double lastdelta = 0;
unsigned long previousTime = 0;
unsigned long currentTime;
double delta;
double output;
double setPoint = 0.0;                    //distance (hysteresis) from GPS Setpoint stored position (you can chang this to 2,5 to eliminate GPS drifting)
double cumdelta;
double ratedelta;
double timer;
float Foutput;
//*****************************************************************************************************
// Bluetooth Variables & Setup

String str;                                                        // raw string received from android to arduino
String lati;
String longi;
String heading;
double bluelati;
double bluelongi;
int blueToothVal;                                                  // stores the last value sent over via bluetooth
int bt_Pin = 34;                                                   // Pin 34 of the Aruino Mega used to test the Bluetooth connection status - Not Used
int run_Speed;
int radius;                                                       //circle mode radius

//*****************************************************************************************************
// GPS Locations

float Distance_To_Home;                                            // variable for distance to home
float Distance_To_TrackPoint;
float Distance;
int loop_dir;
int ac = 0;                                                       // GPS array counter
int ab = 0;                                                       // GPS array counter
int ar = 0;                                                       // GPS tracking mode array counter
int ax = 1;                                                       // GPS tracking nuber of tracking points (max number of stored tracking points)
int gpsCount = 0;                                                 // GPS BT storing waypoint counter
int wpCount = 0;                                                  // GPS waypoint counter
int tpCount = 0;                                                  // GPS track point counter
int beta = 0.0;                                                   // beta angle defining point position on circle
double Home_LATarray[5];                                          // variable for storing the destination Latitude - Only Programmed for 1 waypoint
double Home_LONarray[5];                                          // variable for storing the destination Longitude - up to 1 waypoints
double Previouse_LATarray[300];
double Previouse_LONarray[300];
double Circle_LATcorrection;                                       // corection variable to get circle mode points
double Circle_LONcorrection;                                       // corection variable to get circle mode points
double Circle_LATpoint;                                            // circle mode points variable
double Circle_LONpoint;                                            // circle mode points variable
double Home_LAT[1];
double Home_LON[1];
int increment = 0;
int C = 0;
double Previouse_LAT;
double Previouse_LON;
double Curent_LAT;
double Curent_LON;


// Processing variables
char InputChar;                                         // incoming characters stored here
bool LinkEstablished = false;                           // receive flag
String OutputString = "";                               // outgoing data string to Processing

// Software timer
unsigned long Timer1 = 10000L;                         // 100mS loop ... used when sending data to to Processing
unsigned long Stop1;                                    // Timer1 stops when micros() exceeds this value

// setup()
//***********************************************************************************************
void setup() {

  //----- Set frequency of clock for PWM
  //TCCR4B = TCCR4B & B11111000 | B0000001; // for PWM frequency of  31372.55 Hz; devider 1; pin D8 mega 8 bit
  //TCCR4B = TCCR4B & B11111000 | B0000010; // for PWM frequency of  3921.16 Hz; devider 4; pin D8 mega 8 bit
  //TCCR4B = TCCR4B & B11111000 | B0000011; // for PWM frequency of  490.20 Hz; devider 16; pin D8 mega 8 bit
  //TCCR4B = TCCR4B & B11111000 | B0000100; // for PWM frequency of  122.55 Hz; devider 64; pin D8 mega 8 bit
  TCCR4B = TCCR4B & B11111000 | B0000101; // for PWM frequency of 30.64 Hz; devider 256; pin D6,D7,D8 mega 8 bit

  //-----Set output pins for Stepper motor and Trolling motor
  pinMode(dirPin, OUTPUT);//  Defines D2 pin as outpoot
  pinMode(stepPin, OUTPUT);//  Defines D3 pin as outpoot
  pinMode(runPin, OUTPUT);//  Defines D7 pin as outpoot
  pinMode(enablePin, OUTPUT);// Defines D4 pin as outpoot
  delay(50);
  digitalWrite(enablePin, HIGH); //disable stepper driver A4988



  // ----- Enable comunications
  Wire.begin();
  Wire.setClock(I2Cclock);                                      // 100 kbit/sec I2C speed
  Serial.begin(115200);                                         // Serial 0 is for communication with the computer
  //while (!Serial);                                            // Not required for Arduino UNO but needed when porting code to the M4 Express
  Serial1.begin(9600);                                          // Serial 1 is for Bluetooth communication - DO NOT MODIFY - JY-MCU HC-06 v1.40
  Serial2.begin(9600);                                          // Serial 2 is for GPS communication at 9600 baud -  - Ublox Neo 6-7m defoiult 9600
  lsm.begin();
  //************************************************************************
  //GPS-setup
  for (unsigned int i = 0; i < sizeof(UBLOX_INIT); i++) {           //U-BLOX reprogramiranje NEMA sentence i Rate
    Serial2.write( pgm_read_byte(UBLOX_INIT + i) );
  };
  delay(50);
  //*************************************************************************
  // ----- start software timer
  Stop1 = micros() + Timer1;          // Used by send_data() function

  // ----- start startup procedure
  Startup();                           // Run the Startup procedure on power-up one time

}

// loop()
// ***************************************************************************
void loop() {
  bluetooth();                                 // Run the Bluetooth procedure to see if there is any data being sent via BT
  getGPS();                                    // Update the GPS location
  lsm_procedure();                             // Update LSM calculation

}
