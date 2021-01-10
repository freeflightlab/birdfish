//V4 -> To do implement environmental sensors and SD data logging 

//------ Development log -----
//level 1 - Create Servo Objects for 2 servos - Complete
//level 2 - other AHRS sensors -- taken out--
//level 3 - 
//level 4 - Calculate GPS alt, Current Position, Bearing to Goal - Complete
//level 5 - 
//level 6 -  heading based turn - complete
//level 7  - adjusting gains for altitude based on flight 1 - needs to be tested

//level 8 bme sensors
//level 9 sd data logging 

#include <PWMServo.h> //level 1
#include <SPI.h>
#include <TinyGPS.h> //level4

PWMServo myservo1;  // level 1 create servo object servo1
PWMServo myservo2;  // level 1 create servo object servo2

TinyGPS gps; //level 4

HardwareSerial Uart = HardwareSerial(); //level4
//--------------------
int pos1 = 140; // to set initial positions of servos
int pos2 = 40;  // to set initial position of servos

float poscom1 = pos1;
float poscom2 = pos2;
int n = 0;
int xint=0;
int loopcnt=100;

int turnlevel=50;

float heador = 0;
float headorOLD = 0;

// GOAL WAYPOINT GPS
float latgoal=37.362081;
float longgoal=-120.993190;

float brng = 0;
float brng1 = 0;
float turngain = .25;
float altgain = 1; //gain for altitude 1 <3000m

float ERRORhead = 1; // difference between heading and bearing
float ERabs = 1; 
float ERsplit=1;

float GoalRadius=1; //1km goal 

//..............................................................................//
// *******. Use for Testing Altitude based logic! yea it mostly works ********. 

float testalt= 5; //meters
float dist_calc0 = .2;//initial distance to goal.. needs to be larger than goal
float dist_calc = 10;

//..............................................................................//

//-------------------
//-------------------
unsigned long course = 0;
//-------------------

void gpsdump(TinyGPS &gps);
void printFloat(double f, int digits = 2);



void setup() {
  Serial.begin(9600);
   Uart.begin(9600);
    
    myservo1.attach(3); //set servo 1 to pin 3 pwm level 1
    myservo2.attach(4); //set servo 2 to pin 4 pwm level 1
  Serial.println("hi");   //did the servo part work? Hi test
     
  Serial.println("started");
  Serial.print("Servo1 Position, ");Serial.println(pos1);//level1
  Serial.print("Servo2 Position, ");Serial.println(pos2);//level1

    Serial.println();
}



void loop() {
  
  myservo1.write(pos1);
  myservo2.write(pos2);

   bool newdata = false; //level4
  unsigned long start = millis(); //level4

   // Every 5 seconds we print an update //level4
  while (millis() - start < 5000) {
    if (Uart.available()) {
      char c = Uart.read();
       //Serial.print(c);  // ..............uncomment to see raw GPS data................
      if (gps.encode(c)) {
        newdata = true;
        //break;  // uncomment to print new data immediately!
      }
    }
  }

  
// adjust altitudes gains for density -- Level 7

 if(gps.f_altitude() > 5000) 
  altgain=.1;
  
if(gps.f_altitude() < 5000 && gps.f_altitude() > 3000)
  altgain=.5;

if(gps.f_altitude() < 3000)
  altgain=1; 
 



// When you are below threshold altitude and close to goal. Switch into celebration mode. 

//while(gps.f_altitude() < 1000 && dist_calc < GoalRadius){ 

while(testalt<15 && dist_calc0 < GoalRadius){ //for testing
  
  
  delay(50);
myservo1.write(100);
myservo2.write(pos2);
Serial.print("right turn"); Serial.println();
Serial.print("Goal");Serial.println();
Serial.print(testalt);Serial.println();
delay(45000);
Serial.print("Hands Up");Serial.println();
myservo1.write(pos1);
myservo2.write(pos2);
delay(20000);
Serial.print("left turn");Serial.println();
myservo1.write(pos1);
myservo2.write(80);
delay(45000);
Serial.print("Hands Up");Serial.println();
myservo1.write(pos1);
myservo2.write(pos2);
delay(20000);
Serial.print("Distance to Goal V4: ");
    Serial.println(dist_calc);Serial.println();
}



//**************head for waypoint between release and waypoint

//while((gps.f_altitude()) > 1000 && gps.f_altitude() < 16150 ){

while(testalt >15 && testalt < 1000){ //for testing
  
  myservo1.write(pos1);
  myservo2.write(pos2);
  Serial.println();

brng = brng1;
 
//hands up for +/- 10 degrees on heading
  if(ERsplit < 180 && ERsplit > 160){
    turngain=1; 
   poscom1=pos1;
   poscom2 = pos2; 
   Serial.print("hands up");Serial.println();
}
 if (ERsplit < 20 && ERsplit > 0){
   turngain=1; 
   poscom1=pos1;
   poscom2 = pos2;
    Serial.print("hands up");Serial.println();
   }

   //

//small turn from +/- 10 to +/-45
  if(ERsplit < 160 && ERsplit > 135)
    turngain=0.4;
  if (ERsplit <45 && ERsplit >20)
    turngain=0.4;
    
//medium turn from +/-45 to +/-67.5
  if(ERsplit < 135 && ERsplit > 112.5)
    turngain=.7;
  if(ERsplit < 67.5 && ERsplit > 45)
    turngain=.7;

//fullturn from +/- 90 to +/- 67.5
  if(ERsplit < 112.5 && ERsplit > 67.5)
  turngain = .2;
 
Serial.print(" pos2   ");Serial.println(poscom2);
Serial.print(" pos1   ");Serial.println(poscom1);

if(ERRORhead > 0 && ERabs > 180){
  myservo2.write(poscom2);
  delay(500);
  Serial.println("Left (+)(>180)"); Serial.println();
}
if(ERRORhead > 0 && ERabs < 180){
  myservo1.write(poscom1);
  delay(500);
  Serial.println("Right (+)(<180)"); Serial.println();
}
if(ERRORhead < 0 && ERabs > 180){
  myservo1.write(poscom1);
  delay(500);
  Serial.println("Right (-)(>180)"); Serial.println();
}
if(ERRORhead < 0 && ERabs < 180){
  myservo2.write(poscom2);
  delay(500);
  Serial.println("Left (-)(<180)"); Serial.println();
}

heador = course; //orientation.heading;// 

//heador=200;// ........comment out orientation and put value to test


//-----------------print heading .. in this case course. 
Serial.println(heador);

ERRORhead = brng - (heador);
ERabs = sqrt(sq(ERRORhead));
ERsplit = ERabs/2;

poscom1= floor(pos1-(turnlevel*turngain*altgain));
poscom2= floor(pos2+(turnlevel*turngain*altgain));

Serial.println("Error in Heading:   ");
Serial.print(ERRORhead); Serial.println();

    Serial.print(testalt);
    Serial.println(" m");
    Serial.println("");
    if (newdata) {
    Serial.println("Acquired Data");
    Serial.println("-------------");
    gpsdump(gps);
    Serial.println("-------------");
    Serial.println();
    delay(2000);
    
  }

  Serial.print(F("Duration in Seconds:  "));
  Serial.println(float(millis())/1000);

  
}


//************************Full Stall **************************

//while(gps.f_altitude()> 16150 && gps.f_altitude()<24400){

while(testalt>1000 && testalt <10000) {// for testing
  
//Stall
myservo1.write(70);
myservo2.write(120);
Serial.println("FullStall");
Serial.println();
  
   
   if (newdata) {
    Serial.println("Acquired Data");
    Serial.println("-------------");
    gpsdump(gps);
    Serial.println("-------------");
    Serial.println();

    delay(2000);
}

delay(1000);
} 

}


//GPS function Level 4 Calculate GPS alt, Current Position, Bearing to Goal

void gpsdump(TinyGPS &gps)
{
  
  long lat, lon;
  float flat, flon;
  unsigned long age, date, time, course;//, chars;
  int year;
  byte month, day, hour, minute, second, hundredths;
 
   
   float dist_calc=0;
  float dist_calc2=0;   
   
   gps.get_position(&lat, &lon, &age);
   gps.f_get_position(&flat, &flon, &age);
course = gps.course();
   float teta1 = radians(flat);
    float teta2 = radians(latgoal);
    float delta1 = radians(latgoal- flat);
    float delta2 = radians(longgoal- flon);
  
  
  Serial.print("Lat/Long(float): "); printFloat(flat, 5); Serial.print(", "); printFloat(flon, 5);
  Serial.print(" Fix age: "); Serial.print(age); Serial.println("ms.");
  Serial.print("Alt(meters): "); printFloat(gps.f_altitude()); Serial.println();
  Serial.print("Speed(knots): "); printFloat(gps.f_speed_knots()); Serial.println();

  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  Serial.print("Date: "); Serial.print(static_cast<int>(month)); Serial.print("/"); 
    Serial.print(static_cast<int>(day)); Serial.print("/"); Serial.print(year);
  Serial.print("  Time: "); Serial.print(static_cast<int>(hour)); Serial.print(":"); 
    Serial.print(static_cast<int>(minute)); Serial.print(":"); Serial.print(static_cast<int>(second));
    Serial.print("."); Serial.print(static_cast<int>(hundredths));
     Serial.println(" ");

//calcuate bearing
  float y = sin(delta2) * cos(teta2);
    float x = cos(teta1)*sin(teta2) - sin(teta1)*cos(teta2)*cos(delta2);
    brng1 = atan2(y,x);
    brng1 = degrees(brng1);// radians to degrees
    brng1 = ( ((int)brng1 + 360) % 360 ); 
     
     Serial.print("Bearing to Goal: ");
    Serial.println(brng1);

      //calculate range
dist_calc = (sin(delta1/2.0)*sin(delta1/2.0));
dist_calc2= cos(teta1);
dist_calc2*=cos(teta2);
dist_calc2*=sin(delta2/2.0);
dist_calc2*=sin(delta2/2.0);
dist_calc +=dist_calc2;

dist_calc=(2*atan2(sqrt(dist_calc),sqrt(1.0-dist_calc)));

dist_calc*=6371000.0 / 1000; //Converting to meters -> km
//Serial.println(dist_calc);

Serial.print("Distance to Goal: ");
    Serial.println(dist_calc);


}

  void printFloat(double number, int digits)
{
  // Handle negative numbers
  if (number < 0.0) {
     Serial.print('-');
     number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;
  
  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  Serial.print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    Serial.print("."); 

  // Extract digits from the remainder one at a time
  while (digits-- > 0) {
    remainder *= 10.0;
    int toPrint = int(remainder);
    Serial.print(toPrint);
    remainder -= toPrint;
  }
}
