/////////////////////////////////////////////////////////////////
//                Nunchuck Controlled Turret                   //
//           Written by Charles Dawson, June 2013              //
//              Released under the MIT License                 //
/////////////////////////////////////////////////////////////////

//Special thanks to todbot for the nunchuck library and Paul Badger for the digitalSmooth algorithm

/*
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#include <Wire.h>;
#include <Servo.h>;

//Nunchuck stuff
static uint8_t nunchuck_data[6];   // array to store nunchuck data,

//Servos
Servo aimServo;
Servo triggerServo;

//Calibration Values for servos and sensor
int aimOffset = 2; //Fill in with values from BallisticDuino_Calibration.ino
int triggerOpen = 80; //Fill in with values from BallisticDuino_Calibration.ino
int triggerClosed = 152; //Fill in with values from BallisticDuino_Calibration.ino
int verticalReading = 125; //Fill in with values from the Serial Monitor
int levelReading = 75; //Fill in with values from the Serial Monitor

//Pins
int triggerServoPin = 6;
int aimServoPin = 9;
int nunchuckPowerPin = A3;
int nunchuckGndPin = A2;

//Stuff for digitalSmooth
static const int filterSamples = 13;
int nunchuckSmoothArray[filterSamples];

//===============Setup==================
void setup() {
  //Begin serial
  Serial.begin(9600);
  
  //Attach servos
  triggerServo.attach(triggerServoPin);
  triggerServo.write(triggerClosed); //Quickly do this so that it doesn't fire on reset
  aimServo.attach(aimServoPin);

  //Set up nunchuck power pins
    //You can power the nunchuck off of the GPIOs b/c
    //you know that it won't pull too much current
  nunchuck_setpowerpins();
  
  //Initialize nunchuck
  Wire.begin();
  nunchuck_init();
}
  
//===============Loop===================
void loop() {
  //Get data from nunchuck and print it out
  getNunchuckData();
  Serial.println(nunchuck_accely());
  
  //First aim
  //Get reading for the angle
  int aimReading = nunchuck_accely();
  //Normalize it
  if(aimReading > verticalReading){ aimReading = verticalReading; }
  else if(aimReading < levelReading){ aimReading = levelReading; }
  //Smooth it
  aimReading = digitalSmooth(aimReading, nunchuckSmoothArray);
  //Map it to an angle between 0 and 90
  int aimAngle = map(aimReading, verticalReading, levelReading, 0, 90);
  aimServo.write(aimAngle + aimOffset);
    
  //Then check the triggers. The Z button opens the trigger, the C button closes it
  byte zbutt = nunchuck_zbutton();
  byte cbutt = nunchuck_cbutton();
  Serial.print("\tzbut: "); Serial.print((byte)zbutt,DEC);
  Serial.print("\tcbut: "); Serial.println((byte)cbutt,DEC);
  if( zbutt != 0){ triggerServo.write(triggerOpen); }
  else if( cbutt != 0){ triggerServo.write(triggerClosed); }
    
  delay(1);
    
}

// Uses port C (analog in) pins as power & ground for Nunchuck
static void nunchuck_setpowerpins()
{
#define pwrpin PORTC3
#define gndpin PORTC2
    DDRC |= _BV(pwrpin) | _BV(gndpin);
    PORTC &=~ _BV(gndpin);
    PORTC |=  _BV(pwrpin);
    delay(100);  // wait for things to stabilize        
}

// Join the I2C bus,
// and tell the nunchuck we're talking to it
static void nunchuck_init() { 
    Wire.beginTransmission(0x52);// transmit to device 0x52
    Wire.write((uint8_t)0x40);// sends memory address
    Wire.write((uint8_t)0x00);// sends sent a zero.  
    Wire.endTransmission();// stop transmitting
}

// Send a request for data to the nunchuck
static void nunchuck_send_request(){
    Wire.beginTransmission(0x52);// transmit to device 0x52
    Wire.write((uint8_t)0x00);// sends one byte
    Wire.endTransmission();// stop transmitting
}

// Encode data to format that most wiimote drivers except
// only needed if you use one of the regular wiimote drivers
static char nunchuk_decode_byte (char x){
    x = (x ^ 0x17) + 0x17;
    return x;
}

// Receive data back from the nunchuck, 
// returns 1 on successful read. returns 0 on failure
static int getNunchuckData(){
    int cnt=0;
    Wire.requestFrom (0x52, 6);// request data from nunchuck
    while (Wire.available ()) {
        // receive byte as an integer
        nunchuck_data[cnt] = nunchuk_decode_byte( Wire.read() );
        cnt++;
    }
    nunchuck_send_request();  // send request for next data payload
    // If we recieved the 6 bytes, then go print them
    if (cnt >= 5) {
        return 1;   // success
    }
    return 0; //failure
}

// returns zbutton state: 1=pressed, 0=notpressed
static int nunchuck_zbutton(){
    return ((nunchuck_data[5] >> 0) & 1) ? 0 : 1;  // voodoo
}

// returns zbutton state: 1=pressed, 0=notpressed
static int nunchuck_cbutton(){
    return ((nunchuck_data[5] >> 1) & 1) ? 0 : 1;  // voodoo
}

// returns value of x-axis joystick
static int nunchuck_joyx(){
    return nunchuck_data[0]; 
}

// returns value of y-axis joystick
static int nunchuck_joyy(){
    return nunchuck_data[1];
}

// returns value of x-axis accelerometer
static int nunchuck_accelx(){
    int accel_x_axis = nunchuck_data[2]; //Get first part of accelerometer data
    if ((nunchuck_data[5] >> 2) & 1) //Fill in LSBs from the last byte of the data
        accel_x_axis += 1;
    if ((nunchuck_data[5] >> 3) & 1)
        accel_x_axis += 2;
    return accel_x_axis;
}

// returns value of y-axis accelerometer
static int nunchuck_accely(){
    int accel_y_axis = nunchuck_data[3]; //Get first part of accelerometer data
    if ((nunchuck_data[5] >> 4) & 1) //Fill in LSBs from the last byte of the data
        accel_y_axis += 1;
    if ((nunchuck_data[5] >> 5) & 1)
        accel_y_axis += 2;
    return accel_y_axis;
}

// returns value of z-axis accelerometer
static int nunchuck_accelz(){
    int accel_z_axis = nunchuck_data[4]; //Get first part of accelerometer data
    if ((nunchuck_data[5] >> 6) & 1) //Fill in LSBs from the last byte of the data
        accel_z_axis += 1;
    if ((nunchuck_data[5] >> 7) & 1)
        accel_z_axis += 2;
    return accel_z_axis;
}

//Smooths the input from the nunchuck
int digitalSmooth(int rawIn, int *sensSmoothArray){     // "int *sensSmoothArray" passes an array to the function - the asterisk indicates the array name is a pointer
  int j, k, temp, top, bottom;
  long total;
  static int i;
 // static int raw[filterSamples];
  static int sorted[filterSamples];
  boolean done;

  i = (i + 1) % filterSamples;    // increment counter and roll over if necc. -  % (modulo operator) rolls over variable
  sensSmoothArray[i] = rawIn;                 // input new data into the oldest slot

  // Serial.print("raw = ");

  for (j=0; j<filterSamples; j++){     // transfer data array into anther array for sorting and averaging
    sorted[j] = sensSmoothArray[j];
  }

  done = 0;                // flag to know when we're done sorting              
  while(done != 1){        // simple swap sort, sorts numbers from lowest to highest
    done = 1;
    for (j = 0; j < (filterSamples - 1); j++){
      if (sorted[j] > sorted[j + 1]){     // numbers are out of order - swap
        temp = sorted[j + 1];
        sorted [j+1] =  sorted[j] ;
        sorted [j] = temp;
        done = 0;
      }
    }
  }

/*
  for (j = 0; j < (filterSamples); j++){    // print the array to debug
    Serial.print(sorted[j]); 
    Serial.print("   "); 
  }
  Serial.println();
*/

  // throw out top and bottom 15% of samples - limit to throw out at least one from top and bottom
  bottom = max(((filterSamples * 15)  / 100), 1); 
  top = min((((filterSamples * 85) / 100) + 1  ), (filterSamples - 1));   // the + 1 is to make up for asymmetry caused by integer rounding
  k = 0;
  total = 0;
  for ( j = bottom; j< top; j++){
    total += sorted[j];  // total remaining indices
    k++; 
    // Serial.print(sorted[j]); 
    // Serial.print("   "); 
  }

//  Serial.println();
//  Serial.print("average = ");
//  Serial.println(total/k);
  return total / k;    // divide by number of samples
}
