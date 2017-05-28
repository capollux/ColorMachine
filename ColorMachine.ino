#include <Wire.h>
#include <Math.h>
byte i2cWriteBuffer[10];
byte i2cReadBuffer[10];
#define SensorAddressWrite 0x29
#define SensorAddressRead 0x29 
#define EnableAddress 0xa0 // register address + command bits
#define ATimeAddress 0xa1 // register address + command bits
#define WTimeAddress 0xa3 // register address + command bits
#define ConfigAddress 0xad // register address + command bits
#define ControlAddress 0xaf // register address + command bits
#define IDAddress 0xb2 // register address + command bits
#define ColorAddress 0xb4 // register address + command bits

// PIN numbers
#define DIR_PIN_C 2
#define STEP_PIN_C 3
#define DIR_PIN_M 4
#define STEP_PIN_M 5
#define DIR_PIN_Y 7
#define STEP_PIN_Y 6
#define DIR_PIN_K 8
#define STEP_PIN_K 9
#define DIR_PIN_W 12
#define STEP_PIN_W 11

/*  
Send register address and the byte value you want to write the magnetometer and 
loads the destination register with the value you send
*/
void Writei2cRegisters(byte numberbytes, byte command)
{
    byte i = 0;
    Wire.beginTransmission(SensorAddressWrite);   // Send address with Write bit set
    Wire.write(command);                          // Send command, normally the register address 
    for (i=0;i<numberbytes;i++)                   // Send data 
      Wire.write(i2cWriteBuffer[i]);
    Wire.endTransmission();
    delayMicroseconds(100);      // Allow some time for bus to settle      
}

/*  
Send register address to this function and it returns byte value
for the magnetometer register's contents 
*/
byte Readi2cRegisters(int numberbytes, byte command) {
  byte i = 0;
  Wire.beginTransmission(SensorAddressWrite);   // Write address of read to sensor
  Wire.write(command);
  Wire.endTransmission();
  delayMicroseconds(100);      // Allow some time for bus to settle      
  Wire.requestFrom(SensorAddressRead,numberbytes);   // Read data
  for(i=0;i<numberbytes;i++)
    i2cReadBuffer[i] = Wire.read();
  Wire.endTransmission();   
  delayMicroseconds(100);      // Allow some time for bus to settle      
}

/*
Init color sensor
*/
void init_TCS34725(void) {
  i2cWriteBuffer[0] = 0x10;
  Writei2cRegisters(1,ATimeAddress);    // RGBC timing is 256 - contents x 2.4mS =  
  i2cWriteBuffer[0] = 0x00;
  Writei2cRegisters(1,ConfigAddress);   // Can be used to change the wait time
  i2cWriteBuffer[0] = 0x00;
  Writei2cRegisters(1,ControlAddress);  // RGBC gain control
  i2cWriteBuffer[0] = 0x03;
  Writei2cRegisters(1,EnableAddress);    // Enable ADs and oscillator for sensor  
}

/*
Check the sensor is present
*/
void get_TCS34725ID(void) {
  Readi2cRegisters(1,IDAddress);
  if (i2cReadBuffer[0] = 0x44) {
    Serial.println("TCS34725 is present");    
  } else {
    Serial.println("TCS34725 not responding");    
  }
}

/*
Reads the register values for clear, red, green, and blue.
*/
unsigned int clear_color = 0;

unsigned int cyan_color = 0;
unsigned int magenta_color = 0;
unsigned int yellow_color = 0;
unsigned int black_color = 0;
unsigned int water_vol = 0;

void get_Colors(void)
{

  unsigned int red_color = 0;
  unsigned int green_color = 0;
  unsigned int blue_color = 0;
  double cyan = 0;
  double magenta = 0;
  double yellow = 0;
  double black = 0;
  
  Readi2cRegisters(8,ColorAddress);
  clear_color = (unsigned int)(i2cReadBuffer[1]<<8) + (unsigned int)i2cReadBuffer[0];
  red_color = (unsigned int)(i2cReadBuffer[3]<<8) + (unsigned int)i2cReadBuffer[2];
  green_color = (unsigned int)(i2cReadBuffer[5]<<8) + (unsigned int)i2cReadBuffer[4];
  blue_color = (unsigned int)(i2cReadBuffer[7]<<8) + (unsigned int)i2cReadBuffer[6];
  // Send register values to the serial monitor 
  
  Serial.println("****Color Scan****");
  
  // [RGB colors]
  // Range is 0-65535
  /*
  Serial.print("clear color=");
  Serial.print(clear_color, DEC);    
  Serial.print(" red color=");
  Serial.print(red_color, DEC);    
  Serial.print(" green color=");
  Serial.print(green_color, DEC);    
  Serial.print(" blue color=");
  Serial.println(blue_color, DEC);
  */
  
  // Change range scale 
  clear_color = clear_color/257;
  red_color = red_color/257;
  green_color = green_color/257;
  blue_color = blue_color/257;
  

  // Changed range is 0-255
  Serial.print("clear color=");
  Serial.print(clear_color, DEC);    
  Serial.print(" red color=");
  Serial.print(red_color, DEC);       
  Serial.print(" green color=");
  Serial.print(green_color, DEC);       
  Serial.print(" blue color=");
  Serial.println(blue_color, DEC);
  
  // [RGB > CMYK]
  // RGB values = 0 ÷ 255
  // CMY values = 0 ÷ 1

  cyan = 1 - ((double)red_color/255);
  magenta = 1 - ((double)green_color/255);
  yellow = 1 - ((double)blue_color/255);
 
  // Where CMYK and CMY values = 0 ÷ 1
  black = 1;

  if ( cyan < black )  black = cyan;
  if ( magenta < black )  black = magenta;
  if ( yellow < black )  black = yellow;
  if ( black == 1 ) {
     cyan = 0;
     magenta = 0;
     yellow = 0;
  } else {
     cyan = ( cyan - black ) / ( 1 - black );
     magenta = ( magenta - black ) / ( 1 - black );
     yellow = ( yellow - black ) / ( 1 - black );
  }
  
  cyan_color = cyan*100;
  magenta_color = magenta*100;
  yellow_color = yellow*100;
  black_color = black*100;
  water_vol = 400-(cyan_color + magenta_color + yellow_color + black_color);
  
  // [CMYK colors]
  Serial.print("cyan color=");
  Serial.print(cyan_color, DEC);
  Serial.print(" magenta color=");
  Serial.print(magenta_color, DEC);
  Serial.print(" yellow color=");
  Serial.print(yellow_color, DEC);
  Serial.print(" black color=");
  Serial.print(black_color, DEC);
  Serial.print(" water=");
  Serial.println(water_vol, DEC);
}  

void rotateDeg(float deg, float speed, int dir_pin, int step_pin){ 
  // Rotate a specific number of degrees (negitive for reverse movement)
  // Speed is any number from .01 -> 1 with 1 being fastest - Slower is stronger
  int dir = (deg > 0)? HIGH:LOW;
  digitalWrite(dir_pin, dir); 

  int steps = abs(deg)*(1/0.225);
  float usDelay = (1/speed) * 70;

  for(int i=0; i < steps; i++){ 
    digitalWrite(step_pin, HIGH);
    delayMicroseconds(usDelay);

    digitalWrite(step_pin, LOW);
    delayMicroseconds(usDelay);
  } 
}

void setup() {
  Wire.begin();
  //Serial.begin(9600);  // Start serial for output, comment when use arduino stand alone
  init_TCS34725();
  get_TCS34725ID();     // Get the device ID, this is just a test to see if we're connected
  
  pinMode(DIR_PIN_C, OUTPUT); // Set PINS
  pinMode(DIR_PIN_M, OUTPUT); 
  pinMode(DIR_PIN_Y, OUTPUT); 
  pinMode(DIR_PIN_K, OUTPUT);
  pinMode(DIR_PIN_W, OUTPUT);
  pinMode(STEP_PIN_C, OUTPUT);
  pinMode(STEP_PIN_M, OUTPUT); 
  pinMode(STEP_PIN_Y, OUTPUT);
  pinMode(STEP_PIN_K, OUTPUT);
  pinMode(STEP_PIN_W, OUTPUT);
}

void loop() {
  get_Colors();
  delay(1000);

  if(clear_color > 100) { // Check distance between sensor and color
  
    
    // Rotation test
    /*
    rotateDeg(10, .03, DIR_PIN_C, STEP_PIN_C);
    rotateDeg(10, .03, DIR_PIN_M, STEP_PIN_M);
    rotateDeg(10, .03, DIR_PIN_Y, STEP_PIN_Y);
    rotateDeg(10, .03, DIR_PIN_K, STEP_PIN_K);
    rotateDeg(10, .03, DIR_PIN_W, STEP_PIN_W);
    Serial.println("Done");
    */
    
    // Reuse
    /*
    rotateDeg(15, .03, DIR_PIN_C, STEP_PIN_C);
    Serial.println("Rotate reverse");
    */
  
    // Rotate motors
    rotateDeg(((double)magenta_color/400)*150, .03, DIR_PIN_M, STEP_PIN_M); // Magenta
    Serial.println("Rotate Magenta");
    rotateDeg(((double)cyan_color/400)*150, .03, DIR_PIN_C, STEP_PIN_C); // Cyan
    Serial.println("Rotate Cyan");
    rotateDeg(((double)black_color/900)*150, .03, DIR_PIN_K, STEP_PIN_K); // Black
    Serial.println("Rotate Black");
    rotateDeg(((double)yellow_color/350)*150, .03, DIR_PIN_Y, STEP_PIN_Y); // Yellow
    Serial.println("Rotate Yellow");
    rotateDeg(((double)water_vol/230)*150, .03, DIR_PIN_W, STEP_PIN_W); // Water
    Serial.println("Rotate Water");
    
    delay(10000);
  }
  delay(2000);
}
