
int K1Pin = 8;
int K2Pin = 7;


// Declaring these here as globals
float pH_slope;  //Computed slope for the linear interpolation 
float pH_offset; //Computed offset fo the linear interpolation

// Operational Targets (Eventually this will be configurable)

float pHtarget = 7.0;  //Target pH value 
float highDeadband = 0.05; // Offset value to smooth out life
float lowDeadband = 0.05;

//Integer that defines the current state: 0 = Off, 1 = ON, 2 = Paused
int injectActive = 0;

// Timer Values

unsigned long StartMillis = 0; // This is the time when the current period started (inject or pause)
unsigned long ElapsedMillis = 0;


/***************************************************
 This example uses software solution to calibration the ph meter, not the potentiometer. So it is more easy to use and calibrate.
 This is for SEN0161 and SEN0169.
 
 Created 2016-8-11
 By youyou from DFrobot <youyou.yu@dfrobot.com>
  
 GNU Lesser General Public License.
 See <http://www.gnu.org/licenses/> for details.
 All above must be included in any redistribution
 ****************************************************/
 
/***********Notice and Troubleshooting***************
 1.Connection and Diagram can be found here  http://www.dfrobot.com/wiki/index.php/PH_meter%28SKU:_SEN0161%29
 2.This code is tested on Arduino Uno.
 ****************************************************/

/* Not Using EEPROM
#include <EEPROM.h>
#define EEPROM_write(address, p) {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) EEPROM.write(address+i, pp[i]);}
#define EEPROM_read(address, p)  {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) pp[i]=EEPROM.read(address+i);}
*/

#define ReceivedBufferLength 20
char receivedBuffer[ReceivedBufferLength+1];   // store the serial command
byte receivedBufferIndex = 0;

#define SCOUNT  30           // sum of sample point
int analogBuffer[SCOUNT];    //store the sample voltage
int analogBufferIndex = 0;

/* Not using the EEPROM
#define SlopeValueAddress 0     // (slope of the ph probe)store at the beginning of the EEPROM. The slope is a float number,occupies 4 bytes.
#define InterceptValueAddress (SlopeValueAddress+4) 
*/

// float slopeValue, interceptValue, 
float averageVoltage, pHvalue;
boolean enterCalibrationFlag = 0;  // Set this value through serial to enter calibration

#define SensorPin A0
#define VREF 5000  //for arduino uno, the ADC reference is the power(AVCC), that is 5000mV

//Using Adafruit Libraries
/*********************************************************************
This is an example for our Monochrome OLEDs based on SSD1306 drivers

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/category/63_98

This example is for a 128x64 size display using I2C to communicate
3 pins are required to interface (2 I2C and one reset)

Adafruit invests time and resources providing this open source code, 
please support Adafruit and open-source hardware by purchasing 
products from Adafruit!

Written by Limor Fried/Ladyada  for Adafruit Industries.  
BSD license, check license.txt for more information
All text above, and the splash screen must be included in any redistribution
*********************************************************************/

//Defines the Adafruit libraries I don't necessarily know what it does.

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <gfxfont.h>

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2


#define LOGO16_GLCD_HEIGHT 16 
#define LOGO16_GLCD_WIDTH  16 
static const unsigned char PROGMEM logo16_glcd_bmp[] =
{ B00000000, B11000000,
  B00000001, B11000000,
  B00000001, B11000000,
  B00000011, B11100000,
  B11110011, B11100000,
  B11111110, B11111000,
  B01111110, B11111111,
  B00110011, B10011111,
  B00011111, B11111100,
  B00001101, B01110000,
  B00011011, B10100000,
  B00111111, B11100000,
  B00111111, B11110000,
  B01111100, B11110000,
  B01110000, B01110000,
  B00000000, B00110000 };


void setup()
{
  //Serial.begin(115200);  // If using display turn off the serial monitor to PC
  Serial.begin(9600);
  
  Serial.println("setup start");
  
  //readCharacteristicValues(); //read the slope and intercept of the ph probe
  
//Define relay pins as outputs.
pinMode(K1Pin, OUTPUT);
pinMode(K2Pin, OUTPUT); 

Serial.println("pin defined");

// This is the Adafruit Serial Connection

  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!) This initializes the controller
  
  
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x64)
  
  
  // init done
  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();
  
  Serial.println("before delay");
  
  delay(2000);

// Calibrating on Start up - Until programatic calibration is available I will calibrate here and save the value.

float pH_HighVal = 9.18; // High value pH solution - in this case it's 9.18
float pH_LowVal = 4.01;  // Low value pH solution - in this case it's 4.00

float pH_HighMV = 3418; // 
float pH_LowMV = 1190; //

pH_slope = (pH_HighVal - pH_LowVal) / (pH_HighMV - pH_LowMV);
pH_offset = pH_LowVal - pH_slope*pH_LowMV;

}

void loop()
{

  
  /* Commenting out this function which is part of the original auto cal.
  if(serialDataAvailable() > 0)
  {
    Serial.println("serial data available");
      byte modeIndex = uartParse();
      phCalibration(modeIndex);    // If the correct calibration command is received, the calibration function should be called.
      EEPROM_read(SlopeValueAddress, slopeValue);     // After calibration, the new slope and intercept should be read ,to update current value.
      EEPROM_read(InterceptValueAddress, interceptValue);

  }
  */

  // Read the analog value from the sensor
   static unsigned long sampleTimepoint = millis();
   if(millis()-sampleTimepoint>40U)
   {
     sampleTimepoint = millis();
     analogBuffer[analogBufferIndex] = analogRead(SensorPin)/1024.0*VREF;    //read the voltage and store into the buffer,every 40ms
     analogBufferIndex++;
     if(analogBufferIndex == SCOUNT) 
         analogBufferIndex = 0;
     averageVoltage = getMedianNum(analogBuffer,SCOUNT);   // read the stable value by the median filtering algorithm
   }
   
   static unsigned long printTimepoint = millis();
   if(millis()-printTimepoint>1000U)
   {
     printTimepoint = millis();

     /*
     if(enterCalibrationFlag)             // in calibration mode, print the voltage to user, to watch the stability of voltage
     {
       Serial.print("Voltage:");
       Serial.print(averageVoltage);
       Serial.println("mV");
     }else
    */
   }       
     
     pHvalue = averageVoltage*pH_slope+pH_offset;

/* This block is the implementation of the pulsing on off command for the CO2 when it is active. 
 *  
 * There are different conditions that are possible: 
 *  1) CO2 above threshold
 *  1.1) CO2 above threshold - Initial event
 *  1.2) CO2 above threshold - Elasped time ok
 *  1.3) CO2 above threshold - Elasped time ended
 *  2) CO2 below threshold
 */


// High pH event 

ElapsedMillis = millis() - StartMillis;

if(pHvalue > pHtarget + highDeadband){
  
  // Condition 1.1 Initial event - Set inject Active to 1 (currently injecting)
  if(injectActive == 0){
    injectActive = 1;
    StartMillis = millis();
    Serial.println("Condition 1.1 - Initial detection");
  }
}
  // Condition 1.2 - High pH but within the elapsed time - No Change
  
  // Condition 1.3 - High pH but elapsed time expired
  if(injectActive > 0){
    if(ElapsedMillis > 30000){
      StartMillis = millis(); //reset timer
      if(injectActive == 1){
        injectActive = 2;
        Serial.println("Condition 1.3 - switch to paused");
        }
      else if (injectActive ==2){
        Serial.println("Condition 1.3 - switch to active");
        injectActive = 1;
        }
     }
     else {
      Serial.println("Condition 1.2 - Within Time");
     }
    }
 
   
     
if(pHvalue < pHtarget - lowDeadband){
  injectActive = 0;
}

if(injectActive  == 1){
  injectCO2(1);
  Serial.println("injecting");

}
else{
  injectCO2(0);
  Serial.println("not injecting");
}


    //Display Stuff
    // Clear the buffer.
    display.clearDisplay();

    // Displaying Text - We'll figure out how to do this.
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    display.print("pH Controller");
    if (injectActive == 1){
      display.println(" Active");
    }
     else if(injectActive == 2){
      display.println(" Paused");
     }
      else {
        display.print("\n");
     }
    
    display.setTextColor(WHITE); // To get 'inverted' text us setTextColor(BLACK, WHITE)
    display.print(averageVoltage);
    display.println("mV");
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.print("pH"); 
    display.println(pHvalue);
    display.display();


    Serial.print("pH:");              // in normal mode, print the ph value to user
    Serial.print(pHvalue);
    Serial.print("  mV:");
    Serial.println(averageVoltage);    
    //Serial.println(injectActive);

    }      
  



boolean serialDataAvailable(void)
{
  char receivedChar;
  static unsigned long receivedTimeOut = millis();
  while (Serial.available()>0) 
  {   
    if (millis() - receivedTimeOut > 1000U) 
    {
      receivedBufferIndex = 0;
      memset(receivedBuffer,0,(ReceivedBufferLength+1));
    }
    receivedTimeOut = millis();
    receivedChar = Serial.read();
    if (receivedChar == '\n' || receivedBufferIndex==ReceivedBufferLength){
    receivedBufferIndex = 0;
    strupr(receivedBuffer);
    return true;
    }
    else{
      receivedBuffer[receivedBufferIndex] = receivedChar;
      receivedBufferIndex++;
    }
  }
  return false;
}

byte uartParse()
{
  byte modeIndex = 0;
  if(strstr(receivedBuffer, "CALIBRATION") != NULL) 
      modeIndex = 1;
  else if(strstr(receivedBuffer, "EXIT") != NULL) 
      modeIndex = 4;
  else if(strstr(receivedBuffer, "ACID:") != NULL)   
      modeIndex = 2;  
  else if(strstr(receivedBuffer, "ALKALI:") != NULL)
      modeIndex = 3;
  return modeIndex;
}

/*

void phCalibration(byte mode)
{
    char *receivedBufferPtr;
    static byte acidCalibrationFinish = 0, alkaliCalibrationFinish = 0;
    static float acidValue,alkaliValue;
    static float acidVoltage,alkaliVoltage;
    float acidValueTemp,alkaliValueTemp,newSlopeValue,newInterceptValue;
    switch(mode)
    {
      case 0:
      if(enterCalibrationFlag)
         Serial.println(F("Command Error"));
      break;
      
      case 1:
      Serial.println("Case 1");
      
      receivedBufferPtr=strstr(receivedBuffer, "CALIBRATION");
      enterCalibrationFlag = 1;
      acidCalibrationFinish = 0;
      alkaliCalibrationFinish = 0;
      Serial.println("Enter Calibration Mode");
      break;
     
      case 2:
      if(enterCalibrationFlag)
      {
        Serial.println("Case 2");
          receivedBufferPtr=strstr(receivedBuffer, "ACID:");
          receivedBufferPtr+=strlen("ACID:");
          acidValueTemp = strtod(receivedBufferPtr,NULL);
          if((acidValueTemp>3)&&(acidValueTemp<5))        //typical ph value of acid standand buffer solution should be 4.00
          {
             acidValue = acidValueTemp;
             acidVoltage = averageVoltage/1000.0;        // mV -> V
             acidCalibrationFinish = 1;
             Serial.println(F("Acid Calibration Successful"));
           }else {
             acidCalibrationFinish = 0;
             Serial.println(F("Acid Value Error"));
           }
      }
      break;
 
       case 3:

       Serial.println("Case 3");
       if(enterCalibrationFlag)
       {

          Serial.println("Case 3a");
           receivedBufferPtr=strstr(receivedBuffer, "ALKALI:");
           receivedBufferPtr+=strlen("ALKALI:");
           alkaliValueTemp = strtod(receivedBufferPtr,NULL);
           if((alkaliValueTemp>8)&&(alkaliValueTemp<11))        //typical ph value of alkali standand buffer solution should be 9.18 or 10.01
           {
                 alkaliValue = alkaliValueTemp;
                 alkaliVoltage = averageVoltage/1000.0;
                 alkaliCalibrationFinish = 1;
                 Serial.println(F("Alkali Calibration Successful"));
            }else{

              Serial.println("Case 3b");
               alkaliCalibrationFinish = 0;
               Serial.println(F("Alkali Value Error"));
            }
       }
       break;

        case 4:
        if(enterCalibrationFlag)
        {
            if(acidCalibrationFinish && alkaliCalibrationFinish)
            {
              newSlopeValue = (acidValue-alkaliValue)/(acidVoltage - alkaliVoltage);
              EEPROM_write(SlopeValueAddress, newSlopeValue);
              newInterceptValue = acidValue - (slopeValue*acidVoltage);
              EEPROM_write(InterceptValueAddress, newInterceptValue);
              Serial.print(F("Calibration Successful"));
            }
            else Serial.print(F("Calibration Failed"));       
            Serial.println(F(",Exit Calibration Mode"));
            acidCalibrationFinish = 0;
            alkaliCalibrationFinish = 0;
            enterCalibrationFlag = 0;
        }
        break;

    }
    
}


*/

int getMedianNum(int bArray[], int iFilterLen) 
{
      int bTab[iFilterLen];
      for (byte i = 0; i<iFilterLen; i++)
      {
    bTab[i] = bArray[i];
      }
      int i, j, bTemp;
      for (j = 0; j < iFilterLen - 1; j++) 
      {
    for (i = 0; i < iFilterLen - j - 1; i++) 
          {
      if (bTab[i] > bTab[i + 1]) 
            {
    bTemp = bTab[i];
          bTab[i] = bTab[i + 1];
    bTab[i + 1] = bTemp;
       }
    }
      }
      if ((iFilterLen & 1) > 0)
  bTemp = bTab[(iFilterLen - 1) / 2];
      else
  bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
      return bTemp;
}

/*
void readCharacteristicValues()
{
    EEPROM_read(SlopeValueAddress, slopeValue);
    EEPROM_read(InterceptValueAddress, interceptValue);
    if(EEPROM.read(SlopeValueAddress)==0xFF && EEPROM.read(SlopeValueAddress+1)==0xFF && EEPROM.read(SlopeValueAddress+2)==0xFF && EEPROM.read(SlopeValueAddress+3)==0xFF)
    {
      slopeValue = 3.5;   // If the EEPROM is new, the recommendatory slope is 3.5.
      EEPROM_write(SlopeValueAddress, slopeValue);
    }
    if(EEPROM.read(InterceptValueAddress)==0xFF && EEPROM.read(InterceptValueAddress+1)==0xFF && EEPROM.read(InterceptValueAddress+2)==0xFF && EEPROM.read(InterceptValueAddress+3)==0xFF)
    {
      interceptValue = 0;  // If the EEPROM is new, the recommendatory intercept is 0.
      EEPROM_write(InterceptValueAddress, interceptValue);
    }
}
*/

void injectCO2(bool setting)
{

  if(setting){
    digitalWrite(K2Pin, LOW); //Close K2 
    Serial.println("Injecting CO2");
  }
  else{
    digitalWrite(K2Pin, HIGH); //Open K2
    Serial.println("Turning off CO2");
  }
  

}
