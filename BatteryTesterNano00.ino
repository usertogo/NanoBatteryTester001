/*
  Tests to sort Batteries for Pack assembly
  Used Analog input, serial output

 Reads an analog input pin, representing the voltage of the cell
 Callibrates current, the tests with two types of load 
 measures voltage and current each time and estimates internal resistance of the battery 
 uses repeated measurement and calculates a standard deviation in order to know the quality of results
 
 prints the test results to the serial monitor.

 The circuit:
 * 

analogReference() configures the reference voltage used for analog input (i.e. the value used as the top of the input range). The options are: 
    DEFAULT: the default analog reference of 5 volts (on 5V Arduino boards) or 3.3 volts (on 3.3V Arduino boards)
    INTERNAL: an built-in reference, equal to 1.1 volts on the ATmega168 or ATmega328 and 2.56 volts on the ATmega8 (not available on the Arduino Mega)
    INTERNAL1V1: a built-in 1.1V reference (Arduino Mega only)
    INTERNAL2V56: a built-in 2.56V reference (Arduino Mega only)
    EXTERNAL: the voltage applied to the AREF pin (0 to 5V only) is used as the reference. Used here and bridged to 3.3V via a 1kOhm resistor
   http://www.arduino.cc/en/Reference/AnalogReference 
 */
#include <EEPROM.h>

int baseAddr = 10;

// These constants won't change.  They're used to give names
// to the pins used:
const int aVoltSenseInPin = A0;  // Analog input pin for the first measured Voltage
const int aCurrentSensePin = A7;  // Analog input pin for the ACS712 DC Current Sensor Voltage
const int aStartButton = 12;  // Digital input pin Start button is High when pressed
const int aNanoLED = 13;  // Provide heartbeat showing ready state and Solid while testing (HIGH=on)

const int aLoadPin = 3; // High turns on the bigger Load ( red LED)
const int bLoadPin = 5; // High turns on the smaller Load (green LED)
const int onState = HIGH;  // the state of the output pin that turns the load on
const int offState = LOW;  // the state of the output pin that turns the load off
const char DOONE = 88;

int cycleRepeat = 10;
int heartbeatCount = 0;
int heartbeatTop = 50;
int heartbeatDelay = 20;
int measureDelay = 20;
int incSerial = 1;
int verboseLevel = 1;      // small number less info

float senseVValue = 0;        // value read representing Voltage input
float openVValue  = 0;        // value read representing unloaded Voltage input
float aCurrentZero = 512; // No Current meassured before turning loads on
float currentRawVal = 0;        // value read from the Current sensor ACS712
float currentValue = 0;        // normalized for zero
int thisSerial;
bool reverseHB = false;  // Hearbeat in Reverse to indicate condition like command received!
int reverseHBcount;
int reverseHBcycles = 4;  //Default number of Reversed no further info

float kVRefFactor = 1.349257*1.081571 ; // from 5V(4.74) to 3.3V (3.548) Reference Voltage
float kCurrentFactor = 0.025857/kVRefFactor*1.08667;
float kVoltFactor = 0.00805785/kVRefFactor;
float noLoadVoltage;        // representing open circuit Battery Voltage
float VdropA;
float VdropB;
float CurrentA;
float CurrentB;
float RintA;
float RintB;
float deviation;      // for now a global result of multi measuring functions
float openVdeviation, currentDeviation, loadedVdeviation; //helps to see where the variability comes from...


void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  Serial.println("\t Welcome to the Battery Evaluator 0.101" );
  // setup all IO
  pinMode(aNanoLED, OUTPUT);
  pinMode(aLoadPin, OUTPUT);
  digitalWrite(aLoadPin, offState);
  pinMode(bLoadPin, OUTPUT);
  digitalWrite(bLoadPin, offState);
  pinMode(aStartButton, INPUT);
  analogReference(EXTERNAL); // DEFAULT);
//  setSerial(300);
}

void loop()
  {
    float RintSamples[cycleRepeat];
    while(!digitalRead(aStartButton))
      {
        char some = inputProcess();
        if(some==DOONE)  // possible command processing
              break;
        if (reverseHB)
          {
            reverseHBcount--;
            if(reverseHBcount<1)
             { 
                reverseHBcount = reverseHBcycles;
                reverseHB = false;  // resets flagged condition to normal HB
             }
          }    
        heartbeatCount++;
        if(heartbeatCount>=heartbeatTop)
          {
            digitalWrite(aNanoLED, HIGH^reverseHB);
            heartbeatCount = 0;
            delay(heartbeatDelay);
            digitalWrite(aNanoLED, LOW^reverseHB);
          }
         else delay(heartbeatDelay);
      }
    // We get here because the start button was pressed
    thisSerial = getSerial(baseAddr);
    setSerial(thisSerial+incSerial);  
    int repeat = 0;
    float sumRint = 0;
//    if() 
    {
      Serial.print("\tSerial=," );
      Serial.print(thisSerial);
      while(repeat<cycleRepeat)
      {
            
         if(verboseLevel>4)
         {
            //Serial.print("\t,Serial=," );
            //Serial.print(thisSerial+1);
            Serial.print("\t,Cycle=," );
            Serial.print(cycleRepeat-repeat+1);
         }
            
            RintSamples[repeat] = measureSDevDisplay(repeat); //measureAndDisplay();
            sumRint += RintSamples[repeat];
            //Serial.print(repeat);
            repeat++;
      }
      // calculate stdDeviation
      float averageRint = (float)sumRint / cycleRepeat;
      if(verboseLevel==1)
         {
            //Serial.print("\tSerial=," );
            //Serial.print(thisSerial);
            Serial.print("\t,Rint=," );
            Serial.print(averageRint,4);
         }
      float variance = 0;
      for(int i=0; i<cycleRepeat ; i++)
      {
        variance += pow( (RintSamples[i] - averageRint) , 2);
        //Serial.print(i);
      }
      deviation = sqrt((float)variance/cycleRepeat);
      if(verboseLevel==1)
         {
            Serial.print("\t,stdDev=," );
            Serial.println(deviation,5);
            //Serial.print("\t,Rint=," );
            //Serial.print(averageRint);
         }
      
    }
    
 // if the button is still pressed wait for its release and show a slower Heartbeat   
    while(digitalRead(aStartButton))
      {
        heartbeatCount++;
        if(heartbeatCount>=2)
          {
            digitalWrite(aNanoLED, HIGH);
            heartbeatCount = 0;
            delay(500);
            digitalWrite(aNanoLED, LOW);
          }
         else delay(500);
      }
  }

float measureSDevDisplay(int cycle)
  { 
          
          //doMeasure();
       openVValue = getStdDevMultiSense(aVoltSenseInPin,10,measureDelay);
       openVdeviation = deviation * kVoltFactor;
       noLoadVoltage = openVValue * kVoltFactor;
       if(verboseLevel>1 || cycle==0 )
       {
          Serial.print("\t,OpenVolt=," );
          Serial.print(noLoadVoltage);
       }
       if(verboseLevel>5)
       {
          Serial.print("\t,OVdev=," );
          Serial.print(openVdeviation);
       }
          delay(10);
          aCurrentZero = getStdDevMultiSense(aCurrentSensePin,10,measureDelay);   // calibrate every time
          currentDeviation = deviation * kCurrentFactor;
       if(verboseLevel>6)
       {
          Serial.print("\t,rawZC=," );
          Serial.print(aCurrentZero);
          Serial.print("\t ZCdev = " );
          Serial.print(currentDeviation);
       }
          digitalWrite(bLoadPin, onState);                            // TURN ON small load first
//          delay(10); 
//          senseVValue = getStdDevMultiSense(aVoltSenseInPin,10,measureDelay);
//          loadedVdeviation = deviation * kVoltFactor;
//          Serial.print("\t smLoadVdev = " );
//          Serial.print(loadedVdeviation);
//          VdropB = (openVValue-senseVValue)*kVoltFactor;
//          delay(5); 
//          currentRawVal = getStdDevMultiSense(aCurrentSensePin,10,measureDelay);
//          currentDeviation = deviation * kCurrentFactor;
//          Serial.print("\t smLoadCdev = " );
//          Serial.print(currentDeviation);
          digitalWrite(aLoadPin, onState);                            // TURN ON bigger load
//          CurrentB = (currentRawVal-aCurrentZero)*kCurrentFactor;
//          RintB = VdropB / CurrentB;
//          Serial.print("\t RintB = " );
//          Serial.print(RintB);
          delay(50); 
          senseVValue = getStdDevMultiSense(aVoltSenseInPin,10,measureDelay);
          loadedVdeviation = deviation * kVoltFactor;
          if(verboseLevel>4)
          {
              Serial.print("\t,LoadVolt=," );
              Serial.print(senseVValue*kVoltFactor);
              Serial.print("\t,bgLoadVdev=," );
              Serial.print(loadedVdeviation);
          }
          VdropA = (openVValue-senseVValue)*kVoltFactor;
      //    delay(5000); 
          delay(10); 
          currentRawVal = getStdDevMultiSense(aCurrentSensePin,10,measureDelay);
          currentDeviation = deviation * kCurrentFactor;
          if(verboseLevel>5)
           {
              Serial.print("\t,rawLC=," );
              Serial.print(currentRawVal);
              Serial.print("\t,bgLoadCdev=," );
              Serial.print(currentDeviation);
           }
          // delay(5000);                                             // uncomment for current calibration...m
          digitalWrite(aLoadPin, offState);
          digitalWrite(bLoadPin, offState);
          CurrentA = (currentRawVal-aCurrentZero)*kCurrentFactor;
          RintA = VdropA / CurrentA;
          
       // print the results to the serial monitor:
      
          if(verboseLevel>1)
          {
              Serial.print("\t,RintA=," );
              Serial.print(RintA,3);                // print 3 digits after decimal
          }
          if(verboseLevel>3)
          {
              Serial.print("\t,CurrentMax=," );
              Serial.print(CurrentA);
          }
          if(verboseLevel>1)
          {
              Serial.println(";");
          }
          return RintA;
    }
    
void measureAndDisplay(void)
  { 
          
          //doMeasure();
          openVValue = analogXRead(aVoltSenseInPin);
          noLoadVoltage = openVValue * kVoltFactor;
          Serial.print("\t OpenVoltage = " );
          Serial.print(noLoadVoltage);
          delay(10);
          aCurrentZero = analogXRead(aCurrentSensePin);
          digitalWrite(bLoadPin, onState); // TURN ON small load first
          delay(10); 
          senseVValue = analogXRead(aVoltSenseInPin);
          VdropB = (openVValue-senseVValue)*kVoltFactor;
          delay(5); 
          currentRawVal = analogXRead(aCurrentSensePin);
          digitalWrite(aLoadPin, onState); // TURN ON bigger load
          CurrentB = (currentRawVal-aCurrentZero)*kCurrentFactor;
          RintB = VdropB / CurrentB;
          Serial.print("\t RintB = " );
          Serial.print(RintB);
          delay(10); 
          senseVValue = analogXRead(aVoltSenseInPin);
          VdropA = (openVValue-senseVValue)*kVoltFactor;
      //    delay(5000); 
          delay(10); 
          currentRawVal = analogXRead(aCurrentSensePin);
      //    delay(3000);
          digitalWrite(aLoadPin, offState);
          digitalWrite(bLoadPin, offState);
          CurrentA = (currentRawVal-aCurrentZero)*kCurrentFactor;
          RintA = VdropA / CurrentA;
          
       // print the results to the serial monitor:
      
          Serial.print("\t RintA = " );
          Serial.print(RintA);
          Serial.print("\t CurrentMax = " );
          Serial.println(CurrentA);
          
    }
    
float getStdDevMultiSense(int pin, int reads, int mSdelay)
    {
      int measures[reads];
      int sumSense = 0;
      for(int i=0; i<reads ; i++)
      {
        measures[i] = analogRead(pin);
        sumSense += measures[i];
        delay(mSdelay);
      }
      float average= (float)sumSense / reads;
      float variance = 0;
      for(int i=0; i<reads ; i++)
      {
        variance += pow((float)measures[i] - average, 2);
      }
      deviation = sqrt((float)variance/reads);
      return (average);
    }
    
int analogXRead(int port)
  {
    int sumVal=0;
    for(int i=0; i<10; i++)
      { 
        sumVal += analogRead(port); 
        delay(5);
      }
    return sumVal;
  }
void setSerial(int theNum)
  {
    int aByte = theNum & 255;
//    Serial.print("Serial1 = " );
//    Serial.print(aByte);
    EEPROM.write(baseAddr, aByte);
    aByte = (theNum>>8) & 255;
    EEPROM.write(baseAddr+1, aByte);
//    Serial.print(" Serial2 = " );
//    Serial.println(aByte);
  }
int getSerial(int theBase)
  {
    int theSerial = EEPROM.read(theBase)+EEPROM.read(theBase+1)*256;
//    Serial.print("SerialRead = " );
//    Serial.println(theSerial);
    return(theSerial);
  }
  
char inputProcess(void)
  {
    char what;
    if(hasNewCommand()>0)
        {
          what = Serial.read();
          if(verboseLevel>3)
          {
            Serial.print("\t Received = ");
            Serial.println(what);
          }
          switch (what) 
            {
              case 'R':
              case 'r':
                        //lastCommand=RESET;
                        setSerial(0);
                        clearCommand();
                        reverseHB = true;
                        break;
              case 'M':
              case 'm':
                        //lastCommand=DOONE;
                        what=DOONE;
                        reverseHB = true;
                        clearCommand();
                        break;
              case 'd':
              case 'D':
                        //lastCommand=DOWN;
                        thisSerial = getSerial(baseAddr);
                        setSerial(thisSerial-1);
                        clearCommand();
                        reverseHB = true;
                        break;
              case 'v':
              case 'V':
                        //lastCommand=VERB;
                        verboseLevel=7;
                        incSerial=0;
                        clearCommand();
                        reverseHB = true;
                        break;
              case 'p':
              case 'P':
                        //lastCommand=Production;
                        verboseLevel=1;
                        incSerial=1;
                        clearCommand();
                        reverseHB = true;
                        break;                        
            } 
          }  
          return what;
  }
  
int hasNewCommand()
  {
    // if we get a valid byte, read analog ins:
    int receiveData = Serial.available();
    return receiveData;
  }
 int clearCommand()
  {
    // remove everything from incomming buffer
    int came = 0;
    while (Serial.available()) 
      { // get incoming data:
        came += Serial.read();
      }
    return came;
  }
 int getNewCommand()
  {
    // if we get a valid byte, read analog ins:
    int receiveData = Serial.available();
    int inByte = 0;
    if (receiveData > 0) 
      { // get incoming data:
        inByte = Serial.read();
      }
    return inByte;
  }
 
/*  
void doMeasure()
  {
    // read the analog in value:
    senseVValue = analogRead(aVoltSenseInPin);
    
    Voltage=senseVValue*.05235;
    delay(2);
    currentRawVal = analogRead(aCurrentSensePin);
    currentValue = currentRawVal - aCurrentZero ;
    Current = currentValue * .0123;
  }
void doShow()
  {
    // print the results to the serial monitor:
    Serial.print("sensor = " );
    Serial.print(senseVValue);
    Serial.print("\t Voltage = " );
    Serial.print(Voltage);
    Serial.print("\t CurrentVal = " );
    Serial.print(currentValue);
    Serial.print("\t Current = " );
    Serial.println(Current);

  }
*/
