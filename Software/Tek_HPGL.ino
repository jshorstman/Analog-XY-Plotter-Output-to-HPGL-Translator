/*****************************************************************************************|
 * Tektronix 2232 Digital Storage Oscilloscope analog X-Y pen plotter to HP-GL 
 * translator. (Hewlett-Packard Graphics Language http://en.wikipedia.org/wiki/HPGL)
 * 
 * John Horstman
 * 3/17/2021 Revision M
 */
 const String REVISION = "M"; // Current revision
 /* 
 * This sketch continuously reads the X and Y analog pen plotter voltages, and the 
 * pen up/down signal, digitizes them and packages the ADC values in successive HP-GL 
 * commands. The commands can be serially output to a file, via TeraTerm, or to a plotter 
 * application like SPLOT.
 *  
 * The HP-GL commands used in this sketch are:
 * CO;  Comment (HP-GL/2 command)
 * IN;  Initialize
 * IP;  Input P1 & P2
 * SC;  Scale
 * SP;  Select pen
 * PW;  Pen width (HP-GL/2 command)
 * PU;  Pen UP
 * PD;  Pen DOWN
 * PA;  Plot Absolute
 * 
 * Example HP-GL output of this sketch (with a few HP-GL/2 commands):
 * CO "Analog X-Y Pen Plotter to HP-GL Translator";
 * CO "Revision K";
 * CO "dcXOffset=20 dcYOffset=22";
 * IN;
 * IP0,0,4000,4000;
 * SC45,1003,53,957,0;
 * PW0.5,1;
 * SP1;
 * PU77,984;PD;  // move pen to initial point and put down pen
 * PA77,984;     // plot point
 * ...           // data points up to every 1.6 milliseconds only while pen is down
 * PU;
 * SP;
 * IN;
 * 
 * This sketch is for Arduino UNO with Proto Shield. The Proto Shield contains signal 
 * conditioning circuitry comprised of;
 * 1. Two (2) precision, unity-gain differential amplifiers (wired as summing amps) 
 *    to translate the ±2.5V analog X and Y voltages into 0 to 5V for the Arduino analog 
 *    pins.
 * 2. A 'charge pump' voltage converter to convert Vin to ±Vin to power the amplifiers.
 * 3. A precision 2.50V voltage reference to the amplifiers for the voltage translation.
 * 4. A precision 5.00V voltage reference to the Arduino AREF pin.
 * 5. A voltage divider network so the Vin can be reduced for the Arduino analog pin. 
 *    A schottky diode protects the pin if a greater than 9V power supply is used.
 * 6. A bi-color red/green LED as a Go/No Go indicator that the 9V power supply
 *    is connected, or not. This sketch will stay in the setup() routine until a 
 *    6.5VDC, or greater, power supply is plugged in to the Arduino. These IC's 
 *    cannot run accurately on the USB ~5V power!
 * 7. A slide switch to start the data stream and stop it when the plot is completed.
 * 8. A pair of capacitors to filter the DAC dithering of the plotter X-Y voltage signals.
 * 9. A 9-pin cable with DB-9 male connector for connection to the oscilloscope plotter 
 *    port.
 *    
 * Circuit board found here, released under the 
 * 'Creative Commons Attribution-ShareAlike 4.0 International' license
 * (http://creativecommons.org/licenses/by-sa/4.0/):
 * https://aisler.net/PatrickFranken/signal-conditioning-board-rev-h/tektronix-2232-x-y-plotter-to-hpgl-translator
 * 
 * -- Normal Operation: --
 * Slide switch to OFF.
 * Connect Arduino via DB-9 connector to oscilloscope plotter port. 
 * Connect Arduino USB to PC. Arduino powers up, LED is red.
 * Connect power supply (6.5 to 20V) to Arduino. LED is green.
 * Launch listening application, like TeraTerm or PrintCapture. Ensure your application 
 *  is set to the Arduino COM port, 115200, 8, none, 1.
 * Slide switch to ON. (data stream commences)
 * Immediately press 'Start' key on scope's Plot screen.
 * When scope plot completes immediately slide switch to OFF. (data stream halts)
 * Reset Arduino in order to start the next plot.
 *
 * -- Auto Center and Auto Scaling Operation: --
 * Tie Pin 7 to ground.
 * Setup per Normal Operation.
 * Except, press the 'XY Setup' button on scope's Plot screen.
 * When scope draws the outer box slide switch to OFF.
 * Remove Pin 7 from ground. (values are now in EEPROM)
 * Reset Arduino in order to start a plot in Normal Operation.
 * 
 *****************
 * The MIT License
 *****************
 * https://opensource.org/licenses/MIT
 * Copyright (c) 2017 John Horstman
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************
 * 
 * Open source here:
 * https://github.com/jshorstman/Analog-XY-Plotter-Output-to-HPGL-Translator
 *****************************************************************************************/
//https://playground.arduino.cc/Code/Timer1/
#include <TimerOne.h>
#include <EEPROM.h>
//#define __dummy_data // uncomment to use dummy data for testing
//#define __serial_plotter // uncomment to calibrate with Arduino Serial Plotter tool

// The pins
#define PlotSetupPin 7 // Digital pin 7
#define TestPin 8      // Digital pin 8
#define SwitchPin 9    // Digital pin 9
#define LEDGreenPin 10 // Digital pin 10
#define LEDRedPin 11   // Digital pin 11
#define PenInputPin 12 // Digital pin 12
#define IdleLED LED_BUILTIN // Digital pin 13
#define X_voltage 0    // Analog pin A0
#define Y_voltage 1    // Analog pin A1
#define VinVoltage 2   // Analog pin A2
// The logic levels
#define DOWN LOW
#define UP HIGH
#define NO_GO LOW
#define GO HIGH
#define OFF LOW
#define ON HIGH
// The inlines
#define IdleLED_(x) digitalWrite(IdleLED, x)
// The numbers
#define numReadings 8              // Must be a power of 2 so we can right shift later
#define divisor 3                  // Must be the integer exponent on 2 to get numReadings
#define eeSCAddressBase 0          // EEPROM base address for storing SC command values
#define eeCEAddressBase 8          // EEPROM base address for storing DC offset values
int PenState = UP;                 // Default pen up
int OldPenState = PenState;        //
int Xreadings[numReadings];        // Arrays for computing moving average for smoothing
int Yreadings[numReadings];        //
int Xtotal = numReadings * 511;    // Arrays will be preloaded
int Ytotal = Xtotal;               //
int Xcoor = Xtotal;                //
int Ycoor = Ytotal;                //
int oldXcoor = Xcoor;              //
int oldYcoor = Ycoor;              //
int XCenter = 0;                   // DC offset values during XY Setup
int YCenter = 0;                   //
int MinXExtent = 1024;             // Extent values are computed during XY Setup, initialized here
int MaxXExtent = 0;                //
int MinYExtent = 1024;             //
int MaxYExtent = 0;                //
int eeSCAddress = eeSCAddressBase; // EEPROM address for four SC command values
int eeCEAddress = eeCEAddressBase; // EEPROM address for two dcOffset values
int readIndex = 0;                 // Array index
int dcXOffset = 0;                 // Compensate for any DC offset. Calculated and stored during XY Setup
int dcYOffset = 0;                 //
// The flags
boolean changeFlag = false;            // Set when new XY coordinate differs from previous
boolean PenStateChangeToDown = false;  // Pen state change flag when pen first transitions to down from up
boolean PlotSetupMode = false;         // Flag true if in XY Setup (pin 7 low)
// The strings
String ADCXStr; // ADC X counts string
String ADCYStr; // ADC Y counts string
String CmdStr;  // HPGL command string

#ifdef __dummy_data
const float slew = 614.0; // max slew rate is 3.0 volts/second = 614 counts/second (@5 volts = 1023 counts)
const float Freq = (1.0/PI) * (slew / 1023.0);  // This frequency sinusoidal will have that slew rate (at the zero crossing)
const float TWO_PI_F = TWO_PI * Freq;
#endif

/*
 * The Setup *****************************************************************************|
 */ 
void setup() {
  // Set I/O pins
  pinMode(TestPin, OUTPUT); // timing test pin
  pinMode(IdleLED, OUTPUT);
  pinMode(LEDRedPin, OUTPUT);
  pinMode(LEDGreenPin, OUTPUT);
  pinMode(PlotSetupPin, INPUT_PULLUP);  // used during XYSetup to record XY extent values to EEPROM 
  pinMode(SwitchPin, INPUT_PULLUP);
  pinMode(PenInputPin, INPUT_PULLUP); // Pen signal is from Normally Open (N.O.) relay contacts

  /* For analog input pins, the digital input buffer should be disabled at all times.
   * Atmel-42735B-ATmega328/P_Datasheet_Complete-11/2016 p66
   * Data Input Disable Register 0 (DIDR0)
   * ADC2D, ADC1D and ADC0D set to 1.  p326 */
  bitSet(DIDR0, ADC0D);
  bitSet(DIDR0, ADC1D);
  bitSet(DIDR0, ADC2D);
  
  analogReference(DEFAULT);

  // Set LEDs
  IdleLED_(ON); // Turn on 'idle' LED
  Go_NoGo(NO_GO); // set NO GO LED
  
  // Initialize serial communication:
  /* High baud rate because Serial.println() is in an ISR */
  Serial.begin(115200);
  
  // Initialize Loop timer
  /* Full scale plot swing is 1023 counts in 1.667 seconds 
   * (5 volt full swing / 3.0 volts/second).
   * So, no more than one plot command every 1.630ms is necessary (1.667/1023).
   * ~614 samples/second. */
  Timer1.initialize(1630); // set to 1630 microseconds

  // Delay here for battery voltage to drop when LED first turns on
  delay(200);

  waitForNominalVoltage(); // Wait here until user plugs in a 6.5VDC, or greater, power supply
  // Vin OK, continue

  //Wait here for switch to be switched on (ground the pin)
  while (digitalRead(SwitchPin) == HIGH) {
    waitForNominalVoltage();  // Keep checking voltage until switch is thrown    
    delay(100);
  } 
  // User is ready for data stream and voltage is good, let 'er rip!

  analogReference(EXTERNAL); // External precision 5.00V reference

  // Preset the averaging arrays
  for (int i = 0; i < numReadings; i++) {
    Xreadings[i] = 511;
    Yreadings[i] = 511;
  }
  
  // Decide here if in XY Setup mode
  if (digitalRead(PlotSetupPin) == LOW) {
	  PlotSetupMode = true;
  }
  
  // Do auto center here only during XY Setup
  if (PlotSetupMode) {
	  // take average of numReadings readings
    for (int i = 0; i < numReadings; i++) {
      XCenter += analogRead(X_voltage);
      YCenter += analogRead(Y_voltage);
    }
    XCenter = XCenter >> divisor; // Divide by numReadings
    YCenter = YCenter >> divisor;
    dcXOffset = XCenter - 511;  // offset computed based on how far off center the scope is
    dcYOffset = YCenter - 511;  // offsets can be negative. Therefore, signed integers
	  // Burn offset values to EEPROM
    /* Offsets must be computed prior to computing min and max extents */
    EEPROM.put(eeCEAddress, dcXOffset);
    EEPROM.put(eeCEAddress += sizeof(int), dcYOffset);
    eeCEAddress = eeCEAddressBase; // reset memory pointer
  } // end auto center
  
  // Get offset and extent values from EEPROM
  EEPROM.get(eeCEAddress, dcXOffset);
  EEPROM.get(eeCEAddress += sizeof(int), dcYOffset);
  EEPROM.get(eeSCAddress, MinXExtent);
  EEPROM.get(eeSCAddress += sizeof(int), MaxXExtent);
  EEPROM.get(eeSCAddress += sizeof(int), MinYExtent);
  EEPROM.get(eeSCAddress += sizeof(int), MaxYExtent);
  // ensure values from EEPROM are not garbage
  if ((dcXOffset >= -511 && dcXOffset <= 512) && (dcYOffset >= -511 && dcYOffset < 512)) {
	  // do nothing, values are in range
  } else { // they are garbage, set defaults
	  dcXOffset = 0;
	  dcYOffset = 0;
  }
  if ((MinXExtent >= 0 && MinXExtent < 1024) && (MaxXExtent > 0 && MaxXExtent < 1024) && (MinYExtent >= 0 && MinYExtent < 1024) && (MaxYExtent > 0 && MaxYExtent < 1024)) {
    // do nothing, values are in range
  } else { // they are garbage, set defaults
    MinXExtent = 0;
    MaxXExtent = 1023;
    MinYExtent = 0;
    MaxYExtent = 1023;
  } 
  // end getting values
  
  IdleLED_(OFF); // turn off 'idle' LED, transmission will commence (TX LED will light)

  // HP-GL plot header commands
  Serial.println("CO \"Analog X-Y Pen Plotter to HP-GL Translator\";"); // (COmment is an HP-GL/2 command)
  Serial.println("CO \"Revision " + REVISION + "\";");
  Serial.println("CO \"dcXOffset=" + String(dcXOffset) + " dcYOffset=" + String(dcYOffset) + "\";"); // output for convenience
  Serial.println("IN;"); // INitialize
  Serial.println("IP0,0,4000,4000;"); // Input scaling Points P1 & P2; 4000pu=10cm
  Serial.println("SC" + String(MinXExtent) + "," + String(MaxXExtent) + "," + String(MinYExtent) + "," + String(MaxYExtent) + ",0;");  // SCale, anisotropic
  Serial.println("PW0.5,1;"); // Pen 1 Width 0.5mm (HP-GL/2 command)
  Serial.println("SP1;"); // Select Pen 1, black

  // Initialize values if in XY Setup. New values will be computed in Process().
  if(PlotSetupMode) {
    MinXExtent = 1024;
    MaxXExtent = 0;
    MinYExtent = 1024;
    MaxYExtent = 0;
  }
  
  Timer1.attachInterrupt(Process); // ISR
} // end setup

/*
 * The Loop ******************************************************************************|
 */
void loop() {  
  /* If switch went HIGH, output final commands, stop data stream, the plot is 
   * completed, wait here until RESET */
  if (digitalRead(SwitchPin) == HIGH) {
    Timer1.detachInterrupt(); //stop the timer interrupt
    Serial.println("PU;");
    Serial.println("SP;");
    Serial.println("IN;");
	
	  // Burn scaling values to EEPROM during XY Setup
    /* MaxXExtent-MinXExtent represents the 10cm X-axis span of the scope plot.
     * So, scaling that extent to the 10cm (4000pu) Input Scaling results in a 1:1 scale plot */
    if (PlotSetupMode) {
	  eeSCAddress = eeSCAddressBase; // reset memory pointer
	  EEPROM.put(eeSCAddress, MinXExtent);
	  EEPROM.put(eeSCAddress += sizeof(int), MaxXExtent);
	  EEPROM.put(eeSCAddress += sizeof(int), MinYExtent);
	  EEPROM.put(eeSCAddress += sizeof(int), MaxYExtent);
    }

    while (true) {  // wait here for RESET
      // flash 'idle' LED to indicate need for reset
      IdleLED_(ON);
      delay(200);
      IdleLED_(OFF);
      delay(300);
    }
  }
} // end loop

/*
 * The Process ***************************************************************************|
 */ 
// This ISR takes up to 760 µs to complete with two analogRead's and one Serial.println()
void Process(){ 
  // rising edge of timing test pulse.
  PORTB |= B1; // digitalWrite(TestPin, HIGH)
  
#ifdef __dummy_data
  // dummy data for now, plot a circle 
  float t = (float)millis() / 1000.0;
  float arg = TWO_PI_F * t;
  Xcoor = int(511.5*cos(arg)+511.5);
  Ycoor = int(511.5*sin(arg)+511.5);
  (random(1, 1001) == 1L) ? (PenState = UP) : (PenState = DOWN); // Pen up, sometimes
#else
  /* Read X, Y and Pen signals as close together as possible and compute a moving 
   * average for smoothing */
  Xtotal -= Xreadings[readIndex]; // Subtract an old value from the total
  Ytotal -= Yreadings[readIndex]; //
  
  PenState = digitalRead(PenInputPin); // Read pen input; returns HIGH or LOW
  Xreadings[readIndex] = analogRead(X_voltage); // no delay between readings
  Yreadings[readIndex] = analogRead(Y_voltage) - dcYOffset;
  Xreadings[readIndex] -= dcXOffset; // compensate for dc offset here
  
  Xtotal += Xreadings[readIndex];   // Add the new value to the total
  Ytotal += Yreadings[readIndex++]; // Increment readIndex here
  readIndex %= numReadings;  // Wrap around readIndex here

  Xcoor = Xtotal >> divisor; // Divide by numReadings
  Ycoor = Ytotal >> divisor;
  // End read X, Y and pen
#endif

  /* If pen has changed state then set state change flag
   * Pen UP signal is HIGH (relay open, internal pull up resistor), pen DOWN is LOW 
   * (relay closed to ground) */   
  if (PenState != OldPenState) {
    (PenState == DOWN) ? (PenStateChangeToDown = true) : (PenStateChangeToDown = false);    
    OldPenState = PenState; // Remember pen state
  }

  /* Determine if the new Xtotal and Ytotal are different from the old values. If so, 
   * output the new plot point (set changeFlag). */
  if (Xcoor != oldXcoor) {
    ADCXStr = Xcoor;
    oldXcoor = Xcoor;
    changeFlag = true;
	  if (PlotSetupMode) {
	    if (Xcoor > MaxXExtent) {MaxXExtent = Xcoor;} // used only for XY Setup
	    if (Xcoor < MinXExtent) {MinXExtent = Xcoor;}
	  }
  } 
  if (Ycoor != oldYcoor) {
    ADCYStr = Ycoor;
    oldYcoor = Ycoor;
    changeFlag = true;
    if (PlotSetupMode) {
	    if (Ycoor > MaxYExtent) {MaxYExtent = Ycoor;}
	    if (Ycoor < MinYExtent) {MinYExtent = Ycoor;}
	  }
  } 
  
  // 'Plot Absolute' Ex: PA1023,250;
  // Or, 'Pen Up' move and 'Pen Down'  Ex: PU1023,250;PD;
  if (changeFlag) {
#ifdef __serial_plotter
    CmdStr = "1023 0 " + ADCXStr + " " + ADCYStr; // 1023 0 prevents autoscaling 
#else
    /* When the pen transitions to down from up, output the PU command to move the pen
     * to the new location, then output the PD command. */
    if (PenStateChangeToDown)  {  // pen has just gone down 
      CmdStr = "PU" + ADCXStr + "," + ADCYStr + ";" + "PD;";
      PenStateChangeToDown = false; // do this only once for every pen down transition
    } else {  // else it's a bona fide plot command
	    CmdStr = "PA" + ADCXStr + "," + ADCYStr + ";";
  	}
#endif
    if (PenState == DOWN) {  // plot only when pen down
      // We can afford only one Serial.println so as not to violate our ISR timing.
      Serial.println(CmdStr);
    }
    changeFlag = false;  // don't plot again until the coordinate(s) change
  }
  
  // falling edge of timing test pulse.
  PORTB ^= B1; // digitalWrite(TestPin, LOW);
} // end Process

/*
 * The Functions *************************************************************************|
 */ 
void waitForNominalVoltage(void) {
 /* Check for external power, stay in setup() until >6.5V supply connected
  * 6.5V external supply is 6.3V Vin (due to diode drop on Arduino UNO board)
  * 6.3V is the absolute minimum supply for the op-amps, U2 & U3, and the 5V voltage reference, U5
  * 6.3V through the resistor divider network, R1 & R2, is 3.3V
  * 3.3V, with 5V AREF, is 675 ADC counts (3.3 / 5 * 1023) */
  while (analogRead(VinVoltage) < 675) {
    Go_NoGo(NO_GO); // set NO GO LED
    delay(100);
  } // Wait here until user plugs in a 6.5VDC, or greater, power supply
  Go_NoGo(GO); // set GO LED
} // end waitForNominalVoltage

void Go_NoGo(int state) {
  if (state == GO) {
    digitalWrite(LEDGreenPin, HIGH); // Turn on green LED
    digitalWrite(LEDRedPin, LOW);
  } else {
    digitalWrite(LEDRedPin, HIGH); // Turn on red LED
    digitalWrite(LEDGreenPin, LOW);
  }
} // end Go_NoGo
