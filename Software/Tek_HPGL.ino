/*
 ****************************************************************************************|
 * Tektronix 2232 Digital Storage Oscilloscope analog X-Y pen ploter to HP-GL 
 * translator. (Hewlett-Packard Graphics Language http://en.wikipedia.org/wiki/HPGL)
 * 
 * John Horstman
 * 3/19/2017 Rev H
 * 
 * This sketch continuously reads the X and Y analog pen plotter voltages, and the 
 * pen up/down signal, and packages the ADC values in successive HP-GL commands. The 
 * commands can be serially output to a file, via TeraTerm, or to a plotter application 
 * like PrintCapture at http://www.printcapture.com/.
 *  
 * The HP-GL commands used in this sketch are:
 * IN;  Initialize
 * PS:  Plot size
 * IP;  Input P1 & P2
 * SC;  Scale
 * SP;  Select pen
 * PU;  Pen UP
 * PD;  Pen DOWN
 * PA;  Plot Absolute
 * 
 * Example HPGL output of this sketch:
 * IN;
 * PS8900,8900;
 * IP0,0,8900,8900;
 * SC0,250,0,1023,1;
 * PU;
 * SP1;
 * PA77,984;
 * ...          // data points every 1.6 milliseconds
 * PD;
 * PA53,92;
 * ...
 * PU;
 * SP;
 * IN;
 * 
 * This sketch is for Arduino UNO with Proto Shield. The Proto Shield contains signal 
 * conditioning circuitry comprised of;
 * 1. Two (2) precision, unity-gain differential amplifiers (wired as summing amps) 
 *    to translate the ±2.5V analog X and Y voltages into 0 to 5V for the Arduino analog 
 *    pins.
 * 2. A 'charge pump' voltage converter to convert 9VDC to ±9VDC to power the amplifiers.
 * 3. A precision 2.50V voltage reference to the amplifiers for the voltage translation.
 * 4. A precision 5.00V voltage reference to the Arduino AREF pin.
 * 5. A voltage divider network so the 9V Vin can be reduced to 5V for the Arduino
 *    analog pin. A schottky diode protects the pin if a higher voltage power supply
 *    is used.
 * 6. A bi-color red/green LED as a Go/No Go indicator that the 9V power supply
 *    is connected, or not. This sketch will stay in the setup() routine until a 
 *    9VDC, or greater, power supply is plugged in to the Arduino. These IC's 
 *    cannot run accurately on the USB ~5V power!
 * 7. A slide switch to start the data stream and stop it when the plot is completed.
 * 8. A pair of capacitors to filter the DAC dithering of the plotter X-Y voltage signals.
 * 9. A 9-pin cable with DB-9 male connector for connection to the oscilloscope plotter 
 *    port.
 * 
 * Operation:
 * Slide switch to OFF.
 * Connect Arduino via DB-9 connector to oscilloscope plotter port. 
 * Connect Arduino USB to PC. Arduino powers up, LED is red.
 * Connect 9VDC power to Arduino. LED is green.
 * Launch listening application, like TeraTerm or PrintCapture. Ensure your application 
 *  is set to the Arduino COM port 6, 115200, 8, none, 1.
 * Slide switch to ON. (data stream commences)
 * Immediately press PLOT key on scope.
 * When scope plot completes immediately slide switch to OFF. (data stream halts)
 * Reset Arduino in order to start the next plot.
 ****************************************************************************************|
*/
//#define __serial_plotter // uncomment to calibrate with Arduino Serial Plotter tool
//#define __dummy_data // uncomment to use dummy data for testing

// The pins
#define TestPin 8      // Digital pin 8
#define SwitchPin 9    // Digital pin 9
#define LEDGreenPin 10 // Digital pin 10
#define LEDRedPin 11   // Digital pin 11
#define PenInputPin 12 // Digital pin 12
#define IdleLED 13     // Digital pin 13
#define X_voltage 0    // Analog pin A0
#define Y_voltage 1    // Analog pin A1
#define VinVoltage 2   // Analog pin A2
// The numbers
unsigned long LoopTime; // Marker for loop() delay timer
unsigned int PenState = HIGH;        // Default pen up
unsigned int OldPenState = PenState; //
const unsigned int numReadings = 8;       // Must be a power of two so we can right shift later
unsigned int Xreadings[numReadings];      // Arrays for computing moving average for smoothing
unsigned int Yreadings[numReadings];      //
unsigned int Xtotal = numReadings * 511;  // Arrays will be preloaded
unsigned int Ytotal = Xtotal;             //
unsigned int readIndex = 0;               //
const unsigned int dcXOffset = 0; //21;  // Compensate for any DC offset. Adjust this to get approximately 511,511.
const unsigned int dcYOffset = 0; //23;  //
// The strings
String ADCXStr; // ADC X counts string
String ADCYStr; // ADC Y counts string
String CmdStr;  // HPGL command string

#ifdef __dummy_data
const float slew = 614.0; // max slew rate is 3.0 volts/second = 614 counts/second (@5 volts = 1023 counts)
const float Freq = (1.0/PI) * (slew / 1023.0);  // This frequency sinusoidal will have that slew rate (at the zero crossing)
const float TWO_PI_F = 2.0 * PI * Freq;
#endif

/*
 * The Setup ****************************************************************************|
 */ 
void setup() {
  // Set I/O pins
  pinMode(TestPin, OUTPUT); // timing test pin
  pinMode(IdleLED, OUTPUT);
  pinMode(LEDRedPin, OUTPUT);
  pinMode(LEDGreenPin, OUTPUT);
  pinMode(SwitchPin, INPUT_PULLUP);
  pinMode(PenInputPin, INPUT_PULLUP); // Pen signal is from Normally Open (N.O.) relay contacts
  
  analogReference(DEFAULT);

  // Set LEDs
  digitalWrite(IdleLED, HIGH); // Turn on 'idle' LED
  digitalWrite(LEDRedPin, LOW); // Turn red/green LED off
  digitalWrite(LEDGreenPin, LOW);
  
  // initialize serial communication:
  Serial.begin(115200);

  // Check for 9V Vin external power, stay in setup() until 9V supply connected
  /*
   * Why 900 counts? To gaurantee that a 7.5±5%VDC power pack will not trigger the 
   * 'Go' signal under maximal conditions:
   * 7.5v+5%; R1Ω-1%; R2Ω+1%; USB 5v-5% (AREF); +3 counts (2 LSB ADC error)
   */
  while (analogRead(VinVoltage) < 900) {
    digitalWrite(LEDRedPin, HIGH); // Turn LED red (no-go)
    digitalWrite(LEDGreenPin, LOW);
    delay(100);
  } // Wait here until user plugs in a 9VDC, or greater, power supply
   
  // Vin OK, continue
  digitalWrite(LEDRedPin, LOW); // Turn LED green (go)
  digitalWrite(LEDGreenPin, HIGH);

  //Wait here for switch to be switched on (ground the pin)
  while (digitalRead(SwitchPin) == HIGH) {
    delay(100);
  } 
  // User is ready for data stream, let 'er rip!

  analogReference(EXTERNAL); // External precision 5.00V reference

  // Preset the averaging arrays
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
  Xreadings[thisReading] = 511;
  Yreadings[thisReading] = 511;
  }
  
  digitalWrite(IdleLED, LOW); // turn off 'idle' LED, transmission will commence (TX LED will light)

  // HP-GL plot header commands
  Serial.println("IN;"); // Initialize
  Serial.println("PS8900,8900;"); // Plot size; A-size
  Serial.println("IP0,0,8900,8900;"); // Input Scaling Points P1 & P2; A-size drawing (8.5" x 11", less margins)
  Serial.println("SC0,1023,0,1023,1;"); // Scale; Isotropic
  //Serial.println("OH;"); // Output hard clip limits
  //Serial.println("OP;"); // Output P1 & P2
  Serial.println("PU;"); // Pun Up
  Serial.println("SP1;"); // Select Pen 1
} // end setup

/*
 * The Loop *****************************************************************************|
*/
void loop() {
  LoopTime = micros(); // mark loop start
  
  // Read X, Y and Pen signals as close together as possible and compute a moving average for smoothing
  Xtotal -= Xreadings[readIndex]; // Subtract an old value from the total
  Ytotal -= Yreadings[readIndex]; //
  
  PenState = digitalRead(PenInputPin); // Read pen input; returns HIGH or LOW
  Xreadings[readIndex] = analogRead(X_voltage) - dcXOffset; // compensate for dc offset
  delayMicroseconds(16); // Delay two ADC clock cycles (there is less 'jitter' on the Y readings than for a one ADC clock cycle dalay, don't ask me why)
  Yreadings[readIndex] = analogRead(Y_voltage) - dcYOffset;
  
  Xtotal += Xreadings[readIndex];   // Add the new value to the total
  Ytotal += Yreadings[readIndex++]; // Increment readIndex here
  readIndex %= numReadings;  // Wrap around readIndex here
  
  ADCXStr = Xtotal >> 3; // Total divided by numReadings (and convert to string)
  ADCYStr = Ytotal >> 3;
  //

  // If pen has changed state then output pen command
  // Pen UP signal is HIGH (relay open, interanl pull up resistor), pen DOWN is LOW (relay closed to ground)
  if (PenState != OldPenState) {
    (PenState == HIGH) ? (CmdStr = "PU;") : (CmdStr = "PD;");
    OldPenState = PenState; // Remember pen state
    Serial.println(CmdStr);
  }
  
  #ifdef __dummy_data
   // dummy data for now, plot a circle 
    float t = (float)millis() / 1000.0;
    float arg = TWO_PI_F * t;
    ADCXStr = int(511.5*cos(arg)+511.5);
    ADCYStr = int(511.5*sin(arg)+511.5);
  #endif

  #ifdef __serial_plotter
    int Pen;
    (PenState == HIGH) ? Pen = 10 : Pen = 0;  // View the Pen up/down signal
    CmdStr = "1023 0 " + ADCXStr + " " + ADCYStr + " " + Pen; // 1023 0 prevents autoscaling 
  #else
    // 'Plot Absolute' Ex: PA1023,250;
    CmdStr = "PA" + ADCXStr + "," + ADCYStr + ";";
  #endif
  
  Serial.println(CmdStr);

  // If switch went HIGH, output final commands, stop data stream, the plot is completed, wait here until RESET
  if (digitalRead(SwitchPin) == HIGH) {
    Serial.println("PU;");
    Serial.println("SP;");
    Serial.println("IN;");
    
    
    while (true) {  // wait here for RESET
      // flash 'idle' LED to indicate need for reset
      digitalWrite(IdleLED, HIGH);
      delay(200);
      digitalWrite(IdleLED, LOW);
      delay(300);
    }
  }
  /* Wait here for remainder of loop time.
   * Full scale plot swing is 1023 counts in 1.667 seconds (5 volt full swing / 3.0 volts/second).
   * So, no more than one plot command every 1.630ms is necessary (1.667/1023). ~614 samples/second.
   */
  while (micros() - LoopTime < 1630) {;} // wait here for remainder of loop time 

  // Short (0.250µs) output pulse to test loop timing.
  PORTB |= B1; PORTB ^= B1; // digitalWrite(TestPin, HIGH); digitalWrite(TestPin, LOW);
}  // end loop
