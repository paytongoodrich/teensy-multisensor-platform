// A data logger for the AIFS tomato project.  
//This early-draft version reads a 3-axis accelerometer and records the data to SD and the serial port. 
//Future versions will include readings of a capacitive sensor - see File>Examples>CapacitiveSensor.
//P. Goodrich

// Libraries that we use
#include <SPI.h> //for SD card usage
#include <SD_t3.h> //for SD card usage
#include <SD.h> //for SD card usage
#include <Wire.h> //for I2C connection
#include <TimeLib.h> //for clock
#include <Adafruit_LIS3DH.h>  //for the 3-axis accelerometer
#include <Adafruit_Sensor.h>  //for the 3-axis accelerometer
#include <InternalTemperature.h> //for onboard temperature sensor
#include <CapacitiveSensor.h> //for capacitive water-level sensor

// User-input variables: These are the dials that we can tweak
#define LOG_INTERVAL  500 // time in milliseconds between sensor readings
#define SYNC_INTERVAL 25000 // Only write data to the card at long intervals for power/data savings
#define ECHO_TO_SERIAL   1 // 1 to echo data to serial port, 0 if not
#define WAIT_TO_START    0 // If set to anything but 0, will require a keyboard tap before the program begins
int32_t syncTime = 0; // time of last sync()

// I2C connection to 3-axis accelerometer
Adafruit_LIS3DH lis = Adafruit_LIS3DH();

//onboard LED
int led = 13;

//Capacitive Sensor
CapacitiveSensor cs_4_2 = CapacitiveSensor(4,2); // 2 megohm resistor between pins 4 & 2, pin 2 is sensor pin, add wire, foil

 
// the logging file
File logfile;




//Initialization
void setup(void) 
{
  setSyncProvider(getTeensy3Time);
  Serial.begin(9600); //Initialize serial port at 9600 baud
  Serial.println();

  // If wait_to_start is anything but 0, will require the user to type something to continue onwards.
  #if WAIT_TO_START
    Serial.println("Type any character to start");
    while (!Serial.available());
  #endif //WAIT_TO_START

  // Sync onboard RTC to pc time
  while (!Serial);  // Wait for Arduino Serial Monitor to open
  delay(100);
  if (timeStatus()!= timeSet) {
    Serial.println("Unable to sync with the RTC");
  } else {
    Serial.println("RTC has set the system time");
  }
  
  // initialize the SD card
  Serial.println("Initializing SD card...");
  
  // see if the card is present and can be initialized:
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("Card failed, or not present.  Check to see if it is inserted, the wiring is correct, and chipSelect matches the shield.");
    digitalWrite(led, HIGH);
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
  
////   create a new file. Automatically counts from LOGGER00 to LOGGER99
//  char filename[] = "LOGGER00.csv";
//  for (uint8_t i = 0; i < 100; i++) {
//    filename[6] = i/10 + '0';
//    filename[7] = i%10 + '0';
//    if (! SD.exists(filename)) {
//      // only open a new file if it doesn't exist
//      File logfile = SD.open(filename, FILE_WRITE);
//      break;  // leave the loop!
//    }
//  }
//  
//  if (! logfile) {
//    Serial.print("couldnt create file");
//  }
//  
  File logfile = SD.open("datalog.csv", FILE_WRITE);
  Serial.println("Logging to datalog");


  
  if (! lis.begin(0x18)) // change this to 0x18 or 0x19 for alternative i2c address
  {   
    Serial.println("LIS3DH could not be found.");
    while (1) yield();
  }
  Serial.println("LIS3DH found!");

  Serial.print("Range = "); Serial.print(2 << lis.getRange());
  Serial.println("G");

  lis.setDataRate(LIS3DH_DATARATE_50_HZ);
  Serial.print("Data rate set to: ");
  switch (lis.getDataRate()) 
  {
    case LIS3DH_DATARATE_1_HZ: Serial.println("1 Hz"); break;
    case LIS3DH_DATARATE_10_HZ: Serial.println("10 Hz"); break;
    case LIS3DH_DATARATE_25_HZ: Serial.println("25 Hz"); break;
    case LIS3DH_DATARATE_50_HZ: Serial.println("50 Hz"); break;
    case LIS3DH_DATARATE_100_HZ: Serial.println("100 Hz"); break;
    case LIS3DH_DATARATE_200_HZ: Serial.println("200 Hz"); break;
    case LIS3DH_DATARATE_400_HZ: Serial.println("400 Hz"); break;

    case LIS3DH_DATARATE_POWERDOWN: Serial.println("Powered Down"); break;
    case LIS3DH_DATARATE_LOWPOWER_5KHZ: Serial.println("5 Khz Low Power"); break;
    case LIS3DH_DATARATE_LOWPOWER_1K6HZ: Serial.println("16 Khz Low Power"); break;
  }

  logfile.println("time, clock, date, x_acc, y_acc, z_acc, temperature, capacitance");
  logfile.close();    
#if ECHO_TO_SERIAL
  Serial.println("time, clock, date, x_acc, y_acc, z_acc, temperature, capacitance");
#endif //ECHO_TO_SERIAL

cs_4_2.set_CS_AutocaL_Millis(0xFFFFFFFF); // turn off autocalibrate on channel 1

  
}




//Main Loop
void loop() {

  // delay for the amount of time we want between readings
  delay((LOG_INTERVAL - 1) - (millis() % LOG_INTERVAL));

  // log milliseconds since starting
  int32_t m = millis();   
  #if ECHO_TO_SERIAL
    Serial.print(m);         // milliseconds since start
    Serial.print(", ");  
  #endif

  //Read the sensors
  lis.read();      // get X Y and Z data at once
  sensors_event_t event;
  lis.getEvent(&event);
  long cap1 = cs_4_2.capacitiveSensor(30); //Read cap sensor.Number in () is the number of samples read (increase to increase sensitivity but slower)
  long teensyTemp = InternalTemperature.readTemperatureC();
  
  //Record Data to SD Card
  File logfile = SD.open("datalog.csv", FILE_WRITE);
  logfile.print(m);           // milliseconds since start
  logfile.print(", "); 
  logfile.print(hour());
  logfileprintDigits(minute());
  logfileprintDigits(second());
  logfile.print(", ");
  logfile.print(day());
  logfile.print("/");
  logfile.print(month());
  logfile.print("/");
  logfile.print(year());
  logfile.print(", ");
  logfile.print(event.acceleration.x); 
  logfile.print(", ");
  logfile.print(event.acceleration.y);
  logfile.print(", "); 
  logfile.print(event.acceleration.z); 
  logfile.print(", "); 
  logfile.print(teensyTemp);
  logfile.print(", "); 
  logfile.print(cap1);
  logfile.println();
  logfile.close();
  
  #if ECHO_TO_SERIAL
  Serial.print(hour());
  serialprintDigits(minute());
  serialprintDigits(second());
  Serial.print(", ");
  Serial.print(day());
  Serial.print("/");
  Serial.print(month());
  Serial.print("/");
  Serial.print(year());
  Serial.print(", ");
  Serial.print(event.acceleration.x); 
  Serial.print(", ");
  Serial.print(event.acceleration.y);
  Serial.print(", "); 
  Serial.print(event.acceleration.z);
  Serial.print(", "); 
  Serial.print(teensyTemp);
  Serial.print(", "); 
  Serial.print(cap1);   
  Serial.println(); 
  #endif //ECHO_TO_SERIAL

//  // Now we write data to disk! Don't sync too often because it takes a lot of data and power
//  if ((millis() - syncTime) < SYNC_INTERVAL) return;
//  syncTime = millis();
//  
//  // turn on LED to show we are syncing data
//  digitalWrite(led, HIGH);
//  logfile.flush();
//  digitalWrite(led, LOW);
//    
//  }
}

// Subfunction: Serial Digits
void serialprintDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

// Subfunction: Logfile Digits
void logfileprintDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  logfile.print(":");
  if(digits < 10)
    logfile.print('0');
  logfile.print(digits);
}

// Set PC Time
#define TIME_HEADER  "T"   // Header tag for serial time sync message
unsigned long processSyncMessage() {
  unsigned long pctime = 0L;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013 

  if(Serial.find(TIME_HEADER)) {
     pctime = Serial.parseInt();
     return pctime;
     if( pctime < DEFAULT_TIME) { // check the value is a valid time (greater than Jan 1 2013)
       pctime = 0L; // return 0 to indicate that the time is not valid
     }
  }
  return pctime;
}

// Time sync to Jan 1, 1970
time_t getTeensy3Time()
  {
    return Teensy3Clock.get();
  }  
