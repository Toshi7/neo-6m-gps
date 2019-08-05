#include <TinyGPS++.h> // Download and install the library at http://arduiniana.org/libraries/tinygpsplus/ 
#include <HardwareSerial.h> // should already be installed with Arduino IDE
#include "EEPROM.h" // should already be installed with Arduino IDE

/*
   with EEPROM, data can be cached,
   a small store, so to speak. Data is also after one
   Reset even where they were written. To enter the
   To be able to write memory must be a memory size
   be specified. This is specified in bytes.
   We use these to save the last GPS fix
   so that it is read in the first start as the first. In order to
   prevents, for example, when recording our position
   First jumps to the 0.0 point, somewhere in the Atlantic
   Ocean lies.

   The memory of ESP32 can be between 4 and maximum 1984 bytes
*/
#define EEPROM_SIZE 128

/*
   Next, the program object for the GPS is created, we call
   the variable that points to it simply "gps"
*/

TinyGPSPlus gps;

/*
   Now we have to create a serial connection to the GPS module
   ESP32 supports up to 3 hardware serial connections. That's why
   we also do not need to use the software serial library.
   The number in parenthesis is the uart_nr. This will be the three possible
   Distinguished connections. For ESP32 this value can be 0, 1 or 2
*/
HardwareSerial SerialGPS(1);

/*
   Next, we create a template object where we get all the data
   in a variable read by the GPS module
   Afterwards we create a new variable, "gpsState"
*/
struct GpsDataState_t {
  double originLat = 0;
  double originLon = 0;
  double originAlt = 0;
  double distMax = 0;
  double dist = 0;
  double altMax = -999999;
  double altMin = 999999;
  double spdMax = 0;
  double prevDist = 0;
};
GpsDataState_t gpsState = {};

/*
   The following constant defines the output speed
   in the serial monitor. This is specified in milliseconds.
   Including the associated variables to achieve this restriction
*/
#define TASK_SERIAL_RATE 1000 // ms
uint32_t nextSerialTaskTs = 0;
uint32_t nextOledTaskTs = 0;

/*
   Helper functions to simplify reading and writing memory
*/
template <class T> int EEPROM_writeAnything(int ee, const T& value)
{
  const byte* p = (const byte*)(const void*)&value;
  int i;
  for (i = 0; i < sizeof(value); i++)
    EEPROM.write(ee++, *p++);
  return i;
}

template <class T> int EEPROM_readAnything(int ee, T& value)
{
  byte* p = (byte*)(void*)&value;
  int i;
  for (i = 0; i < sizeof(value); i++)
    *p++ = EEPROM.read(ee++);
 
}
/*
   Setup Funktion.
   Diese wird einmal beim Systemstart ausgeführt
   Enthält alle Initialisierungen
*/
void setup() {

  // Serial is the output in the Serial Monitor
  Serial.begin(115200);

  /*
      The connection with the GPS module. We
      void begin (unsigned long baud, uint32_t config = SERIAL_8N1, int8_t rxPin = -1, int8_t txPin = -1, bool invert = false, unsigned long timeout_ms = 20000UL);
      baud: baudrate according to the GPS module specification, in this case 9600
      config: default value
      rxPin: a RX pin eg 16
      txPin: a RX pin eg 17
  */
  SerialGPS.begin(9600, SERIAL_8N1, 16, 17);

  /*
     Initialize EEPROM memory if it does not exist
  */
  while (!EEPROM.begin(EEPROM_SIZE)) {
    true;
  }

  /*
     The three axis directions x, y, z from the memory
     read and deposit
  */
  long readValue;
  EEPROM_readAnything(0, readValue);
  gpsState.originLat = (double)readValue / 1000000;

  EEPROM_readAnything(4, readValue);
  gpsState.originLon = (double)readValue / 1000000;

  EEPROM_readAnything(8, readValue);
  gpsState.originAlt = (double)readValue / 1000000;

}



void loop() {

  static int p0 = 0;

  // GPS Koordinaten von Modul lesen
  gpsState.originLat = gps.location.lat();
  gpsState.originLon = gps.location.lng();
  gpsState.originAlt = gps.altitude.meters();

  // Aktuelle Position in nichtflüchtigen ESP32-Speicher schreiben
  long writeValue;
  writeValue = gpsState.originLat * 1000000;
  EEPROM_writeAnything(0, writeValue);
  writeValue = gpsState.originLon * 1000000;
  EEPROM_writeAnything(4, writeValue);
  writeValue = gpsState.originAlt * 1000000;
  EEPROM_writeAnything(8, writeValue);
  EEPROM.commit(); // erst mit commit() werden die Daten geschrieben

  gpsState.distMax = 0;
  gpsState.altMax = -999999;
  gpsState.spdMax = 0;
  gpsState.altMin = 999999;

  /*
   * Raw data from serial link to GPS module
   * read. The data is processed using TinyGPS ++
   * The data becomes conscious only after the assignment of the variables
   * read so that we can simplify in the following 
   * Can do calculations.
   */
  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
  }

  /*
   * Various calculations of maximum and minimum values ​​and distance traveled
   * These are only made if at least one fix with 4 satellites exists
   * is, at most, the accuracy would not be given and it would be wrong
   * Values ​​are calculated.
   */
  if (gps.satellites.value() > 4) {
    gpsState.dist = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), gpsState.originLat, gpsState.originLon);

    if (gpsState.dist > gpsState.distMax && abs(gpsState.prevDist - gpsState.dist) < 50) {
      gpsState.distMax = gpsState.dist;
    }
    gpsState.prevDist = gpsState.dist;

    if (gps.altitude.meters() > gpsState.altMax) {
      gpsState.altMax = gps.altitude.meters();
    }

    if (gps.speed.kmph() > gpsState.spdMax) {
      gpsState.spdMax = gps.speed.kmph();
    }

    if (gps.altitude.meters() < gpsState.altMin) {
      gpsState.altMin = gps.altitude.meters();
    }
  }

   /*
     So that not too much data is output in the Serial Monitor,
     let's limit the output to the number of milliseconds
     which we saved in the constant "TASK_SERIAL_RATE"
  */
  if (nextSerialTaskTs < millis()) {
    Serial.print("LAT=");  Serial.println(gps.location.lat(), 6);
    Serial.print("LONG="); Serial.println(gps.location.lng(), 6);
    Serial.print("ALT=");  Serial.println(gps.altitude.meters());
    Serial.print("Sats=");  Serial.println(gps.satellites.value());
    Serial.print("DST: ");
    Serial.println(gpsState.dist, 1);
    nextSerialTaskTs = millis() + TASK_SERIAL_RATE;
  }


}
