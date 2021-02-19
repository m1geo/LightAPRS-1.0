#include <LibAPRS.h>        //Modified version of https://github.com/markqvist/LibAPRS
#include <SoftwareSerial.h>
#include <TinyGPS++.h>      //https://github.com/mikalhart/TinyGPSPlus
//#include <LowPower.h>       //https://github.com/rocketscream/Low-Power
#include <Wire.h>
#include <Adafruit_BMP085.h>//https://github.com/adafruit/Adafruit-BMP085-Library
#include <avr/wdt.h>

#define RfPDPin     19
#define GpsVccPin   18
#define RfPwrHLPin  21
#define RfPttPin    20
#define BattPin     A2
#define PIN_DRA_RX  22
#define PIN_DRA_TX  23


#define ADC_REFERENCE REF_3V3
#define OPEN_SQUELCH true


#define GpsON         digitalWrite(GpsVccPin, LOW) //PNP
#define GpsOFF        digitalWrite(GpsVccPin, HIGH)
#define RfON          digitalWrite(RfPDPin, HIGH)
#define RfOFF         digitalWrite(RfPDPin, LOW)
#define RfPwrHigh     pinMode(RfPwrHLPin, INPUT)
#define RfPwrLow      pinMode(RfPwrHLPin, OUTPUT);digitalWrite(RfPwrHLPin, LOW)
#define RfPttON       digitalWrite(RfPttPin, HIGH) //NPN
#define RfPttOFF      digitalWrite(RfPttPin, LOW)
#define AprsPinInput  pinMode(12,INPUT);pinMode(13,INPUT);pinMode(14,INPUT);pinMode(15,INPUT)
#define AprsPinOutput pinMode(12,OUTPUT);pinMode(13,OUTPUT);pinMode(14,OUTPUT);pinMode(15,OUTPUT)


//****************************************************************************
char  CallSign[7] = "M1GEO"; //DO NOT FORGET TO CHANGE YOUR CALLSIGN
int   CallNumber = 9; //SSID http://www.aprs.org/aprs11/SSIDs.txt
char  Symbol = '>'; // '/>' for car, '/k' for truck, for more info : http://www.aprs.org/symbols/symbols-new.txt
bool alternateSymbolTable = false ; //false = '/' , true = '\'

char Frequency[9] = "144.8000"; //default frequency. 144.3900 for US, 144.8000 for Europe

char comment[50] = "GE07CAR"; // Max 43 char
char StatusMessage[50] = "George LightAPRS-1.0 http://m1geo.com";
//*****************************************************************************


unsigned int BeaconInterval = 90;      // seconds sleep for next beacon (TX).
unsigned int MinimumInterval = 5;      // maximum rate, packet every <this> seconds.
int          CourseChangeBeacon = 40;  // degrees before sending beacon
unsigned int TxDelay = 350;            // milliseeconds tx delay
boolean      aliveStatus = true;       // for tx status message on first wake-up just once.
//int          pathSize = 2;             // 2 for WIDE1-N,WIDE2-N ; 1 for WIDE2-N

static char telemetry_buff[100];// telemetry buffer
uint16_t TxCount = 0;
uint16_t RxCount = 0;
int previous_course = 0;
unsigned long previous_location_time = 0;
unsigned long previous_status_time = 0;

TinyGPSPlus gps;
Adafruit_BMP085 bmp;
String serialCommand;


void setup() {
  wdt_enable(WDTO_8S);
  //analogReference(INTERNAL2V56);
  pinMode(RfPDPin, OUTPUT);
  pinMode(GpsVccPin, OUTPUT);
  pinMode(RfPwrHLPin, OUTPUT);
  pinMode(RfPttPin, OUTPUT);
  //pinMode(BattPin, INPUT);
  pinMode(PIN_DRA_TX, INPUT);

  // Set initial conditions
  RfPttOFF;
  RfPwrHigh;
  RfON;
  GpsON;
  
  Serial.begin(115200);
  Serial1.begin(9600);

  Serial.println(F("George M1GEO"));
  Serial.println(F("Starting..."));
  
  APRS_init(ADC_REFERENCE, OPEN_SQUELCH);
  APRS_setCallsign(CallSign, CallNumber);
  APRS_setDestination("APZMDM", 0);
  APRS_setMessageDestination("APZMDM", 0);
  APRS_setPath1("WIDE1", 1);
  APRS_setPath2("WIDE2", 1);
  APRS_useAlternateSymbolTable(alternateSymbolTable);
  APRS_setSymbol(Symbol);
  APRS_setPreamble(1000UL); // DRA818 is slow!
  APRS_setTail(100UL);
  //APRS_setPathSize(pathSize);  // Use standard LibAPRS (not modified version)
  //APRS_setPower(1);
  //APRS_setHeight(0);
  //APRS_setGain(3);
  //APRS_setDirectivity(0);
  AprsPinInput;

  wdt_reset();

  configDra818(Frequency, 0, 8); // freq, squ, vol.
  
  bmp.begin();

  APRS_printSettings();
}


void loop() {
  wdt_reset();

  updateGpsData(2000);
  gpsDebug();
  
  if (aliveStatus) {
    sendCompilation();
    wdt_reset();
    delay(1000);
    sendStatus();
    previous_status_time = millis();
    aliveStatus = false;
    wdt_reset();
  }

  //send status message every 10 minutes
  if ((millis() - previous_status_time) > 600000UL) {
    wdt_reset();
    sendStatus();
    previous_status_time = millis();
  }

  if ((gps.location.age() < 1000 || gps.location.isUpdated()) && gps.location.isValid()) {
    if (gps.satellites.isValid() && (gps.satellites.value() > 3)) {
      updatePosition();
      updateTelemetry();

      wdt_reset();

      // calculate the absoloute difference of course.
      int course_change = gps.course.deg();
      if (course_change > previous_course) {
        course_change = course_change - previous_course;
      } else {
        course_change = previous_course - course_change;
      }
      
      if (
        ( ((millis() - previous_location_time) > (BeaconInterval * 1000UL)) ||    // If the last time over the BeaconInterval seconds ago, or;
          (course_change > CourseChangeBeacon) ||                                 // If the course has changed, or; 
          (previous_location_time == 0)                                           // If we just started,
        ) && ((millis() - previous_location_time) > (MinimumInterval * 1000UL))   // And, we last transmitted over MinimumInterval seconds ago.
      ) {
        if (gps.course.isValid()) {
          previous_course = (int) gps.course.deg();
        }
        
        //APRS_setPathSize(pathSize); // Use standard LibAPRS (not modified version)
        
        wdt_reset();
        sendLocation();
        previous_location_time = millis();
      }
      
    } else {
      Serial.println(F("Not enough sattelites"));
    }
  }
  
  freeMem();
  Pkts();
  
  // Process any received packet
  processPacket();
  Serial.flush();
}


boolean gotPacket = false;
AX25Msg incomingPacket;
uint8_t *packetData;
void aprs_msg_callback(struct AX25Msg *msg) {
  RxCount++;
  // If we already have a packet waiting to be
  // processed, we must drop the new one.
  if (!gotPacket) {
    // Set flag to indicate we got a packet
    gotPacket = true;

    // The memory referenced as *msg is volatile
    // and we need to copy all the data to a
    // local variable for later processing.
    memcpy(&incomingPacket, msg, sizeof(AX25Msg));

    // We need to allocate a new buffer for the
    // data payload of the packet. First we check
    // if there is enough free RAM.
    if (freeMemory() > msg->len) {
      packetData = (uint8_t*)malloc(msg->len);
      memcpy(packetData, msg->info, msg->len);
      incomingPacket.info = packetData;
    } else {
      // We did not have enough free RAM to receive
      // this packet, so we drop it.
      gotPacket = false;
    }
  }
}


// Here's a function to process incoming packets
// Remember to call this function often, so you
// won't miss any packets due to one already
// waiting to be processed
void processPacket() {
  if (gotPacket) {
    gotPacket = false;
    
    Serial.print(F("Received APRS packet. SRC: "));
    Serial.print(incomingPacket.src.call);
    Serial.print(F("-"));
    Serial.print(incomingPacket.src.ssid);
    Serial.print(F(". DST: "));
    Serial.print(incomingPacket.dst.call);
    Serial.print(F("-"));
    Serial.print(incomingPacket.dst.ssid);
    Serial.print(F(". Data: "));

    for (int i = 0; i < incomingPacket.len; i++) {
      Serial.write(incomingPacket.info[i]);
    }
    Serial.println("");

    // Remeber to free memory for our buffer!
    free(packetData);

    // You can print out the amount of free
    // RAM to check you don't have any memory
    // leaks
    // Serial.print(F("Free RAM: ")); Serial.println(freeMemory());
  }
}


byte configDra818(char *freq, uint8_t squelch, uint8_t volume) {
  /* 
   *  AT+DMOSETGROUP=GBW,TFV, RFV,Tx_CTCSS,SQ,Rx_CTCSS<CR><LF>
   *  
   *  GBW: Channels space. 1=25k. The RFIC RDA1846S offers the channel space selection
   *    for 12.5KHz and 25KHz. The firmware only chooses 25KHz option fixedly. However
   *    the frequency modulation deviation in 25KHz channel space is still very narrow
   *    (about +/-2.5KHz) so the module works more likely at 12.5KHz channel space.
   *  TFV: Transmit frequency. Range: 134.0000~174.0000MHz
   *  RFV: Receive frequency.  Range: 134.0000~174.0000MHz
   *  Tx_CTCSS: CTCSS value in transmit 
   *  SQ: Squelch level (0~8). 0=monitor mode which can’t be used in scanning mode.
   *  Rx_CTCSS: CTCSS value in receive
   */
  squelch = squelch <= 8 ? squelch : 8;
  volume  = volume  <= 8 ? volume  : 8;
  SoftwareSerial Serial_dra(PIN_DRA_RX, PIN_DRA_TX);
  Serial_dra.begin(9600);
  char ack[3];
  int n;
  delay(2000);
  char cmd[50];
  // bw(0=12.5k/1=25k), tx_f, rx_f, tx_ctcss, squelch, rx_ctcss
  sprintf(cmd, "AT+DMOSETGROUP=0,%s,%s,0000,%c,0000", freq, freq, squelch + '0');
  Serial_dra.println(cmd);
  ack[2] = 0;
  while (ack[2] != '\n')
  {
    if (Serial_dra.available() > 0) {
      ack[0] = ack[1];
      ack[1] = ack[2];
      ack[2] = Serial_dra.read();
    }
  }
  if (ack[0] == '0') {
    Serial.println(F("Frequency updated..."));
  } else {
    Serial.println(F("Frequency update error!"));
  }
  
  delay(50);
  sprintf(cmd, "AT+DMOSETVOLUME=%c", volume + '0');
  Serial_dra.println(cmd);
  ack[2] = 0;
  while (ack[2] != '\n')
  {
    if (Serial_dra.available() > 0) {
      ack[0] = ack[1];
      ack[1] = ack[2];
      ack[2] = Serial_dra.read();
    }
  }
  if (ack[0] == '0') {
    Serial.println(F("Volume updated..."));
  } else {
    Serial.println(F("Volume update error!"));
  }

  Serial_dra.end();
  pinMode(PIN_DRA_TX, INPUT);
  return (ack[0] == '0') ? 1 : 0;
}


void updatePosition() {
  // Convert and set latitude NMEA string Degree Minute Hundreths of minutes ddmm.hh[S,N].
  char latStr[10];
  int temp = 0;
  double d_lat = gps.location.lat();
  double dm_lat = 0.0;

  if (d_lat < 0.0) {
    temp = -(int)d_lat;
    dm_lat = temp * 100.0 - (d_lat + temp) * 60.0;
  } else {
    temp = (int)d_lat;
    dm_lat = temp * 100 + (d_lat - temp) * 60.0;
  }

  dtostrf(dm_lat, 7, 2, latStr);

  if (dm_lat < 1000) {
    latStr[0] = '0';
  }

  if (d_lat >= 0.0) {
    latStr[7] = 'N';
  } else {
    latStr[7] = 'S';
  }

  APRS_setLat(latStr);

  // Convert and set longitude NMEA string Degree Minute Hundreths of minutes ddmm.hh[E,W].
  char lonStr[10];
  double d_lon = gps.location.lng();
  double dm_lon = 0.0;

  if (d_lon < 0.0) {
    temp = -(int)d_lon;
    dm_lon = temp * 100.0 - (d_lon + temp) * 60.0;
  } else {
    temp = (int)d_lon;
    dm_lon = temp * 100 + (d_lon - temp) * 60.0;
  }

  dtostrf(dm_lon, 8, 2, lonStr);

  if (dm_lon < 10000) {
    lonStr[0] = '0';
  }
  if (dm_lon < 1000) {
    lonStr[1] = '0';
  }
  if (dm_lon < 100) { // gs bodge
    lonStr[2] = '0';
  }
    if (dm_lon < 10) { // gs bodge
    lonStr[3] = '0';
  }
  if (d_lon >= 0.0) {
    lonStr[8] = 'E';
  } else {
    lonStr[8] = 'W';
  }

  APRS_setLon(lonStr);
}


void updateTelemetry() {
  sprintf(telemetry_buff, "%03d", gps.course.isValid() ? (int)gps.course.deg() : 0);
  telemetry_buff[3] += '/';
  sprintf(telemetry_buff + 4, "%03d", gps.speed.isValid() ? (int)gps.speed.knots() : 0);
  telemetry_buff[7] = '/';
  telemetry_buff[8] = 'A';
  telemetry_buff[9] = '=';
  sprintf(telemetry_buff + 10, "%06d", (long)gps.altitude.feet());
  telemetry_buff[16] = ' ';
  sprintf(telemetry_buff + 17, "%03d", TxCount);
  telemetry_buff[20] = 'T';
  telemetry_buff[21] = 'X';
  telemetry_buff[22] = 'P';
  telemetry_buff[23] = ' '; 
  sprintf(telemetry_buff + 24, "%03d", RxCount);
  telemetry_buff[27] = 'R';
  telemetry_buff[28] = 'X';
  telemetry_buff[29] = 'P';
  telemetry_buff[30] = ' '; 
  float tempC = bmp.readTemperature();//-21.4;//
  dtostrf(tempC, 6, 2, telemetry_buff + 31);
  telemetry_buff[37] = 'C';
  telemetry_buff[38] = ' '; float pressure = bmp.readPressure() / 100.0; //Pa to hPa
  dtostrf(pressure, 7, 2, telemetry_buff + 39);
  telemetry_buff[46] = 'h';
  telemetry_buff[47] = 'P';
  telemetry_buff[48] = 'a';
  telemetry_buff[49] = ' ';
  //dtostrf(readBatt(), 5, 2, telemetry_buff + 50);
  dtostrf(0, 5, 2, telemetry_buff + 50);
  telemetry_buff[55] = 'V';
  telemetry_buff[56] = ' ';
  sprintf(telemetry_buff + 57, "%02d", gps.satellites.isValid() ? (int)gps.satellites.value() : 0);
  telemetry_buff[59] = 'S';
  telemetry_buff[60] = ' ';
  sprintf(telemetry_buff + 61, "%s", comment);
}


void sendLocation() {
  Serial.print(F("Location sending with comment: "));
  Serial.println(telemetry_buff); // telemetry buffer
  AprsPinOutput;
  RfPttON;
  delay(TxDelay);
  APRS_sendLoc(telemetry_buff, strlen(telemetry_buff));
  delay(50);
  while (digitalRead(1)); //LibAprs TX Led pin PB1 - wait for TX to finish
  RfPttOFF;
  AprsPinInput;
  TxCount++;
}


void sendStatus() {
  Serial.println(F("sendStatus"));
  AprsPinOutput;
  RfPttON;
  delay(TxDelay);
  APRS_sendStatus(StatusMessage, strlen(StatusMessage));
  while (digitalRead(1)); //LibAprs TX Led pin PB1 - wait for TX to finish
  RfPttOFF;
  AprsPinInput;
  TxCount++;
}


void sendCompilation() {
  Serial.println(F("sendCompilation"));
  char sz[100];
  sprintf(sz, "M1GEO FW: %s %s", __DATE__, __TIME__);
  AprsPinOutput;
  RfPttON;
  delay(TxDelay);
  APRS_sendStatus(sz, strlen(sz));
  while (digitalRead(1)); //LibAprs TX Led pin PB1 - wait for TX to finish
  RfPttOFF;
  AprsPinInput;
  TxCount++;
}


static void updateGpsData(int ms)
{
  while (!Serial1) {
    delayMicroseconds(1); // wait for serial port to connect.
  }
  unsigned long start = millis();
  unsigned long bekle = 0;
  do
  {
    while (Serial1.available() > 0) {
      char c;
      c = Serial1.read();
      gps.encode(c);
      bekle = millis();
    }
    if (bekle != 0 && bekle + 10 < millis()) break;
  } while (millis() - start < ms);
}

/*
float readBatt() {
  float R1 = 560000.0; // 560K
  float R2 = 100000.0; // 100K
  float value = 0.0;
  do {
    value = analogRead(BattPin);
    delay(5);
    value = analogRead(BattPin);
    value = value - 8;
    value = (value * 2.56) / 1024.0;
    value = value / (R2 / (R1 + R2));
  } while (value > 16.0);
  return value ;
}
*/

void freeMem() {
  Serial.print(F("Free RAM: "));
  Serial.print(freeMemory());
  Serial.println(F(" byte"));
}


void Pkts() {
  Serial.print(F("Packets: "));
  Serial.print(TxCount);
  Serial.print(F(" TX / "));
  Serial.print(RxCount);
  Serial.print(F(" RX."));
}


void gpsDebug() {
  Serial.println();
  Serial.println(F("Sats HDOP Latitude   Longitude   Fix  Date       Time     Date Alt    Course Speed Card Chars Sentences Checksum"));
  Serial.println(F("          (deg)      (deg)       Age                      Age  (m)    --- from GPS ----  RX    RX        Fail"));
  Serial.println(F("-----------------------------------------------------------------------------------------------------------------"));

  printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
  printInt(gps.hdop.value(), gps.hdop.isValid(), 5);
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  printInt(gps.location.age(), gps.location.isValid(), 5);
  printDateTime(gps.date, gps.time);
  printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
  printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
  printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.value()) : "*** ", 6);

  printInt(gps.charsProcessed(), true, 6);
  printInt(gps.sentencesWithFix(), true, 10);
  printInt(gps.failedChecksum(), true, 9);
  Serial.println();
}


static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid) {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  } else {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i = flen; i < len; ++i)
      Serial.print(' ');
  }
}


static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i = strlen(sz); i < len; ++i)
    sz[i] = ' ';
  if (len > 0)
    sz[len - 1] = ' ';
  Serial.print(sz);
}


static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    Serial.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.day(), d.month(), d.year());
    Serial.print(sz);
  }

  if (!t.isValid())
  {
    Serial.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
}


static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i = 0; i < len; ++i)
    Serial.print(i < slen ? str[i] : ' ');
}

// taken from modified version of LibAPRS
void APRS_sendStatus(void *_buffer, size_t length) {
    size_t payloadLength = 1+length;

    uint8_t *packet = (uint8_t*)malloc(payloadLength);
    uint8_t *ptr = packet;
    packet[0] = '>';
    ptr++;

    if (length > 0) {
        uint8_t *buffer = (uint8_t *)_buffer;
        memcpy(ptr, buffer, length);
    }

    APRS_sendPkt(packet, payloadLength);
    free(packet);
}
