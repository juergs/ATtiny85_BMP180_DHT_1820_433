//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ATMEL ATTINY45 / ARDUINO
//
//                  +-\/-+
// Ain0 (D 5) PB5  1|    |8  Vcc
// Ain3 (D 3) PB3  2|    |7  PB2 (D 2)  Ain1  [I2C SCL]
// Ain2 (D 4) PB4  3|    |6  PB1 (D 1)  pwm1
//            GND  4|    |5  PB0 (D 0)  pwm0  [I2C SDA]
//                  +----+
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Original from:  Matthias Busse 20.6.2014 Version 1.0, BMP180 Luftdruck und Temperatur Sensor
//                 http://shelvin.de/bmp180-luftdruck-und-temp-sensor-am-arduino-uno/#more-900
// Inspired by:    Selbstbau Funkthermometer 433Mhz (juergs)
//                 https://forum.fhem.de/index.php/topic,52755.0.html 
//                 http://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
// Thanks to:      FHEM-Forum: simonberry: https://forum.fhem.de/index.php/topic,50333.0.html for evolving the protocol.
// Specials:       http://www.dexterindustries.com/howto/working-with-avr/any-port-any-pin-a-twi-master-for-attiny-atmega/
//                 http://www.daemon.de/blog/2012/10/10/182/error-twsr-undeclared-first-use-function/
//                 Natürlich hat das nichts gerbacht, zumal die avr/io.h bereits included war. 
//                 Langer Rede, kurzer Sinn: letzlich habe ich herausgefunden, dass der Attiny85 halt keinen TWI Support hat. 
//                 Somit sind die entsprechenden Macros nicht definiert und daher kamen die Fehler.
//                 Zum Glück hat der Attiny85 aber USI Support eingebaut und dafür gibt es auch eine I2C Slave Library. Läuft.
//                 http://playground.arduino.cc/Code/USIi2c  statts twi-Lib
//                 Be sure to use pullups! (4.7K on 5V). I2C with these libs is not as forgiving as the Wire lib. (Maybe the internal pullups are not set.)
//                 http://brohogan.blogspot.de/search/label/ATtiny85 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
  readVcc: 
  ========
  Improving Accuracy
  While the large tolerance of the internal 1.1 volt reference greatly limits the accuracy of this measurement, 
  for individual projects we can compensate for greater accuracy. 
  To do so, simply measure your Vcc with a voltmeter and with our readVcc() function. 
  Then, replace the constant 1125300L with a new constant:
          scale_constant = internal1.1Ref * 1023 * 1000
  where
          internal1.1Ref = 1.1 * Vcc1 (per voltmeter) / Vcc2 (per readVcc() function)
                         = 1.1 * 5126 / 5258

  This calibrated value will be good for the AVR chip measured only, and may be subject to temperature variation. 
  Feel free to experiment with your own measurements.
*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
#include "Wire.h"
#include "Narcoleptic.h"
#include "LaCrosse.h"    // FHEM-Forum: simonberry: https://forum.fhem.de/index.php/topic,50333.0.html
#include "DHT22.h"
//
//#define VERBOSE_MODE    1
//
#define SENSOR_PIN      3   //   PIN2  --- not used for BMP180
#define TX_433_PIN      4   //   PINx   - PIN_SEND in LaCrosse.cpp

//--- BMP180 has I2C-Interface using:
//      SDA = Pin5 (MOSI/PB0/D0) 
//      SCL = Pin7 (SCL/PB2/D2)

#define PIN_SEND      TX_433_PIN    //--- where digal pin 433 Sender (TX) is connected PB0 on ATtiny-85 

//*********************************************
#define SENSORID_BMP    117         // MaxID = 127 (!), Sensor-ID wird in Setup-Methode gesetzt!
#define SENSORID_DHT    118         // MaxID = 127 (!), Sensor-ID wird in Setup-Methode gesetzt!
#define SENSORID_VCC    119         // MaxID = 127 (!), Sensor-ID wird in Setup-Methode gesetzt!
//*********************************************
#define DHT22_PIN       SENSOR_PIN // --- not used for BMP180, because I2C has defined ports. 
//*********************************************
//
//--- Einstellungen für den BMP180
#define I2C_ADDRESS 0x77

//--- setup a global DHT22 instance
DHT22 myDHT22(DHT22_PIN);

const unsigned char oversampling_setting = 3; //oversamplig:  0 ungenau (13ms) ... 3 genau (34ms)
const unsigned char pressure_waittime[4] = { 5, 8, 14, 26 };
int ac1, ac2, ac3, b1, b2, mb, mc, md;
unsigned int ac4, ac5, ac6;

int   temp          = 20;
int   temp_mittel   = 200;
long  druck         = 1013;      
long  druck_mittel  = 101300;
float temp_offset   = 0.0;
float druck_offset  = 1.0;      //--- Korrekturwerte
int   mitteln       = 5;
float dht_temp      = 0.0;
float dht_hum       = 0.0;
float controller_VCC = 0.0; 
long  vcc_reading    = 0; 
//----------------------------------------------------
//long readVcc();           // prototype
//----------------------------------------------------
void setup() 
{
   #if USE_WITH_LEONARDO
      Serial.begin(57600);  
      delay(2000);    
      Serial.println("BMP180 Temperatur und Luftdruck auslesen.");
   #endif
    
   //--- inits
   pinMode(PIN_SEND, OUTPUT);
  
   #if USE_WITH_LEONARDO
      pinMode(13, OUTPUT);
      digitalWrite(13,LOW); 
   #endif 

  //--- preset SensorId
  LaCrosse.bSensorId = SENSORID_BMP;
  LaCrosse.setTxPinMode(OUTPUT);
    
  //--- initialize I2C, fixed pins 
  Wire.begin();
  
  //--- initialize BMP
  bmp180_get_cal_data();
  bmp180_read_temperature_and_pressure(&temp_mittel,&druck_mittel); //--- erstmal Mittelwerte lesen
}
//---------------------------------------------------------------------
void loop() 
{
  bmp180_read_temperature_and_pressure(&temp, &druck); // dauert ca. 34ms
  temp_mittel = ((temp_mittel * (mitteln-1)) + temp) / mitteln;
  druck_mittel = ((druck_mittel * (mitteln-1)) + druck) / mitteln;

  float temperature =   ((float) temp_mittel/10.0)   + temp_offset; 
  float luftdruck    =  (((float) druck_mittel/100.0) + druck_offset - 700.0)/10; 

  #if USE_WITH_LEONARDO
    Serial.print("Temp.: ");
    Serial.print(( (float) temp_mittel/10.0) + temp_offset, 1);
    Serial.print(" Druck: ");
    Serial.print(((float)druck_mittel/100.0)+druck_offset, 2);
    Serial.print(" Druck[corr]: ");
    Serial.println((float) luftdruck, 2);
  #endif
  
  delay(1000); // ms

  #if USE_WITH_LEONARDO
    digitalWrite(13,HIGH);
  #endif 

  //--- transfer measured values to LaCrosse-instance
  LaCrosse.bSensorId = SENSORID_BMP;
  LaCrosse.t = temperature;   //temperature;
  LaCrosse.h = luftdruck;    //luftdruck;
  LaCrosse.sendTemperature();
  LaCrosse.sleep(1);        /* 1 second, no power-reduction! */
  LaCrosse.sendPress();

  #if USE_WITH_LEONARDO
    digitalWrite(13,LOW);
  #endif
  
  LaCrosse.sleep(1);        /* 1 second, no power-reduction! */

  #if USE_WITH_LEONARDO
    digitalWrite(13,HIGH);
  #endif 
  
  DHT22_ERROR_t errorCode = myDHT22.readData();
  //LaCrosse.sleep(1);
  //errorCode = myDHT22.readData(); // doppelte Auslesung um einen aktuellen Wert zu erhalten
  
  switch (errorCode)
  {
  case DHT_ERROR_NONE:
    //---Get Data
    dht_temp = myDHT22.getTemperatureC();
    dht_hum  = myDHT22.getHumidity();
    break;
  case DHT_ERROR_CHECKSUM:
    //--- send erraneous data as err indicator
    dht_temp = -99.99;
    dht_hum = -99.99;
    break;
  case DHT_BUS_HUNG:
    dht_temp = -88.88;
    dht_hum = -88.88;
    break;
  case DHT_ERROR_NOT_PRESENT:
    dht_temp = -77.77;
    dht_hum = -77.77;
    break;
  case DHT_ERROR_ACK_TOO_LONG:
    dht_temp = -66.66;
    dht_hum = -66.66;
    break;
  case DHT_ERROR_SYNC_TIMEOUT:
    dht_temp = -55.55;
    dht_hum = -55.55;
    break;
  case DHT_ERROR_DATA_TIMEOUT:
    dht_temp = -44.44;
    dht_hum = -44.44;
    break;
  case DHT_ERROR_TOOQUICK:
    dht_temp = -33.33;
    dht_hum = -33.33;
    break;
  default:
    dht_temp = -22.22;
    dht_hum = -22.22;
  }

  //--- transfer measured values to LaCrosse-instance
  LaCrosse.bSensorId = SENSORID_DHT;
  LaCrosse.t = dht_temp;  
  LaCrosse.sendTemperature();
  LaCrosse.sleep(1);        /* 1 second, no power-reduction! */
  LaCrosse.h = dht_hum;
  LaCrosse.sendHumidity();

  #if USE_WITH_LEONARDO
    digitalWrite(13,LOW);
  #endif

  LaCrosse.sleep(1);        /* 1 second, no power-reduction! */

  #if USE_WITH_LEONARDO
    digitalWrite(13,HIGH);
  
    long vcc = readVcc(); 
    Serial.print("VCC = ");
    Serial.println(vcc,DEC);  
    Serial.println("==============================================="); 
    Serial.println();
  #endif 

  vcc_reading = readVCC(); 
  controllerVCC = 1.1 * 1023 / vcc_reading; 

  LaCrosse.bSensorId = SENSORID_VCC;
  LaCrosse.t = controllerVCC;
  LaCrosse.sendTemperature();

  #if USE_WITH_LEONARDO
    digitalWrite(13,LOW);
    delay(60000); // ms
  #else
    Narcoleptic.delay_minutes(3);
  #endif 
}
//---------------------------------------------------------------------
void bmp180_read_temperature_and_pressure(int* temp, long* druck) 
{
  int  ut= bmp180_read_ut();
  long up = bmp180_read_up();
  long x1, x2, x3, b3, b5, b6, p;
  unsigned long b4, b7;

  x1 = ((long)ut - ac6) * ac5 >> 15;    //--- Temperatur berechnen
  x2 = ((long) mc << 11) / (x1 + md);
  b5 = x1 + x2;
  *temp = (b5 + 8) >> 4;

  b6 = b5 - 4000; //Druck berechnen
  x1 = (b2 * (b6 * b6 >> 12)) >> 11;
  x2 = ac2 * b6 >> 11;
  x3 = x1 + x2;

  if (oversampling_setting == 3) b3 = ((int32_t) ac1 * 4 + x3 + 2) << 1;
  if (oversampling_setting == 2) b3 = ((int32_t) ac1 * 4 + x3 + 2);
  if (oversampling_setting == 1) b3 = ((int32_t) ac1 * 4 + x3 + 2) >> 1;
  if (oversampling_setting == 0) b3 = ((int32_t) ac1 * 4 + x3 + 2) >> 2;

  x1 = ac3 * b6 >> 13;
  x2 = (b1 * (b6 * b6 >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (ac4 * (uint32_t) (x3 + 32768)) >> 15;
  b7 = ((uint32_t) up - b3) * (50000 >> oversampling_setting);
  p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;

  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  *druck = p + ((x1 + x2 + 3791) >> 4);
}
//---------------------------------------------------------------------
unsigned int bmp180_read_ut() 
{
  write_register(0xf4,0x2e);
  delay(5); //mehr als 4.5 ms
  return read_int_register(0xf6);
}
//---------------------------------------------------------------------
void bmp180_get_cal_data() 
{
  ac1 = read_int_register(0xAA);
  ac2 = read_int_register(0xAC);
  ac3 = read_int_register(0xAE);
  ac4 = read_int_register(0xB0);
  ac5 = read_int_register(0xB2);
  ac6 = read_int_register(0xB4);
  b1 = read_int_register(0xB6);
  b2 = read_int_register(0xB8);
  mb = read_int_register(0xBA);
  mc = read_int_register(0xBC);
  md = read_int_register(0xBE);
}
//---------------------------------------------------------------------
long bmp180_read_up() 
{
  write_register(0xf4,0x34+(oversampling_setting<<6));
  delay(pressure_waittime[oversampling_setting]);

  unsigned char msb, lsb, xlsb;
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(0xf6);
  Wire.endTransmission();

  Wire.requestFrom(I2C_ADDRESS, 3); 
  while(!Wire.available()) {} // warten
  msb = Wire.read(); 
  while(!Wire.available()) {} // warten
  lsb |= Wire.read(); 
  while(!Wire.available()) {} // warten
  xlsb |= Wire.read(); 
  return (((long)msb<<16) | ((long)lsb<<8) | ((long)xlsb)) >> (8-oversampling_setting);
}
//---------------------------------------------------------------------
void write_register(unsigned char r, unsigned char v) 
{
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(r); 
  Wire.write(v); 
  Wire.endTransmission();
}
//---------------------------------------------------------------------
char read_register(unsigned char r) 
{
  unsigned char v;

  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(r); 
  Wire.endTransmission();

  Wire.requestFrom(I2C_ADDRESS, 1); 
  while(!Wire.available()) {} // warten
  v = Wire.read(); 
  return v;
}
//---------------------------------------------------------------------
int read_int_register(unsigned char r) 
{
  unsigned char msb, lsb;

  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(r); 
  Wire.endTransmission();

  Wire.requestFrom(I2C_ADDRESS, 2);
  while(!Wire.available()) {} // warten
  msb = Wire.read(); 
  while(!Wire.available()) {} // warten
  lsb = Wire.read(); 
  return (((int)msb<<8) | ((int)lsb));
}
//---------------------------------------------------------------------
long readVcc() 
{
  //--- read 1.1V reference against AVcc
  //--- set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  

  delay(2); // Wait for Vref to settle
  
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

/*
 *        internal1.1Ref = 1.1 * Vcc1 (per voltmeter) / Vcc2 (per readVcc() function)
                         = 1.1 * 5126 / 5258 => 1.09 ==> 1.09*1023*1000 = 1097049
*/ 

  //result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  result   = 1097049L / result; // korrigierter Wert
  
  return result; // Vcc in millivolts
}
//---------------------------------------------------------------------
// <eof>
//---------------------------------------------------------------------

