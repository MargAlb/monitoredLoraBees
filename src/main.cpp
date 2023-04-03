/* Heltec Automation send communication test example
 *
 * Function:
 * 1. Send data from a CubeCell device over hardware 
 * 
 * 
 * this project also realess in GitHub:
 * https://github.com/HelTecAutomation/ASR650x-Arduino
 * */

#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <HX711.h>

// #define MODE_TESTING_ONLY 1

#define ONE_WIRE_BUS GPIO0
OneWire oneWire(ONE_WIRE_BUS);


// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);


#define SEALEVELPRESSURE_HPA (1013.25)
#define USEIIC 1
#define BME280_ADDRESS (0x76)
Adafruit_BME280 bme280; // I2C


/*
 * set LoraWan_RGB to 1,the RGB active in loraWan
 * RGB red means sending;
 * RGB green means received done;
 */
#ifndef LoraWan_RGB
#define LoraWan_RGB 0
#endif

#define RF_FREQUENCY                                868700000 // Hz
#define TX_OUTPUT_POWER                             14        // dBm
#define LORA_BANDWIDTH                              0         // [0: 125 kHz,                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false


#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                150 // Define the payload size here

// ---- toms paramater 15 minuten takt
#ifndef MODE_TESTING_ONLY
  #define timetillwakeup 1000*60*15
  const char DEVICE_ID[] PROGMEM = "4567";
#endif
#ifdef MODE_TESTING_ONLY
  const char DEVICE_ID[] PROGMEM = "5678";
  #define timetillwakeup 1000*3
#endif

#define WITH_SERIAL_LOGGING    1


const int LOADCELL_DOUT_PIN = GPIO4;
const int LOADCELL_SCK_PIN = GPIO5;
HX711 scale; 

#define LOADCELL_CALIB_FACTOR 11353
#define LOADCELL_TARA 21875

static TimerEvent_t sleep;
static TimerEvent_t wakeUp;
uint8_t lowpower=0;


char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );

double insideTemp; //field1
double outsideTemp; //field2
double humidity; //field3
double airPressure; //field4
double voltage;//field5
double weight; // field6
double airPressureHPA;
double unusedField;


char logMsg[255];

void logSerial( char *msg );

int16_t rssi,rxSize;
void  DoubleToString( char *str, double double_num,unsigned int len);
unsigned BME280detected;


typedef enum
{
    READSENSORDATA,
    TX,
    WAITING
}States_t;

States_t state;

void logSerial( char *msg ) {
  if (WITH_SERIAL_LOGGING) {
    Serial.printf("OOOOOO   %s\r\n",msg);  
  }
}

void onSleep()
{
  sprintf(logMsg,"Going into lowpower mode, %d ms later wake up.\r\n",timetillwakeup);
  logSerial( logMsg );
  digitalWrite(Vext, HIGH);
  //delay(500);
  lowpower=1;
  //timetillwakeup ms later wake up;
  TimerSetValue( &wakeUp, timetillwakeup );
  TimerStart( &wakeUp );
}

void onWakeUp()
{
  logSerial("Woke up");
  lowpower=0;
  digitalWrite(Vext, LOW);
  delay(500);
  state=READSENSORDATA;
}


void sendLoraData()
{
    //turnOnRGB(COLOR_RECEIVED,0); //change rgb color  ... funktioniert nicht mit lowpower und sensor
  generateDataPacket();  
    //turnOnRGB(COLOR_SEND,0); //change rgb color    ... funktioniert nicht mit lowpower und sensor
    sprintf(logMsg,"sending packet \"%s\" , length %d\r\n",txpacket, strlen(txpacket));
    logSerial( logMsg );
  Radio.Send( (uint8_t *)txpacket, strlen(txpacket) ); //send the package out 
  logSerial("STATE: SENDING");
  state=WAITING;

  //delay(500);
    //turnOffRGB();    //... funktioniert nicht mit lowpower und sensor
 
}

void generateDataPacket( void )
{
    sprintf(txpacket,"<%s>field1=",DEVICE_ID);
    DoubleToString(txpacket,insideTemp,1);
    sprintf(txpacket,"%s&field2=",txpacket);
    DoubleToString(txpacket,outsideTemp,1);
    sprintf(txpacket,"%s&field3=",txpacket);
    DoubleToString(txpacket,humidity,1);
    sprintf(txpacket,"%s&field4=",txpacket);
    DoubleToString(txpacket,airPressureHPA,3);  
    sprintf(txpacket,"%s&field5=",txpacket);
    DoubleToString(txpacket,voltage,3);
    sprintf(txpacket,"%s&field6=",txpacket);
    DoubleToString(txpacket,weight,3);
}
inline
boolean isUndefined(float value) {
  return isnan(value) || value == -127.0f;
}

void readSensorValues( void )
{
  logSerial("bin im readSensoValues");
    digitalWrite(Vext, LOW);
    delay(2000);
    BME280detected=bme280.begin();
    if (BME280detected) {
      logSerial("BME280 detected");
      outsideTemp=bme280.readTemperature();
      humidity=bme280.readHumidity();
      airPressure=bme280.readPressure();
      airPressureHPA=(airPressure / 100.0);
      unusedField=bme280.readAltitude(SEALEVELPRESSURE_HPA);
    }
    //wire Temperatures
    sensors.requestTemperatures();
    delay(10);
    sensors.requestTemperatures();

    insideTemp=sensors.getTempCByIndex(0);
    Wire.end();

    if (scale.is_ready()) {
       logSerial("HX711 detected");
       long reading = scale.read() - LOADCELL_TARA;
           Serial.printf("hx711   %s\r\n","AHA!");  
           Serial.printf("hx711   %ld\r\n",reading);  

       weight = (1.0 * reading / LOADCELL_CALIB_FACTOR);
    } 

    uint16_t voltageRAW = getBatteryVoltage();
    voltage = static_cast<double>(voltageRAW);
    voltage = (voltage / 1000.0);
  
  state=TX;
  logSerial("STATE: TX");
}

void OnTxDone( void )
{
  logSerial("TX done!");
  //delay(500);
  state=WAITING;
  onSleep();
    //turnOnRGB(0,0); // ... funktioniert nicht mit lowpower und sensor
}

void OnTxTimeout( void )
{
    //Radio.Sleep( );
    logSerial("TX Timeout......");
    state=READSENSORDATA;
    logSerial("STATE: READSENSORDATA");
}

/**
  * @brief  Double To String
  * @param  str: Array or pointer for storing strings
  * @param  double_num: Number to be converted
  * @param  len: Fractional length to keep
  * @retval None
  */
void  DoubleToString( char *str, double double_num,unsigned int len) { 
  double fractpart, intpart;
  fractpart = modf(double_num, &intpart);
  fractpart = fractpart * (pow(10,len));
  sprintf(str + strlen(str),"%d", (int)(intpart)); //Integer part
  sprintf(str + strlen(str), ".%d", (int)(fractpart)); //Decimal part
}

void setupHX711() {
  logSerial("Initializing the LOAD_CELL");
  delay(5000);
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  //Serial.println("Before setting up the scale:");
  //Serial.print("read: \t\t");
  //Serial.println(scale.read());      // print a raw reading from the ADC

  
  //Serial.print("read average: \t\t");
  //Serial.println(scale.read_average(20));   // print the average of 20 readings from the ADC

  //Serial.print("get value: \t\t");
  //Serial.println(scale.get_value(5));   // print the average of 5 readings from the ADC minus the tare weight (not set yet)

  //Serial.print("get units: \t\t");
  //Serial.println(scale.get_units(5), 1);  // print the average of 5 readings from the ADC minus tare weight (not set) divided
            // by the SCALE parameter (not set yet)

  //scale.set_scale(2280.f);                      // this value is obtained by calibrating the scale with known weights; see the README for details
  scale.tare();               // reset the scale to 0

  logSerial("After setting up the scale:");
}
void setup() {
    pinMode(Vext, OUTPUT);
    digitalWrite(Vext, LOW);
    delay(500);
    if (WITH_SERIAL_LOGGING) {
      Serial.begin(115200);    
      delay(2000);
      while(!Serial);    // time to get serial running
    }
    logSerial("monitoredBees is initializing ");
    sensors.begin();
    BME280detected=bme280.begin();
  
    if (BME280detected) {
      sprintf(logMsg,"BME280 initialized= %s ","true");
      logSerial(logMsg);
    }
    if (!BME280detected) {
       sprintf(logMsg,"BME280 initialized= %s ","false");
       logSerial(logMsg);
    }
    //digitalWrite(Vext, LOW);
    #ifndef MODE_TESTING_ONLY
       setupHX711();
    #endif
    
    insideTemp=0.0;
    outsideTemp=0.0;
    humidity=0.0;
    airPressure=0.0;
    airPressureHPA=0.0;
    unusedField=0.0;
    voltage=0;
    weight=0;
    rssi=0;
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;

    Radio.Init( &RadioEvents );
    Radio.SetChannel( RF_FREQUENCY );
    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 ); 

    TimerInit( &sleep, onSleep );
    TimerInit( &wakeUp, onWakeUp );
    state=READSENSORDATA;
    logSerial("STATE:READSENSORDATA");
   }



void loop()
{ 
  if (lowpower==1) {
    lowPowerHandler();
  } else {
    switch(state)
    {
      case TX: {
        sendLoraData();
        break;
      }
      case READSENSORDATA:
      {
        readSensorValues();
        break;
      }
      case WAITING:
      {
        Radio.IrqProcess();
        break;
      }
      default:
      {  Radio.IrqProcess();
        logSerial("STATE:nicht vorhanden");
      }

    }
  }
}