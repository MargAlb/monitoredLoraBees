#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <HX711.h>
#include "./config.h"




#define ONE_WIRE_BUS GPIO0
OneWire oneWire(ONE_WIRE_BUS);


// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);


#define SEALEVELPRESSURE_HPA (1013.25)
#define USEIIC 1
#define BME280_ADDRESS (0x76)
Adafruit_BME280 bme280; // I2C



/*
 * set LoraWan_RGB to Active,the RGB active in loraWan
 * RGB red means sending;
 * RGB purple means joined done;
 * RGB blue means RxWindow1;
 * RGB yellow means RxWindow2;
 * RGB green means received done;
 */


/*LoraWan channelsmask, default channels 0-7*/ 
uint16_t userChannelsMask[6]={ 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = LORAWAN_CLASS;

/*the application data transmission duty cycle.  value in [ms].*/
#ifndef MODE_TESTING_ONLY
/* max 30sec airtime per day so we can choose: 
  -> sf7 = 0.06 s airtime ≈ 428msg/day
  -> sf8 = 0,12 sec airtime  ≈ 250 msg/day
  -> sf12 = 1,49s airtime   ≈ 20 msg/day
  uint32_t appTxDutyCycle = 1000*60*120;
*/
// every 15 minutes
uint32_t appTxDutyCycle = 1000*60*15;
#endif

#ifdef MODE_TESTING_ONLY
  uint32_t appTxDutyCycle = 60000;
#endif
/*
 * set LoraWan_RGB to 1,the RGB active in loraWan
 * RGB red means sending;
 * RGB green means received done;
 */
#ifndef LoraWan_RGB
 #define LoraWan_RGB 0
#endif

/*OTAA or ABP*/
bool overTheAirActivation = LORAWAN_NETMODE;


/*ADR enable*/
bool loraWanAdr = LORAWAN_ADR;

/* set LORAWAN_Net_Reserve ON, the node could save the network info to flash, when node reset not need to join again */
bool keepNet = LORAWAN_NET_RESERVE;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = LORAWAN_UPLINKMODE;

/* Application port */
uint8_t appPort = 2;
/*!
* Number of trials to transmit the frame, if the LoRaMAC layer did not
* receive an acknowledgment. The MAC performs a datarate adaptation,
* according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
* to the following table:
*
* Transmission nb | Data Rate
* ----------------|-----------
* 1 (first)       | DR
* 2               | DR
* 3               | max(DR-1,0)
* 4               | max(DR-1,0)
* 5               | max(DR-2,0)
* 6               | max(DR-2,0)
* 7               | max(DR-3,0)
* 8               | max(DR-3,0)
*
* Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
* the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
uint8_t confirmedNbTrials = 4;

boolean generateLoraPacket( void );
void setupHX711() ;


/* Prepares the payload of the frame */
static void prepareTxFrame( uint8_t port )
{
	/*appData size is LORAWAN_APP_DATA_MAX_SIZE which is defined in "commissioning.h".
	*appDataSize max value is LORAWAN_APP_DATA_MAX_SIZE.
	*if enabled AT, don't modify LORAWAN_APP_DATA_MAX_SIZE, it may cause system hanging or failure.
	*if disabled AT, LORAWAN_APP_DATA_MAX_SIZE can be modified, the max value is reference to lorawan region and SF.
	*for example, if use REGION_CN470, 
	*the max value for different DR can be found in MaxPayloadOfDatarateCN470 refer to DataratesCN470 and BandwidthsCN470 in "RegionCN470.h".
	*/
    appDataSize = 12;
    appData[0] = 0x00;
    appData[1] = 0x01;
    appData[2] = 0x02;
    appData[4] = 0x04;
    appData[5] = 0x05;
    appData[6] = 0x06;
    appData[7] = 0x07;
    appData[8] = 0x08;
    appData[9] = 0x09;
    appData[10] = 0x0a;
    appData[11] = 0x0b;
		generateLoraPacket();
}

char logMsg[255];


void logSerial( char *msg ) {
  if (WITH_SERIAL_LOGGING) {
    Serial.printf("OOOOOO   %s\r\n",msg);  
		delay(200);
  }
}

// LoadCell
const int LOADCELL_DOUT_PIN = GPIO4;
const int LOADCELL_SCK_PIN = GPIO5;
HX711 scale; 

#define LOADCELL_CALIB_FACTOR 11353
#define LOADCELL_TARA 21875

unsigned BME280detected;

char txpacket[150];
double insideTemp; //field1
double outsideTemp; //field2
double humidity; //field3
double airPressure; //field4
double voltage;//field5
double weight; // field6
double airPressureHPA;
double unusedField;

#define UNDEFINED_VALUE -32768


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


void initSensors(){
	  pinMode(Vext, OUTPUT);
    digitalWrite(Vext, LOW);
    delay(500);
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

inline
boolean isUndefined(float value) {
  return isnan(value) || value == -127.0f;
}

inline
boolean isDefined(float value) { return ! isUndefined(value); }


void add2LoraPacket(int index, float value, int precision){
  int factor = pow(10,precision);
  short currentValue = value * factor;
  appData[index+1] = highByte(currentValue);
  appData[index] = lowByte(currentValue);
	//appData[index] = asShort(value);
  // sprintf(logMsg, "currentValue: %d und nur als Byte : %02X %02X",currentValue,appData[index+1],appData[index]);
  // logSerial(logMsg);
}
void printAppData(){
  if (WITH_SERIAL_LOGGING) {
    int i;
    sprintf(logMsg,"Appdata: ");
    for (i = 0; i < 12; i++)
    {
        // if (i > 0) printf(":");
        sprintf(logMsg,"%s %02X",logMsg, appData[i]);
    }
    logSerial(logMsg);
  }
}
boolean generateLoraPacket( void )
{
    add2LoraPacket(0,insideTemp,2);
    add2LoraPacket( 2,outsideTemp,2);
    add2LoraPacket( 4,humidity,2);
    add2LoraPacket( 6,airPressureHPA,1);
    add2LoraPacket( 8,voltage,2);
    add2LoraPacket( 10,weight,2);
		printAppData();
    return true;
}


long readSensorValues( void )
{
	long int t1 = millis();

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
           sprintf(logMsg,"hx711   %s\r\n","AHA!");  
           logSerial(logMsg);
           sprintf(logMsg,"hx711   %ld\r\n",reading);  
           logSerial(logMsg);

       weight = (1.0 * reading / LOADCELL_CALIB_FACTOR);
    } 

    uint16_t voltageRAW = getBatteryVoltage();
    voltage = static_cast<double>(voltageRAW);
    voltage = (voltage / 1000.0);
    long int t2 = millis();
    boolean validAppData=generateLoraPacket();
    if (validAppData) logSerial("gültige Daten vorhanden");
  return (t2 -t1);
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

void setup() {
  if (WITH_SERIAL_LOGGING) {
      Serial.begin(115200);    
      delay(2000);
      while(!Serial);    // time to get serial running
  }
  prepareTxFrame( appPort );
	initSensors();

#if(AT_SUPPORT)
	enableAt();
#endif
	deviceState = DEVICE_STATE_INIT;
	LoRaWAN.ifskipjoin();
}

void loop() {
	switch( deviceState )
			{
				case DEVICE_STATE_INIT:
				{
        logSerial("##Device_Init## :");
		#if(LORAWAN_DEVEUI_AUTO)
					LoRaWAN.generateDeveuiByChipID();
		#endif
		#if(AT_SUPPORT)
					getDevParam();
		#endif
					printDevParam();
					LoRaWAN.init(loraWanClass,loraWanRegion);
					deviceState = DEVICE_STATE_JOIN;
					break;
				}
				case DEVICE_STATE_JOIN:
				{
        logSerial("##Join##: ");
					LoRaWAN.join();
					break;
				}
				case DEVICE_STATE_SEND:
				{
        logSerial("##Send## :");
				delay(200);
					// prepareTxFrame( appPort );
          long meassurementDelay = readSensorValues();
					sprintf(logMsg,"Die Messung dauerte %d ms" , meassurementDelay);
          logSerial(logMsg);
          generateDataPacket();
          boolean valid = generateLoraPacket();
          sprintf(logMsg,"sending packet \"%s\" , length %d\r\n",txpacket, strlen(txpacket));
          logSerial(logMsg);
					LoRaWAN.send();
          delay(1000);
          deviceState = DEVICE_STATE_CYCLE;
					break;
				}
				case DEVICE_STATE_CYCLE:
				{
					// Schedule next packet transmission
          logSerial("##Cycle## :");
					txDutyCycleTime = appTxDutyCycle + randr( 0, APP_TX_DUTYCYCLE_RND );
					LoRaWAN.cycle(txDutyCycleTime);
					deviceState = DEVICE_STATE_SLEEP;
					break;
				}
				case DEVICE_STATE_SLEEP:
				{
        //logSerial("bin im Sleep");
					LoRaWAN.sleep();
					break;
				}
				default:
				{
        logSerial("##Init## :");
					deviceState = DEVICE_STATE_INIT;
					break;
				}
			}
}
