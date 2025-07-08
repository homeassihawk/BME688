/*
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
TODO:
v0.3
- Figure out how to read real sensor data ✓
- Figure out how to have same sensor array like in sensorList[] for our mock sensors 
  -> not needed, just a few sensors for the data we need ✓
- Write sensor data from real sensors to virtual sensors ✓
- Update virtual sensor state to keep HA up-to-date ✓

v0.4
- Only update virtual sensor when data changed ✓
- Adjust polling from 1 second (don't need so many data points) ✓
- Figure out if BSEC adjusts IAQ level ✓ -> runtime increases IAQ accuracy
- Figure out why temperature is off by 4 degrees ✓ -> need calibration because sensor heating
- Not sure if we need to subscribe to all sensors, when we only read one ✓ -> we subscribe to sensor data not all sensors.
*/


////////////////////////////////////////////////
///                 EXPLANATION              ///
////////////////////////////////////////////////
/*
Home-Assistant with Mosquitto Add-On acts as a MQTT Broker. 
ESP32 Feather board logs into MQTT Broker with username and password defined via 
Users in Home-Assistant.
Then ESP32 updates sensor data to which the Home-Assistant is subscribed.

Data handled by the ESP32 : 
  Bsec2 envSensor[NUM_OF_SENS];
    An array of 8 physical BME688 Sensors.
    Each Bsec2 object handles one sensor.
    Calling .run() on each sensor and then retrieving outputs via getOutputs()

Indoor-air-quality (IAQ) gives an indication of the relative change in ambient TVOCs detected by BME68x. 
      
  The IAQ scale ranges from 0 (clean air) to 500 (heavily polluted air). During operation, algorithms 
  automatically calibrate and adapt themselves to the typical environments where the sensor is operated 
  (e.g., home, workplace, inside a car, etc.).This automatic background calibration ensures that users experience 
  consistent IAQ performance. The calibration process considers the recent measurement history (typ. up to four 
  days) to ensure that IAQ=50 corresponds to typical good air and IAQ=200 indicates typical polluted air.

  NOT SURE IF THIS IS WORKING FOR US 
  
*/

// These headers are taken from a ha-mqtt-entities library example of a sensor that shows mock data in Home-Assistant
// #include "secrets.h" not important for our use-case but typically stores userdata and passwords
#include<Arduino.h>
#include<WiFi.h>
#include<PubSubClient.h>
#include<mat.h>
#include<HaMqttEntities.h>

// These headers are taken from the BSEC2 8x example
#include<bsec2.h>
#include<commMux\commMux.h>


////////////////////////////////////////////////
///           MQTT AND WIFI SETUP            ///
////////////////////////////////////////////////

#define WIFI_SSID     "nub022025"
#define WIFI_PASSWORD "nub022025"
#define MQTT_SERVER   "10.108.2.179"
#define MQTT_PORT     1883
#define MQTT_USER     "MQTTUser"
#define MQTT_PASSWORD "admin"

WiFiClient wifi_client;
PubSubClient mqtt_client(wifi_client);

////////////////////////////////////////////////
///           BME688 8x SETUP                ///
////////////////////////////////////////////////

#define NUM_OF_SENS   8
#define PANIC_LED     LED_BUILTIN
#define ERROR_DUR     1000

#define SAMPLE_RATE   BSEC_SAMPLE_RATE_ULP

/* Helper functions declerations */
void errLeds(void);
void checkBsecStatus(Bsec2 bsec);

/**
   @brief : This function is called by the BSEC library when a new output is available
   @param[in] input     : BME68X sensor data before processing
   @param[in] outputs   : Processed BSEC BSEC output data
   @param[in] bsec      : Instance of BSEC2 calling the callback
*/
void newDataCallback(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec);

// Create an array of objects of the class Bsec2 
Bsec2 envSensor[NUM_OF_SENS];
comm_mux communicationSetup[NUM_OF_SENS];
uint8_t bsecMemBlock[NUM_OF_SENS][BSEC_INSTANCE_SIZE];
uint8_t sensor = 0;


////////////////////////////////////////////////
///          HOMEASSISTANT PARTS             ///
////////////////////////////////////////////////

// HA Parts
#define ENTITIES_COUNT 2
#define HA_DEVICE_ID   "BME688_Implementation"
#define HA_DEVICE_FRIENDLY_NAME "BME688_Implementation"

#define UNIT_FOR_TEMP "°C"
#define UNIT_FOR_CO2 "ppm"
#define UNIT_FOR_PRESSURE "hPa"
#define UNIT_FOR_HUMIDITY "%"

#define PRECISION 2  // Number of decimals


////////////////////////////////////////////////
///             SENSORS FOR MQTT             ///
////////////////////////////////////////////////

HADevice ha_device = HADevice(HA_DEVICE_ID,HA_DEVICE_FRIENDLY_NAME,"1.0");

HASensorNumeric ha_sensor_temp = HASensorNumeric("sensor_temp", "Temperatur", ha_device, UNIT_FOR_TEMP,PRECISION);
HASensorNumeric ha_sensor_raw_humidity = HASensorNumeric("sensor_raw_humidity", "Feuchtigkeit", ha_device, UNIT_FOR_HUMIDITY, PRECISION);
HASensorNumeric ha_sensor_iaq = HASensorNumeric("sensor_iaq", "Air Quality", ha_device);


////////////////////////////////////////////////
///                 ESP32 SETUP              ///
////////////////////////////////////////////////

void setup() {

  // BSEC2 Setup
  /* Desired subscription list of BSEC2 outputs */
  bsecSensor sensorList[] = {
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_STABILIZATION_STATUS,
    BSEC_OUTPUT_RUN_IN_STATUS,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
    BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_GAS_PERCENTAGE,
    BSEC_OUTPUT_COMPENSATED_GAS
  };

  // Initialize the communication interfaces
  Serial.begin(115200);
  comm_mux_begin(Wire, SPI);
  pinMode(PANIC_LED, OUTPUT);
  delay(100);

  // Valid for boards with USB-COM. Wait until the port is open
  while (!Serial) delay(10);

  for (uint8_t i = 0; i < NUM_OF_SENS; i++)
  {
    /* Sets the Communication interface for the sensors */
    communicationSetup[i] = comm_mux_set_config(Wire, SPI, i, communicationSetup[i]);

    /* Assigning a chunk of memory block to the bsecInstance */
    envSensor[i].allocateMemory(bsecMemBlock[i]);

    /* Initialize the library and interfaces */
    if (!envSensor[i].begin(BME68X_SPI_INTF, comm_mux_read, comm_mux_write, comm_mux_delay, &communicationSetup[i]))
    {
      checkBsecStatus (envSensor[i]);
    }
	
	/*
	 *	The default offset provided has been determined by testing the sensor in LP and ULP mode on application board 3.0
	 *	Please update the offset value after testing this on your product 
	 */
	if (SAMPLE_RATE == BSEC_SAMPLE_RATE_ULP)
	{
		envSensor[i].setTemperatureOffset(6.5f);
	}
	else if (SAMPLE_RATE == BSEC_SAMPLE_RATE_LP)
	{
		envSensor[i].setTemperatureOffset(TEMP_OFFSET_LP);
	}
	
    /* Subscribe to the desired BSEC2 outputs */
    if (!envSensor[i].updateSubscription(sensorList, ARRAY_LEN(sensorList), SAMPLE_RATE))
    {
      checkBsecStatus (envSensor[i]);
    }

    /* Whenever new data is available call the newDataCallback function */
    envSensor[i].attachCallback(newDataCallback);
  }

  Serial.println("BSEC library version " + \
                 String(envSensor[0].version.major) + "." \
                 + String(envSensor[0].version.minor) + "." \
                 + String(envSensor[0].version.major_bugfix) + "." \
                 + String(envSensor[0].version.minor_bugfix));

  // MQTT Setup
    mqtt_client.setServer(MQTT_SERVER, MQTT_PORT);

    HAMQTT.begin(mqtt_client,ENTITIES_COUNT);
    HAMQTT.addEntity(ha_sensor_temp);
    HAMQTT.addEntity(ha_sensor_raw_humidity);
    HAMQTT.addEntity(ha_sensor_iaq);

  // WiFi Setup
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

}


////////////////////////////////////////////////
///                 ESP32 LOOP               ///
////////////////////////////////////////////////

/* 
1. Have MQTT Sensors equvivalent to the BME Sensors
2. Read real sensor data
3. Update mock sensor state like Mock but with actual sensor data
*/

void loop() {
    HAMQTT.loop();

    if(!HAMQTT.connected() && !HAMQTT.connect(HA_DEVICE_ID,MQTT_USER,MQTT_PASSWORD)) 
    {
      Serial.println("HAMQTT not connected");
      delay(1000);
    }

    if (!envSensor[sensor].run())
    {
      checkBsecStatus(envSensor[sensor]);
    }
}


////////////////////////////////////////////////
///                 FUNCTIONS                ///
////////////////////////////////////////////////

void errLeds(void)
{
  while (1)
  {
    digitalWrite(PANIC_LED, HIGH);
    delay(ERROR_DUR);
    digitalWrite(PANIC_LED, LOW);
    delay(ERROR_DUR);
  }
}

void checkBsecStatus(Bsec2 bsec)
{
  if (bsec.status < BSEC_OK)
  {
    Serial.println("BSEC error code : " + String(bsec.status));
    errLeds(); /* Halt in case of failure */
  }
  else if (bsec.status > BSEC_OK)
  {
    Serial.println("BSEC warning code : " + String(bsec.status));
  }

  if (bsec.sensor.status < BME68X_OK)
  {
    Serial.println("BME68X error code : " + String(bsec.sensor.status));
    errLeds(); /* Halt in case of failure */
  }
  else if (bsec.sensor.status > BME68X_OK)
  {
    Serial.println("BME68X warning code : " + String(bsec.sensor.status));
  }
}

void newDataCallback(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec)
{
  if (!outputs.nOutputs)
  {
    return;
  }

  Serial.println("BSEC outputs:\n\tSensor num = " + String(sensor));
  Serial.println("\tTime stamp = " + String((int) (outputs.output[0].time_stamp / INT64_C(1000000))));
  for (uint8_t i = 0; i < outputs.nOutputs; i++)
  {
    const bsecData output  = outputs.output[i];
    switch (output.sensor_id)
    {
      case BSEC_OUTPUT_IAQ:
        Serial.println("\tIAQ = " + String(output.signal));
        Serial.println("\tIAQ accuracy = " + String((int) output.accuracy));
        ha_sensor_iaq.setState(output.signal);
        break;
      case BSEC_OUTPUT_RAW_TEMPERATURE:
        Serial.println("\tTemperature = " + String(output.signal));
        break;
      case BSEC_OUTPUT_RAW_PRESSURE:
        Serial.println("\tPressure = " + String(output.signal));
        break;
      case BSEC_OUTPUT_RAW_HUMIDITY:
        Serial.println("\tHumidity = " + String(output.signal));
        ha_sensor_raw_humidity.setState(output.signal);
        break;
      case BSEC_OUTPUT_RAW_GAS:
        Serial.println("\tGas resistance = " + String(output.signal));
        break;
      case BSEC_OUTPUT_STABILIZATION_STATUS:
        Serial.println("\tStabilization status = " + String(output.signal));
        break;
      case BSEC_OUTPUT_RUN_IN_STATUS:
        Serial.println("\tRun in status = " + String(output.signal));
        break;
      case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
        Serial.println("\tCompensated temperature = " + String(output.signal));
        ha_sensor_temp.setState(output.signal);
        break;
      case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
        Serial.println("\tCompensated humidity = " + String(output.signal));
        break;
      case BSEC_OUTPUT_STATIC_IAQ:
        Serial.println("\tStatic IAQ = " + String(output.signal));
        break;
      case BSEC_OUTPUT_CO2_EQUIVALENT:
        Serial.println("\tCO2 Equivalent = " + String(output.signal));
        break;
      case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
        Serial.println("\tbVOC equivalent = " + String(output.signal));
        break;
      case BSEC_OUTPUT_GAS_PERCENTAGE:
        Serial.println("\tGas percentage = " + String(output.signal));
        break;
      case BSEC_OUTPUT_COMPENSATED_GAS:
        Serial.println("\tCompensated gas = " + String(output.signal));
        break;

      default:
        break;
    }
  }
}