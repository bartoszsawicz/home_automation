// Enable debug prints
#define MY_DEBUG
//#define MY_DEBUG_VERBOSE_SIGNING 

// Enable and select radio type attached
#define MY_RADIO_NRF24
#define MY_REPEATER_FEATURE
#define MY_RF24_PA_LEVEL RF24_PA_MAX
#define MY_SIGNING_SOFT
#define MY_SIGNING_REQUEST_SIGNATURES

#include <SPI.h>
#include <MySensors.h>
#include <Bounce2.h>
#include <DHT.h>

// Set this to the pin you connected the DHT's data pin to
#define DHT_DATA_PIN 3

#define PIN_FIRST_DOOR 4
#define NUMBER_OF_DOORS 1
#define CHILD_ID_FIST_DOOR 2
#define CHILD_ID_HEARTBEAT 99

// Set this offset if the sensor has a permanent small offset to the real temperatures
#define SENSOR_TEMP_OFFSET 0

// Sleep time between sensor updates (in milliseconds)
// Must be >1000ms for DHT22 and >2000ms for DHT11
static const uint64_t UPDATE_INTERVAL = 60000;
static const uint64_t HEARTBEAT_INTERVAL = 120000;
static const uint64_t MAX_RETRIES = 100;

// Force sending an update of the temperature after n sensor reads, so a controller showing the
// timestamp of the last update doesn't show something like 3 hours in the unlikely case, that
// the value didn't change since;
// i.e. the sensor would force sending an update every UPDATE_INTERVAL*FORCE_UPDATE_N_READS [ms]
static const uint8_t FORCE_UPDATE_N_READS = 10;

#define CHILD_ID_HUM 0
#define CHILD_ID_TEMP 1

float lastTemp;
float lastHum;
uint8_t nNoUpdatesTemp;
uint8_t nNoUpdatesHum;
bool metric = true;

int doorOpen[NUMBER_OF_DOORS];

long lastTemperatureUpdateMilis = 0;
long lastHeartbeatTime = 0;
bool heartbeatState = false;
int retryCount = 0;

MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage doorMsgs[NUMBER_OF_DOORS];
MyMessage msgHeartbeat(CHILD_ID_HEARTBEAT, V_TRIPPED);
DHT dht;

Bounce doorDebouncers[NUMBER_OF_DOORS];

void presentation()
{
  // Send the sketch version information to the gateway
  sendSketchInfo("Bedroom Sensor Node", "1.0");

  // Register all sensors to gw (they will be created as child devices)
  present(CHILD_ID_HUM, S_HUM);
  present(CHILD_ID_TEMP, S_TEMP);

  for (int sensor = 0; sensor < NUMBER_OF_DOORS; sensor++) {
    int child_id = sensor + CHILD_ID_FIST_DOOR;
    present(child_id, S_DOOR);
  }
  present(CHILD_ID_HEARTBEAT, S_DOOR);
  metric = getControllerConfig().isMetric;
}


void setup()
{
  dht.setup(DHT_DATA_PIN); // set data pin of DHT sensor
  if (UPDATE_INTERVAL <= dht.getMinimumSamplingPeriod()) {
    Serial.println("Warning: UPDATE_INTERVAL is smaller than supported by the sensor!");
  }

  // DOOR PIN SETUP
  for (int sensor = 0; sensor < NUMBER_OF_DOORS; sensor++) {
    doorOpen[sensor] = -1;
    int pin = sensor + PIN_FIRST_DOOR;
    int child_id = sensor + CHILD_ID_FIST_DOOR;
    // Then set relay pins in output mode
    pinMode(pin, INPUT);
    digitalWrite(pin, HIGH); //pull up resistor
    doorMsgs[sensor] = MyMessage(child_id, V_TRIPPED);
    doorDebouncers[sensor] = Bounce();
    // After setting up the button, setup debouncer
    doorDebouncers[sensor].attach(pin);
    doorDebouncers[sensor].interval(5);
  }

  // Sleep for the time of the minimum sampling period to give the sensor time to power up
  // (otherwise, timeout errors might occure for the first reading)
  sleep(dht.getMinimumSamplingPeriod());
}


void loop()
{
  if (shouldUpdateTemperature()) {
    updateTemperature();
  }
  if (shouldSendHeartbeat()) {
    sendHeartBeat();
  }

  for (int i = 0; i < NUMBER_OF_DOORS; i++) {
    doorDebouncers[i].update();
    int currentRead = doorDebouncers[i].read();
    if (currentRead != doorOpen[i]) {
      bool result = send(doorMsgs[i].set(currentRead == HIGH ? 1 : 0), true);
      if (result || retryCount > MAX_RETRIES) {
        doorOpen[i] = currentRead;
        retryCount = 0;
      } else {
        retryCount++;
      }
    }
  }
}

bool shouldUpdateTemperature() {
  return millis() > lastTemperatureUpdateMilis + UPDATE_INTERVAL;
}

bool shouldSendHeartbeat() {
  return millis() > lastHeartbeatTime + HEARTBEAT_INTERVAL;
}

void sendHeartBeat() {
  bool result = send(msgHeartbeat.set(heartbeatState), true);    
  if (result) {
    heartbeatState = !heartbeatState;
  }
  lastHeartbeatTime = millis();
}

void updateTemperature() {
  // Force reading sensor, so it works also after sleep()
  dht.readSensor(true);

  // Get temperature from DHT library
  float temperature = dht.getTemperature();
  if (isnan(temperature)) {
    Serial.println("Failed reading temperature from DHT!");
  } else if (temperature != lastTemp || nNoUpdatesTemp == FORCE_UPDATE_N_READS) {
    // Only send temperature if it changed since the last measurement or if we didn't send an update for n times
    lastTemp = temperature;
    if (!metric) {
      temperature = dht.toFahrenheit(temperature);
    }
    // Reset no updates counter
    nNoUpdatesTemp = 0;
    temperature += SENSOR_TEMP_OFFSET;
    send(msgTemp.set(temperature, 1));

#ifdef MY_DEBUG
    Serial.print("T: ");
    Serial.println(temperature);
#endif
  } else {
    // Increase no update counter if the temperature stayed the same
    nNoUpdatesTemp++;
  }

  // Get humidity from DHT library
  float humidity = dht.getHumidity();
  if (isnan(humidity)) {
    Serial.println("Failed reading humidity from DHT");
  } else if (humidity != lastHum || nNoUpdatesHum == FORCE_UPDATE_N_READS) {
    // Only send humidity if it changed since the last measurement or if we didn't send an update for n times
    lastHum = humidity;
    // Reset no updates counter
    nNoUpdatesHum = 0;
    send(msgHum.set(humidity, 1));

#ifdef MY_DEBUG
    Serial.print("H: ");
    Serial.println(humidity);
#endif
  } else {
    // Increase no update counter if the humidity stayed the same
    nNoUpdatesHum++;
  }

  lastTemperatureUpdateMilis = millis();
}

