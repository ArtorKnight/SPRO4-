/*
DISCLAIMER:
Indiviuals components were tested succesfully.

Description:
In this code we structure the software in functions that can be callend in the main loop.
Due to sensors measurements in order to avoid possible delays in the control loop was planned
to use dual core feature of the Rasperri Pico if need after full assembly and testing.
Being this a last time decision.

To do list:
* WRITE DATA IN SD (Did not arrive in time)
* CONTROL LOOP LOGIC (Unfinished)
*/

#include <HX711.h>            //MIT licence
#include <dht11.h>            //GPLv3 licence
#include <Wire.h>             //CC Share-Alike v4.0
#include <Adafruit_Sensor.h>  //BSD license
#include "Adafruit_BMP5xx.h"

dht11 DHT;            // Create DH11 object
Adafruit_BMP5xx bmp;  // Create BMP5XX object
HX711 loadcell;       // Create HX711 object
#define SEALEVELPRESSURE_HPA (1013.25)
#define DHT11_PIN 2  //undefined

#define BATTERY_VOUT_PIN 26  //GPIO 26
#define HX711_DOUT_PIN 21    //GPIO 21

#define HX711_SCK_PIN 1  //undefined
#define I2C_ADDR1 0x40   //adress encoder

bmp5xx_powermode_t desiredMode = BMP5XX_POWERMODE_STANDBY;  // Cache desired power mode

const float known_weight = 1000.0f;  // Weight to calibration of load cell
const float vref = 3.3f;             //max operational voltage reference
const float vmcu = 4095.0f;          //mcu adc bit reading
const float divider = 1.0f;          //voltage divider(need pcb skecth to adjust value)
const float gravity = 9.81f;
float degAngle;
float deltaAngle;
float correctedAngle;
float startAngle;

float rpm = 0;
float lastAngle = 0;
unsigned long lastTime = 0;

struct myData {
  float humid;  //humidity  (dont know)
  float temp1;  //temp int  (ºc)
  float temp2;  //temp ext  (ºc)
  float press;  //pressure  (hPa)
  float volts;  //battery life (not implemented yet)
  float force;  //tension (N)
};
myData data;
volatile bool readSlowSensors = false;  // flag fot interrupt reading
repeating_timer_t timer;
void setup() {
  add_repeating_timer_ms(
    5000,
    timerCallback,
    NULL,
    &timer);
  Wire.begin();
  Wire.setClock(400000);
  analogReadResolution(12);
  Serial.begin(115200);
  if (!bmp.begin(BMP5XX_DEFAULT_ADDRESS, &Wire)) {  //**! Default I2C address */#define BMP5XX_DEFAULT_ADDRESS (0x46)/**! Alternative I2C address */#define BMP5XX_ALTERNATIVE_ADDRESS (0x47)
    while (1) {
      Serial.println("Not found barometer");
      delay(10);
    }
  }
  loadcell.begin(HX711_DOUT_PIN, HX711_SCK_PIN);
  sensorConfig();
  calibration();
}

void loop() {

  if (readSlowSensors) {
    readSlowSensors = false;
    sens(data);
    Serial.print("H:");
    Serial.print(data.humid);

    Serial.print(" T1:");
    Serial.print(data.temp1);

    Serial.print(" T2:");
    Serial.print(data.temp2);

    Serial.print(" P:");
    Serial.print(data.press);

    Serial.print(" V:");
    Serial.print(data.volts);

    Serial.print(" F:");
    Serial.println(data.force);
  }
}
bool timerCallback(repeating_timer_t *rt) {
  readSlowSensors = true;
  return true;
}

void sens(myData &d) {  //humidity,temperatures and pressure values
  uint32_t sum = 0;     // sampling battery volt
  desiredMode = BMP5XX_POWERMODE_FORCED;
  bmp.setPowerMode(desiredMode);
  if (!bmp.performReading()) {
    return;
  }
  desiredMode = BMP5XX_POWERMODE_DEEP_STANDBY;
  bmp.setPowerMode(desiredMode);
  for (int i = 0; i < 16; i++) {
    sum += analogRead(BATTERY_VOUT_PIN);
  }
  float raw = sum / 16.0f;
  float voltage = raw * (vref / vmcu) * divider;
  DHT.read(DHT11_PIN);

  d.humid = DHT.humidity;
  d.temp1 = DHT.temperature;
  d.temp2 = bmp.temperature;
  d.press = bmp.pressure;  //i checked with the lib it return hPa no need conversion
  d.volts = voltage;
  // d.force remains untouched for the c.loop
}

void sensorConfig() {
  /* Temperature & Pressure  Oversampling Settings:
   * BMP5XX_OVERSAMPLING_1X   - 1x oversampling (fastest, least accurate)
   * BMP5XX_OVERSAMPLING_2X   - 2x oversampling  
   * BMP5XX_OVERSAMPLING_4X   - 4x oversampling
   * BMP5XX_OVERSAMPLING_8X   - 8x oversampling
   * BMP5XX_OVERSAMPLING_16X  - 16x oversampling  
   * BMP5XX_OVERSAMPLING_32X  - 32x oversampling
   * BMP5XX_OVERSAMPLING_64X  - 64x oversampling
   * BMP5XX_OVERSAMPLING_128X - 128x oversampling (slowest, most accurate)
   */
  bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_4X);
  bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_16X);  //16x oversampling  (recommended by Adafruit)


  /* IIR Filter Coefficient Settings:
   * BMP5XX_IIR_FILTER_BYPASS   - No filtering (fastest response)
   * BMP5XX_IIR_FILTER_COEFF_1  - Light filtering
   * BMP5XX_IIR_FILTER_COEFF_3  - Medium filtering
   * BMP5XX_IIR_FILTER_COEFF_7  - More filtering
   * BMP5XX_IIR_FILTER_COEFF_15 - Heavy filtering
   * BMP5XX_IIR_FILTER_COEFF_31 - Very heavy filtering
   * BMP5XX_IIR_FILTER_COEFF_63 - Maximum filtering
   * BMP5XX_IIR_FILTER_COEFF_127- Maximum filtering (slowest response)
   */
  bmp.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);
  /* Output Data Rate Settings (Hz):
   * BMP5XX_ODR_240_HZ, BMP5XX_ODR_218_5_HZ, BMP5XX_ODR_199_1_HZ
   * BMP5XX_ODR_179_2_HZ, BMP5XX_ODR_160_HZ, BMP5XX_ODR_149_3_HZ
   * BMP5XX_ODR_140_HZ, BMP5XX_ODR_129_8_HZ, BMP5XX_ODR_120_HZ
   * BMP5XX_ODR_110_1_HZ, BMP5XX_ODR_100_2_HZ, BMP5XX_ODR_89_6_HZ
   * BMP5XX_ODR_80_HZ, BMP5XX_ODR_70_HZ, BMP5XX_ODR_60_HZ, BMP5XX_ODR_50_HZ
   * BMP5XX_ODR_45_HZ, BMP5XX_ODR_40_HZ, BMP5XX_ODR_35_HZ, BMP5XX_ODR_30_HZ
   * BMP5XX_ODR_25_HZ, BMP5XX_ODR_20_HZ, BMP5XX_ODR_15_HZ, BMP5XX_ODR_10_HZ
   * BMP5XX_ODR_05_HZ, BMP5XX_ODR_04_HZ, BMP5XX_ODR_03_HZ, BMP5XX_ODR_02_HZ
   * BMP5XX_ODR_01_HZ, BMP5XX_ODR_0_5_HZ, BMP5XX_ODR_0_250_HZ, BMP5XX_ODR_0_125_HZ
   */
  bmp.setOutputDataRate(BMP5XX_ODR_50_HZ);
  /* Power Mode Settings:
   * BMP5XX_POWERMODE_STANDBY     - Standby mode (no measurements)
   * BMP5XX_POWERMODE_NORMAL      - Normal mode (periodic measurements)
   * BMP5XX_POWERMODE_FORCED      - Forced mode (single measurement then standby)
   * BMP5XX_POWERMODE_CONTINUOUS  - Continuous mode (fastest measurements)
   * BMP5XX_POWERMODE_DEEP_STANDBY - Deep standby (lowest power)
   */
  desiredMode = BMP5XX_POWERMODE_DEEP_STANDBY;
  bmp.setPowerMode(desiredMode);
}

void calibration() {
  loadcell.set_scale();
  loadcell.tare();
  //add know weight into the scale
  delay(9000);  //9 secs
  float reading = loadcell.get_units(10);
  //divide by your know weight and use that parameter in set_scale(parameter)
  float calibration_factor = reading / known_weight;
  loadcell.set_scale(calibration_factor);
  Serial.println(calibration_factor);
}
void encoderRead() {
  uint16_t rawData = 0;
  //--sending the command
  Wire.beginTransmission(I2C_ADDR1);
  Wire.write(0xFE);
  Wire.endTransmission(false);  // repeated start

  //--receiving the reading
  Wire.requestFrom(I2C_ADDR1, 2);

  if (Wire.available() >= 2) {
    uint8_t highB = Wire.read();
    uint8_t lowB = Wire.read();
    unsigned long now = millis();
    float deltaTime = (now - lastTime) / 1000.0f;
    if (deltaTime <= 0) return;

    rawData = ((uint16_t)highB << 6) | (lowB & 0x3F);  //removing the top 2 bits(non-important data)
    degAngle = (float)rawData * 360.0f / 16384.0f;     //conversion of the encoder data
    correctedAngle = degAngle - startAngle;            //tares angle
    if (correctedAngle < 0.0f) {
      correctedAngle += 360.0f;
    }
    deltaAngle = degAngle - lastAngle;
    // GET A RANGE -180 TO 180
    if (deltaAngle > 180.0f) {
      deltaAngle -= 360.0f;
    }

    if (deltaAngle < -180.0f) {
      deltaAngle += 360.0f;
    }
    rpm = (deltaAngle / 360.0f) * (60.0f / deltaTime);
    lastAngle = degAngle;
    lastTime = now;
  }
}
void loadRead(myData &d) {
  // 4. Acquire reading without blocking
  if (loadcell.wait_ready_timeout(1000)) {
    float reading = loadcell.get_units(10);
    d.force = reading * gravity / 1000.0f;
  } else {
    Serial.println("HX711 not found.");
  }
}


