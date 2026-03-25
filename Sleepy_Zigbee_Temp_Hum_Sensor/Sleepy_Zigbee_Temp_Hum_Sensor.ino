#ifndef ZIGBEE_MODE_ED
#error "Zigbee end device mode is not selected in Tools -> Zigbee mode"
#endif

#include "Zigbee.h"
#include "DFRobot_AHT20.h"
#include "driver/gpio.h"

/* --- Configuration --- */
#define TEMP_SENSOR_ENDPOINT_NUMBER 10
#define BAT_ADC_PIN 0
#define VOLTAGE_DIVIDER_RATIO 2.0  // Corrected for 1M/1M resistors
#define TIME_TO_SLEEP 60           // Check every 60s
#define MAX_REPORTS_TO_SKIP 60     // Force report every 60 mins (Heartbeat)
#define TEMP_THRESHOLD 0.5         // Only report if change > 0.5°C
#define HUM_THRESHOLD 2.0          // Only report if change > 2.0%
#define uS_TO_S_FACTOR 1000000ULL
#define MANUFACTURER "DFRobot"     // Manufacturer name
#define MODEL "Beetle-C6-Task"     // Model name

/* --- Persistent Memory (Survives Deep Sleep) --- */
RTC_DATA_ATTR float lastReportedTemp = -99.0;
RTC_DATA_ATTR float lastReportedHum = -99.0;
RTC_DATA_ATTR int reportCounter = 0;

/* --- Global Objects --- */
ZigbeeTempSensor zbTempSensor = ZigbeeTempSensor(TEMP_SENSOR_ENDPOINT_NUMBER);
DFRobot_AHT20 aht20;
volatile uint8_t dataToSend = 0;

/* --- Functions --- */

float readBatteryVoltage() {
  return (analogReadMilliVolts(BAT_ADC_PIN) / 1000.0) * VOLTAGE_DIVIDER_RATIO;
}

uint8_t calculatePercentage(float v) {
  if (v >= 4.1) return 100;
  if (v <= 3.2) return 0;
  return (uint8_t)((v - 3.2) / (4.1 - 3.2) * 100);
}

void onGlobalResponse(zb_cmd_type_t cmd, esp_zb_zcl_status_t status, uint8_t ep, uint16_t cluster) {
  if (status == ESP_ZB_ZCL_STATUS_SUCCESS && ep == TEMP_SENSOR_ENDPOINT_NUMBER) {
    if (dataToSend > 0) dataToSend--;
  }
}

void prepareSleepAndExit() {
  Wire.end();
  pinMode(6, INPUT_PULLUP); // Match 3.3V rail
  pinMode(7, INPUT_PULLUP);
  gpio_hold_en((gpio_num_t)6);
  gpio_hold_en((gpio_num_t)7);
  Serial.println("Entering Deep Sleep...");
  Serial.flush();
  esp_deep_sleep_start();
}

/* --- Main Logic Task --- */
void sensorTask(void *pv) {
  // Give sensor a moment to stabilize after I2C start
  vTaskDelay(pdMS_TO_TICKS(100));
  
  // 1. Measure Immediately (Radio is still OFF)
  if (aht20.startMeasurementReady(true)) {
    float currentTemp = aht20.getTemperature_C() - 1.0; // Offset
    float currentHum = aht20.getHumidity_RH();
    
    bool shouldReport = false;
    reportCounter++;

    // Check thresholds
    if (abs(currentTemp - lastReportedTemp) >= TEMP_THRESHOLD || 
        abs(currentHum - lastReportedHum) >= HUM_THRESHOLD ||
        reportCounter >= MAX_REPORTS_TO_SKIP) {
      shouldReport = true;
    }

    if (!shouldReport) {
      Serial.printf("No change (T:%.1f, H:%.1f). Skipping Zigbee.\n", currentTemp, currentHum);
      prepareSleepAndExit(); // GO TO SLEEP WITHOUT TURNING ON ZIGBEE
    }

    // 2. Turn on Zigbee only if needed
    Serial.println("Significant change detected. Connecting Zigbee...");
    esp_zb_cfg_t zbConfig = ZIGBEE_DEFAULT_ED_CONFIG();
    zbConfig.nwk_cfg.zed_cfg.keep_alive = 10000;
    
    if (Zigbee.begin(&zbConfig, false)) {
      unsigned long startJoin = millis();
      while (!Zigbee.connected() && (millis() - startJoin < 15000)) {
        vTaskDelay(pdMS_TO_TICKS(50));
      }

      if (Zigbee.connected()) {
        float vbat = readBatteryVoltage();
        uint8_t per = calculatePercentage(vbat);
        
        zbTempSensor.setTemperature(currentTemp);
        zbTempSensor.setHumidity(currentHum);
        zbTempSensor.setBatteryPercentage(per);
        zbTempSensor.setBatteryVoltage((uint8_t)(vbat * 10));

        dataToSend = 3; 
        zbTempSensor.report();
        zbTempSensor.reportBatteryPercentage();

        // Wait for ACKs
        unsigned long startWait = millis();
        while (dataToSend > 0 && (millis() - startWait < 3000)) {
          vTaskDelay(pdMS_TO_TICKS(50));
        }

        // Save state for next wake
        lastReportedTemp = currentTemp;
        lastReportedHum = currentHum;
        reportCounter = 0;
        Serial.println("Report successful.");
      }
    }
  }
  prepareSleepAndExit();
}

void setup() {// 1. MUST RELEASE PINS IMMEDIATELY
  gpio_hold_dis((gpio_num_t)6);
  gpio_hold_dis((gpio_num_t)7);
  
  Serial.begin(115200);
  
  // 2. Initialize I2C and Sensor
  Wire.begin(6, 7);
  if (aht20.begin() != 0) {
    Serial.println("AHT20 Fail. Checking bus...");
  }

  analogReadResolution(12);
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

  zbTempSensor.setManufacturerAndModel(MANUFACTURER, MODEL);
  zbTempSensor.addHumiditySensor(0, 100, 0.1);
  zbTempSensor.setPowerSource(ZB_POWER_SOURCE_BATTERY, 100, 36);
  
  Zigbee.onGlobalDefaultResponse(onGlobalResponse);
  Zigbee.addEndpoint(&zbTempSensor);

  xTaskCreate(sensorTask, "sens", 4096, NULL, 5, NULL);
}

void loop() { 
  vTaskDelay(pdMS_TO_TICKS(1000)); 
}
