#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <math.h>
#include "driver/i2c.h"
#include <string.h>
#include "driver/uart.h"
#include "esp_vfs_dev.h"
#include "sdkconfig.h"

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t ir_channel = ADC_CHANNEL_6;      //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_channel_t ult_channel = ADC_CHANNEL_3;     //GPIO39 (A3)
static const adc_channel_t bat_channel = ADC_CHANNEL_5;     //GPIO33 (33)
static const adc_channel_t therm_channel = ADC_CHANNEL_0;   // GPI36 (A4)
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

float ir_dist;
float ult_dist;
float battery_volt;
float temp;

static float get_temp(float resistance) {
  if (resistance > 31000) {
    return 0;
  } else if (resistance > 28000) {
    return 2;
  }else if (resistance > 25000) {
    return 5;
  } else if (resistance > 21000) {
    return 8;
  } else if (resistance > 18500) {
    return 11;
  } else if (resistance > 16000) {
    return 14;
  } else if (resistance > 14500) {
    return 17;
  } else if (resistance > 12000) {
    return 20;
  } else if (resistance > 10500) {
    return 23;
  } else if (resistance > 10000) {
    return 24;
  } else if (resistance > 9000) {
    return 25;
  } else if (resistance > 8300) {
    return 28;
  } else if (resistance > 7400) {
    return 31;
  } else if (resistance > 6600) {
    return 34;
  } else if (resistance > 5900) {
    return 37;
  } else if (resistance > 5200) {
    return 40;
  } else if (resistance > 4600) {
    return 43;
  } else {
    return 50;
  }
}

// Calculates distance for IR sensor (From Erin's skill)
static float get_distance(int voltage) {
  float distance = 0;

  if (voltage > 1500)
      distance = 0;
  else if (voltage <= 1500 && voltage > 1200)
      distance = 20;
  else if (voltage <= 1200 && voltage > 1000)
      distance = 30;
  else if (voltage <= 1000 && voltage > 700)
      distance = 40;
  else if (voltage <= 700 && voltage > 600)
      distance = 50;
  else if (voltage <= 600 && voltage > 500)
      distance = 60;
  else if (voltage <= 500 && voltage > 450)
      distance = 70;
  else if (voltage <= 450 && voltage > 400)
      distance = 80;
  else if (voltage <= 400 && voltage > 375)
      distance = 90;
  else if (voltage <= 375 && voltage > 350)
      distance = 100;
  else if (voltage <= 350 && voltage > 325)
      distance = 110;
  else if (voltage <= 325 && voltage > 300)
      distance = 120;
  else if (voltage <= 300 && voltage > 275)
      distance = 130;
  else if (voltage <= 275 && voltage > 250)
      distance = 140;
  else if (voltage < 250)
      distance = 150;

  return distance;
}

static void read_therm() {
  //Configure ADC
  if (unit == ADC_UNIT_1) {
      adc1_config_width(ADC_WIDTH_BIT_12);
      adc1_config_channel_atten(therm_channel, atten);
  } else {
      adc2_config_channel_atten((adc2_channel_t)therm_channel, atten);
  }

  //Characterize ADC
  adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);

  //Continuously sample ADC1
  while (1) {
    uint32_t adc_reading = 0;
    //Multisampling
    for (int i = 0; i < NO_OF_SAMPLES; i++) {
        if (unit == ADC_UNIT_1) {
            adc_reading += adc1_get_raw((adc1_channel_t)therm_channel);
        } else {
            int raw;
            adc2_get_raw((adc2_channel_t)therm_channel, ADC_WIDTH_BIT_12, &raw);
            adc_reading += raw;
        }
    }
    adc_reading /= NO_OF_SAMPLES;
    //Convert adc_reading to voltage in mV
    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    float vfloat = voltage;
    float res = 10000/(vfloat/1000) - 2000; // calculates resistance using voltage divider and voltage reading
    temp = get_temp(res);
  }
}

static void read_battery() {
  //Configure ADC
  if (unit == ADC_UNIT_1) {
      adc1_config_width(ADC_WIDTH_BIT_12);
      adc1_config_channel_atten(bat_channel, atten);
  } else {
      adc2_config_channel_atten((adc2_channel_t)bat_channel, atten);
  }

  //Characterize ADC
  adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);

  //Continuously sample ADC1
  while (1) {
    uint32_t adc_reading = 0;
    //Multisampling
    for (int i = 0; i < NO_OF_SAMPLES; i++) {
        if (unit == ADC_UNIT_1) {
            adc_reading += adc1_get_raw((adc1_channel_t)bat_channel);
        } else {
            int raw;
            adc2_get_raw((adc2_channel_t)bat_channel, ADC_WIDTH_BIT_12, &raw);
            adc_reading += raw;
        }
    }
    adc_reading /= NO_OF_SAMPLES;
    //Convert adc_reading to voltage in mV
    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    battery_volt = voltage;
  }
}

static void read_ultra() {
  //Configure ADC
  if (unit == ADC_UNIT_1) {
      adc1_config_width(ADC_WIDTH_BIT_10);
      adc1_config_channel_atten(ult_channel, atten);
  } else {
      adc2_config_channel_atten((adc2_channel_t)ult_channel, atten);
  }

  //Characterize ADC
  adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);

  //Continuously sample ADC1
  while (1) {
    uint32_t adc_reading = 0;
    //Multisampling
    for (int i = 0; i < NO_OF_SAMPLES; i++) {
        if (unit == ADC_UNIT_1) {
            adc_reading += adc1_get_raw((adc1_channel_t)ult_channel);
        } else {
            int raw;
            adc2_get_raw((adc2_channel_t)ult_channel, ADC_WIDTH_BIT_12, &raw);
            adc_reading += raw;
        }
    }
    adc_reading /= NO_OF_SAMPLES;

    ult_dist = (adc_reading * 5 + 300); // Gets distance
    ult_dist /= 1000; // Converts from mm to m
  }
}

static void read_ir() {
  //Configure ADC
  if (unit == ADC_UNIT_1) {
      adc1_config_width(ADC_WIDTH_BIT_12);
      adc1_config_channel_atten(ir_channel, atten);
  } else {
      adc2_config_channel_atten((adc2_channel_t)ir_channel, atten);
  }

  //Characterize ADC
  adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);

  //Continuously sample ADC1
  while (1) {
      uint32_t adc_reading = 0;
      //Multisampling
      for (int i = 0; i < NO_OF_SAMPLES; i++) {
          if (unit == ADC_UNIT_1) {
              adc_reading += adc1_get_raw((adc1_channel_t)ir_channel);
          } else {
              int raw;
              adc2_get_raw((adc2_channel_t)ir_channel, ADC_WIDTH_BIT_12, &raw);
              adc_reading += raw;
          }
      }
      adc_reading /= NO_OF_SAMPLES;
      //Convert adc_reading to voltage in mV
      uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
      ir_dist = get_distance(voltage);
      ir_dist /= 100;
  }
}

static void print_data() {
  int sample = 0;
  while(1) {
    printf("%d %.2f %.2f %.1f %.1f\n\n", sample, ir_dist, ult_dist, battery_volt, temp);
    sample++;
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void app_main(void)
{
    xTaskCreate(read_ir,"read_ir", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(read_ultra,"read_ultra", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(read_battery,"read_battery", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(read_therm,"read_therm", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(print_data,"print_data", 1024*2, NULL, configMAX_PRIORITIES, NULL);
}
