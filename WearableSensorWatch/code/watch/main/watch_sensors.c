#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>


// Combine SENSORS
#include <stdio.h>
#include <stdlib.h>
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "sdkconfig.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "driver/uart.h"
#include "esp_vfs_dev.h"
#include <math.h>
#include "driver/i2c.h"
#include "./ADXL343.h"


#define EXAMPLE_WIFI_SSID CONFIG_WIFI_SSID
#define EXAMPLE_WIFI_PASS CONFIG_WIFI_PASSWORD

#ifdef CONFIG_EXAMPLE_IPV4
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
#else
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#endif

#define PORT CONFIG_EXAMPLE_PORT

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;

const int IPV4_GOTIP_BIT = BIT0;
const int IPV6_GOTIP_BIT = BIT1;

static const char *TAG = "Wearable Device";
char payload[128] = "Message from ESP32 ";
int msgBack = -1;
int extraCycles = 0;

// ------Combining SENSORS------
// For LED alerts
#define BLINK_BLUE 12
#define BLINK_RED 27

#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0
#define LEDC_HS_CH0_GPIO       (12)

#define LEDC_LS_TIMER          LEDC_TIMER_1
#define LEDC_LS_MODE           LEDC_LOW_SPEED_MODE

#define LEDC_TEST_DUTY         (4000)
#define LEDC_TEST_FADE_TIME    (3000)

// For thermistor
#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_channel_t bat_channel = ADC_CHANNEL_3;     //GPIO39 (A3)
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

float accel_X;
float accel_Y;
float accel_Z;
float battery;
float temp;
int steps = 0;
float battery_volt;

int water_delay = 20; // in seconds
int time_left = 20; // in seconds

// For vibrate
#define BOARD_LED 13

#define GPIO_OUTPUT_IO_0    18
#define GPIO_OUTPUT_IO_1    19
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_0) | (1ULL<<GPIO_OUTPUT_IO_1))
#define GPIO_INPUT_IO_0     4
#define GPIO_INPUT_IO_1     5
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1))
#define ESP_INTR_FLAG_DEFAULT 0

static xQueueHandle gpio_evt_queue = NULL;

bool tapped = false;

// for accelerometer
// Master I2C
#define I2C_EXAMPLE_MASTER_SCL_IO          22   // gpio number for i2c clk
#define I2C_EXAMPLE_MASTER_SDA_IO          23   // gpio number for i2c data
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_0  // i2c port
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000     // i2c master clock freq
#define WRITE_BIT                          I2C_MASTER_WRITE // i2c master write
#define READ_BIT                           I2C_MASTER_READ  // i2c master read
#define ACK_CHECK_EN                       true // i2c master will check ack
#define ACK_CHECK_DIS                      false// i2c master will not check ack
#define ACK_VAL                            0x00 // i2c ack value
#define NACK_VAL                           0xFF // i2c nack value

// ADXL343
#define SLAVE_ADDR                         ADXL343_ADDRESS // 0x53

// Function to initiate i2c -- note the MSB declaration!
static void i2c_master_init(){
  // Debug
  printf("\n>> i2c Config\n");
  int err;

  // Port configuration
  int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;

  /// Define I2C configurations
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;                              // Master mode
  conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;              // Default SDA pin
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
  conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;              // Default SCL pin
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
  conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;       // CLK frequency
  err = i2c_param_config(i2c_master_port, &conf);           // Configure
  if (err == ESP_OK) {printf("- parameters: ok\n");}

  // Install I2C driver
  err = i2c_driver_install(i2c_master_port, conf.mode,
                     I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                     I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
  if (err == ESP_OK) {printf("- initialized: yes\n");}

  // Data in MSB mode
  i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
}

// Utility  Functions //////////////////////////////////////////////////////////

// Utility function to test for I2C device address -- not used in deploy
int testConnection(uint8_t devAddr, int32_t timeout) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  int err = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return err;
}

// Utility function to scan for i2c device
static void i2c_scanner() {
  int32_t scanTimeout = 1000;
  printf("\n>> I2C scanning ..."  "\n");
  uint8_t count = 0;
  for (uint8_t i = 1; i < 127; i++) {
    // printf("0x%X%s",i,"\n");
    if (testConnection(i, scanTimeout) == ESP_OK) {
      printf( "- Device found at address: 0x%X%s", i, "\n");
      count++;
    }
  }
  if (count == 0) {printf("- No I2C devices found!" "\n");}
}

////////////////////////////////////////////////////////////////////////////////

// ADXL343 Functions ///////////////////////////////////////////////////////////

// Get Device ID
int getDeviceID(uint8_t *data) {
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, ADXL343_REG_DEVID, ACK_CHECK_EN);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, data, ACK_CHECK_DIS);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}

// Write one byte to register
int writeRegister(uint8_t reg, uint8_t data) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return 0;
}

// Read register
uint8_t readRegister(uint8_t reg, uint8_t* regData) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, regData, ACK_CHECK_DIS);
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return 0;
}

// read 16 bits (2 bytes)
int16_t read16(uint8_t reg, uint8_t *first, uint8_t *second) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, first, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, second, ACK_CHECK_DIS);
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return 0;
}

void setRange(range_t range) {
  /* Red the data format register to preserve bits */
  uint8_t format;
  readRegister(ADXL343_REG_DATA_FORMAT, &format);

  /* Update the data rate */
  format &= ~0x0F;
  format |= range;

  /* Make sure that the FULL-RES bit is enabled for range scaling */
  format |= 0x08;

  /* Write the register back to the IC */
  writeRegister(ADXL343_REG_DATA_FORMAT, format);

}

range_t getRange(void) {
  /* Read the data format register to preserve bits */
  uint8_t regData;
  readRegister(ADXL343_REG_DATA_FORMAT, &regData);
  return (range_t)(regData & 0x03);
}

dataRate_t getDataRate(void) {
  uint8_t regData;
  readRegister(ADXL343_REG_BW_RATE, &regData);
  return (dataRate_t) (regData & 0x0F);
}

////////////////////////////////////////////////////////////////////////////////

// function to get acceleration
void getAccel(float * xp, float *yp, float *zp) {

  uint8_t regDataX0;
  uint8_t regDataY0;
  uint8_t regDataZ0;

  uint8_t regDataX1;
  uint8_t regDataY1;
  uint8_t regDataZ1;

  readRegister(ADXL343_REG_DATAX0, &regDataX0);
  readRegister(ADXL343_REG_DATAX1, &regDataX1);

  readRegister(ADXL343_REG_DATAY0, &regDataY0);
  readRegister(ADXL343_REG_DATAY1, &regDataY1);

  readRegister(ADXL343_REG_DATAZ0, &regDataZ0);
  readRegister(ADXL343_REG_DATAZ1, &regDataZ1);

  uint16_t dataX;
  uint16_t dataY;
  uint16_t dataZ;

  dataX = (regDataX0 | (regDataX1 << 8));
  dataY = (regDataY0 | (regDataY1 << 8));
  dataZ = (regDataZ0 | (regDataZ1 << 8));

  int16_t intX = dataX;
  int16_t intY = dataY;
  int16_t intZ = dataZ;

  // casting to float
  *xp = intX * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
  *yp = intY * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
  *zp = intZ * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;

  accel_X = *xp;
  accel_Y = *yp;
  accel_Z = *zp;
}

// Task to continuously poll acceleration and calculate roll and pitch
static void test_adxl343() {
  printf("\n>> Polling ADAXL343\n");
  while (1) {
    float xVal, yVal, zVal;
    getAccel(&xVal, &yVal, &zVal);
    vTaskDelay(500 / portTICK_RATE_MS);
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
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    adc_reading /= NO_OF_SAMPLES;
    //Convert adc_reading to voltage in mV
    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    battery_volt = voltage;
    battery_volt /= 1000;
    vTaskDelay(50);
  }
}

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

static void read_therm() {
  //Configure ADC
  if (unit == ADC_UNIT_1) {
      adc1_config_width(ADC_WIDTH_BIT_12);
      adc1_config_channel_atten(channel, atten);
  } else {
      adc2_config_channel_atten((adc2_channel_t)channel, atten);
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
            adc_reading += adc1_get_raw((adc1_channel_t)channel);
        } else {
            int raw;
            adc2_get_raw((adc2_channel_t)channel, ADC_WIDTH_BIT_12, &raw);
            adc_reading += raw;
        }
    }
    adc_reading /= NO_OF_SAMPLES;
    //Convert adc_reading to voltage in mV
    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    float vfloat = voltage;
    float res = 10000/(vfloat/1000) - 2000; // calculates resistance using voltage divider and voltage reading
    temp = get_temp(res);
    vTaskDelay(75);
  }
}

static void print_data() {
  int sample = 0;
  while(1) {
    sprintf(payload,"%d %.1f %d %.2f %.2f %.2f %.2f %d", sample, temp, steps, battery_volt, accel_X, accel_Y, accel_Z, time_left);
    printf("%s\n", payload);
    sample++;
    if (time_left == 0) {
      time_left = water_delay;
    } else {
      time_left--;
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_example(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY) && !tapped) {
            tapped = true;
            gpio_set_level(BOARD_LED, 1);
            steps++; // increments universal step counter
        }
    }
}

static void debounce() {
  // resets value of tapped and turns off LED every 0.75 seconds
  while(1) {
    tapped = false;
    gpio_set_level(BOARD_LED, 0);
    vTaskDelay(75);
  }
}

static void vibrate_config() {
  printf("Waiting...\n");
  gpio_config_t io_conf;
  //disable interrupt
  io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
  //set as output mode
  io_conf.mode = GPIO_MODE_OUTPUT;
  //bit mask of the pins that you want to set,e.g.GPIO18/19
  io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
  //disable pull-down mode
  io_conf.pull_down_en = 0;
  //disable pull-up mode
  io_conf.pull_up_en = 0;
  //configure GPIO with the given settings
  gpio_config(&io_conf);

  //interrupt of rising edge
  io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
  //bit mask of the pins, use GPIO4/5 here
  io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
  //set as input mode
  io_conf.mode = GPIO_MODE_INPUT;
  //enable pull-up mode
  io_conf.pull_up_en = 1;
  gpio_config(&io_conf);

  //change gpio intrrupt type for one pin
  gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_ANYEDGE);

  //create a queue to handle gpio event from isr
  gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

  //install gpio isr service
  gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
  //hook isr handler for specific gpio pin
  gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
  //hook isr handler for specific gpio pin
  gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);

  //remove isr handler for gpio number.
  gpio_isr_handler_remove(GPIO_INPUT_IO_0);
  //hook isr handler for specific gpio pin again
  gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
}

static void water_alert() {
  // Configure LED
  gpio_pad_select_gpio(BLINK_BLUE);
  gpio_set_direction(BLINK_BLUE, GPIO_MODE_OUTPUT);

  /* Install UART driver for interrupt-driven reads and writes */
  ESP_ERROR_CHECK( uart_driver_install(UART_NUM_0,
    256, 0, 0, NULL, 0) );

  /* Tell VFS to use UART driver */
  esp_vfs_dev_uart_use_driver(UART_NUM_0);

  ledc_timer_config_t ledc_timer = {
      .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
      .freq_hz = 5000,                      // frequency of PWM signal
      .speed_mode = LEDC_HS_MODE,           // timer mode
      .timer_num = LEDC_HS_TIMER,            // timer index
      .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
  };
  // Set configuration of timer0 for high speed channels
  ledc_timer_config(&ledc_timer);

  // Prepare and set configuration of timer1 for low speed channels
  ledc_timer.speed_mode = LEDC_LS_MODE;
  ledc_timer.timer_num = LEDC_LS_TIMER;
  ledc_timer_config(&ledc_timer);

  ledc_channel_config_t ledc_channel = {
          .channel    = LEDC_HS_CH0_CHANNEL,
          .duty       = 0,
          .gpio_num   = LEDC_HS_CH0_GPIO,
          .speed_mode = LEDC_HS_MODE,
          .hpoint     = 0,
          .timer_sel  = LEDC_HS_TIMER
  };

  ledc_channel_config(&ledc_channel);

  // Initialize fade service.
  ledc_fade_func_install(0);

  while(1) {
    // flashes rapidly
    if (time_left == 0) {
      for (int i = 0; i < 5; i++) {
        ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, 5000);
        ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
        time_left = 0;
          vTaskDelay(25);
        time_left = 0;
        ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, 0);
        ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
        time_left = 0;
        vTaskDelay(25);
        time_left = 0;
      }
      time_left = water_delay;
    } else {
      vTaskDelay(50);
    }
  }
}


static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_START");
        break;
    case SYSTEM_EVENT_STA_CONNECTED:
        /* enable ipv6 */
        tcpip_adapter_create_ip6_linklocal(TCPIP_ADAPTER_IF_STA);
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, IPV4_GOTIP_BIT);
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_GOT_IP");
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently auto-reassociate. */
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, IPV4_GOTIP_BIT);
        xEventGroupClearBits(wifi_event_group, IPV6_GOTIP_BIT);
        break;
    case SYSTEM_EVENT_AP_STA_GOT_IP6:
        xEventGroupSetBits(wifi_event_group, IPV6_GOTIP_BIT);
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_GOT_IP6");

        char *ip6 = ip6addr_ntoa(&event->event_info.got_ip6.ip6_info.ip);
        ESP_LOGI(TAG, "IPv6: %s", ip6);
    default:
        break;
    }
    return ESP_OK;
}

static void initialise_wifi(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_WIFI_SSID,
            .password = EXAMPLE_WIFI_PASS,
        },
    };
    ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}

static void wait_for_ip()
{
    uint32_t bits = IPV4_GOTIP_BIT | IPV6_GOTIP_BIT ;

    ESP_LOGI(TAG, "Waiting for AP connection...");
    xEventGroupWaitBits(wifi_event_group, bits, false, true, portMAX_DELAY);
    ESP_LOGI(TAG, "Connected to AP");

    // Blink LED when connected
    gpio_pad_select_gpio(BLINK_RED);
    gpio_set_direction(BLINK_RED, GPIO_MODE_OUTPUT);
    gpio_set_level(BLINK_RED, 1);
    vTaskDelay(200);
    gpio_set_level(BLINK_RED, 0);

}

static void udp_client_task(void *pvParameters)
{
    char rx_buffer[128];
    char addr_str[128];
    int addr_family;
    int ip_protocol;

    while (1) {

#ifdef CONFIG_EXAMPLE_IPV4
        struct sockaddr_in destAddr;
        destAddr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
        destAddr.sin_family = AF_INET;
        destAddr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);
#else // IPV6
        struct sockaddr_in6 destAddr;
        inet6_aton(HOST_IP_ADDR, &destAddr.sin6_addr);
        destAddr.sin6_family = AF_INET6;
        destAddr.sin6_port = htons(PORT);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
        inet6_ntoa_r(destAddr.sin6_addr, addr_str, sizeof(addr_str) - 1);
#endif

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

        while (1) {

            int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&destAddr, sizeof(destAddr));
            if (err < 0) {
                ESP_LOGE(TAG, "Error occured during sending: errno %d", errno);
                break;
            }
            ESP_LOGI(TAG, "%s Message sent", payload);

            struct sockaddr_in sourceAddr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(sourceAddr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&sourceAddr, &socklen);

            // Error occured during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                if(strcmp(rx_buffer,"Ok!") == 0){
                  ESP_LOGI(TAG, "Scheduled water break, %s", rx_buffer);
                }else{
                  msgBack = atoi(rx_buffer);
                  time_left = msgBack;
                  ESP_LOGI(TAG, "New alarm: %d", msgBack);
                  msgBack = -1;
                }
            }
            vTaskDelay(1000 / portTICK_PERIOD_MS);

        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

void app_main()
{
    ESP_ERROR_CHECK( nvs_flash_init() );
    initialise_wifi();
    wait_for_ip();

    // ------Combining SENSORS
    // Routine
    i2c_master_init();
    i2c_scanner();

    // Check for ADXL343
    uint8_t deviceID;
    getDeviceID(&deviceID);
    if (deviceID == 0xE5) {
      printf("\n>> Found ADAXL343\n");
    }

    // Disable interrupts
    writeRegister(ADXL343_REG_INT_ENABLE, 0);

    // Set range
    setRange(ADXL343_RANGE_16_G);
    // Display range
    printf  ("- Range:         +/- ");
    switch(getRange()) {
      case ADXL343_RANGE_16_G:
        printf  ("16 ");
        break;
      case ADXL343_RANGE_8_G:
        printf  ("8 ");
        break;
      case ADXL343_RANGE_4_G:
        printf  ("4 ");
        break;
      case ADXL343_RANGE_2_G:
        printf  ("2 ");
        break;
      default:
        printf  ("?? ");
        break;
    }
    printf(" g\n");

    // Display data rate
    printf ("- Data Rate:    ");
    switch(getDataRate()) {
      case ADXL343_DATARATE_3200_HZ:
        printf  ("3200 ");
        break;
      case ADXL343_DATARATE_1600_HZ:
        printf  ("1600 ");
        break;
      case ADXL343_DATARATE_800_HZ:
        printf  ("800 ");
        break;
      case ADXL343_DATARATE_400_HZ:
        printf  ("400 ");
        break;
      case ADXL343_DATARATE_200_HZ:
        printf  ("200 ");
        break;
      case ADXL343_DATARATE_100_HZ:
        printf  ("100 ");
        break;
      case ADXL343_DATARATE_50_HZ:
        printf  ("50 ");
        break;
      case ADXL343_DATARATE_25_HZ:
        printf  ("25 ");
        break;
      case ADXL343_DATARATE_12_5_HZ:
        printf  ("12.5 ");
        break;
      case ADXL343_DATARATE_6_25HZ:
        printf  ("6.25 ");
        break;
      case ADXL343_DATARATE_3_13_HZ:
        printf  ("3.13 ");
        break;
      case ADXL343_DATARATE_1_56_HZ:
        printf  ("1.56 ");
        break;
      case ADXL343_DATARATE_0_78_HZ:
        printf  ("0.78 ");
        break;
      case ADXL343_DATARATE_0_39_HZ:
        printf  ("0.39 ");
        break;
      case ADXL343_DATARATE_0_20_HZ:
        printf  ("0.20 ");
        break;
      case ADXL343_DATARATE_0_10_HZ:
        printf  ("0.10 ");
        break;
      default:
        printf  ("???? ");
        break;
    }
    printf(" Hz\n\n");

    // Enable measurements
    writeRegister(ADXL343_REG_POWER_CTL, 0x08);

    gpio_pad_select_gpio(BOARD_LED);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BOARD_LED, GPIO_MODE_OUTPUT);

    vibrate_config();

    //start gpio task and debounce task
    xTaskCreate(test_adxl343,"test_adxl343", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(gpio_task_example, "gpio_task_example", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(debounce, "debounce", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(read_therm, "read_therm", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(read_battery, "read_battery", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(water_alert, "water_alert", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(print_data,"print_data", 1024*2, NULL, configMAX_PRIORITIES, NULL);

    xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);
}
