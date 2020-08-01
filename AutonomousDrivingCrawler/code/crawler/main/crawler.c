#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#include "driver/i2c.h"

// IP chat
#include <string.h>
#include <sys/param.h>
//#include "freertos/FreeRTOS.h"
//#include "freertos/task.h"
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


//You can get these value from the datasheet of servo you use, in general pulse width varies between 1000 to 2000 mocrosecond
#define SERVO_MIN_PULSEWIDTH 650 //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2150 //Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 100 //Maximum angle in degree upto which servo can rotate

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

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

// Lidar
#define SLAVE_ADDR_2                       0x62          // Default I2C Address of LIDAR-Lite.
#define REGISTER                           0x00          // Register to write to initiate ranging.
#define VALUE                              0x04          // Value to initiate ranging.
#define HIGH_LOW                           0x8f          // Register to get both High and Low bytes in 1 call. // not working for unknown reason

// 14-Segment Display
#define SLAVE_ADDR                         0x70 // alphanumeric address
#define OSC                                0x21 // oscillator cmd
#define HT16K33_BLINK_DISPLAYON            0x01 // Display on cmd
#define HT16K33_BLINK_OFF                  0    // Blink off cmd
#define HT16K33_BLINK_CMD                  0x80 // Blink cmd
#define HT16K33_CMD_BRIGHTNESS             0xE0 // Brightness cmd

// IP chat
#define EXAMPLE_WIFI_SSID CONFIG_WIFI_SSID
#define EXAMPLE_WIFI_PASS CONFIG_WIFI_PASSWORD

#ifdef CONFIG_EXAMPLE_IPV4
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
#else
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#endif

#define PORT CONFIG_EXAMPLE_PORT


////////////////////////////////////////////////////////////////////////////////
// IP chat /////////////////////////////////////////////////////////////////////
/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;

const int IPV4_GOTIP_BIT = BIT0;
const int IPV6_GOTIP_BIT = BIT1;

static const char *TAG = "Auto Driving";
char payload[30] = "Message from ESP32 ";
int onoff = 0;

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
            ESP_LOGI(TAG, "Message sent");

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
                ESP_LOGI(TAG, "%s", rx_buffer);
                onoff = atoi(rx_buffer);
            }

            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

////////////////////////////////////////////////////////////////////////////////
// LIDAR and Alphanumeric //////////////////////////////////////////////////////

// display table consisting only numbers
uint16_t alphafonttable[] =  {
  0b0000110000111111, // 0
  0b0000000000000110, // 1
  0b0000000011011011, // 2
  0b0000000010001111, // 3
  0b0000000011100110, // 4
  0b0010000001101001, // 5
  0b0000000011111101, // 6
  0b0000000000000111, // 7
  0b0000000011111111, // 8
  0b0000000011101111, // 9
  0b0000000000000000  //
};

int distance = 0;

////////////////////////////////////////////////////////////////////////////////
// Crawler Control /////////////////////////////////////////////////////////////

// For speed
static esp_adc_cal_characteristics_t *speed_adc_chars;
static const adc_channel_t speed_channel = ADC_CHANNEL_6;   // A2
static const adc_atten_t speed_atten = ADC_ATTEN_DB_11;
static const adc_unit_t speed_unit = ADC_UNIT_1;

// For steering
static esp_adc_cal_characteristics_t *steer_adc_chars;
static const adc_channel_t steer_channel = ADC_CHANNEL_3;   // A3
static const adc_atten_t steer_atten = ADC_ATTEN_DB_11;
static const adc_unit_t steer_unit = ADC_UNIT_1;

int dt = 100;

int steerSetpoint = 50;
float speedSetpoint = 0.1;
int range;
float speed;

uint32_t speedpw = 1164;
int steerAngle = 54;
int status = 0;
int cali_done = 0;

bool pulsed = false;
int count;

void adc_task(void *arg)
{
  // Initialize counter
  count = 0;

  //Configure ADC
  if (speed_unit == ADC_UNIT_1) {
      adc1_config_width(ADC_WIDTH_BIT_12);
      adc1_config_channel_atten(speed_channel, speed_atten);
  } else {
      adc2_config_channel_atten((adc2_channel_t)speed_channel, speed_atten);
  }

  //Characterize ADC
  speed_adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_characterize(speed_unit, speed_atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, speed_adc_chars);

  //Continuously sample ADC1
  while (1) {
      uint32_t adc_reading = 0;
      //Multisampling
      for (int i = 0; i < NO_OF_SAMPLES; i++) {
          if (speed_unit == ADC_UNIT_1) {
              adc_reading += adc1_get_raw((adc1_channel_t)speed_channel);
          } else {
              int raw;
              adc2_get_raw((adc2_channel_t)speed_channel, ADC_WIDTH_BIT_12, &raw);
              adc_reading += raw;
          }
      }
      adc_reading /= NO_OF_SAMPLES;

      if (adc_reading < 4095 && pulsed == false)
      {
        count++;
        pulsed = true;
      } else if (adc_reading == 4095) {
        pulsed = false;
      }

      vTaskDelay(pdMS_TO_TICKS(5));
  }
}

static void timer() {
  while(1) {
    count = 0;
    vTaskDelay(100);
    speed = (count * (2*3.14159*5/8))/100; //0.005; //(count / 6) * 0.62;
    printf("Speed is %.1fm/s\n", speed);
    count = 0;
  }
}

static void speedPID() {

  float Kp = 0.2; //0.2;
  float Ki = 0.001; //0.001;
  float Kd = 0.08; //0.08;
  float previous_error = 0.00;
  float integral = 0.00;

  while(1)
  {
    float error = speedSetpoint - speed;

    if (error > 0) {
      printf("You are too close to the wall\n");
    } else if (error < 0) {
      printf("You are too far from the wall\n");
    } else if (error == 0) {
      printf("You are just right!\n");
      steerAngle = 54;
    }
    integral = error * dt;
    float derivative = (error - previous_error) / dt;
    float output = Kp * error + Ki * integral + Kd * derivative;
    previous_error = error;

    speedpw -= output;
    if (speedpw > 1215) {
       speedpw = 1215;
    } else if (speedpw < 1160) {
      speedpw = 1165;
    }
    vTaskDelay(dt);
  }
}

static void steerPID() {

  float Kp = 0.2;
  float Ki = 0.001;
  float Kd = 0.08;
  float previous_error = 0.00;
  float integral = 0.00;

  while(1)
  {
    float error = steerSetpoint - range;
    if (error > 0) {
      printf("You are too close to the wall\n");
    } else if (error < 0) {
      printf("You are too far from the wall\n");
    } else if (error == 0) {
      printf("You are just right!\n");
      steerAngle = 54;
    }
    integral = error * dt;
    float derivative = (error - previous_error) / dt;
    float output = Kp * error + Ki * integral + Kd * derivative;
    previous_error = error;
    // printf("-----------------\nPID found the following: \n\tError:\t\t%.0f \n\tIntegral:\t%.1f \n\tDerivative:\t%.1f \n\tOutput is: \t%.1f\n-----------------\n", error, integral, derivative, output);
    steerAngle -= output;
    if (steerAngle > 70) {
      steerAngle = 70;
    } else if (steerAngle < 30) {
      steerAngle = 30;
    }
    vTaskDelay(dt);
  }
}

static void mcpwm_example_gpio_initialize(void)
{
    printf("initializing mcpwm servo control gpio......\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 27);    //Set GPIO 27 as PWM0A, to which servo is connected
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, 12);    //Set GPIO 12 as PWM0B
}


static uint32_t servo_per_degree_init(uint32_t degree_of_rotation)
{
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
    return cal_pulsewidth;
}

void calibrateESC() {
  printf("----------------------------------------Turn on crawler in 4 sec: \n");
  vTaskDelay(4000 / portTICK_PERIOD_MS);  // Give yourself time to turn on crawler

  printf("Set to HIGH\n");
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 2100); // HIGH signal in microseconds
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  printf("Set to LOW\n");
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 700);  // LOW signal in microseconds
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  printf("Set to NEUTRAL\n");
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400); // NEUTRAL signal in microseconds
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  printf("Set to NEUTRAL! Calibration DONE\n");
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400); // reset the ESC to neutral (non-moving) value

  steerAngle = 54;
}

/**
 * @brief Configure MCPWM module
 */

void set_speed(){
    while(1){
      if(status && onoff && cali_done){

        if (speedpw > 1170 || speedpw < 1160){
          speedpw = 1164;
        }else if(speedpw % 2){
          speedpw++;
        }else{
          speedpw--;
        }

      } else {
        speedpw = 1215;
      }
      vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void mcpwm_servo_control(void *arg)
{
    uint32_t angle;
    //1. mcpwm gpio initialization
    mcpwm_example_gpio_initialize();

    //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm......\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings

    printf("Calibrating ESC......\n");
    calibrateESC();

    cali_done = 1;

    while (1) {
      printf("\nAngle of rotation: %d\n", steerAngle);
      angle = servo_per_degree_init(steerAngle);

      // printf("Steering servo pulse width: %dus\n", angle);
      printf("ESC pulse width: %dus\n\n", speedpw);

      mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, speedpw);
      mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, angle);
      vTaskDelay(500 / portTICK_PERIOD_MS);     //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
    }
}

static int getRange(int voltage) {
  int dis = 0;

  if (voltage > 1500)
      dis = 0;
  else if (voltage <= 1500 && voltage > 1200)
      dis = 20;
  else if (voltage <= 1200 && voltage > 1000)
      dis = 30;
  else if (voltage <= 1000 && voltage > 700)
      dis = 40;
  else if (voltage <= 700 && voltage > 600)
      dis = 50;
  else if (voltage <= 600 && voltage > 500)
      dis = 60;
  else if (voltage <= 500 && voltage > 450)
      dis = 70;
  else if (voltage <= 450 && voltage > 400)
      dis = 80;
  else if (voltage <= 400 && voltage > 375)
      dis = 90;
  else if (voltage <= 375 && voltage > 350)
      dis = 100;
  else if (voltage <= 350 && voltage > 325)
      dis = 110;
  else if (voltage <= 325 && voltage > 300)
      dis = 120;
  else if (voltage <= 300 && voltage > 275)
      dis = 130;
  else if (voltage <= 275 && voltage > 250)
      dis = 140;
  else if (voltage < 250)
      dis = 150;

  return dis;
}

static void read_ir() {
  //Configure ADC
  if (steer_unit == ADC_UNIT_1) {
      adc1_config_width(ADC_WIDTH_BIT_12);
      adc1_config_channel_atten(steer_channel, steer_atten);
  } else {
      adc2_config_channel_atten((adc2_channel_t)steer_channel, steer_atten);
  }

  //Characterize ADC
  steer_adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_characterize(steer_unit, steer_atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, steer_adc_chars);

  //Continuously sample ADC1
  uint32_t adc_reading = 0;

  //Multisampling
  for (int i = 0; i < NO_OF_SAMPLES; i++) {
      if (steer_unit == ADC_UNIT_1) {
          adc_reading += adc1_get_raw((adc1_channel_t)steer_channel);
      } else {
          int raw;
          adc2_get_raw((adc2_channel_t)steer_channel, ADC_WIDTH_BIT_12, &raw);
          adc_reading += raw;
      }
  }
  adc_reading /= NO_OF_SAMPLES;
  //Convert adc_reading to voltage in mV
  uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, steer_adc_chars);
  range = getRange(voltage);
  steerSetpoint = range; // initializes value for distance from wall

  while (1) {
      uint32_t adc_reading = 0;
      //Multisampling
      for (int i = 0; i < NO_OF_SAMPLES; i++) {
          if (steer_unit == ADC_UNIT_1) {
              adc_reading += adc1_get_raw((adc1_channel_t)steer_channel);
          } else {
              int raw;
              adc2_get_raw((adc2_channel_t)steer_channel, ADC_WIDTH_BIT_12, &raw);
              adc_reading += raw;
          }
      }
      adc_reading /= NO_OF_SAMPLES;
      //Convert adc_reading to voltage in mV
      uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, steer_adc_chars);
      range = getRange(voltage);
      vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

////////////////////////////////////////////////////////////////////////////////
// I2C /////////////////////////////////////////////////////////////////////////

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
  uint8_t count_i2c = 0;
  for (uint8_t i = 1; i < 127; i++) {
    // printf("0x%X%s",i,"\n");
    if (testConnection(i, scanTimeout) == ESP_OK) {
      printf( "- Device found at address: 0x%X%s", i, "\n");
      count_i2c++;
    }
  }
  if (count_i2c == 0) {printf("- No I2C devices found!" "\n");}
}

////////////////////////////////////////////////////////////////////////////////
// Lidar Functions /////////////////////////////////////////////////////////////

// Write one byte to register
int writeRegister(uint8_t reg, uint8_t data) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR_2 << 1 ) | WRITE_BIT, ACK_CHECK_EN);
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
  i2c_master_write_byte(cmd, ( SLAVE_ADDR_2 << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR_2 << 1 ) | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, regData, ACK_CHECK_DIS);
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return 0;
}

// read 16 bits (2 bytes) -- not working for unknown reason
int16_t read16(uint8_t reg, uint8_t *first, uint8_t *second) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR_2 << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR_2 << 1 ) | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, first, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, second, ACK_CHECK_DIS);
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Task to continuously poll acceleration and calculate roll and pitch

static void test_lidar() {
  printf("\n>> Testing Lidar\n");

  int temp_dist = 0;
  int num = 0;
  int cali_dist = 15; // calibrating distance
  int samples = 5; // number of samples

  while(1) {
    uint8_t reg = REGISTER;
    uint8_t data = VALUE;
    writeRegister(reg, data);

    vTaskDelay(20 / portTICK_RATE_MS);

    uint8_t high_dist = 0;
    uint8_t low_dist = 0;
    uint8_t *first = &high_dist;
    uint8_t *second = &low_dist;

    uint8_t regH = 0x0f;
    uint8_t regL = 0x10;

    readRegister(regH, first);
    readRegister(regL, second);

    // Multisampling
    if(num < samples){
      temp_dist += (high_dist << 8) + low_dist - cali_dist;
      num++;
    }else{
      distance = temp_dist / samples;
      num = 0;
      temp_dist = 0;

      // Check if in range to stop
      if (distance <= 33) {
        status = 0;
      }else{
        status = 1;
      }
    }
  }
}


////////////////////////////////////////////////////////////////////////////////
// Alphanumeric Functions //////////////////////////////////////////////////////

// Turn on oscillator for alpha display
int alpha_oscillator() {
  int ret;
  i2c_cmd_handle_t cmd0 = i2c_cmd_link_create();
  i2c_master_start(cmd0);
  i2c_master_write_byte(cmd0, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd0, OSC, ACK_CHECK_EN);
  i2c_master_stop(cmd0);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd0, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd0);
  vTaskDelay(200 / portTICK_RATE_MS);
  return ret;
}

// Set blink rate to off
int no_blink() {
  int ret;
  i2c_cmd_handle_t cmd2 = i2c_cmd_link_create();
  i2c_master_start(cmd2);
  i2c_master_write_byte(cmd2, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd2, HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (HT16K33_BLINK_OFF << 1), ACK_CHECK_EN);
  i2c_master_stop(cmd2);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd2, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd2);
  vTaskDelay(200 / portTICK_RATE_MS);
  return ret;
}

// Set Brightness
int set_brightness_max(uint8_t val) {
  int ret;
  i2c_cmd_handle_t cmd3 = i2c_cmd_link_create();
  i2c_master_start(cmd3);
  i2c_master_write_byte(cmd3, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd3, HT16K33_CMD_BRIGHTNESS | val, ACK_CHECK_EN);
  i2c_master_stop(cmd3);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd3, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd3);
  vTaskDelay(200 / portTICK_RATE_MS);
  return ret;
}

////////////////////////////////////////////////////////////////////////////////

static void test_alpha_display() {
    // Debug
    int ret;
    printf(">> Test Alphanumeric Display: \n");

    // Set up routines
    // Turn on alpha oscillator
    ret = alpha_oscillator();
    if(ret == ESP_OK) {printf("- oscillator: ok \n");}
    // Set display blink off
    ret = no_blink();
    if(ret == ESP_OK) {printf("- blink: off \n");}
    ret = set_brightness_max(0xF);
    if(ret == ESP_OK) {printf("- brightness: max \n");}

    // Write to characters to buffer
    uint16_t displaybuffer[8];
    displaybuffer[0] = 0b0101001000000001;  // T.
    displaybuffer[1] = 0b0101001000001111;  // D.
    displaybuffer[2] = 0b0100000000111001;  // C.
    displaybuffer[3] = 0b0100000000111000;  // L.

    int character;
    int newdis;

    // Continually writes the same command
    while (1) {

      newdis = distance;

      for(int i = 3; i >= 0; i--){
        if(newdis > 0){
          character = newdis % 10;
          displaybuffer[i] = alphafonttable[character];
        }
        else{
          displaybuffer[i] = alphafonttable[10];
        }
        newdis = newdis / 10;
      }

      // Send commands characters to display over I2C
      i2c_cmd_handle_t cmd4 = i2c_cmd_link_create();
      i2c_master_start(cmd4);
      i2c_master_write_byte(cmd4, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
      i2c_master_write_byte(cmd4, (uint8_t)0x00, ACK_CHECK_EN);
      for (uint8_t i=0; i<8; i++) {
        i2c_master_write_byte(cmd4, displaybuffer[i] & 0xFF, ACK_CHECK_EN);
        i2c_master_write_byte(cmd4, displaybuffer[i] >> 8, ACK_CHECK_EN);
      }
      i2c_master_stop(cmd4);
      ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd4, 1000 / portTICK_RATE_MS);
      i2c_cmd_link_delete(cmd4);

      if(ret == ESP_OK) {
        printf("- wrote %d -\n\n", distance);
      }

      vTaskDelay(100);
    }


}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void app_main(void)
{

    // Routine

    printf("Testing servo motor.......\n");

    ESP_ERROR_CHECK( nvs_flash_init() );
    initialise_wifi();
    wait_for_ip();

    i2c_master_init();
    i2c_scanner();

    // Create task for IP chat
    xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);

    // Create task to poll LIDAR
    xTaskCreate(test_lidar,"test_lidar", 4096, NULL, 5, NULL);

    // Create task to run alphanumeric display
    xTaskCreate(test_alpha_display,"test_alpha_display", 4096, NULL, 5, NULL);

    xTaskCreate(mcpwm_servo_control, "mcpwm_servo_control", 4096, NULL, 5, NULL);
    xTaskCreate(set_speed, "set_speed", 1024, NULL, 5, NULL);
    xTaskCreate(steerPID,"steerPID", 4096, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(speedPID,"speedPID", 4096, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(read_ir,"read_ir", 4096, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(adc_task, "adc_task", 4096, NULL, 5, NULL);
    xTaskCreate(timer, "timer", 4096, NULL, 5, NULL);
}
