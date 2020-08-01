#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/adc.h"
#include "string.h"

#include "esp_attr.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#include "esp_adc_cal.h"

#include "driver/i2c.h"

// IP chat
#include <string.h>
#include <sys/param.h>
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

// Local time
#include <stdlib.h>
#include <time.h>

#include <stdbool.h>

// For ADC
#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling


// For microLIDAR sensors
#define TX_FRONT  (GPIO_NUM_17)
#define RX_FRONT  (GPIO_NUM_16)
#define ECHO_TEST_RTS  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS  (UART_PIN_NO_CHANGE)
#define TX_REAR  (GPIO_NUM_25)
#define RX_REAR  (GPIO_NUM_26)

// For IR Receiver
#define BEACON_TXD  (GPIO_NUM_12) // 12
#define BEACON_RXD  (GPIO_NUM_4) // A5

#define BUF_SIZE (1024)

// For steering
#define SERVO_MIN_PULSEWIDTH 650 //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2150 //Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 100 //Maximum angle in degree upto which servo can rotate

// For Master I2C
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

// For Lidar
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

// For ADC
static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_6;     //GPIO34 if ADC1 (A2)
static const adc_atten_t atten = ADC_ATTEN_DB_0;
static const adc_unit_t unit = ADC_UNIT_1;

// msg = "0 speed - - -"
//     = "ID speed hour min sec" when encounters a new beacon

char payload[30] = "0 0 0 0 0";

bool pulsed = false;
int count;

int front_range = 0; // distance from front sensor
int rear_range = 0; // distance from rear sensor
int lidar_range = 0; // distance from lidar in front

char traffic_light = 'X';
int beacon_id = 0; // id = 0 initially

// Controls to store one split time at each beacon
int previous_id = 0;
int split_time = 0;

// split time
int split_hour = -1;
int split_min = -1;
int split_sec = -1;

int start_hour = -1;
int start_min = -1;
int start_sec = -1;

// controls from ipchat
int manual = 0;
int stop = 0;
int forwards = 1;
int angleChange = 54;

// status for Auto Driving
int status = 0;

bool cali_done = false;
int steerAngle = 54;
uint32_t speedpw = 1164;
float speed = 0;

////////////////////////////////////////////////////////////////////////////////
// Read Beacon /////////////////////////////////////////////////////////////////
static void read_beacon(void *arg)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 1200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, BEACON_TXD, BEACON_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS);
    uart_set_line_inverse(UART_NUM_1, UART_INVERSE_RXD);
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    unsigned int checksum;

    while (1) {
        // Read data from the UART
        int len = uart_read_bytes(UART_NUM_1, data, BUF_SIZE, 20 / portTICK_RATE_MS);
        // printf("reading something: %d bytes\n", len);

        if (len > 0) {
          for (int i = 0; i < len; i++) {
            if ((int) data[i] == 0x1B) {
              checksum = 0x1B ^ data[i + 1];
              checksum = checksum ^ data[i + 2];
              if (checksum == data[i + 3]) {
                // printf("------------------------------Found a beacon----------------------------\n");
                int light_int = (int) data[i + 1];
                traffic_light = (char) light_int;
                beacon_id = (int) data[i + 2];
              }else{
                traffic_light = 'X';
                beacon_id = 0;
              }
              break;
            }
          }
        }
        vTaskDelay(30);
    }
}


////////////////////////////////////////////////////////////////////////////////
// Wheel Speed /////////////////////////////////////////////////////////////////

void get_ticks(void *arg)
{
  // Initialize counter
  count = 0;

  //Configure ADC
  if (unit == ADC_UNIT_1) {
      adc1_config_width(ADC_WIDTH_BIT_12);
      adc1_config_channel_atten(channel, atten);
  } else {
      adc2_config_channel_atten((adc2_channel_t)channel, atten);
  }

  //Characterize ADC
  adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));

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

      if (adc_reading < 4095 && pulsed == false)
      {
        count++;
        printf("Count incremented\n");
        pulsed = true;
      } else if (adc_reading == 4095) {
        pulsed = false;
      }
      vTaskDelay(pdMS_TO_TICKS(100));
  }
}

static void get_speed() {
  while(1) {
    count = 0;
    vTaskDelay(100);
    speed = (count * (2*3.14159*7/6))/100;
    count = 0;
  }
}


////////////////////////////////////////////////////////////////////////////////
// Status for Steering and Speed ///////////////////////////////////////////////

void set_auto_status() {
  while(1){
    if(manual == 0) { // Auto mode

      if(status == 0)
        status = 1; // default straight

      if(lidar_range > 150 && front_range > 100 && rear_range > 100)
        status = 1; // straight
      else if(lidar_range < 100 && front_range > 100 && rear_range > 100)
        status = 2; // stop assume obstacle but not corner
      else if(lidar_range < 200 && (front_range < 100 || rear_range < 100))
        status = 3; // corner
      else if(lidar_range > 200 && front_range < 100 && rear_range < 100)
        status = 1; // along wall

      if(stop == 1)
        status = 2; // stop

      if(traffic_light == 'R' || traffic_light == 'Y')
        status = 2; // stop at RED and YELLOW

    }else{ // Manual mode
      status = 0;
    }

    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void static read_ultra_rear() {

  /* Configure parameters of an UART driver,
   * communication pins and install the driver */
  uart_config_t uart_config = {
      .baud_rate = 9600,
      .data_bits = UART_DATA_8_BITS,
      .parity    = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
  };
  uart_param_config(UART_NUM_2, &uart_config);
  uart_set_pin(UART_NUM_2, TX_REAR, RX_REAR, ECHO_TEST_RTS, ECHO_TEST_CTS);
  uart_set_line_inverse(UART_NUM_2, UART_INVERSE_RXD);
  uart_driver_install(UART_NUM_2, BUF_SIZE * 2, 0, 0, NULL, 0);

  // Configure a temporary buffer for the incoming data
  uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

  while (1) {
      // Read data from the UART
      int len = uart_read_bytes(UART_NUM_2, data, BUF_SIZE, 20 / portTICK_RATE_MS);

      if (len > 0) {
        char first = (char) data[1];
        char second = (char) data[2];
        char third = (char) data[3];
        char str[100] = {0};
        strncat(str, &first, 1);
        strncat(str, &second, 1);
        strncat(str, &third, 1);
        rear_range = atoi(str);
        rear_range = rear_range * 2.54; // convert inches to cm
        rear_range = 75;
        front_range = rear_range;
        // printf("Distance is: %d\n", dist);
      }
      vTaskDelay(5);
  }
}


////////////////////////////////////////////////////////////////////////////////
// Crawler /////////////////////////////////////////////////////////////////////

// Calibration of crawler
static void mcpwm_example_gpio_initialize(void)
{
    printf("initializing mcpwm servo control gpio......");
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
  gpio_set_level(13, 1);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  gpio_set_level(13, 0);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  gpio_set_level(13, 1);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  gpio_set_level(13, 0);
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  steerAngle = 75;
  // sprintf(payload, "0 0 0 0 0 -----Turn on crawler in 4 sec:");
  // vTaskDelay(4000 / portTICK_PERIOD_MS);  // Give yourself time to turn on crawler

  printf("Set to HIGH\n");
  // sprintf(payload, "0 0 0 0 0 Set to HIGH");
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 2100); // HIGH signal in microseconds
  vTaskDelay(1000 / portTICK_PERIOD_MS);


  printf("Set to LOW\n");
  // sprintf(payload, "0 0 0 0 0 Set to LOW");
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 700);  // LOW signal in microseconds
  vTaskDelay(1000 / portTICK_PERIOD_MS);


  printf("Set to NEUTRAL\n");
  // sprintf(payload, "0 0 0 0 0 Set to NEUTRAL");
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400); // NEUTRAL signal in microseconds
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  printf("Set to NEUTRAL! Cali DONE\n");
  // sprintf(payload, "0 0 0 0 0 Set to NEUTRAL! Calibration DONE");
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400); // reset the ESC to neutral (non-moving) value

  steerAngle = 54;
}

void mcpwm_servo_control(void *arg)
{
    uint32_t angle;
    //1. mcpwm gpio initialization
    mcpwm_example_gpio_initialize();

    //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm......");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings

    printf("Calibrating ESC......");
    calibrateESC();

    cali_done = true;

    while (1) {
      // printf("\nAngle of rotation: %d\n", steerAngle);
      angle = servo_per_degree_init(steerAngle);

      // printf("ESC pulse width: %dus\n\n", speedpw);

      mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, speedpw);
      mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, angle);
      vTaskDelay(100 / portTICK_PERIOD_MS);     //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
    }
}

////////////////////////////////////////////////////////////////////////////////
void set_steer() {

  while(1) {

    if(status == 0) { // manual mode
      steerAngle = angleChange;
      vTaskDelay(1000 / portTICK_RATE_MS);
      continue;

    // auto mode
    } else if(status == 3){ // corner
      steerAngle = 95;
      speed = 0.05;
      vTaskDelay(5000 / portTICK_RATE_MS);

    } else if(status == 4){
      if (front_range < 50) {
        steerAngle = 84;
      } else if (front_range < 60) {
        steerAngle = 74;
      } else if (front_range < 70) {
        steerAngle = 64;
      } else if (front_range > 80) {
        steerAngle = 44;
      } else if (front_range > 90) {
        steerAngle = 24;
      } else if (front_range < 100) {
        steerAngle = 5;
      }

      vTaskDelay(2000 / portTICK_RATE_MS);
    }

    vTaskDelay(1000 / portTICK_RATE_MS);
    steerAngle = 54;
  }
}


////////////////////////////////////////////////////////////////////////////////
void set_speed(){
  float Kp = 0.2; //0.2;
  float Ki = 0.001; //0.001;
  float Kd = 0.08; //0.08;
  float previous_error = 0.00;
  float integral = 0.00;
  int dt = 0.5;

    while(1){
      if(cali_done){
        if(status > 0){ // constant speed under auto duty_mode

          float error = 0.1 - speed;

          integral = error * dt;
          float derivative = (error - previous_error) / dt;
          float output = Kp * error + Ki * integral + Kd * derivative;
          previous_error = error;

          speedpw -= output;

          if (speedpw > 1170 || speedpw < 1160){
            speedpw = 1164;
          }

          if(status == 2){
            speedpw = 1215;
          }

          if (status == 3) {
          }

        }else{ // manual mode
          if((forwards == 1) && (stop == 0)) {
            float error = 0.1 - speed;

            integral = error * dt;
            float derivative = (error - previous_error) / dt;
            float output = Kp * error + Ki * integral + Kd * derivative;
            previous_error = error;

            speedpw -= output;

            if (speedpw > 1170 || speedpw < 1160){
              speedpw = 1164;
            }

          } else if((forwards == 0) && (stop == 0)) {
            float error = -0.1 - speed;

            integral = error * dt;
            float derivative = (error - previous_error) / dt;
            float output = Kp * error + Ki * integral + Kd * derivative;
            previous_error = error;

            speedpw -= output;

            if (speedpw > 1440 || speedpw < 1450) {
              speedpw = 1444;
            }
          } else {
            speedpw = 1215;
          }

        }
      } else {
        speedpw = 1215;
      }
      vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

////////////////////////////////////////////////////////////////////////////////
// IP chat /////////////////////////////////////////////////////////////////////

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;

const int IPV4_GOTIP_BIT = BIT0;
const int IPV6_GOTIP_BIT = BIT1;

static const char *TAG = "Capture the Flag";

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

        char *token;

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

                // msgback = 'manual stop forward angle'
                ESP_LOGI(TAG, "%s", rx_buffer);

                // Extract the tokens
                token = strtok(rx_buffer, " ");
                manual = atoi(token);
                token = strtok(NULL, " ");
                stop = atoi(token);
                token = strtok(NULL, " ");
                forwards = atoi(token);
                token = strtok(NULL, " ");
                angleChange = atoi(token);

            }

            vTaskDelay(500 / portTICK_PERIOD_MS);
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
// I2C /////////////////////////////////////////////////////////////////////////

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
  printf(">> I2C scanning ...");
  uint8_t count_i2c = 0;
  for (uint8_t i = 1; i < 127; i++) {
    // printf("0x%X%s",i,"\n");
    if (testConnection(i, scanTimeout) == ESP_OK) {
      printf("- Device found at address: 0x%X", i);
      count_i2c++;
    }
  }
  if (count_i2c == 0) {printf("- No I2C devices found!");}
}

////////////////////////////////////////////////////////////////////////////////
// Read LIDAR Range ////////////////////////////////////////////////////////////

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


// Task to continuously poll acceleration and calculate roll and pitch

static void read_lidar() {
  printf(">> Testing Lidar");

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
      lidar_range = temp_dist / samples;
      num = 0;
      temp_dist = 0;

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
static void alpha_display() {
    // Debug
    int ret;
    printf(">> Test Alphanumeric Display:");

    // Set up routines
    // Turn on alpha oscillator
    ret = alpha_oscillator();
    if(ret == ESP_OK) {printf("- oscillator: ok ");}
    // Set display blink off
    ret = no_blink();
    if(ret == ESP_OK) {printf("- blink: off");}
    ret = set_brightness_max(0xF);
    if(ret == ESP_OK) {printf("- brightness: max");}

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

      newdis = split_time;

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
        // printf("- wrote %d -\n\n", lidar_range);
      }

      vTaskDelay(100);
    }

}

////////////////////////////////////////////////////////////////////////////////
// Printer /////////////////////////////////////////////////////////////////////

void static printer() {
  while(1) {
    vTaskDelay(100);

    printf("status = %d\n", status);
    printf("manual = %d\n", manual);
    printf("Lidar Range: \t%dcm\nFront Range: \t%dcm\nRear Range: \t%dcm\nSteer Angle: \t%d\nSpeed PW: \t%d\nSpeed: \t%.2f\nBeacon ID: \t%d\nLight: \t%c\n\n", lidar_range, front_range, rear_range, steerAngle, speedpw, speed, beacon_id, traffic_light);

    if(cali_done){
      // msg = "ID speed hour min sec"
      // sprintf(payload, "%d %.2f %d %d %d status=%d-Lidar:%dcm-Front:%dcm-Rear:%dcm-Angle:%d-Speedpw:%d-BeaconID:%d-Light:%c", beacon_id, speed, split_hour, split_min, split_sec, status, lidar_range, front_range, rear_range, steerAngle, speedpw, beacon_id, traffic_light);

      if((previous_id != beacon_id) && beacon_id != 0){
        time_t curtime;
        struct tm *loc_time;

        char hour_str[5];
        char min_str[5];
        char sec_str[5];

        int current_hour = -1;
        int current_min = -1;
        int current_sec = -1;

        //Getting current time of system
        curtime = time (NULL);

        // Converting current time to local time
        loc_time = localtime (&curtime);

        strftime(hour_str, 5, "%I", loc_time);
        strftime(min_str, 5, "%M", loc_time);
        strftime(sec_str, 5, "%S", loc_time);

        if(previous_id == 0){
          start_hour = atoi(hour_str);
          start_min = atoi(min_str);
          start_sec = atoi(sec_str);
          split_time = 0;
          sprintf(payload, "%d %.2f 0 0 0", beacon_id, speed);
          // printf("\n%d START: %d %d %d\n", i, start_hour, start_min, start_sec);
        }else{
          current_hour = atoi(hour_str);
          current_min = atoi(min_str);
          current_sec = atoi(sec_str);
          // printf("%d CURRENT: %d %d %d\n", i, current_hour, current_min, current_sec);

          if(start_hour < 0){
            split_hour = current_hour;
            split_min = current_min;
            split_sec = current_sec;
            split_time = 0;
          }else{
            split_hour = current_hour - start_hour;
            split_min = current_min - start_min;
            split_sec = current_sec - start_sec;
            split_time = split_min*60 + split_sec;

            split_sec = split_time%60;
            split_hour = split_time/3600;
            split_min = split_time/60 - split_hour*60;
          }
          // printf("SPLIT: %d %d %d\n", split_hour, split_min, split_sec);

          sprintf(payload, "%d %.2f %d %d %d", beacon_id, speed, split_hour, split_min, split_sec);

        }

        previous_id = beacon_id;
      }

      else{
        sprintf(payload, "0 %.2f 0 0 0", speed);
      }

    }

  }
}

////////////////////////////////////////////////////////////////////////////////
// MAIN ////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void app_main(void)
{

  gpio_pad_select_gpio(13);
  gpio_set_direction(13, GPIO_MODE_OUTPUT);

    ESP_ERROR_CHECK( nvs_flash_init() );
    initialise_wifi();
    wait_for_ip();

    i2c_master_init();
    i2c_scanner();

    // IP chat
    xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);

    // Lidar
    xTaskCreate(read_lidar,"read_lidar", 4096, NULL, 5, NULL);

    // Alphanumeric display
    xTaskCreate(alpha_display,"alpha_display", 4096, NULL, 5, NULL);

    // Optical Encoder
    xTaskCreate(get_ticks,"get_ticks", 4096, NULL, 5, NULL);
    //xTaskCreate(get_speed,"get_speed", 4096, NULL, 5, NULL);

    // IR Receiver
    xTaskCreate(read_beacon, "read_beacon", 4096, NULL, 5, NULL);

    xTaskCreate(set_auto_status,"set_auto_status", 4096, NULL, 5, NULL);
    //xTaskCreate(fake_time,"fake_time", 4096, NULL, configMAX_PRIORITIES, NULL);

    xTaskCreate(mcpwm_servo_control, "mcpwm_servo_control", 4096, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(set_speed, "set_speed", 4096, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(set_steer, "set_steer", 4096, NULL, configMAX_PRIORITIES, NULL);
    // xTaskCreate(read_micro_front,"read_micro_front", 4096, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(read_ultra_rear,"read_ultra_rear", 4096, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(printer,"printer", 4096, NULL, configMAX_PRIORITIES, NULL);
}
