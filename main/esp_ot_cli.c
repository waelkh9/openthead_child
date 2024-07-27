/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * OpenThread Command Line Example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "driver/uart.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_ieee802154.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_types.h"
#include "esp_openthread.h"
#include "esp_openthread_cli.h"
#include "esp_openthread_lock.h"
#include "esp_openthread_netif_glue.h"
#include "esp_openthread_types.h"
#include "esp_ot_config.h"
#include "esp_vfs_eventfd.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/uart_types.h"
#include "nvs_flash.h"
#include "openthread/cli.h"
#include "openthread/instance.h"
#include "openthread/logging.h"
#include "openthread/tasklet.h"
#include "openthread/ping_sender.h"
#include "openthread/udp.h"
#include <bh1750_i2c_hal.h>
#include <bh1750_i2c.h>
#include <bme680.h>
#include <string.h>

#if CONFIG_OPENTHREAD_CLI_ESP_EXTENSION
#include "esp_ot_cli_extension.h"
#endif // CONFIG_OPENTHREAD_CLI_ESP_EXTENSION

#define TAG "ot_esp_cli"

#define UDP_JOINER_PORT 49155
#define BME680_I2C_ADDR 0x77
#define PORT 0
#define CONFIG_EXAMPLE_I2C_MASTER_SDA 21
#define CONFIG_EXAMPLE_I2C_MASTER_SCL 22
#define CONFIG_I2C_MASTER_SCL 19
#define CONFIG_I2C_MASTER_SDA 18
otUdpSocket socket_info;
volatile float temperature=0;
volatile float humidity=0;
volatile float pressure=0;
volatile float lux=0;

void bme680_test()
{
    bme680_t sensor;
    memset(&sensor, 0, sizeof(bme680_t));

    ESP_ERROR_CHECK(bme680_init_desc(&sensor, BME680_I2C_ADDR, PORT, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));

    // init the sensor
    ESP_ERROR_CHECK(bme680_init_sensor(&sensor));

    // Changes the oversampling rates to 4x oversampling for temperature
    // and 2x oversampling for humidity. Pressure measurement is skipped.
    bme680_set_oversampling_rates(&sensor, BME680_OSR_4X, BME680_OSR_2X, BME680_OSR_2X);

    // Change the IIR filter size for temperature and pressure to 7.
    bme680_set_filter_size(&sensor, BME680_IIR_SIZE_7);

    // Change the heater profile 0 to 200 degree Celsius for 100 ms.
    bme680_set_heater_profile(&sensor, 0, 200, 100);
    bme680_use_heater_profile(&sensor, 0);

    // Set ambient temperature to 10 degree Celsius
    bme680_set_ambient_temperature(&sensor, 10);

    // as long as sensor configuration isn't changed, duration is constant
    uint32_t duration;
    bme680_get_measurement_duration(&sensor, &duration);

    TickType_t last_wakeup = xTaskGetTickCount();

    bme680_values_float_t values;
    
    
        // trigger the sensor to start one TPHG measurement cycle
        if (bme680_force_measurement(&sensor) == ESP_OK)
        {
            // passive waiting until measurement results are available
            vTaskDelay(duration);
            temperature = values.temperature;
            humidity = values.humidity;
            pressure = values.pressure;
            // get the results and do something with them
            if (bme680_get_results_float(&sensor, &values) == ESP_OK)
                printf("temperature: %.2f °C, %.2f %%, %.2f hPa, %.2f Ohm\n",
                        values.temperature, values.humidity, values.pressure, values.gas_resistance);

            temperature = values.temperature;
            humidity = values.humidity;
            pressure = values.pressure;
            vTaskDelay(1000);
        }
        
        // passive waiting until 1 second is over
        vTaskDelayUntil(&last_wakeup, pdMS_TO_TICKS(1000));
        //vTaskDelete(NULL);
        i2c_driver_delete(0);
       
    
}
void bht1750()
{
    int x=0;
    bh1750_dev_t dev_1;
    esp_err_t err;

    bh1750_i2c_hal_init();

    /* Device init */
    dev_1.i2c_addr = I2C_ADDRESS_BH1750;
    dev_1.mtreg_val = DEFAULT_MEAS_TIME_REG_VAL;

    /* Perform device reset */
    err = bh1750_i2c_dev_reset(dev_1); 
    ESP_LOGI(TAG, "Device reset: %s", err == BH1750_OK ? "Successful" : "Failed");

    err += bh1750_i2c_set_power_mode(dev_1, BH1750_POWER_ON);
    ESP_LOGI(TAG, "Changing power mode to ON: %s", err == BH1750_OK ? "Successful" : "Failed");

    /* Change measurement time with  50% optical window transmission rate */
    err += bh1750_i2c_set_mtreg_val(&dev_1, 50);
    ESP_LOGI(TAG, "Changing measurement time: %s", err == BH1750_OK ? "Successful" : "Failed");

    /* Configure device */
    err += bh1750_i2c_set_resolution_mode(&dev_1, BH1750_CONT_H_RES_MODE);
    if (err == BH1750_OK)
    {
        ESP_LOGI(TAG, "BH1750 config successful");
    }
    else{
        ESP_LOGE(TAG, "BH1750 config failed!");
    }
    /* End of device config */

    if (err == BH1750_OK)
    {
        ESP_LOGI(TAG, "BH1750 initialization successful");
        //Start reading data
        uint16_t data_light;
        while(x<3){
        
            bh1750_i2c_read_data(dev_1, &data_light);
            ESP_LOGI(TAG, "Light Intensity: %d Lux", data_light);
            lux = data_light;
            x+=1;
        } 
        
    }
    else{
        ESP_LOGE(TAG, "BH1750 initialization failed!");
    }
    //vTaskDelete(NULL);
    i2c_driver_delete(0);
    
}


static esp_netif_t *init_openthread_netif(const esp_openthread_platform_config_t *config)
{
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_OPENTHREAD();
    esp_netif_t *netif = esp_netif_new(&cfg);
    assert(netif != NULL);
    ESP_ERROR_CHECK(esp_netif_attach(netif, esp_openthread_netif_glue_init(config)));

    return netif;
}


void udp_rx_callback(void *aContext, otMessage *aMessage, const otMessageInfo *aMessageInfo)
{
    char buf[1500];
    int length = otMessageRead(aMessage, otMessageGetOffset(aMessage), buf, sizeof(buf) - 1);
    buf[length] = '\0';  // Ensure null-termination
    ESP_LOGI(TAG, "Received UDP message: %s", buf);
    
}

void init_udp(void)
{
    otInstance * thread_instance = esp_openthread_get_instance();
    otSockAddr bind_info;
    otNetifIdentifier netif = OT_NETIF_THREAD;
    memset(&bind_info, 0, sizeof(otSockAddr));
    otIp6AddressFromString("::", &bind_info.mAddress);
    bind_info.mPort = UDP_JOINER_PORT;
    otError err = otUdpOpen(thread_instance, &socket_info, udp_rx_callback, NULL);    

    if (err != OT_ERROR_NONE)
    {
        ESP_LOGE(TAG, "UDP initialization failed because of the error %d", err); 
    } else
    {
        ESP_LOGI(TAG, "udp initialization was successful"); 
    }
    err = otUdpBind(thread_instance, &socket_info, &bind_info, netif);
    if (err != OT_ERROR_NONE)
    {
        ESP_LOGE(TAG, "UDP was not bind successfully %d", err);  
    }
}
void format_message(char *buffer, size_t buffer_size, float temperature, float humidity, float lux) {
    snprintf(buffer, buffer_size, "=%.2fH=%.2f%%L=%.2f", temperature, humidity, lux);
}
void send_udp()
{   
    char buf1[25];

    otMessageInfo messageInfo;
    otMessageSettings new_msg_settings;
    new_msg_settings.mLinkSecurityEnabled = true;
    new_msg_settings.mPriority = 1;
    otIp6AddressFromString("ff02::1", &messageInfo.mPeerAddr);
    messageInfo.mPeerPort = UDP_JOINER_PORT;
    otMessage * message = otUdpNewMessage(esp_openthread_get_instance(), &new_msg_settings);
   // const char * buf1 = "hello";
    format_message(buf1, sizeof(buf1), temperature, humidity, lux);
    //printf("this is value %s", buf1);
    const char * buf = buf1;
    otError err = otMessageAppend(message, buf, (uint16_t) strlen(buf));
    if (err != OT_ERROR_NONE)
    {
        ESP_LOGE(TAG, "message creation failed %d", err); 
    }
    err = otUdpSend(esp_openthread_get_instance(), &socket_info, message, &messageInfo);
    if (err != OT_ERROR_NONE)
    {
        ESP_LOGE(TAG, "failed to send message %d", err);
    }
}
void read_sensor_data() {

        while(1) {
        i2c_driver_delete(0);
        ESP_ERROR_CHECK(i2cdev_init());
        bht1750();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "finished light measurement");
        bme680_test();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "finished temperature and humiditiy measurement");

        }
        
        
        vTaskDelete(NULL);
         

}
static void udp_send_task()
{
    while(1)
    {   

        send_udp();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);

}

static void ot_task_worker(void *aContext) {
  esp_openthread_platform_config_t config = {
      .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
      .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
      .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
  };
  ESP_LOGI(TAG, "initialization of thread");

  // Initialize the OpenThread stack
  ESP_ERROR_CHECK(esp_openthread_init(&config));

#if CONFIG_OPENTHREAD_LOG_LEVEL_DYNAMIC
  // The OpenThread log level directly matches ESP log level
  (void)otLoggingSetLevel(CONFIG_LOG_DEFAULT_LEVEL);
#endif
  // Initialize the OpenThread cli
#if CONFIG_OPENTHREAD_CLI
  esp_openthread_cli_init();
#endif

  esp_netif_t *openthread_netif;
  // Initialize the esp_netif bindings
  openthread_netif = init_openthread_netif(&config);
  esp_netif_set_default_netif(openthread_netif);

#if CONFIG_OPENTHREAD_CLI_ESP_EXTENSION
  esp_cli_custom_command_init();
#endif // CONFIG_OPENTHREAD_CLI_ESP_EXTENSION
  ESP_LOGI(TAG, "start the main loop initialization ");
  // Run the main loop
#if CONFIG_OPENTHREAD_CLI
  esp_openthread_cli_create_task();
#endif
#if CONFIG_OPENTHREAD_AUTO_START
  otOperationalDatasetTlvs dataset;
  otError error =
      otDatasetGetActiveTlvs(esp_openthread_get_instance(), &dataset);
  ESP_ERROR_CHECK(
      esp_openthread_auto_start((error == OT_ERROR_NONE) ? &dataset : NULL));
#endif
  ESP_LOGI(TAG, "finished main loop");
  
  
  esp_openthread_launch_mainloop();

  // Clean up
  esp_netif_destroy(openthread_netif);
  esp_openthread_netif_glue_deinit();
  esp_vfs_eventfd_unregister();
  
  //vTaskDelete(NULL);
}
void app_main(void)
{
    // Used eventfds:
    // * netif
    // * ot task queue
    // * radio driver
    esp_vfs_eventfd_config_t eventfd_config = {
        .max_fds = 3,
    };

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_vfs_eventfd_register(&eventfd_config));

    xTaskCreate(ot_task_worker, "ot_cli_main", 10240, xTaskGetCurrentTaskHandle(), 5, NULL);
    //adding delays to avoid crashes
    vTaskDelay(5000);
    
    xTaskCreate(read_sensor_data, "sensor_data", 4096, NULL, 5, NULL);
    vTaskDelay(1000);
    init_udp();

    xTaskCreate(udp_send_task, "udp_send", 4096, NULL, 5, NULL);   

}
