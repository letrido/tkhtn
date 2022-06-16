
/* Blink Example

  This example code is in the Public Domain (or CC0 licensed, at your option.)

  Unless required by applicable law or agreed to in writing, this
  software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
  CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "esp32/rom/uart.h"
#include "freertos/event_groups.h"
#include "smbus.h"
#include "i2c_lcd1602.h"
#include "keyboard.h"
#include "dht11.h"
#define TAG "app"

#define BOM_NUM GPIO_NUM_33
#define QUAT_NUM GPIO_NUM_25
#define BAT_QUAT gpio_set_level(QUAT_NUM, 0)
#define TAT_QUAT gpio_set_level(QUAT_NUM, 1)
#define BAT_BOM gpio_set_level(BOM_NUM, 0)
#define TAT_BOM gpio_set_level(BOM_NUM, 1)
#define MANUAL_MODE 0
#define AUTO_MODE 1
#define CONFIG_MODE 2
// #define LCD_NUM_ROWS 2
// #define LCD_NUM_COLUMNS 32
// #define LCD_NUM_VISIBLE_COLUMNS 16
// LCD2004
#define LCD_NUM_ROWS 4
#define LCD_NUM_COLUMNS 40
#define LCD_NUM_VISIBLE_COLUMNS 20

#define USE_STDIN 1
//#undef USE_STDIN
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_TX_BUF_LEN 0 // disabled
#define I2C_MASTER_RX_BUF_LEN 0 // disabled
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_SDA_IO 21 // 19
#define I2C_MASTER_SCL_IO 22 // 18

uint32_t ulCount = 0;

SemaphoreHandle_t xSemaphore;

// void pass_word_mode(void i2c_lcd1602_info_t *infolcd);
void manual_mode(i2c_lcd1602_info_t *infolcd);
void Auto_mode(i2c_lcd1602_info_t *infolcd);
void config(i2c_lcd1602_info_t *infolcd);
// void pass_word_mode(i2c_lcd1602_info_t *infolcd);
static signed char temp_config = 50, humi_config = 100, Time_config = 0;
QueueHandle_t xQueue_nhietdo = NULL;
/*=======================================================================*/
static int u8Count = 0;
static char key;
uint8_t mode = MANUAL_MODE;
TimerHandle_t xTimers[1];
uint8_t bom = 0, quat = 0;

static struct dht11_reading dht11_last_data, dht11_cur_data;
typedef void (*gpio_check_logic_t)(void);

typedef void (*CHECK_MODE_t)(i2c_lcd1602_info_t *);
typedef void (*Interface_t)(i2c_lcd1602_info_t *);

Interface_t Interface;
const CHECK_MODE_t CHECK_MODE[3] = {
    manual_mode,
    Auto_mode,
    config};

uint8_t Status = 0;
static char temp[10], humi[5];
static char temp_config_t[10], humi_config_t[5], Time_config_t[5], Status_t[5];
const gpio_check_logic_t keypad_row_en[4] = {
    keypad_row1_en,
    keypad_row2_en,
    keypad_row3_en,
    keypad_row4_en};
static void i2c_master_init(void)
{
   int i2c_master_port = I2C_MASTER_NUM;
   i2c_config_t conf;
   conf.mode = I2C_MODE_MASTER;
   conf.sda_io_num = I2C_MASTER_SDA_IO;
   conf.sda_pullup_en = GPIO_PULLUP_DISABLE; // GY-2561 provides 10kΩ pullups
   conf.scl_io_num = I2C_MASTER_SCL_IO;
   conf.scl_pullup_en = GPIO_PULLUP_DISABLE; // GY-2561 provides 10kΩ pullups
   conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
   i2c_param_config(i2c_master_port, &conf);
   i2c_driver_install(i2c_master_port, conf.mode,
                      I2C_MASTER_RX_BUF_LEN,
                      I2C_MASTER_TX_BUF_LEN, 0);
}
void manual_mode(i2c_lcd1602_info_t *infolcd)
{
   xTimerStop(xTimers[0], 0);
   struct dht11_reading *newcmd;
   i2c_lcd1602_clear(infolcd);
   i2c_lcd1602_move_cursor(infolcd, 0, 0);
   i2c_lcd1602_write_string(infolcd, "[B]->AUTO/[C]CONFIG");

   i2c_lcd1602_move_cursor(infolcd, 0, 1);
   i2c_lcd1602_write_string(infolcd, "TEMP:");

   i2c_lcd1602_move_cursor(infolcd, 0, 2);
   i2c_lcd1602_write_string(infolcd, "HUMID:");

   i2c_lcd1602_move_cursor(infolcd, 0, 3);
   i2c_lcd1602_write_string(infolcd, "Do am dat:");
   i2c_lcd1602_move_cursor(infolcd, 12, 1);
   i2c_lcd1602_write_string(infolcd, "[1]");
   i2c_lcd1602_move_cursor(infolcd, 16, 1);
   i2c_lcd1602_write_string(infolcd, "[2]");

   i2c_lcd1602_move_cursor(infolcd, 12, 2);
   i2c_lcd1602_write_string(infolcd, "[3]");
   i2c_lcd1602_move_cursor(infolcd, 16, 2);
   i2c_lcd1602_write_string(infolcd, "[4]");

   i2c_lcd1602_move_cursor(infolcd, 12, 3);
   i2c_lcd1602_write_string(infolcd, "[5]");
   i2c_lcd1602_move_cursor(infolcd, 16, 3);
   i2c_lcd1602_write_string(infolcd, "[6]");
   while (1)
   {

      xQueueReceive(xQueue_nhietdo, (void *)&newcmd, (TickType_t)0);
      /* xEventGroupWaitBits() returned because both bits were set. */
      sprintf(temp, "%.1f", newcmd->temperature);
      sprintf(humi, "%.1f%%", newcmd->humidity);
      i2c_lcd1602_move_cursor(infolcd, 6, 1);
      i2c_lcd1602_write_string(infolcd, temp);

      i2c_lcd1602_move_cursor(infolcd, 6, 2);
      i2c_lcd1602_write_string(infolcd, humi);

      i2c_lcd1602_move_cursor(infolcd, 6, 3);
      i2c_lcd1602_write_string(infolcd, "20%%");
      keypad_row_en[u8Count]();
      key = waitKey(u8Count);

      //==================DIEU KHIEN==============
      if (key == '1')
      {
         printf("quat bat\n");
         BAT_QUAT;
         // gpio_set_level(QUAT_NUM, 1);
      }
      else if (key == '2')
      {
         // gpio_set_level(QUAT_NUM, 0);
         TAT_QUAT;
         printf("quat tat\n");
      }
      if (key == '4')
      {
         printf("bom bat\n");
         BAT_BOM;
         // gpio_set_level(BOM_NUM, 1);
      }
      else if (key == '5')
      {
         // gpio_set_level(BOM_NUM, 0);
         TAT_BOM;
         printf("bom bat\n");
      }

      //======================================
      if (key == 'C')
         mode = CONFIG_MODE;
      if (key == 'B')
         mode = AUTO_MODE;
      if (mode == CONFIG_MODE || mode == AUTO_MODE)
      {
         break;
      }
      // if (key != '0')
      //    printf("%c", key);
      if (++u8Count == 4)
      {
         u8Count = 0;
      }
      vTaskDelay(20 / portTICK_PERIOD_MS);
   }
}

void config(i2c_lcd1602_info_t *infolcd)
{
   uint8_t lc = 0;

   i2c_lcd1602_clear(infolcd);
   i2c_lcd1602_move_cursor(infolcd, 1, 0);
   i2c_lcd1602_write_string(infolcd, "Cai Dat:");
   i2c_lcd1602_move_cursor(infolcd, 0, 1);
   i2c_lcd1602_write_string(infolcd, "Nhiet Do:");
   i2c_lcd1602_move_cursor(infolcd, 0, 2);
   i2c_lcd1602_write_string(infolcd, "Do am kk:");
   i2c_lcd1602_move_cursor(infolcd, 0, 3);
   i2c_lcd1602_write_string(infolcd, "Time_set:");
   while (1)
   {

      sprintf(temp_config_t, "%d", temp_config);
      sprintf(humi_config_t, "%d", humi_config);
      sprintf(Time_config_t, "%d", Time_config);
      sprintf(Status_t, "%d", Status);
      i2c_lcd1602_move_cursor(infolcd, 10, 1);
      i2c_lcd1602_write_string(infolcd, temp_config_t);
      i2c_lcd1602_move_cursor(infolcd, 10, 2);
      i2c_lcd1602_write_string(infolcd, humi_config_t);
      i2c_lcd1602_move_cursor(infolcd, 10, 3);
      i2c_lcd1602_write_string(infolcd, Time_config_t);

      i2c_lcd1602_move_cursor(infolcd, 17, 3);
      i2c_lcd1602_write_string(infolcd, Status_t);

      keypad_row_en[u8Count]();
      key = waitKey(u8Count);
      //=================================================
      if (lc == 0)
      {
         if (key == '#')
         {
            temp_config++;
         }
         else if (key == '*')
         {
            temp_config--;
         }

         sprintf(temp_config_t, "%d", temp_config);
         // printf("%d", temp_config);
         i2c_lcd1602_move_cursor(infolcd, 10, 1);

         i2c_lcd1602_write_string(infolcd, temp_config_t);
         i2c_lcd1602_write_string(infolcd, "   ");

         if (key == 'D')
            lc = 1;
      }
      //======================================
      else if (lc == 1)
      {
         if (key == '#')
         {
            humi_config++;
         }
         else if (key == '*')
         {
            humi_config--;
            if (humi_config < 0)
               humi_config = 0;
         }
         sprintf(humi_config_t, "%d", humi_config);
         // printf("%d", humi_config);
         i2c_lcd1602_move_cursor(infolcd, 10, 2);

         i2c_lcd1602_write_string(infolcd, humi_config_t);
         i2c_lcd1602_write_string(infolcd, "   ");

         if (key == 'D')
         {
            lc = 2;
            // mode = AUTO_MODE;
            // break;
         }
      }
      //======================================
      else if (lc == 2)
      {
         if (key == '#')
         {
            Time_config++;
         }
         else if (key == '*')
         {
            Time_config--;
            if (Time_config < 0)
               Time_config = 0;
         }
         if (key == '1')
            Status = 1;
         else if (key == 'o')
            Status = 0;
         sprintf(Time_config_t, "%d", Time_config);
         // printf("%d", humi_config);
         i2c_lcd1602_move_cursor(infolcd, 10, 3);

         i2c_lcd1602_write_string(infolcd, Time_config_t);
         i2c_lcd1602_write_string(infolcd, "   ");

         if (key == 'D')
         {

            mode = AUTO_MODE;
            break;
         }
      }
      //==================chuyen mode =============================
      if (key == 'A')
      {

         mode = MANUAL_MODE;
      }

      if (key == 'B')
      {

         mode = AUTO_MODE;
      }

      if (mode == MANUAL_MODE || mode == AUTO_MODE)
      {
         break;
      }
      if (++u8Count == 4)
      {
         u8Count = 0;
      }
      vTaskDelay(20 / portTICK_PERIOD_MS);
   }
}
void Auto_mode(i2c_lcd1602_info_t *infolcd)
{
   xTimerStart(xTimers[0], 0);
   struct dht11_reading *newcmd;

   i2c_lcd1602_clear(infolcd);
   i2c_lcd1602_move_cursor(infolcd, 0, 1);
   i2c_lcd1602_write_string(infolcd, "Nhiet Do:");
   i2c_lcd1602_move_cursor(infolcd, 0, 2);
   i2c_lcd1602_write_string(infolcd, "Do am kk:");
   i2c_lcd1602_move_cursor(infolcd, 0, 3);
   i2c_lcd1602_write_string(infolcd, "Do am dat:");
   i2c_lcd1602_move_cursor(infolcd, 0, 0);
   i2c_lcd1602_write_string(infolcd, "[A]-MANUAL/[C]CONFIG");
   while (1)
   {

      xQueueReceive(xQueue_nhietdo, (void *)&newcmd, (TickType_t)0);
      sprintf(temp, "%.1f", newcmd->temperature);
      sprintf(humi, "%.1f%%", newcmd->humidity);
      // sprintf(temp,"%.1f*C",dht11_last_data.temperature);
      // sprintf(humi,"%.1f%%",dht11_last_data.humidity);

      i2c_lcd1602_move_cursor(infolcd, 10, 1);
      i2c_lcd1602_write_string(infolcd, temp);
      i2c_lcd1602_move_cursor(infolcd, 10, 2);
      i2c_lcd1602_write_string(infolcd, humi);
      i2c_lcd1602_move_cursor(infolcd, 10, 3);
      i2c_lcd1602_write_string(infolcd, "20");
      keypad_row_en[u8Count]();
      key = waitKey(u8Count);

      if (newcmd->temperature > temp_config && bom == 0)
      {
         bom = 1;
         BAT_QUAT;
         printf("bat quat len nao\n");
      }
      else if (newcmd->temperature < temp_config && bom == 1)
      {
         bom = 0;
         TAT_QUAT;
         printf("tat quat\n");
      }

      if (newcmd->humidity > humi_config && quat == 0)
      {
         bom = 1;
         BAT_BOM;
         printf("bat BOM len nao\n");
      }
      else if (newcmd->humidity < humi_config && quat == 1)
      {
         quat = 0;
         TAT_BOM;
         printf("tat BOM\n");
      }

      // =======================chuyen mode============================================
      if (key == 'A')
         mode = MANUAL_MODE;
      if (key == 'C')
         mode = CONFIG_MODE;
      if (mode == MANUAL_MODE || mode == CONFIG_MODE)
      {
         break;
      }
      if (++u8Count == 4)
      {
         u8Count = 0;
      }
      vTaskDelay(20 / portTICK_PERIOD_MS);
   }
}

void lcd1602_task(void *pvParameter)
{

   // Set up I2C
   i2c_master_init();
   i2c_port_t i2c_num = I2C_MASTER_NUM;
   uint8_t address = 0x27;
   printf("chao\n");
   // Set up the SMBus
   smbus_info_t *smbus_info = smbus_malloc();
   ESP_ERROR_CHECK(smbus_init(smbus_info, i2c_num, address));
   ESP_ERROR_CHECK(smbus_set_timeout(smbus_info, 1000 / portTICK_RATE_MS));

   // Set up the LCD1602 device with backlight off
   i2c_lcd1602_info_t *lcd_info = i2c_lcd1602_malloc();
   ESP_ERROR_CHECK(i2c_lcd1602_init(lcd_info, smbus_info, true,
                                    LCD_NUM_ROWS, LCD_NUM_COLUMNS, LCD_NUM_VISIBLE_COLUMNS));

   ESP_ERROR_CHECK(i2c_lcd1602_reset(lcd_info));

   // turn off backlight
   ESP_LOGI(TAG, "backlight off");
   //_wait_for_user();
   i2c_lcd1602_set_backlight(lcd_info, false);

   // turn on backlight
   ESP_LOGI(TAG, "backlight on");
   i2c_lcd1602_set_backlight(lcd_info, true);
   ESP_LOGI(TAG, "cursor on");
   i2c_lcd1602_set_cursor(lcd_info, false);
   while (1)
   {
      Interface = CHECK_MODE[mode];
      Interface(lcd_info);
   }
}
void Dht11_task(void *pvParameter)
{
   struct dht11_reading *newcmd;
   newcmd = &dht11_last_data;
   // newcmd =(struct dht11_reading*)pvPortMalloc(sizeof(struct dht11_reading));
   while (1)
   {
      dht11_cur_data = DHT11_read();
      xQueueSend(xQueue_nhietdo, &newcmd, (TickType_t)0);
      if (dht11_cur_data.status == 0) // read oke
      {
         dht11_last_data = dht11_cur_data;
         // printf(" %.1f   %.1f\n", dht11_last_data.humidity, dht11_last_data.temperature);
         printf(": %.1f   %.1f\n", newcmd->humidity, newcmd->temperature);
      }
      else
      {
         printf("dht11 read fail\n");
      }
      vTaskDelay(1000 / portTICK_PERIOD_MS);
   }
}
void Time_Setup_task(void *pvParameter)
{
   uint64_t __stop, Time_ms, Time_base;
   while (1)
   {
      xSemaphoreTake(xSemaphore, portMAX_DELAY);
      uint64_t tick = xTaskGetTickCount();

      printf("bat bom , bat quat\n");
      printf("quat: %d\n,",quat);
      printf("bom: %d\n,",bom);
      BAT_BOM;
      BAT_QUAT;
      while (1)
      {
         __stop = xTaskGetTickCount();
         Time_base = __stop - tick;
         Time_ms = Time_base * portTICK_PERIOD_MS;
         if (Time_ms > 3000)
         {
            printf("tat bom ,Tat quat\n");
            if (quat == 1 && bom==0 )TAT_BOM;
            if (quat == 0 && bom==1 )TAT_QUAT;
            if (bom == 0 && quat ==0)
            {
                  TAT_BOM;
                  TAT_QUAT;
            } 
            // else
            // {
            //    TAT_QUAT;
            //    TAT_BOM;
            // }
            // if (bom == 1)TAT_QUAT;
            // else
            // {
            //    TAT_QUAT;
            //    TAT_BOM;
            // }
            // TAT_QUAT;
            break;
         }

         vTaskDelay(50 / portTICK_PERIOD_MS);
      }
      vTaskDelay(50 / portTICK_PERIOD_MS);
   }
}
void vTimerCallback(TimerHandle_t xTimer)
{

   configASSERT(xTimer);
   // printf("timer\n");
   if (Status == 1)
   {
      ulCount++;
      printf("%d\n", ulCount);
      if (ulCount == Time_config)
      {
         xSemaphoreGive(xSemaphore);
         printf("givesema\n");
         ulCount = 0;
      }
   }
}

void app_main()
{
   xTimers[0] = xTimerCreate("Timer", pdMS_TO_TICKS(1000), pdTRUE, (void *)0, vTimerCallback);
   xSemaphore = xSemaphoreCreateBinary();
   // gpio_pad_select_gpio(GPIO_NUM_12);   // bom
   // /* Set the GPIO as a push/pull output */
   // gpio_set_direction(GPIO_NUM_12, GPIO_MODE_OUTPUT);
   // gpio_pad_select_gpio(GPIO_NUM_14);    // quat
   // /* Set the GPIO as a push/pull output */
   // gpio_set_direction(GPIO_NUM_14, GPIO_MODE_OUTPUT);
   gpio_pad_select_gpio(GPIO_NUM_25); // bom
   /* Set the GPIO as a push/pull output */
   gpio_set_direction(GPIO_NUM_25, GPIO_MODE_OUTPUT);
   gpio_pad_select_gpio(GPIO_NUM_33); // quat
   /* Set the GPIO as a push/pull output */
   gpio_set_direction(GPIO_NUM_33, GPIO_MODE_OUTPUT);
   TAT_BOM;
   TAT_QUAT;
   DHT11_init(GPIO_NUM_19);
   KeyBoard_Init();
   xQueue_nhietdo = xQueueCreate(1, sizeof(struct dht11_reading *));
   if (xQueue_nhietdo != NULL)
   {
      xTaskCreate(&lcd1602_task, "lcd1602_task", 4096, NULL, 5, NULL);
      xTaskCreate(&Dht11_task, "dht11", 4096, NULL, 6, NULL);
      xTaskCreate(&Time_Setup_task, "Alarm", 4096, NULL, 5, NULL);
      // xTimerStart(xTimers[0], 0);
   }
}
