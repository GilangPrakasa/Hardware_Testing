#include "Arduino.h"
#include "Freenove_WS2812_Lib_for_ESP32.h"
#include "ds1307.h"
#include "SerialFlash.h"
#include "i2cdev.h"
#include "Flash_AVR/ElegantOTA.h"
#include "Peripheral/motor_controller.h"
#include "Peripheral/runbtn.h"
#include "WiFi.h"
#include "SerialFlash.h"
#include "ext_wdt.h"

#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "stdio.h"
#include "sdkconfig.h"
#include "Peripheral/lcd.h"
#include "Peripheral/LiquidCrystal_I2C.h"
#include "Flash_AVR/eventbus.h"

// SET ADDRESS & GPIO
#define RGB_LED         4
#define Beeper          12
#define R_Button        27
#define BTN0            32
#define BTN1            36
#define BTN2            13
#define BTN3            14
#define Flash_CS        5
#define GPIO_I2C_SDA    21
#define GPIO_I2C_SCL    22
#define LCD_ADDR        0x38
#define LCD_COLS        20
#define LCD_ROWS        4
// COLOR LED
#define BLANK 0
#define ORANGE 1 //check avr
#define CORAL 2 //check wdt
#define MAGENTA 3 //check point
#define YELLOW 4 //check rtc
#define BLUE 5 //check nor-flash
#define GREEN 6 //check rssi
#define WHITE 7 //check pmw 30
#define ROSE 8 //check pmw 70
#define RED 9 //check pmw 100
#define CYAN 10 //check complete
// STATE
#define check_button 11
#define check_wdt 12
#define check_point 13

LiquidCrystal_I2C lcd(LCD_ADDR,LCD_COLS,LCD_ROWS);
Freenove_ESP32_WS2812 strip = Freenove_ESP32_WS2812(1, RGB_LED, 0, TYPE_GRB);
ElegantOtaClass ElegantOTA_X;
SerialFlashFile file;

const unsigned long testIncrement = 4096;

unsigned long pressedTime  = 900000; //20000
unsigned long releasedTime = 0;
unsigned long startDuration = 0;
unsigned long endDuration = 0;
int pressCount = 0;
int lastState = 0;
const int LONG_PRESS_TIME = 3000;
int flag = 0;

String t_task = "STARTING";
String t_cmd = "QC PERIPHERAL CHECK";
String t_note = "";
String SSID = "SSID NOT FOUND";
String value_rssi = "--------------";

void lcd_display(void* param);
void btn_display(void* param);

const char* welcomeMsg =
    R"(    
        * ████┼ ███ ┼┼ ███ ███ ███ █▄┼▄█ █┼┼┼█ ███ ███ ███
        * █┼┼█┼ █┼┼ ┼┼ █▄┼ ┼█┼ █▄┼ █┼█┼█ █┼█┼█ █▄█ █▄┼ █▄┼
        * █▄██▄ ███ ┼┼ █┼┼ ▄█▄ █┼█ █┼┼┼█ █▄█▄█ █┼█ █┼█ █▄▄
    )";
    
const char * id2chip(const unsigned char *id)
{
	if (id[0] == 0xEF) {
		// Winbond
		if (id[1] == 0x40) {
			if (id[2] == 0x14) return "W25Q80BV";
			if (id[2] == 0x15) return "W25Q16DV";
			if (id[2] == 0x17) return "W25Q64FV";
			if (id[2] == 0x18) return "W25Q128FV";
			if (id[2] == 0x19) return "W25Q256FV";
		}
	}
	if (id[0] == 0x01) {
		// Spansion
		if (id[1] == 0x02) {
			if (id[2] == 0x16) return "S25FL064A";
			if (id[2] == 0x19) return "S25FL256S";
			if (id[2] == 0x20) return "S25FL512S";
		}
		if (id[1] == 0x20) {
			if (id[2] == 0x18) return "S25FL127S";
		}
	}
	if (id[0] == 0xC2) {
		// Macronix
		if (id[1] == 0x20) {
			if (id[2] == 0x18) return "MX25L12805D";
		}
	}
	if (id[0] == 0x20) {
		// Micron
		if (id[1] == 0xBA) {
			if (id[2] == 0x20) return "N25Q512A";
			if (id[2] == 0x21) return "N25Q00AA";
		}
		if (id[1] == 0xBB) {
			if (id[2] == 0x22) return "MT25QL02GC";
		}
	}
	if (id[0] == 0xBF) {
		// SST
		if (id[1] == 0x25) {
			if (id[2] == 0x02) return "SST25WF010";
			if (id[2] == 0x03) return "SST25WF020";
			if (id[2] == 0x04) return "SST25WF040";
			if (id[2] == 0x41) return "SST25VF016B";
			if (id[2] == 0x4A) return "SST25VF032";
		}
		if (id[1] == 0x25) {
			if (id[2] == 0x01) return "SST26VF016";
			if (id[2] == 0x02) return "SST26VF032";
			if (id[2] == 0x43) return "SST26VF064";
		}
	}
  	if (id[0] == 0x1F) {
    		// Adesto
   		if (id[1] == 0x89) {
      			if (id[2] == 0x01) return "AT25SF128A";
    		}  
  	} 	
	return "(unknown chip)";
}

void create_signature(unsigned long address, unsigned char *data)
{
	data[0] = address >> 24;
	data[1] = address >> 16;
	data[2] = address >> 8;
	data[3] = address;
	unsigned long hash = 2166136261ul;
	for (unsigned char i=0; i < 4; i++) {
		hash ^= data[i];
		hash *= 16777619ul;
	}
	data[4] = hash;
	data[5] = hash >> 8;
	data[6] = hash >> 16;
	data[7] = hash >> 24;
}

bool equal_signatures(const unsigned char *data1, const unsigned char *data2)
{
	for (unsigned char i=0; i < 8; i++) {
		if (data1[i] != data2[i]) return false;
	}
	return true;
}

bool is_erased(const unsigned char *data, unsigned int len)
{
	while (len > 0) {
		if (*data++ != 255) return false;
		len = len - 1;
	}
	return true;
}

void printbuf(const void *buf, uint32_t len)
{
  const uint8_t *p = (const uint8_t *)buf;
  do {
    unsigned char b = *p++;
    Serial.print(b >> 4, HEX);
    Serial.print(b & 15, HEX);
    //Serial.printf("%02X", *p++);
    Serial.print(' ');
  } while (--len > 0);
  Serial.println();
}

bool test_nor_flash() 
{
  unsigned char buf[256], sig[256], buf2[8];
  unsigned long address, count, chipsize, blocksize;
  unsigned long usec;
  bool first;

  Serial.println();
  Serial.println(F("Read Chip Identification:"));
  SerialFlash.readID(buf);
  Serial.print(F("  JEDEC ID:     "));
  Serial.print(buf[0], HEX);
  Serial.print(' ');
  Serial.print(buf[1], HEX);
  Serial.print(' ');
  Serial.println(buf[2], HEX);
  Serial.print(F("  Part Number: "));
  Serial.println(id2chip(buf));
  Serial.print(F("  Memory Size:  "));
  chipsize = SerialFlash.capacity(buf);
  Serial.print(chipsize);
  Serial.println(F(" bytes"));
  if (chipsize == 0) return false;
  Serial.print(F("  Block Size:   "));
  blocksize = SerialFlash.blockSize();
  Serial.print(blocksize);
  Serial.println(F(" bytes"));

  Serial.println();
  Serial.println(F("Reading Chip..."));
  memset(buf, 0, sizeof(buf));
  memset(sig, 0, sizeof(sig));
  memset(buf2, 0, sizeof(buf2));
  address = 0;
  count = 0;
  first = true;
  while (address < chipsize) {
    SerialFlash.read(address, buf, 8);
    create_signature(address, sig);
    if (is_erased(buf, 8) == false) {
      if (equal_signatures(buf, sig) == false) {
        Serial.print(F("  Previous data found at address "));
        Serial.println(address);
        Serial.println(F("  You must fully erase the chip before this test"));
        Serial.print(F("  found this: "));
        printbuf(buf, 8);
        Serial.print(F("     correct: "));
        printbuf(sig, 8);
        return false;
      }
    } else {
      count = count + 1; // number of blank signatures
    }
    if (first) {
      address = address + (testIncrement - 8);
      first = false;
    } else {
      address = address + 8;
      first = true;
    }
  }
  if (count > 0) {
    Serial.println();
    Serial.print(F("Writing "));
    Serial.print(count);
    Serial.println(F(" signatures"));
    memset(buf, 0, sizeof(buf));
    memset(sig, 0, sizeof(sig));
    memset(buf2, 0, sizeof(buf2));
    address = 0;
    first = true;
    while (address < chipsize) {
      SerialFlash.read(address, buf, 8);
      if (is_erased(buf, 8)) {
        create_signature(address, sig);
        SerialFlash.write(address, sig, 8);
        while (!SerialFlash.ready()) ; // wait
        SerialFlash.read(address, buf, 8);
        if (equal_signatures(buf, sig) == false) {
          Serial.print(F("  error writing signature at "));
          Serial.println(address);
          Serial.print(F("  Read this: "));
          printbuf(buf, 8);
          Serial.print(F("  Expected:  "));
          printbuf(sig, 8);
          return false;
        }
      }
      if (first) {
        address = address + (testIncrement - 8);
        first = false;
      } else {
        address = address + 8;
        first = true;
      }
    }
  } else {
    Serial.println(F("  all signatures present from prior tests"));
  }
  Serial.println();
  Serial.println(F("Double Checking All Signatures:"));
  memset(buf, 0, sizeof(buf));
  memset(sig, 0, sizeof(sig));
  memset(buf2, 0, sizeof(buf2));
  count = 0;
  address = 0;
  first = true;
  while (address < chipsize) {
    SerialFlash.read(address, buf, 8);
    create_signature(address, sig);
    if (equal_signatures(buf, sig) == false) {
      Serial.print(F("  error in signature at "));
      Serial.println(address);
      Serial.print(F("  Read this: "));
      printbuf(buf, 8);
      Serial.print(F("  Expected:  "));
      printbuf(sig, 8);
      return false;
    }
    count = count + 1;
    if (first) {
      address = address + (testIncrement - 8);
      first = false;
    } else {
      address = address + 8;
      first = true;
    }
  }
  Serial.print(F("  all "));
  Serial.print(count);
  Serial.println(F(" signatures read ok"));

  Serial.println();
  Serial.println(F("Checking Signature Pairs"));
  memset(buf, 0, sizeof(buf));
  memset(sig, 0, sizeof(sig));
  memset(buf2, 0, sizeof(buf2));
  count = 0;
  address = testIncrement - 8;
  first = true;
  while (address < chipsize - 8) {
    SerialFlash.read(address, buf, 16);
    create_signature(address, sig);
    create_signature(address + 8, sig + 8);
    if (memcmp(buf, sig, 16) != 0) {
      Serial.print(F("  error in signature pair at "));
      Serial.println(address);
      Serial.print(F("  Read this: "));
      printbuf(buf, 16);
      Serial.print(F("  Expected:  "));
      printbuf(sig, 16);
      return false;
    }
    count = count + 1;
    address = address + testIncrement;
  }
  Serial.print(F("  all "));
  Serial.print(count);
  Serial.println(F(" signature pairs read ok"));

  Serial.println();
  Serial.println(F("Checking Read-While-Write (Program Suspend)"));
  address = 256;
  while (address < chipsize) { // find a blank space
    SerialFlash.read(address, buf, 256);
    if (is_erased(buf, 256)) break;
    address = address + 256;
  }
  if (address >= chipsize) {
    Serial.println(F("  error, unable to find any blank space!"));
    return false;
  }
  for (int i=0; i < 256; i += 8) {
    create_signature(address + i, sig + i);
  }
  Serial.print(F("  write 256 bytes at "));
  Serial.println(address);
  Serial.flush();
  SerialFlash.write(address, sig, 256);
  usec = micros();
  if (SerialFlash.ready()) {
    Serial.println(F("  error, chip did not become busy after write"));
    return false;
  }
  SerialFlash.read(0, buf2, 8); // read while busy writing
  while (!SerialFlash.ready()) ; // wait
  usec = micros() - usec;
  Serial.print(F("  write time was "));
  Serial.print(usec);
  Serial.println(F(" microseconds."));
  SerialFlash.read(address, buf, 256);
  if (memcmp(buf, sig, 256) != 0) {
    Serial.println(F("  error writing to flash"));
    Serial.print(F("  Read this: "));
    printbuf(buf, 256);
    Serial.print(F("  Expected:  "));
    printbuf(sig, 256);
    return false;
  }
  create_signature(0, sig);
  if (memcmp(buf2, sig, 8) != 0) {
    Serial.println(F("  error, incorrect read while writing"));
    Serial.print(F("  Read this: "));
    printbuf(buf2, 256);
    Serial.print(F("  Expected:  "));
    printbuf(sig, 256);
    return false;
  }
  Serial.print(F("  read-while-writing: "));
  printbuf(buf2, 8);
  Serial.println(F("  test passed, good read while writing"));

  if (chipsize >= 262144 + blocksize + testIncrement) {
    Serial.println();
    Serial.println(F("Checking Read-While-Erase (Erase Suspend)"));
    memset(buf, 0, sizeof(buf));
    memset(sig, 0, sizeof(sig));
    memset(buf2, 0, sizeof(buf2));
    SerialFlash.eraseBlock(262144);
    usec = micros();
    delayMicroseconds(50);
    if (SerialFlash.ready()) {
      Serial.println(F("  error, chip did not become busy after erase"));
      return false;
    }
    SerialFlash.read(0, buf2, 8); // read while busy writing
    while (!SerialFlash.ready()) ; // wait
    usec = micros() - usec;
    Serial.print(F("  erase time was "));
    Serial.print(usec);
    Serial.println(F(" microseconds."));
    // read all signatures, check ones in this block got
    // erased, and all the others are still intact
    address = 0;
    first = true;
    while (address < chipsize) {
      SerialFlash.read(address, buf, 8);
      if (address >= 262144 && address < 262144 + blocksize) {
        if (is_erased(buf, 8) == false) {
          Serial.print(F("  error in erasing at "));
          Serial.println(address);
          Serial.print(F("  Read this: "));
          printbuf(buf, 8);
          return false;
        }
      } else {
        create_signature(address, sig);
        if (equal_signatures(buf, sig) == false) {
          Serial.print(F("  error in signature at "));
          Serial.println(address);
          Serial.print(F("  Read this: "));
          printbuf(buf, 8);
          Serial.print(F("  Expected:  "));
          printbuf(sig, 8);
          return false;
        }
      }
      if (first) {
        address = address + (testIncrement - 8);
        first = false;
      } else {
        address = address + 8;
        first = true;
      }
    }
    Serial.print(F("  erase correctly erased "));
    Serial.print(blocksize);
    Serial.println(F(" bytes"));
    // now check if the data we read during erase is good
    create_signature(0, sig);
    if (memcmp(buf2, sig, 8) != 0) {
      Serial.println(F("  error, incorrect read while erasing"));
      Serial.print(F("  Read this: "));
      printbuf(buf2, 256);
      Serial.print(F("  Expected:  "));
      printbuf(sig, 256);
      return false;
    }
    Serial.print(F("  read-while-erasing: "));
    printbuf(buf2, 8);
    Serial.println(F("  test passed, good read while erasing\n"));

  } else {
    Serial.println(F("Skip Read-While-Erase, this chip is too small"));
  }
  return true;
}

void setColor(uint8_t color)
{
    switch(color)
    {
        case RED:
            strip.setLedColorData(0,255,0,0);
            strip.show();
            break;
        case YELLOW:
            strip.setLedColorData(0,255,255,0);
            strip.show();
            break;
        case GREEN:
            strip.setLedColorData(0,0,255,0);
            strip.show();
            break;
        case BLUE:
            strip.setLedColorData(0,0,0,255);
            strip.show();
            break;
        case CYAN:
            strip.setLedColorData(0,0,255,255);
            strip.show();
            break;
        case MAGENTA:
            strip.setLedColorData(0,255,0,255);
            strip.show();
            break;
        case ORANGE:
            strip.setLedColorData(0,255,68,0);
            strip.show();
            break;    
        case BLANK:
            strip.setLedColorData(0,0,0,0);
            strip.show();
            break;
        case WHITE:
            strip.setLedColorData(0,255,255,255);
            strip.show();
            break;
        case ROSE:
            strip.setLedColorData(0,255,80,80);
            strip.show();
            break;
        case CORAL:
            strip.setLedColorData(0,255,127,80);
            strip.show();
            break;
    }
}

void LED_BLINK(uint8_t color,uint8_t _loop, uint8_t periodMs)
{
    for(uint8_t i = 0 ; i < _loop ; i++)
    {
        setColor(color);
        delay(periodMs);
        setColor(BLANK);
        delay(periodMs);
    }
}

void LED_FILL(uint8_t color)
{
    setColor(BLANK);
    delay(50);
    setColor(color);
}

void alarm(int loop, uint8_t color)
{
    LED_FILL(color);
    for(uint8_t i = 0; i < loop; i++)
    {
        digitalWrite(Beeper,HIGH);
        delay(100);
        digitalWrite(Beeper,LOW);
        delay(500);
    }
    LED_FILL(BLANK);
}

void beep(uint32_t periodMs,uint8_t loop)
{
    for(uint8_t i = 0; i < loop; i++)
    {
        digitalWrite(Beeper,HIGH);
        delay(periodMs);
        digitalWrite(Beeper,LOW);
        delay(periodMs);
    }
}

void button_case(uint8_t task)
{
  switch(task)
  {
    case check_button:
      ESP_LOGW("RUN BUTTON", "CHECKING RUN BUTTON");
      ESP_LOGI("STATE", "CLICK 10x FOR CHECK BUTTON / LONG PRESS FOR FLASH AVR");
      for(;;) 
      {
        int State_button = digitalRead(R_Button);
        if(lastState == 1 && State_button == LOW)
        {
          pressCount++;
          pressedTime = millis();
          ESP_LOGI("RUN BUTTON", "RUN BUTTON PRESSED %d",pressCount);
          LED_FILL(BLANK);
          t_cmd = "PRESSED";t_note = pressCount;
          beep(100,1);
          if (pressCount >= 10)
          {
            pressedTime = 900000;
            releasedTime = 0;
            pressCount = 0;
            lastState = 0;
            flag = 0;
            break;
          }  
        }
        if(lastState == 0 && State_button == HIGH) 
        {
          releasedTime = millis();
          LED_FILL(ORANGE);
          long pressDuration = releasedTime - pressedTime;
          if( pressDuration > LONG_PRESS_TIME)
          {
            t_task = "STARTING FLASH AVR";t_cmd = "ON PROGRESS";t_note = "...........";
            ESP_LOGW("AVR", "STARTING FLASH AVR");
            LED_BLINK(ORANGE,5,1000);
            vTaskDelay(pdMS_TO_TICKS(1000));
            ElegantOTA.flashImage();
            Serial.println(response);
            if (response == "Failed to connect" )
            {
              t_task = "FLASH AVR";t_cmd = "NOT COMPLETE";t_note = response;
              alarm(10,ORANGE);
              ESP.restart();
            }
            t_task = "FLASH AVR";t_cmd = "COMPLETE";t_note = response;
            ESP_LOGW("AVR", "FLASH AVR COMPLETE");
            LED_BLINK(ORANGE,5,1000);
            ESP.restart();
          }
        }
        lastState = State_button;
        delay(10);
      }
      break;
    case check_wdt:
      ESP_LOGW("WDT", "CHECKING WDT");
      ESP_LOGI("STATE", "CLICK 3x FOR SKIP CHECK WDT / LONG PRESS FOR CHECK WDT");
      for(;;) 
      {
        int State_button = digitalRead(R_Button);
        if(lastState == 1 && State_button == LOW)
        {
          pressCount++;
          pressedTime = millis();
          LED_FILL(BLANK);
          beep(100,1);
          if (pressCount >= 3)
          {
            pressedTime = 900000;
            releasedTime = 0;
            pressCount = 0;
            lastState = 0;
            flag = 1;
            break;
          }  
        }
        if(lastState == 0 && State_button == HIGH) 
        {
          releasedTime = millis();
          LED_FILL(CORAL);
          long pressDuration = releasedTime - pressedTime;
          if( pressDuration > LONG_PRESS_TIME)
          {
            t_task = "CHECKING WDT";t_cmd = "ON PROGRESS";t_note = "WAIT UNTIL RESTART";
            ESP_LOGW("WDT", "CHECKING WDT, WAIT UNTIL RESTART");
            LED_BLINK(CORAL,5,1000);
            vTaskDelay(pdMS_TO_TICKS(1000));
            ext_wdt_stop();
            vTaskDelay(pdMS_TO_TICKS(3000));
            t_task = "CHECKING WDT";t_cmd = "NOT COMPLETE";t_note = "WDT FAILURE";
            ESP_LOGW("WDT", "CHECKING WDT FAILED");
            alarm(10,CORAL);
          }
        }
        lastState = State_button;
        delay(10);
      }
      break;
    case check_point:
      ESP_LOGW("STATE", "CLICK 3x FOR NEXT TEST / LONG PRESS FOR REPEAT TEST");
      for(;;) 
      {
        int State_button = digitalRead(R_Button);
        if(lastState == 1 && State_button == LOW)
        {
          pressCount++;
          pressedTime = millis();
          LED_FILL(BLANK);
          beep(100,1);
          if (pressCount >= 3)
          {
            pressedTime = 900000;
            releasedTime = 0;
            pressCount = 0;
            lastState = 0;
            flag = 0;
            break;
          }  
        }
        if(lastState == 0 && State_button == HIGH) 
        {
          releasedTime = millis();
          LED_FILL(MAGENTA);
          long pressDuration = releasedTime - pressedTime;
          if( pressDuration > LONG_PRESS_TIME)
          {
            pressedTime = 900000;
            releasedTime = 0;
            pressCount = 0;
            lastState = 0;
            flag = 1;
            break;
          }
        }
        lastState = State_button;
        delay(10);
      }
      break;
  }
}

void check_led()
{
    ESP_LOGW("RGB-LED", "CHECKING RGB LED");
    LED_FILL(RED);
    vTaskDelay(pdMS_TO_TICKS(2000));
    LED_FILL(GREEN);
    vTaskDelay(pdMS_TO_TICKS(2000));
    LED_FILL(BLUE);
    vTaskDelay(pdMS_TO_TICKS(2000));
    LED_FILL(YELLOW);
    vTaskDelay(pdMS_TO_TICKS(2000));
    LED_FILL(CYAN);
    vTaskDelay(pdMS_TO_TICKS(2000));
    LED_FILL(MAGENTA);
    vTaskDelay(pdMS_TO_TICKS(2000));
    LED_FILL(ORANGE);
    vTaskDelay(pdMS_TO_TICKS(2000));
    LED_FILL(WHITE);
    vTaskDelay(pdMS_TO_TICKS(2000));
}

void check_rtc()
{
    ESP_LOGW("RTC", "CHECKING RTC");
    ESP_LOGI("RTC", "WAIT FOR 10 SECONDS");
    static i2c_dev_t dev;
    struct tm time = {
            time.tm_sec  = 0,
            time.tm_min  = 0,
            time.tm_hour = 12,
            time.tm_mday = 12,
            time.tm_mon  = 12,
            time.tm_year = 2023
        };

    esp_err_t ret = ds1307_init_desc(&dev, 0, (gpio_num_t)GPIO_I2C_SDA, (gpio_num_t)GPIO_I2C_SCL);
    if (ret != ESP_OK) {
        alarm(10,YELLOW);
        ESP_LOGE("RTC", "FAILED TO INIT RTC");
        return;
    }

    ds1307_set_time(&dev, &time);
    if (ret != ESP_OK) {
        alarm(10,YELLOW);   
        ESP_LOGE("RTC", "FAILED TO SET TIME RTC");
        return;
    }
    ESP_LOGI("RTC","SET time: %d-%02d-%02d %02d:%02d:%02d", time.tm_year, time.tm_mon, time.tm_mday,
           time.tm_hour, time.tm_min, time.tm_sec);

    vTaskDelay(pdMS_TO_TICKS(10000));

    ds1307_get_time(&dev, &time);
    if (ret != ESP_OK) {
        alarm(10,YELLOW);
        ESP_LOGE("RTC", "FAILED GET TIME RTC");
        return;
    }
    ESP_LOGI("RTC","GET time: %d-%02d-%02d %02d:%02d:%02d", time.tm_year+1792, time.tm_mon, time.tm_mday,
           time.tm_hour, time.tm_min, time.tm_sec);

    if (time.tm_sec < 9 || time.tm_sec > 10) 
    {
        ESP_LOGE("RTC", "FAILURE RTC");
        ESP_LOGE("INFO", "RESTART PROCESS");
        t_task = "RTC";t_cmd = "NOT COMPLETE";t_note = "RTC FAILURE ";
        alarm(10,YELLOW);
        ESP.restart();
    }else{
        ESP_LOGW("RTC", "RTC OK");
        t_task = "RTC";t_cmd = "COMPLETE";t_note = "RTC OK";
        beep(100,3);
        LED_BLINK(YELLOW,3,1000);
    }
}

void check_nor_flash() 
{
  ESP_LOGW("NOR-FLASH", "CHECKING NOR-FLASH");
    SerialFlash.eraseAll();
    if (test_nor_flash()) {
      ESP_LOGW("NOR-FLASH", "NOR FLASH OK");
      t_task = "NOR-FLASH";t_cmd = "COMPLETE";t_note = "NOR-FLASH OK";
      beep(100,3);
      LED_BLINK(BLUE,3,1000);
    } else {
      ESP_LOGE("NOR-FLASH", "NOR FLASH ERROR");
      t_task = "NOR-FLASH";t_cmd = "NOT COMPLETE";t_note = "NOR-FLASH FAILURE";
      alarm(10,BLUE);
      ESP.restart();
    }
    SerialFlash.eraseAll();
}

void rssi_level()
{
    int n = WiFi.scanNetworks();
    const char* SSID_search = "AksesPoin_eFishery";
    String mac = WiFi.macAddress();
    ESP_LOGW("WIFI", "MAC ADDRESS: %s", mac.c_str());
    if (n == 0) {
        ESP_LOGI("WIFI", "no networks found");
    } else {
        ESP_LOGI("WIFI", "%d networks found", n);
        for (int i = 0; i < n; ++i) {
            String quality;
            vTaskDelay(10 / portTICK_PERIOD_MS);
            if (WiFi.RSSI(i) >= -50 && WiFi.RSSI(i) < 0) {
                quality = "Excellent";
            } else if (WiFi.RSSI(i) >= -60 && WiFi.RSSI(i) < -50) {
                quality = "Very Good";
            } else if (WiFi.RSSI(i) >= -70 && WiFi.RSSI(i) < -60) {
                quality = "Good";
            } else if (WiFi.RSSI(i) >= -80 && WiFi.RSSI(i) < -70) {
                quality = "Low";
            } else if (WiFi.RSSI(i) >= -90 && WiFi.RSSI(i) < -80) {
                quality = "Very Low";
            } else {
                quality = "No Signal";
            }
            ESP_LOGW("WIFI","%s: %d dBm [%s]",WiFi.SSID(i).c_str(), WiFi.RSSI(i),quality.c_str());
        }
    }
    for (int i = 0; i < n; i++) {
      if (strstr(WiFi.SSID(i).c_str(), SSID_search) != NULL) {
        value_rssi = WiFi.RSSI(i);
        SSID = WiFi.SSID(i);
      }
    }
    int rssi = value_rssi.toFloat();
    if (SSID != "AksesPoin_eFishery" or rssi <= -70){
      ESP_LOGI("RSSI-LEVEL", "SSID NOT FOUND");
      alarm(10,GREEN);
    }
    else{
      beep(100,3);
      LED_BLINK(GREEN,3,1000); 
    }
    t_task = "RSSI-LEVEL";t_cmd = SSID ; t_note = rssi;
      
}

void check_motor()
{
    initrunbtn();
    initMotorController(0);
    int duty_select = getcounter();
    int last_select = duty_select;
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGW("AVR", "CHECKING TEST RUN");
    ESP_LOGI("INFO", "CLICK BUTTON FOR RUNNING MOTOR (MAX 3x RUN)");
    t_task = "RUNNING PWM 30";t_cmd = "CLICK BTN TO START", t_note =" ";
    for(;;) 
    {
        duty_select = getcounter();
        if (last_select != duty_select) 
        {
            last_select = duty_select;
            switch(duty_select){
                case 1:
                    t_task = "RUNNING PWM 30";t_cmd = "CLICK BTN TO STOP";
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    beep(200,2);
                    LED_FILL(WHITE);
                    th_pwm1en1(30);
                    vTaskDelay(2000 / portTICK_RATE_MS);
                    dos_pwm1en1();
                    ESP_LOGI("STATE", "PWM 30 START %d", getcounter());
                    break;
                case 0:
                    pressCount++;
                    beep(50,5);
                    LED_FILL(ORANGE);
                    dos_pwm0en0();                 
                    vTaskDelay(2000 / portTICK_RATE_MS);
                    th_pwm0en0();
                    ESP_LOGI("STATE", "PWM 30 STOP %d", getcounter());
                    vTaskDelay(pdMS_TO_TICKS(2000));
                    t_task = "RUNNING PWM 70";t_cmd = "CLICK BTN TO START";
                    goto PWM_70;
                }
        }
        vTaskDelay(50 / portTICK_RATE_MS);     
    }
    PWM_70:
    vTaskDelay(pdMS_TO_TICKS(2000));
    for(;;) 
    {
        duty_select = getcounter();
        if (last_select != duty_select) 
        {
            last_select = duty_select;
            switch(duty_select){
                case 1:
                    t_task = "RUNNING PWM 70";t_cmd = "CLICK BTN TO STOP";
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    beep(200,2);
                    LED_FILL(ROSE);
                    th_pwm1en1(70);
                    vTaskDelay(2000 / portTICK_RATE_MS);
                    dos_pwm1en1();
                    ESP_LOGI("STATE", "PWM 70 START %d", getcounter());
                    break;
                case 0:
                    pressCount++;
                    beep(50,5);
                    LED_FILL(ORANGE);
                    dos_pwm0en0();                 
                    vTaskDelay(2000 / portTICK_RATE_MS);
                    th_pwm0en0();
                    ESP_LOGI("STATE", "PWM 70 STOP %d", getcounter());
                    vTaskDelay(pdMS_TO_TICKS(2000));
                    t_task = "RUNNING PWM 100";t_cmd = "CLICK BTN TO START";
                    goto PWM_100;
                }
        }
        vTaskDelay(50 / portTICK_RATE_MS);            
    }
    PWM_100:
    vTaskDelay(pdMS_TO_TICKS(2000));
    for(;;) 
    {
        duty_select = getcounter();
        if (last_select != duty_select) 
        {
            last_select = duty_select;
            switch(duty_select){
                case 1:
                    t_task = "RUNNING PWM 100";t_cmd = "CLICK BTN TO STOP";
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    beep(200,2);
                    LED_FILL(RED);
                    th_pwm1en1(100);
                    vTaskDelay(2000 / portTICK_RATE_MS);
                    dos_pwm1en1();
                    ESP_LOGI("STATE", "PWM 100 START %d", getcounter());
                    break;
                case 0:
                    pressCount++;
                    beep(50,5);
                    LED_FILL(ORANGE);
                    dos_pwm0en0();                 
                    vTaskDelay(2000 / portTICK_RATE_MS);
                    th_pwm0en0();
                    ESP_LOGI("STATE", "PWM 100 STOP %d", getcounter());
                    vTaskDelay(pdMS_TO_TICKS(2000));
                    break;
                }
        }
        vTaskDelay(50 / portTICK_RATE_MS);
        if (pressCount >=3)
        {
            break;
        }            
    }
}

extern "C" void app_main()
{
    // SETUP
    initArduino();
    ext_wdt_start(500);
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    vTaskDelay(pdMS_TO_TICKS(100));
    pinMode(Beeper, OUTPUT);
    pinMode(R_Button,INPUT_PULLUP);
    pinMode(BTN0,INPUT_PULLUP);
    pinMode(BTN1,INPUT_PULLUP);
    pinMode(BTN2,INPUT_PULLUP);
    pinMode(BTN3,INPUT_PULLUP);
    digitalWrite(Beeper, LOW);
    esp_err_t err = i2cdev_init();
    if (err != ESP_OK) 
    {
      ESP_LOGE("I2C", "Cannot initialize i2cdev");
      alarm(10,RED);
    }
    lcd_start();
    vTaskDelay(pdMS_TO_TICKS(200));
    
    xTaskCreate(lcd_display, "LcdTask", 2048, NULL, 2, NULL);
    xTaskCreatePinnedToCore(btn_display, "ButtonTask1", 4096, (void*)BTN0, 1, NULL, 0);
    xTaskCreatePinnedToCore(btn_display, "ButtonTask2", 4096, (void*)BTN1, 1, NULL, 0);
    xTaskCreatePinnedToCore(btn_display, "ButtonTask3", 4096, (void*)BTN2, 1, NULL, 0);
    xTaskCreatePinnedToCore(btn_display, "ButtonTask4", 4096, (void*)BTN3, 1, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(200));

    strip.begin();
    strip.setBrightness(200);
    SerialFlash.begin(Flash_CS);
    ESP_LOGW("MSG","\r\n %s", welcomeMsg);
    vTaskDelay(pdMS_TO_TICKS(1000));

    FlashAVR:
    t_task = "R_BUTTON / FLASH AVR";t_cmd = "CLICK 3X FOR NEXT";t_note = "LONG PRESS FOR AVR";
    vTaskDelay(pdMS_TO_TICKS(1000));
    LED_FILL(MAGENTA);
    button_case(check_button);
    ESP_LOGI("STATE", "BUTTON CHECKED");

    // WDT
    t_task = "WDT";t_cmd = "CLICK 3X FOR NEXT";t_note = "LONG PRESS FOR WDT";
    vTaskDelay(pdMS_TO_TICKS(1000));
    LED_FILL(MAGENTA);
    button_case(check_wdt);
    ESP_LOGI("STATE", "BUTTON CHECKED");
    
    t_task = "DISPLAY BUTTON";t_cmd = "TEST DISPLAY BUTTON";t_note = "RUN BUTTON FOR NEXT";
    ESP_LOGW("DISPLAY BUTTON", "TEST DISPLAY BUTTON / Click 3x RUN BUTTON FOR NEXT");
    button_case(check_point);
    if (flag == 1){
      ESP_LOGI("STATE", "BACK STATE");flag = 0;goto FlashAVR;
    }else{
      ESP_LOGI("STATE", "NEXT STATE");
    }
    ESP_LOGI("STATE", "DISPLAY BUTTON CHECKED");

    BUZZER:
    t_task = "BUZZER";t_cmd = "LISTEN TO BUZZER";
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGW("Buzzer", "CHECKING BUZZER");
    beep(250,10);
    ESP_LOGI("STATE", "BUZZER CHECKED");
    
    button_case(check_point);
    if (flag == 1){
      ESP_LOGI("STATE", "BACK STATE");flag = 0;goto BUZZER;
    }else{
      ESP_LOGI("STATE", "NEXT STATE");
    }

    LED:
    t_task = "LED RGB";t_cmd = "LIST COLOR :";t_note = "R,G,B,Y,CY,M,O,W";
    vTaskDelay(pdMS_TO_TICKS(1000));
    check_led();
    ESP_LOGI("STATE", "LED CHECKED");

    button_case(check_point);
    if (flag == 1){
      ESP_LOGI("STATE", "BACK STATE");flag = 0;goto LED;
    }else{
      ESP_LOGI("STATE", "NEXT STATE");
    }

    RTC:
    t_task = "RTC";t_cmd = "ON PROGRESS";t_note = "...........";
    vTaskDelay(pdMS_TO_TICKS(1000));
    LED_FILL(YELLOW);
    check_rtc();
    ESP_LOGI("STATE", "RTC CHECKED");
    vTaskDelay(pdMS_TO_TICKS(1000));

    button_case(check_point);
    if (flag == 1){
      ESP_LOGI("STATE", "BACK STATE");flag = 0;goto RTC;
    }else{
      ESP_LOGI("STATE", "NEXT STATE");
    }

    NOR:
    t_task = "NOR-FLASH";t_cmd = "ON PROGRESS";t_note = "...........";
    vTaskDelay(pdMS_TO_TICKS(1000));
    LED_FILL(BLUE);
    check_nor_flash();
    ESP_LOGI("STATE", "NOR-FLASH CHECKED");
    vTaskDelay(pdMS_TO_TICKS(1000));

    button_case(check_point);
    if (flag == 1){
      ESP_LOGI("STATE", "BACK STATE");flag = 0;goto NOR;
    }else{
      ESP_LOGI("STATE", "NEXT STATE");
    }

    RSSI:
    t_task = "RSSI-LEVEL";t_cmd = "ON PROGRESS";t_note = "...........";
    vTaskDelay(pdMS_TO_TICKS(1000));
    LED_FILL(GREEN);
    rssi_level();
    ESP_LOGI("STATE", "RSSI CHECKED");
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    button_case(check_point);
    if (flag == 1){
      ESP_LOGI("STATE", "BACK STATE");flag = 0;goto RSSI;
    }else{
      ESP_LOGI("STATE", "NEXT STATE");
    }

    MOTOR:
    LED_FILL(ORANGE);
    check_motor();
    ESP_LOGI("STATE", "MOTOR CHECKED");

    button_case(check_point);
    if (flag == 1){
      ESP_LOGI("STATE", "BACK STATE");flag = 0;goto MOTOR;
    }else{
      ESP_LOGI("STATE", "NEXT STATE");
    }

    ESP_LOGW("TEST", "TEST FINISHED");
    ESP_LOGI("TEST", "UNPLUG ESP-PROG");
    t_task = "TEST COMPLETE";t_cmd = "UNPLUG ESP-PROG";
    for(;;)
    {
        alarm(5,CYAN);
        // LED_BLINK(CYAN,10,100);
    }
}

void lcd_display(void* param)
{
    while (true) 
    {
      lcd_main(t_task.c_str(),t_cmd.c_str(),t_note.c_str());
      vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void btn_display(void *param) 
{
  int buttonPin = (int)param;
  pinMode(buttonPin, INPUT_PULLUP);
  bool buttonState = true;
  bool lastButtonState = true;
  while (1) {
    buttonState = digitalRead(buttonPin);
    if (buttonState != lastButtonState) 
    {
      if (buttonState == LOW) 
      {
        // Serial.printf("Button %d pressed\n", buttonPin);
        ESP_LOGI("DISPLAY BUTTON", "BUTTON %d PRESSED",buttonPin);
        LED_FILL(MAGENTA);
        beep(100,1);
      } 
      else 
      {
        LED_FILL(BLANK);
        // Serial.printf("Button %d released\n", buttonPin);
      }
      lastButtonState = buttonState;
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}
