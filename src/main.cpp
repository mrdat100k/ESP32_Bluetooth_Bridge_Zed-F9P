/*
Code test kết nối serial bluetooth giữa ESP32 với SimpleRTK2B.
ESP32 sử dụng cổng hardware serial với chân RX là 16, chân TX là 17; kết nối với cổng UART1 của Zed-F9P
*/
#define MAIN_MODE //TEST_MODE - MAIN_MODE
#ifdef MAIN_MODE
#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_SSD1306.h"
#include "PUBX_Parse.h"
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RST -1
const uint8_t oled_addr = 0x3C;
Adafruit_SSD1306 oled_display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

// Khai báo Hardware serial và Bluetooth buffers
#include "BluetoothSerial.h"
HardwareSerial GNSS(2);
BluetoothSerial Serial_BT;
#define RX2 16
#define TX2 17
#define SERIAL_SIZE_BUFFER 16384 //Khai báo buffer lớn
uint8_t r_buffer[SERIAL_SIZE_BUFFER]; //buffer for reading Zed-F9P
uint8_t w_buffer[SERIAL_SIZE_BUFFER]; //buffer for writing Zed-F9P

//Global variables
uint8_t unit_MAC_address[6]; //MAC address in BT broadcast
char device_name[20]; //the serial string that is broadcast
bool in_test_mode = false; // use to retrafic Bluetooth while in test
 //Buffer for NMEA processing
char NMEA_buffer[120];
PUBX nmea(NMEA_buffer, sizeof(NMEA_buffer));
//Khai báo nguyên mẫu hàm:
// Hàm khởi tạo bluetooth
void beginBT();
// Hàm callback khi có sự kiện ngắt bluetooth
void btCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);
// Hàm khởi động bluetooth
bool startBluetooth();
// Tác vụ ghi dữ liệu vào F9P
void F9PSerialWriteTask(void *e);
// Tác vụ đọc dữ liệu từ F9P
void F9PSerialReadTask(void *e);
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); //Uart0 for programming and debugging
  Serial.setRxBufferSize(SERIAL_SIZE_BUFFER);
  Serial.setTimeout(1);
  GNSS.begin(115200);
  GNSS.setRxBufferSize(SERIAL_SIZE_BUFFER);
  GNSS.setTimeout(1);
  beginBT();
  oled_display.begin(SSD1306_SWITCHCAPVCC, oled_addr);
  oled_display.display();
  delay(2000);
  oled_display.clearDisplay();
  oled_display.drawPixel(10, 10, WHITE);
  oled_display.display();
  delay(2000);
  oled_display.clearDisplay();
  oled_display.setCursor(0, 32);
  oled_display.fillCircle(64, 32, 15, WHITE);
  oled_display.display();
  delay(2000);
  oled_display.clearDisplay();
  oled_display.setTextSize(1);
  oled_display.setTextColor(WHITE);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(200);
  /*
  Serial.print("Num of satellite: ");
  Serial.println(nmea.getNumSatellites());
  Serial.print("HDOP: ");
  Serial.println(nmea.getHDOP()/10., 1);
  Serial.print("Latitude (deg): ");
  Serial.println(nmea.getLatitude() / 1000000., 6);
  Serial.print("Longitude (deg): ");
  Serial.println(nmea.getLongitude() / 1000000., 6);
  */
  oled_display.clearDisplay();
  oled_display.setCursor(0, 0);
  oled_display.printf("%d-%d-%d", nmea.getYear(), nmea.getMonth(), nmea.getDay());
  oled_display.setCursor(0, 8);
  oled_display.printf("Time %d:%d:%d", nmea.getHour()+7, nmea.getMinute(), nmea.getSecond());
  oled_display.setCursor(0, 16);
  oled_display.printf("Lat: %.6f", nmea.getLatitude()/1000000.0);
  oled_display.setCursor(0, 24);
  oled_display.printf("Lon: %.6f", nmea.getLongitude()/1000000.0);
  oled_display.setCursor(0, 32);
  oled_display.printf("hAcc: %.2f m", nmea.getHorizontalAccuracy()/100.0);
  oled_display.setCursor(0, 40);
  oled_display.printf("vAcc: %.2f m", nmea.getVerticalAccuracy()/100.0);
  oled_display.setCursor(0, 48);
  oled_display.printf("Speed: %.3f km/h", nmea.getSpeed()/1000.0);
  oled_display.setCursor(0, 56);
  oled_display.printf("Nav mode: NULL");
  oled_display.display();
}

//Get MAC, start radio
void beginBT()
{
  //Get unit MAC address
  esp_read_mac(unit_MAC_address, ESP_MAC_WIFI_STA);
  unit_MAC_address[5] += 2; //Convert MAC address to Bluetooth MAC (add 2): https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/system.html#mac-address

  Serial_BT.register_callback(btCallback);
  if (startBluetooth() == false)
  {
    Serial.println("An error occurred initializing Bluetooth");
  }
  else
  {
    //do nothing
  }  
}

//Call back for when BT connection event happens (connected/disconnect)
//Used for updating the bluetoothState state machine
void btCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  if (event == ESP_SPP_SRV_OPEN_EVT) {
    Serial.println("Client Connected");
  }

  if (event == ESP_SPP_CLOSE_EVT ) {
    Serial.println("Client disconnected");
    Serial_BT.end();
    Serial_BT.begin(device_name);
  }
}

bool startBluetooth() {
  sprintf(device_name,"IMET RTK rover -%02X%02X", unit_MAC_address[4], unit_MAC_address[5]);
  if (Serial_BT.begin(device_name) == false) {
    return (false);
  }
  Serial.print("Bluetooth broadcasting as: ");
  Serial.println(device_name);
  //Start task for handling incomming and outgoing BT bytes to/from Zed-F9p
  xTaskCreate(F9PSerialReadTask, "F9Read", 10000, NULL, 0, NULL);
  xTaskCreate(F9PSerialWriteTask, "F9Write", 10000, NULL, 0, NULL);

  Serial_BT.setTimeout(1);

  return (true);
}

//If the phone has any new data (NTRIP RTCM, etc), read it in over Bluetooth and pass along to ZED
//Task for writing to the GNSS receiver
void F9PSerialWriteTask(void *e)
{
  while (true)
  {
    //Receive corrections from either the ESP32 USB or bluetooth
    //and write to the GPS
        if (Serial.available())
        {
          auto s = Serial.readBytes(w_buffer, SERIAL_SIZE_BUFFER);
          GNSS.write(w_buffer, s);
        }

    if (Serial_BT.available())
    {
      while (Serial_BT.available())
      {
        if (in_test_mode == false)
        {
          //Pass bytes tp GNSS receiver
          auto s = Serial_BT.readBytes(w_buffer, SERIAL_SIZE_BUFFER);
          GNSS.write(w_buffer, s);
        }
        else
        {
          Serial.printf("I heard: %c\n", Serial_BT.read());
        }
      }
    }

    taskYIELD();
  }
}

//If the ZED has any new NMEA data, pass it out over Bluetooth
//Task for reading data from the GNSS receiver.
void F9PSerialReadTask(void *e)
{
  while (true)
  {
    if (GNSS.available())
    {
      auto s = GNSS.readBytes(r_buffer, SERIAL_SIZE_BUFFER);
      //debug
      //Serial.printf("Buffer length = %d\r\n", s);
      //Pass the stream to NMEA processor
      uint16_t iter;
      for (iter = 0; iter < s; iter++) {
        nmea.process(r_buffer[iter]);
      }
      //end
    if (Serial_BT.connected())
      {
        Serial_BT.write(r_buffer, s);
      }
    }
    taskYIELD();
  }
}
#endif

#ifdef TEST_MODE
//Test the oled display
//The oled display use I2C interface with: SDA pin is connected to GPIO21; SCL pin connected to GPIO22
#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_SSD1306.h"
#include "PUBX_Parse.h"
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RST -1
const uint8_t oled_addr = 0x3C;
Adafruit_SSD1306 oled_display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

//Serial for F9P
HardwareSerial GNSS(2);
#define RX2 16
#define TX2 17
#define SERIAL_SIZE_BUFFER 16384 //Khai báo buffer lớn
uint8_t r_buffer[SERIAL_SIZE_BUFFER]; //buffer for reading Zed-F9P
uint8_t w_buffer[SERIAL_SIZE_BUFFER]; //buffer for writing Zed-F9P

 //Buffer for NMEA processing
char NMEA_buffer[120];
PUBX nmea(NMEA_buffer, sizeof(NMEA_buffer));
void F9PSerialReadTask(void *e);
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); //Uart0 for programming and debugging
  Serial.setRxBufferSize(SERIAL_SIZE_BUFFER);
  Serial.setTimeout(1);
  GNSS.begin(115200);
  GNSS.setRxBufferSize(SERIAL_SIZE_BUFFER);
  GNSS.setTimeout(1);
  oled_display.begin(SSD1306_SWITCHCAPVCC, oled_addr);
  oled_display.display();
  delay(2000);
  oled_display.clearDisplay();
  oled_display.drawPixel(10, 10, WHITE);
  oled_display.display();
  delay(2000);
  oled_display.clearDisplay();
  oled_display.setCursor(0, 32);
  oled_display.fillCircle(64, 32, 15, WHITE);
  oled_display.display();
  delay(2000);
  oled_display.clearDisplay();
  oled_display.setTextSize(1);
  oled_display.setTextColor(WHITE);
  oled_display.setCursor(0, 0);
  oled_display.printf("2021-03-30");
  oled_display.setCursor(0, 8);
  oled_display.printf("Time 11:11:11");
  oled_display.setCursor(0, 16);
  oled_display.printf("Lat: 20.916743");
  oled_display.setCursor(0, 24);
  oled_display.printf("Lon: 105.3256336");
  oled_display.setCursor(0, 32);
  oled_display.printf("HAcc: 0.1m");
  oled_display.setCursor(0, 40);
  oled_display.printf("VAcc: 0.2m");
  oled_display.setCursor(0, 48);
  oled_display.printf("Speed: 0.3 km/h");
  oled_display.setCursor(0, 56);
  oled_display.printf("Nav mode: RTK Float");
  oled_display.display();
  delay(5000);
  oled_display.clearDisplay();
  xTaskCreate(F9PSerialReadTask, "F9Read", 10000, NULL, 0, NULL);
}

void loop() {

  // put your main code here, to run repeatedly:
  delay(200);
  oled_display.clearDisplay();
  oled_display.setCursor(0, 0);
  oled_display.printf("%d-%d-%d", nmea.getYear(), nmea.getMonth(), nmea.getDay());
  oled_display.setCursor(0, 8);
  oled_display.printf("Time %d:%d:%d", nmea.getHour()+7, nmea.getMinute(), nmea.getSecond());
  oled_display.setCursor(0, 16);
  oled_display.printf("Lat: %.6f", nmea.getLatitude()/1000000.0);
  oled_display.setCursor(0, 24);
  oled_display.printf("Lon: %.6f", nmea.getLongitude()/1000000.0);
  oled_display.setCursor(0, 32);
  oled_display.printf("HAcc: %.1f m", nmea.getHorizontalAccuracy()/10.0);
  oled_display.setCursor(0, 40);
  oled_display.printf("VAcc: %.1f m", nmea.getVerticalAccuracy()/10.0);
  oled_display.setCursor(0, 48);
  oled_display.printf("Speed: %.3f km/h", nmea.getSpeed()*1.85/1000);
  oled_display.setCursor(0, 56);
  oled_display.printf("Nav mode: NULL");
  oled_display.display();
}

void F9PSerialReadTask(void *e)
{
  while (true)
  {
    if (GNSS.available())
    {
      auto s = GNSS.readBytes(r_buffer, SERIAL_SIZE_BUFFER);
      //debug
      //Serial.printf("Buffer length = %d\r\n", s);
      //Pass the stream to NMEA processor
      uint16_t iter;
      for (iter = 0; iter < s; iter++) {
        nmea.process(r_buffer[iter]);
      }
      //end
    }
    taskYIELD();
  }
}
#endif