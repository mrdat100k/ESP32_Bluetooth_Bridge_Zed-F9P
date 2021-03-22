/*
Code test kết nối serial bluetooth giữa ESP32 với SimpleRTK2B.
ESP32 sử dụng cổng hardware serial với chân RX là 16, chân TX là 17; kết nối với cổng UART1 của Zed-F9P
*/

#include <Arduino.h>
#include "MicroNMEA.h"

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
MicroNMEA nmea(NMEA_buffer, sizeof(NMEA_buffer));
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
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(100);
  Serial.print("Num of satellite: ");
  Serial.println(nmea.getNumSatellites());
  Serial.print("HDOP: ");
  Serial.println(nmea.getHDOP()/10., 1);
  Serial.print("Latitude (deg): ");
  Serial.println(nmea.getLatitude() / 1000000., 6);
  Serial.print("Longitude (deg): ");
  Serial.println(nmea.getLongitude() / 1000000., 6);
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