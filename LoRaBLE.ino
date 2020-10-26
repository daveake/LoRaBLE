// This is generic firmware to make various devices into HAB LoRa receivers.  Currently hash settings for:

// TTGO T-Beam
// TTGO LoRa OLED V1
// TTGO LoRa OLED V2
// Uputronics LoRaGo

// The receiver can be connected to a phone or tablet or PC, using Bluetooth or BLE (Bluetooth Low Power) or USB Serial so in order to view the received telemetry.

// Additionally, if the device includes a GPS receiver, then this can be used to get the local GPS position, meaning that the device can be used with (for example)
// a programmable bluetooth watch to show the distance and direction to the payload.

// UNCOMMENT ONE AND ONLY ONE OF THESE LINES

#define TBEAM
// #define OLEDV1
// #define OLEDV2
// #define LORAGO

// If you have Bluetooth, set the device names:
#define BLE_DEVICE   "HABRX"
// #define BT_DEVICE "HAB BT"

//---------
//
// Board definitions

#ifdef TBEAM
  #define ESP32
  #define BLUE
  #define AXP
  #define LORA_NSS           18
  #define LORA_RST           14
  #define LORA_DIO0          26                
  #define SCK                 5
  #define MISO               19
  #define MOSI               27
  #define GPSSerial     Serial1
  #define GPS_TX             34
  #define GPS_RX             12
#endif
  
#ifdef OLEDV1
  #define ESP32
  #define BLUE
  #define OLED
  #define OLED_RST         16
  #define OLED_SDA            4
  #define OLED_SCL           15
  #define SCREEN_WIDTH      128
  #define SCREEN_HEIGHT      64
  #define LORA_NSS           18                
  #define LORA_DIO0          26                
  #define SCK                 5
  #define MISO               19
  #define MOSI               27
  #define LED                 2
#endif
  
#ifdef OLEDV2
  #define ESP32
  #define BLE
  #define BLUE
  #define OLED
  #define OLED_RST         16
  #define OLED_SDA           21
  #define OLED_SCL           22
  #define SCREEN_WIDTH      128
  #define SCREEN_HEIGHT      64
  #define LORA_NSS           18                
  #define LORA_DIO0          26                
  #define SCK                 5
  #define MISO               19
  #define MOSI               27
  #define LED                25  // Pin 23 on V2_1.5 or pin 25 on V2_1.6 acording to http://www.lilygo.cn/prod_view.aspx?TypeId=50003&Id=1270&FId=t3:50003:3
#endif
  
#ifdef LORAGO
  #define LORA_NSS            8                // Comment out to disable LoRa code
  #define LORA_DIO0           7                
  #define LORA_RST            4
#endif

// Power management
#ifdef AXP
  #include <axp20x.h>
#endif  

  
//------
//
// Bluetooth

#ifdef BLUE 
  #include <BLEDevice.h>
  #include <BLEServer.h>
  #include <BLEUtils.h>
  #include <BLE2902.h>
  #include <BluetoothSerial.h>
#endif  

#ifdef OLED
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
#endif  


#include <string.h>
#include <ctype.h>
#include <SPI.h>
#include <EEPROM.h>
#include <Wire.h>

// DEFINES


char character;
byte currentMode = 0x81;
unsigned long UpdateClientAt=0;

struct TSettings
{
  double Frequency;
  int LoRaMode;
  int ImplicitOrExplicit;
  int ErrorCoding;
  int Bandwidth;
  int SpreadingFactor;
  int LowDataRateOptimize;
} Settings;

#define EEPROM_SIZE (sizeof(Settings) + 2)


#ifdef BLUE
  BLECharacteristic *pTxCharacteristic;
  bool BLEConnected = false;
#endif  

unsigned long LEDOff=0;

#ifdef OLED
  Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);
#endif

#define REG_FIFO                    0x00
#define REG_FIFO_ADDR_PTR           0x0D 
#define REG_FIFO_TX_BASE_AD         0x0E
#define REG_FIFO_RX_BASE_AD         0x0F
#define REG_RX_NB_BYTES             0x13
#define REG_OPMODE                  0x01
#define REG_FIFO_RX_CURRENT_ADDR    0x10
#define REG_IRQ_FLAGS               0x12
#define REG_PACKET_SNR              0x19
#define REG_PACKET_RSSI             0x1A
#define REG_RSSI_CURRENT            0x1B
#define REG_DIO_MAPPING_1           0x40
#define REG_DIO_MAPPING_2           0x41
#define REG_MODEM_CONFIG            0x1D
#define REG_MODEM_CONFIG2           0x1E
#define REG_MODEM_CONFIG3           0x26
#define REG_PAYLOAD_LENGTH          0x22
#define REG_IRQ_FLAGS_MASK          0x11
#define REG_HOP_PERIOD              0x24
#define REG_FREQ_ERROR              0x28
#define REG_DETECT_OPT              0x31
#define REG_DETECTION_THRESHOLD     0x37

// MODES
#define RF96_MODE_RX_CONTINUOUS     0x85
#define RF96_MODE_SLEEP             0x80
#define RF96_MODE_STANDBY           0x81

#define PAYLOAD_LENGTH              80

// Modem Config 1
#define EXPLICIT_MODE               0x00
#define IMPLICIT_MODE               0x01

#define ERROR_CODING_4_5            0x02
#define ERROR_CODING_4_6            0x04
#define ERROR_CODING_4_7            0x06
#define ERROR_CODING_4_8            0x08

#define BANDWIDTH_7K8               0x00
#define BANDWIDTH_10K4              0x10
#define BANDWIDTH_15K6              0x20
#define BANDWIDTH_20K8              0x30
#define BANDWIDTH_31K25             0x40
#define BANDWIDTH_41K7              0x50
#define BANDWIDTH_62K5              0x60
#define BANDWIDTH_125K              0x70
#define BANDWIDTH_250K              0x80
#define BANDWIDTH_500K              0x90

// Modem Config 2

#define SPREADING_6                 0x60
#define SPREADING_7                 0x70
#define SPREADING_8                 0x80
#define SPREADING_9                 0x90
#define SPREADING_10                0xA0
#define SPREADING_11                0xB0
#define SPREADING_12                0xC0

#define CRC_OFF                     0x00
#define CRC_ON                      0x04


// POWER AMPLIFIER CONFIG
#define REG_PA_CONFIG               0x09
#define PA_MAX_BOOST                0x8F
#define PA_LOW_BOOST                0x81
#define PA_MED_BOOST                0x8A
#define PA_MAX_UK                   0x88
#define PA_OFF_BOOST                0x00
#define RFO_MIN                     0x00

// LOW NOISE AMPLIFIER
#define REG_LNA                     0x0C
#define LNA_MAX_GAIN                0x23  // 0010 0011
#define LNA_OFF_GAIN                0x00


//#ifdef BLUE
//  BluetoothSerial SerialBT;
//#endif  

#ifdef AXP
  AXP20X_Class axp;
#endif

char Hex[] = "0123456789ABCDEF";

void SetParametersFromLoRaMode(int LoRaMode)
{
  Settings.LowDataRateOptimize = 0;

#ifdef OLED
  display.setCursor(0,20);
  display.print("Mode: ");
  display.print(LoRaMode);
  display.display();
#endif
  
  Settings.LoRaMode = LoRaMode;
  
  if (LoRaMode == 7)
  {
    Settings.ImplicitOrExplicit = EXPLICIT_MODE;
    Settings.ErrorCoding = ERROR_CODING_4_5;
    Settings.Bandwidth = BANDWIDTH_20K8;
    Settings.SpreadingFactor = SPREADING_7;
  }
  else if (LoRaMode == 6)
  {
    Settings.ImplicitOrExplicit = IMPLICIT_MODE;
    Settings.ErrorCoding = ERROR_CODING_4_5;
    Settings.Bandwidth = BANDWIDTH_41K7;
    Settings.SpreadingFactor = SPREADING_6;
  }
  else if (LoRaMode == 5)
  {
    Settings.ImplicitOrExplicit = EXPLICIT_MODE;
    Settings.ErrorCoding = ERROR_CODING_4_8;
    Settings.Bandwidth = BANDWIDTH_41K7;
    Settings.SpreadingFactor = SPREADING_11;
  }
  else if (LoRaMode == 4)
  {
    Settings.ImplicitOrExplicit = IMPLICIT_MODE;
    Settings.ErrorCoding = ERROR_CODING_4_5;
    Settings.Bandwidth = BANDWIDTH_250K;
    Settings.SpreadingFactor = SPREADING_6;
  }
  else if (LoRaMode == 3)
  {
    Settings.ImplicitOrExplicit = EXPLICIT_MODE;
    Settings.ErrorCoding = ERROR_CODING_4_6;
    Settings.Bandwidth = BANDWIDTH_250K;
    Settings.SpreadingFactor = SPREADING_7;
  }
  else if (LoRaMode == 2)
  {
    Settings.ImplicitOrExplicit = EXPLICIT_MODE;
    Settings.ErrorCoding = ERROR_CODING_4_8;
    Settings.Bandwidth = BANDWIDTH_62K5;
    Settings.SpreadingFactor = SPREADING_8;

  }
  else if (LoRaMode == 1)
  {
    Settings.ImplicitOrExplicit = IMPLICIT_MODE;
    Settings.ErrorCoding = ERROR_CODING_4_5;
    Settings.Bandwidth = BANDWIDTH_20K8;
    Settings.SpreadingFactor = SPREADING_6;

  }
  else if (LoRaMode == 0)
  {
    Settings.ImplicitOrExplicit = EXPLICIT_MODE;
    Settings.ErrorCoding = ERROR_CODING_4_8;
    Settings.Bandwidth = BANDWIDTH_20K8;
    Settings.SpreadingFactor = SPREADING_11;
    Settings.LowDataRateOptimize = 0x08;
  }
}

#ifdef BLUE
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      Serial.println("BLE Connected");
      BLEConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      BLEConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0)
      {
        char Line[21];
        
        for (int i = 0; i < rxValue.length(); i++)
        {
          Line[i] = rxValue[i];
        }
        Line[rxValue.length()] = '\0';

        Serial.print("Command = "); Serial.println(Line);

        ProcessCommand(Line);
      }
    }
};
#endif

void LoadDefaults()
{
//  Settings.PPM = 0.0;
  
  Settings.Frequency = 434.325;
//  Settings.Rx.ImplicitOrExplicit = 1;
//  Settings.Rx.ErrorCoding = 5;
//  Settings.Rx.Bandwidth = 3;
//  Settings.Rx.SpreadingFactor = 6;

}

void LoadSettings(void)
{
  int i;
  unsigned char *ptr;

  ptr = (unsigned char *)(&Settings);
  for (i=0; i<sizeof(Settings); i++, ptr++)
  {
    *ptr = EEPROM.read(i+2);
  }
}

void StoreSettings(void)
{
  int i;
  unsigned char *ptr;
  
  // Signature
  EEPROM.write(0, 'D');
  EEPROM.write(1, 'A');

  // Settings
  ptr = (unsigned char *)(&Settings);
  for (i=0; i<sizeof(Settings); i++, ptr++)
  {
    EEPROM.write(i+2, *ptr);
  }

#ifdef ESP32
  EEPROM.commit();
#endif  
}


// initialize the library with the numbers of the interface pins

void setup()
{
  Serial.begin(57600);
  
  Serial.println("");
  Serial.println("HAB LoRa Receiver V1.1");
  Serial.println("");

  // EEPROM
  #ifdef ESP32
    EEPROM.begin(EEPROM_SIZE);  
  #endif
   
  #ifdef LED
    pinMode(LED, OUTPUT);
    digitalWrite(LED, 1);
  #endif

  #ifdef AXP
    Wire.begin(21, 22);
    if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS))
    {
      Serial.println("AXP192 Begin PASS");
    }
    else
    {
      Serial.println("AXP192 Begin FAIL");
    }  

    axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);
    axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);
    axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
    axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);
    axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);
    axp.setDCDC1Voltage(3300);
  
    Serial.printf("DCDC1: %s\n", axp.isDCDC1Enable() ? "ENABLE" : "DISABLE");
    Serial.printf("DCDC2: %s\n", axp.isDCDC2Enable() ? "ENABLE" : "DISABLE");
    Serial.printf("LDO2: %s\n", axp.isLDO2Enable() ? "ENABLE" : "DISABLE");
    Serial.printf("LDO3: %s\n", axp.isLDO3Enable() ? "ENABLE" : "DISABLE");
    Serial.printf("DCDC3: %s\n", axp.isDCDC3Enable() ? "ENABLE" : "DISABLE");
    Serial.printf("Exten: %s\n", axp.isExtenEnable() ? "ENABLE" : "DISABLE");

    if (axp.isChargeing())
    {
      Serial.println("Charging");
    }    
  #endif

  // OLED
  #ifdef OLED
    pinMode(OLED_RST, OUTPUT);
    digitalWrite(OLED_RST, LOW);
    delay(20);
    digitalWrite(OLED_RST, HIGH);  

    Wire.begin(OLED_SDA, OLED_SCL);

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false))
    {
      // Address 0x3C for 128x32
      Serial.println(F("SSD1306 allocation failed"));
      for(;;); // Don't proceed, loop forever
    }  
    display.clearDisplay();  
    display.setTextColor(WHITE, 0);
    display.setTextSize(1);
    display.setCursor(0,0);
    display.print("LoRa Receiver V1.0");

    #ifdef BLUE
//      display.setCursor(0,32);
//      display.print(" BT Device ");  display.print(BT_DEVICE);
      display.setCursor(0,42);
      display.print("BLE Device ");  display.print(BLE_DEVICE);
    #endif
  
    display.display();
  #endif

  #ifdef GPSSerial
    SetupGPS();
  #endif

  if ((EEPROM.read(0) == 'D') && (EEPROM.read(1) == 'A'))
  {
    // Load settings from EEPROM
    LoadSettings();
    Serial.println("Settings loaded from EEPROM");
  }
  else
  {
    LoadDefaults();
    Serial.println("Loaded default settings");
    StoreSettings();
  }
  
  SetParametersFromLoRaMode(Settings.LoRaMode);

  setupRFM98();

  SetParametersFromLoRaMode(Settings.LoRaMode);

  #ifdef LED
    digitalWrite(LED, 0);
  #endif  

  #ifdef BLUE
    // BLE
    // Create the BLE Device
    BLEDevice::init(BLE_DEVICE);
    BLEDevice::setPower(ESP_PWR_LVL_P7);

    // BT
    // SerialBT.begin(BT_DEVICE);

    // Create the BLE Server
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create the BLE Service
    BLEService *pService = pServer->createService(SERVICE_UUID);

    // Create a BLE Characteristic
    pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
                      
    pTxCharacteristic->addDescriptor(new BLE2902());

    BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);

    pRxCharacteristic->setCallbacks(new MyCallbacks());

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();  

    // Start the service
    pService->start();

    // Start advertising
    pServer->getAdvertising()->start();
    Serial.println("BLE Server Ready");
  #endif  
}

void loop()
{
  CheckPC();

#ifdef BLUE
  CheckBT();
#endif
  
  CheckRx();
  
  #ifdef GPSSerial
    CheckGPS();
  #endif

  UpdateClient();
}

void UpdateClient(void)
{
  if (millis() >= UpdateClientAt)
  {
    int CurrentRSSI;
    char Line[20];

    if (Settings.Frequency > 525)
    {
      CurrentRSSI = readRegister(REG_RSSI_CURRENT) - 157;
    }
    else
    {
      CurrentRSSI = readRegister(REG_RSSI_CURRENT) - 164;
    }

    sprintf(Line, "CurrentRSSI=%d\r\n", CurrentRSSI);
    SendToHosts(Line);

  #ifdef OLED
    sprintf(Line, "RSSI: %3d ", CurrentRSSI);
    display.setCursor(50,20);
    display.print(Line);
    display.display();
  #endif    

    UpdateClientAt = millis() + 1000;
  }
}
  
void SendToHosts(char *Line)
{
  char *ptr;
  int Done;

  Serial.print(Line);
  
  #ifdef BLUE
    // SerialBT.print(Line);
     
    if (BLEConnected)
    {
      ptr = Line;
      Done = 0;
      while (!Done)
      {
        pTxCharacteristic->setValue(ptr);
        pTxCharacteristic->notify();
  
        if (strlen(ptr) > 20)
        {
          ptr += 20;
        }
        else
        {
          Done = 1;
        }
      }
    }
  #endif  
}

double FrequencyReference(void)
{
  switch (Settings.Bandwidth)
  {
    case  BANDWIDTH_7K8:  return 7800;
    case  BANDWIDTH_10K4:   return 10400; 
    case  BANDWIDTH_15K6:   return 15600; 
    case  BANDWIDTH_20K8:   return 20800; 
    case  BANDWIDTH_31K25:  return 31250; 
    case  BANDWIDTH_41K7:   return 41700; 
    case  BANDWIDTH_62K5:   return 62500; 
    case  BANDWIDTH_125K:   return 125000; 
    case  BANDWIDTH_250K:   return 250000; 
    case  BANDWIDTH_500K:   return 500000; 
  }

  return 0;
}


double FrequencyError(void)
{
  int32_t Temp;
  double T;
  
  Temp = (int32_t)readRegister(REG_FREQ_ERROR) & 7;
  Temp <<= 8L;
  Temp += (int32_t)readRegister(REG_FREQ_ERROR+1);
  Temp <<= 8L;
  Temp += (int32_t)readRegister(REG_FREQ_ERROR+2);
  
  if (readRegister(REG_FREQ_ERROR) & 8)
  {
    Temp = Temp - 524288;
  }

  T = (double)Temp;
  T *=  (16777216.0 / 32000000.0);
  T *= (FrequencyReference() / 500000.0);

  return -T;
} 

int receiveMessage(unsigned char *message)
{
  int i, Bytes, currentAddr;

  int x = readRegister(REG_IRQ_FLAGS);
  Bytes = 0;
  // printf("Message status = %02Xh\n", x);
  
  // clear the rxDone flag
  // writeRegister(REG_IRQ_FLAGS, 0x40); 
  writeRegister(REG_IRQ_FLAGS, 0xFF); 
   
  // check for payload crc issues (0x20 is the bit we are looking for
  if((x & 0x20) == 0x20)
  {
    Serial.println("CRC Failure");
    // reset the crc flags
    writeRegister(REG_IRQ_FLAGS, 0x20); 
  }
  else
  {
    Serial.println("Received");
    currentAddr = readRegister(REG_FIFO_RX_CURRENT_ADDR);
    Bytes = readRegister(REG_RX_NB_BYTES);
    // printf ("%d bytes in packet\n", Bytes);

    // printf("RSSI = %d\n", readRegister(REG_RSSI) - 137);
  
    writeRegister(REG_FIFO_ADDR_PTR, currentAddr);   
    // now loop over the fifo getting the data
    for(i = 0; i < Bytes; i++)
    {
      message[i] = (unsigned char)readRegister(REG_FIFO);
    }
    message[Bytes] = '\0';

    // writeRegister(REG_FIFO_ADDR_PTR, 0);  // currentAddr);   
  } 
  
  return Bytes;
}

void ReplyOK(void)
{
  SendToHosts("*\r\n");
}

void ReplyBad(void)
{
  SendToHosts("?");
  // Serial.println('?');
  // SerialBT.println('?');
}

void SetFrequency(char *Line)
{
  double Freq;

  Freq = atof(Line);

  if (Freq > 0)
  {
    char Line[20];

    Settings.Frequency = Freq;
    
    StoreSettings();
    
    ReplyOK();

    sprintf(Line, "Frequency=%.3lf\r\n", Settings.Frequency);
    SendToHosts(Line);
    
    startReceiving();
  }
  else
  {
    ReplyBad();
  }
}

void SetMode(char *Line)
{
  int Mode;

  Mode = atoi(Line);

  if ((Mode >= 0) && (Mode <= 7))
  {
    char Line[20];
    
    Settings.LoRaMode = Mode;
    
    SetParametersFromLoRaMode(Mode);
    
    StoreSettings();
    
    ReplyOK();

    sprintf(Line, "Mode=%d\r\n", Mode);
    SendToHosts(Line);

    startReceiving();
    
  }
  else
  {
    ReplyBad();
  }
}

void SetBandwidth(char *Line)
{
  if (strcmp(Line, "7K8") == 0)
  {
    Settings.Bandwidth = BANDWIDTH_7K8;
    StoreSettings();
    ReplyOK();
    startReceiving();
  }
  else if (strcmp(Line, "10K4") == 0)
  {
    Settings.Bandwidth = BANDWIDTH_10K4;
    StoreSettings();
    ReplyOK();
    startReceiving();
  }
  else if (strcmp(Line, "15K6") == 0)
  {
    Settings.Bandwidth = BANDWIDTH_15K6;
    StoreSettings();
    ReplyOK();
    startReceiving();
  }
  else if (strcmp(Line, "20K8") == 0)
  {
    Settings.Bandwidth = BANDWIDTH_20K8;
    StoreSettings();
    ReplyOK();
    startReceiving();
  }
  else if (strcmp(Line, "31K25") == 0)
  {
    Settings.Bandwidth = BANDWIDTH_31K25;
    StoreSettings();
    ReplyOK();
    startReceiving();
  }
  else if (strcmp(Line, "41K7") == 0)
  {
    Settings.Bandwidth = BANDWIDTH_41K7;
    StoreSettings();
    ReplyOK();
    startReceiving();
  }
  else if (strcmp(Line, "62K5") == 0)
  {
    Settings.Bandwidth = BANDWIDTH_62K5;
    StoreSettings();
    ReplyOK();
    startReceiving();
  }
  else if (strcmp(Line, "125K") == 0)
  {
    Settings.Bandwidth = BANDWIDTH_125K;
    StoreSettings();
    ReplyOK();
    startReceiving();
  }
  else if (strcmp(Line, "250K") == 0)
  {
    Settings.Bandwidth = BANDWIDTH_250K;
    StoreSettings();
    ReplyOK();
    startReceiving();
  }
  else if (strcmp(Line, "500K") == 0)
  {
    Settings.Bandwidth = BANDWIDTH_500K;
    StoreSettings();
    ReplyOK();
    startReceiving();
  }
  else
  {
    ReplyBad();
  }
}

void SetErrorCoding(char *Line)
{
  int Coding;

  Coding = atoi(Line);

  if ((Coding >= 5) && (Coding <= 8))
  {
    Settings.ErrorCoding = (Coding-4) << 1;
    StoreSettings();
    ReplyOK();
    startReceiving();
  }
  else
  {
    ReplyBad();
  }
}

void SetSpreadingFactor(char *Line)
{
  int Spread;

  Spread = atoi(Line);

  if ((Spread >= 6) && (Spread <= 12))
  {
    Settings.SpreadingFactor = Spread << 4;
    StoreSettings();
    ReplyOK();
    startReceiving();
  }
  else
  {
    ReplyBad();
  }
}

void SetImplicit(char *Line)
{
  int Implicit;

  Implicit = atoi(Line);

  Settings.ImplicitOrExplicit = Implicit ? IMPLICIT_MODE : EXPLICIT_MODE;

  StoreSettings();
  
  ReplyOK();
  
  startReceiving();
}

void SetLowOpt(char *Line)
{
  int LowOpt;

  LowOpt = atoi(Line);

  Settings.ImplicitOrExplicit = LowOpt ? 0x08 : 0;

  StoreSettings();

  ReplyOK();
  
  startReceiving();
}

void ProcessCommand(char *Line)
{
  char Command;

  Command = Line[1];
  Line += 2;
       
  if (Command == 'F')
  {
    SetFrequency(Line);
  }
  else if (Command == 'M')
  {
    SetMode(Line);
  }
  else if (Command == 'B')
  {
    SetBandwidth(Line);
  }
  else if (Command == 'E')
  {
    SetErrorCoding(Line);
  }
  else if (Command == 'S')
  {
    SetSpreadingFactor(Line);
  }
  else if (Command == 'I')
  {
    SetImplicit(Line);
  }
  else if (Command == 'L')
  {
    SetLowOpt(Line);
  }
  else
  {
    ReplyBad();
  }
}

void CheckPC()
{
  static char Line[32];
  static int Length=0;
  char Character;

  while (Serial.available())
  { 
    Character = Serial.read();
    
    if (Character == '~')
    {
      Line[0] = Character;
      Length = 1;
    }
    else if (Length >= sizeof(Line))
    {
      Length = 0;
    }
    else if (Length > 0)
    {
      if (Character == '\r')
      {
        Line[Length] = '\0';
        ProcessCommand(Line);
        Length = 0;
      }
      else
      {
        Line[Length++] = Character;
      }
    }
  }
}

#ifdef BLUE
void CheckBT()
{
  /*
  static char Line[32];
  static int Length=0;
  char Character;

  while (SerialBT.available())
  { 
    Character = SerialBT.read();
    
    if (Character == '~')
    {
      Line[0] = Character;
      Length = 1;
    }
    else if (Length >= sizeof(Line))
    {
      Length = 0;
    }
    else if (Length > 0)
    {
      if (Character == '\r')
      {
        Line[Length] = '\0';
        ProcessCommand(Line);
        Length = 0;
      }
      else
      {
        Line[Length++] = Character;
      }
    }
  }
  */
}
#endif

void CheckRx()
{
#ifdef LED
  if ((LEDOff > 0) && (millis() >= LEDOff))
  {
    digitalWrite(LED, LOW);
    LEDOff = 0;
  }
#endif
  
  if (digitalRead(LORA_DIO0))
  {
    unsigned char Message[256];
    char Line[20];
    int Bytes, SNR, RSSI, i;
    long Altitude;

    #ifdef LED
      digitalWrite(LED, 1);
      LEDOff = millis() + 1000;
    #endif
    
    Bytes = receiveMessage(Message);
    
    sprintf(Line, "FreqErr=%.1lf\r\n", FrequencyError()/1000.0);
    SendToHosts(Line);

    SNR = readRegister(REG_PACKET_SNR);
    SNR /= 4;
    
    if (Settings.Frequency > 525)
    {
      RSSI = readRegister(REG_PACKET_RSSI) - 157;
    }
    else
    {
      RSSI = readRegister(REG_PACKET_RSSI) - 164;
    }
    
    if (SNR < 0)
    {
      RSSI += SNR;
    }
    
//    Serial.print("PacketRSSI="); Serial.println(RSSI);
//    Serial.print("PacketSNR="); Serial.println(SNR);
//    SerialBT.print("PacketRSSI="); SerialBT.println(RSSI);
//    SerialBT.print("PacketSNR="); SerialBT.println(SNR);
    sprintf(Line, "PacketRSSI=%d\r\n", RSSI);
    SendToHosts(Line);
    sprintf(Line, "PacketSNR=%d\r\n", SNR);
    SendToHosts(Line);
    
    // Serial.print("Packet size = "); Serial.println(Bytes);

    // Telemetry='$$LORA1,108,20:30:39,51.95027,-2.54445,00141,0,0,11*9B74

    if (Message[0] == '$')
    {
       char ShortMessage[21];
       char Line[256];
       
      // Remove LF
      Message[strlen((char *)Message)-1] = '\0';
      
//      Serial.print("Message=");
//      Serial.println((char *)Message);
//      SerialBT.print("Message=");
//      SerialBT.println((char *)Message);
      sprintf(Line, "Message=%s\r\n", Message);
      SendToHosts(Line);
          
#ifdef OLED
      strncpy(ShortMessage, (char *)Message, 20);
      ShortMessage[20] = '\0';
      display.setCursor(0,32);
      display.print(ShortMessage);
      
      strncpy(ShortMessage, (char *)Message+20, 20);
      ShortMessage[20] = '\0';
      display.setCursor(0,42);
      display.print(ShortMessage);
      
      strncpy(ShortMessage, (char *)Message+40, 20);
      ShortMessage[20] = '\0';
      display.setCursor(0,52);
      display.print(ShortMessage);

      display.display();          
#endif      
    }
    else if (Message[0] == '%')
    {
      char *ptr, *ptr2;

      Message[0] = '$';
      
      ptr = (char *)Message;
      do
      {
        if ((ptr2 = strchr(ptr, '\n')) != NULL)
        {
          *ptr2 = '\0';
//          Serial.print("Message=");
//          Serial.println(ptr);
//          SerialBT.print("Message=");
//          SerialBT.println(ptr);
          SendToHosts("Message");
          SendToHosts((char *)Message);
          //SendToHosts("\r\n");
          ptr = ptr2 + 1;
        }
      } while (ptr2 != NULL);
    }
    else
    {
      Serial.print("Hex=");
#ifdef BLUE
      // SerialBT.print("Hex=");
#endif      
      for (i=0; i<Bytes; i++)
      {
        if (Message[i] < 0x10)
        {
          Serial.print("0");
#ifdef BLUE
          // SerialBT.print("0");
#endif          
        } 
        Serial.print(Message[i], HEX);
#ifdef BLUE
        // SerialBT.print("0");
#endif          
      }
      Serial.println();
#ifdef BLUE
      // SerialBT.println();
#endif          
    }
  }
}


/////////////////////////////////////
//    Method:   Change the mode
//////////////////////////////////////
void setMode(byte newMode)
{
  if(newMode == currentMode)
    return;  
  
  switch (newMode) 
  {
    case RF96_MODE_RX_CONTINUOUS:
      writeRegister(REG_PA_CONFIG, PA_OFF_BOOST);  // TURN PA OFF FOR RECIEVE??
      writeRegister(REG_LNA, LNA_MAX_GAIN);  // LNA_MAX_GAIN);  // MAX GAIN FOR RECIEVE
      writeRegister(REG_OPMODE, newMode);
      currentMode = newMode; 
      break;
      
      break;
    case RF96_MODE_SLEEP:
      writeRegister(REG_OPMODE, newMode);
      currentMode = newMode; 
      break;
    case RF96_MODE_STANDBY:
      writeRegister(REG_OPMODE, newMode);
      currentMode = newMode; 
      break;
    default: return;
  } 
  
  if(newMode != RF96_MODE_SLEEP)
  {
    delay(10);
  }
   
  return;
}


/////////////////////////////////////
//    Method:   Read Register
//////////////////////////////////////

byte readRegister(byte addr)
{
  select();
  SPI.transfer(addr & 0x7F);
  byte regval = SPI.transfer(0);
  unselect();
  return regval;
}

/////////////////////////////////////
//    Method:   Write Register
//////////////////////////////////////

void writeRegister(byte addr, byte value)
{
  select();
  SPI.transfer(addr | 0x80); // OR address with 10000000 to indicate write enable;
  SPI.transfer(value);
  unselect();
}

/////////////////////////////////////
//    Method:   Select Transceiver
//////////////////////////////////////
void select() 
{
  digitalWrite(LORA_NSS, LOW);
}

/////////////////////////////////////
//    Method:   UNSelect Transceiver
//////////////////////////////////////
void unselect() 
{
  digitalWrite(LORA_NSS, HIGH);
}

void SetLoRaFrequency()
{
  unsigned long FrequencyValue;
  double Temp;
  
  Temp = Settings.Frequency * 7110656 / 434;
  FrequencyValue = (unsigned long)(Temp);

//  Serial.print("FrequencyValue is ");
//  Serial.println(FrequencyValue);

#ifdef OLED
  display.setCursor(0,10);
  display.print("Freq: ");
  display.print(Settings.Frequency, 4);
  display.print(" MHz");
  display.display();
#endif

  writeRegister(0x06, (FrequencyValue >> 16) & 0xFF);    // Set frequency
  writeRegister(0x07, (FrequencyValue >> 8) & 0xFF);
  writeRegister(0x08, FrequencyValue & 0xFF);
}

void SetLoRaParameters()
{
  writeRegister(REG_MODEM_CONFIG, Settings.ImplicitOrExplicit | Settings.ErrorCoding | Settings.Bandwidth);
  writeRegister(REG_MODEM_CONFIG2, Settings.SpreadingFactor | CRC_ON);
  writeRegister(REG_MODEM_CONFIG3, 0x04 | Settings.LowDataRateOptimize);                  // 0x04: AGC sets LNA gain
  writeRegister(REG_DETECT_OPT, (readRegister(REG_DETECT_OPT) & 0xF8) | ((Settings.SpreadingFactor == SPREADING_6) ? 0x05 : 0x03));  // 0x05 For SF6; 0x03 otherwise
  writeRegister(REG_DETECTION_THRESHOLD, (Settings.SpreadingFactor == SPREADING_6) ? 0x0C : 0x0A);    // 0x0C for SF6, 0x0A otherwise

}

void startReceiving()
{
  setMode(RF96_MODE_SLEEP);
  writeRegister(REG_OPMODE,0x80);  
  setMode(RF96_MODE_SLEEP);

  SetLoRaFrequency();
  
  SetLoRaParameters();
  
  writeRegister(REG_PAYLOAD_LENGTH, 255);
  writeRegister(REG_RX_NB_BYTES, 255);
  
  writeRegister(REG_FIFO_RX_BASE_AD, 0);
  writeRegister(REG_FIFO_ADDR_PTR, 0);
  
  // Setup Receive Continous Mode
  setMode(RF96_MODE_RX_CONTINUOUS);
}

void setupRFM98(void)
{
  // initialize the pins
  pinMode(LORA_RST, OUTPUT);
  digitalWrite(LORA_RST, HIGH);
  delay(10);          // Module needs this before it's ready  
  
  pinMode(LORA_NSS, OUTPUT);
  pinMode(LORA_DIO0, INPUT);

#ifdef ESP32
  SPI.begin(SCK,MISO,MOSI,LORA_NSS);
#else
  SPI.begin();
#endif

  startReceiving();
  
  Serial.println("Setup Complete");
}
