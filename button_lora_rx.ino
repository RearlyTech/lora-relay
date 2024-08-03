#include "LoRaWan_APP.h"

// LoRa parameters
#define RF_FREQUENCY  915000000  // Hz
#define RX_TIMEOUT_VALUE  1000
#define BUFFER_SIZE  30  // Define the payload size here
#include <Wire.h>
#include "HT_SSD1306Wire.h"

// OLED f_display setup
SSD1306Wire f_display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

char rxpacket[BUFFER_SIZE];  // Buffer for the received packet
bool lora_idle = true;
static RadioEvents_t RadioEvents;

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void OnRxError(void);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
  // Initialize Radio events
  f_display.init();
  f_display.flipScreenVertically();
  f_display.setFont(ArialMT_Plain_16);
  f_display.drawString(0, 0, "Waiting...");
  f_display.display();
  pinMode(45, OUTPUT);  // Set the button pin as input
  digitalWrite(45, LOW);
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.RxError = OnRxError;
  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetRxConfig(MODEM_LORA, 0, 0x0c,4 , 0, 8, 0, false, 0, true, 0, 0, false, true);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(lora_idle)
    {
      lora_idle = false;
      Serial.println("into RX mode");
      Radio.Rx(0);
    }
  Radio.IrqProcess( );
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
    memcpy(rxpacket, payload, size);
    rxpacket[size] = '\0';  // Null-terminate the string
    Serial.printf("Received '%s' with RSSI %d\n", rxpacket, rssi);
    f_display.clear();
    //f_display.drawString(0, 0, "Received:");
    f_display.drawString(0, 16, String(rssi));
    f_display.display();

    if (strcmp(rxpacket, "ON") == 0) {
        
        f_display.drawString(0, 32, "Relay ON!");
        digitalWrite(45, HIGH);
    } else {
        digitalWrite(45, LOW);
        f_display.drawString(0, 32, "Relay OFF!");
    }
    f_display.display();
    Radio.Sleep();
    lora_idle = true;
}

void OnRxError(void) {
  Serial.println("RX Error...");
  f_display.clear();
  f_display.drawString(0, 0, "RX Error!");
  f_display.display();
  Radio.Sleep();
  lora_idle = true;
}