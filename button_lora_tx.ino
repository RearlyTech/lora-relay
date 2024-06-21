#include "Arduino.h"
#include "LoRaWan_APP.h"
#include <Wire.h>
#include "HT_SSD1306Wire.h"

// OLED display setup
SSD1306Wire f_display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

// LoRa parameters
#define RF_FREQUENCY  915000000  // Hz
#define TX_OUTPUT_POWER  22  // dBm
#define LORA_BANDWIDTH  0  // [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define LORA_SPREADING_FACTOR  0x0c
// [SF7..SF12]
#define LORA_CODINGRATE  4  // [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
#define LORA_PREAMBLE_LENGTH  8
#define LORA_SYMBOL_TIMEOUT  0
#define LORA_FIX_LENGTH_PAYLOAD_ON  false
#define LORA_IQ_INVERSION_ON  false

char txpacket[30]; // Define the payload size here
double txNumber;
static RadioEvents_t RadioEvents;
void OnTxDone(void);
void OnTxTimeout(void);

bool lora_idle=true;
void setup() {
    Serial.begin(115200);
    Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
    pinMode(45, INPUT);  // Set the button pin as input
   // lora_idle == true
    f_display.init();
    f_display.flipScreenVertically();
    f_display.setFont(ArialMT_Plain_16);
    f_display.drawString(0, 0, "Connecting...");
    f_display.display();

    // Initialize Radio events
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;

    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);
    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                      LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                      LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                      true, 0, 0, LORA_IQ_INVERSION_ON, 3000);
}

void loop() {
    int buttonState = digitalRead(45);  // Read the state of the button


     if (buttonState == HIGH) {  // If the button is pressed
      Serial.printf("button high");
        sprintf(txpacket, "ON");  // Prepare the ON packet
        f_display.clear();
        f_display.drawString(0, 0, "Sending ON...");
        f_display.display();
        Serial.printf("\r\nsending packet \"%s\" , length %d\r\n",txpacket, strlen(txpacket));

        Radio.Send((uint8_t *)txpacket, strlen(txpacket));  // Send the ON packet

        //delay(2000);  // Debouncing delay
    } else{
       f_display.clear();
       f_display.drawString(0, 0, "Relay off");
       f_display.display();
    delay(1000);
    txNumber += 0.01;
    sprintf(txpacket,"Hello world number %0.2f",txNumber);  //start a package
     
    Serial.printf("\r\nsending packet \"%s\" , length %d\r\n",txpacket, strlen(txpacket));

    Radio.Send( (uint8_t *)txpacket, strlen(txpacket) ); //send the package out 
    }
}

void OnTxDone(void) {


    Serial.println("TX done......");
    //delay(2000); // Delay to view message
    lora_idle = true;

}

void OnTxTimeout(void) {
    f_display.clear();
    f_display.drawString(0, 0, "TX Timeout...");
    f_display.display();

    Serial.println("TX Timeout......");
    //delay(2000); // Delay to view message

    f_display.clear();
    f_display.drawString(0, 0, "Ready");
    f_display.display();
}
