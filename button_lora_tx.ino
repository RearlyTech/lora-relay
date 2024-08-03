#include "LoRaWan_APP.h"

#define LED_R 46
#define LED_G 48
#define LED_B 47
#define ADC_CTRL 37
#define BATT_ADC_IN 1
#define BLUE_LED_TIME 500
#define RED_LED_TIME 100
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
#define LORA_SEND_INTVL 1500
#define LORA_RLY_INTVL 500

int blue_led_st = 255;
int red_led_st = 255;
bool charging_st;
double charging_led_int = 0;
double low_batt_led_int = 0;
double lora_send_intvl = 0;
double lora_rly_intvl = 0;
int red_val = 0;
int green_val = 0;
char txpacket[30]; // Define the payload size here
int txNumber;
static RadioEvents_t RadioEvents;
int button_state_prev = LOW;

void OnTxDone(void);
void OnTxTimeout(void);

void setup() {
  // put your setup code here, to run once:
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(ADC_CTRL, OUTPUT);
  digitalWrite(ADC_CTRL, LOW);
  pinMode(BATT_ADC_IN, INPUT);
  // Initialize Radio events
  pinMode(45, INPUT);  // Set the button pin as input
    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);
    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                      LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                      LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                      true, 0, 0, LORA_IQ_INVERSION_ON, 3000);
}

void loop() {
  int buttonState = digitalRead(45);  // Read the state of the button
  // put your main code here, to run repeatedly:
  int analogValue = analogRead(BATT_ADC_IN);
  float map_val = (analogValue * 4.12) / 980;
  if (map_val >= 3.9) {
    charging_st = true;
  }
  else {
    charging_st = false;
  }
  
  if (charging_st == true) {  //fn to handle LED during charging
    analogWrite(LED_R, 0);
    analogWrite(LED_G, 0);
    if (millis() - charging_led_int > BLUE_LED_TIME) {
      analogWrite(LED_B, blue_led_st);
      charging_led_int = millis();
      blue_led_st = 255 - blue_led_st;
    }
  }
  else {  //fn to handle LED during discharging
    if (map_val > 3.75) { //full charge
        analogWrite(LED_B, 0);
        analogWrite(LED_R, 0);
        analogWrite(LED_G, 255);
    }
    else if (map_val < 3.31)  //critical battery level
    {
        analogWrite(LED_B, 0);
        analogWrite(LED_G, 0);
      if (millis() - low_batt_led_int > RED_LED_TIME) {
        analogWrite(LED_R, red_led_st);
        low_batt_led_int = millis();
        red_led_st = 255 - red_led_st;
    }
    }
    else {
      if (map_val >= 3.53) {  //above half charge
        red_val = 255 - (((map_val - 3.53) * 255) / 0.22);
        green_val = 255;
      }
      else {  //below half charge
        green_val = ((map_val - 3.31) * 255) / 0.21;
        red_val = 255;
      }
      analogWrite(LED_B, 0);
      analogWrite(LED_G, green_val);
      analogWrite(LED_R, red_val);
    }
  }
  if (buttonState == HIGH) {  // If the button is pressed
    if ((millis() - lora_rly_intvl > LORA_RLY_INTVL)) {
      lora_rly_intvl = millis();
      sprintf(txpacket, "ON");  // Prepare the ON packet
      Radio.Send((uint8_t *)txpacket, strlen(txpacket));  // Send the ON packet
      button_state_prev = buttonState;
    }
  }
  else if ((millis() - lora_send_intvl > LORA_SEND_INTVL) or (buttonState == LOW && button_state_prev == HIGH)) {
    lora_send_intvl = millis();
    txNumber += 1;
    sprintf(txpacket,"%0.2f", txNumber);  //start a package
    //sprintf(txpacket, "Battery level: %0.2f", map_val);  // Prepare the send packet
    Radio.Send((uint8_t *)txpacket, strlen(txpacket));  // Send the batt voltage packet
    button_state_prev = buttonState;
  }
  else {
    button_state_prev = buttonState;
  }
}