#include "driver/twai.h"
#include "xiaomi_cybergear_driver.h"

// Pins used to connect to CAN bus transceiver:
#define RX_PIN D7
#define TX_PIN D6

// Intervall:
#define TRANSMIT_RATE_MS 1000
#define POLLING_RATE_MS 1000

static bool driver_installed = false;
unsigned long previousMillis = 0;  // will store last time a message was send

uint8_t CYBERGEAR_CAN_ID = 0x7F;
uint8_t MASTER_CAN_ID = 0x00;
XiaomiCyberGearDriver cybergear = XiaomiCyberGearDriver(CYBERGEAR_CAN_ID, MASTER_CAN_ID);

void setup() {
  // initialize TWAI (CAN) interface to communicate with Xiaomi CyberGear
  // this needs to be called only once for any cybergear
  cybergear.init_twai(RX_PIN, TX_PIN, /*serial_debug=*/true);

  // Serial.begin(115200) will be called in cybergear.init_twai function 

  cybergear.init_motor(MODE_POSITION);
  cybergear.set_limit_speed(10.0f); /* set the maximum speed of the motor */
  cybergear.set_limit_current(5.0); /* current limit allows faster operation */
  cybergear.enable_motor(); /* turn on the motor */

  cybergear.set_position_ref(0.0); /* set initial rotor position */
  cybergear.stop_motor(); /* stop the motor */

  // TWAI driver is now successfully installed and started
  driver_installed = true;
}

static void handle_rx_message(twai_message_t& message) {
  if (((message.identifier & 0xFF00) >> 8) == CYBERGEAR_CAN_ID){
    cybergear.process_message(message);
  }

  // print received message
  // Serial.printf("ID: %x\nByte:", message.identifier);
  // if (!(message.rtr)) {
  //   for (int i = 0; i < message.data_length_code; i++) {
  //     Serial.printf(" %d = %02x,", i, message.data[i]);
  //   }
  //   Serial.println("");
  // }
}

static void check_alerts(){
  // Check if alert happened
  uint32_t alerts_triggered;
  twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(POLLING_RATE_MS));
  twai_status_info_t twai_status;
  twai_get_status_info(&twai_status);

  // Handle alerts
  if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
    Serial.println("Alert: TWAI controller has become error passive.");
  }
  if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
    Serial.println("Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
    Serial.printf("Bus error count: %d\n", twai_status.bus_error_count);
  }
  if (alerts_triggered & TWAI_ALERT_TX_FAILED) {
    Serial.println("Alert: The Transmission failed.");
    Serial.printf("TX buffered: %d\t", twai_status.msgs_to_tx);
    Serial.printf("TX error: %d\t", twai_status.tx_error_counter);
    Serial.printf("TX failed: %d\n", twai_status.tx_failed_count);
  }
  // if (alerts_triggered & TWAI_ALERT_TX_SUCCESS) {
  //   Serial.println("Alert: The Transmission was successful.");
  //   Serial.printf("TX buffered: %d\t", twai_status.msgs_to_tx);
  // }

  // Check if message is received
  if (alerts_triggered & TWAI_ALERT_RX_DATA) {
    twai_message_t message;
    while (twai_receive(&message, 0) == ESP_OK) {
      handle_rx_message(message);
    }
  }
}

void loop() {
  if (!driver_installed) {
    delay(1000);
    return;
  }

  delay(1000);

  static float pos = 0.0;
  static float inc_val = 1;
  pos += inc_val;
  if (pos > 10.0) inc_val = -1;
  if (pos < -10.0) inc_val = 1;
  cybergear.set_position_ref(pos);

  check_alerts();

  XiaomiCyberGearStatus cybergear_status = cybergear.get_status();
  Serial.printf("POS:%f V:%f T:%f temp:%d\n", cybergear_status.position, cybergear_status.speed, cybergear_status.torque, cybergear_status.temperature);

  // send a request to the cybergear to receive motor status (position, speed, torque, temperature)
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= TRANSMIT_RATE_MS) {
    previousMillis = currentMillis;
    cybergear.request_status();
  }
}
