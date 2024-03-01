# Xiaomi_CyberGear_Arduino
Arduino TWAI (Two-Wire Automotive Interface - concept conceived by Espressif, implements the CAN Bus) library for Xiaomi CyberGear.


### Usage:
Include the Xiaomi CyberGear driver and declare the `XiaomiCyberGearDriver` object:

```pp
#include "xiaomi_cybergear_driver.h"

uint8_t CYBERGEAR_CAN_ID = 0x7F; // CAN ID of the CyberGear motor
uint8_t MASTER_CAN_ID = 0x00; // Arduino CAN ID
XiaomiCyberGearDriver cybergear = XiaomiCyberGearDriver(CYBERGEAR_CAN_ID, MASTER_CAN_ID);
```

In `setup()` call the `.init_twai()` function to configure the TWAI interface.
`serial_debug` flag passed with `true` value will setup Serial interface for logging.

Then use `cybergear`'s methods to configure the motor with desired mode and parameters.

```cpp
#define RX_PIN D7
#define TX_PIN D6

void setup(){
  cybergear.init_twai(RX_PIN, TX_PIN, /*serial_debug=*/true);

  cybergear.init_motor(MODE_POSITION);
  cybergear.set_limit_speed(10.0f);    /* set the maximum speed of the motor */
  cybergear.set_limit_current(5.0);    /* current limit allows faster operation */
  cybergear.enable_motor();            /* turn on the motor */
  cybergear.set_position_ref(0.0);     /* set initial rotor position */
  ...
}
```

To get the CyberGear status call `cybergear.request_status()` which will trigger the motor to respond with its status.<br/>
```cpp
void loop(){
  ...
  // send a request to the cybergear to receive motor status (position, speed, torque, temperature)
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= TRANSMIT_RATE_MS) {
    previousMillis = currentMillis;
    cybergear.request_status();
  }
}
```

The response message needs to be processed (`cybergear.process_message()`) in `handle_rx_message()` function:
```cpp
static void handle_rx_message(twai_message_t& message) {
  if (((message.identifier & 0xFF00) >> 8) == CYBERGEAR_CAN_ID){
    cybergear.process_message(message);
  }
}
```
After processing the motor message its status can be requested using:
```cpp
void loop(){
  ...
  XiaomiCyberGearStatus cybergear_status = cybergear.get_status();
  Serial.printf("POS:%f V:%f T:%f temp:%d\n", cybergear_status.position, cybergear_status.speed, cybergear_status.torque, cybergear_status.temperature);
  ...
}
```

See the [Xiaomi_CyberGear_Arduino](https://github.com/DanielKalicki/Xiaomi_CyberGear_Arduino/tree/main)/[xiaomi_cybergear/xiaomi_cybergear.ino](https://github.com/DanielKalicki/Xiaomi_CyberGear_Arduino/tree/main/xiaomi_cybergear/xiaomi_cybergear.ino) for  full example.

### Documentation:
Xiaomi CyberGear technical drawing:
<p align="center">
  <img src="https://raw.githubusercontent.com/DanielKalicki/Xiaomi_CyberGear_Arduino/tree/main/technical_drawing/Xiaomi CyberGear.jpeg">
</p>