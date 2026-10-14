// initialization of the canbus system for esp32
// uses the built-in TWAI controller; connect a transceiver (SN65HVD230) to the pins below

#include "driver/twai.h"

#define CAN_TX_PIN GPIO_NUM_22
#define CAN_RX_PIN GPIO_NUM_21

void setup() {
    Serial.begin(115200);

    twai_general_config_t general_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t timing_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    twai_driver_install(&general_config, &timing_config, &filter_config);
    twai_start();
}

void loop() {
}
