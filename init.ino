// initialization of the canbus system for arduino

#include <SPI.h>
#include <mcp_can.h>

#define CAN_CS_PIN 10
MCP_CAN CAN(CAN_CS_PIN);

void setup() {
        Serial.begin(9600);
        while (!Serial);

        // Initialize MCP2515 CAN controller at 500kbps
        if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK) {
                Serial.println("MCP2515 Initialized Successfully!");
        } else {
                Serial.println("Error Initializing MCP2515...");
                while (1);
        }
        CAN.setMode(MCP_NORMAL); // Set operation mode to normal so the MCP2515 sends/receives data

        Serial.println("CAN BUS Shield Init OK!");
}

void loop() {
        delay(1000); // Wait 1 second before sending/receiving again
}