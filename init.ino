// initialization of the canbus system for arduino


#include <SPI.h>
#include <mcp_can.h>

MCP_CAN CAN(10);

void setup() {
        Serial.begin(9600);
        while (!Serial);
        CAN.begin(MCP_STDEXT, CAN_500KBPS, MCP_16MHZ);
}

void loop() {