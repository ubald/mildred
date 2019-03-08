#include <DynamixelSDK.h>
#include <stdarg.h>

#define DXL_USB_VER 20170915

#define CMD_PORT              SerialUSB      // USB
#define DXL_PORT              Serial1
#define DXL_BAUD              57600

#define DXL_TX_BUFFER_LENGTH  64

uint8_t tx_buffer[DXL_TX_BUFFER_LENGTH];

long previousMillis = 0; // will store the last time the LED was updated
int interval = 1000;  // interval at which to blink (in milliseconds)

void setup() {
    pinMode(BOARD_LED_PIN, OUTPUT);

    CMD_PORT.begin(57600);
    DXL_PORT.setDxlMode(true);
    DXL_PORT.begin(DXL_BAUD);

    drv_dxl_begin(0);
    drv_dxl_tx_enable(0, FALSE);

    //while(!Serial);
}

void loop() {
    update_dxl();

    //if ((int)millis() - previousMillis > interval) {
    //    previousMillis = millis();
    //    toggleLED();
    //}

//    if (CMD_PORT.getBaudRate() != DXL_PORT.getBaudRate()) {
//        DXL_PORT.begin(CMD_PORT.getBaudRate());
//    }
}


void update_dxl() {
    int length;
    int i;

    // USB -> DXL
    length = CMD_PORT.available();
    if (length > 0) {
        drv_dxl_tx_enable(0, TRUE);
        for (i = 0; i < length; ++i) {
            DXL_PORT.write(CMD_PORT.read());
            DXL_PORT.flush();
        }
        drv_dxl_tx_enable(0, FALSE);
        //toggleLED();
        digitalWrite(BOARD_LED_PIN, HIGH);
    }

    // DXL -> USB
    length = DXL_PORT.available();
    if (length > 0) {
        if (length > DXL_TX_BUFFER_LENGTH) {
            length = DXL_TX_BUFFER_LENGTH;
        }
        for (i = 0; i < length; ++i) {
            tx_buffer[i] = DXL_PORT.read();
        }
        CMD_PORT.write(tx_buffer, length);
        //CMD_PORT.flush();
        //toggleLED();
        digitalWrite(BOARD_LED_PIN, LOW);
    }
}
