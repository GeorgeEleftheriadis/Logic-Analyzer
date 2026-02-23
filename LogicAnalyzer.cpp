/**
 * ESP32 Multi-Protocol Logic Analyzer v1.0
 * Author: Tomberar
 *
 * Features:
 * - Capture I2C, SPI, UART signals
 * - Buffered capture for high-speed sampling (~1-2 MHz)
 * - CSV export: sample,time,protocol,event,address/data/byte
 * - Modular: add more protocols or channels easily
 *
 * Notes:
 * - I2C: SDA=SDA_PIN, SCL=SCL_PIN
 * - SPI: CLK=SPI_CLK, MOSI=SPI_MOSI, MISO=SPI_MISO, CS=SPI_CS
 * - UART: TX=UART_TX, RX=UART_RX
 * - Pull-ups required for I2C, SPI idle HIGH recommended
 */

#include <Arduino.h>

// ---------------------- CONFIG ----------------------

// I2C pins
#define SDA_PIN 32
#define SCL_PIN 33

// SPI pins
#define SPI_CLK 25
#define SPI_MOSI 26
#define SPI_MISO 27
#define SPI_CS 14

// UART pins
#define UART_TX 12
#define UART_RX 13

#define SAMPLES 5000       // total samples per capture
#define USEC_DELAY 1       // microseconds between samples

// ---------------------- BUFFERS ----------------------
uint32_t timestamps[SAMPLES];

// I2C buffers
uint8_t sdaBuffer[SAMPLES];
uint8_t sclBuffer[SAMPLES];
int i2cSampleIndex = 0;

// SPI buffers
uint8_t spiClkBuffer[SAMPLES];
uint8_t spiMosiBuffer[SAMPLES];
uint8_t spiMisoBuffer[SAMPLES];
uint8_t spiCsBuffer[SAMPLES];
int spiSampleIndex = 0;

// UART buffers
uint8_t uartTxBuffer[SAMPLES];
uint8_t uartRxBuffer[SAMPLES];
int uartSampleIndex = 0;

// ---------------------- SETUP ----------------------
void setup() {
    Serial.begin(115200);

    // I2C pins
    pinMode(SDA_PIN, INPUT_PULLUP);
    pinMode(SCL_PIN, INPUT_PULLUP);

    // SPI pins
    pinMode(SPI_CLK, INPUT_PULLUP);
    pinMode(SPI_MOSI, INPUT_PULLUP);
    pinMode(SPI_MISO, INPUT_PULLUP);
    pinMode(SPI_CS, INPUT_PULLUP);

    // UART pins
    pinMode(UART_TX, INPUT_PULLUP);
    pinMode(UART_RX, INPUT_PULLUP);

    Serial.println("ESP32 Multi-Protocol Logic Analyzer Started");
}

// ---------------------- HELPER FUNCTIONS ----------------------

/** Fast GPIO read for I2C */
void readI2C(uint8_t &sda, uint8_t &scl) {
    uint32_t gpio = GPIO.in;
    sda = (gpio & (1UL << SDA_PIN)) ? 1 : 0;
    scl = (gpio & (1UL << SCL_PIN)) ? 1 : 0;
}

/** Fast GPIO read for SPI */
void readSPI(uint8_t &clk, uint8_t &mosi, uint8_t &miso, uint8_t &cs) {
    uint32_t gpio = GPIO.in;
    clk  = (gpio & (1UL << SPI_CLK)) ? 1 : 0;
    mosi = (gpio & (1UL << SPI_MOSI)) ? 1 : 0;
    miso = (gpio & (1UL << SPI_MISO)) ? 1 : 0;
    cs   = (gpio & (1UL << SPI_CS)) ? 1 : 0;
}

/** Fast GPIO read for UART */
void readUART(uint8_t &tx, uint8_t &rx) {
    uint32_t gpio = GPIO.in;
    tx = (gpio & (1UL << UART_TX)) ? 1 : 0;
    rx = (gpio & (1UL << UART_RX)) ? 1 : 0;
}

/** Capture fast into buffers */
void captureI2C() {
    i2cSampleIndex = 0;
    for (int i = 0; i < SAMPLES; i++) {
        timestamps[i] = micros();
        readI2C(sdaBuffer[i], sclBuffer[i]);
        i2cSampleIndex++;
        delayMicroseconds(USEC_DELAY);
    }
}

void captureSPI() {
    spiSampleIndex = 0;
    for (int i = 0; i < SAMPLES; i++) {
        timestamps[i] = micros();
        readSPI(spiClkBuffer[i], spiMosiBuffer[i], spiMisoBuffer[i], spiCsBuffer[i]);
        spiSampleIndex++;
        delayMicroseconds(USEC_DELAY);
    }
}

void captureUART() {
    uartSampleIndex = 0;
    for (int i = 0; i < SAMPLES; i++) {
        timestamps[i] = micros();
        readUART(uartTxBuffer[i], uartRxBuffer[i]);
        uartSampleIndex++;
        delayMicroseconds(USEC_DELAY);
    }
}

// ---------------------- CSV EXPORT ----------------------
void exportI2C() {
    Serial.println("sample,time,event,address/data");
    uint8_t prevSDA = 1, prevSCL = 1;
    for (int i = 0; i < i2cSampleIndex; i++) {
        uint8_t sda = sdaBuffer[i];
        uint8_t scl = sclBuffer[i];
        uint32_t t = timestamps[i];

        // START
        if (prevSDA==1 && sda==0 && scl==1) Serial.printf("%d,%lu,START,\n", i, t);
        // STOP
        if (prevSDA==0 && sda==1 && scl==1) Serial.printf("%d,%lu,STOP,\n", i, t);

        prevSDA = sda;
        prevSCL = scl;
    }
}

void exportSPI() {
    Serial.println("sample,time,CLK,MOSI,MISO,CS");
    for (int i = 0; i < spiSampleIndex; i++) {
        Serial.printf("%d,%lu,%d,%d,%d,%d\n",
                      i, timestamps[i],
                      spiClkBuffer[i],
                      spiMosiBuffer[i],
                      spiMisoBuffer[i],
                      spiCsBuffer[i]);
    }
}

void exportUART() {
    Serial.println("sample,time,TX,RX");
    for (int i = 0; i < uartSampleIndex; i++) {
        Serial.printf("%d,%lu,%d,%d\n",
                      i, timestamps[i],
                      uartTxBuffer[i],
                      uartRxBuffer[i]);
    }
}

// ---------------------- MAIN LOOP ----------------------
void loop() {
    // Capture and export one protocol at a time
    captureI2C();
    exportI2C();
    delay(500);

    captureSPI();
    exportSPI();
    delay(500);

    captureUART();
    exportUART();
    delay(1000);
}

/**
 * ---------------------- FUTURE EXTENSIONS ----------------------
 * - Decode I2C bytes/address
 * - Decode SPI bytes with CS/CLK edges
 * - Decode UART bytes with start/stop/data bits
 * - Trigger-based capture
 * - Export combined CSV for multi-protocol visualization
 */
