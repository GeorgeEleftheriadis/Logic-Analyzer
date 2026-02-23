/**
 * ESP32 Ultimate Logic Analyzer v2
 * Author: George Eleftheriadis
 *
 * Features:
 * - Capture and decode I2C, SPI, UART signals
 * - Buffered capture (~1-2 MHz)
 * - Detect I2C START/STOP + address/data
 * - Decode SPI bytes per CS/CLK
 * - Decode UART bytes (8N1)
 * - CSV export: sample,time,protocol,event,address/data/byte
 *
 * Notes:
 * - Pins configurable below
 * - Pull-ups for I2C required
 * - Idle HIGH recommended for SPI/UART lines
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

#define SAMPLES 5000
#define USEC_DELAY 1

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

    Serial.println("Ultimate ESP32 Logic Analyzer Started");
}

// ---------------------- HELPER FUNCTIONS ----------------------

// Fast GPIO reads
void readI2C(uint8_t &sda, uint8_t &scl) {
    uint32_t gpio = GPIO.in;
    sda = (gpio & (1UL << SDA_PIN)) ? 1 : 0;
    scl = (gpio & (1UL << SCL_PIN)) ? 1 : 0;
}

void readSPI(uint8_t &clk, uint8_t &mosi, uint8_t &miso, uint8_t &cs) {
    uint32_t gpio = GPIO.in;
    clk  = (gpio & (1UL << SPI_CLK)) ? 1 : 0;
    mosi = (gpio & (1UL << SPI_MOSI)) ? 1 : 0;
    miso = (gpio & (1UL << SPI_MISO)) ? 1 : 0;
    cs   = (gpio & (1UL << SPI_CS)) ? 1 : 0;
}

void readUART(uint8_t &tx, uint8_t &rx) {
    uint32_t gpio = GPIO.in;
    tx = (gpio & (1UL << UART_TX)) ? 1 : 0;
    rx = (gpio & (1UL << UART_RX)) ? 1 : 0;
}

// ---------------------- CAPTURE FUNCTIONS ----------------------
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

// ---------------------- DECODERS ----------------------

// Simple I2C decoder: START/STOP + address/data
void decodeI2C() {
    Serial.println("sample,time,protocol,event,address/data");
    uint8_t prevSDA=1, prevSCL=1;
    uint8_t byteValue=0;
    int bitCount=0;
    bool inByte=false;
    bool addressCaptured=false;
    uint8_t address=0;

    for (int i=0;i<i2cSampleIndex;i++){
        uint8_t sda=sdaBuffer[i];
        uint8_t scl=sclBuffer[i];
        uint32_t t=timestamps[i];

        if(prevSDA==1 && sda==0 && scl==1){Serial.printf("%d,%lu,I2C,START,\n",i,t); inByte=true; byteValue=0; bitCount=0; addressCaptured=false;}
        if(prevSDA==0 && sda==1 && scl==1){Serial.printf("%d,%lu,I2C,STOP,\n",i,t); inByte=false; bitCount=0;}

        if(prevSCL==0 && scl==1 && inByte){
            byteValue=(byteValue<<1)|sda;
            bitCount++;
            if(bitCount==8){
                if(!addressCaptured){address=byteValue; Serial.printf("%d,%lu,I2C,ADDRESS,0x%02X\n",i,t,address); addressCaptured=true;}
                else Serial.printf("%d,%lu,I2C,DATA,0x%02X\n",i,t,byteValue);
                bitCount=0; byteValue=0;
            }
        }

        prevSDA=sda; prevSCL=scl;
    }
}

// Simple SPI decoder: capture bytes while CS LOW
void decodeSPI() {
    Serial.println("sample,time,protocol,event,MOSI,MISO");
    uint8_t prevClk=0, byteValue=0, bitCount=0;
    bool inByte=false;

    for(int i=0;i<spiSampleIndex;i++){
        uint8_t clk=spiClkBuffer[i];
        uint8_t mosi=spiMosiBuffer[i];
        uint8_t miso=spiMisoBuffer[i];
        uint8_t cs=spiCsBuffer[i];
        uint32_t t=timestamps[i];

        if(cs==0){ // active
            if(prevClk==0 && clk==1){ // rising edge
                byteValue=(byteValue<<1)|(mosi&1);
                bitCount++;
                if(bitCount==8){Serial.printf("%d,%lu,SPI,BYTE,0x%02X,0x%02X\n",i,t,byteValue,miso); byteValue=0; bitCount=0;}
            }
        }

        prevClk=clk;
    }
}

// Simple UART decoder: 8N1, start/stop/data bits
void decodeUART() {
    Serial.println("sample,time,protocol,event,byte");
    uint8_t prevTx=1, bitCount=0, byteValue=0;
    bool inByte=false;

    for(int i=0;i<uartSampleIndex;i++){
        uint8_t tx=uartTxBuffer[i];
        uint32_t t=timestamps[i];

        if(!inByte && prevTx==1 && tx==0){ // start bit
            inByte=true; bitCount=0; byteValue=0;
        }

        if(inByte){
            byteValue=(byteValue>>1)|(tx<<7); // shift in LSB first
            bitCount++;
            if(bitCount==8){Serial.printf("%d,%lu,UART,BYTE,0x%02X\n",i,t,byteValue); inByte=false;}
        }

        prevTx=tx;
    }
}

// ---------------------- MAIN LOOP ----------------------
void loop() {
    captureI2C(); decodeI2C(); delay(500);
    captureSPI(); decodeSPI(); delay(500);
    captureUART(); decodeUART(); delay(1000);
}
