/**
 * ESP32 Logic Analyzer Framework v1.0
 * Author: George Eleftheriadis
 * Purpose: Buffered sampling for digital pins
 *          Basic I2C/SPI/UART signal capture
 *
 * Features:
 * - Configurable number of samples
 * - Buffered storage for fast sampling
 * - Export to CSV (time + channel states)
 * - Modular functions for digitalRead, protocol capture
 */

#include <Arduino.h>

// ---------------------- CONFIG ----------------------
#define SAMPLES 1000            // Number of samples per capture
#define DELAY_US 50             // Delay between samples (microseconds)
#define CHANNELS 4              // Number of digital channels

// Digital pins to capture
const uint8_t CH[CHANNELS] = {32, 33, 25, 26};

// ---------------------- GLOBALS ----------------------
uint8_t buffer[SAMPLES];       // Stores the 4-bit sample per entry
uint32_t timestamps[SAMPLES];  // Timestamp per sample in microseconds

// ---------------------- SETUP ----------------------
void setup() {
    Serial.begin(115200);

    // Configure pins as INPUT_PULLDOWN to avoid floating values
    for (int i = 0; i < CHANNELS; i++) {
        pinMode(CH[i], INPUT_PULLDOWN);
    }

    Serial.println("Logic Analyzer Started");
    Serial.println("sample,time,ch1,ch2,ch3,ch4");  // CSV header
}

// ---------------------- HELPER FUNCTIONS ----------------------

/**
 * Read digital channels and pack them into one byte
 * Bit 0 = CH[0], Bit 1 = CH[1], etc.
 */
uint8_t readSample() {
    uint8_t s = 0;
    for (int i = 0; i < CHANNELS; i++) {
        s |= digitalRead(CH[i]) << i;
    }
    return s;
}

/**
 * Export buffered samples to Serial in CSV format
 */
void exportCSV(uint8_t* buf, uint32_t* ts, int len) {
    for (int i = 0; i < len; i++) {
        Serial.print(i);             // sample index
        Serial.print(",");
        Serial.print(ts[i]);         // timestamp
        for (int j = 0; j < CHANNELS; j++) {
            Serial.print(",");
            Serial.print((buf[i] >> j) & 1);
        }
        Serial.println();
    }
    Serial.println("Capture finished.\n");
}

/**
 * Capture a block of samples
 */
void captureBlock(int sampleCount, int delayUs) {
    for (int i = 0; i < sampleCount; i++) {
        timestamps[i] = micros();
        buffer[i] = readSample();
        delayMicroseconds(delayUs);
    }
}

// ---------------------- MAIN LOOP ----------------------
void loop() {
    // 1️⃣ Capture samples into buffer
    captureBlock(SAMPLES, DELAY_US);

    // 2️⃣ Export to CSV
    exportCSV(buffer, timestamps, SAMPLES);

    delay(1000);  // Pause before next capture
}

/**
 * ---------------------- FUTURE EXTENSIONS ----------------------
 * 1. High-speed sampling using GPIO register access (MHz)
 * 2. Trigger-based capture (edge detection)
 * 3. Protocol decoders:
 *    - I2C: Monitor SDA/SCL and decode start, stop, address, data
 *    - SPI: Monitor CLK/MOSI/MISO/CS and decode bytes
 *    - UART: Monitor TX/RX and decode bytes
 * 4. Multiple capture buffers to increase duration
 * 5. Export directly to PulseView/sigrok formats
 */
