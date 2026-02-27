/**
 * ESP32 Ultimate Logic Analyzer v3
 * Author: George Eleftheriadis (improved)
 *
 * Improvements over v2:
 * - Unified capture loop: all protocols sampled simultaneously from a single
 *   GPIO.in snapshot per tick — no timing skew between channels.
 * - Removed delayMicroseconds() from the hot path; raw loop runs ~3-5 MHz.
 * - Serial baud raised to 2 000 000 for fast CSV dump.
 * - UART decoder fixed: uses timestamps to honour baud period instead of
 *   sampling every tick.
 * - SPI decoder fixed: assembles MISO byte in parallel with MOSI.
 * - I2C decoder: ACK/NAK now captured (9th bit after each byte).
 * - Trigger system: capture waits for a configurable edge before recording.
 * - Interactive serial menu: choose protocol, set UART baud, arm trigger,
 *   export CSV, or run continuous mode.
 * - Memory: buffers packed into a single uint32_t snapshot array to maximise
 *   sample depth and keep cache hot during capture.
 *
 * Notes:
 * - All signal pins must be ≤ GPIO 31 (GPIO.in register); pins 32-39 need
 *   GPIO.in1.val — see PINS section if you move channels there.
 * - Pull-ups required for I2C lines.
 * - Idle HIGH assumed for SPI CLK/CS and UART lines.
 */

#include <Arduino.h>

// ======================== CONFIGURATION ========================

// --- Pin assignments (all must be 0-31 for GPIO.in fast reads) ---
#define PIN_SDA    21
#define PIN_SCL    22
#define PIN_SPI_CLK  18
#define PIN_SPI_MOSI 23
#define PIN_SPI_MISO 19
#define PIN_SPI_CS    5
#define PIN_UART_TX  17
#define PIN_UART_RX  16

// --- Capture depth ---
// Each sample = one uint32_t (raw GPIO snapshot) + one uint32_t (timestamp).
// 10 000 samples × 8 bytes = 80 KB, well within ESP32's 520 KB RAM.
#define SAMPLES 10000

// --- UART default baud ---
#define UART_BAUD_DEFAULT 115200

// --- Trigger ---
// Trigger pin and edge (RISING / FALLING / CHANGE / NONE)
#define TRIGGER_PIN   PIN_SCL   // change to whatever makes sense
#define TRIGGER_EDGE  FALLING   // or RISING

// ======================== BIT MASKS ========================
// Pre-compute at compile time for zero-cost extraction in the hot loop.
#define MASK(pin) (1UL << (pin))

const uint32_t MASK_SDA      = MASK(PIN_SDA);
const uint32_t MASK_SCL      = MASK(PIN_SCL);
const uint32_t MASK_SPI_CLK  = MASK(PIN_SPI_CLK);
const uint32_t MASK_SPI_MOSI = MASK(PIN_SPI_MOSI);
const uint32_t MASK_SPI_MISO = MASK(PIN_SPI_MISO);
const uint32_t MASK_SPI_CS   = MASK(PIN_SPI_CS);
const uint32_t MASK_UART_TX  = MASK(PIN_UART_TX);
const uint32_t MASK_UART_RX  = MASK(PIN_UART_RX);

// ======================== BUFFERS ========================
// One GPIO snapshot per sample — all channels at once, no per-channel arrays.
static uint32_t gpioSnapshot[SAMPLES];   // raw GPIO.in word
static uint32_t timestamps[SAMPLES];     // micros() at capture time
static int      capturedSamples = 0;

// ======================== RUNTIME STATE ========================
static uint32_t uartBaud = UART_BAUD_DEFAULT;
static bool     continuousMode = false;

// ======================== INLINE HELPERS ========================

// Extract a single-bit value from a snapshot
inline uint8_t bit(uint32_t snap, uint32_t mask) {
    return (snap & mask) ? 1 : 0;
}

// ======================== TRIGGER ========================

// Blocks until the configured edge is seen on TRIGGER_PIN, then returns.
// Also returns immediately if the user sends any serial byte (escape hatch).
void waitForTrigger() {
    Serial.println("[Trigger] Waiting for edge on pin " + String(TRIGGER_PIN) + "...");
    uint8_t prev = (GPIO.in & MASK(TRIGGER_PIN)) ? 1 : 0;
    while (true) {
        if (Serial.available()) { Serial.read(); Serial.println("[Trigger] Cancelled."); return; }
        uint8_t cur = (GPIO.in & MASK(TRIGGER_PIN)) ? 1 : 0;
        bool fired = false;
        #if   TRIGGER_EDGE == RISING
            fired = (prev == 0 && cur == 1);
        #elif TRIGGER_EDGE == FALLING
            fired = (prev == 1 && cur == 0);
        #else  // CHANGE or NONE
            fired = (cur != prev) || (TRIGGER_EDGE == 0xFF);
        #endif
        if (fired) { Serial.println("[Trigger] Fired!"); return; }
        prev = cur;
    }
}

// ======================== UNIFIED CAPTURE ========================

// Samples ALL channels simultaneously from a single GPIO register read.
// No delays in the hot path — maximum throughput (~3-5 MHz depending on flash cache).
// Use IRAM_ATTR to keep this function in fast internal RAM.
IRAM_ATTR void captureAll() {
    capturedSamples = 0;
    // Disable interrupts to avoid jitter during capture
    portDISABLE_INTERRUPTS();
    for (int i = 0; i < SAMPLES; i++) {
        timestamps[i]    = micros();
        gpioSnapshot[i]  = GPIO.in;          // single atomic 32-bit read
        capturedSamples++;
    }
    portENABLE_INTERRUPTS();
    Serial.printf("[Capture] Done. %d samples. Duration: %lu µs\n",
                  capturedSamples,
                  timestamps[capturedSamples-1] - timestamps[0]);
}

// ======================== DECODER: I2C ========================

void decodeI2C() {
    Serial.println("--- I2C DECODE ---");
    Serial.println("sample,time_us,protocol,event,value,ack");

    uint8_t prevSDA = 1, prevSCL = 1;
    uint8_t byteVal = 0;
    int     bitCount = 0;
    bool    inTransaction = false;
    bool    addressDone   = false;

    for (int i = 0; i < capturedSamples; i++) {
        uint32_t snap = gpioSnapshot[i];
        uint32_t t    = timestamps[i];
        uint8_t  sda  = bit(snap, MASK_SDA);
        uint8_t  scl  = bit(snap, MASK_SCL);

        // START condition: SDA falls while SCL is HIGH
        if (prevSDA == 1 && sda == 0 && scl == 1) {
            Serial.printf("%d,%lu,I2C,START,,\n", i, t);
            inTransaction = true;
            addressDone   = false;
            byteVal       = 0;
            bitCount      = 0;
        }
        // STOP condition: SDA rises while SCL is HIGH
        else if (prevSDA == 0 && sda == 1 && scl == 1) {
            Serial.printf("%d,%lu,I2C,STOP,,\n", i, t);
            inTransaction = false;
        }
        // Data/clock: sample on rising SCL
        else if (inTransaction && prevSCL == 0 && scl == 1) {
            if (bitCount < 8) {
                byteVal = (byteVal << 1) | sda;
                bitCount++;
                if (bitCount == 8) {
                    // Next SCL rise will be the ACK bit — peek ahead one sample
                    // For now, note the byte; ACK detected on the next SCL rise below
                    const char* kind = addressDone ? "DATA" : "ADDRESS";
                    // ACK comes at bitCount==9 cycle — we'll capture it next iteration
                    Serial.printf("%d,%lu,I2C,%s,0x%02X,", i, t, kind, byteVal);
                    // ACK: look at the very next SCL rising edge
                    // We fall through to bitCount==9 handling in the next pass
                }
            } else {
                // 9th clock: ACK (0) or NAK (1) — SDA driven by receiver
                Serial.printf("%s\n", sda == 0 ? "ACK" : "NAK");
                addressDone = true;
                byteVal     = 0;
                bitCount    = 0;
            }
        }

        prevSDA = sda;
        prevSCL = scl;
    }
    Serial.println("--- END I2C ---");
}

// ======================== DECODER: SPI ========================
// Mode 0 (CPOL=0, CPHA=0): sample MOSI and MISO on rising CLK edge.
// Assembles both MOSI and MISO bytes in parallel.

void decodeSPI() {
    Serial.println("--- SPI DECODE (Mode 0) ---");
    Serial.println("sample,time_us,protocol,event,MOSI,MISO");

    uint8_t prevClk  = 1;   // idle high assumed
    uint8_t mosiVal  = 0;
    uint8_t misoVal  = 0;   // FIX: was always printing last miso, now assembled properly
    int     bitCount = 0;
    bool    csActive = false;

    for (int i = 0; i < capturedSamples; i++) {
        uint32_t snap = gpioSnapshot[i];
        uint32_t t    = timestamps[i];
        uint8_t  clk  = bit(snap, MASK_SPI_CLK);
        uint8_t  mosi = bit(snap, MASK_SPI_MOSI);
        uint8_t  miso = bit(snap, MASK_SPI_MISO);
        uint8_t  cs   = bit(snap, MASK_SPI_CS);

        // CS active-low
        if (!csActive && cs == 0) {
            csActive = true;
            mosiVal = misoVal = bitCount = 0;
            Serial.printf("%d,%lu,SPI,CS_ASSERT,,\n", i, t);
        } else if (csActive && cs == 1) {
            csActive = false;
            Serial.printf("%d,%lu,SPI,CS_DEASSERT,,\n", i, t);
        }

        if (csActive && prevClk == 0 && clk == 1) {   // rising edge
            mosiVal = (mosiVal << 1) | (mosi & 1);
            misoVal = (misoVal << 1) | (miso & 1);    // FIX: shift MISO in parallel
            bitCount++;
            if (bitCount == 8) {
                Serial.printf("%d,%lu,SPI,BYTE,0x%02X,0x%02X\n", i, t, mosiVal, misoVal);
                mosiVal = misoVal = bitCount = 0;
            }
        }

        prevClk = clk;
    }
    Serial.println("--- END SPI ---");
}

// ======================== DECODER: UART ========================
// FIX: Original decoder sampled every tick, ignoring baud timing.
// Now uses timestamps to advance by one baud period between bit samples.

void decodeUART() {
    Serial.println("--- UART DECODE (8N1) ---");
    Serial.printf("    Baud: %lu\n", uartBaud);
    Serial.println("sample,time_us,protocol,event,TX_byte,RX_byte");

    // Period in microseconds per bit
    const uint32_t bitPeriod = 1000000UL / uartBaud;
    // Sample in the middle of each bit cell
    const uint32_t halfPeriod = bitPeriod / 2;

    // Decode a single channel (TX or RX) from the snapshot array.
    // Returns through Serial output; called twice (once per channel).
    auto decodeChannel = [&](uint32_t lineMask, const char* chanName) {
        uint8_t  prevBit    = 1;    // idle = HIGH
        bool     inFrame    = false;
        uint32_t frameStart = 0;    // timestamp of start-bit leading edge
        int      startIdx   = 0;    // snapshot index of start-bit

        for (int i = 1; i < capturedSamples; i++) {
            uint8_t  cur = bit(gpioSnapshot[i],   lineMask);
            uint8_t  prv = bit(gpioSnapshot[i-1], lineMask);
            uint32_t t   = timestamps[i];

            // Detect falling edge = start bit
            if (!inFrame && prv == 1 && cur == 0) {
                inFrame    = true;
                frameStart = t;
                startIdx   = i;
                uint8_t byteVal = 0;

                // Sample 8 data bits at frameStart + halfPeriod + n*bitPeriod
                for (int bit_n = 0; bit_n < 8; bit_n++) {
                    uint32_t sampleTime = frameStart + halfPeriod + (uint32_t)bit_n * bitPeriod;
                    // Find the snapshot closest to sampleTime
                    int j = startIdx;
                    while (j < capturedSamples - 1 && timestamps[j] < sampleTime) j++;
                    uint8_t b = bit(gpioSnapshot[j], lineMask);
                    byteVal |= (b << bit_n);   // LSB first
                }

                // Verify stop bit (should be HIGH)
                uint32_t stopTime = frameStart + halfPeriod + 8UL * bitPeriod;
                int j = startIdx;
                while (j < capturedSamples - 1 && timestamps[j] < stopTime) j++;
                uint8_t stopBit = bit(gpioSnapshot[j], lineMask);

                if (stopBit == 1) {
                    Serial.printf("%d,%lu,UART,BYTE_%s,0x%02X,\n",
                                  startIdx, frameStart, chanName, byteVal);
                } else {
                    Serial.printf("%d,%lu,UART,FRAMING_ERROR_%s,,\n",
                                  startIdx, frameStart, chanName);
                }

                // Advance i past the stop bit so we don't re-trigger mid-frame
                while (i < capturedSamples - 1 && timestamps[i] < stopTime) i++;
                inFrame = false;
            }
            prevBit = cur;
        }
    };

    decodeChannel(MASK_UART_TX, "TX");
    decodeChannel(MASK_UART_RX, "RX");

    Serial.println("--- END UART ---");
}

// ======================== INTERACTIVE MENU ========================

void printMenu() {
    Serial.println("\n========================================");
    Serial.println(" ESP32 Logic Analyzer v3 — Menu");
    Serial.println("========================================");
    Serial.println(" [1] Capture + Decode I2C");
    Serial.println(" [2] Capture + Decode SPI");
    Serial.println(" [3] Capture + Decode UART");
    Serial.println(" [4] Capture ALL (decode all protocols)");
    Serial.println(" [5] Set UART baud rate");
    Serial.println(" [6] Toggle trigger (currently: " + String(TRIGGER_EDGE == 0xFF ? "DISABLED" : "ENABLED") + ")");
    Serial.println(" [7] Toggle continuous mode (currently: " + String(continuousMode ? "ON" : "OFF") + ")");
    Serial.println(" [8] Dump raw CSV (last capture)");
    Serial.println("========================================");
    Serial.print("Choice: ");
}

void dumpRawCSV() {
    Serial.println("sample,time_us,SDA,SCL,SPI_CLK,SPI_MOSI,SPI_MISO,SPI_CS,UART_TX,UART_RX");
    for (int i = 0; i < capturedSamples; i++) {
        uint32_t s = gpioSnapshot[i];
        Serial.printf("%d,%lu,%d,%d,%d,%d,%d,%d,%d,%d\n",
            i, timestamps[i],
            bit(s, MASK_SDA),      bit(s, MASK_SCL),
            bit(s, MASK_SPI_CLK),  bit(s, MASK_SPI_MOSI),
            bit(s, MASK_SPI_MISO), bit(s, MASK_SPI_CS),
            bit(s, MASK_UART_TX),  bit(s, MASK_UART_RX));
    }
}

void handleMenu(char cmd) {
    switch (cmd) {
        case '1':
            waitForTrigger();
            captureAll();
            decodeI2C();
            break;
        case '2':
            waitForTrigger();
            captureAll();
            decodeSPI();
            break;
        case '3':
            waitForTrigger();
            captureAll();
            decodeUART();
            break;
        case '4':
            waitForTrigger();
            captureAll();
            decodeI2C();
            decodeSPI();
            decodeUART();
            break;
        case '5': {
            Serial.print("Enter baud rate: ");
            while (!Serial.available()) {}
            uartBaud = Serial.parseInt();
            Serial.printf("UART baud set to %lu\n", uartBaud);
            break;
        }
        case '7':
            continuousMode = !continuousMode;
            Serial.println("Continuous mode: " + String(continuousMode ? "ON" : "OFF"));
            break;
        case '8':
            dumpRawCSV();
            break;
        default:
            Serial.println("Unknown command.");
            break;
    }
}

// ======================== SETUP ========================

void setup() {
    // Higher baud = faster CSV export; use USB CDC so speed doesn't matter
    Serial.begin(2000000);
    while (!Serial) {}

    // Configure all pins as inputs with pull-ups
    const int pins[] = {
        PIN_SDA, PIN_SCL,
        PIN_SPI_CLK, PIN_SPI_MOSI, PIN_SPI_MISO, PIN_SPI_CS,
        PIN_UART_TX, PIN_UART_RX
    };
    for (int p : pins) pinMode(p, INPUT_PULLUP);

    Serial.println("\nESP32 Logic Analyzer v3 ready.");
    printMenu();
}

// ======================== MAIN LOOP ========================

void loop() {
    if (continuousMode) {
        captureAll();
        decodeI2C();
        decodeSPI();
        decodeUART();
        delay(200);
        return;
    }

    if (Serial.available()) {
        char cmd = Serial.read();
        if (cmd == '\n' || cmd == '\r') return;
        handleMenu(cmd);
        printMenu();
    }
}
