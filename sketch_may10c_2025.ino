// Include necessary libraries
#include <Wire.h>                    // I2C communication library
#include <Adafruit_GFX.h>           // Core graphics library for OLED
#include <Adafruit_SSD1306.h>       // Library for SSD1306 OLED display
#include "BluetoothSerial.h"        // Library for Bluetooth Serial communication

BluetoothSerial SerialBT;           // Create Bluetooth serial object
Adafruit_SSD1306 display(128, 64, &Wire);  // Create OLED display object (128x64 resolution using I2C)

// Define constants
#define SCREEN_ADDRESS 0x3C         // I2C address for the OLED display
#define CI_V_ADDRESS 0xA4           // CI-V address of the IC-705 transceiver (source address)
#define DAC_PIN 25                  // GPIO25 is connected to internal DAC1 of the ESP32

// Define a structure to represent each ham radio band with voltage settings
struct BandSetting {
  const char* name;                 // Name of the band (e.g., "40m")
  uint32_t freqLow;                 // Lower frequency boundary in Hz
  uint32_t freqHigh;                // Upper frequency boundary in Hz
  float voltage;                    // Voltage to output on DAC for this band
};

// List of supported bands and their frequency ranges with corresponding DAC voltages
BandSetting bands[] = {
  {"160m = 0.23V", 800000, 2999999, 0.15},
  {"80m = 0.46V",  3000000, 4499999, 0.39},
  {"60m = 0.69V",  4500000, 5999999, 0.63},
  {"40m = 0.92V",  6000000, 8999999, 0.90},
  {"30m = 1.15V", 9000000, 12999999, 1.15},
  {"20m = 1.38V", 13000000, 16999999, 1.38},
  {"17m = 1.61V", 17000000, 19999999, 1.61},
  {"15m = 1.84V", 20000000, 22999999, 1.88},
  {"12m = 2.07V", 23000000, 25999999, 2.12},
  {"10m = 2.30V", 26000000, 39999999, 2.36},
  {"6m = 2.53V",  40000000, 60000000, 2.63}
};

void setup() {
  Serial.begin(115200);             // Initialize serial monitor
  Wire.begin();                     // Initialize I2C bus

  // Initialize the OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("SSD1306 OLED not found!");
    while (1); // Stop execution if OLED initialization fails
  }

  // Show initial message on OLED
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.println("IC-705 / XPA125b");
  display.println();
  display.println("V1.00a May 10, 2025");
  display.println();
  display.println();
  display.println("Waiting for CI-V...");
  display.display();

  // Initialize Bluetooth with name "IC705_XPA125B"
  SerialBT.begin("IC705_XPA125B");
  Serial.println("Bluetooth ready.");
}

void loop() {
  static uint8_t buffer[12];        // Buffer to store incoming CI-V packet
  static uint8_t idx = 0;           // Index for buffer position

  // Read bytes from Bluetooth if available
  while (SerialBT.available()) {
    uint8_t b = SerialBT.read();    // Read a byte from Bluetooth
    Serial.printf("%02X ", b);      // Print received byte in HEX for debugging

    // If it's the first byte, wait for the CI-V preamble 0xFE
    if (idx == 0 && b != 0xFE) continue;
    buffer[idx++] = b;              // Store the byte in buffer

    // Process packet when it's complete (ends with 0xFD or max size reached)
    if (idx >= 12 || b == 0xFD) {
      // Valid CI-V frame must start with 0xFE 0xFE and end with 0xFD
      if (idx >= 8 && buffer[0] == 0xFE && buffer[1] == 0xFE && buffer[idx - 1] == 0xFD) {
        parseCIV(buffer, idx);      // Pass to parser function
      }
      idx = 0;                      // Reset buffer index for next packet
      Serial.println();             // Print newline for clarity
    }
  }
}

// Parse CI-V packet and extract frequency information
void parseCIV(uint8_t *packet, uint8_t len) {
  if (packet[3] != CI_V_ADDRESS) return;  // Ignore packets not from IC-705

  // Check if this is a frequency data command (0x00)
  if (packet[4] == 0x00) {
    uint32_t freq = decodeFrequency(&packet[5]);  // Decode frequency from 5 BCD bytes
    float voltage = 0;
    const char* band = getBand(freq, voltage);    // Identify band and voltage from frequency

    // Debug output
    Serial.printf("Freq: %lu Hz | Band: %s | Voltage: %.2f V\n", freq, band, voltage);

    setBandVoltage(voltage);         // Output corresponding voltage via DAC
    updateDisplay(freq, band, voltage); // Update OLED with new values
  }
}

// Decode 5 BCD-encoded bytes into a 32-bit frequency in Hz
uint32_t decodeFrequency(uint8_t *data) {
  uint32_t freq = 0;
  uint32_t multiplier = 1;

  // CI-V frequency is stored as 5 bytes in little-endian BCD
  for (int i = 0; i < 5; i++) {
    uint8_t bcd = data[i];
    uint8_t lowNibble = bcd & 0x0F;         // Units digit
    uint8_t highNibble = (bcd >> 4) & 0x0F; // Tens digit
    freq += (lowNibble + highNibble * 10) * multiplier;
    multiplier *= 100;                      // Increase magnitude for each byte
  }

  return freq;  // Frequency in Hz
}

// Match frequency to known band, and return name and voltage
const char* getBand(uint32_t freq, float &voltage) {
  for (auto &b : bands) {
    if (freq >= b.freqLow && freq <= b.freqHigh) {
      voltage = b.voltage;
      return b.name;
    }
  }
  voltage = 0.0; // Default if no match
  return "Unknown";
}

// Output voltage to ESP32 DAC pin
void setBandVoltage(float volts) {
  volts = constrain(volts, 0.0, 3.3);                  // Limit voltage range to 0–3.3V
  uint8_t value = (uint8_t)((volts / 3.3) * 255);       // Convert voltage to 8-bit DAC value
  dacWrite(DAC_PIN, value);                            // Write analog voltage
}

// Display frequency, band, and voltage on the OLED screen
void updateDisplay(uint32_t freq, const char* band, float voltage) {
  display.clearDisplay();                              // Clear screen

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Freq:");

  display.setTextSize(2);
  display.setCursor(0, 10);
  display.printf("%.3f MHz", freq / 1e6);              // Convert Hz to MHz

  display.setTextSize(1);
  display.setCursor(0, 35);
  display.print("Band: ");
  display.print(band);

  display.setCursor(0, 50);
  display.printf("DAC: %.2f V Corrected", voltage);              // Show voltage output

  display.display();                                   // Refresh OLED display
}
