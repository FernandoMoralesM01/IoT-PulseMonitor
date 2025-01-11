#include <Wire.h>
#include "MAX30105.h"

// Constants for sensor configuration
constexpr byte LED_BRIGHTNESS = 0x3F; // 0=Off to 255=50mA
constexpr byte SAMPLE_AVERAGE = 4;    // Reduced for faster acquisition
constexpr byte LED_MODE = 2;          // Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
constexpr int SAMPLE_RATE = 400;      //  // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
constexpr int PULSE_WIDTH = 118;      // Options: 69, 118, 215, 411
constexpr int ADC_RANGE = 4096;       // Options: 2048, 4096, 8192, 16384
constexpr int SERIAL_BAUD = 115200;   // Serial communication baud rate

// Pin definitions
constexpr int SDA_PIN = 14;           // SDA pin
constexpr int SCL_PIN = 15;           // SCL pin
constexpr int INTERRUPT_PIN = 2;      // Interrupt pin

volatile bool dataReady = false;      // Flag for data acquisition interrupt
hw_timer_t* timer = nullptr;          // Timer for periodic interrupts
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

MAX30105 particleSensor;              // Instance of MAX30105


// Interrupt Service Routine for acquiring data
void IRAM_ATTR acquireDataISR() {
  portENTER_CRITICAL_ISR(&timerMux);
  dataReady = true;
  portEXIT_CRITICAL_ISR(&timerMux);
}


// High-pass filter configuration
constexpr float CUTOFF_FREQUENCY = 0.1; // Cutoff frequency in Hz
constexpr float RC = 1.0 / (2.0 * 3.14159265359 * CUTOFF_FREQUENCY);
constexpr float ALPHA = RC / (RC + (1.0 / SAMPLE_RATE));

float prevIR = 0.0, prevFilteredIR = 0.0; // Variables for IR high-pass filter
float prevRed = 0.0, prevFilteredRed = 0.0; // Variables for Red high-pass filter


void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.println("Initializing MAX30105...");

  // Initialize I2C with custom SDA and SCL pins
  Wire.begin(SDA_PIN, SCL_PIN);

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30105 not found. Check wiring and power.");
    while (true); // Halt execution
  }

  // Configure the sensor
  particleSensor.setup(LED_BRIGHTNESS, SAMPLE_AVERAGE, LED_MODE, SAMPLE_RATE, PULSE_WIDTH, ADC_RANGE);

  // Enable data-ready interrupt
  particleSensor.enableDATARDY();

  // Configure timer for periodic interrupts
  timer = timerBegin(0, 80, true); // Timer with 1 microsecond resolution
  timerAttachInterrupt(timer, &acquireDataISR, true);
  timerAlarmWrite(timer, 1000000 / SAMPLE_RATE, true); // Trigger at the sample rate
  timerAlarmEnable(timer);

  Serial.println("Setup complete.");
}



void loop() {
  if (dataReady) {
    portENTER_CRITICAL(&timerMux);
    dataReady = false;
    portEXIT_CRITICAL(&timerMux);

    // Acquire sensor data
    float irValue = particleSensor.getIR();
    float redValue = particleSensor.getRed();

    Serial.print("IR:");
    Serial.print(irValue);
    Serial.print(",RED:");
    Serial.println(redValue);
    
    // Apply high-pass filter to IR
    //float filteredIR = ALPHA * (prevFilteredIR + irValue - prevIR);
    //prevIR = irValue;
    //prevFilteredIR = filteredIR;

    // Apply high-pass filter to Red
    //float filteredRed = ALPHA * (prevFilteredRed + redValue - prevRed);
    //prevRed = redValue;
    //prevFilteredRed = filteredRed;

    //Serial.print("Filtered IR: ");
    //Serial.print(filteredIR);
    //Serial.print(", Filtered Red: ");
    //Serial.println(filteredRed);
  }
}
