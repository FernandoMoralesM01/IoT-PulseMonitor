#include <WiFi.h>
#include <Wire.h>
#include "MAX30105.h"

// Wi-Fi credentials
const char* ssid = "TP-LINK_92834A";
const char* password = "14189548";

// UDP configuration
const char* udpAddress = "192.168.1.69"; // Replace with the host computer's IP
const int udpPort = 12345;

// Sensor configuration for MAX30105
constexpr byte LED_BRIGHTNESS = 0x1F;
constexpr byte SAMPLE_AVERAGE = 1;
constexpr byte LED_MODE = 2;
constexpr int SAMPLE_RATE = 200;
constexpr int PULSE_WIDTH = 118;
constexpr int ADC_RANGE = 4096;

// Pin definitions
constexpr int SDA_PIN = D1;           // SDA pin
constexpr int SCL_PIN = D2;           // SCL pin
constexpr int INTERRUPT_PIN = D10;    // Interrupt pin

volatile bool dataReady = false;      // Flag for data acquisition interrupt
hw_timer_t* timer = nullptr;          // Timer for periodic interrupts
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Wi-Fi and UDP objects
WiFiUDP udp;
MAX30105 particleSensor;

// Interrupt Service Routine for acquiring data
void IRAM_ATTR acquireDataISR() {
  portENTER_CRITICAL_ISR(&timerMux);
  dataReady = true;
  portEXIT_CRITICAL_ISR(&timerMux);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing...");

  // Initialize Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to Wi-Fi...");
  }
  Serial.println("Connected to Wi-Fi");

  // Initialize the MAX30105 sensor
  Wire.begin(SDA_PIN, SCL_PIN);
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30105 not found. Check wiring and power.");
    while (true); // Halt execution
  }

  // Configure the MAX30105 sensor
  particleSensor.setup(LED_BRIGHTNESS, SAMPLE_AVERAGE, LED_MODE, SAMPLE_RATE, PULSE_WIDTH, ADC_RANGE);
  particleSensor.enableDATARDY();

  // Configure timer for periodic interrupts
  timer = timerBegin(0, 80, true); // Timer with 1 microsecond resolution
  timerAttachInterrupt(timer, &acquireDataISR, true);
  timerAlarmWrite(timer, 1000000 / SAMPLE_RATE, true); // Trigger at the sample rate
  timerAlarmEnable(timer);

  Serial.println("Setup complete.");
}

float b_ecg[] = {0.48083843,  0.        , -0.96167686,  0.        ,  0.48083843};
float a_ecg[] = { 1.        , -1.17455937, -0.21696622,  0.14704836,  0.25232463};

float b_ppt[] = { 0.13432327,  0.        , -0.13432327};
float a_ppt[] = { 1.        , -1.72963133,  0.73135345};

float x_ecg[5] = {0, 0, 0, 0, 0};
float y_ecg[5] = {0, 0, 0, 0, 0};

float x_ppt[3] = {0, 0, 0};
float y_ppt[3] = {0, 0, 0};

float applyFilter(float input, float* x, float* y, float* b, float* a, int order) {
  // Shift buffers
  for (int i = order; i > 0; i--) {
    x[i] = x[i - 1];
    y[i] = y[i - 1];
  }

  // Add new input
  x[0] = input;

  // Apply the filter equation
  y[0] = b[0] * x[0];
  for (int i = 1; i <= order; i++) {
    y[0] += b[i] * x[i] - a[i] * y[i];
  }

  return y[0];
}

void loop() {
  // Acquire sensor data
  if (dataReady) {
    dataReady = false;

    // Read MAX30105 data
    float irValue = particleSensor.getIR();
    float redValue = particleSensor.getRed();

    // Read analog sensor data
    float signal = analogReadMilliVolts(A0);
    float filteredECG = applyFilter(signal, x_ecg, y_ecg, b_ecg, a_ecg, 4);
    float filteredirValue= applyFilter(irValue, x_ppt, y_ppt, b_ppt, a_ppt, 2);
    float filteredredValue = applyFilter(irValue, x_ppt, y_ppt, b_ppt, a_ppt, 2);
    // Combine data into a single UDP packet
    String dataPacket = String(filteredirValue) + "," + String(redValue) + "," + String(filteredECG);

    // Send data via UDP
    udp.beginPacket(udpAddress, udpPort);
    udp.print(dataPacket);
    udp.endPacket();

    // Debugging
    // Serial.println("Sent: " + dataPacket);
  }
}
