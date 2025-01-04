#include <WiFi.h>
#include <Wire.h>
#include "MAX30105.h"

// Wi-Fi credentials
//const char* ssid = "INFINITUM9A90";
//const char* password = "DJag9JyC6c";

const char* ssid = "TP-LINK_92834A";
const char* password = "14189548";


// UDP configuration
const char* udpAddress = "192.168.1.69"; // Replace with the host computer's IP
const int udpPort = 12345;


//const char* udpAddress = "172.174.210.23"; // Replace with the host computer's IP
//const int udpPort = 8000;


// Sensor configuration
constexpr byte LED_BRIGHTNESS = 0x1F;
constexpr byte SAMPLE_AVERAGE = 4;
constexpr byte LED_MODE = 2;
constexpr int SAMPLE_RATE = 400;
constexpr int PULSE_WIDTH = 118;
constexpr int ADC_RANGE = 4096;

// Pin definitions
constexpr int SDA_PIN = D1;           // SDA pin
constexpr int SCL_PIN = D2;           // SCL pin
constexpr int INTERRUPT_PIN = D10;      // Interrupt pin

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

  // Initialize the sensor
  Wire.begin(SDA_PIN, SCL_PIN);
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30105 not found. Check wiring and power.");
    while (true); // Halt execution
  }

  // Configure the sensor
  particleSensor.setup(LED_BRIGHTNESS, SAMPLE_AVERAGE, LED_MODE, SAMPLE_RATE, PULSE_WIDTH, ADC_RANGE);
  particleSensor.enableDATARDY();

  // Configure timer for periodic interrupts
  timer = timerBegin(0, 80, true); // Timer with 1 microsecond resolution
  timerAttachInterrupt(timer, &acquireDataISR, true);
  timerAlarmWrite(timer, 1000000 / SAMPLE_RATE, true); // Trigger at the sample rate
  timerAlarmEnable(timer);
  
  Serial.println("Setup complete.");

}

void loop() {
  // Acquire sensor data
  if (dataReady) {
    dataReady = false;
    float irValue = particleSensor.getIR();
    float redValue = particleSensor.getRed();
    
    // Send data via UDP
    String dataPacket = String(irValue) + "," + String(redValue);
    udp.beginPacket(udpAddress, udpPort);
    udp.print(dataPacket);
    udp.endPacket();

    // Debugging
    // Serial.println("Sent: " + dataPacket);

    // delay(1000 / SAMPLE_RATE);
  }
}
