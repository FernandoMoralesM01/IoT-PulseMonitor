#include <WiFi.h>
#include <Wire.h>

// Wi-Fi credentials
const char* ssid = "TP-LINK_92834A";
const char* password = "14189548";


// UDP configuration
const char* udpAddress = "192.168.1.69"; // Replace with the host computer's IP
const int udpPort = 12345;


//const char* udpAddress = "172.174.210.23"; // Replace with the host computer's IP
//const int udpPort = 8000;


// Sensor configuration
constexpr int SAMPLE_RATE = 400;

volatile bool dataReady = false;      // Flag for data acquisition interrupt
hw_timer_t* timer = nullptr;          // Timer for periodic interrupts
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;


// Wi-Fi and UDP objects
WiFiUDP udp;

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

  // Configure the sensor

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
    uint32_t signal;
    signal = analogReadMilliVolts(A0);
    // Send data via UDP
    String dataPacket = String(signal) + "," + String(signal);
    udp.beginPacket(udpAddress, udpPort);
    udp.print(dataPacket);
    udp.endPacket();

    // Debugging
    // Serial.println("Sent: " + dataPacket);

    // delay(1000 / SAMPLE_RATE);
  }
}
