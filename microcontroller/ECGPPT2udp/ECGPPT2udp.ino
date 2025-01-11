#include <WiFi.h>
#include <Wire.h>
#include "MAX30105.h"

#define MAX_LENWIN 500
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

float ir_arr[MAX_LENWIN];
float red_arr[MAX_LENWIN];

float filtir_arr[MAX_LENWIN];
float filtred_arr[MAX_LENWIN];



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

  for(int i = 0; i < ir_arr[i]!=NULL; i++)
  {
    ir_arr[i] = 0;
    red_arr[i] = 0;
    filtir_arr[i] = 0;
    filtred_arr[i] = 0;
  }

  Serial.println("Setup complete.");


}

float b_ecg[] = {0.48083843,  0.        , -0.96167686,  0.        ,  0.48083843};
float a_ecg[] = { 1.        , -1.17455937, -0.21696622,  0.14704836,  0.25232463};

float b_ppt[] = { 0.13432327,  0.        , -0.13432327};
float a_ppt[] = { 1.        , -1.72963133,  0.73135345};

float x_ecg[5] = {0, 0, 0, 0, 0};
float y_ecg[5] = {0, 0, 0, 0, 0};



float x_ir[3] = {0, 0, 0};
float y_ir[3] = {0, 0, 0};

float x_red[3] = {0, 0, 0};
float y_red[3] = {0, 0, 0};


float applyFilter(float input, float* x, float* y, float* b, float* a, int order) {
  for (int i = order; i > 0; i--) {
    x[i] = x[i - 1];
    y[i] = y[i - 1];
  }

  x[0] = input;

  y[0] = b[0] * x[0];
  for (int i = 1; i <= order; i++) {
    y[0] += b[i] * x[i] - a[i] * y[i];
  }

  return y[0];
}

int index_count = 0;

float get_mean(float *arr)
{
  int i;
  float value = 0;
  for(i = 0; arr[i]!= NULL; i++)
    value += arr[i];
  return value /= i;
}

float get_peak2peak_distance(float *arr)
{
  float min_val = 10000;
  float max_val = -10000;
  int i;
  for(i = 0; arr[i]!=NULL; i++)
    if (arr[i] < min_val)
      min_val = arr[i];
  
    if (arr[i] > max_val)
      max_val = arr[i];
    
    return (max_val - min_val);
}

float get_spo2(float *arr_ir, float *arr_red, float *filt_ir, float *filt_red)
{
  float ir_DC = get_mean(arr_ir);
  float red_DC = get_mean(arr_red);

  float ir_AC = get_peak2peak_distance(filt_ir);
  float red_AC = get_peak2peak_distance(filt_red);

  float R = (red_AC / red_DC) / (ir_AC / ir_DC);
  float a = 1.5958422;
  float b = -34.6596622;
  float c = 112.6898759 ;
  return (a * (R*R) + b*R + c);
}

void loop() {
  if (dataReady) {
    dataReady = false;
    
    if (index_count >= 300);
      index_count = 0;
    
    float irValue = particleSensor.getIR();
    float redValue = particleSensor.getRed();

    ir_arr[index_count] = irValue;
    red_arr[index_count] = redValue;
    
    float spo2 = get_spo2(ir_arr, red_arr, filtir_arr, filtred_arr);

    float signal = analogReadMilliVolts(A0);
    float filteredECG = applyFilter(signal, x_ecg, y_ecg, b_ecg, a_ecg, 4);
    float filteredirValue= applyFilter(irValue, x_ir, y_ir, b_ppt, a_ppt, 2);
    float filteredredValue = applyFilter(redValue, x_red, y_red, b_ppt, a_ppt, 2);
    filtir_arr[index_count] = filteredirValue;
    filtred_arr[index_count] = filteredredValue;
    String dataPacket = String(filteredirValue) + "," + String(filteredredValue) + "," + String(filteredECG) + "," + String(spo2);

    udp.beginPacket(udpAddress, udpPort);
    udp.print(dataPacket);
    udp.endPacket();
    index_count += 1;

  }
}
