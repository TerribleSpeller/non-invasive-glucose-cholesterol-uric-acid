#include <Wire.h>
#include "MAX30100.h" 
#include "MAX30100_PulseOximeter.h"  
#include "RandomForestModel.h" //gsr_1
#include "UricAcid-RandomForest.h" //gsr_2
#include "Chol-RandomForest.h" //gsr_3
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <cstring>

//Important Defs
#define WIRE1_SCL 5
#define WIRE1_SDA 17
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1 //NOT USED. 

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);

const int LED = 13;
const int GSR = 36;

MAX30100 rawSensor;
PulseOximeter pox;

// Error flags
bool oled_ok = false;
bool max30100_ok = false;
bool pulseOx_ok = false;

const unsigned long COLLECTION_TIME_PER_PHASE = 60000; // 1 minute per phase
unsigned long startTime = 0;
bool isCollecting = false;
int phase = 0; //0 = not started, 1 = raw phase, 2 = processed phase
unsigned long lastPrint = 0;

// Finger detection variables
bool fingerDetected = false;
unsigned long fingerDetectedTime = 0;
const unsigned long FINGER_STABLE_TIME = 2000; // Wait 2 seconds after finger detection to start
const int FINGER_THRESHOLD = 3000; // Adjust based on your sensor - typical range 30000-60000
const int SAMPLES_FOR_DETECTION = 10;
int detectionSamples[10];
int detectionIndex = 0;
bool detectionBufferFull = false;

//Buffer
const int BUFFER_SIZE = 3000;
float gsrBuffer[BUFFER_SIZE];
float irBuffer[BUFFER_SIZE];
float redBuffer[BUFFER_SIZE];
float irAvgPeak; 
int bufferIndex = 0;
bool bufferFull = false;
float quartile25;
float IQRRawInfra;
float IRq1, IRq3;
float KurtosisRawIR;

float KurtosisRawRED;
float currentEstimateRF;
float currentEstimatePoly;
float currentUricEstimateRF;
float currentUricEstimatePoly;
float currentCholEstimateRF;
float currentCholEstimatePoly;
float currentEstimate;
float currentUricEstimate;
float currentCholEstimate;
float gsrMin, gsrMean;
float irMin, irMean;
float redMin, redMean;
bool gotQuartile = false;
bool gotPeaks = false;

// Updated polynomial coefficients with proper decimal numbers (no scientific notation)
const float polyCoeffs[9] = {
  123.00005783,       // c0 - intercept (x^0)
  0.000000000000000000289602108,    // c1 - 2.89602108e-19
  -0.0000000000000000000000527951084, // c2 - -5.27951084e-23
  -0.00000000000000000000268137510,   // c3 - -2.68137510e-21
  -0.000000000000000000963931628,     // c4 - -9.63931628e-19
  -0.000000000000000267549521,        // c5 - -2.67549521e-16
  -0.0000000000457884267,             // c6 - -4.57884267e-14
  0.000000000000000176492640,         // c7 - 1.76492640e-16
  -0.000000000000000000227694393      // c8 - -2.27694393e-19
};

const float UricpolyCoeffs[5] = {
  6.38647957,           // c0 - intercept (x^0)
  -0.09240583,         // c1 - x^1
  -0.08989593,          // c2 - x^2
  0.01261649,           // c3 - x^3
  -0.00022762           // c4 - x^4
};

const float CholpolyCoeffs[5] = {
  188.01452453,      // c0 - intercept (x^0)
  4.44063723,        // c1 - x^1
  -6.82298462,       // c2 - x^2
  0.82880367,        // c3 - x^3
  -0.01462045        // c4 - x^4
};

struct PeakResult {
  int peakCount;
  float averagePeakPosition;
};

struct PeakToPeakResult {
  float peakToPeakValue;
  float minValue;
  float maxValue;
  int sampleCount;
};

struct StatisticalResults {
  float minValue;
  float meanValue;
  float maxValue;
  int sampleCount;
};

void setup() {
  Serial.begin(115200);
  analogReadResolution(10);  
  Wire.begin();      
  delay(2000);       
  Wire1.begin(WIRE1_SDA, WIRE1_SCL);  

  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  // Initialize OLED
  Serial.print("Initializing OLED on Wire1 (SDA=16, SCL=17)...");
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    oled_ok = false;
    // Can't use OLED for display since it failed, just blink LED
    while(1) {
      digitalWrite(LED, HIGH);
      delay(100);
      digitalWrite(LED, LOW);
      delay(100);
    }
  } else {
    oled_ok = true;
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Diploy Pepper");
    display.display(); 
    Serial.println("SUCCESS");
  }

  // Initialize MAX30100 for raw data
  Serial.print("Initializing MAX30100 for raw data on Wire (SDA=21, SCL=22)...");
  if (!rawSensor.begin()) {
    Serial.println("FAILED");
    Serial.println("Check wiring:");
    Serial.println("VCC -> 3.3V/5V");
    Serial.println("GND -> GND");
    Serial.println("SCL -> GPIO22");
    Serial.println("SDA -> GPIO21");
    max30100_ok = false;
    
    // Show error on OLED if available
    if (oled_ok) {
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.println("MAX30100 ERROR!");
      display.println("Check wiring:");
      display.println("SCL->GPIO22");
      display.println("SDA->GPIO21");
      display.println("VCC->3.3V/5V");
      display.println("GND->GND");
      display.display();
    }
    
    // Blink LED to indicate error
    while(1) {
      digitalWrite(LED, HIGH);
      delay(500);
      digitalWrite(LED, LOW);
      delay(500);
    }
  } else {
    max30100_ok = true;
    Serial.println("SUCCESS");
    rawSensor.setMode(MAX30100_MODE_SPO2_HR);
    rawSensor.setLedsCurrent(MAX30100_LED_CURR_50MA, MAX30100_LED_CURR_46_8MA);
    rawSensor.setLedsPulseWidth(MAX30100_SPC_PW_1600US_16BITS);
    rawSensor.setSamplingRate(MAX30100_SAMPRATE_100HZ);
    rawSensor.setHighresModeEnabled(true);
  }

  // Initialize PulseOximeter
  Serial.print("Initializing PulseOximeter on Wire (SDA=21, SCL=22)...");
  if (!pox.begin()) {
    Serial.println("FAILED");
    pulseOx_ok = false;
    
    // Show error on OLED if available
    if (oled_ok) {
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("PulseOx ERROR!");
      display.println("Check MAX30100");
      display.display();
    }
    
    // Blink LED to indicate error but continue (might still work)
    for(int i = 0; i < 5; i++) {
      digitalWrite(LED, HIGH);
      delay(200);
      digitalWrite(LED, LOW);
      delay(200);
    }
  } else {
    pulseOx_ok = true;
    Serial.println("SUCCESS");
    pox.setIRLedCurrent(MAX30100_LED_CURR_11MA);
  }

  Serial.println("\n==========================================");
  Serial.println("Place your finger on the sensor to start");
  Serial.println("The system will automatically detect your finger");
  Serial.println("and begin the 2-minute data collection");
  Serial.println("==========================================");
  
  // Initialize detection buffer
  for (int i = 0; i < SAMPLES_FOR_DETECTION; i++) {
    detectionSamples[i] = 0;
  }
}

void loop() {
  // Only update sensors if they're working
  if (max30100_ok) {
    rawSensor.update();
  }
  if (pulseOx_ok) {
    pox.update();
  }

  // Check for finger if not collecting and sensors are working
  if (!isCollecting && max30100_ok) {
    checkForFinger();
  }

  updateDisplay();

  // Keep serial command as backup option
  if (Serial.available() > 0) {
    char command = Serial.read();
    if (command == 's' || command == 'S') {
      if (max30100_ok && pulseOx_ok) {
        fingerDetected = true; // Force start
        startCollection();
      } else {
        Serial.println("Cannot start: Sensors not initialized properly");
        if (oled_ok) {
          display.clearDisplay();
          display.setCursor(0, 0);
          display.println("Sensor Error!");
          display.println("Check connections");
          display.display();
          delay(2000);
        }
      }
    }
  }

  if (isCollecting) {
    unsigned long elapsed = millis() - startTime;
    
    if (phase == 1) {
      collectRawData(elapsed);
    }
    else if (phase == 2) {
      collectProcessedData(elapsed);
    }
  }
}

void checkForFinger() {
  uint16_t ir_raw, red_raw;
  
  // Get current readings
  if (rawSensor.getRawValues(&ir_raw, &red_raw)) {
    // Add to detection buffer
    detectionSamples[detectionIndex] = ir_raw;
    detectionIndex = (detectionIndex + 1) % SAMPLES_FOR_DETECTION;
    if (detectionIndex == 0) {
      detectionBufferFull = true;
    }
    
    // Only check if buffer is full
    if (detectionBufferFull) {
      // Calculate average of buffer
      long sum = 0;
      for (int i = 0; i < SAMPLES_FOR_DETECTION; i++) {
        sum += detectionSamples[i];
      }
      float average = sum / (float)SAMPLES_FOR_DETECTION;
      
      // Check if finger is detected (IR value above threshold)
      if (average > FINGER_THRESHOLD) {
        if (!fingerDetected) {
          fingerDetectedTime = millis();
          fingerDetected = true;
          Serial.println("Finger detected! Waiting for stable reading...");
        } else {
          // Check if finger has been stable for required time
          if (millis() - fingerDetectedTime >= FINGER_STABLE_TIME) {
            Serial.println("Finger stable! Starting data collection...");
            startCollection();
          }
        }
      } else {
        // Finger removed or not detected
        if (fingerDetected) {
          fingerDetected = false;
          Serial.println("Finger removed. Waiting for finger...");
        }
      }
    }
  }
}

// Updated polynomial evaluation functions with correct array sizes
float evaluatePolynomialHorner(float x) {
  // Using Horner's method: (((((c8*x + c7)*x + c6)*x + c5)*x + c4)*x + c3)*x + c2)*x + c1)*x + c0
  float result = polyCoeffs[8];  // Start with highest degree coefficient
  for (int i = 7; i >= 0; i--) {
    result = result * x + polyCoeffs[i];
  }
  return result;
}

float UriAcidevaluatePolynomialHorner(float x) {
  // Using Horner's method for 4th degree polynomial
  float result = UricpolyCoeffs[4];  // Start with highest degree coefficient
  for (int i = 3; i >= 0; i--) {
    result = result * x + UricpolyCoeffs[i];
  }
  return result;
}

float CholevaluatePolynomialHorner(float x) {
  // Using Horner's method for 4th degree polynomial
  float result = CholpolyCoeffs[4];  // Start with highest degree coefficient
  for (int i = 3; i >= 0; i--) {
    result = result * x + CholpolyCoeffs[i];
  }
  return result;
}

float calculatePercentile(float* data, int length, float percentile) {
  if (length == 0) return 0;
  
  static float sorted[BUFFER_SIZE];
  memcpy(sorted, data, length * sizeof(float));
  qsort(sorted, length, sizeof(float), compareFloat);
  
  int position = (int)(length * percentile);
  if (position >= length) position = length - 1;
  return sorted[position];
}

void calculateIQR(float* data, int length, float& q1, float& q3, float& iqr) {
  q1 = calculatePercentile(data, length, 0.25);
  q3 = calculatePercentile(data, length, 0.75);
  iqr = q3 - q1;
}

float calculateKurtosis(float* data, int length) {
  if (length < 4) return 0;
  
  float mean = 0;
  for (int i = 0; i < length; i++) {
    mean += data[i];
  }
  mean /= length;
  
  float sumSquaredDiff = 0;
  float sumFourthDiff = 0;
  
  for (int i = 0; i < length; i++) {
    float diff = data[i] - mean;
    float diffSquared = diff * diff;
    sumSquaredDiff += diffSquared;
    sumFourthDiff += diffSquared * diffSquared;
  }
  
  float variance = sumSquaredDiff / length;
  float fourthMoment = sumFourthDiff / length;
  float kurtosis = fourthMoment / (variance * variance) - 3;
  
  return kurtosis;
}

float peakToPeak(const float* data, size_t length) {
    if (length == 0) return 0.0f;
    
    float minVal = data[0];
    float maxVal = data[0];
    
    for (size_t i = 1; i < length; i++) {
        if (data[i] < minVal) minVal = data[i];
        if (data[i] > maxVal) maxVal = data[i];
    }
    
    return maxVal - minVal;
}

float calculateMin(float* data, int length) {
  if (length <= 0) return 0;
  
  float minVal = data[0];
  for (int i = 1; i < length; i++) {
    if (data[i] < minVal) {
      minVal = data[i];
    }
  }
  return minVal;
}

float calculateMean(float* data, int length) {
  if (length <= 0) return 0;
  
  float sum = 0;
  for (int i = 0; i < length; i++) {
    sum += data[i];
  }
  return sum / length;
}

PeakToPeakResult calculatePeakToPeakStatic(const float* data, int length) {
  PeakToPeakResult result = {0.0f, 0.0f, 0.0f, 0};
  
  if (length <= 0 || data == nullptr) {
    return result;
  }
  
  float minVal = data[0];
  float maxVal = data[0];
  
  for (int i = 1; i < length; i++) {
    if (data[i] < minVal) {
      minVal = data[i];
    }
    if (data[i] > maxVal) {
      maxVal = data[i];
    }
  }
  
  result.peakToPeakValue = maxVal - minVal;
  result.minValue = minVal;
  result.maxValue = maxVal;
  result.sampleCount = length;
  
  return result;
}

PeakResult findPeaks(const float* data, int length, int distance = 5) {
  PeakResult result = {0, 0.0f};
  
  if (length < 3) return result;
  
  bool* isPeak = new bool[length]();
  int peakCount = 0;
  
  for (int i = 1; i < length - 1; i++) {
    if (data[i] > data[i-1] && data[i] > data[i+1]) {
      isPeak[i] = true;
      peakCount++;
    }
  }
  
  if (peakCount > 0 && distance > 0) {
    int lastPeakPos = -distance;
    
    for (int i = 0; i < length; i++) {
      if (isPeak[i]) {
        if (i - lastPeakPos >= distance) {
          lastPeakPos = i;
        } else {
          isPeak[i] = false;
          peakCount--;
        }
      }
    }
  }
  
  float sumPositions = 0;
  int validPeaks = 0;
  
  for (int i = 0; i < length; i++) {
    if (isPeak[i]) {
      sumPositions += i;
      validPeaks++;
    }
  }
  
  result.peakCount = validPeaks;
  result.averagePeakPosition = (validPeaks > 0) ? (sumPositions / validPeaks) : 0;
  
  delete[] isPeak;
  return result;
}

int compareFloat(const void* a, const void* b) {
  float fa = *(const float*)a;
  float fb = *(const float*)b;
  if (fa < fb) return -1;
  if (fa > fb) return 1;
  return 0;
}

PeakResult findPeaksStatic(const float* data, int length, int distance = 5) {
  PeakResult result = {0, 0.0f};
  
  if (length < 3 || length > BUFFER_SIZE) return result;
  
  static bool isPeak[BUFFER_SIZE];
  
  int peakCount = 0;
  for (int i = 1; i < length - 1; i++) {
    if (data[i] > data[i-1] && data[i] > data[i+1]) {
      isPeak[i] = true;
      peakCount++;
    } else {
      isPeak[i] = false;
    }
  }
  isPeak[0] = false;
  isPeak[length-1] = false;
  
  if (peakCount > 0 && distance > 0) {
    int lastPeakPos = -distance;
    for (int i = 0; i < length; i++) {
      if (isPeak[i]) {
        if (i - lastPeakPos >= distance) {
          lastPeakPos = i;
        } else {
          isPeak[i] = false;
          peakCount--;
        }
      }
    }
  }
  
  float sumPositions = 0;
  int validPeaks = 0;
  
  for (int i = 0; i < length; i++) {
    if (isPeak[i]) {
      sumPositions += i;
      validPeaks++;
    }
  }
  
  result.peakCount = validPeaks;
  result.averagePeakPosition = (validPeaks > 0) ? (sumPositions / validPeaks) : 0;
  
  return result;
}

void updateDisplay() {
  if (!oled_ok) return;  // Exit if OLED isn't working
  
  display.clearDisplay();
  display.setCursor(0, 0);
  
  // Show sensor status if there are issues
  if (!max30100_ok || !pulseOx_ok) {
    display.println("SENSOR ERROR!");
    display.println("Check connections:");
    if (!max30100_ok) display.println("- MAX30100 failed");
    if (!pulseOx_ok) display.println("- PulseOx failed");
    display.display();
    return;
  }
  
  if (isCollecting) {
    unsigned long elapsed = millis() - startTime;
    unsigned long remaining = (phase == 1) ? 
      (COLLECTION_TIME_PER_PHASE - elapsed) : 
      (COLLECTION_TIME_PER_PHASE * 2 - elapsed);
    
    display.println("Collecting Data");
    display.print("Phase: ");
    display.println(phase == 1 ? "RAW" : "PROC");
    display.print("Time: ");
    display.print(elapsed / 1000);
    display.println("s");
    display.print("Remaining: ");
    display.print(remaining / 1000);
    display.println("s");
  } else {
    display.println("System Ready");
    
    // Show finger detection status
    if (fingerDetected) {
      unsigned long stableTime = millis() - fingerDetectedTime;
      if (stableTime < FINGER_STABLE_TIME) {
        display.println("Finger detected!");
        display.print("Stabilizing: ");
        display.print((FINGER_STABLE_TIME - stableTime) / 1000);
        display.println("s");
      } else {
        display.println("Starting soon...");
      }
    } else {
      display.println("Place finger on");
      display.println("sensor to start");
      display.println("(or press 's')");
    }
    
    if(gotQuartile && gotPeaks) {
        currentEstimate = (currentEstimateRF + currentEstimatePoly)/2;
        currentUricEstimate = (currentUricEstimateRF + currentUricEstimatePoly)/2;
        currentCholEstimate = (currentCholEstimateRF + currentCholEstimatePoly)/2;
        display.print("Glucose: ");
        display.print(currentEstimate);
        display.println(" mg/dL");
        display.print("Uric Acid: ");
        display.print(currentUricEstimate, 1);
        display.println(" mg/dL");
        display.print("Chol: ");
        display.print(currentCholEstimate, 1);
        display.println(" mg/dL");
    }
  }
  
  display.display();
}

void collectRawData(unsigned long elapsed) {
  if (!max30100_ok) return;  // Exit if sensor not working
  
  uint16_t ir_raw, red_raw;
  
  while (rawSensor.getRawValues(&ir_raw, &red_raw)) {
    if (elapsed >= COLLECTION_TIME_PER_PHASE) {
      if (bufferIndex > 0) {
        gotQuartile = true;
        
        gsrMin = calculateMin(gsrBuffer, bufferIndex);
        gsrMean = calculateMean(gsrBuffer, bufferIndex);
        
        irMin = calculateMin(irBuffer, bufferIndex);
        irMean = calculateMean(irBuffer, bufferIndex);
        
        redMin = calculateMin(redBuffer, bufferIndex);
        redMean = calculateMean(redBuffer, bufferIndex);
        
        PeakResult irPeaks = findPeaksStatic(irBuffer, bufferIndex, 5);
        irAvgPeak = irPeaks.averagePeakPosition;
        gotPeaks = true;
        currentEstimatePoly = evaluatePolynomialHorner(irAvgPeak);
        
        PeakToPeakResult irPeakToPeak = calculatePeakToPeakStatic(irBuffer, bufferIndex);
        PeakToPeakResult redPeakToPeak = calculatePeakToPeakStatic(redBuffer, bufferIndex);
        PeakToPeakResult gsrPeakToPeak = calculatePeakToPeakStatic(gsrBuffer, bufferIndex);

        int16_t features[1] = {(int16_t)redPeakToPeak.peakToPeakValue};
        currentEstimateRF = rf_gsr_predict(features, 1); 
        currentEstimatePoly = evaluatePolynomialHorner(redPeakToPeak.peakToPeakValue);
        
        int16_t features2[1] = {(int16_t)gsrMean};
        currentUricEstimateRF = rf_gsr2_predict(features2, 1);
        KurtosisRawIR = calculateKurtosis(irBuffer, bufferIndex);
        currentUricEstimatePoly = UriAcidevaluatePolynomialHorner(KurtosisRawIR);
        
        int16_t features3[1] = {(int16_t)gsrMin};
        currentCholEstimateRF = rf_gsr3_predict(features3, 1);
        KurtosisRawRED = calculateKurtosis(redBuffer, bufferIndex);
        currentCholEstimatePoly = CholevaluatePolynomialHorner(KurtosisRawRED);

        Serial.println("\n=== PEAK ANALYSIS RESULTS ===");
        Serial.print("IR Peaks Count: "); Serial.println(irPeaks.peakCount);
        Serial.print("IR Avg Peak Position: "); Serial.println(irAvgPeak);
        Serial.println("==============================\n");
      }
      switchToProcessedPhase();
      return;
    }
    
    float timeInTenths = elapsed / 100.0;
    
    int gsrRaw = analogRead(GSR);
    float gsrVoltage = gsrRaw * (5.0 / 1023.0);
    
    if (bufferIndex < BUFFER_SIZE) {
      if (ir_raw > 0 && red_raw > 0) {
        gsrBuffer[bufferIndex] = gsrVoltage;
        irBuffer[bufferIndex] = ir_raw;
        redBuffer[bufferIndex] = red_raw;
      }
      bufferIndex++;
    }
    
    Serial.print("RAW,");
    Serial.print(timeInTenths, 1);
    Serial.print(",");
    Serial.print(ir_raw);
    Serial.print(",");
    Serial.print(red_raw);
    Serial.print(",");
    Serial.print(gsrVoltage, 3);
    Serial.print(",");
    Serial.println(gsrRaw);
    
    digitalWrite(LED, !digitalRead(LED));
  }
}

void collectProcessedData(unsigned long elapsed) {
  static unsigned long lastProcessedPrint = 0;
  
  if (elapsed >= COLLECTION_TIME_PER_PHASE * 2) {
    stopCollection();
    return;
  }
  
  if (millis() - lastProcessedPrint >= 100) {
    float timeInTenths = elapsed / 100.0;
    
    float bpm = pulseOx_ok ? pox.getHeartRate() : 0;
    float spo2 = pulseOx_ok ? pox.getSpO2() : 0;
    
    int gsrRaw = analogRead(GSR);
    float gsrVoltage = gsrRaw * (5.0 / 1023.0);
    
    Serial.print("PROC,");
    Serial.print(timeInTenths, 1);
    Serial.print(",");
    
    if (bpm > 0 && bpm < 220) {
      Serial.print(bpm, 1);
    } else {
      Serial.print("0");
    }
    Serial.print(",");
    
    if (spo2 > 50 && spo2 <= 100) {
      Serial.print(spo2, 1);
    } else {
      Serial.print("0");
    }
    Serial.print(",");
    
    Serial.print(gsrVoltage, 3);
    Serial.print(",");
    Serial.println(gsrRaw);
    
    digitalWrite(LED, !digitalRead(LED));
    lastProcessedPrint = millis();
  }
}

void startCollection() {
  bufferIndex = 0;
  gotQuartile = false;
  gotPeaks = false;
  fingerDetected = false; // Reset finger detection
  
  memset(gsrBuffer, 0, sizeof(gsrBuffer));
  memset(irBuffer, 0, sizeof(irBuffer));
  memset(redBuffer, 0, sizeof(redBuffer));
  
  Serial.println("\n==========================================");
  Serial.println("=== 2-MINUTE DATA COLLECTION STARTED ===");
  Serial.println("PHASE 1 (0-600 tenths of seconds): RAW IR/Red data");
  Serial.println("Format: RAW,Time_0.1s,IR_raw,Red_raw,GSR_V,GSR_raw");
  Serial.println("------------------------------------------");
  
  startTime = millis();
  phase = 1; 
  isCollecting = true;
  digitalWrite(LED, HIGH);
}

void switchToProcessedPhase() {
  phase = 2;
  Serial.println("\n------------------------------------------");
  Serial.println("PHASE 2 (600-1200 tenths of seconds): Processed BPM/SpO2 data");
  Serial.println("Format: PROC,Time_0.1s,BPM,SpO2,GSR_V,GSR_raw");
  Serial.println("------------------------------------------");
  
  digitalWrite(LED, LOW);
  delay(500);
  digitalWrite(LED, HIGH);
}

void stopCollection() {
  isCollecting = false;
  phase = 0;
  digitalWrite(LED, LOW);
  
  Serial.println("\n==========================================");
  Serial.println("=== 2-MINUTE DATA COLLECTION COMPLETE ===");
  Serial.println("\nData Summary:");
  Serial.println("- Minute 1 (0-600 tenths sec): RAW IR/Red values");
  Serial.println("- Minute 2 (600-1200 tenths sec): Processed BPM/SpO2");
  Serial.println("- Full 2 minutes: GSR data (0.1s resolution)");
  Serial.println("\nFormat markers:");
  Serial.println("  RAW,Time_0.1s,IR,Red,GSR_V,GSR_raw");
  Serial.println("  PROC,Time_0.1s,BPM,SpO2,GSR_V,GSR_raw");
  Serial.println("\nCopy all data above and save to CSV file");
  Serial.println("Place finger on sensor again to start new collection");
  Serial.println("Or press 's' to force start");
  
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED, HIGH);
    delay(100);
    digitalWrite(LED, LOW);
    delay(100);
  }
}

float calculateQuartile25(float* data, int length) {
  if (length == 0) return 0;
  
  static float sorted[BUFFER_SIZE];
  memcpy(sorted, data, length * sizeof(float));
  qsort(sorted, length, sizeof(float), compareFloat);
  
  int position = (int)(length * 0.25);
  return sorted[position];
}