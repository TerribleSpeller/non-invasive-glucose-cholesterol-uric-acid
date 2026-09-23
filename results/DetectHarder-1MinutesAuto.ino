#include <Wire.h>
#include "MAX30100.h"
#include "MAX30100_PulseOximeter.h"
#include "RandomForestModel.h"      //gsr_1
#include "UricAcid-RandomForest.h"  //gsr_2
#include "Chol-RandomForest.h"      //gsr_3
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <cstring>

//Important Defs
#define WIRE1_SCL 5
#define WIRE1_SDA 17
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1  //NOT USED.

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);

const int LED = 13;
const int GSR = 36;

MAX30100 rawSensor;
PulseOximeter pox;

bool oled_ok = false;
bool max30100_ok = false;
bool pulseOx_ok = false;

const unsigned long COLLECTION_TIME_PER_PHASE = 60000;  // 1 minute per phase
unsigned long startTime = 0;
bool isCollecting = false;
int phase = 0;  //0 = not started, 1 = raw phase, 2 = processed phase
unsigned long lastPrint = 0;

bool fingerDetected = false;
unsigned long fingerDetectedTime = 0;
const unsigned long FINGER_STABLE_TIME = 10000; 
const int FINGER_THRESHOLD = 5000;               
const int SAMPLES_FOR_DETECTION = 10;
float detectionSamples[10];
int sampleCount = 0;

// Cooldown to prevent immediate re-triggering
bool cooldownActive = false;
unsigned long cooldownStartTime = 0;
const unsigned long COOLDOWN_PERIOD = 10000;  // 10 seconds cooldown after collection ends

//hahha I'm a buffer!
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
float gsrMin, gsrMean, gsrMax;
float irMin, irMean;
float redMin, redMean;
bool gotQuartile = false;
bool gotPeaks = false;

// Glucose polynomial coefficients - ORIGINAL (using double for precision)
const double polyCoeffsDouble[9] = {
  123.00005783,     // c0 - intercept (x^0)
  2.89602108e-19,   // c1 - x^1
  -5.27951084e-23,  // c2 - x^2
  -2.68137510e-21,  // c3 - x^3
  -9.63931628e-19,  // c4 - x^4
  -2.67549521e-16,  // c5 - x^5
  -4.57884267e-14,  // c6 - x^6
  1.76492640e-16,   // c7 - x^7
  -2.27694393e-19   // c8 - x^8
};

// SCALED coefficients for when input is divided by 1000
// x_scaled = x/1000
const float polyCoeffsScaled1000[9] = {
  123.00005783f,     // c0
  2.89602108e-16f,   // c1 * 1000
  -5.27951084e-20f,  // c2 * 1000^2
  -2.68137510e-18f,  // c3 * 1000^3
  -9.63931628e-16f,  // c4 * 1000^4
  -2.67549521e-13f,  // c5 * 1000^5
  -4.57884267e-11f,  // c6 * 1000^6
  1.76492640e-13f,   // c7 * 1000^7
  -2.27694393e-16f   // c8 * 1000^8
};

// Uric Acid polynomial coefficients
const float UricpolyCoeffs[5] = {
  6.38647957f,   // c0 - intercept (x^0)
  -0.09240583f,  // c1 - x^1
  -0.08989593f,  // c2 - x^2
  0.01261649f,   // c3 - x^3
  -0.00022762f   // c4 - x^4
};

// Cholesterol polynomial coefficients
const float CholpolyCoeffs[5] = {
  188.01452453f,  // c0 - intercept (x^0)
  4.44063723f,    // c1 - x^1
  -6.82298462f,   // c2 - x^2
  0.82880367f,    // c3 - x^3
  -0.01462045f    // c4 - x^4
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

  Serial.print("Initializing OLED on Wire1 (SDA=16, SCL=17)...");
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    oled_ok = false;
    while (1) {
      digitalWrite(LED, HIGH);
      delay(100);
      digitalWrite(LED, LOW);
      delay(100);
    }
  } else {
    oled_ok = true;
    display.setRotation(2);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Diploy Pepper");
    display.display();
    Serial.println("SUCCESS");
  }

  Serial.print("Initializing MAX30100 for raw data on Wire (SDA=21, SCL=22)...");
  if (!rawSensor.begin()) {
    Serial.println("FAILED");
    Serial.println("Check wiring:");
    Serial.println("VCC -> 3.3V/5V");
    Serial.println("GND -> GND");
    Serial.println("SCL -> GPIO22");
    Serial.println("SDA -> GPIO21");
    max30100_ok = false;

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

    while (1) {
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

  Serial.print("Initializing PulseOximeter on Wire (SDA=21, SCL=22)...");
  if (!pox.begin()) {
    Serial.println("FAILED");
    pulseOx_ok = false;

    if (oled_ok) {
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("PulseOx ERROR!");
      display.println("Check MAX30100");
      display.display();
    }

    for (int i = 0; i < 5; i++) {
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

  // Initialize detection samples
  resetDetectionBuffer();
}

void loop() {
  if (max30100_ok) {
    rawSensor.update();
  }
  if (pulseOx_ok) {
    pox.update();
  }

  if (!isCollecting && max30100_ok) {
    checkForFinger();
  }

  updateDisplay();

  if (Serial.available() > 0) {
    char command = Serial.read();
    if (command == 's' || command == 'S') {
      if (max30100_ok && pulseOx_ok) {
        cooldownActive = false;  
        resetDetectionBuffer();
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
    } else if (phase == 2) {
      collectProcessedData(elapsed);
    }
  }
}

void checkForFinger() {
  if (cooldownActive) {
    if (millis() - cooldownStartTime >= COOLDOWN_PERIOD) {
      cooldownActive = false;
      resetDetectionBuffer();
      Serial.println("Cooldown complete. Ready for next finger detection.");
    }
    return;
  }

  uint16_t ir_raw, red_raw;

  // Get current readings
  if (rawSensor.getRawValues(&ir_raw, &red_raw)) {
    if (sampleCount < SAMPLES_FOR_DETECTION) {
      detectionSamples[sampleCount] = ir_raw;
      sampleCount++;

      static unsigned long lastProgressMsg = 0;
      if (millis() - lastProgressMsg > 1000) {
        Serial.print("Filling detection buffer: ");
        Serial.print(sampleCount);
        Serial.print("/");
        Serial.println(SAMPLES_FOR_DETECTION);
        lastProgressMsg = millis();
      }
    }

    // Only check if we have enough samples
    if (sampleCount >= SAMPLES_FOR_DETECTION) {
      float sum = 0;
      for (int i = 0; i < SAMPLES_FOR_DETECTION; i++) {
        sum += detectionSamples[i];
      }
      float average = sum / SAMPLES_FOR_DETECTION;

      if (average > FINGER_THRESHOLD) {
        if (!fingerDetected) {
          fingerDetectedTime = millis();
          fingerDetected = true;
          Serial.print("Finger detected! (avg=");
          Serial.print(average, 0);
          Serial.println(") Waiting for stable reading...");
        } else {
          if (millis() - fingerDetectedTime >= FINGER_STABLE_TIME) {
            Serial.println("Finger stable! Starting data collection...");
            startCollection();
            // Reset detection state after starting //Thanks Deepseek!
            fingerDetected = false;
            fingerDetectedTime = 0;
            sampleCount = 0;  // Reset sample count for next detection
          }
        }
      } else {
        // Finger removed or not detected
        if (fingerDetected) {
          fingerDetected = false;
          fingerDetectedTime = 0;
          Serial.print("Finger removed. (avg=");
          Serial.print(average, 0);
          Serial.println(") Waiting for finger...");
        }
        // Reset sample count to get fresh samples if finger is removed
        sampleCount = 0;  // Start collecting fresh samples
      }
    }
  }
}

float evaluateGlucosePolynomialScaled(float x) {
  if (isnan(x) || isinf(x) || x < 0 || x > 1000000) {
    Serial.println("Glucose: Invalid input");
    return 0.0f;
  }
  float scaledX = x / 1000.0f;

  Serial.print("Glucose - Input: ");
  Serial.print(x);
  Serial.print(", Scaled: ");
  Serial.println(scaledX, 6);

  float result = polyCoeffsScaled1000[8];
  for (int i = 7; i >= 0; i--) {
    result = result * scaledX + polyCoeffsScaled1000[i];
    if (isinf(result) || isnan(result)) {
      Serial.println("Glucose: Overflow detected");
      return 0.0f;
    }
  }

  if (result < 0.0f || result > 500.0f) {
    Serial.print("Glucose: Out of range (");
    Serial.print(result);
    Serial.println(")");
    return 0.0f;
  }

  Serial.print("Glucose Result: ");
  Serial.println(result, 3);
  return result;
}

float evaluateUricAcidPolynomial(float x) {
  if (isnan(x) || isinf(x) || x < 0 || x > 1000000) {
    return 0.0f;
  }

  float result = UricpolyCoeffs[4];
  for (int i = 3; i >= 0; i--) {
    result = result * x + UricpolyCoeffs[i];
    if (isinf(result) || isnan(result)) {
      return 0.0f;
    }
  }

  if (result < 0.0f || result > 15.0f) {
    return 0.0f;
  }
  return result;
}

float evaluateCholesterolPolynomial(float x) {
  if (isnan(x) || isinf(x) || x < 0 || x > 1000000) {
    return 0.0f;
  }

  float result = CholpolyCoeffs[4];
  for (int i = 3; i >= 0; i--) {
    result = result * x + CholpolyCoeffs[i];
    if (isinf(result) || isnan(result)) {
      return 0.0f;
    }
  }

  if (result < 0.0f || result > 400.0f) {
    return 0.0f;
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

float calculateMax(float* data, int length) {
  if (length <= 0) return 0;

  float maxVal = data[0];
  for (int i = 1; i < length; i++) {
    if (data[i] > maxVal) {
      maxVal = data[i];
    }
  }
  return maxVal;
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
  PeakToPeakResult result = { 0.0f, 0.0f, 0.0f, 0 };

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
  PeakResult result = { 0, 0.0f };

  if (length < 3) return result;

  bool* isPeak = new bool[length]();
  int peakCount = 0;

  for (int i = 1; i < length - 1; i++) {
    if (data[i] > data[i - 1] && data[i] > data[i + 1]) {
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
  PeakResult result = { 0, 0.0f };

  if (length < 3 || length > BUFFER_SIZE) return result;

  static bool isPeak[BUFFER_SIZE];

  int peakCount = 0;
  for (int i = 1; i < length - 1; i++) {
    if (data[i] > data[i - 1] && data[i] > data[i + 1]) {
      isPeak[i] = true;
      peakCount++;
    } else {
      isPeak[i] = false;
    }
  }
  isPeak[0] = false;
  isPeak[length - 1] = false;

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
  if (!oled_ok) return;

  display.clearDisplay();
  display.setCursor(0, 0);

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
    unsigned long remaining = (phase == 1) ? (COLLECTION_TIME_PER_PHASE - elapsed) : (COLLECTION_TIME_PER_PHASE * 2 - elapsed);

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

    if (cooldownActive) {
      unsigned long cooldownRemaining = (COOLDOWN_PERIOD - (millis() - cooldownStartTime)) / 1000;
      display.println("Please remove");
      display.println("your finger");
      display.print("Cooldown: ");
      display.print(cooldownRemaining);
      display.println("s");
    } else if (fingerDetected) {
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

    if (gotQuartile && gotPeaks) {
      if (currentEstimatePoly == 0.0f || isnan(currentEstimatePoly) || isinf(currentEstimatePoly)) {
        currentEstimate = currentEstimateRF;
      } else {
        currentEstimate = (currentEstimateRF + currentEstimatePoly) / 2;
      }
      
      if (currentUricEstimatePoly == 0.0f || isnan(currentUricEstimatePoly) || isinf(currentUricEstimatePoly)) {
        currentUricEstimate = currentUricEstimateRF;
      } else {
        currentUricEstimate = (currentUricEstimateRF + currentUricEstimatePoly) / 2;
      }
      
      if (currentCholEstimatePoly == 0.0f || isnan(currentCholEstimatePoly) || isinf(currentCholEstimatePoly)) {
        currentCholEstimate = currentCholEstimateRF;
      } else {
        currentCholEstimate = (currentCholEstimateRF + currentCholEstimatePoly) / 2;
      }

      if (isnan(currentEstimate) || isinf(currentEstimate) || currentEstimate < 0 || currentEstimate > 500) {
        currentEstimate = 0;
      }

      if (isnan(currentUricEstimate) || isinf(currentUricEstimate) || currentUricEstimate < 0 || currentUricEstimate > 15) {
        currentUricEstimate = 0;
      }

   
      if (isnan(currentCholEstimate) || isinf(currentCholEstimate) || currentCholEstimate < 0 || currentCholEstimate > 400) {
        currentCholEstimate = 0;
      }
      char buffer[30];

      if (currentEstimate > 0 && currentEstimate < 500) {
        dtostrf(currentEstimate, 5, 1, buffer);
        display.print("Glu: ");
        display.print(buffer);
      } else {
        display.print("Glu: --.-");
      }
      display.println(" mg/dL");

      if (currentUricEstimate > 0 && currentUricEstimate < 15) {
        dtostrf(currentUricEstimate, 4, 1, buffer);
        display.print("Ur.A: ");
        display.print(buffer);
      } else {
        display.print("Ur.A: --.-");
      }
      display.println(" mg/dL");

      if (currentCholEstimate > 0 && currentCholEstimate < 400) {
        dtostrf(currentCholEstimate, 5, 1, buffer);
        display.print("Chol: ");
        display.print(buffer);
      } else {
        display.print("Chol: ---.-");
      }
      display.println(" mg/dL");
    }
  }

  display.display();
}

void collectRawData(unsigned long elapsed) {
  if (!max30100_ok) return; 

  uint16_t ir_raw, red_raw;

  while (rawSensor.getRawValues(&ir_raw, &red_raw)) {
    if (elapsed >= COLLECTION_TIME_PER_PHASE) {
      if (bufferIndex > 0) {
        gotQuartile = true;

        gsrMin = calculateMin(gsrBuffer, bufferIndex);
        gsrMean = calculateMean(gsrBuffer, bufferIndex);
        gsrMax = calculateMax(gsrBuffer, bufferIndex);

        irMin = calculateMin(irBuffer, bufferIndex);
        irMean = calculateMean(irBuffer, bufferIndex);

        redMin = calculateMin(redBuffer, bufferIndex);
        redMean = calculateMean(redBuffer, bufferIndex);

        PeakResult irPeaks = findPeaksStatic(irBuffer, bufferIndex, 5);
        irAvgPeak = irPeaks.averagePeakPosition;
        gotPeaks = true;

        PeakToPeakResult irPeakToPeak = calculatePeakToPeakStatic(irBuffer, bufferIndex);
        PeakToPeakResult redPeakToPeak = calculatePeakToPeakStatic(redBuffer, bufferIndex);
        PeakToPeakResult gsrPeakToPeak = calculatePeakToPeakStatic(gsrBuffer, bufferIndex);

        int16_t features[1] = { (int16_t)redPeakToPeak.peakToPeakValue };
        currentEstimateRF = rf_gsr_predict(features, 1);
        currentEstimatePoly = evaluateGlucosePolynomialScaled(redPeakToPeak.peakToPeakValue);

        int16_t features2[1] = { (int16_t)gsrMean };
        currentUricEstimateRF = rf_gsr2_predict(features2, 1);
        KurtosisRawIR = calculateKurtosis(irBuffer, bufferIndex);
        currentUricEstimatePoly = evaluateUricAcidPolynomial(KurtosisRawIR);

        int16_t features3[1] = { (int16_t)gsrMax };
        currentCholEstimateRF = rf_gsr3_predict(features3, 1);
        KurtosisRawRED = calculateKurtosis(redBuffer, bufferIndex);
        currentCholEstimatePoly = evaluateCholesterolPolynomial(KurtosisRawRED);

        Serial.println("\n=== RESULTS ===");
        Serial.print("Glucose RF: ");
        Serial.println(currentEstimateRF);
        Serial.print("Glucose Poly: ");
        Serial.println(currentEstimatePoly);
        Serial.print("Uric Acid RF: ");
        Serial.println(currentUricEstimateRF);
        Serial.print("Uric Acid Poly: ");
        Serial.println(currentUricEstimatePoly);
        Serial.print("Cholesterol RF: ");
        Serial.println(currentCholEstimateRF);
        Serial.print("Cholesterol Poly: ");
        Serial.println(currentCholEstimatePoly);
        Serial.println("==============\n");
      }
      //PROC UNEEDED
      stopCollection();
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

  resetDetectionBuffer();

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

void resetDetectionBuffer() {
  for (int i = 0; i < SAMPLES_FOR_DETECTION; i++) {
    detectionSamples[i] = 0;
  }
  sampleCount = 0;
  fingerDetected = false;
  fingerDetectedTime = 0;
  Serial.println("Detection buffer reset");
}

void stopCollection() {
  isCollecting = false;
  phase = 0;
  digitalWrite(LED, LOW);

  // Start cooldown to prevent immediate re-triggering
  //Thanksagain Deepsseeek!
  cooldownActive = true;
  cooldownStartTime = millis();

  // Reset detection buffer
  resetDetectionBuffer();

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
  Serial.println("Please remove your finger from the sensor");
  Serial.print("Cooldown period: ");
  Serial.print(COOLDOWN_PERIOD / 1000);
  Serial.println(" seconds");
  Serial.println("After cooldown, place finger on sensor again to start new collection");
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