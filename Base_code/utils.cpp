#pragma once 

#include <Arduino.h>
#include <Wire.h>
#include "pin_def.h"
#include "utils.h"


// // ====== PROGRAM VARIABLES ======
int16_t gyroZ;                // Raw gyro Z-axis reading
// float gyroZOffset = 0;        // Calibration offset
// float gyroXOffset  = 0;
// float gyroYOffset  = 0;
float currentAngle = 0;       // Current angle in degrees
unsigned long lastTime = 0;   // Last read time
CRGB leds[NUM_LEDS];          // Current LED Color values
Servo scanServo;              // Servo

//=======================QuickSort=============================== 

// Function to swap two elements
void swap(int* a, int* b) {
    int t = *a;
    *a = *b;
    *b = t;
}

// Function to partition the array (Lomuto partition scheme, using last element as pivot)
int partition(int arr[], int low, int high) {
    int pivot = arr[high]; // Choose the last element as the pivot
    int i = (low - 1); // Index of the smaller element

    for (int j = low; j < high; j++) {
        // If current element is smaller than or equal to pivot
        if (arr[j] <= pivot) {
            i++; // Increment index of smaller element
            swap(&arr[i], &arr[j]);
        }
    }

    swap(&arr[i + 1], &arr[high]); // Swap the pivot element with the element at the correct pivot position
    return (i + 1); // Return the partition point
}

// The main function that implements QuickSort
void quickSort(int arr[], int low, int high) {
    if (low < high) {
        // pi is partitioning index, arr[pi] is now at right place
        int pi = partition(arr, low, high);

        // Recursively sort elements before partition and after partition
        quickSort(arr, low, pi - 1);
        quickSort(arr, pi + 1, high);
    }
}

//===============================================================================================================

// ====== LED FUNCTIONS ======

void ledOn(CRGB color) {
  leds[0] = color;
  FastLED.show();
}

void ledOff() {
  leds[0] = CRGB::Black;
  FastLED.show();
}


// ===== SERVO FUNCTIONS =====
;  // your calibrated center in logical degrees

void setServoAngle(int angle) {
  static int lastAngle = -1;
  angle = constrain(angle, 0, 200);

  if (angle != lastAngle) {
    scanServo.write(angle);
    delay(15);  // Allow servo to settle
    lastAngle = angle;
  }
}


// Center the servo
void centerServo() {
  setServoAngle(88);
}

// ====== GYRO FUNCTIONS ======

// Initialize Gyro Sensor
bool setupGyro() {
  Wire.begin();
  Wire.beginTransmission(GYRO);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // Wake up MPU6050
  byte error = Wire.endTransmission();
  
  if (error != 0) {
    return false;
  }
  
  // Configure gyro sensitivity (±250 deg/s)
  Wire.beginTransmission(GYRO);
  Wire.write(0x1B);  // GYRO_CONFIG register
  Wire.write(0x00);  // ±250 deg/s
  Wire.endTransmission();
  
  lastTime = millis();
  return true;
}

// Calibrate gyro (robot must be stationary!)
void calibrateGyro() {
  delay(500);
  
  long sum = 0;
  int samples = 500;
  
  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(GYRO);
    Wire.write(0x47);  // GYRO_ZOUT_H register
    Wire.endTransmission(false);
    Wire.requestFrom(GYRO, 2, true);
    
    int16_t gz = Wire.read() << 8 | Wire.read();
    sum += gz;
    delay(10);
  }
  
  gyroZOffset = sum / samples;
  currentAngle = 0;
}

// void calibrateGyro() {
//   delay(500);
//   long sumX = 0, sumY = 0, sumZ = 0;
//   int samples = 500;

//   for (int i = 0; i < samples; i++) {
//     Wire.beginTransmission(GYRO);
//     Wire.write(0x43);  // GYRO_XOUT_H — all 3 gyro axes from here
//     Wire.endTransmission(false);
//     Wire.requestFrom(GYRO, 6, true);

//     sumX += (int16_t)(Wire.read() << 8 | Wire.read());
//     sumY += (int16_t)(Wire.read() << 8 | Wire.read());
//     sumZ += (int16_t)(Wire.read() << 8 | Wire.read());
//     delay(2);
//   }

//   gyroXOffset = sumX / samples;
//   gyroYOffset = sumY / samples;
//   gyroZOffset = sumZ / samples;

//   Serial.print("[GYRO] Offsets X:"); Serial.print(gyroXOffset);
//   Serial.print(" Y:"); Serial.print(gyroYOffset);
//   Serial.print(" Z:"); Serial.println(gyroZOffset);
// }

// ===== ULTRASONIC SENSOR FUNCTIONS =====

int getDistance() {
  int validReading = 0;
  int attempts = 0;
  
  while (validReading == 0 && attempts < 3) {
    if (attempts > 0) delay(60);  // Only delay on retries
    digitalWrite(US_OUT, LOW);
    delayMicroseconds(2);
    digitalWrite(US_OUT, HIGH);
    delayMicroseconds(10);
    digitalWrite(US_OUT, LOW);
    long duration = pulseIn(US_IN, HIGH, 30000);
    int distance = duration * 0.034 / 2;

    if (duration > 0 && distance <= 200) {
      validReading = distance;
    }
    
    attempts++;
  }
  
  return validReading;
}



// Returns distance in centimeters, or 0 if invalid
//XXXXXXXXX Over Engineered for more statistically significant and error corrected value XXXXXXXXX
// int getDistance_accuracy() { 
//   int validReading = 0;
//   int attempts = 0;
  
//   while (validReading == 0 && attempts < 3) {
//     if (attempts > 0) delay(60);  // Only delay on retries
    
//     // digitalWrite(US_OUT, LOW);
//     // delayMicroseconds(2);
//     // digitalWrite(US_OUT, HIGH);
//     // delayMicroseconds(10);
//     // digitalWrite(US_OUT, LOW);
//     long duration = pulseIn(US_IN, HIGH, 30000);
//     int distance = duration * 0.034 / 2;
//     int sample_size = 10;

//     int distance_array[sample_size] = {0};
//     Serial.println("---------------Sampling start--------------------");
//     for (int i = 0; i < sample_size; i++){

//       digitalWrite(US_OUT, LOW);
//       delayMicroseconds(2);
//       digitalWrite(US_OUT, HIGH);
//       delayMicroseconds(10);
//       digitalWrite(US_OUT, LOW);
//       duration = pulseIn(US_IN, HIGH, 30000);
//       distance = duration * 0.034 / 2;
//       distance_array[i] = distance;
//       Serial.println(String(distance) + "---->" + String(i));
//       delay(45);
//     }
//     Serial.println("---------------Sampling end--------------------");

//     quickSort(distance_array, 0, sample_size - 1); 

//     distance = distance_array[(sample_size - 1)/2];
//     if (duration > 0 && distance <= 200) {
//       validReading = distance;
//     }
    
//     attempts++;
//   }
  
//   return validReading;
// }