/*
 * BasicReading.ino - SpresenseIMU Library Basic Example
 * 
 * This example demonstrates basic usage of the SpresenseIMU library:
 * - Initialize the IMU sensor on Multi-IMU Add-on board
 * - Configure accelerometer and gyroscope ranges
 * - Read and display raw sensor data
 * 
 * Hardware Required:
 * - Sony Spresense main board
 * - Spresense Multi-IMU Add-on board
 * 
 * Circuit:
 * - Connect Multi-IMU Add-on board to Spresense main board
 */

#include <SpresenseIMU.h>

// Create IMU instance
SpresenseIMU imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect
  }
  
  Serial.println("SpresenseIMU Basic Reading Example");
  Serial.println("==================================");
  
  // Initialize the IMU with 100Hz sample rate
  Serial.print("Initializing IMU... ");
  if (!imu.begin(100)) {
    Serial.println("FAILED!");
    Serial.print("Error: ");
    Serial.println(imu.getErrorString(imu.getLastError()));
    while (1) {
      delay(1000);
    }
  }
  Serial.println("OK");
  
  // Configure sensor ranges
  Serial.print("Setting accelerometer range to ±4g... ");
  if (!imu.setAccelRange(4)) {
    Serial.println("FAILED!");
  } else {
    Serial.println("OK");
  }
  
  Serial.print("Setting gyroscope range to ±500°/s... ");
  if (!imu.setGyroRange(500)) {
    Serial.println("FAILED!");
  } else {
    Serial.println("OK");
  }
  
  // Enable moving average filter for smoother data
  Serial.print("Enabling moving average filter... ");
  imu.enableMovingAverage(true);
  imu.setMovingAverageWindow(5);
  Serial.println("OK");
  
  // Display device information
  SpresenseIMU::DeviceInfo info = imu.getDeviceInfo();
  Serial.print("Device Info - Chip ID: 0x");
  Serial.print(info.chip_id, HEX);
  Serial.print(", SW Rev: ");
  Serial.print(info.sw_revision);
  Serial.print(", HW Rev: ");
  Serial.println(info.hw_revision);
  
  Serial.println();
  Serial.println("Starting data acquisition...");
  Serial.println("Format: Timestamp(μs) | Accel(g) | Gyro(°/s)");
  Serial.println("------------------------------------------------");
  
  delay(1000);
}

void loop() {
  SpresenseIMU::IMUData data;
  
  // Read IMU data
  if (imu.read(data)) {
    // Display raw data
    Serial.print(data.timestamp);
    Serial.print(" | ");
    
    // Accelerometer data
    Serial.print("A: ");
    Serial.print(data.ax, 3);
    Serial.print(", ");
    Serial.print(data.ay, 3);
    Serial.print(", ");
    Serial.print(data.az, 3);
    Serial.print(" | ");
    
    // Gyroscope data
    Serial.print("G: ");
    Serial.print(data.gx, 2);
    Serial.print(", ");
    Serial.print(data.gy, 2);
    Serial.print(", ");
    Serial.print(data.gz, 2);
    Serial.println();
    
    // Display performance statistics every 50 samples
    static uint32_t sample_count = 0;
    sample_count++;
    
    if (sample_count % 50 == 0) {
      SpresenseIMU::PerformanceStats stats = imu.getStats();
      Serial.print("Stats - Total: ");
      Serial.print(stats.total_samples);
      Serial.print(", Dropped: ");
      Serial.print(stats.dropped_samples);
      Serial.print(", Rate: ");
      Serial.print(stats.actual_sample_rate, 1);
      Serial.println("Hz");
    }
  } else {
    Serial.print("Read failed: ");
    Serial.println(imu.getErrorString(imu.getLastError()));
  }
  
  delay(100); // 10Hz display rate
}