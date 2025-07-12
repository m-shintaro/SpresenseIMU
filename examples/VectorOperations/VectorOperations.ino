/*
 * VectorOperations.ino - SpresenseIMU Library Vector Math Example
 * 
 * This example demonstrates vector operations available in the SpresenseIMU library:
 * - Using getVector() method for different data types
 * - Vector mathematics (dot product, cross product, normalization)
 * - Converting between different units
 * - Calculating derived values like linear acceleration
 */

#include <SpresenseIMU.h>

// Create IMU instance
SpresenseIMU imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect
  }
  
  Serial.println("SpresenseIMU Vector Operations Example");
  Serial.println("=====================================");
  
  // Initialize the IMU
  Serial.print("Initializing IMU... ");
  if (!imu.begin(50)) { // 50Hz for this example
    Serial.println("FAILED!");
    Serial.print("Error: ");
    Serial.println(imu.getErrorString(imu.getLastError()));
    while (1) {
      delay(1000);
    }
  }
  Serial.println("OK");
  
  // Configure ranges
  imu.setAccelRange(2);   // ±2g for better precision
  imu.setGyroRange(250);  // ±250°/s
  
  Serial.println();
  Serial.println("Demonstrating vector operations...");
  Serial.println();
  
  delay(1000);
}

void loop() {
  // Get different types of vector data
  Vector<3> accel = imu.getVector(SpresenseIMU::VECTOR_ACCELEROMETER);
  Vector<3> gyro_deg = imu.getVector(SpresenseIMU::VECTOR_GYROSCOPE);
  Vector<3> gyro_rad = imu.getVector(SpresenseIMU::VECTOR_GYROSCOPE_RAD);
  Vector<3> linear_accel = imu.getVector(SpresenseIMU::VECTOR_LINEAR_ACCEL);
  Vector<3> gravity = imu.getVector(SpresenseIMU::VECTOR_GRAVITY);
  
  Serial.println("=== Raw Sensor Data ===");
  Serial.print("Acceleration [g]: ");
  printVector(accel);
  
  Serial.print("Gyroscope [°/s]: ");
  printVector(gyro_deg);
  
  Serial.print("Gyroscope [rad/s]: ");
  printVector(gyro_rad);
  
  Serial.println();
  Serial.println("=== Derived Data ===");
  Serial.print("Linear Accel [g]: ");
  printVector(linear_accel);
  
  Serial.print("Gravity [g]: ");
  printVector(gravity);
  
  Serial.println();
  Serial.println("=== Vector Mathematics ===");
  
  // Vector magnitudes
  Serial.print("Acceleration magnitude: ");
  Serial.print(accel.magnitude(), 4);
  Serial.println(" g");
  
  Serial.print("Angular velocity magnitude: ");
  Serial.print(gyro_deg.magnitude(), 2);
  Serial.println(" °/s");
  
  // Normalize acceleration vector to get orientation
  Vector<3> accel_normalized = accel;
  accel_normalized.normalize();
  Serial.print("Normalized accel: ");
  printVector(accel_normalized);
  
  // Calculate angle from vertical (tilt)
  Vector<3> vertical(0, 0, 1);
  double dot_product = accel_normalized.dot(vertical);
  double tilt_angle = acos(abs(dot_product)) * 180.0 / M_PI;
  Serial.print("Tilt from vertical: ");
  Serial.print(tilt_angle, 1);
  Serial.println("°");
  
  // Demonstrate cross product (perpendicular vector)
  Vector<3> cross_result = accel.cross(gyro_deg);
  Serial.print("Accel × Gyro: ");
  printVector(cross_result);
  
  // Vector arithmetic
  Vector<3> total_motion = linear_accel + gyro_deg * 0.01; // Scale gyro for addition
  Serial.print("Combined motion: ");
  printVector(total_motion);
  
  Serial.println();
  Serial.println("=== Unit Conversions ===");
  
  // Convert acceleration to m/s²
  Vector<3> accel_ms2 = accel * 9.81; // g to m/s²
  Serial.print("Acceleration [m/s²]: ");
  printVector(accel_ms2);
  
  // Convert gyro from deg/s to rpm
  Vector<3> gyro_rpm = gyro_deg * (60.0 / 360.0);
  Serial.print("Angular velocity [rpm]: ");
  printVector(gyro_rpm);
  
  // Demonstrate scaling
  Vector<3> scaled_accel = accel.scale(2.0);
  Serial.print("2× Acceleration: ");
  printVector(scaled_accel);
  
  // Demonstrate inversion
  Vector<3> inverted_gyro = gyro_deg.invert();
  Serial.print("Inverted gyro: ");
  printVector(inverted_gyro);
  
  Serial.println();
  Serial.println("=== Advanced Calculations ===");
  
  // Calculate approximate orientation (simplified)
  double roll = atan2(accel.y(), accel.z()) * 180.0 / M_PI;
  double pitch = atan2(-accel.x(), sqrt(accel.y()*accel.y() + accel.z()*accel.z())) * 180.0 / M_PI;
  
  Serial.print("Estimated Roll: ");
  Serial.print(roll, 1);
  Serial.println("°");
  
  Serial.print("Estimated Pitch: ");
  Serial.print(pitch, 1);
  Serial.println("°");
  
  // Check for motion detection
  if (linear_accel.magnitude() > 0.1) {
    Serial.println("Motion detected!");
  }
  
  if (gyro_deg.magnitude() > 10.0) {
    Serial.println("Rotation detected!");
  }
  
  Serial.println("=====================================");
  Serial.println();
  
  delay(2000); // Display every 2 seconds
}

/**
 * Helper function to print vector components
 */
void printVector(const Vector<3>& vec) {
  Serial.print("(");
  Serial.print(vec.x(), 4);
  Serial.print(", ");
  Serial.print(vec.y(), 4);
  Serial.print(", ");
  Serial.print(vec.z(), 4);
  Serial.println(")");
}