#ifndef SPRESENSE_IMU_H
#define SPRESENSE_IMU_H

#include <Arduino.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <nuttx/sensors/cxd5602pwbimu.h>
#include <arch/board/cxd56_cxd5602pwbimu.h>

#include "utility/vector.h"
#include "utility/quaternion.h"
#include "utility/matrix.h"

using namespace spresense_imu;

/**
 * @brief Main class for Sony Spresense CXD5602PWB 6-axis IMU sensor
 * 
 * This class provides comprehensive access to the built-in IMU sensor including:
 * - 3-axis accelerometer data
 * - 3-axis gyroscope data  
 * - Configuration and calibration
 * - Vector mathematics utilities
 * - Basic orientation estimation
 */
class SpresenseIMU {
  public:
    /**
     * @brief Operation modes for the IMU sensor
     */
    enum OperationMode {
      MODE_ACCELEROMETER_ONLY,  ///< Accelerometer only
      MODE_GYROSCOPE_ONLY,      ///< Gyroscope only  
      MODE_IMU                  ///< 6-axis IMU (default)
    };

    /**
     * @brief Vector data types available from sensor
     */
    enum VectorType {
      VECTOR_ACCELEROMETER,     ///< Acceleration vector [g]
      VECTOR_GYROSCOPE,         ///< Angular velocity vector [deg/s]
      VECTOR_GYROSCOPE_RAD,     ///< Angular velocity vector [rad/s]
      VECTOR_LINEAR_ACCEL,      ///< Linear acceleration (gravity removed) [g]
      VECTOR_GRAVITY            ///< Gravity vector [g]
    };

    /**
     * @brief Error codes
     */
    enum ErrorCode {
      ERROR_NONE = 0,
      ERROR_INIT_FAILED,
      ERROR_DEVICE_OPEN_FAILED,
      ERROR_INVALID_RANGE,
      ERROR_INVALID_SAMPLE_RATE,
      ERROR_READ_FAILED,
      ERROR_NOT_INITIALIZED
    };

    /**
     * @brief IMU data structure containing all sensor readings
     */
    struct IMUData {
      uint64_t timestamp;    ///< Timestamp in microseconds
      float ax, ay, az;      ///< Acceleration [g]
      float gx, gy, gz;      ///< Angular velocity [deg/s]
    };

    /**
     * @brief Calibration status structure
     */
    struct CalibrationStatus {
      uint8_t system;        ///< System calibration status (0-3)
      uint8_t accel;         ///< Accelerometer calibration status (0-3)
      uint8_t gyro;          ///< Gyroscope calibration status (0-3)
    };

    /**
     * @brief Calibration offsets structure
     */
    struct CalibrationOffsets {
      int16_t accel_offset_x, accel_offset_y, accel_offset_z;
      int16_t gyro_offset_x, gyro_offset_y, gyro_offset_z;
      float accel_scale_x, accel_scale_y, accel_scale_z;
      float gyro_scale_x, gyro_scale_y, gyro_scale_z;
    };

    /**
     * @brief Device information structure
     */
    struct DeviceInfo {
      uint8_t chip_id;       ///< Chip ID
      uint8_t sw_revision;   ///< Software revision
      uint8_t hw_revision;   ///< Hardware revision
    };

    /**
     * @brief Performance statistics structure
     */
    struct PerformanceStats {
      uint32_t total_samples;    ///< Total samples read
      uint32_t dropped_samples;  ///< Dropped samples count
      float actual_sample_rate;  ///< Actual sample rate achieved
      uint32_t buffer_overruns;  ///< Buffer overrun count
    };

    // Constructor and initialization
    /**
     * @brief Constructor
     * @param sensorID Sensor identifier (for multiple sensors, not used currently)
     */
    SpresenseIMU(int32_t sensorID = -1);

    /**
     * @brief Initialize the IMU sensor
     * @param odr Output data rate in Hz (1-200, default: 200)
     * @return true if initialization successful, false otherwise
     */
    bool begin(uint16_t odr = 200);

    /**
     * @brief Check if IMU is initialized and ready
     * @return true if ready, false otherwise
     */
    bool isReady() const;

    // Mode and configuration
    /**
     * @brief Set operation mode
     * @param mode Operation mode to set
     */
    void setMode(OperationMode mode);

    /**
     * @brief Get current operation mode
     * @return Current operation mode
     */
    OperationMode getMode() const;

    /**
     * @brief Set accelerometer range
     * @param range Range in g (2, 4, 8, 16)
     * @return true if successful, false if invalid range
     */
    bool setAccelRange(uint8_t range);

    /**
     * @brief Set gyroscope range  
     * @param range Range in deg/s (125, 250, 500, 1000, 2000)
     * @return true if successful, false if invalid range
     */
    bool setGyroRange(uint16_t range);

    /**
     * @brief Set sample rate
     * @param rate Sample rate in Hz (1-200)
     * @return true if successful, false if invalid rate
     */
    bool setSampleRate(uint16_t rate);

    /**
     * @brief Get current sample rate
     * @return Current sample rate in Hz
     */
    uint16_t getSampleRate() const;

    // Data reading
    /**
     * @brief Read latest IMU data
     * @param data Reference to structure to store data
     * @return true if read successful, false otherwise
     */
    bool read(IMUData& data);

    /**
     * @brief Read raw IMU data (without calibration applied)
     * @param data Reference to structure to store data
     * @return true if read successful, false otherwise
     */
    bool readRaw(IMUData& data);

    /**
     * @brief Get vector data of specified type
     * @param type Type of vector data to retrieve
     * @return 3D vector containing requested data
     */
    Vector<3> getVector(VectorType type);

    /**
     * @brief Get current orientation as quaternion
     * @return Quaternion representing current orientation
     */
    Quaternion getQuaternion();

    /**
     * @brief Get Euler angles (roll, pitch, yaw)
     * @return Vector containing Euler angles in degrees [roll, pitch, yaw]
     */
    Vector<3> getEulerAngles();

    /**
     * @brief Get rotation matrix representing current orientation
     * @return 3x3 rotation matrix
     */
    Matrix<3> getRotationMatrix();

    /**
     * @brief Get sensor temperature
     * @return Temperature in Celsius, or NaN if read failed
     */
    float getTemperature();

    // Calibration
    /**
     * @brief Get calibration status
     * @return Calibration status structure
     */
    CalibrationStatus getCalibration();

    /**
     * @brief Check if fully calibrated
     * @return true if all sensors are fully calibrated
     */
    bool isFullyCalibrated();

    /**
     * @brief Get calibration offsets
     * @param offsets Reference to structure to store offsets
     * @return true if successful
     */
    bool getOffsets(CalibrationOffsets& offsets);

    /**
     * @brief Set calibration offsets
     * @param offsets Offsets structure to apply
     * @return true if successful
     */
    bool setOffsets(const CalibrationOffsets& offsets);

    /**
     * @brief Start automatic calibration
     * @param duration_ms Duration in milliseconds (default: 10000)
     * @return true if calibration started successfully
     */
    bool startAutoCalibration(uint32_t duration_ms = 10000);

    /**
     * @brief Start gyroscope bias calibration (sensor must be stationary)
     * @param samples Number of samples to collect (default: 1000)
     * @return true if calibration started successfully
     */
    bool startGyroBiasCalibration(uint16_t samples = 1000);

    /**
     * @brief Start accelerometer calibration (6-point calibration)
     * Call this method at each of 6 orientations (+X, -X, +Y, -Y, +Z, -Z)
     * @param orientation Orientation index (0-5)
     * @param samples Number of samples to collect per orientation (default: 500)
     * @return true if calibration step completed successfully
     */
    bool startAccelCalibration(uint8_t orientation, uint16_t samples = 500);

    /**
     * @brief Check if calibration is in progress
     * @return true if calibrating
     */
    bool isCalibrating();

    /**
     * @brief Get calibration progress
     * @return Progress from 0.0 to 1.0
     */
    float getCalibrationProgress();

    /**
     * @brief Calculate and apply calibration from collected data
     * @return true if calibration was successfully calculated and applied
     */
    bool finishCalibration();

    // System information and diagnostics
    /**
     * @brief Get device information
     * @return Device information structure
     */
    DeviceInfo getDeviceInfo();

    /**
     * @brief Perform self-test
     * @return true if self-test passed
     */
    bool selfTest();

    /**
     * @brief Get last error code
     * @return Error code
     */
    ErrorCode getLastError() const;

    /**
     * @brief Get error description string
     * @param error Error code
     * @return Error description
     */
    String getErrorString(ErrorCode error);

    /**
     * @brief Get performance statistics
     * @return Performance statistics structure
     */
    PerformanceStats getStats();

    /**
     * @brief Reset performance statistics
     */
    void resetStats();

    // Basic filtering (Phase 1 implementation)
    /**
     * @brief Enable/disable simple moving average filter
     * @param enable true to enable, false to disable
     */
    void enableMovingAverage(bool enable);

    /**
     * @brief Set moving average window size
     * @param samples Number of samples to average (1-10)
     */
    void setMovingAverageWindow(uint8_t samples);

  private:
    // Private member variables
    int32_t _sensorID;
    int _imu_fd;
    bool _initialized;
    OperationMode _mode;
    uint16_t _sample_rate;
    uint8_t _accel_range;
    uint16_t _gyro_range;
    ErrorCode _last_error;

    // Calibration variables
    bool _calibrating;
    uint32_t _calibration_start_time;
    uint32_t _calibration_duration;
    CalibrationOffsets _offsets;
    bool _offsets_applied;
    
    // Enhanced calibration data
    enum CalibrationMode {
      CALIB_NONE,
      CALIB_GYRO_BIAS,
      CALIB_ACCEL_6POINT,
      CALIB_AUTO
    };
    CalibrationMode _calibration_mode;
    uint16_t _calibration_samples_target;
    uint16_t _calibration_samples_collected;
    uint8_t _calibration_orientation;
    Vector<3> _gyro_bias_sum;
    Vector<3> _accel_samples[6];  // 6-point calibration data
    bool _accel_orientation_done[6];

    // Orientation estimation
    Quaternion _current_orientation;
    bool _orientation_initialized;
    Vector<3> _gravity_estimate;

    // Performance tracking
    PerformanceStats _stats;
    uint32_t _last_sample_time;

    // Moving average filter
    bool _moving_avg_enabled;
    uint8_t _moving_avg_window;
    uint8_t _filter_index;
    Vector<3> _accel_history[10];
    Vector<3> _gyro_history[10];

    // Private methods
    bool _initializeHardware();
    bool _configureRanges();
    bool _enableSensor();
    void _setError(ErrorCode error);
    void _updateStats();
    void _applyMovingAverage(IMUData& data);
    Vector<3> _calculateGravity(const Vector<3>& accel);
    Vector<3> _removeGravity(const Vector<3>& accel, const Vector<3>& gravity);
    
    // Enhanced calibration methods
    void _processCalibrationSample(const IMUData& data);
    bool _calculateGyroBias();
    bool _calculateAccelCalibration();
    void _updateOrientation(const Vector<3>& accel, const Vector<3>& gyro, float dt);
    void _initializeOrientation(const Vector<3>& accel);
};

#endif // SPRESENSE_IMU_H