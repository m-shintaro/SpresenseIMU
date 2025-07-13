#include "SpresenseIMU.h"

/**
 * @brief Constructor
 */
SpresenseIMU::SpresenseIMU(int32_t sensorID) 
  : _sensorID(sensorID)
  , _imu_fd(-1)
  , _initialized(false)
  , _mode(MODE_IMU)
  , _sample_rate(200)
  , _accel_range(2)
  , _gyro_range(125)
  , _last_error(ERROR_NONE)
  , _calibrating(false)
  , _calibration_start_time(0)
  , _calibration_duration(0)
  , _offsets_applied(false)
  , _calibration_mode(CALIB_NONE)
  , _calibration_samples_target(0)
  , _calibration_samples_collected(0)
  , _calibration_orientation(0)
  , _orientation_initialized(false)
  , _moving_avg_enabled(false)
  , _moving_avg_window(5)
  , _filter_index(0)
{
  // Initialize statistics
  memset(&_stats, 0, sizeof(_stats));
  _last_sample_time = 0;

  // Initialize offsets to identity
  memset(&_offsets, 0, sizeof(_offsets));
  _offsets.accel_scale_x = 1.0f;
  _offsets.accel_scale_y = 1.0f;
  _offsets.accel_scale_z = 1.0f;
  _offsets.gyro_scale_x = 1.0f;
  _offsets.gyro_scale_y = 1.0f;
  _offsets.gyro_scale_z = 1.0f;

  // Initialize filter history
  for (uint8_t i = 0; i < 10; i++) {
    _accel_history[i] = Vector<3>(0, 0, 0);
    _gyro_history[i] = Vector<3>(0, 0, 0);
  }

  // Initialize calibration arrays
  _gyro_bias_sum = Vector<3>(0, 0, 0);
  for (uint8_t i = 0; i < 6; i++) {
    _accel_samples[i] = Vector<3>(0, 0, 0);
    _accel_orientation_done[i] = false;
  }
  
  // Initialize orientation
  _current_orientation = Quaternion(); // Identity quaternion
  _gravity_estimate = Vector<3>(0, 0, 1); // Assume Z-up initially
}

/**
 * @brief Initialize the IMU sensor
 */
bool SpresenseIMU::begin(uint16_t odr) {
  _sample_rate = odr;
  
  if (!_initializeHardware()) {
    _setError(ERROR_INIT_FAILED);
    return false;
  }

  // Follow the exact sequence from working basic.ino
  // 1. Set sample rate first
  if (!setSampleRate(odr)) {
    _setError(ERROR_INVALID_SAMPLE_RATE);
    return false;
  }

  // 2. Set ranges
  if (!_configureRanges()) {
    _setError(ERROR_INVALID_RANGE);
    return false;
  }

  // 3. Enable sensor (without duplicate sample rate setting)
  if (!_enableSensor()) {
    _setError(ERROR_INIT_FAILED);
    return false;
  }

  _initialized = true;
  _setError(ERROR_NONE);
  resetStats();
  
  return true;
}

/**
 * @brief Check if IMU is ready
 */
bool SpresenseIMU::isReady() const {
  return _initialized && (_imu_fd >= 0);
}

/**
 * @brief Set operation mode
 */
void SpresenseIMU::setMode(OperationMode mode) {
  _mode = mode;
  // Note: For Phase 1, we always use 6-axis mode
  // Future phases can implement mode switching
}

/**
 * @brief Get current operation mode
 */
SpresenseIMU::OperationMode SpresenseIMU::getMode() const {
  return _mode;
}

/**
 * @brief Set accelerometer range
 */
bool SpresenseIMU::setAccelRange(uint8_t range) {
  if (range != 2 && range != 4 && range != 8 && range != 16) {
    _setError(ERROR_INVALID_RANGE);
    return false;
  }
  
  _accel_range = range;
  
  if (_initialized) {
    return _configureRanges();
  }
  
  return true;
}

/**
 * @brief Set gyroscope range
 */
bool SpresenseIMU::setGyroRange(uint16_t range) {
  if (range != 125 && range != 250 && range != 500 && range != 1000 && range != 2000) {
    _setError(ERROR_INVALID_RANGE);
    return false;
  }
  
  _gyro_range = range;
  
  if (_initialized) {
    return _configureRanges();
  }
  
  return true;
}

/**
 * @brief Set sample rate
 */
bool SpresenseIMU::setSampleRate(uint16_t rate) {
  if (rate < 1 || rate > 200) {
    _setError(ERROR_INVALID_SAMPLE_RATE);
    return false;
  }
  
  _sample_rate = rate;
  
  if (_initialized && _imu_fd >= 0) {
    if (ioctl(_imu_fd, SNIOC_SSAMPRATE, rate) < 0) {
      _setError(ERROR_INVALID_SAMPLE_RATE);
      return false;
    }
  }
  
  return true;
}

/**
 * @brief Get current sample rate
 */
uint16_t SpresenseIMU::getSampleRate() const {
  return _sample_rate;
}

/**
 * @brief Read latest IMU data
 */
bool SpresenseIMU::read(IMUData& data) {
  if (!isReady()) {
    _setError(ERROR_NOT_INITIALIZED);
    return false;
  }

  cxd5602pwbimu_data_t raw_data;
  ssize_t bytes_read = ::read(_imu_fd, &raw_data, sizeof(raw_data));
  
  if (bytes_read != sizeof(raw_data)) {
    _setError(ERROR_READ_FAILED);
    _stats.dropped_samples++;
    return false;
  }

  // Convert raw data to our format
  data.timestamp = raw_data.timestamp;
  data.ax = raw_data.ax;
  data.ay = raw_data.ay;
  data.az = raw_data.az;
  data.gx = raw_data.gx;
  data.gy = raw_data.gy;
  data.gz = raw_data.gz;

  // Apply calibration offsets if available
  if (_offsets_applied) {
    data.ax = (data.ax - _offsets.accel_offset_x / 1000.0f) * _offsets.accel_scale_x;
    data.ay = (data.ay - _offsets.accel_offset_y / 1000.0f) * _offsets.accel_scale_y;
    data.az = (data.az - _offsets.accel_offset_z / 1000.0f) * _offsets.accel_scale_z;
    
    data.gx = (data.gx - _offsets.gyro_offset_x / 100.0f) * _offsets.gyro_scale_x;
    data.gy = (data.gy - _offsets.gyro_offset_y / 100.0f) * _offsets.gyro_scale_y;
    data.gz = (data.gz - _offsets.gyro_offset_z / 100.0f) * _offsets.gyro_scale_z;
  }

  // Apply moving average filter if enabled
  if (_moving_avg_enabled) {
    _applyMovingAverage(data);
  }

  // Process calibration samples if calibrating
  if (_calibrating) {
    _processCalibrationSample(data);
  }

  _updateStats();
  _setError(ERROR_NONE);
  
  return true;
}

/**
 * @brief Read raw IMU data (without calibration)
 */
bool SpresenseIMU::readRaw(IMUData& data) {
  if (!isReady()) {
    _setError(ERROR_NOT_INITIALIZED);
    return false;
  }

  cxd5602pwbimu_data_t raw_data;
  ssize_t bytes_read = ::read(_imu_fd, &raw_data, sizeof(raw_data));
  
  if (bytes_read != sizeof(raw_data)) {
    _setError(ERROR_READ_FAILED);
    _stats.dropped_samples++;
    return false;
  }

  // Convert raw data without any calibration or filtering
  data.timestamp = raw_data.timestamp;
  data.ax = raw_data.ax;
  data.ay = raw_data.ay;
  data.az = raw_data.az;
  data.gx = raw_data.gx;
  data.gy = raw_data.gy;
  data.gz = raw_data.gz;

  _updateStats();
  _setError(ERROR_NONE);
  
  return true;
}

/**
 * @brief Get vector data of specified type
 */
Vector<3> SpresenseIMU::getVector(VectorType type) {
  IMUData data;
  Vector<3> result(0, 0, 0);
  
  if (!read(data)) {
    return result;
  }

  switch (type) {
    case VECTOR_ACCELEROMETER:
      result = Vector<3>(data.ax, data.ay, data.az);
      break;
      
    case VECTOR_GYROSCOPE:
      result = Vector<3>(data.gx, data.gy, data.gz);
      break;
      
    case VECTOR_GYROSCOPE_RAD:
      result = Vector<3>(data.gx, data.gy, data.gz);
      result.toRadians();
      break;
      
    case VECTOR_LINEAR_ACCEL: {
      Vector<3> accel(data.ax, data.ay, data.az);
      Vector<3> gravity = _calculateGravity(accel);
      result = _removeGravity(accel, gravity);
      break;
    }
    
    case VECTOR_GRAVITY: {
      Vector<3> accel(data.ax, data.ay, data.az);
      result = _calculateGravity(accel);
      break;
    }
    
    default:
      break;
  }
  
  return result;
}

/**
 * @brief Get current orientation as quaternion
 */
Quaternion SpresenseIMU::getQuaternion() {
  // Update orientation with latest data
  IMUData data;
  if (read(data)) {
    Vector<3> accel(data.ax, data.ay, data.az);
    Vector<3> gyro(data.gx, data.gy, data.gz);
    
    if (!_orientation_initialized) {
      _initializeOrientation(accel);
      _orientation_initialized = true;
    } else {
      // Calculate time delta (approximate)
      float dt = 1.0f / _sample_rate;
      _updateOrientation(accel, gyro, dt);
    }
  }
  
  return _current_orientation;
}

/**
 * @brief Get Euler angles (roll, pitch, yaw)
 */
Vector<3> SpresenseIMU::getEulerAngles() {
  Quaternion q = getQuaternion();
  Vector<3> euler = q.toEuler();
  
  // Convert to degrees
  euler.toDegrees();
  return euler;
}

/**
 * @brief Get rotation matrix representing current orientation
 */
Matrix<3> SpresenseIMU::getRotationMatrix() {
  Quaternion q = getQuaternion();
  
  // Convert quaternion to rotation matrix
  double w = q.w(), x = q.x(), y = q.y(), z = q.z();
  
  return Matrix<3>(
    1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y),
        2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x),
        2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y)
  );
}

/**
 * @brief Get sensor temperature
 */
float SpresenseIMU::getTemperature() {
  if (!isReady()) {
    return NAN;
  }

  cxd5602pwbimu_data_t raw_data;
  ssize_t bytes_read = ::read(_imu_fd, &raw_data, sizeof(raw_data));
  
  if (bytes_read != sizeof(raw_data)) {
    return NAN;
  }

  return raw_data.temp;
}

/**
 * @brief Get calibration status
 */
SpresenseIMU::CalibrationStatus SpresenseIMU::getCalibration() {
  CalibrationStatus status;
  
  // Basic calibration status based on whether offsets are applied
  if (_offsets_applied) {
    status.system = 3;
    status.accel = 3;
    status.gyro = 3;
  } else {
    status.system = 0;
    status.accel = 0;
    status.gyro = 0;
  }
  
  return status;
}

/**
 * @brief Check if fully calibrated
 */
bool SpresenseIMU::isFullyCalibrated() {
  return _offsets_applied;
}

/**
 * @brief Get calibration offsets
 */
bool SpresenseIMU::getOffsets(CalibrationOffsets& offsets) {
  offsets = _offsets;
  return true;
}

/**
 * @brief Set calibration offsets
 */
bool SpresenseIMU::setOffsets(const CalibrationOffsets& offsets) {
  _offsets = offsets;
  _offsets_applied = true;
  return true;
}

/**
 * @brief Start gyroscope bias calibration
 */
bool SpresenseIMU::startGyroBiasCalibration(uint16_t samples) {
  if (!isReady()) {
    return false;
  }
  
  _calibrating = true;
  _calibration_mode = CALIB_GYRO_BIAS;
  _calibration_samples_target = samples;
  _calibration_samples_collected = 0;
  _calibration_start_time = millis();
  _gyro_bias_sum = Vector<3>(0, 0, 0);
  
  return true;
}

/**
 * @brief Start accelerometer calibration
 */
bool SpresenseIMU::startAccelCalibration(uint8_t orientation, uint16_t samples) {
  if (!isReady() || orientation >= 6) {
    return false;
  }
  
  _calibrating = true;
  _calibration_mode = CALIB_ACCEL_6POINT;
  _calibration_orientation = orientation;
  _calibration_samples_target = samples;
  _calibration_samples_collected = 0;
  _calibration_start_time = millis();
  _accel_samples[orientation] = Vector<3>(0, 0, 0);
  
  return true;
}

/**
 * @brief Start automatic calibration
 */
bool SpresenseIMU::startAutoCalibration(uint32_t duration_ms) {
  if (!isReady()) {
    return false;
  }
  
  _calibrating = true;
  _calibration_mode = CALIB_AUTO;
  _calibration_start_time = millis();
  _calibration_duration = duration_ms;
  
  // Reset offsets for calibration
  memset(&_offsets, 0, sizeof(_offsets));
  _offsets.accel_scale_x = 1.0f;
  _offsets.accel_scale_y = 1.0f;
  _offsets.accel_scale_z = 1.0f;
  _offsets.gyro_scale_x = 1.0f;
  _offsets.gyro_scale_y = 1.0f;
  _offsets.gyro_scale_z = 1.0f;
  
  return true;
}

/**
 * @brief Check if calibration is in progress
 */
bool SpresenseIMU::isCalibrating() {
  if (_calibrating) {
    switch (_calibration_mode) {
      case CALIB_AUTO:
        {
          uint32_t elapsed = millis() - _calibration_start_time;
          if (elapsed >= _calibration_duration) {
            _calibrating = false;
            _offsets_applied = true;
          }
        }
        break;
        
      case CALIB_GYRO_BIAS:
      case CALIB_ACCEL_6POINT:
        // These finish when enough samples are collected
        if (_calibration_samples_collected >= _calibration_samples_target) {
          _calibrating = false;
        }
        break;
        
      default:
        _calibrating = false;
        break;
    }
  }
  return _calibrating;
}

/**
 * @brief Get calibration progress
 */
float SpresenseIMU::getCalibrationProgress() {
  if (!_calibrating) {
    return _offsets_applied ? 1.0f : 0.0f;
  }
  
  switch (_calibration_mode) {
    case CALIB_AUTO:
      {
        uint32_t elapsed = millis() - _calibration_start_time;
        return (float)elapsed / (float)_calibration_duration;
      }
      
    case CALIB_GYRO_BIAS:
    case CALIB_ACCEL_6POINT:
      return (float)_calibration_samples_collected / (float)_calibration_samples_target;
      
    default:
      return 0.0f;
  }
}

/**
 * @brief Finish calibration and calculate offsets
 */
bool SpresenseIMU::finishCalibration() {
  if (_calibrating) {
    return false; // Still in progress
  }
  
  bool success = false;
  
  switch (_calibration_mode) {
    case CALIB_GYRO_BIAS:
      success = _calculateGyroBias();
      break;
      
    case CALIB_ACCEL_6POINT: {
      // Mark this orientation as done
      _accel_orientation_done[_calibration_orientation] = true;
      
      // Check if all 6 orientations are complete
      bool all_done = true;
      for (uint8_t i = 0; i < 6; i++) {
        if (!_accel_orientation_done[i]) {
          all_done = false;
          break;
        }
      }
      
      if (all_done) {
        success = _calculateAccelCalibration();
      } else {
        success = true; // Partial success
      }
    }
    break;
      
    default:
      success = true;
      break;
  }
  
  if (success) {
    _calibration_mode = CALIB_NONE;
    _offsets_applied = true;
  }
  
  return success;
}

/**
 * @brief Get device information
 */
SpresenseIMU::DeviceInfo SpresenseIMU::getDeviceInfo() {
  DeviceInfo info;
  info.chip_id = 0x02; // CXD5602PWB
  info.sw_revision = 1;
  info.hw_revision = 1;
  return info;
}

/**
 * @brief Perform self-test
 */
bool SpresenseIMU::selfTest() {
  if (!isReady()) {
    return false;
  }
  
  // Basic self-test: try to read data
  IMUData data;
  return readRaw(data);
}

/**
 * @brief Get last error code
 */
SpresenseIMU::ErrorCode SpresenseIMU::getLastError() const {
  return _last_error;
}

/**
 * @brief Get error description string
 */
String SpresenseIMU::getErrorString(ErrorCode error) {
  switch (error) {
    case ERROR_NONE: return "No error";
    case ERROR_INIT_FAILED: return "Initialization failed";
    case ERROR_DEVICE_OPEN_FAILED: return "Device open failed";
    case ERROR_INVALID_RANGE: return "Invalid range";
    case ERROR_INVALID_SAMPLE_RATE: return "Invalid sample rate";
    case ERROR_READ_FAILED: return "Read failed";
    case ERROR_NOT_INITIALIZED: return "Not initialized";
    default: return "Unknown error";
  }
}

/**
 * @brief Get performance statistics
 */
SpresenseIMU::PerformanceStats SpresenseIMU::getStats() {
  return _stats;
}

/**
 * @brief Reset performance statistics
 */
void SpresenseIMU::resetStats() {
  memset(&_stats, 0, sizeof(_stats));
  _last_sample_time = millis();
}

/**
 * @brief Enable/disable moving average filter
 */
void SpresenseIMU::enableMovingAverage(bool enable) {
  _moving_avg_enabled = enable;
  if (enable) {
    _filter_index = 0;
    // Clear filter history
    for (uint8_t i = 0; i < 10; i++) {
      _accel_history[i] = Vector<3>(0, 0, 0);
      _gyro_history[i] = Vector<3>(0, 0, 0);
    }
  }
}

/**
 * @brief Set moving average window size
 */
void SpresenseIMU::setMovingAverageWindow(uint8_t samples) {
  if (samples >= 1 && samples <= 10) {
    _moving_avg_window = samples;
  }
}

// Private methods

/**
 * @brief Initialize hardware
 */
bool SpresenseIMU::_initializeHardware() {
  // Initialize IMU driver
  if (board_cxd5602pwbimu_initialize(5) < 0) {
    return false;
  }

  // Small delay after driver initialization
  delay(10);

  // Open device
  _imu_fd = open("/dev/imu0", O_RDONLY);
  if (_imu_fd < 0) {
    _setError(ERROR_DEVICE_OPEN_FAILED);
    return false;
  }

  // Small delay after device open
  delay(5);

  return true;
}

/**
 * @brief Configure sensor ranges
 */
bool SpresenseIMU::_configureRanges() {
  if (_imu_fd < 0) {
    return false;
  }

  cxd5602pwbimu_range_t range;
  range.accel = _accel_range;
  range.gyro = _gyro_range;
  
  if (ioctl(_imu_fd, SNIOC_SDRANGE, (unsigned long)(uintptr_t)&range) < 0) {
    return false;
  }

  return true;
}

/**
 * @brief Enable sensor
 */
bool SpresenseIMU::_enableSensor() {
  if (_imu_fd < 0) {
    return false;
  }

  // Enable sensor (sample rate already set in setSampleRate())
  if (ioctl(_imu_fd, SNIOC_ENABLE, 1) < 0) {
    return false;
  }

  return true;
}

/**
 * @brief Set error code
 */
void SpresenseIMU::_setError(ErrorCode error) {
  _last_error = error;
}

/**
 * @brief Update performance statistics
 */
void SpresenseIMU::_updateStats() {
  _stats.total_samples++;
  
  uint32_t current_time = millis();
  if (_last_sample_time > 0) {
    uint32_t delta = current_time - _last_sample_time;
    if (delta > 0) {
      _stats.actual_sample_rate = 1000.0f / delta;
    }
  }
  _last_sample_time = current_time;
}

/**
 * @brief Apply moving average filter
 */
void SpresenseIMU::_applyMovingAverage(IMUData& data) {
  // Store current data in history
  _accel_history[_filter_index] = Vector<3>(data.ax, data.ay, data.az);
  _gyro_history[_filter_index] = Vector<3>(data.gx, data.gy, data.gz);
  
  // Calculate average
  Vector<3> accel_avg(0, 0, 0);
  Vector<3> gyro_avg(0, 0, 0);
  
  uint8_t samples = min(_moving_avg_window, (uint8_t)10);
  for (uint8_t i = 0; i < samples; i++) {
    accel_avg += _accel_history[i];
    gyro_avg += _gyro_history[i];
  }
  
  accel_avg /= samples;
  gyro_avg /= samples;
  
  // Update data with filtered values
  data.ax = accel_avg.x();
  data.ay = accel_avg.y();
  data.az = accel_avg.z();
  data.gx = gyro_avg.x();
  data.gy = gyro_avg.y();
  data.gz = gyro_avg.z();
  
  // Update filter index
  _filter_index = (_filter_index + 1) % _moving_avg_window;
}

/**
 * @brief Calculate gravity vector (simple lowpass approximation)
 */
Vector<3> SpresenseIMU::_calculateGravity(const Vector<3>& accel) {
  // Simple approach: assume static gravity is dominant Z component
  // In Phase 2, this will be replaced with proper sensor fusion
  static Vector<3> gravity_estimate(0, 0, 1.0);
  
  // Simple low-pass filter
  float alpha = 0.1f;
  gravity_estimate = gravity_estimate * (1.0f - alpha) + accel * alpha;
  
  return gravity_estimate;
}

/**
 * @brief Remove gravity from acceleration
 */
Vector<3> SpresenseIMU::_removeGravity(const Vector<3>& accel, const Vector<3>& gravity) {
  return accel - gravity;
}

/**
 * @brief Process calibration sample
 */
void SpresenseIMU::_processCalibrationSample(const IMUData& data) {
  if (!_calibrating || _calibration_samples_collected >= _calibration_samples_target) {
    return;
  }
  
  switch (_calibration_mode) {
    case CALIB_GYRO_BIAS:
      _gyro_bias_sum += Vector<3>(data.gx, data.gy, data.gz);
      break;
      
    case CALIB_ACCEL_6POINT:
      _accel_samples[_calibration_orientation] += Vector<3>(data.ax, data.ay, data.az);
      break;
      
    default:
      break;
  }
  
  _calibration_samples_collected++;
}

/**
 * @brief Calculate gyroscope bias from collected samples
 */
bool SpresenseIMU::_calculateGyroBias() {
  if (_calibration_samples_collected == 0) {
    return false;
  }
  
  // Calculate average bias
  Vector<3> bias = _gyro_bias_sum / (double)_calibration_samples_collected;
  
  // Update offsets
  _offsets.gyro_offset_x = (int16_t)(bias.x() * 100.0f);
  _offsets.gyro_offset_y = (int16_t)(bias.y() * 100.0f);
  _offsets.gyro_offset_z = (int16_t)(bias.z() * 100.0f);
  
  return true;
}

/**
 * @brief Calculate accelerometer calibration from 6-point data
 */
bool SpresenseIMU::_calculateAccelCalibration() {
  // Check if we have all 6 orientations
  for (uint8_t i = 0; i < 6; i++) {
    if (!_accel_orientation_done[i]) {
      return false;
    }
  }
  
  // Calculate average for each orientation
  Vector<3> avg_samples[6];
  for (uint8_t i = 0; i < 6; i++) {
    avg_samples[i] = _accel_samples[i] / (double)_calibration_samples_target;
  }
  
  // Calculate offsets (simple approach - assuming perfect alignment)
  // Orientations: +X, -X, +Y, -Y, +Z, -Z
  Vector<3> offset = Vector<3>(
    (avg_samples[0].x() + avg_samples[1].x()) / 2.0,
    (avg_samples[2].y() + avg_samples[3].y()) / 2.0,
    (avg_samples[4].z() + avg_samples[5].z()) / 2.0
  );
  
  // Calculate scale factors
  Vector<3> scale = Vector<3>(
    2.0 / (avg_samples[0].x() - avg_samples[1].x()),
    2.0 / (avg_samples[2].y() - avg_samples[3].y()),
    2.0 / (avg_samples[4].z() - avg_samples[5].z())
  );
  
  // Update offsets
  _offsets.accel_offset_x = (int16_t)(offset.x() * 1000.0f);
  _offsets.accel_offset_y = (int16_t)(offset.y() * 1000.0f);
  _offsets.accel_offset_z = (int16_t)(offset.z() * 1000.0f);
  
  _offsets.accel_scale_x = scale.x();
  _offsets.accel_scale_y = scale.y();
  _offsets.accel_scale_z = scale.z();
  
  return true;
}

/**
 * @brief Initialize orientation from accelerometer
 */
void SpresenseIMU::_initializeOrientation(const Vector<3>& accel) {
  // Normalize accelerometer reading
  Vector<3> accel_norm = accel;
  accel_norm.normalize();
  
  // Calculate initial orientation from gravity vector
  // Assuming device is initially level (no roll/pitch from gyro)
  Vector<3> gravity_world(0, 0, -1); // Z-down in world frame
  
  // Calculate rotation from world gravity to measured gravity
  Vector<3> axis = gravity_world.cross(accel_norm);
  double angle = acos(gravity_world.dot(accel_norm));
  
  if (axis.magnitude() > 0.001) {
    axis.normalize();
    _current_orientation.fromAxisAngle(axis, angle);
  } else {
    _current_orientation = Quaternion(); // Identity if aligned
  }
  
  _gravity_estimate = accel_norm;
}

/**
 * @brief Update orientation using gyroscope integration
 */
void SpresenseIMU::_updateOrientation(const Vector<3>& accel, const Vector<3>& gyro, float dt) {
  // Convert gyroscope data to rad/s
  Vector<3> gyro_rad = gyro;
  gyro_rad.toRadians();
  
  // Integrate gyroscope for orientation change
  double gyro_magnitude = gyro_rad.magnitude();
  if (gyro_magnitude > 0.001) {
    Vector<3> axis = gyro_rad / gyro_magnitude;
    double angle = gyro_magnitude * dt;
    
    Quaternion gyro_rotation(axis, angle);
    _current_orientation = _current_orientation * gyro_rotation;
  }
  
  // Accelerometer correction (complementary filter)
  Vector<3> accel_norm = accel;
  accel_norm.normalize();
  
  // Update gravity estimate with low-pass filter
  float alpha = 0.02f; // Accelerometer trust factor
  _gravity_estimate = _gravity_estimate * (1.0f - alpha) + accel_norm * alpha;
  
  // Apply accelerometer correction to quaternion
  Vector<3> predicted_gravity = _current_orientation.rotateVector(Vector<3>(0, 0, -1));
  Vector<3> error_axis = predicted_gravity.cross(accel_norm);
  
  if (error_axis.magnitude() > 0.001) {
    error_axis.normalize();
    double error_angle = asin(min(error_axis.magnitude(), 1.0)) * alpha;
    
    Quaternion correction(error_axis, error_angle);
    _current_orientation = correction * _current_orientation;
  }
  
  // Normalize quaternion to prevent drift
  _current_orientation.normalize();
}