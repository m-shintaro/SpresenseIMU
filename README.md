# SpresenseIMU Library

A comprehensive Arduino library for Sony Spresense Multi-IMU Add-on board. This library provides easy access to the 6-axis IMU sensors (accelerometer and gyroscope) on the add-on board with advanced features including calibration, filtering, and vector mathematics.

## Features

- **6-axis IMU Support**: 3-axis accelerometer + 3-axis gyroscope
- **Flexible Configuration**: Configurable ranges and sample rates
- **Vector Mathematics**: Built-in 3D vector operations for sensor data processing
- **Calibration Support**: Automatic and manual calibration with offset storage
- **Filtering**: Moving average filter for noise reduction
- **Performance Monitoring**: Real-time statistics and error handling
- **Multiple Data Types**: Raw data, calibrated data, and derived vectors
- **Arduino Compatible**: Follows Arduino library standards

## Supported Hardware

- Sony Spresense main board with Multi-IMU Add-on board
- Requires Spresense Arduino Core v3.4.3 or higher

## Quick Start

```cpp
#include <SpresenseIMU.h>

SpresenseIMU imu;

void setup() {
  Serial.begin(115200);

  // Initialize IMU with 100Hz sample rate
  if (!imu.begin(100)) {
    Serial.println("IMU initialization failed!");
    while (1);
  }

  // Configure ranges
  imu.setAccelRange(4);    // ±4g
  imu.setGyroRange(500);   // ±500°/s
}

void loop() {
  SpresenseIMU::IMUData data;

  if (imu.read(data)) {
    Serial.print("Accel: ");
    Serial.print(data.ax); Serial.print(", ");
    Serial.print(data.ay); Serial.print(", ");
    Serial.println(data.az);

    Serial.print("Gyro: ");
    Serial.print(data.gx); Serial.print(", ");
    Serial.print(data.gy); Serial.print(", ");
    Serial.println(data.gz);
  }

  delay(100);
}
```

## API Reference

### Initialization

```cpp
SpresenseIMU imu;                    // Create instance
bool begin(uint16_t odr = 200);      // Initialize with sample rate
bool isReady();                      // Check if ready
```

### Configuration

```cpp
bool setAccelRange(uint8_t range);   // Set accelerometer range (2, 4, 8, 16)
bool setGyroRange(uint16_t range);   // Set gyroscope range (125, 250, 500, 1000, 2000)
bool setSampleRate(uint16_t rate);   // Set sample rate (1-200 Hz)
uint16_t getSampleRate();            // Get current sample rate
```

### Data Reading

```cpp
bool read(IMUData &data);            // Read calibrated data
bool readRaw(IMUData &data);         // Read raw data
Vector<3> getVector(VectorType type); // Get vector data
float getTemperature();              // Get temperature (not supported)
```

### Vector Types

- `VECTOR_ACCELEROMETER`: Acceleration in g
- `VECTOR_GYROSCOPE`: Angular velocity in deg/s
- `VECTOR_GYROSCOPE_RAD`: Angular velocity in rad/s
- `VECTOR_LINEAR_ACCEL`: Linear acceleration (gravity removed)
- `VECTOR_GRAVITY`: Gravity vector

### Vector Operations

```cpp
Vector<3> vec(x, y, z);      // Create vector
double magnitude();          // Get magnitude
void normalize();            // Normalize to unit vector
double dot(Vector<3> &v);    // Dot product
Vector<3> cross(Vector<3> &v); // Cross product
Vector<3> scale(double s);   // Scale by factor
void toDegrees();           // Convert rad to deg
void toRadians();           // Convert deg to rad
```

### Calibration

```cpp
bool startAutoCalibration(uint32_t duration_ms); // Start auto calibration
bool isCalibrating();                            // Check calibration status
float getCalibrationProgress();                  // Get progress (0.0-1.0)
bool getOffsets(CalibrationOffsets &offsets);    // Get calibration offsets
bool setOffsets(const CalibrationOffsets &offsets); // Set calibration offsets
```

### Filtering

```cpp
void enableMovingAverage(bool enable);       // Enable/disable moving average
void setMovingAverageWindow(uint8_t samples); // Set window size (1-10)
```

### Error Handling

```cpp
ErrorCode getLastError();              // Get last error code
String getErrorString(ErrorCode error); // Get error description
```

## Examples

The library includes several examples:

- **BasicReading**: Basic sensor data reading and display
- **VectorOperations**: Demonstrates vector mathematics and operations

## Data Structures

### IMUData

```cpp
struct IMUData {
  uint64_t timestamp;  // Timestamp in microseconds
  float ax, ay, az;    // Acceleration in g
  float gx, gy, gz;    // Angular velocity in deg/s
};
```

### CalibrationStatus

```cpp
struct CalibrationStatus {
  uint8_t system;  // System calibration (0-3)
  uint8_t accel;   // Accelerometer calibration (0-3)
  uint8_t gyro;    // Gyroscope calibration (0-3)
};
```

## Specifications

- **Accelerometer Range**: ±2g, ±4g, ±8g, ±16g
- **Gyroscope Range**: ±125°/s, ±250°/s, ±500°/s, ±1000°/s, ±2000°/s
- **Sample Rate**: 1-200 Hz
- **Data Format**: 32-bit floating point
- **Interface**: NuttX sensor driver (/dev/imu0)

## Performance Notes

- Maximum sample rate: 200 Hz
- Typical power consumption: ~2.5 mA (active mode)
- Memory usage: ~2KB RAM, ~15KB flash
- Processing overhead: <1ms per sample @ 100MHz

## Troubleshooting

### Common Issues

**"IMU initialization failed"**

- Check that Multi-IMU Add-on board is properly connected
- Verify Spresense core version ≥3.4.3
- Try power cycling the board

**"Read failed" errors**

- Check sample rate is not too high
- Ensure IMU is properly initialized
- Verify no other processes are using /dev/imu0

**Noisy data**

- Enable moving average filter
- Use appropriate range settings
- Ensure stable power supply

### Error Codes

- `ERROR_NONE`: No error
- `ERROR_INIT_FAILED`: Hardware initialization failed
- `ERROR_DEVICE_OPEN_FAILED`: Cannot open device
- `ERROR_INVALID_RANGE`: Invalid range parameter
- `ERROR_INVALID_SAMPLE_RATE`: Invalid sample rate
- `ERROR_READ_FAILED`: Data read failed
- `ERROR_NOT_INITIALIZED`: IMU not initialized

## Contributing

Contributions are welcome! Please:

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests for new functionality
5. Submit a pull request

## License

This library is released under the MIT License. See LICENSE file for details.

## Changelog

### Version 1.0.0

- Initial release
- Basic IMU data reading
- Vector mathematics utilities
- Calibration support
- Moving average filtering
- Performance monitoring

## Author

Created by Shintaro Matsumoto for Sony Spresense platform.

## Acknowledgments

- Sony Spresense development team
- NuttX real-time operating system
- Arduino community

---

# 日本語ドキュメント

## SpresenseIMU ライブラリ

Sony Spresense Multi-IMU Add-on ボード用の包括的な Arduino ライブラリです。このライブラリは、キャリブレーション、フィルタリング、ベクトル演算などの高度な機能を含む、アドオンボード上の 6 軸 IMU センサー（加速度センサー + ジャイロスコープ）への簡単なアクセスを提供します。

## 特徴

- **6 軸 IMU サポート**: 3 軸加速度センサー + 3 軸ジャイロスコープ
- **柔軟な設定**: 設定可能な測定範囲とサンプリングレート
- **ベクトル演算**: センサーデータ処理のための内蔵 3D ベクトル操作
- **キャリブレーションサポート**: オフセット保存機能付きの自動・手動キャリブレーション
- **フィルタリング**: ノイズ軽減のための移動平均フィルター
- **パフォーマンス監視**: リアルタイム統計とエラーハンドリング
- **複数のデータタイプ**: 生データ、キャリブレーション済みデータ、導出ベクトル
- **Arduino 互換**: Arduino ライブラリ標準に準拠

## サポートハードウェア

- Sony Spresense メインボード + Multi-IMU Add-on ボード
- Spresense Arduino Core v3.4.3 以上が必要

## クイックスタート

```cpp
#include <SpresenseIMU.h>

SpresenseIMU imu;

void setup() {
  Serial.begin(115200);

  // 100HzサンプリングレートでIMUを初期化
  if (!imu.begin(100)) {
    Serial.println("IMU initialization failed!");
    while (1);
  }

  // 測定範囲を設定
  imu.setAccelRange(4);    // ±4g
  imu.setGyroRange(500);   // ±500°/s
}

void loop() {
  SpresenseIMU::IMUData data;

  if (imu.read(data)) {
    Serial.print("Accel: ");
    Serial.print(data.ax); Serial.print(", ");
    Serial.print(data.ay); Serial.print(", ");
    Serial.println(data.az);

    Serial.print("Gyro: ");
    Serial.print(data.gx); Serial.print(", ");
    Serial.print(data.gy); Serial.print(", ");
    Serial.println(data.gz);
  }

  delay(100);
}
```

## 主な API

### 初期化

- `bool begin(uint16_t odr = 200)`: 指定サンプリングレートで初期化
- `bool isReady()`: 準備状態の確認

### 設定

- `bool setAccelRange(uint8_t range)`: 加速度センサー範囲設定（2, 4, 8, 16g）
- `bool setGyroRange(uint16_t range)`: ジャイロスコープ範囲設定（125, 250, 500, 1000, 2000°/s）
- `bool setSampleRate(uint16_t rate)`: サンプリングレート設定（1-200 Hz）

### データ読み取り

- `bool read(IMUData &data)`: キャリブレーション済みデータの読み取り
- `bool readRaw(IMUData &data)`: 生データの読み取り
- `Vector<3> getVector(VectorType type)`: ベクトルデータの取得

### キャリブレーション

- `bool startAutoCalibration(uint32_t duration_ms)`: 自動キャリブレーション開始
- `bool isCalibrating()`: キャリブレーション状態の確認
- `float getCalibrationProgress()`: キャリブレーション進行状況（0.0-1.0）

## 仕様

- **加速度センサー範囲**: ±2g, ±4g, ±8g, ±16g
- **ジャイロスコープ範囲**: ±125°/s, ±250°/s, ±500°/s, ±1000°/s, ±2000°/s
- **サンプリングレート**: 1-200 Hz
- **データ形式**: 32 ビット浮動小数点
- **インターフェース**: NuttX センサードライバー（/dev/imu0）

## トラブルシューティング

### よくある問題

**"IMU initialization failed"**

- Multi-IMU Add-on ボードが正しく接続されているか確認
- Spresense コアバージョンが 3.4.3 以上であることを確認
- ボードの電源を再投入してみる

**"Read failed"エラー**

- サンプリングレートが高すぎないか確認
- IMU が正しく初期化されているか確認
- 他のプロセスが/dev/imu0 を使用していないか確認

**ノイズの多いデータ**

- 移動平均フィルターを有効にする
- 適切な測定範囲設定を使用
- 安定した電源供給を確保

## ライセンス

このライブラリは MIT ライセンスの下でリリースされています。詳細は LICENSE ファイルを参照してください。

## 作者

Sony Spresense プラットフォーム用に Shintaro Matsumoto が作成。
