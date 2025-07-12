#ifndef SPRESENSE_IMU_MATRIX_H
#define SPRESENSE_IMU_MATRIX_H

#include <math.h>
#include "vector.h"

namespace spresense_imu {

// Forward declaration for circular dependency
class Quaternion;

/**
 * @brief Template class for N×N matrix operations
 * @tparam N Size of the square matrix (typically 3 for 3×3 matrices)
 */
template<uint8_t N>
class Matrix {
  public:
    /**
     * @brief Default constructor - initializes all elements to zero
     */
    Matrix() {
      for (uint8_t i = 0; i < N; i++) {
        for (uint8_t j = 0; j < N; j++) {
          _data[i][j] = 0.0;
        }
      }
    }

    /**
     * @brief Constructor for 3x3 matrix with explicit values
     * Values are provided row by row
     */
    Matrix(double m00, double m01, double m02,
           double m10, double m11, double m12,
           double m20, double m21, double m22) {
      static_assert(N == 3, "This constructor is only for 3x3 matrices");
      _data[0][0] = m00; _data[0][1] = m01; _data[0][2] = m02;
      _data[1][0] = m10; _data[1][1] = m11; _data[1][2] = m12;
      _data[2][0] = m20; _data[2][1] = m21; _data[2][2] = m22;
    }

    /**
     * @brief Create identity matrix
     * @return Identity matrix
     */
    static Matrix<N> identity() {
      Matrix<N> result;
      for (uint8_t i = 0; i < N; i++) {
        result._data[i][i] = 1.0;
      }
      return result;
    }

    /**
     * @brief Access matrix element at (row, col)
     * @param row Row index (0-based)
     * @param col Column index (0-based)
     * @return Reference to matrix element
     */
    double& cell(uint8_t row, uint8_t col) {
      return _data[row][col];
    }

    /**
     * @brief Access matrix element at (row, col) - const version
     * @param row Row index (0-based)
     * @param col Column index (0-based)
     * @return Matrix element value
     */
    double cell(uint8_t row, uint8_t col) const {
      return _data[row][col];
    }

    /**
     * @brief Function call operator for matrix element access
     * @param row Row index (0-based)
     * @param col Column index (0-based)
     * @return Reference to matrix element
     */
    double& operator()(uint8_t row, uint8_t col) {
      return _data[row][col];
    }

    /**
     * @brief Function call operator for matrix element access - const version
     */
    double operator()(uint8_t row, uint8_t col) const {
      return _data[row][col];
    }

    /**
     * @brief Get a row as a vector
     * @param row Row index
     * @return Row vector
     */
    Vector<N> rowToVector(uint8_t row) const {
      Vector<N> result;
      for (uint8_t j = 0; j < N; j++) {
        result[j] = _data[row][j];
      }
      return result;
    }

    /**
     * @brief Get a column as a vector
     * @param col Column index
     * @return Column vector
     */
    Vector<N> colToVector(uint8_t col) const {
      Vector<N> result;
      for (uint8_t i = 0; i < N; i++) {
        result[i] = _data[i][col];
      }
      return result;
    }

    /**
     * @brief Set a row from a vector
     * @param v Vector to set as row
     * @param row Row index
     */
    void vectorToRow(const Vector<N>& v, uint8_t row) {
      for (uint8_t j = 0; j < N; j++) {
        _data[row][j] = v[j];
      }
    }

    /**
     * @brief Set a column from a vector
     * @param v Vector to set as column
     * @param col Column index
     */
    void vectorToCol(const Vector<N>& v, uint8_t col) {
      for (uint8_t i = 0; i < N; i++) {
        _data[i][col] = v[i];
      }
    }

    /**
     * @brief Calculate matrix transpose
     * @return Transposed matrix
     */
    Matrix<N> transpose() const {
      Matrix<N> result;
      for (uint8_t i = 0; i < N; i++) {
        for (uint8_t j = 0; j < N; j++) {
          result._data[j][i] = _data[i][j];
        }
      }
      return result;
    }

    /**
     * @brief Calculate matrix determinant
     * @return Determinant value
     */
    double determinant() const {
      if (N == 1) {
        return _data[0][0];
      } else if (N == 2) {
        return _data[0][0] * _data[1][1] - _data[0][1] * _data[1][0];
      } else if (N == 3) {
        return _data[0][0] * (_data[1][1] * _data[2][2] - _data[1][2] * _data[2][1])
             - _data[0][1] * (_data[1][0] * _data[2][2] - _data[1][2] * _data[2][0])
             + _data[0][2] * (_data[1][0] * _data[2][1] - _data[1][1] * _data[2][0]);
      } else {
        // General case using cofactor expansion (inefficient for large matrices)
        double det = 0.0;
        for (uint8_t j = 0; j < N; j++) {
          Matrix<N-1> minor = minorMatrix(0, j);
          double cofactor = ((j % 2 == 0) ? 1.0 : -1.0) * minor.determinant();
          det += _data[0][j] * cofactor;
        }
        return det;
      }
    }

    /**
     * @brief Calculate matrix inverse
     * @return Inverse matrix (returns identity if not invertible)
     */
    Matrix<N> inverse() const {
      if (N == 3) {
        return inverse3x3();
      } else {
        // For other sizes, return identity (placeholder)
        return Matrix<N>::identity();
      }
    }

    /**
     * @brief Calculate trace (sum of diagonal elements)
     * @return Trace value
     */
    double trace() const {
      double tr = 0.0;
      for (uint8_t i = 0; i < N; i++) {
        tr += _data[i][i];
      }
      return tr;
    }

    /**
     * @brief Get minor matrix by removing specified row and column
     * @param row Row to remove
     * @param col Column to remove
     * @return Minor matrix of size (N-1)×(N-1)
     */
    Matrix<N-1> minorMatrix(uint8_t row, uint8_t col) const {
      static_assert(N > 1, "Cannot create minor matrix of 1×1 matrix");
      Matrix<N-1> result;
      
      uint8_t dest_row = 0;
      for (uint8_t i = 0; i < N; i++) {
        if (i == row) continue;
        
        uint8_t dest_col = 0;
        for (uint8_t j = 0; j < N; j++) {
          if (j == col) continue;
          
          result._data[dest_row][dest_col] = _data[i][j];
          dest_col++;
        }
        dest_row++;
      }
      
      return result;
    }

    /**
     * @brief Multiply matrix with vector
     * @param v Vector to multiply
     * @return Result vector
     */
    Vector<N> multiplyVector(const Vector<N>& v) const {
      Vector<N> result;
      for (uint8_t i = 0; i < N; i++) {
        result[i] = 0.0;
        for (uint8_t j = 0; j < N; j++) {
          result[i] += _data[i][j] * v[j];
        }
      }
      return result;
    }

    // Operator overloads
    /**
     * @brief Matrix addition
     */
    Matrix<N> operator+(const Matrix<N>& m) const {
      Matrix<N> result;
      for (uint8_t i = 0; i < N; i++) {
        for (uint8_t j = 0; j < N; j++) {
          result._data[i][j] = _data[i][j] + m._data[i][j];
        }
      }
      return result;
    }

    /**
     * @brief Matrix subtraction
     */
    Matrix<N> operator-(const Matrix<N>& m) const {
      Matrix<N> result;
      for (uint8_t i = 0; i < N; i++) {
        for (uint8_t j = 0; j < N; j++) {
          result._data[i][j] = _data[i][j] - m._data[i][j];
        }
      }
      return result;
    }

    /**
     * @brief Matrix multiplication
     */
    Matrix<N> operator*(const Matrix<N>& m) const {
      Matrix<N> result;
      for (uint8_t i = 0; i < N; i++) {
        for (uint8_t j = 0; j < N; j++) {
          result._data[i][j] = 0.0;
          for (uint8_t k = 0; k < N; k++) {
            result._data[i][j] += _data[i][k] * m._data[k][j];
          }
        }
      }
      return result;
    }

    /**
     * @brief Scalar multiplication
     */
    Matrix<N> operator*(double scalar) const {
      Matrix<N> result;
      for (uint8_t i = 0; i < N; i++) {
        for (uint8_t j = 0; j < N; j++) {
          result._data[i][j] = _data[i][j] * scalar;
        }
      }
      return result;
    }

    /**
     * @brief Matrix-vector multiplication
     */
    Vector<N> operator*(const Vector<N>& v) const {
      return multiplyVector(v);
    }

    /**
     * @brief In-place addition
     */
    Matrix<N>& operator+=(const Matrix<N>& m) {
      for (uint8_t i = 0; i < N; i++) {
        for (uint8_t j = 0; j < N; j++) {
          _data[i][j] += m._data[i][j];
        }
      }
      return *this;
    }

    /**
     * @brief In-place subtraction
     */
    Matrix<N>& operator-=(const Matrix<N>& m) {
      for (uint8_t i = 0; i < N; i++) {
        for (uint8_t j = 0; j < N; j++) {
          _data[i][j] -= m._data[i][j];
        }
      }
      return *this;
    }

    /**
     * @brief In-place multiplication
     */
    Matrix<N>& operator*=(const Matrix<N>& m) {
      *this = *this * m;
      return *this;
    }

    /**
     * @brief In-place scalar multiplication
     */
    Matrix<N>& operator*=(double scalar) {
      for (uint8_t i = 0; i < N; i++) {
        for (uint8_t j = 0; j < N; j++) {
          _data[i][j] *= scalar;
        }
      }
      return *this;
    }

    // 3D-specific rotation matrix creation methods
    /**
     * @brief Create rotation matrix around X-axis (3×3 only)
     * @param angle Rotation angle in radians
     * @return Rotation matrix
     */
    static Matrix<3> rotationX(double angle) {
      static_assert(N == 3, "This method is only for 3×3 matrices");
      double c = cos(angle);
      double s = sin(angle);
      return Matrix<3>(
        1.0, 0.0, 0.0,
        0.0,   c,  -s,
        0.0,   s,   c
      );
    }

    /**
     * @brief Create rotation matrix around Y-axis (3×3 only)
     * @param angle Rotation angle in radians
     * @return Rotation matrix
     */
    static Matrix<3> rotationY(double angle) {
      static_assert(N == 3, "This method is only for 3×3 matrices");
      double c = cos(angle);
      double s = sin(angle);
      return Matrix<3>(
          c, 0.0,   s,
        0.0, 1.0, 0.0,
         -s, 0.0,   c
      );
    }

    /**
     * @brief Create rotation matrix around Z-axis (3×3 only)
     * @param angle Rotation angle in radians
     * @return Rotation matrix
     */
    static Matrix<3> rotationZ(double angle) {
      static_assert(N == 3, "This method is only for 3×3 matrices");
      double c = cos(angle);
      double s = sin(angle);
      return Matrix<3>(
          c,  -s, 0.0,
          s,   c, 0.0,
        0.0, 0.0, 1.0
      );
    }

    /**
     * @brief Create rotation matrix from Euler angles (3×3 only)
     * @param euler Euler angles [roll, pitch, yaw] in radians
     * @return Rotation matrix
     */
    static Matrix<3> fromEuler(const Vector<3>& euler) {
      static_assert(N == 3, "This method is only for 3×3 matrices");
      return rotationZ(euler.z()) * rotationY(euler.y()) * rotationX(euler.x());
    }

    /**
     * @brief Convert matrix to Euler angles (3×3 only)
     * @return Euler angles [roll, pitch, yaw] in radians
     */
    Vector<3> toEuler() const {
      static_assert(N == 3, "This method is only for 3×3 matrices");
      
      Vector<3> euler;
      
      // Extract Euler angles from rotation matrix
      // Assuming ZYX rotation order
      double sy = sqrt(_data[0][0] * _data[0][0] + _data[1][0] * _data[1][0]);
      
      bool singular = sy < 1e-6;
      
      if (!singular) {
        euler.x() = atan2(_data[2][1], _data[2][2]);  // Roll
        euler.y() = atan2(-_data[2][0], sy);          // Pitch
        euler.z() = atan2(_data[1][0], _data[0][0]);  // Yaw
      } else {
        euler.x() = atan2(-_data[1][2], _data[1][1]); // Roll
        euler.y() = atan2(-_data[2][0], sy);          // Pitch
        euler.z() = 0;                                // Yaw
      }
      
      return euler;
    }

  private:
    double _data[N][N];

    /**
     * @brief Calculate inverse of 3×3 matrix using cofactor method
     */
    Matrix<3> inverse3x3() const {
      static_assert(N == 3, "This method is only for 3×3 matrices");
      
      double det = determinant();
      if (abs(det) < 1e-10) {
        return Matrix<3>::identity(); // Return identity if not invertible
      }
      
      Matrix<3> result;
      
      // Calculate cofactor matrix
      result._data[0][0] = (_data[1][1] * _data[2][2] - _data[1][2] * _data[2][1]) / det;
      result._data[0][1] = (_data[0][2] * _data[2][1] - _data[0][1] * _data[2][2]) / det;
      result._data[0][2] = (_data[0][1] * _data[1][2] - _data[0][2] * _data[1][1]) / det;
      
      result._data[1][0] = (_data[1][2] * _data[2][0] - _data[1][0] * _data[2][2]) / det;
      result._data[1][1] = (_data[0][0] * _data[2][2] - _data[0][2] * _data[2][0]) / det;
      result._data[1][2] = (_data[0][2] * _data[1][0] - _data[0][0] * _data[1][2]) / det;
      
      result._data[2][0] = (_data[1][0] * _data[2][1] - _data[1][1] * _data[2][0]) / det;
      result._data[2][1] = (_data[0][1] * _data[2][0] - _data[0][0] * _data[2][1]) / det;
      result._data[2][2] = (_data[0][0] * _data[1][1] - _data[0][1] * _data[1][0]) / det;
      
      return result;
    }
};

} // namespace spresense_imu

#endif // SPRESENSE_IMU_MATRIX_H