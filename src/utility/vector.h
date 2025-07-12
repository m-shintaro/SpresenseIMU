#ifndef SPRESENSE_IMU_VECTOR_H
#define SPRESENSE_IMU_VECTOR_H

#include <math.h>

namespace spresense_imu {

/**
 * @brief Template class for N-dimensional vector operations
 * @tparam N Dimension of the vector (typically 3 for 3D vectors)
 */
template<uint8_t N>
class Vector {
  public:
    /**
     * @brief Default constructor - initializes all components to zero
     */
    Vector() {
      for (uint8_t i = 0; i < N; i++) {
        _data[i] = 0.0;
      }
    }

    /**
     * @brief Constructor for 3D vector with x, y, z components
     * @param x X component
     * @param y Y component  
     * @param z Z component
     */
    Vector(double x, double y, double z) {
      static_assert(N >= 3, "Vector must have at least 3 dimensions for x,y,z constructor");
      _data[0] = x;
      _data[1] = y;
      _data[2] = z;
      for (uint8_t i = 3; i < N; i++) {
        _data[i] = 0.0;
      }
    }

    /**
     * @brief Get reference to X component (index 0)
     * @return Reference to X component
     */
    double& x() { return _data[0]; }
    double x() const { return _data[0]; }

    /**
     * @brief Get reference to Y component (index 1) 
     * @return Reference to Y component
     */
    double& y() { 
      static_assert(N >= 2, "Vector must have at least 2 dimensions for y component");
      return _data[1]; 
    }
    double y() const { 
      static_assert(N >= 2, "Vector must have at least 2 dimensions for y component");
      return _data[1]; 
    }

    /**
     * @brief Get reference to Z component (index 2)
     * @return Reference to Z component
     */
    double& z() { 
      static_assert(N >= 3, "Vector must have at least 3 dimensions for z component");
      return _data[2]; 
    }
    double z() const { 
      static_assert(N >= 3, "Vector must have at least 3 dimensions for z component");
      return _data[2]; 
    }

    /**
     * @brief Array access operator
     * @param index Component index (0 to N-1)
     * @return Reference to component at index
     */
    double& operator[](uint8_t index) {
      return _data[index];
    }
    double operator[](uint8_t index) const {
      return _data[index];
    }

    /**
     * @brief Calculate vector magnitude (length)
     * @return Magnitude of the vector
     */
    double magnitude() const {
      double sum = 0.0;
      for (uint8_t i = 0; i < N; i++) {
        sum += _data[i] * _data[i];
      }
      return sqrt(sum);
    }

    /**
     * @brief Normalize vector to unit length
     */
    void normalize() {
      double mag = magnitude();
      if (mag > 0.0) {
        for (uint8_t i = 0; i < N; i++) {
          _data[i] /= mag;
        }
      }
    }

    /**
     * @brief Calculate dot product with another vector
     * @param v Other vector
     * @return Dot product result
     */
    double dot(const Vector<N>& v) const {
      double result = 0.0;
      for (uint8_t i = 0; i < N; i++) {
        result += _data[i] * v._data[i];
      }
      return result;
    }

    /**
     * @brief Calculate cross product (3D vectors only)
     * @param v Other vector
     * @return Cross product vector
     */
    Vector<N> cross(const Vector<N>& v) const {
      static_assert(N == 3, "Cross product is only defined for 3D vectors");
      Vector<N> result;
      result._data[0] = _data[1] * v._data[2] - _data[2] * v._data[1];
      result._data[1] = _data[2] * v._data[0] - _data[0] * v._data[2];
      result._data[2] = _data[0] * v._data[1] - _data[1] * v._data[0];
      return result;
    }

    /**
     * @brief Scale vector by scalar value
     * @param scalar Scaling factor
     * @return Scaled vector
     */
    Vector<N> scale(double scalar) const {
      Vector<N> result;
      for (uint8_t i = 0; i < N; i++) {
        result._data[i] = _data[i] * scalar;
      }
      return result;
    }

    /**
     * @brief Invert vector (negate all components)
     * @return Inverted vector
     */
    Vector<N> invert() const {
      return scale(-1.0);
    }

    /**
     * @brief Convert radians to degrees for all components
     */
    void toDegrees() {
      for (uint8_t i = 0; i < N; i++) {
        _data[i] = _data[i] * 180.0 / M_PI;
      }
    }

    /**
     * @brief Convert degrees to radians for all components
     */
    void toRadians() {
      for (uint8_t i = 0; i < N; i++) {
        _data[i] = _data[i] * M_PI / 180.0;
      }
    }

    // Operator overloads
    Vector<N> operator+(const Vector<N>& v) const {
      Vector<N> result;
      for (uint8_t i = 0; i < N; i++) {
        result._data[i] = _data[i] + v._data[i];
      }
      return result;
    }

    Vector<N> operator-(const Vector<N>& v) const {
      Vector<N> result;
      for (uint8_t i = 0; i < N; i++) {
        result._data[i] = _data[i] - v._data[i];
      }
      return result;
    }

    Vector<N> operator*(double scalar) const {
      return scale(scalar);
    }

    Vector<N> operator/(double scalar) const {
      return scale(1.0 / scalar);
    }

    Vector<N>& operator+=(const Vector<N>& v) {
      for (uint8_t i = 0; i < N; i++) {
        _data[i] += v._data[i];
      }
      return *this;
    }

    Vector<N>& operator-=(const Vector<N>& v) {
      for (uint8_t i = 0; i < N; i++) {
        _data[i] -= v._data[i];
      }
      return *this;
    }

    Vector<N>& operator*=(double scalar) {
      for (uint8_t i = 0; i < N; i++) {
        _data[i] *= scalar;
      }
      return *this;
    }

    Vector<N>& operator/=(double scalar) {
      for (uint8_t i = 0; i < N; i++) {
        _data[i] /= scalar;
      }
      return *this;
    }

  private:
    double _data[N];
};

} // namespace spresense_imu

#endif // SPRESENSE_IMU_VECTOR_H