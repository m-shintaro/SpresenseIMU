#ifndef SPRESENSE_IMU_QUATERNION_H
#define SPRESENSE_IMU_QUATERNION_H

#include <math.h>
#include "vector.h"

namespace spresense_imu {

/**
 * @brief Quaternion class for representing rotations and orientations
 * 
 * Quaternions provide a mathematically robust way to represent rotations
 * without gimbal lock issues. A quaternion consists of four components:
 * w (scalar) and x, y, z (vector part).
 */
class Quaternion {
  public:
    /**
     * @brief Default constructor - creates identity quaternion (no rotation)
     */
    Quaternion() : _w(1.0), _x(0.0), _y(0.0), _z(0.0) {}

    /**
     * @brief Constructor with explicit components
     * @param w Scalar component
     * @param x X vector component
     * @param y Y vector component  
     * @param z Z vector component
     */
    Quaternion(double w, double x, double y, double z) 
      : _w(w), _x(x), _y(y), _z(z) {}

    /**
     * @brief Constructor from axis-angle representation
     * @param axis Rotation axis (should be normalized)
     * @param angle Rotation angle in radians
     */
    Quaternion(const Vector<3>& axis, double angle) {
      fromAxisAngle(axis, angle);
    }

    // Component accessors
    /**
     * @brief Get/set W (scalar) component
     */
    double& w() { return _w; }
    double w() const { return _w; }

    /**
     * @brief Get/set X component
     */
    double& x() { return _x; }
    double x() const { return _x; }

    /**
     * @brief Get/set Y component
     */
    double& y() { return _y; }
    double y() const { return _y; }

    /**
     * @brief Get/set Z component
     */
    double& z() { return _z; }
    double z() const { return _z; }

    /**
     * @brief Calculate quaternion magnitude (norm)
     * @return Magnitude of the quaternion
     */
    double magnitude() const {
      return sqrt(_w*_w + _x*_x + _y*_y + _z*_z);
    }

    /**
     * @brief Normalize quaternion to unit length
     * Unit quaternions represent pure rotations
     */
    void normalize() {
      double mag = magnitude();
      if (mag > 0.0) {
        _w /= mag;
        _x /= mag;
        _y /= mag;
        _z /= mag;
      }
    }

    /**
     * @brief Get normalized quaternion without modifying this one
     * @return Normalized quaternion
     */
    Quaternion normalized() const {
      Quaternion result = *this;
      result.normalize();
      return result;
    }

    /**
     * @brief Calculate conjugate quaternion
     * For unit quaternions, conjugate represents inverse rotation
     * @return Conjugate quaternion
     */
    Quaternion conjugate() const {
      return Quaternion(_w, -_x, -_y, -_z);
    }

    /**
     * @brief Calculate inverse quaternion
     * @return Inverse quaternion
     */
    Quaternion inverse() const {
      double mag_sq = _w*_w + _x*_x + _y*_y + _z*_z;
      if (mag_sq > 0.0) {
        return Quaternion(_w/mag_sq, -_x/mag_sq, -_y/mag_sq, -_z/mag_sq);
      }
      return Quaternion(); // Return identity if magnitude is zero
    }

    /**
     * @brief Scale quaternion by scalar value
     * @param scalar Scaling factor
     * @return Scaled quaternion
     */
    Quaternion scale(double scalar) const {
      return Quaternion(_w * scalar, _x * scalar, _y * scalar, _z * scalar);
    }

    /**
     * @brief Set quaternion from axis-angle representation
     * @param axis Rotation axis (should be normalized)
     * @param angle Rotation angle in radians
     */
    void fromAxisAngle(const Vector<3>& axis, double angle) {
      double half_angle = angle * 0.5;
      double sin_half = sin(half_angle);
      
      _w = cos(half_angle);
      _x = axis.x() * sin_half;
      _y = axis.y() * sin_half;
      _z = axis.z() * sin_half;
    }

    /**
     * @brief Convert quaternion to axis-angle representation
     * @param axis Reference to store rotation axis
     * @param angle Reference to store rotation angle in radians
     */
    void toAxisAngle(Vector<3>& axis, double& angle) const {
      // Normalize quaternion first
      Quaternion q = normalized();
      
      // Calculate angle
      angle = 2.0 * acos(abs(q._w));
      
      // Calculate axis
      double sin_half = sqrt(1.0 - q._w * q._w);
      if (sin_half > 0.001) {
        axis = Vector<3>(q._x / sin_half, q._y / sin_half, q._z / sin_half);
      } else {
        // Arbitrary axis for small rotations
        axis = Vector<3>(1.0, 0.0, 0.0);
      }
    }

    /**
     * @brief Convert quaternion to Euler angles (roll, pitch, yaw)
     * @return Vector containing Euler angles in radians [roll, pitch, yaw]
     */
    Vector<3> toEuler() const {
      // Normalize quaternion first
      Quaternion q = normalized();
      
      Vector<3> euler;
      
      // Roll (x-axis rotation)
      double sinr_cosp = 2 * (q._w * q._x + q._y * q._z);
      double cosr_cosp = 1 - 2 * (q._x * q._x + q._y * q._y);
      euler.x() = atan2(sinr_cosp, cosr_cosp);
      
      // Pitch (y-axis rotation)
      double sinp = 2 * (q._w * q._y - q._z * q._x);
      if (abs(sinp) >= 1) {
        euler.y() = copysign(M_PI / 2, sinp); // Use 90 degrees if out of range
      } else {
        euler.y() = asin(sinp);
      }
      
      // Yaw (z-axis rotation)
      double siny_cosp = 2 * (q._w * q._z + q._x * q._y);
      double cosy_cosp = 1 - 2 * (q._y * q._y + q._z * q._z);
      euler.z() = atan2(siny_cosp, cosy_cosp);
      
      return euler;
    }

    /**
     * @brief Set quaternion from Euler angles
     * @param euler Euler angles in radians [roll, pitch, yaw]
     */
    void fromEuler(const Vector<3>& euler) {
      double cr = cos(euler.x() * 0.5);
      double sr = sin(euler.x() * 0.5);
      double cp = cos(euler.y() * 0.5);
      double sp = sin(euler.y() * 0.5);
      double cy = cos(euler.z() * 0.5);
      double sy = sin(euler.z() * 0.5);

      _w = cr * cp * cy + sr * sp * sy;
      _x = sr * cp * cy - cr * sp * sy;
      _y = cr * sp * cy + sr * cp * sy;
      _z = cr * cp * sy - sr * sp * cy;
    }

    /**
     * @brief Rotate a vector by this quaternion
     * @param v Vector to rotate
     * @return Rotated vector
     */
    Vector<3> rotateVector(const Vector<3>& v) const {
      // Convert vector to quaternion (w=0, x=v.x, y=v.y, z=v.z)
      Quaternion vec_q(0, v.x(), v.y(), v.z());
      
      // Perform rotation: q * v * q^-1
      Quaternion result = (*this) * vec_q * conjugate();
      
      return Vector<3>(result._x, result._y, result._z);
    }

    /**
     * @brief Calculate dot product with another quaternion
     * @param q Other quaternion
     * @return Dot product
     */
    double dot(const Quaternion& q) const {
      return _w * q._w + _x * q._x + _y * q._y + _z * q._z;
    }

    /**
     * @brief Spherical linear interpolation between two quaternions
     * @param q Target quaternion
     * @param t Interpolation parameter (0.0 to 1.0)
     * @return Interpolated quaternion
     */
    Quaternion slerp(const Quaternion& q, double t) const {
      // Compute the cosine of the angle between quaternions
      double dot_product = dot(q);
      
      // If the dot product is negative, slerp won't take the shorter path.
      // Note that v1 and -v1 are equivalent when the represent rotations.
      Quaternion q1 = q;
      if (dot_product < 0.0) {
        q1 = q.scale(-1.0);
        dot_product = -dot_product;
      }
      
      // If the inputs are too close, linearly interpolate
      if (dot_product > 0.9995) {
        Quaternion result = *this + (q1 - *this).scale(t);
        result.normalize();
        return result;
      }
      
      // Calculate angle and its sine
      double theta_0 = acos(dot_product);
      double sin_theta_0 = sin(theta_0);
      double theta = theta_0 * t;
      double sin_theta = sin(theta);
      
      // Calculate interpolation coefficients
      double s0 = cos(theta) - dot_product * sin_theta / sin_theta_0;
      double s1 = sin_theta / sin_theta_0;
      
      return scale(s0) + q1.scale(s1);
    }

    // Operator overloads
    /**
     * @brief Quaternion multiplication (composition of rotations)
     * @param q Right-hand side quaternion
     * @return Product quaternion
     */
    Quaternion operator*(const Quaternion& q) const {
      return Quaternion(
        _w * q._w - _x * q._x - _y * q._y - _z * q._z,
        _w * q._x + _x * q._w + _y * q._z - _z * q._y,
        _w * q._y - _x * q._z + _y * q._w + _z * q._x,
        _w * q._z + _x * q._y - _y * q._x + _z * q._w
      );
    }

    /**
     * @brief Quaternion addition
     * @param q Right-hand side quaternion
     * @return Sum quaternion
     */
    Quaternion operator+(const Quaternion& q) const {
      return Quaternion(_w + q._w, _x + q._x, _y + q._y, _z + q._z);
    }

    /**
     * @brief Quaternion subtraction
     * @param q Right-hand side quaternion
     * @return Difference quaternion
     */
    Quaternion operator-(const Quaternion& q) const {
      return Quaternion(_w - q._w, _x - q._x, _y - q._y, _z - q._z);
    }

    /**
     * @brief Scalar multiplication
     * @param scalar Scalar value
     * @return Scaled quaternion
     */
    Quaternion operator*(double scalar) const {
      return scale(scalar);
    }

    /**
     * @brief Scalar division
     * @param scalar Scalar value
     * @return Scaled quaternion
     */
    Quaternion operator/(double scalar) const {
      return scale(1.0 / scalar);
    }

    /**
     * @brief In-place multiplication
     */
    Quaternion& operator*=(const Quaternion& q) {
      *this = *this * q;
      return *this;
    }

    /**
     * @brief In-place addition
     */
    Quaternion& operator+=(const Quaternion& q) {
      _w += q._w;
      _x += q._x;
      _y += q._y;
      _z += q._z;
      return *this;
    }

    /**
     * @brief In-place subtraction
     */
    Quaternion& operator-=(const Quaternion& q) {
      _w -= q._w;
      _x -= q._x;
      _y -= q._y;
      _z -= q._z;
      return *this;
    }

  private:
    double _w, _x, _y, _z;
};

} // namespace spresense_imu

#endif // SPRESENSE_IMU_QUATERNION_H