"""Quaternion attitude estimation for the Joy-Con IMU.

The Joy-Con exposes a gyroscope and an accelerometer, but no magnetometer.
Consequently roll and pitch can be corrected against gravity while yaw must be
integrated from the (calibrated) gyroscope.  The Mahony-style feedback below
keeps the integration on SO(3), avoiding the axis coupling and saturation that
result from estimating yaw from a single rotated-vector component.
"""

import math
import threading

import numpy as np
from scipy.spatial.transform import Rotation


class MahonyAttitudeEstimator:
    """Six-axis Mahony filter with continuous, zeroable yaw.

    ``gyro_in_rad`` is expected in radians/second and ``accel_in_g`` in units of
    gravity.  Joy-Con sensor axes are converted to the control convention used
    by the original package:

    * output roll rate  = ``-gyro_x``
    * output pitch rate = ``+gyro_y``
    * output yaw rate   = ``-gyro_z``

    The same proper rotation (180 degrees around Y) is applied to acceleration,
    so a horizontally resting controller measures world-up as ``[0, 0, 1]``.
    """

    _SENSOR_TO_CONTROL = np.array([-1.0, 1.0, -1.0])
    _WORLD_UP = np.array([0.0, 0.0, 1.0])

    def __init__(
            self, sample_period=0.005, proportional_gain=2.5,
            integral_gain=0.05, accel_rejection=0.25,
            integral_limit=0.25):
        self.sample_period = float(sample_period)
        self.proportional_gain = float(proportional_gain)
        self.integral_gain = float(integral_gain)
        self.accel_rejection = float(accel_rejection)
        self.integral_limit = float(integral_limit)
        if not np.isfinite(self.sample_period) or self.sample_period <= 0.0:
            raise ValueError("sample_period must be finite and positive")
        if (not np.isfinite(self.proportional_gain)
                or self.proportional_gain < 0.0):
            raise ValueError("proportional_gain must be finite and non-negative")
        if not np.isfinite(self.integral_gain) or self.integral_gain < 0.0:
            raise ValueError("integral_gain must be finite and non-negative")
        if not np.isfinite(self.accel_rejection) or self.accel_rejection <= 0.0:
            raise ValueError("accel_rejection must be finite and positive")
        if not np.isfinite(self.integral_limit) or self.integral_limit < 0.0:
            raise ValueError("integral_limit must be finite and non-negative")

        self._lock = threading.RLock()
        self._yaw_adjustment = 0.0
        self.reset()

    @staticmethod
    def _wrap_pi(angle):
        return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi

    def reset(self):
        """Reset the full estimate to identity.

        The next valid accelerometer sample initializes roll and pitch directly
        from gravity.  This method is intended for the two-second, level-table
        calibration sequence.
        """
        with self._lock:
            self._rotation = Rotation.identity()
            self._integral_correction = np.zeros(3)
            self._initialized = False
            self._last_wrapped_yaw = 0.0
            self._continuous_yaw = 0.0
            self._yaw_zero = 0.0

    def reset_yaw(self):
        """Make the current heading zero without disturbing roll or pitch."""
        with self._lock:
            self._yaw_zero = self._continuous_yaw
            # Gravity cannot observe heading, so retaining a Z integral term
            # after an explicit zero operation only encourages unwanted drift.
            self._integral_correction[2] = 0.0

    def set_yaw_diff(self, value):
        """Retain the vendored library's optional stick-based yaw offset."""
        value = float(value)
        if not np.isfinite(value):
            raise ValueError("yaw offset must be finite")
        with self._lock:
            self._yaw_adjustment = value

    def _initialize_from_acceleration(self, acceleration):
        norm = float(np.linalg.norm(acceleration))
        if norm <= 1e-9:
            return False
        up_body = acceleration / norm
        roll = math.atan2(up_body[1], up_body[2])
        pitch = math.atan2(
            -up_body[0], math.hypot(up_body[1], up_body[2]))
        self._rotation = Rotation.from_euler("xyz", [roll, pitch, 0.0])
        self._last_wrapped_yaw = 0.0
        self._continuous_yaw = 0.0
        self._yaw_zero = 0.0
        self._initialized = True
        return True

    def _update_continuous_yaw(self):
        wrapped_yaw = float(self._rotation.as_euler("xyz")[2])
        delta = self._wrap_pi(wrapped_yaw - self._last_wrapped_yaw)
        self._continuous_yaw += delta
        self._last_wrapped_yaw = wrapped_yaw

    def update(self, gyro_in_rad, accel_in_g, dt=None):
        """Fuse one IMU sample and return ``[roll, pitch, yaw]`` in radians."""
        gyro = np.asarray(gyro_in_rad, dtype=float)
        accel = np.asarray(accel_in_g, dtype=float)
        if gyro.shape != (3,) or accel.shape != (3,):
            raise ValueError("gyro and acceleration samples must each have three values")
        if not np.all(np.isfinite(gyro)) or not np.all(np.isfinite(accel)):
            return self.get_euler()

        dt = self.sample_period if dt is None else float(dt)
        if not np.isfinite(dt) or dt <= 0.0:
            raise ValueError("dt must be finite and positive")

        gyro = gyro * self._SENSOR_TO_CONTROL
        acceleration = accel * self._SENSOR_TO_CONTROL
        acceleration_norm = float(np.linalg.norm(acceleration))

        with self._lock:
            if not self._initialized:
                if not self._initialize_from_acceleration(acceleration):
                    return self._get_euler_unlocked()

            correction = np.zeros(3)
            deviation = abs(acceleration_norm - 1.0)
            if acceleration_norm > 1e-9 and deviation < self.accel_rejection:
                # Reject linear acceleration smoothly instead of interpreting
                # it as tilt. At exactly 1 g the correction has full authority.
                trust = 1.0 - deviation / self.accel_rejection
                measured_up_body = acceleration / acceleration_norm
                predicted_up_body = self._rotation.inv().apply(self._WORLD_UP)
                gravity_error = np.cross(
                    measured_up_body, predicted_up_body)

                if self.integral_gain > 0.0:
                    self._integral_correction += (
                        self.integral_gain * trust * gravity_error * dt)
                    self._integral_correction = np.clip(
                        self._integral_correction,
                        -self.integral_limit, self.integral_limit)
                correction = (
                    self.proportional_gain * trust * gravity_error
                    + self._integral_correction)

            angular_velocity = gyro + correction
            self._rotation = (
                self._rotation
                * Rotation.from_rotvec(angular_velocity * dt))
            self._update_continuous_yaw()
            return self._get_euler_unlocked()

    def _get_euler_unlocked(self):
        roll, pitch, _ = self._rotation.as_euler("xyz")
        yaw = (
            self._continuous_yaw
            - self._yaw_zero
            - self._yaw_adjustment)
        return np.array([roll, pitch, yaw], dtype=float)

    def get_euler(self):
        """Return a thread-safe snapshot of roll, pitch and continuous yaw."""
        with self._lock:
            return self._get_euler_unlocked()

    def get_quaternion(self):
        """Return the zeroed output attitude as an ``[x, y, z, w]`` quaternion."""
        with self._lock:
            return Rotation.from_euler(
                "xyz", self._get_euler_unlocked()).as_quat()
