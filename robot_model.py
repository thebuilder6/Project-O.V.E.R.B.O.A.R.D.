import numpy as np


class RobotConfig:
    """
    Robot configuration parsed from Choreo-like JSON config file.

    Default values are typical for a small FLL robot with LEGO motors.
    """

    # Default values for typical FLL robot
    DEFAULT_MASS = 0.723  # kg
    DEFAULT_INERTIA = 0.0024  # kg·m²
    DEFAULT_TRACK_WIDTH = 0.0965  # m (96.5 mm)
    DEFAULT_WHEEL_RADIUS = 0.028  # m (28 mm)
    DEFAULT_VMAX = 15.7  # rad/s (150 RPM)
    DEFAULT_TMAX = 0.04  # N·m
    DEFAULT_GEARING = 1.0
    DEFAULT_COF = 0.45  # coefficient of friction
    GRAVITY = 9.81  # m/s²
    DEFAULT_TORQUE_HEADROOM = 0.85  # 15% headroom for torque corrections
    DEFAULT_SPEED_HEADROOM = 0.90  # 10% headroom for speed corrections

    def __init__(self, config_dict):
        """
        Initialize robot configuration from config dictionary.

        Args:
            config_dict: Dictionary containing robot configuration in Choreo or JSON format
        """
        # Detect format: new JSON format has "robot" key, old Choreo format has "config" key
        if "robot" in config_dict:
            # New JSON format
            robot_cfg = config_dict.get("robot", {})
            self.mass = robot_cfg.get("mass", self.DEFAULT_MASS)
            self.inertia = robot_cfg.get("inertia", self.DEFAULT_INERTIA)
            self.track_width = robot_cfg.get("track_width", self.DEFAULT_TRACK_WIDTH)
            self.wheel_radius = robot_cfg.get("wheel_radius", self.DEFAULT_WHEEL_RADIUS)
            self.v_max_rad_s = robot_cfg.get("v_max_rad_s", self.DEFAULT_VMAX)
            self.t_max_nm = robot_cfg.get("t_max_nm", self.DEFAULT_TMAX)
            self.gearing = robot_cfg.get("gearing", self.DEFAULT_GEARING)
            self.cof = robot_cfg.get("cof", self.DEFAULT_COF)
            self.g = robot_cfg.get("gravity", self.GRAVITY)
            self.torque_headroom = robot_cfg.get("torque_headroom", self.DEFAULT_TORQUE_HEADROOM)
            self.speed_headroom = robot_cfg.get("speed_headroom", self.DEFAULT_SPEED_HEADROOM)
            
            # Store multiverse config if present
            self.multiverse_config = config_dict.get("multiverse", {})
        else:
            # Old Choreo format
            cfg = config_dict.get("config", {})
            self.mass = cfg.get("mass", {}).get("val", self.DEFAULT_MASS)
            self.inertia = cfg.get("inertia", {}).get("val", self.DEFAULT_INERTIA)
            self.track_width = cfg.get("differentialTrackWidth", {}).get("val", self.DEFAULT_TRACK_WIDTH)
            self.wheel_radius = cfg.get("radius", {}).get("val", self.DEFAULT_WHEEL_RADIUS)
            self.v_max_rad_s = cfg.get("vmax", {}).get("val", self.DEFAULT_VMAX)
            self.t_max_nm = cfg.get("tmax", {}).get("val", self.DEFAULT_TMAX)
            self.gearing = cfg.get("gearing", {}).get("val", self.DEFAULT_GEARING)
            self.cof = cfg.get("cof", {}).get("val", self.DEFAULT_COF)
            self.g = self.GRAVITY
            self.torque_headroom = cfg.get("torqueHeadroom", {}).get("val", self.DEFAULT_TORQUE_HEADROOM)
            self.speed_headroom = cfg.get("speedHeadroom", {}).get("val", self.DEFAULT_SPEED_HEADROOM)
            
            # No multiverse config in old format
            self.multiverse_config = {}

    def get_max_force_at_velocity(self, v_wheel, apply_headroom=True):
        """
        Calculates max force magnitude a motor can apply at a given wheel velocity.

        Uses a linear motor curve (symmetric braking/driving limit):
        - At v=0: max force = t_max / wheel_radius
        - At v=v_max: max force = 0

        Args:
            v_wheel: Wheel velocity in m/s
            apply_headroom: If True, applies safety margin for real-world tracking

        Returns:
            Maximum force in Newtons
        """
        omega = (v_wheel / self.wheel_radius) * self.gearing
        torque = self.t_max_nm * (1.0 - abs(omega) / self.v_max_rad_s)
        torque = max(0, torque)
        force = (torque / self.wheel_radius) * self.gearing
        if apply_headroom:
            force *= self.torque_headroom
        return force

    def max_linear_speed(self, apply_headroom=True):
        """No-load linear speed of the wheel (m/s)."""
        speed = self.v_max_rad_s * self.wheel_radius
        if apply_headroom:
            speed *= self.speed_headroom
        return speed


class DifferentialDriveModel:
    """
    Differential drive robot dynamics model.

    Calculates required wheel forces for given motion and checks
    physical constraints (motor limits, traction limits).
    """

    def __init__(self, config: RobotConfig):
        """
        Initialize dynamics model with robot configuration.

        Args:
            config: RobotConfig object containing physical parameters
        """
        self.cfg = config

    def get_dynamics(self, vl, vr, al, ar):
        """
        Calculate required wheel forces for given velocities and accelerations.

        Uses differential drive kinematics:
        - Linear acceleration: a = (al + ar) / 2
        - Angular acceleration: alpha = (ar - al) / track_width

        Args:
            vl: Left wheel velocity (m/s)
            vr: Right wheel velocity (m/s)
            al: Left wheel acceleration (m/s²)
            ar: Right wheel acceleration (m/s²)

        Returns:
            Tuple of (fl, fr) - Left and right wheel forces (N)
        """
        # Linear and angular acceleration
        a = (al + ar) / 2.0
        alpha = (ar - al) / self.cfg.track_width

        # Required total force and moment
        f_total = self.cfg.mass * a
        m_total = self.cfg.inertia * alpha

        # Solve for individual wheel forces
        # f_total = fl + fr
        # m_total = (fr - fl) * (track_width / 2)
        fr = (f_total + (2.0 * m_total / self.cfg.track_width)) / 2.0
        fl = f_total - fr

        return fl, fr

    def get_wheel_normal_forces(self, vl, vr, al, ar):
        """
        Calculate normal force on each wheel considering weight transfer.

        During acceleration and turning, weight shifts between wheels:
        - Longitudinal acceleration: weight shifts to rear wheels
        - Lateral acceleration (turning): weight shifts to outer wheels

        Args:
            vl: Left wheel velocity (m/s)
            vr: Right wheel velocity (m/s)
            al: Left wheel acceleration (m/s²)
            ar: Right wheel acceleration (m/s²)

        Returns:
            Tuple of (nl, nr) - Normal forces on left and right wheels (N)
        """
        # Linear and angular acceleration
        a = (al + ar) / 2.0
        alpha = (ar - al) / self.cfg.track_width

        # Base static weight distribution (assume 50/50)
        base_normal = (self.cfg.mass * self.cfg.g) / 2.0

        # Longitudinal weight transfer (during acceleration/deceleration)
        # Assuming center of mass height h_cg (estimate as 50mm for FLL robot)
        h_cg = 0.05  # meters
        wheelbase = self.cfg.track_width  # approximate wheelbase as track width
        longitudinal_transfer = (self.cfg.mass * a * h_cg) / wheelbase

        # Lateral weight transfer (during turning)
        # Assuming center of mass at track center
        lateral_transfer = (self.cfg.mass * (a * 0) * h_cg) / self.cfg.track_width  # No lateral force from pure differential drive

        # For differential drive, lateral force comes from centripetal acceleration during turns
        v = (vl + vr) / 2.0
        omega = (vr - vl) / self.cfg.track_width
        centripetal_accel = v * omega
        lateral_transfer = (self.cfg.mass * centripetal_accel * h_cg) / self.cfg.track_width

        # Normal forces with weight transfer
        # During acceleration, rear wheels get more load (assuming forward motion)
        # During turning, outer wheel gets more load
        nl = base_normal - longitudinal_transfer - lateral_transfer
        nr = base_normal - longitudinal_transfer + lateral_transfer

        return nl, nr

    def check_constraints(self, vl, vr, al, ar, apply_headroom=True):
        """
        Check if given state violates motor or traction limits.

        Args:
            vl: Left wheel velocity (m/s)
            vr: Right wheel velocity (m/s)
            al: Left wheel acceleration (m/s²)
            ar: Right wheel acceleration (m/s²)
            apply_headroom: If True, applies safety margin for real-world tracking

        Returns:
            Dictionary with violation details:
            - left_motor_violation: N (0 if no violation)
            - right_motor_violation: N (0 if no violation)
            - left_wheel_slip: N (0 if no slip)
            - right_wheel_slip: N (0 if no slip)
            - traction_violation: N (0 if no violation, legacy total check)
        """
        fl, fr = self.get_dynamics(vl, vr, al, ar)

        # Motor force limits
        fl_max = self.cfg.get_max_force_at_velocity(vl, apply_headroom)
        fr_max = self.cfg.get_max_force_at_velocity(vr, apply_headroom)

        left_motor_violation = max(0, abs(fl) - fl_max)
        right_motor_violation = max(0, abs(fr) - fr_max)

        # Individual wheel slip detection with weight transfer
        nl, nr = self.get_wheel_normal_forces(vl, vr, al, ar)
        left_traction_limit = self.cfg.cof * nl
        right_traction_limit = self.cfg.cof * nr

        left_wheel_slip = max(0, abs(fl) - left_traction_limit)
        right_wheel_slip = max(0, abs(fr) - right_traction_limit)

        # Legacy total traction check (friction circle)
        traction_limit = self.cfg.cof * self.cfg.mass * self.cfg.g
        traction_violation = max(0, abs(fl) + abs(fr) - traction_limit)

        return {
            "left_motor_violation": left_motor_violation,
            "right_motor_violation": right_motor_violation,
            "left_wheel_slip": left_wheel_slip,
            "right_wheel_slip": right_wheel_slip,
            "traction_violation": traction_violation,
            "left_normal_force": nl,
            "right_normal_force": nr
        }
