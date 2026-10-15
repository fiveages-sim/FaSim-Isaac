"""4-wheel steer inverse kinematics used by the Linkhou S2 and Taku chassis graphs.

State stays on the instance. A ScriptNode kept the same values in module globals,
which is what made Isaac disable every OmniGraph until the user opted in.
"""

from __future__ import annotations

import numpy as np

PROFILE_LINKHOU_S2_V1 = "linkhou_s2_v1"
PROFILE_TAKU = "taku"

_SPEED_EPS = 1e-6

_LINKHOU_WHEELS = np.array(
    [
        [0.23072, 0.21738],
        [-0.22928, 0.21738],
        [-0.23072, -0.21738],
        [0.22928, -0.21738],
    ]
)
_LINKHOU_STEER_JOINTS = (
    "fl_steer_joint",
    "rl_steer_joint",
    "rr_steer_joint",
    "fr_steer_joint",
)
_LINKHOU_WHEEL_JOINTS = (
    "fl_wheel_joint",
    "rl_wheel_joint",
    "rr_wheel_joint",
    "fr_wheel_joint",
)

_TAKU_WHEELS = np.array(
    [
        [0.2745, 0.2045],
        [-0.2745, 0.2045],
        [-0.2745, -0.2045],
        [0.2745, -0.2045],
    ]
)
_TAKU_STEER_JOINTS = (
    "agv_left_front_steering_joint",
    "agv_left_back_steering_joint",
    "agv_right_back_steering_joint",
    "agv_right_front_steering_joint",
)
_TAKU_WHEEL_JOINTS = (
    "agv_left_front_rotate_joint",
    "agv_left_back_rotate_joint",
    "agv_right_back_rotate_joint",
    "agv_right_front_rotate_joint",
)

_TAKU_WHEEL_RADIUS = 0.0625
_TAKU_CMD_LIN_DEADBAND = 0.02
_TAKU_CMD_ANG_DEADBAND = 0.05
_TAKU_CMD_TIMEOUT_S = 0.25
_TAKU_STEER_SLEW_RAD_S = np.deg2rad(180.0)
_TAKU_ALIGN_FULL_RAD = np.deg2rad(8.0)
_TAKU_ALIGN_ZERO_RAD = np.deg2rad(35.0)
_TAKU_FLIP_HYST_RAD = np.deg2rad(20.0)


def _wrap_to_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def _angle_diff(target, reference):
    return _wrap_to_pi(target - reference)


def _pick_steer_and_wheel_speed(raw_angle, speed, ref_angle, flip_hysteresis=0.0):
    a = _wrap_to_pi(raw_angle)
    va = speed
    b = _wrap_to_pi(raw_angle + np.pi)
    vb = -speed
    da = abs(_angle_diff(a, ref_angle))
    db = abs(_angle_diff(b, ref_angle))
    if db < da - flip_hysteresis:
        return b, vb
    return a, va


def _align_scale(err):
    amount = abs(err)
    if amount <= _TAKU_ALIGN_FULL_RAD:
        return 1.0
    if amount >= _TAKU_ALIGN_ZERO_RAD:
        return 0.0
    return (_TAKU_ALIGN_ZERO_RAD - amount) / (_TAKU_ALIGN_ZERO_RAD - _TAKU_ALIGN_FULL_RAD)


class SwerveMemory:
    def __init__(self):
        self.reset()

    def reset(self):
        self.prev_pos_cmd = np.zeros(4)
        self.prev_vel_unscaled = np.zeros(4)
        self.steer_actual = np.zeros(4)
        self.has_prev_steer = False
        self.last_msg_count = -1
        self.since_msg = 0.0
        self.held_vx = 0.0
        self.held_vy = 0.0
        self.held_omega = 0.0


def compute_swerve(
    memory: SwerveMemory,
    profile: str,
    vx: float,
    vy: float,
    omega_cmd: float,
    linear_gain: float,
    angular_gain: float,
    flip_vel_scale: float,
    dt: float = 0.0,
    msg_count: int = 0,
):
    """Return steer positions, wheel velocities, steer joint names, wheel joint names."""
    if profile == PROFILE_LINKHOU_S2_V1:
        pos_cmd, vel_cmd = _compute_linkhou(
            memory, vx, vy, omega_cmd, linear_gain, angular_gain, flip_vel_scale
        )
        return pos_cmd, vel_cmd, _LINKHOU_STEER_JOINTS, _LINKHOU_WHEEL_JOINTS
    if profile == PROFILE_TAKU:
        pos_cmd, vel_cmd = _compute_taku(
            memory, vx, vy, omega_cmd, linear_gain, angular_gain, flip_vel_scale, dt, msg_count
        )
        return pos_cmd, vel_cmd, _TAKU_STEER_JOINTS, _TAKU_WHEEL_JOINTS
    raise ValueError(f"unknown swerve profile {profile}")


def _compute_linkhou(memory, vx, vy, omega_cmd, linear_gain, angular_gain, flip_vel_scale):
    flip_vel_scale = max(0.0, min(1.0, float(flip_vel_scale)))
    pos_cmd = np.zeros(4)
    vel_cmd = np.zeros(4)
    omega = float(omega_cmd) * float(angular_gain)
    vx = float(vx)
    vy = float(vy)

    for i in range(4):
        x_i, y_i = _LINKHOU_WHEELS[i]
        vx_total = vx + (-omega * y_i)
        vy_total = vy + (omega * x_i)
        angle = np.arctan2(vy_total, vx_total)
        speed = np.linalg.norm([vx_total, vy_total])
        if speed < _SPEED_EPS:
            pos_cmd[i] = memory.prev_pos_cmd[i] if memory.has_prev_steer else 0.0
            vel_cmd[i] = 0.0
        else:
            ref = memory.prev_pos_cmd[i] if memory.has_prev_steer else 0.0
            pos_cmd[i], vel_cmd[i] = _pick_steer_and_wheel_speed(angle, speed, ref)
            if memory.has_prev_steer:
                previous = memory.prev_vel_unscaled[i]
                current = vel_cmd[i]
                if (
                    abs(previous) > _SPEED_EPS
                    and abs(current) > _SPEED_EPS
                    and previous * current < 0
                    and flip_vel_scale < 1.0
                ):
                    vel_cmd[i] *= flip_vel_scale

    vel_cmd[:] *= float(linear_gain)
    if np.linalg.norm([vx, vy]) < 0.0001 and abs(float(omega_cmd)) < 0.0001:
        pos_cmd[:] = 0
        vel_cmd[:] = 0

    inv_gain = 1.0 / float(linear_gain) if float(linear_gain) != 0.0 else 0.0
    memory.prev_vel_unscaled = vel_cmd * inv_gain
    memory.prev_pos_cmd = pos_cmd.copy()
    memory.has_prev_steer = True
    return pos_cmd, vel_cmd


def _compute_taku(memory, vx_in, vy_in, omega_in, linear_gain, angular_gain, flip_vel_scale, dt, msg_count):
    flip_vel_scale = max(0.0, min(1.0, float(flip_vel_scale)))
    dt = float(dt or 0.0)
    if dt <= 1e-6 or dt > 0.1:
        dt = 1.0 / 60.0
    msg_count = int(msg_count or 0)
    vx_in = float(vx_in)
    vy_in = float(vy_in)
    omega_in = float(omega_in)

    if msg_count != memory.last_msg_count:
        memory.last_msg_count = msg_count
        memory.since_msg = 0.0
        memory.held_vx = vx_in
        memory.held_vy = vy_in
        memory.held_omega = omega_in
    else:
        memory.since_msg += dt

    if memory.last_msg_count < 0 or memory.since_msg > _TAKU_CMD_TIMEOUT_S:
        vx = 0.0
        vy = 0.0
        omega_cmd = 0.0
    else:
        vx = memory.held_vx
        vy = memory.held_vy
        omega_cmd = memory.held_omega

    if np.linalg.norm([vx, vy]) < _TAKU_CMD_LIN_DEADBAND and abs(omega_cmd) < _TAKU_CMD_ANG_DEADBAND:
        vx = 0.0
        vy = 0.0
        omega_cmd = 0.0

    omega = omega_cmd * float(angular_gain)
    pos_cmd = np.zeros(4)
    vel_cmd = np.zeros(4)
    stopped = abs(vx) < _SPEED_EPS and abs(vy) < _SPEED_EPS and abs(omega_cmd) < _SPEED_EPS

    for i in range(4):
        x_i, y_i = _TAKU_WHEELS[i]
        vx_total = vx - omega * y_i
        vy_total = vy + omega * x_i
        angle = np.arctan2(vy_total, vx_total)
        speed = np.linalg.norm([vx_total, vy_total])
        if stopped or speed < _SPEED_EPS:
            pos_cmd[i] = memory.steer_actual[i] if memory.has_prev_steer else 0.0
            vel_cmd[i] = 0.0
        else:
            ref = memory.steer_actual[i] if memory.has_prev_steer else 0.0
            pos_cmd[i], vel_cmd[i] = _pick_steer_and_wheel_speed(
                angle, speed, ref, flip_hysteresis=_TAKU_FLIP_HYST_RAD
            )
            if memory.has_prev_steer:
                previous = memory.prev_vel_unscaled[i]
                current = vel_cmd[i]
                if (
                    abs(previous) > _SPEED_EPS
                    and abs(current) > _SPEED_EPS
                    and previous * current < 0
                    and flip_vel_scale < 1.0
                ):
                    vel_cmd[i] *= flip_vel_scale

    ik_vel = vel_cmd.copy()
    max_step = _TAKU_STEER_SLEW_RAD_S * dt
    aligns = np.ones(4)
    for i in range(4):
        err = _angle_diff(pos_cmd[i], memory.steer_actual[i])
        memory.steer_actual[i] = _wrap_to_pi(memory.steer_actual[i] + np.clip(err, -max_step, max_step))
        aligns[i] = _align_scale(_angle_diff(pos_cmd[i], memory.steer_actual[i]))
    if stopped:
        vel_cmd[:] = 0.0
    else:
        vel_cmd *= float(np.min(aligns))
        vel_cmd[:] *= float(linear_gain)

    memory.prev_vel_unscaled = ik_vel
    memory.prev_pos_cmd = pos_cmd.copy()
    memory.has_prev_steer = True
    return memory.steer_actual.copy(), vel_cmd
