"""Geometry, target storage and docking control; no ROS or motor imports."""

from dataclasses import dataclass
import math
import os
from pathlib import Path
import tempfile

import numpy as np
import yaml


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_matrix(quaternion):
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)) or np.linalg.norm(q) < 1e-8:
        raise ValueError('Invalid quaternion')
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


def matrix_quaternion(rotation):
    # Eigenvector form is well behaved around 180 degrees (a front-facing tag).
    r = np.asarray(rotation, dtype=float)
    k = np.array([
        [r[0, 0]-r[1, 1]-r[2, 2], r[0, 1]+r[1, 0], r[0, 2]+r[2, 0], r[2, 1]-r[1, 2]],
        [r[0, 1]+r[1, 0], r[1, 1]-r[0, 0]-r[2, 2], r[1, 2]+r[2, 1], r[0, 2]-r[2, 0]],
        [r[0, 2]+r[2, 0], r[1, 2]+r[2, 1], r[2, 2]-r[0, 0]-r[1, 1], r[1, 0]-r[0, 1]],
        [r[2, 1]-r[1, 2], r[0, 2]-r[2, 0], r[1, 0]-r[0, 1], np.trace(r)],
    ]) / 3.0
    _, vectors = np.linalg.eigh(k)
    q = vectors[:, -1]
    return q if q[3] >= 0 else -q


def transform(position, quaternion):
    t = np.eye(4)
    t[:3, :3] = quaternion_matrix(quaternion)
    t[:3, 3] = position
    return validate_transform(t)


def validate_transform(value):
    t = np.asarray(value, dtype=float)
    if (t.shape != (4, 4) or not np.all(np.isfinite(t))
            or not np.allclose(t[3], [0, 0, 0, 1], atol=1e-6)
            or not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(t[:3, :3]), 1.0, atol=1e-5)):
        raise ValueError('Expected a finite rigid 4x4 transform')
    return t.copy()


def inverse(t):
    result = np.eye(4)
    result[:3, :3] = t[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ t[:3, 3]
    return result


def rotation_distance(a, b):
    return math.acos(float(np.clip((np.trace(a.T @ b) - 1) / 2, -1, 1)))


def average_transforms(samples, translation_spread, rotation_spread):
    if not samples:
        raise ValueError('No teaching samples')
    samples = [validate_transform(t) for t in samples]
    mean = np.eye(4)
    mean[:3, 3] = np.mean([t[:3, 3] for t in samples], axis=0)
    u, _, vt = np.linalg.svd(np.mean([t[:3, :3] for t in samples], axis=0))
    mean[:3, :3] = u @ np.diag([1, 1, np.linalg.det(u @ vt)]) @ vt
    if any(np.linalg.norm(t[:3, 3] - mean[:3, 3]) > translation_spread
           or rotation_distance(t[:3, :3], mean[:3, :3]) > rotation_spread
           for t in samples):
        raise ValueError('Teaching observations are unstable; stop the robot and try again')
    return mean


def target_error(current_marker, goal_marker):
    # C<-G = (C<-M) (G<-M)^-1. Includes camera offsets and tag orientation.
    delta = current_marker @ inverse(goal_marker)
    return np.array([delta[0, 3], delta[1, 3], math.atan2(delta[1, 0], delta[0, 0])])


def save_target(path, matrix, context, base_frame):
    if not context or not base_frame:
        raise ValueError('Missing detector context or base frame')
    data = {'version': 1, 'detector_context': context, 'base_frame': base_frame,
            'base_from_marker': validate_transform(matrix).tolist()}
    path = Path(path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    name = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, delete=False) as stream:
            name = stream.name
            yaml.safe_dump(data, stream)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(name, path)
    finally:
        if name and os.path.exists(name):
            os.unlink(name)


def load_target(path, base_frame):
    try:
        with Path(path).expanduser().open() as stream:
            data = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        raise ValueError('Malformed target YAML; teach again') from exc
    if (not isinstance(data, dict) or data.get('version') != 1
            or data.get('base_frame') != base_frame
            or not isinstance(data.get('detector_context'), str)
            or not data['detector_context']):
        raise ValueError('Invalid target version, frame or detector context; teach again')
    return validate_transform(data['base_from_marker']), data['detector_context']


@dataclass
class DockingSettings:
    pose_command_timeout: float = 0.15
    pose_timeout: float = 0.75
    approach_timeout: float = 120.0
    progress_timeout: float = 8.0
    progress_distance: float = 0.004
    settle_time: float = 0.6
    # Mechanical calibration: positive values move the desired robot pose to
    # the robot's right relative to the taught pose.
    target_right_offset: float = 0.0
    # Optional closed-loop push after the taught pose has settled. Keep the
    # core default disabled; the robot configuration explicitly opts in.
    extra_insert_distance: float = 0.0
    extra_insert_speed: float = 0.025
    extra_insert_tolerance: float = 0.001
    extra_insert_timeout: float = 2.0
    tolerance_x: float = 0.010
    tolerance_y: float = 0.005
    tolerance_yaw: float = 0.025
    align_y: float = 0.015
    align_yaw: float = 0.05
    fine_align_distance: float = 0.10
    insertion_distance: float = 0.05
    far_stop_y: float = 0.08
    far_stop_yaw: float = 0.20
    insertion_y: float = 0.005
    insertion_yaw: float = 0.025
    insertion_abort_y: float = 0.025
    insertion_abort_yaw: float = 0.10
    max_start_distance: float = 1.5
    max_start_yaw: float = 0.7
    max_pose_jump: float = 0.12
    max_rotation_jump: float = 0.3
    max_vertical_error: float = 0.04
    far_max_vertical_error: float = 0.08
    max_tilt_error: float = 0.15
    max_forward: float = 0.04
    near_max_forward: float = 0.028
    insertion_max_forward: float = 0.025
    min_forward: float = 0.025
    max_lateral: float = 0.04
    min_lateral: float = 0.025
    max_yaw_rate: float = 0.12
    min_heading_yaw_rate: float = 0.10
    gain_x: float = 0.4
    gain_y: float = 1.0
    gain_yaw: float = 0.6

    def __post_init__(self):
        positive_settings = {
            name: value for name, value in vars(self).items()
            if name not in {'extra_insert_distance', 'target_right_offset'}
        }
        if any(not math.isfinite(v) or v <= 0 for v in positive_settings.values()):
            raise ValueError('All docking settings must be finite and positive')
        if (not math.isfinite(self.extra_insert_distance)
                or self.extra_insert_distance < 0):
            raise ValueError('extra_insert_distance must be finite and non-negative')
        if not math.isfinite(self.target_right_offset) or self.target_right_offset < 0:
            raise ValueError('target_right_offset must be finite and non-negative')
        if (self.extra_insert_distance > 0
                and self.extra_insert_tolerance >= self.extra_insert_distance):
            raise ValueError('extra_insert_tolerance must be smaller than the extra distance')
        if self.align_y < self.tolerance_y or self.align_yaw < self.tolerance_yaw:
            raise ValueError('Alignment thresholds must be at least the final tolerances')
        if not (self.fine_align_distance > self.insertion_distance > self.tolerance_x):
            raise ValueError('Expected fine_align_distance > insertion_distance > tolerance_x')
        if self.max_start_distance <= self.fine_align_distance:
            raise ValueError('max_start_distance must exceed fine_align_distance')
        if self.far_stop_y <= self.align_y or self.far_stop_yaw <= self.align_yaw:
            raise ValueError('Far stop thresholds must exceed the fine alignment thresholds')
        if not (self.tolerance_y <= self.insertion_y <= self.align_y
                and self.tolerance_yaw <= self.insertion_yaw <= self.align_yaw):
            raise ValueError('Insertion thresholds must lie between final and fine thresholds')
        if (self.insertion_abort_y <= self.insertion_y
                or self.insertion_abort_yaw <= self.insertion_yaw):
            raise ValueError('Insertion abort thresholds must exceed insertion thresholds')
        if not (self.min_forward <= self.insertion_max_forward
                <= self.near_max_forward <= self.max_forward):
            raise ValueError('Forward speed limits must increase from insertion to far approach')
        if not self.min_forward <= self.extra_insert_speed <= self.insertion_max_forward:
            raise ValueError('extra_insert_speed must be within the effective insertion limits')
        if self.min_lateral > self.max_lateral:
            raise ValueError('min_lateral cannot exceed max_lateral')
        if self.min_heading_yaw_rate > self.max_yaw_rate:
            raise ValueError('min_heading_yaw_rate cannot exceed max_yaw_rate')
        if self.far_max_vertical_error < self.max_vertical_error:
            raise ValueError('Far vertical limit must be at least the near vertical limit')
        if self.pose_command_timeout > self.pose_timeout:
            raise ValueError('pose_command_timeout cannot exceed pose_timeout')


class DockingControl:
    def __init__(self, settings=None):
        self.settings = settings or DockingSettings()
        self.target = None
        self.observation = None
        self.observed_at = None
        self.active = False
        self.state = 'IDLE'
        self.reason = 'Waiting for an explicit start command'
        self.error = None
        self._settling_since = None
        self._inserting = False
        self._extra_target = None
        self._extra_started_at = None
        self._coarse_heading_first = False
        self._previous_command_axes = np.zeros(3, dtype=bool)

    def stop(self, state, reason):
        self.active = False
        self.state = state
        self.reason = reason
        self._settling_since = None

    def observe(self, matrix, now):
        matrix = validate_transform(matrix)
        if self.active and self.observation is not None:
            dt = now - self.observed_at
            if dt <= self.settings.pose_timeout and (
                    np.linalg.norm(matrix[:3, 3] - self.observation[:3, 3]) > self.settings.max_pose_jump
                    or rotation_distance(matrix[:3, :3], self.observation[:3, :3])
                    > self.settings.max_rotation_jump):
                self.stop('FAILED', 'Detected pose jumped; reacquire and start again')
                return False
        self.observation = matrix
        self.observed_at = now
        return True

    def start(self, target, now):
        target = validate_transform(target)
        if self.settings.target_right_offset > 0:
            # The taught marker pose is expressed in the taught base frame.
            # Moving the desired base to its right (-Y) makes the marker appear
            # farther left (+Y) in the new base frame.
            right_compensation = np.eye(4)
            right_compensation[1, 3] = self.settings.target_right_offset
            target = right_compensation @ target
        if self.observed_at is None or now - self.observed_at > self.settings.pose_timeout:
            raise ValueError('No fresh marker observation')
        error = target_error(self.observation, target)
        self.check_planar_target(target)
        if np.linalg.norm(error[:2]) > self.settings.max_start_distance:
            raise ValueError('Target is outside the allowed starting distance')
        if abs(error[2]) > self.settings.max_start_yaw:
            raise ValueError('Starting angle is too large; position the robot in front of the dock')
        if error[0] < -self.settings.tolerance_x:
            raise ValueError('Target is behind the robot; reverse manually and start again')
        self.target = target
        self.active = True
        self.state = 'ALIGNING'
        self.reason = (
            f'Aligning to the taught target with '
            f'{1000.0*self.settings.target_right_offset:.1f} mm right compensation'
        )
        self._started_at = now
        self._progress_at = now
        self._best_progress_error = np.abs(error) * [1.0, 1.0, 0.25]
        self._settling_since = None
        self._inserting = False
        self._extra_target = None
        self._extra_started_at = None
        self._coarse_heading_first = False
        self._previous_command_axes[:] = False

    def check_planar_target(self, target):
        """Reject motion requiring height/tilt changes; also usable while waiting."""
        delta = self.observation @ inverse(target)
        height = float(delta[2, 3])
        tilt = math.acos(float(np.clip(delta[2, 2], -1, 1)))
        planar_distance = float(np.linalg.norm(delta[:2, 3]))
        # ArUco's height estimate is noisier at long range and large lateral
        # offsets. Relax only that consistency check with distance, then
        # return continuously to the original 4 cm limit at fine approach.
        distance_span = self.settings.max_start_distance - self.settings.fine_align_distance
        relaxation = float(np.clip(
            (planar_distance-self.settings.fine_align_distance) / distance_span,
            0.0, 1.0))
        height_limit = (self.settings.max_vertical_error
                        + relaxation*(self.settings.far_max_vertical_error
                                      - self.settings.max_vertical_error))
        if abs(height) > height_limit or tilt > self.settings.max_tilt_error:
            raise ValueError(
                f'Target has excessive height/tilt error: height={height:+.3f} m '
                f'(limit {height_limit:.3f} m at planar distance '
                f'{planar_distance:.3f} m), '
                f'tilt={math.degrees(tilt):.2f} deg '
                f'(limit {math.degrees(self.settings.max_tilt_error):.2f} deg); '
                'check marker pose and teaching')

    def tick(self, now):
        zero = np.zeros(3)
        if not self.active:
            return zero
        s = self.settings
        pose_age = math.inf if self.observed_at is None else now-self.observed_at
        if pose_age > s.pose_timeout:
            self.stop('FAILED', 'Marker observation timed out')
            return zero
        if pose_age > s.pose_command_timeout:
            self.state = 'WAITING_FOR_POSE'
            self.reason = 'Waiting briefly for a fresh marker observation; commanding zero velocity'
            return zero
        if now - self._started_at > s.approach_timeout:
            self.stop('FAILED', 'Approach timed out')
            return zero
        try:
            self.check_planar_target(self.target)
        except ValueError as exc:
            self.stop('FAILED', str(exc))
            return zero
        self.error = target_error(self.observation, self.target)
        x, y, yaw = self.error
        if self._extra_target is not None:
            return self._tick_extra_insert(now, self.error)
        if x < -s.tolerance_x:
            self.stop('FAILED', 'Passed the target; reverse manually before restarting')
            return zero
        if abs(x) <= s.tolerance_x and abs(y) <= s.tolerance_y and abs(yaw) <= s.tolerance_yaw:
            if self._settling_since is None:
                self._settling_since = now
            self.state = 'SETTLING'
            self.reason = 'Holding zero velocity inside the target tolerances'
            if now - self._settling_since >= s.settle_time:
                if s.extra_insert_distance > 0:
                    offset = np.eye(4)
                    offset[0, 3] = -s.extra_insert_distance
                    # Desired marker pose after the base advances in its local
                    # +X direction by the configured amount.
                    self._extra_target = offset @ self.observation
                    self._extra_started_at = now
                    self._settling_since = None
                    self._progress_at = now
                    self._previous_command_axes[:] = False
                    self.error = target_error(self.observation, self._extra_target)
                    self.state = 'EXTRA_INSERT'
                    self.reason = (
                        f'Taught pose reached; inserting an additional '
                        f'{1000.0*s.extra_insert_distance:.1f} mm straight ahead'
                    )
                else:
                    self.stop(
                        'REACHED',
                        'Taught pose reached; mechanical engagement and charging unverified')
            return zero
        self._settling_since = None

        lateral = self._axis_command(y, s.tolerance_y, s.gain_y,
                                     s.max_lateral, s.min_lateral)
        angular = self._axis_command(yaw, s.tolerance_yaw, s.gain_yaw,
                                     s.max_yaw_rate)
        far_lateral = abs(y) >= s.far_stop_y
        far_heading = abs(yaw) >= s.far_stop_yaw

        if self._inserting:
            if abs(y) > s.insertion_abort_y or abs(yaw) > s.insertion_abort_yaw:
                self.stop('FAILED',
                          'Alignment lost during insertion; reverse manually before restarting')
                return zero
            if x <= s.tolerance_x:
                self.stop('FAILED',
                          'Reached insertion depth outside final alignment tolerances')
                return zero
            phase = 'INSERTING'
            command = np.array([
                self._axis_command(x, s.tolerance_x, s.gain_x,
                                   s.insertion_max_forward, s.min_forward),
                0.0, 0.0])
            reason = 'Low-speed straight insertion; lateral and heading corrections locked'
        elif x > s.fine_align_distance:
            # With a large initial heading error, rotating changes the target's
            # apparent body-frame lateral error substantially. Turn first, then
            # establish the lateral baseline and translate. Hysteresis avoids
            # switching repeatedly around the coarse heading threshold.
            if far_heading:
                self._coarse_heading_first = True
            if self._coarse_heading_first and abs(yaw) > s.align_yaw:
                phase = 'FAR_HEADING'
                command = np.array([
                    0.0, 0.0,
                    self._axis_command(yaw, s.align_yaw, s.gain_yaw,
                                       s.max_yaw_rate, s.min_heading_yaw_rate)])
                reason = 'Large heading error; rotating in place before lateral alignment'
            else:
                self._coarse_heading_first = False
                if far_lateral:
                    phase = 'FAR_ALIGN'
                    command = np.array([0.0, lateral, angular])
                    reason = 'Large lateral error; pausing forward approach'
                else:
                    # Gradually reduce forward speed as the coarse alignment error
                    # approaches its hard stop. This keeps a 3 cm offset moving on
                    # a diagonal instead of waiting for perfect alignment in place.
                    scale_y = float(np.clip(
                        (s.far_stop_y-abs(y)) / (s.far_stop_y-s.align_y), 0.0, 1.0))
                    scale_yaw = float(np.clip(
                        (s.far_stop_yaw-abs(yaw)) / (s.far_stop_yaw-s.align_yaw), 0.0, 1.0))
                    forward = min(s.gain_x*x, s.max_forward) * min(scale_y, scale_yaw)
                    phase = 'FAR_APPROACH'
                    command = np.array([forward, lateral, angular])
                    reason = 'Coarse approach with simultaneous lateral/heading correction'
        elif x > s.insertion_distance:
            if abs(y) > s.align_y or abs(yaw) > s.align_yaw:
                phase = 'FINE_ALIGN'
                command = np.array([0.0, lateral, angular])
                reason = 'Fine alignment before reaching the fixture entrance'
            else:
                phase = 'FINE_APPROACH'
                command = np.array([
                    self._axis_command(x, s.tolerance_x, s.gain_x,
                                       s.near_max_forward, s.min_forward),
                    lateral, angular])
                reason = 'Reduced-speed approach toward the fixture entrance'
        elif abs(y) > s.insertion_y or abs(yaw) > s.insertion_yaw:
            phase = 'FINAL_ALIGN'
            command = np.array([0.0, lateral, angular])
            reason = 'Final alignment before straight insertion'
        else:
            self._inserting = True
            phase = 'INSERTING'
            command = np.array([
                self._axis_command(x, s.tolerance_x, s.gain_x,
                                   s.insertion_max_forward, s.min_forward),
                0.0, 0.0])
            reason = 'Low-speed straight insertion; lateral and heading corrections locked'

        # Track each axis independently: a large forward error must not hide
        # lateral/heading improvements. Keep the existing 0.25 m/rad weighting.
        progress_error = np.abs(self.error) * [1.0, 1.0, 0.25]
        commanded_axes = np.abs(command) > 1e-9
        newly_commanded = commanded_axes & ~self._previous_command_axes
        # Rotation can turn a small body-frame lateral error into the true
        # lateral displacement. When translation starts after heading-first
        # alignment, use that current value as its progress baseline without
        # resetting the watchdog timer itself.
        self._best_progress_error[newly_commanded] = progress_error[newly_commanded]
        improved = progress_error < self._best_progress_error - s.progress_distance
        # Only count progress on axes that this phase actually commands. Phase
        # changes alone therefore cannot keep an immobile robot alive.
        improved &= commanded_axes
        if np.any(improved):
            # Retain each best value across phase changes, so oscillating between
            # ALIGNING and APPROACHING cannot repeatedly reset the watchdog.
            self._best_progress_error[improved] = progress_error[improved]
            self._progress_at = now
        elif now - self._progress_at > s.progress_timeout:
            self.stop('FAILED',
                      f'No visual progress; possible obstruction; phase={phase}, '
                      f'error_x={x:+.3f} m, error_y={y:+.3f} m, '
                      f'error_yaw={math.degrees(yaw):+.2f} deg, '
                      f'no progress for {now-self._progress_at:.1f} s')
            return zero
        self._previous_command_axes = commanded_axes
        self.state = phase
        self.reason = reason
        return command

    def _tick_extra_insert(self, now, taught_error):
        """Advance a measured distance from the settled taught pose."""
        s = self.settings
        zero = np.zeros(3)
        _, taught_y, taught_yaw = taught_error
        if abs(taught_y) > s.insertion_abort_y or abs(taught_yaw) > s.insertion_abort_yaw:
            self.stop(
                'FAILED',
                'Alignment lost during extra insertion; reverse manually before restarting')
            return zero
        if now - self._extra_started_at > s.extra_insert_timeout:
            self.stop(
                'FAILED',
                f'Extra insertion did not reach {1000.0*s.extra_insert_distance:.1f} mm '
                f'within {s.extra_insert_timeout:.1f} s; possible obstruction')
            return zero

        self.error = target_error(self.observation, self._extra_target)
        x, _, _ = self.error
        if x < -s.tolerance_x:
            self.stop('FAILED', 'Passed the extra insertion target; reverse manually')
            return zero
        if x <= s.extra_insert_tolerance:
            if self._settling_since is None:
                self._settling_since = now
            self.state = 'EXTRA_SETTLING'
            self.reason = 'Holding zero velocity after the additional straight insertion'
            if now - self._settling_since >= s.settle_time:
                self.stop(
                    'REACHED',
                    f'Taught pose plus {1000.0*s.extra_insert_distance:.1f} mm reached; '
                    'mechanical engagement and charging unverified')
            return zero

        self._settling_since = None
        self.state = 'EXTRA_INSERT'
        self.reason = (
            f'Low-speed straight push for an additional '
            f'{1000.0*s.extra_insert_distance:.1f} mm'
        )
        self._previous_command_axes = np.array([True, False, False])
        return np.array([s.extra_insert_speed, 0.0, 0.0])

    @staticmethod
    def _axis_command(error, tolerance, gain, maximum, minimum=0.0):
        if abs(error) <= tolerance:
            return 0.0
        magnitude = min(gain*abs(error), maximum)
        if minimum:
            magnitude = max(magnitude, minimum)
        return math.copysign(magnitude, error)
