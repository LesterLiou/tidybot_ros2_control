import math

import cv2
import numpy as np
import pytest

from aruco_detection import ArucoPoseEstimator
from docking_core import (DockingControl, DockingSettings, average_transforms, inverse,
                          load_target, matrix_quaternion, quaternion_matrix, save_target,
                          target_error, transform, validate_transform)


def planar(x=0.0, y=0.0, yaw=0.0):
    return transform([x, y, 0], [0, 0, math.sin(yaw/2), math.cos(yaw/2)])


def test_error_is_goal_base_pose_in_current_base_even_with_offset_tag():
    goal = transform([0.8, 0.15, 0.4], [0.5, -0.5, 0.5, 0.5])
    current_base = planar(-0.3, 0.06, 0.2)
    observation = inverse(current_base) @ goal
    expected = inverse(current_base)
    assert np.allclose(target_error(observation, goal),
                       [expected[0, 3], expected[1, 3], -0.2])


def test_quaternion_roundtrip_near_pi():
    for quaternion in [[1, 0, 0, 0], [0.5, -0.5, 0.5, 0.5], [0, 0, 0.999999, 0.001]]:
        rotation = quaternion_matrix(quaternion)
        assert np.allclose(quaternion_matrix(matrix_quaternion(rotation)), rotation)


@pytest.mark.parametrize('bad', [np.zeros((4, 4)), np.full((4, 4), np.nan),
                               np.diag([1, 1, -1, 1])])
def test_invalid_transforms_rejected(bad):
    with pytest.raises(ValueError):
        validate_transform(bad)


def test_teaching_rejects_motion_and_averages_rotation():
    samples = [planar(0.8+i*0.0001, yaw=math.pi+i*0.001) for i in range(10)]
    mean = average_transforms(samples, 0.005, 0.02)
    assert mean[0, 3] == pytest.approx(0.80045)
    with pytest.raises(ValueError, match='unstable'):
        average_transforms(samples+[planar(0.9)], 0.005, 0.02)


def test_target_atomic_roundtrip_and_bad_data(tmp_path):
    path = tmp_path/'nested'/'target.yaml'
    target = planar(0.8, 0.02, 0.1)
    save_target(path, target, 'mount-and-marker-fingerprint', 'base_link')
    loaded, context = load_target(path, 'base_link')
    assert np.allclose(loaded, target)
    assert context == 'mount-and-marker-fingerprint'
    with pytest.raises(ValueError):
        load_target(path, 'wrong_frame')
    path.write_text('version: [broken YAML')
    with pytest.raises(ValueError, match='Malformed'):
        load_target(path, 'base_link')


def test_closed_loop_converges_with_offset_marker_and_bounded_body_commands():
    control = DockingControl()
    marker = transform([0.6, 0.12, 0.4], [0.5, -0.5, 0.5, 0.5])
    base = planar(-0.3, 0.05, 0.12)
    control.observe(inverse(base) @ marker, 0)
    control.start(marker, 0)
    states = set()
    for step in range(2250):
        now = step*0.02
        control.observe(inverse(base) @ marker, now)
        cmd = control.tick(now)
        assert np.all(np.abs(cmd) <= [0.04+1e-9, 0.04+1e-9, 0.12+1e-9])
        states.add(control.state)
        base = base @ planar(cmd[0]*0.02, cmd[1]*0.02, cmd[2]*0.02)
        if not control.active:
            break
    assert control.state == 'REACHED', control.reason
    assert 'FAR_APPROACH' in states
    assert 'INSERTING' in states
    assert np.all(control.tick(now+1) == 0)


def test_stale_pose_and_cancel_never_resume():
    control = DockingControl()
    control.observe(planar(1), 0)
    control.start(planar(0.7), 0)
    assert control.tick(0.1)[0] > 0
    assert np.all(control.tick(0.2) == 0)
    assert control.state == 'WAITING_FOR_POSE'
    assert control.active
    control.observe(planar(0.99), 0.21)
    assert control.tick(0.21)[0] > 0
    assert control.active
    assert np.all(control.tick(1.0) == 0)
    assert control.state == 'FAILED'
    control.observe(planar(0.9), 1.01)
    assert np.all(control.tick(1.01) == 0)
    assert not control.active


def test_start_accepts_briefly_stale_pose_but_holds_motion_until_reacquired():
    control = DockingControl()
    control.observe(planar(1.0), 0)
    control.start(planar(0.7), 0.3)
    assert np.all(control.tick(0.3) == 0)
    assert control.state == 'WAITING_FOR_POSE'
    assert control.active
    control.observe(planar(0.99), 0.31)
    assert control.tick(0.31)[0] > 0


def test_reached_needs_fresh_observations_and_stable_dwell():
    control = DockingControl()
    control.observe(planar(0.8), 0)
    control.start(planar(0.8), 0)
    control.tick(0)
    assert control.state == 'SETTLING'
    control.tick(0.8)
    assert control.state == 'FAILED'
    control.observe(planar(0.8), 1)
    control.start(planar(0.8), 1)
    for step in range(36):
        now = 1+step*0.02
        control.observe(planar(0.8), now)
        assert np.all(control.tick(now) == 0)
    assert control.state == 'REACHED'


def test_right_compensation_shifts_effective_goal_to_robot_right():
    settings = DockingSettings(target_right_offset=0.02)
    control = DockingControl(settings)
    taught_goal = planar(0.5)
    control.observe(taught_goal, 0.0)
    control.start(taught_goal, 0.0)

    # A robot-right goal displacement (-Y) makes the marker's effective goal
    # appear at +Y, so the controller initially commands negative Y (right).
    assert control.target[1, 3] == pytest.approx(0.02)
    command = control.tick(0.0)
    assert command[1] < 0

    compensated_goal = planar(0.0, 0.02) @ taught_goal
    control.observe(compensated_goal, 0.02)
    assert np.all(control.tick(0.02) == 0)
    assert control.state == 'SETTLING'


def test_right_compensation_rejects_negative_or_nonfinite_values():
    with pytest.raises(ValueError, match='target_right_offset'):
        DockingSettings(target_right_offset=-0.001)
    with pytest.raises(ValueError, match='target_right_offset'):
        DockingSettings(target_right_offset=math.inf)


def test_extra_insert_advances_five_mm_after_taught_pose_settles():
    settings = DockingSettings(
        settle_time=0.1,
        extra_insert_distance=0.005,
        extra_insert_tolerance=0.001,
        extra_insert_timeout=1.0,
    )
    control = DockingControl(settings)
    goal = planar(0.5)
    control.observe(goal, 0.0)
    control.start(goal, 0.0)

    assert np.all(control.tick(0.0) == 0)
    control.observe(goal, 0.1)
    assert np.all(control.tick(0.1) == 0)
    assert control.state == 'EXTRA_INSERT'
    control.observe(goal, 0.12)
    assert control.tick(0.12)[0] == pytest.approx(settings.extra_insert_speed)

    inserted_pose = planar(-0.005) @ goal
    control.observe(inserted_pose, 0.2)
    assert np.all(control.tick(0.2) == 0)
    assert control.state == 'EXTRA_SETTLING'
    control.observe(inserted_pose, 0.32)
    assert np.all(control.tick(0.32) == 0)
    assert control.state == 'REACHED'
    assert '5.0 mm' in control.reason


def test_extra_insert_times_out_if_fixture_blocks_motion():
    settings = DockingSettings(
        settle_time=0.1,
        extra_insert_distance=0.005,
        extra_insert_tolerance=0.001,
        extra_insert_timeout=0.3,
    )
    control = DockingControl(settings)
    goal = planar(0.5)
    control.observe(goal, 0.0)
    control.start(goal, 0.0)
    control.tick(0.0)
    control.observe(goal, 0.1)
    control.tick(0.1)
    control.observe(goal, 0.41)

    assert np.all(control.tick(0.41) == 0)
    assert control.state == 'FAILED'
    assert 'possible obstruction' in control.reason


def test_obstruction_jump_and_overshoot_are_failures_not_success():
    settings = DockingSettings(progress_timeout=0.4)
    control = DockingControl(settings)
    control.observe(planar(1), 0)
    control.start(planar(0.8), 0)
    for step in range(30):
        control.observe(planar(1), step*0.02)
        control.tick(step*0.02)
    assert control.state == 'FAILED' and 'obstruction' in control.reason
    control.start(planar(0.8), 0.59)
    assert not control.observe(planar(0.7), 0.6)
    assert control.state == 'FAILED'
    with pytest.raises(ValueError, match='behind'):
        control.start(planar(1.2), 0.6)


@pytest.mark.parametrize('axis', ['lateral', 'heading'])
def test_slow_correction_counts_progress_during_far_approach(axis):
    control = DockingControl(DockingSettings(progress_timeout=5.0))
    goal = transform([0.46, 0, 0.4], [0.5, -0.5, -0.5, 0.5])
    for step in range(451):
        now = step*0.02
        y = 0.03-0.0015*now if axis == 'lateral' else 0.0
        yaw = 0.12-0.006*now if axis == 'heading' else 0.0
        control.observe(planar(0.3, y, yaw) @ goal, now)
        if step == 0:
            control.start(goal, now)
        command = control.tick(now)
        assert control.active, control.reason
        assert control.state == 'FAR_APPROACH'
        assert command[0] > 0


def test_slow_start_has_time_to_make_progress_after_five_seconds():
    control = DockingControl()
    goal = planar(0.5)
    for step in range(451):
        now = step*0.02
        x = 0.1-0.003*max(0.0, now-6.0)
        control.observe(planar(x) @ goal, now)
        if step == 0:
            control.start(goal, now)
        command = control.tick(now)
        assert control.active, control.reason
        assert control.state == 'FINE_APPROACH'
        assert command[0] > 0


def test_stationary_robot_still_stops_after_eight_seconds_with_fresh_poses():
    control = DockingControl()
    control.observe(planar(0.6), 0)
    control.start(planar(0.5), 0)
    for step in range(411):
        now = step*0.02
        control.observe(planar(0.6), now)
        command = control.tick(now)
        if now < 8.0:
            assert control.active
    assert control.state == 'FAILED'
    assert 'No visual progress' in control.reason
    assert np.all(command == 0)


def test_phase_oscillation_does_not_keep_resetting_progress_timeout():
    control = DockingControl(DockingSettings(progress_timeout=0.6))
    goal = planar(0.5)
    states = set()
    for step in range(46):
        now = step*0.02
        y = 0.018 if (step//5) % 2 == 0 else 0.012
        control.observe(planar(0.1, y) @ goal, now)
        if step == 0:
            control.start(goal, now)
        command = control.tick(now)
        states.add(control.state)
    assert {'FINE_ALIGN', 'FINE_APPROACH'} <= states
    assert control.state == 'FAILED'
    assert 'No visual progress' in control.reason
    assert np.all(command == 0)


def test_alignment_does_not_count_forward_change_as_progress():
    control = DockingControl(DockingSettings(progress_timeout=0.4))
    goal = planar(0.5)
    for step in range(46):
        now = step*0.02
        control.observe(planar(0.3-0.02*now, 0.09) @ goal, now)
        if step == 0:
            control.start(goal, now)
        command = control.tick(now)
        assert control.state in {'FAR_ALIGN', 'FAILED'}
        assert command[0] == 0
    assert control.state == 'FAILED'
    assert 'No visual progress' in control.reason


def test_real_observed_offset_moves_forward_and_laterally_at_long_range():
    control = DockingControl()
    goal = planar(0.5)
    control.observe(planar(0.162, -0.033, math.radians(0.3)) @ goal, 0)
    control.start(goal, 0)

    command = control.tick(0)

    assert control.state == 'FAR_APPROACH'
    assert command[0] == pytest.approx(0.04*(0.08-0.033)/(0.08-0.015))
    assert command[1] == pytest.approx(-0.033)
    assert command[2] == 0


def test_large_far_offset_pauses_forward_until_coarsely_aligned():
    control = DockingControl()
    goal = planar(0.5)
    control.observe(planar(0.16, 0.09) @ goal, 0)
    control.start(goal, 0)

    command = control.tick(0)

    assert control.state == 'FAR_ALIGN'
    assert command[0] == 0
    assert command[1] == pytest.approx(0.04)


def test_large_heading_turns_first_then_tracks_lateral_progress():
    control = DockingControl(DockingSettings(progress_timeout=0.4))
    goal = planar()
    control.observe(planar(1.107, 0.075, math.radians(27.5)) @ goal, 0)
    control.start(goal, 0)
    command = control.tick(0)
    assert control.state == 'FAR_HEADING'
    assert np.all(command[:2] == 0)
    assert command[2] == pytest.approx(control.settings.max_yaw_rate)

    # Turning to face the target exposes the larger body-frame lateral error.
    # Once heading is aligned, lateral progress must be measured from here,
    # rather than from the deceptively small initial 7.5 cm value.
    turn_samples = [
        (0.02, 0.38), (-0.06, 0.28), (-0.14, 0.18),
        (-0.22, 0.08), (-0.274, 0.0),
    ]
    for step in range(1, 14):
        now = step*0.1
        if step <= len(turn_samples):
            y, yaw = turn_samples[step-1]
        else:
            y, yaw = -0.274+0.018*(step-len(turn_samples)), 0.0
        control.observe(planar(1.06, y, yaw) @ goal, now)
        command = control.tick(now)
        assert control.active, control.reason
        if step < len(turn_samples):
            assert control.state == 'FAR_HEADING'
            assert np.all(command[:2] == 0)
            assert command[2] >= control.settings.min_heading_yaw_rate
        else:
            assert control.state == 'FAR_ALIGN'
            assert command[0] == 0 and command[1] < 0


def test_fine_approach_requires_the_tighter_alignment_thresholds():
    control = DockingControl()
    goal = planar(0.5)
    control.observe(planar(0.08, 0.03) @ goal, 0)
    control.start(goal, 0)
    command = control.tick(0)
    assert control.state == 'FINE_ALIGN'
    assert command[0] == 0
    assert command[1] > 0

    control.observe(planar(0.08, 0.01) @ goal, 0.02)
    command = control.tick(0.02)
    assert control.state == 'FINE_APPROACH'
    assert command[0] == pytest.approx(control.settings.near_max_forward)


def test_fine_and_insertion_speeds_stay_above_measured_forward_dead_zone():
    control = DockingControl()
    goal = planar(0.5)
    control.observe(planar(0.095) @ goal, 0)
    control.start(goal, 0)
    command = control.tick(0)
    assert control.state == 'FINE_APPROACH'
    assert command[0] == pytest.approx(0.028)

    control.observe(planar(0.054) @ goal, 0.01)
    command = control.tick(0.01)
    assert control.state == 'FINE_APPROACH'
    assert command[0] == pytest.approx(0.025)

    control.observe(planar(0.04) @ goal, 0.02)
    command = control.tick(0.02)
    assert control.state == 'INSERTING'
    assert command[0] == pytest.approx(0.025)

    control.observe(planar(0.011) @ goal, 0.04)
    command = control.tick(0.04)
    assert control.state == 'INSERTING'
    assert command[0] == pytest.approx(0.025)


def test_final_alignment_latches_low_speed_straight_insertion():
    control = DockingControl()
    goal = planar(0.5)
    control.observe(planar(0.04, 0.01) @ goal, 0)
    control.start(goal, 0)
    command = control.tick(0)
    assert control.state == 'FINAL_ALIGN'
    assert command[0] == 0
    assert command[1] >= control.settings.min_lateral

    control.observe(planar(0.04, 0.004, 0.02) @ goal, 0.02)
    command = control.tick(0.02)
    assert control.state == 'INSERTING'
    assert command[0] == pytest.approx(control.settings.insertion_max_forward)
    assert np.all(command[1:] == 0)

    # Small visual drift cannot introduce sideways motion inside the fixture.
    control.observe(planar(0.03, 0.012, 0.04) @ goal, 0.04)
    command = control.tick(0.04)
    assert control.state == 'INSERTING'
    assert command[0] > 0
    assert np.all(command[1:] == 0)


def test_insertion_stops_if_alignment_is_lost():
    control = DockingControl()
    goal = planar(0.5)
    control.observe(planar(0.04, 0.004) @ goal, 0)
    control.start(goal, 0)
    assert control.tick(0)[0] > 0
    control.observe(planar(0.03, 0.03) @ goal, 0.02)

    assert np.all(control.tick(0.02) == 0)
    assert control.state == 'FAILED'
    assert 'Alignment lost during insertion' in control.reason


def test_aruco_known_size_and_negative_frames():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    draw = getattr(cv2.aruco, 'generateImageMarker', getattr(cv2.aruco, 'drawMarker', None))
    marker = draw(dictionary, 0, 160)
    image = np.full((720, 960), 255, dtype=np.uint8)
    image[280:440, 400:560] = marker
    k = np.array([[800., 0, 480], [0, 800., 360], [0, 0, 1]])
    estimator = ArucoPoseEstimator(marker_size=0.12)
    result = estimator.estimate(image, k, np.zeros(5))
    assert result is not None
    matrix, _, error = result
    assert matrix[2, 3] == pytest.approx(0.6, abs=0.005)
    assert np.linalg.norm(matrix[:2, 3]) < 0.002
    assert error < 0.2
    assert ArucoPoseEstimator(marker_id=1).estimate(image, k, np.zeros(5)) is None
    assert estimator.estimate(np.full_like(image, 255), k, np.zeros(5)) is None
    image[280:440, 100:260] = marker
    assert estimator.estimate(image, k, np.zeros(5)) is None


def test_unreachable_height_or_tilt_is_rejected():
    control = DockingControl()
    raised = planar(1)
    raised[2, 3] = 0.08
    control.observe(raised, 0)
    with pytest.raises(ValueError, match='height/tilt'):
        control.start(planar(0.8), 0)
    control.observe(planar(1), 0.1)
    control.start(planar(0.8), 0.1)
    control.observe(raised, 0.2)
    assert np.all(control.tick(0.2) == 0)
    assert control.state == 'FAILED'


def test_far_height_noise_is_allowed_but_near_limit_stays_strict():
    goal = planar(0.5)
    far_delta = planar(1.047, -0.105)
    far_delta[2, 3] = -0.043
    control = DockingControl()
    control.observe(far_delta @ goal, 0)
    control.start(goal, 0)
    assert control.tick(0)[1] < 0
    assert control.active

    near_delta = planar(0.05)
    near_delta[2, 3] = -0.043
    near_control = DockingControl()
    near_control.observe(near_delta @ goal, 0)
    with pytest.raises(ValueError, match=r'height=-0\.043 m \(limit 0\.040 m'):
        near_control.start(goal, 0)


def test_height_and_tilt_rejections_report_measurements_and_limits():
    control = DockingControl()
    raised = planar(1)
    raised[2, 3] = 0.08
    control.observe(raised, 0)
    with pytest.raises(ValueError, match=r'height=\+0\.080 m \(limit 0\.040 m'):
        control.start(planar(0.95), 0)
    angle = 0.2
    tilted = transform([1, 0, 0], [math.sin(angle/2), 0, 0, math.cos(angle/2)])
    control.observe(tilted, 0.1)
    with pytest.raises(ValueError, match=r'tilt=11\.46 deg \(limit 8\.59 deg\)'):
        control.start(planar(0.8), 0.1)
    assert not control.active
    assert np.all(control.tick(0.2) == 0)
