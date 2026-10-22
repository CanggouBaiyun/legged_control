"""Run a standing body-pose or keyboard/controlled-stop scenario; simulation only.

Use ROS Python after sourcing ROS and the workspace. Publishes height/tilt
commands, records target limits, rejects invalid requests, and evaluates
measured tracking. Existing result files are not overwritten.
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import pty
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import Pose, Twist
from ocs2_msgs.msg import MpcObservation, MpcTargetTrajectories

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--prefix', type=Path, required=True)
parser.add_argument('--domain', type=int, default=218)
parser.add_argument('--keyboard', action='store_true',
                    help='Drive the real terminal keyboard through a PTY; include trot and P stop.')
args = parser.parse_args()
if not 0 <= args.domain <= 232:
    parser.error('--domain must be in [0, 232]')
PREFIX = args.prefix.resolve()
PREFIX.parent.mkdir(parents=True, exist_ok=True)
for suffix in ('.csv', '.json', '.commands.csv', '.session.json', '_launcher.log',
               '_command.log', '_mpc.log', '_keyboard.log', '_checks.json'):
    if Path(str(PREFIX) + suffix).exists():
        raise RuntimeError(f'Result already exists: {PREFIX}{suffix}')

os.environ['ROS_DOMAIN_ID'] = str(args.domain)
os.environ.setdefault('ROS_LOG_DIR', '/tmp/go2_body_pose_ros_logs')
rclpy.init()
node = rclpy.create_node('body_pose_scenario_check')
publisher = node.create_publisher(Pose, '/go2_control/body_pose', 1)
velocity = node.create_publisher(Twist, '/cmd_vel', 1)
stamp = -1.0
targets = []

def observe(message):
    global stamp
    stamp = message.time

def target(message):
    targets.append(message)

obs_sub = node.create_subscription(MpcObservation, '/legged_robot_mpc_observation', observe, 1)
target_sub = node.create_subscription(MpcTargetTrajectories, '/legged_robot_mpc_target', target, 10)

def pose(z, pitch=0.0, roll=0.0, yaw=0.0):
    message = Pose()
    message.position.z = z
    sr, cr = math.sin(roll/2), math.cos(roll/2)
    sp, cp = math.sin(pitch/2), math.cos(pitch/2)
    sy, cy = math.sin(yaw/2), math.cos(yaw/2)
    message.orientation.w = cr*cp*cy + sr*sp*sy
    message.orientation.x = sr*cp*cy - cr*sp*sy
    message.orientation.y = cr*sp*cy + sr*cp*sy
    message.orientation.z = cr*cp*sy - sr*sp*cy
    return message

bad_quaternion = pose(0.27)
bad_quaternion.orientation.w = 0.0
events = [
    (3.0, 'raise', pose(0.28)),
    (4.5, 'reject_zero_quaternion', bad_quaternion),
    (4.8, 'reject_height', pose(0.4)),
    (5.1, 'reject_yaw', pose(0.27, yaw=0.1)),
    (6.0, 'pitch', pose(0.27, pitch=math.radians(3))),
    (9.0, 'roll', pose(0.27, roll=math.radians(-3))),
    (12.0, 'lower', pose(0.26)),
    (15.0, 'combined', pose(0.28, pitch=math.radians(2), roll=math.radians(-2))),
    (17.0, 'reset', pose(0.27)),
]
if args.keyboard:
    events = [
        (3.0, 'raise', 'rr'), (5.0, 'lower', 'fff'), (7.0, 'reset', 'u'),
        (8.0, 'pitch', 'ii'), (10.0, 'roll', 'ull'), (12.0, 'trot', 't'),
        (14.0, 'move_and_pose', 'urijw'), (18.0, 'stop', 'p'),
    ]
sent = []
command = ['bash', 'scripts/run_cpp_simulation.sh', '--mpc-solver', 'sqp',
           '--online', '--headless', '--duration', '25' if args.keyboard else '20', '--sensor-startup',
           '--joint-kp', '0', '--joint-kd', '1', '--centroidal-base-accel',
           '--wbc-config', 'config/wbc/upstream_a1_tasks_go2.json',
           '--csv', str(PREFIX) + '.csv']
env = os.environ.copy()
env['GO2_MPC_LOG'] = str(PREFIX) + '_mpc.log'
process = None
keyboard = None
keyboard_stream = None
terminal_master = None
post_stop_request_sent = False
try:
    if args.keyboard:
        terminal_master, terminal_slave = pty.openpty()
        keyboard_stream = Path(str(PREFIX) + '_keyboard.log').open('w')
        try:
            keyboard = subprocess.Popen(
                ['/usr/bin/python3', str(ROOT / 'tools/keyboard_teleop.py')],
                cwd=ROOT, env=env, stdin=terminal_slave, stdout=keyboard_stream,
                stderr=subprocess.STDOUT, start_new_session=True)
        finally:
            os.close(terminal_slave)
    with Path(str(PREFIX) + '_launcher.log').open('w') as stream:
        process = subprocess.Popen(command, cwd=ROOT, env=env, stdout=stream,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic() + 130
        while process.poll() is None:
            if time.monotonic() > deadline:
                raise RuntimeError('Scenario timed out')
            rclpy.spin_once(node, timeout_sec=0.02)
            if keyboard is not None and keyboard.poll() is not None:
                if keyboard.returncode != 0 or not any(e['name'] == 'stop' for e in sent):
                    raise RuntimeError('Keyboard exited before a successful P request')
            if len(sent) < len(events) and stamp >= events[len(sent)][0]:
                _, name, message = events[len(sent)]
                if publisher.get_subscription_count() == 0:
                    raise RuntimeError('Body pose subscriber missing')
                if args.keyboard:
                    os.write(terminal_master, message.encode())
                else:
                    publisher.publish(message)
                sent.append({'name': name, 'sent_at_s': stamp})
                print(sent[-1], flush=True)
            if args.keyboard and stamp >= 18.4 and not post_stop_request_sent:
                # A different client must not undo a latched session stop.
                publisher.publish(pose(.29, pitch=math.radians(5)))
                moving = Twist()
                moving.linear.x = .10
                velocity.publish(moving)
                post_stop_request_sent = True
        if process.returncode != 0:
            raise RuntimeError(f'Simulation exited {process.returncode}; see launcher log')
    assert len(sent) == len(events)
    assert len(targets) > 200
    for message in targets:
        assert len(message.state_trajectory) == 2
        first, last = (state.value for state in message.state_trajectory)
        horizon = message.time_trajectory[1] - message.time_trajectory[0]
        assert all(math.isfinite(x) for x in first + last)
        for index, limit in ((8, 0.02), (10, 0.10), (11, 0.10)):
            assert abs(last[index] - first[index]) <= limit*horizon + 1e-6
        assert abs(first[2] - (last[8]-first[8])/horizon) < 1e-6
        assert abs(last[2] - first[2]) < 1e-6
    command_csv = Path(str(PREFIX) + '.commands.csv')
    with command_csv.open() as stream:
        command_rows = [{k: float(v) if v else math.nan for k, v in row.items()}
                for row in csv.DictReader(stream)]
    for first, last in zip(command_rows, command_rows[1:]):
        dt = last['time_s'] - first['time_s']
        for axis, rate in (('z_m', .02), ('pitch_rad', .10), ('roll_rad', .10)):
            assert abs(last['body_reference_'+axis] - first['body_reference_'+axis]) <= rate*dt + 1e-9
    for row in command_rows:
        if not args.keyboard and 3.5 < row['time_s'] < 5.8:
            assert abs(row['body_target_z_m']-.28) < 1e-9
            assert abs(row['body_target_pitch_rad']) < 1e-9
            assert abs(row['body_target_roll_rad']) < 1e-9
    with Path(str(PREFIX) + '.csv').open() as stream:
        rows = [{k: float(v) if v else math.nan for k, v in row.items()}
                for row in csv.DictReader(stream)]
    summary = json.loads(Path(str(PREFIX) + '.json').read_text())
    session = json.loads(Path(str(PREFIX) + '.session.json').read_text())
    if args.keyboard:
        assert session['reason'] == 'controlled_stop', session
        assert 18.4 < float(session['time_s']) < 23.1
        assert keyboard.wait(timeout=3) == 0
        assert post_stop_request_sent
    else:
        assert len(rows) == 10000
        assert session['reason'] == 'duration_limit', session
    assert all(math.isfinite(row[key]) for row in rows for key in
               ('measured_z_m', 'measured_pitch_rad', 'measured_roll_rad'))
    if not args.keyboard:
        assert all(row['mode'] == 15 for row in rows)
    assert max(row['max_torque_clip_nm'] for row in rows) == 0
    phases = []
    phase_definitions = [
        ('raise', 4.0, 5.8, .28, 0, 0),
        ('pitch', 7.3, 8.8, .27, math.radians(3), 0),
        ('roll', 10.3, 11.8, .27, 0, math.radians(-3)),
        ('lower', 13.3, 14.8, .26, 0, 0),
        ('combined', 16.0, 16.8, .28, math.radians(2), math.radians(-2)),
        ('reset', 18.5, 19.9, .27, 0, 0),
    ]
    if args.keyboard:
        phase_definitions = [
            ('raise', 4.0, 4.8, .28, 0, 0),
            ('lower', 6.2, 6.8, .265, 0, 0),
            ('pitch', 9.0, 9.8, .27, math.radians(2), 0),
            ('roll', 11.2, 11.8, .27, 0, math.radians(-2)),
            ('walking_pose', 15.5, 17.5, .275, math.radians(1), math.radians(1)),
        ]
    for name, start, end, z, pitch, roll in phase_definitions:
        selected = [r for r in rows if start <= r['time_s'] < end]
        mean = lambda key: sum(r[key] for r in selected) / len(selected)
        measured = [mean('measured_z_m'), mean('measured_pitch_rad'), mean('measured_roll_rad')]
        errors = [a-b for a, b in zip(measured, (z, pitch, roll))]
        phases.append({'name': name, 'evaluation_interval_s': [start, end],
                       'target_z_pitch_roll': [z, pitch, roll],
                       'mean_measured_z_pitch_roll': measured, 'errors': errors,
                       'passed': abs(errors[0]) <= .005 and max(abs(e) for e in errors[1:]) <= math.radians(1)})
    stop_check = None
    if args.keyboard:
        tail = rows[-150:]
        assert len(tail) == 150 and all(r['mode'] == 15 for r in tail)
        assert all(abs(r['estimated_z_m']-.27) < .008 and
                   abs(r['measured_pitch_rad']) < math.radians(1) and
                   abs(r['measured_roll_rad']) < math.radians(1) for r in tail)
        assert any(r['mode'] in (6, 9) for r in rows)
        walking = [r for r in rows if 15.5 <= r['time_s'] < 17.5]
        assert any(r['mode'] in (6, 9) for r in walking)
        latched = [r for r in command_rows if r['stop_latched'] == 1]
        assert len(latched) >= 6
        assert all(abs(r['body_target_z_m']-.27) < 1e-9 and
                   abs(r['body_target_pitch_rad']) < 1e-9 and
                   abs(r['body_target_roll_rad']) < 1e-9 and
                   abs(r['effective_vx_mps']) < 1e-9 for r in latched)
        stop_check = {'passed': True, 'reason': session['reason'],
                      'stopped_at_s': float(session['time_s']),
                      'neutral_stable_samples': len(tail),
                      'post_stop_motion_and_pose_ignored': True}
    result = {'keyboard': args.keyboard, 'events': sent, 'target_samples_checked': len(targets),
              'command_samples_checked': len(command_rows), 'control_steps': len(rows),
              'rate_and_vertical_reference_checks_passed': True,
              'invalid_command_checks_passed': None if args.keyboard else True,
              'stop_check': stop_check,
              'mean_tracking_tolerances': {'height_m': .005, 'angle_deg': 1.0},
              'phases': phases,
              'passed': all(p['passed'] for p in phases)}
    Path(str(PREFIX) + '_checks.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2), flush=True)
    if not result['passed']:
        raise RuntimeError('Body pose tracking did not pass initial gates')
finally:
    if keyboard is not None and keyboard.poll() is None:
        os.killpg(keyboard.pid, signal.SIGTERM)
        try:
            keyboard.wait(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(keyboard.pid, signal.SIGKILL)
            keyboard.wait(timeout=3)
    if terminal_master is not None:
        os.close(terminal_master)
    if keyboard_stream is not None:
        keyboard_stream.close()
    if process is not None and process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)
    node.destroy_node()
    rclpy.try_shutdown()
