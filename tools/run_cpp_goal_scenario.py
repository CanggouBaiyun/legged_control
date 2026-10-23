"""Validate planar goal arrival, cancellation, keyboard takeover and P in MuJoCo.

ROS Python, simulation only. Uses the actual keyboard parser through a PTY.
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
from geometry_msgs.msg import PoseStamped
from ocs2_msgs.msg import MpcObservation, MpcTargetTrajectories
from std_msgs.msg import String

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', type=Path, required=True)
    parser.add_argument('--domain', type=int, default=218)
    args = parser.parse_args()
    if not 0 <= args.domain <= 232:
        parser.error('--domain must be in [0, 232]')
    prefix = args.prefix.resolve()
    prefix.parent.mkdir(parents=True, exist_ok=True)
    if list(prefix.parent.glob(prefix.name + '*')):
        parser.error('Result prefix already exists; select a new prefix')
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ.setdefault('ROS_LOG_DIR', '/tmp/go2_goal_ros_logs')
    rclpy.init()
    node = rclpy.create_node('go2_goal_scenario')
    goal_pub = node.create_publisher(PoseStamped, '/go2_control/goal', 1)
    stamp, statuses, targets = [-1.0], [], []
    obs_sub = node.create_subscription(MpcObservation, '/legged_robot_mpc_observation',
                                       lambda m: stamp.__setitem__(0, m.time), 1)
    status_sub = node.create_subscription(String, '/go2_control/goal_status',
                                          lambda m: statuses.append((stamp[0], m.data)), 20)
    target_sub = node.create_subscription(MpcTargetTrajectories, '/legged_robot_mpc_target',
                                          lambda m: targets.append(m), 20)

    def goal(x, y, yaw, frame='base_link'):
        msg = PoseStamped()
        msg.header.frame_id = frame
        msg.pose.position.x, msg.pose.position.y = float(x), float(y)
        msg.pose.orientation.z = math.sin(math.radians(yaw)/2)
        msg.pose.orientation.w = math.cos(math.radians(yaw)/2)
        return msg

    invalid = goal(.3, 0, 0, 'map')
    zero_quaternion = goal(.3, 0, 0)
    zero_quaternion.pose.orientation.w = 0.0
    events = [(3.0, 'trot', 't'),
              (5.0, 'relative_goal', goal(.12, .04, 10)),
              (6.0, 'reject_frame', invalid), (6.3, 'reject_quaternion', zero_quaternion),
              (12.0, 'absolute_goal', goal(.02, -.02, -5, 'odom')),
              (12.7, 'space_cancel', 'x'),
              (13.2, 'goal_before_takeover', goal(.1, 0, 0)),
              (14.0, 'velocity_takeover', 'w'), (14.8, 'zero', 'x'),
              (15.2, 'goal_before_stance', goal(.1, 0, 5)), (16.0, 'stance_cancel', 'g'),
              (17.0, 'trot_again', 't'), (18.5, 'goal_before_stop', goal(.1, 0, 5)),
              (19.0, 'body_pose', 'ri'), (21.0, 'P_stop', 'p'),
              (21.4, 'ignored_after_stop', goal(1., 0, 60))]
    env = os.environ.copy()
    env['GO2_MPC_LOG'] = str(prefix) + '_mpc.log'
    env['GO2_TARGET'] = str(prefix) + '_target.json'
    master, slave = pty.openpty()
    keyboard = simulation = None
    sent = []
    try:
        with Path(str(prefix)+'_keyboard.log').open('w') as keys, Path(str(prefix)+'_launcher.log').open('w') as log:
            keyboard = subprocess.Popen(['/usr/bin/python3', str(ROOT/'tools/keyboard_teleop.py')],
                stdin=slave, stdout=keys, stderr=subprocess.STDOUT, cwd=ROOT, env=env, start_new_session=True)
            os.close(slave)
            slave = None
            command = ['bash', 'scripts/run_cpp_simulation.sh', '--mpc-solver', 'sqp',
                       '--online', '--sensor-startup', '--headless', '--duration', '27',
                       '--joint-kp', '0', '--joint-kd', '1', '--centroidal-base-accel',
                       '--wbc-config', 'config/wbc/upstream_a1_tasks_go2.json', '--csv', str(prefix)+'.csv']
            simulation = subprocess.Popen(command, cwd=ROOT, env=env, stdout=log,
                                          stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic()+130
            while simulation.poll() is None:
                if time.monotonic()>deadline:
                    raise RuntimeError('Simulation timed out')
                rclpy.spin_once(node, timeout_sec=.01)
                if len(sent)<len(events) and stamp[0]>=events[len(sent)][0]:
                    _, name, msg = events[len(sent)]
                    if isinstance(msg, str):
                        os.write(master, msg.encode())
                    else:
                        if goal_pub.get_subscription_count()==0:
                            raise RuntimeError('Goal subscriber missing')
                        goal_pub.publish(msg)
                    sent.append({'name': name, 'time_s': stamp[0]})
                    print(sent[-1], flush=True)
            if simulation.returncode:
                raise RuntimeError(f'Simulation exited {simulation.returncode}; see {prefix}_launcher.log')
            assert keyboard.wait(timeout=3)==0
        with Path(str(prefix)+'.commands.csv').open() as f:
            commands = [{k: float(v) for k,v in row.items()} for row in csv.DictReader(f)]
        with Path(str(prefix)+'.csv').open() as f:
            rows = [{k: float(v) if v else math.nan for k,v in row.items()} for row in csv.DictReader(f)]
        summary = json.loads(Path(str(prefix)+'.json').read_text())
        session = json.loads(Path(str(prefix)+'.session.json').read_text())
        assert len(sent)==len(events)
        assert session['reason']=='controlled_stop', session
        assert any(s=='1:reached' and t<12 for t,s in statuses), 'First goal did not settle before replacement'
        for expected in ('2:cancelled', '3:velocity_takeover', '4:cancelled', '5:session_stop'):
            assert any(s==expected for _,s in statuses), expected
        assert max(r['goal_sequence'] for r in commands)==5, 'Invalid/after-stop goal accepted'
        assert all(r['goal_active']==1 and r['goal_sequence']==1 for r in commands if 6.5<r['time_s']<9)
        absolute = next(r for r in commands if r['goal_sequence']==2)
        assert abs(absolute['goal_x_m']-.02)<1e-9 and abs(absolute['goal_y_m']+.02)<1e-9
        assert abs(absolute['goal_yaw_rad']-math.radians(-5))<1e-9
        reached = next(r for r in commands if r['goal_sequence']==1 and r['goal_reached']==1)
        # Independently compare MuJoCo truth with the requested final goal.
        settled_rows = [r for r in rows if reached['time_s']<=r['time_s']<11.8]
        assert settled_rows
        errors = [math.hypot(r['measured_x_m']-reached['goal_x_m'], r['measured_y_m']-reached['goal_y_m']) for r in settled_rows]
        assert max(errors)<.02, max(errors)
        assert max(r['max_torque_clip_nm'] for r in rows)==0
        assert any(r['goal_active']==1 and r['command_sequence']>10 for r in commands), 'Missing zero heartbeat coexistence'
        assert all(r['goal_active']==0 and abs(r['effective_vx_mps'])<1e-9 for r in commands if r['stop_latched']==1)
        assert any(len(m.state_trajectory)==11 for m in targets)
        for m in targets:
            assert len(m.time_trajectory)==len(m.state_trajectory)==len(m.input_trajectory)
            assert all(b>a for a,b in zip(m.time_trajectory,m.time_trajectory[1:]))
            for state in m.state_trajectory:
                assert len(state.value)==24 and all(math.isfinite(x) for x in state.value)
                assert math.hypot(*state.value[:2])<=.05+1e-7
        report = {'passed': True, 'events': sent, 'first_goal_reached_s': reached['time_s'],
                  'first_goal_estimated_position_error_m': reached['goal_position_error_m'],
                  'first_goal_estimated_yaw_error_deg': math.degrees(reached['goal_yaw_error_rad']),
                  'first_goal_truth_max_position_error_after_arrival_m': max(errors),
                  'session': session, 'target_publications_received': len(targets),
                  'online_goals': summary['online_commands']['goals']}
        Path(str(prefix)+'_checks.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(report, indent=2), flush=True)
    finally:
        for process in (keyboard, simulation):
            if process is not None and process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
        if slave is not None:
            os.close(slave)
        os.close(master)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':
    main()
