"""Check the actual ROS goal parser and command arbitration using synthetic observations.

No physics or hardware commands. Run with ROS Python after sourcing the workspace.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from ocs2_msgs.msg import MpcObservation, MpcTargetTrajectories
from std_msgs.msg import Empty, String

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', type=Path, required=True)
    parser.add_argument('--domain', type=int, default=219)
    args = parser.parse_args()
    if not 0 <= args.domain <= 232:
        parser.error('Invalid ROS domain')
    prefix = args.prefix.resolve()
    if list(prefix.parent.glob(prefix.name+'*')):
        parser.error('Result prefix exists')
    prefix.parent.mkdir(parents=True, exist_ok=True)
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ.setdefault('ROS_LOG_DIR', '/tmp/go2_goal_interface_logs')
    rclpy.init()
    node = rclpy.create_node('goal_interface_check')
    observation = node.create_publisher(MpcObservation, '/legged_robot_mpc_observation', 1)
    goal = node.create_publisher(PoseStamped, '/go2_control/goal', 1)
    velocity = node.create_publisher(Twist, '/cmd_vel', 1)
    stop = node.create_publisher(Empty, '/go2_control/stop', 1)
    targets, statuses = [], []
    target_sub = node.create_subscription(MpcTargetTrajectories, '/legged_robot_mpc_target', targets.append, 20)
    status_sub = node.create_subscription(String, '/go2_control/goal_status', lambda m: statuses.append(m.data), 20)
    sim_time, mode = 10., 15
    yaw = math.radians(179)

    def spin(seconds, observations=True, zero_heartbeat=False):
        nonlocal sim_time
        end = time.monotonic()+seconds
        while time.monotonic()<end:
            if observations:
                m = MpcObservation()
                sim_time += .01
                m.time, m.mode = sim_time, mode
                state = [0.]*24
                state[6:12] = [.4, -.2, .27, yaw, 0., 0.]
                m.state.value = state
                m.input.value = [0.]*24
                observation.publish(m)
            if zero_heartbeat:
                velocity.publish(Twist())
            rclpy.spin_once(node, timeout_sec=.01)

    def message(frame='base_link', x=.1, y=0., angle=2.):
        m = PoseStamped()
        m.header.frame_id = frame
        m.pose.position.x, m.pose.position.y = float(x), float(y)
        m.pose.orientation.z = math.sin(math.radians(angle)/2)
        m.pose.orientation.w = math.cos(math.radians(angle)/2)
        return m

    process = None
    try:
        with Path(str(prefix)+'.log').open('w') as stream:
            process = subprocess.Popen(['ros2', 'run', 'go2_control_cpp', 'online_command', '--ros-args',
                '-p', 'referenceFile:='+str(ROOT/'config/ocs2/go2/reference.info'),
                '-p', 'taskFile:='+str(ROOT/'config/ocs2/go2/task.info')],
                stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
            spin(2.)
            assert goal.get_subscription_count()>0 and targets
            goal.publish(message())
            spin(.3, zero_heartbeat=True)
            assert statuses[-1]=='1:waiting_for_trot', statuses[-1]
            waiting = targets[-1]
            assert all(abs(m.value[6]-.4)<1e-6 and abs(m.value[9]-yaw)<1e-6 and
                       math.hypot(*m.value[:2])<1e-9 for m in waiting.state_trajectory)
            for invalid in (message('map'), message(x=2.1), message(x=math.nan)):
                goal.publish(invalid)
                spin(.1)
                assert statuses[-1]=='1:waiting_for_trot'
            invalid = message()
            invalid.pose.orientation.x = .1
            goal.publish(invalid)
            spin(.1)
            assert statuses[-1]=='1:waiting_for_trot'
            mode = 9
            spin(3.3, zero_heartbeat=True)
            assert statuses[-1]=='1:tracking'
            future = targets[-1].state_trajectory[-1].value
            assert abs(future[6]-(.4+.1*math.cos(yaw)))<1e-6
            assert abs(future[7]-(-.2+.1*math.sin(yaw)))<1e-6
            assert abs(future[9]-math.radians(181))<1e-6
            goal.publish(message('odom', .4, -.2, -179))
            spin(.2)
            assert statuses[-1]=='2:tracking'
            assert abs(targets[-1].state_trajectory[-1].value[9]-math.radians(181))<1e-6
            spin(.35, observations=False)
            spin(.15)
            assert statuses[-1]=='2:observation_lost', statuses[-1]
            assert math.hypot(*targets[-1].state_trajectory[0].value[:2])<1e-9
            goal.publish(message())
            spin(.2)
            assert statuses[-1]=='3:tracking'
            sim_time = 0.
            mode = 15
            spin(.15)
            assert statuses[-1]=='3:idle', statuses[-1]
            # Exercise the end-user helper and its acknowledgement on the same graph.
            client = subprocess.Popen(['/usr/bin/python3', str(ROOT/'tools/send_goal.py'),
                                       '--x', '.1', '--yaw-deg', '2'],
                                      stdout=stream, stderr=subprocess.STDOUT)
            deadline = time.monotonic()+10
            while client.poll() is None and time.monotonic()<deadline:
                spin(.05)
            if client.poll() is None:
                client.terminate()
                client.wait(timeout=3)
                raise RuntimeError('Goal helper timed out')
            assert client.returncode==0
            assert statuses[-1]=='4:waiting_for_trot'
            stop.publish(Empty())
            spin(.15)
            assert statuses[-1]=='4:session_stop'
            goal.publish(message(x=.2))
            moving = Twist()
            moving.linear.x = .1
            velocity.publish(moving)
            spin(.2)
            assert statuses[-1]=='4:session_stop'
            assert all(math.hypot(*m.value[:2])<1e-9 for m in targets[-1].state_trajectory)
            assert process.poll() is None
        report = {'passed': True, 'checks': ['stance waiting', 'zero heartbeat coexistence',
                  'frame/range/NaN/tilt rejection', 'relative frame conversion', 'yaw +179 to -179 shortest path',
                  'absolute goal', 'observation loss cancellation', 'time rewind reset',
                  'send_goal helper acknowledgement', 'stop priority'], 'physics_test': False}
        Path(str(prefix)+'.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(report, indent=2))
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':
    main()
