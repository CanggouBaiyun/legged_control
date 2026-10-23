"""Send one latched planar goal to the running online simulation (ROS Python)."""
import argparse
import math
import os
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--x', type=float, default=0.0)
    parser.add_argument('--y', type=float, default=0.0)
    parser.add_argument('--yaw-deg', type=float, default=0.0)
    parser.add_argument('--frame', choices=('base_link', 'odom'), default='base_link')
    parser.add_argument('--cancel', action='store_true')
    args = parser.parse_args()
    if not all(math.isfinite(x) for x in (args.x, args.y, args.yaw_deg)):
        parser.error('Goal values must be finite')
    if args.frame == 'base_link' and math.hypot(args.x, args.y) > 2:
        parser.error('A single relative translation must be <= 2 m')
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from ocs2_msgs.msg import MpcObservation
    from std_msgs.msg import Empty, String
    os.environ.setdefault('ROS_LOG_DIR', '/tmp/go2_goal_ros_logs')
    rclpy.init()
    node = rclpy.create_node('go2_send_goal')
    goal = node.create_publisher(PoseStamped, '/go2_control/goal', 1)
    cancel = node.create_publisher(Empty, '/go2_control/cancel_goal', 1)
    received = [0.0]
    status = ['']
    obs_sub = node.create_subscription(MpcObservation, '/legged_robot_mpc_observation',
                                       lambda m: received.__setitem__(0, time.monotonic()), 1)
    status_sub = node.create_subscription(String, '/go2_control/goal_status',
                                          lambda m: status.__setitem__(0, m.data), 1)
    try:
        deadline = time.monotonic() + 5
        publisher = cancel if args.cancel else goal
        while (publisher.get_subscription_count() == 0 or not status[0] or
               time.monotonic() - received[0] > .2):
            rclpy.spin_once(node, timeout_sec=.05)
            if time.monotonic() > deadline:
                raise RuntimeError('No fresh online simulation; check startup, ROS_DOMAIN_ID and --online')
        previous_sequence = int(status[0].split(':', 1)[0])
        if args.cancel:
            publisher.publish(Empty())
        else:
            message = PoseStamped()
            message.header.frame_id = args.frame
            message.header.stamp = node.get_clock().now().to_msg()
            message.pose.position.x, message.pose.position.y = args.x, args.y
            angle = math.radians(args.yaw_deg)
            message.pose.orientation.z = math.sin(angle/2)
            message.pose.orientation.w = math.cos(angle/2)
            publisher.publish(message)
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            sequence, state = status[0].split(':', 1)
            if (args.cancel and state not in ('tracking', 'waiting_for_trot', 'reached') or
                    not args.cancel and int(sequence) > previous_sequence):
                if args.cancel:
                    detail = '目标已取消。'
                elif state == 'waiting_for_trot':
                    detail = '目标已接收，等待 trot；请在键盘终端按 T。'
                elif state == 'reached':
                    detail = '已到达目标，保持位置和朝向，步态不变。'
                else:
                    detail = '正在追踪目标，无需再按 T。到达后保持位置和朝向，步态不变。'
                print(f'节点确认：{status[0]}。{detail}')
                return 0
        raise RuntimeError('No acknowledgement: goal rejected or session stop latched; see command log')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
