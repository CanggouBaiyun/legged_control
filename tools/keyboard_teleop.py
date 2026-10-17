"""Terminal keyboard control for the Go2 ROS 2 simulation.

Run with /usr/bin/python3 after sourcing ROS and the OCS2 workspace. Give this
terminal keyboard focus. Direction keys latch an axis command; Space clears all
axes. Leaving this program requests zero velocity and stance, leaving the
simulation window open.
"""
import argparse
import math
import os
import select
import signal
import sys
import termios
import time
import tty


HELP = """
键盘控制（焦点放在这个终端）：
  T：原地 trot       G：零速并请求四足站立
  W / S：前进 / 后退     A / D：左移 / 右移
  Q / E：左转 / 右转     空格 / X：所有速度归零
  方向键也可前后、左右移动；H：显示帮助
  Esc / Ctrl+C：退出键盘程序，请求零速和站立，仿真窗口继续运行

按一下方向键会持续运动；不同轴可以叠加。空格停止移动但保留 trot。
先按 T，等 MPC 切入 trot 后速度才会发送。键盘退出不会关闭 MuJoCo。
"""


class KeyboardControl:
    def __init__(self, node, args):
        from geometry_msgs.msg import Twist
        from ocs2_msgs.msg import ModeSchedule, MpcObservation
        self.node, self.args = node, args
        self.Twist, self.ModeSchedule = Twist, ModeSchedule
        self.velocity = node.create_publisher(Twist, "/cmd_vel", 1)
        self.gait = node.create_publisher(ModeSchedule, "/legged_robot_mpc_mode_schedule", 1)
        self.observation_sub = node.create_subscription(
            MpcObservation, "/legged_robot_mpc_observation", self.observe, 1)
        self.command = [0.0, 0.0, 0.0]
        self.requested_gait = None
        self.observation_time = -1.0
        self.received = 0.0
        self.last_walking = 0.0
        self.was_connected = False

    def observe(self, message):
        if not math.isfinite(message.time) or message.time < 0 or message.mode not in (15, 9, 6):
            return
        if message.time < self.observation_time:
            self.command = [0.0, 0.0, 0.0]
            self.requested_gait = None
            self.last_walking = 0.0
            print("仿真时间重置；速度已清零，请重新按 T。", flush=True)
        self.observation_time = message.time
        self.received = time.monotonic()
        if message.mode in (6, 9):
            self.last_walking = self.received

    def connected(self):
        return (self.observation_time >= 0 and time.monotonic() - self.received <= 0.2 and
                self.velocity.get_subscription_count() > 0)

    def request_gait(self, trot):
        self.command = [0.0, 0.0, 0.0]
        if not self.connected() or self.gait.get_subscription_count() == 0:
            print("等待仿真和 MPC 就绪后，再按 T/G。", flush=True)
            return False
        if self.requested_gait != trot:
            message = self.ModeSchedule()
            message.event_times = [0.0, 0.30, 0.35, 0.65, 0.70] if trot else [0.0, 0.5]
            message.mode_sequence = [9, 15, 6, 15] if trot else [15]
            self.gait.publish(message)
        self.requested_gait = trot
        print("已请求 trot（步态切换需要等待 MPC 排程）。" if trot else "速度已归零，已请求四足站立。", flush=True)
        return True

    def key(self, key):
        key = key.lower()
        if key in (" ", "x"):
            self.command = [0.0, 0.0, 0.0]
            print("所有速度已归零。", flush=True)
        elif key in ("t", "g"):
            self.request_gait(key == "t")
        elif key == "h":
            print(HELP, flush=True)
        elif key in "wsadqe" and key:
            if not self.connected() or self.requested_gait is not True:
                print("请先等待仿真就绪并按 T；当前保持零速。", flush=True)
                return
            axis, value = {
                "w": (0, self.args.forward_speed), "s": (0, -self.args.forward_speed),
                "a": (1, self.args.lateral_speed), "d": (1, -self.args.lateral_speed),
                "q": (2, self.args.yaw_rate), "e": (2, -self.args.yaw_rate),
            }[key]
            self.command[axis] = value
            print(f"目标 vx={self.command[0]:+.3f} m/s, vy={self.command[1]:+.3f} m/s, "
                  f"yaw_rate={self.command[2]:+.3f} rad/s；空格停止。", flush=True)

    def publish(self):
        connected = self.connected()
        if self.was_connected and not connected:
            self.command = [0.0, 0.0, 0.0]
            self.requested_gait = None
            self.last_walking = 0.0
            print("观测连接中断，速度已清零；恢复后请重新按 T。", flush=True)
        if connected and not self.was_connected:
            print("已收到仿真观测。按 T 开始原地 trot，G 站立，H 查看按键。", flush=True)
        self.was_connected = connected
        message = self.Twist()
        # Wait for the first walking mode and tolerate the brief four-foot
        # phases in trot; a sustained stance suppresses movement commands.
        if connected and self.requested_gait is True and time.monotonic() - self.last_walking <= 0.5:
            message.linear.x, message.linear.y, message.angular.z = self.command
        self.velocity.publish(message)


def read_key(fd):
    raw = os.read(fd, 1)
    if not raw:
        return "\x1b"
    key = raw.decode("utf-8", errors="ignore")
    if key != "\x1b":
        return key
    sequence = ""
    while len(sequence) < 2 and select.select([fd], [], [], 0.02)[0]:
        sequence += os.read(fd, 1).decode("ascii", errors="ignore")
    if not sequence:
        return "\x1b"
    return {"[A": "w", "[B": "s", "[D": "a", "[C": "d"}.get(sequence, "")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--forward-speed", type=float, default=0.03)
    parser.add_argument("--lateral-speed", type=float, default=0.02)
    parser.add_argument("--yaw-rate", type=float, default=0.08)
    args = parser.parse_args()
    for value, maximum in ((args.forward_speed, 0.10), (args.lateral_speed, 0.05), (args.yaw_rate, 0.20)):
        if not math.isfinite(value) or not 0 < value <= maximum:
            parser.error("Speeds must be finite, positive and within 0.10/0.05/0.20 limits")
    if not sys.stdin.isatty():
        parser.error("Run keyboard control in an interactive terminal")
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    os.environ.setdefault("ROS_LOG_DIR", "/tmp/go2_keyboard_ros_logs")
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("go2_keyboard_teleop")
    control = KeyboardControl(node, args)
    fd = sys.stdin.fileno()
    original = termios.tcgetattr(fd)
    def terminate(signum, frame):
        raise KeyboardInterrupt
    previous_term = signal.signal(signal.SIGTERM, terminate)
    try:
        tty.setcbreak(fd)
        print(HELP, flush=True)
        next_publish = 0.0
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            if select.select([fd], [], [], 0.01)[0]:
                key = read_key(fd)
                if key in ("\x1b", "\x03"):
                    break
                if not key and not sys.stdin.isatty():
                    break
                control.key(key)
            now = time.monotonic()
            if now >= next_publish:
                control.publish()
                next_publish = now + 0.1
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, original)
        signal.signal(signal.SIGTERM, previous_term)
        if rclpy.ok():
            stance_requested = control.request_gait(False)
            until = time.monotonic() + 0.3
            while time.monotonic() < until:
                control.command = [0.0, 0.0, 0.0]
                control.publish()
                rclpy.spin_once(node, timeout_sec=0.05)
            print("键盘控制已退出；已发送零速和站立请求，仿真窗口继续运行。" if stance_requested
                  else "键盘控制已退出；未连接到仿真，未发送步态请求。", flush=True)
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
