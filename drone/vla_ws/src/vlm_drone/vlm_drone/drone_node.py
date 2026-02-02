#!/usr/bin/env python3
"""
PX4 Offboard VLA controller (ROS2 + px4_msgs, no MAVROS)

Pipeline:
- Subscribe camera image -> sample -> publish /vla_image -> send to local server -> get action
- Subscribe /fmu/out/vehicle_odometry -> compute yaw
- Publish OffboardControlMode + TrajectorySetpoint at 20Hz continuously
- After pre-streaming setpoints, send VehicleCommand to ARM + set OFFBOARD
"""

import math
import time
import cv2
import numpy as np
import requests

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleOdometry,
)

# -----------------------------
# PX4 vehicle command constants
# -----------------------------
VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
VEHICLE_CMD_DO_SET_MODE = 176

# PX4 custom mode (main mode) numbers:
# These are commonly used values for PX4. If your PX4 version differs, adjust.
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6


class VLAOffboardNode(Node):
    def __init__(self):
        super().__init__("vla_offboard_node")

        # -----------------------------
        # Parameters (tune in launch)
        # -----------------------------
        self.declare_parameter("image_topic", "/d455/fisheye1/image_raw")
        self.declare_parameter("vla_image_topic", "/vla_image")

        self.declare_parameter("server_url", "http://127.0.0.1:5000/predict_action")
        self.declare_parameter("instruction", "Fly through the gates and avoid obstacles")

        # Sample every N images for inference (low freq). But control is always 20Hz.
        self.declare_parameter("c_rate", 20)

        # Output scaling
        self.declare_parameter("vel_k_z", 0.5)
        self.declare_parameter("vel_l_xy", 1.875)
        self.declare_parameter("yaw_k", 1.5)

        # Control publish rate (Hz) - must be > 2Hz for PX4 offboard, recommend 20Hz
        self.declare_parameter("control_rate_hz", 20.0)

        # If your model outputs vz positive-up, set this True to convert to PX4 NED (z down)
        self.declare_parameter("px4_ned_z_down", True)

        # After this many control ticks, send ARM+OFFBOARD commands
        # e.g. 40 ticks @20Hz = 2s pre-streaming
        self.declare_parameter("prestream_ticks", 40)

        # -----------------------------
        # Load params
        # -----------------------------
        self.image_topic = self.get_parameter("image_topic").value
        self.vla_image_topic = self.get_parameter("vla_image_topic").value

        self.server_url = self.get_parameter("server_url").value
        self.instruction = self.get_parameter("instruction").value
        self.c_rate = int(self.get_parameter("c_rate").value)

        self.vel_k_z = float(self.get_parameter("vel_k_z").value)
        self.vel_l_xy = float(self.get_parameter("vel_l_xy").value)
        self.yaw_k = float(self.get_parameter("yaw_k").value)

        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.px4_ned_z_down = bool(self.get_parameter("px4_ned_z_down").value)

        self.prestream_ticks = int(self.get_parameter("prestream_ticks").value)

        # -----------------------------
        # State
        # -----------------------------
        self.bridge = CvBridge()
        self.counter = 0
        self.allow_inference = True

        self.frame = None
        self.yaw_drone = 0.0

        # latest model output (body frame): vx, vy, vz, dyaw
        self.last_action_body = [0.0, 0.0, 0.0, 0.0]
        # latest command in world/NED frame for PX4 setpoint: vx, vy, vz, yaw_rate
        self.last_cmd_ned = [0.0, 0.0, 0.0, 0.0]

        # Offboard state machine
        self.control_tick = 0
        self.offboard_enabled = False
        self.armed_sent = False
        self.offboard_sent = False

        # -----------------------------
        # Publishers (PX4)
        # -----------------------------
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", 10
        )
        self.traj_sp_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10
        )
        self.vehicle_cmd_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", 10
        )

        # Debug image republish
        self.vla_image_pub = self.create_publisher(Image, self.vla_image_topic, 10)

        # -----------------------------
        # Subscribers
        # -----------------------------
        self.create_subscription(Image, self.image_topic, self.image_callback, 10)
        self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry", self.odom_callback, 10)

        # -----------------------------
        # Timers
        # -----------------------------
        period = 1.0 / max(self.control_rate_hz, 5.0)  # safety
        self.control_timer = self.create_timer(period, self.control_loop)

        self.get_logger().info("VLAOffboardNode started.")
        self.get_logger().info(f"Image topic: {self.image_topic}")
        self.get_logger().info(f"Server URL : {self.server_url}")
        self.get_logger().info(f"Control rate: {self.control_rate_hz} Hz, prestream_ticks={self.prestream_ticks}")

    # -----------------------------
    # Math helpers
    # -----------------------------
    @staticmethod
    def yaw_from_quat_wxyz(q_wxyz):
        # VehicleOdometry.q is [w, x, y, z]
        w, x, y, z = q_wxyz
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(t3, t4)

    def prepare_vel_world(self, vel_body):
        """
        Convert body-frame command (vx,vy,vz,dyaw) to world frame using current yaw,
        and apply scaling factors.
        """
        vx_b, vy_b, vz_b, dyaw = vel_body

        # rotate body xy into world xy by yaw
        vx_w = self.vel_l_xy * (vx_b * math.cos(self.yaw_drone) - vy_b * math.sin(self.yaw_drone))
        vy_w = self.vel_l_xy * (vx_b * math.sin(self.yaw_drone) + vy_b * math.cos(self.yaw_drone))

        # z scaling
        vz = self.vel_k_z * vz_b

        # yaw rate scaling (we keep yaw_rate command)
        yaw_rate = self.yaw_k * dyaw

        return [vx_w, vy_w, vz, yaw_rate]

    # -----------------------------
    # PX4 publishing
    # -----------------------------
    def now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self.now_us()

        # We control velocity setpoint
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        self.offboard_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self, cmd_ned):
        """
        Publish velocity setpoint in PX4 NED (x forward, y right, z down).
        cmd_ned = [vx, vy, vz, yaw_rate]
        """
        vx, vy, vz, yaw_rate = cmd_ned

        sp = TrajectorySetpoint()
        sp.timestamp = self.now_us()

        # Velocity in NED
        sp.vx = float(vx)
        sp.vy = float(vy)

        # PX4 NED: z down. If model vz is positive-up, convert here.
        if self.px4_ned_z_down:
            sp.vz = float(-vz)
        else:
            sp.vz = float(vz)

        # Yaw: TrajectorySetpoint has yaw (absolute yaw), not yaw_rate.
        # If you only have yaw_rate, simplest is to not command yaw directly:
        sp.yaw = float("nan")

        self.traj_sp_pub.publish(sp)

        # NOTE: If you really want yaw control, you can integrate yaw_rate to an absolute yaw
        # and publish sp.yaw. Keeping NaN is common to let PX4 hold yaw.

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.now_us()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = int(command)

        # These IDs are usually 1 for single vehicle in sim; on real system it can differ.
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        self.vehicle_cmd_pub.publish(msg)

    def arm(self):
        self.publish_vehicle_command(VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.armed_sent = True
        self.get_logger().info("ARM command sent.")

    def set_offboard_mode(self):
        # Typical PX4: VEHICLE_CMD_DO_SET_MODE with param1 = 1 (custom), param2 = main mode
        # Some stacks use different mapping; if it doesn't switch, we will adjust.
        self.publish_vehicle_command(VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=float(PX4_CUSTOM_MAIN_MODE_OFFBOARD))
        self.offboard_sent = True
        self.get_logger().info("OFFBOARD command sent.")

    # -----------------------------
    # Callbacks
    # -----------------------------
    def odom_callback(self, msg: VehicleOdometry):
        try:
            self.yaw_drone = self.yaw_from_quat_wxyz(msg.q)
        except Exception as e:
            self.get_logger().warn(f"Yaw parse failed: {e}")

    def image_callback(self, msg: Image):
        """
        Low-frequency inference trigger:
        - Every c_rate frames, publish /vla_image and run inference once
        - Update last_cmd_ned, which will be streamed at high frequency in control_loop
        """
        self.counter += 1
        if self.counter >= self.c_rate:
            self.counter = 0
            self.allow_inference = True

        if not self.allow_inference:
            return

        # Republish sampled image for debug/bag
        self.vla_image_pub.publish(msg)

        # Convert to cv2 frame
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.frame = np.asanyarray(frame)
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            self.allow_inference = False
            return

        # Inference via local server
        try:
            result = self.send_image_to_server(self.frame, self.instruction)
            action_body = self.parse_action(result)  # [vx,vy,vz,dyaw]
            if action_body is not None:
                self.last_action_body = action_body
                self.last_cmd_ned = self.prepare_vel_world(action_body)

                vx, vy, vz, dyaw = action_body
                self.get_logger().info(
                    f"Action(body): vx={vx:.3f} vy={vy:.3f} vz={vz:.3f} dyaw={dyaw:.3f} | "
                    f"Cmd(world): vx={self.last_cmd_ned[0]:.3f} vy={self.last_cmd_ned[1]:.3f} vz={self.last_cmd_ned[2]:.3f}"
                )
        except Exception as e:
            self.get_logger().warn(f"Inference failed: {e}")

        self.allow_inference = False

    # -----------------------------
    # Server I/O (Flask)
    # -----------------------------
    def send_image_to_server(self, frame_bgr, instruction: str):
        ok, image_encoded = cv2.imencode(".jpg", frame_bgr)
        if not ok:
            raise RuntimeError("cv2.imencode failed")

        t0 = time.time()
        resp = requests.post(
            self.server_url,
            files={"image": ("image.jpg", image_encoded.tobytes(), "image/jpeg")},
            data={"instruction": instruction},
            timeout=5.0,
        )
        dt = time.time() - t0

        if resp.status_code != 200:
            raise RuntimeError(f"Server error {resp.status_code}: {resp.text}")

        self.get_logger().info(f"Server RTT: {dt:.2f}s")
        return resp.json()

    @staticmethod
    def parse_action(result_json):
        """
        Expect:
        {
          "velocities": {"x":..., "y":..., "z":...},
          "delta_yaw": ...
        }
        """
        v = result_json.get("velocities", {})
        vx = v.get("x", None)
        vy = v.get("y", None)
        vz = v.get("z", None)
        dyaw = result_json.get("delta_yaw", None)

        if None in (vx, vy, vz, dyaw):
            return None
        return [float(vx), float(vy), float(vz), float(dyaw)]

    # -----------------------------
    # High-rate control streaming
    # -----------------------------
    def control_loop(self):
        """
        This runs at control_rate_hz (default 20Hz).
        It continuously streams OffboardControlMode + TrajectorySetpoint (required by PX4),
        and after prestream_ticks sends ARM+OFFBOARD commands once.
        """
        self.publish_offboard_mode()
        self.publish_trajectory_setpoint(self.last_cmd_ned)

        self.control_tick += 1

        # Pre-stream setpoints first, then send arm/offboard
        if self.control_tick < self.prestream_ticks:
            return

        if not self.offboard_sent:
            self.set_offboard_mode()

        if not self.armed_sent:
            self.arm()


def main():
    rclpy.init()
    node = VLAOffboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
