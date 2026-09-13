#!/usr/bin/env python3
"""Dual-arm MTC forwarding with a prior-baseline extension."""

from __future__ import annotations

import argparse
import json
import os
import pickle
import socket
import time
from typing import Any

import numpy as np
import rclpy
from moveit_task_constructor_msgs.msg import SubTrajectory
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import String


def default_workspace_dir() -> str:
    return os.environ.get("MTC_WORKSPACE_DIR", os.path.expanduser("~/ws_humble"))


class PriorSubtrajectorySubscriber(Node):
    def __init__(self) -> None:
        super().__init__("prior_subtrajectory_subscriber")
        self.declare_parameter("trajectory_topic", "/mtc_sub_trajectory")
        self.declare_parameter("prior_trajectory_topic", "/prior_transition/follower_subtrajectory")
        self.declare_parameter("ready_topic", "/prior_transition/trajectory_ready")
        self.declare_parameter("central_plan_ip", "10.157.175.222")
        self.declare_parameter("left_mios_ip", "10.157.174.87")
        self.declare_parameter("left_mios_traj_port", 12345)
        self.declare_parameter("left_mios_tcp_traj_port", 12345)
        self.declare_parameter("right_mios_ip", "10.157.174.97")
        self.declare_parameter("right_mios_traj_port", 12345)
        self.declare_parameter("right_mios_tcp_traj_port", 12345)
        self.declare_parameter("workspace_dir", default_workspace_dir())
        self.declare_parameter("send_to_robot", True)
        self.declare_parameter("socket_delay_s", 0.15)

        self.trajectory_topic = str(self.get_parameter("trajectory_topic").value)
        self.prior_trajectory_topic = str(self.get_parameter("prior_trajectory_topic").value)
        self.ready_topic = str(self.get_parameter("ready_topic").value)
        self.central_plan_ip = str(self.get_parameter("central_plan_ip").value)
        self.left_mios_ip = str(self.get_parameter("left_mios_ip").value)
        self.left_mios_traj_port = int(self.get_parameter("left_mios_traj_port").value)
        self.left_mios_tcp_traj_port = int(self.get_parameter("left_mios_tcp_traj_port").value)
        self.right_mios_ip = str(self.get_parameter("right_mios_ip").value)
        self.right_mios_traj_port = int(self.get_parameter("right_mios_traj_port").value)
        self.right_mios_tcp_traj_port = int(self.get_parameter("right_mios_tcp_traj_port").value)
        self.workspace_dir = str(self.get_parameter("workspace_dir").value)
        self.send_to_robot = bool(self.get_parameter("send_to_robot").value)
        self.socket_delay_s = float(self.get_parameter("socket_delay_s").value)

        self.task_id = "0"
        self.follower_output_dir = os.path.join(self.workspace_dir, "trajectories_follower")
        self.leader_output_dir = os.path.join(self.workspace_dir, "trajectories_leader")
        self.dual_subscription = self.create_subscription(
            SubTrajectory, self.trajectory_topic, self.subtraj_callback, 10
        )
        self.prior_subscription = self.create_subscription(
            SubTrajectory, self.prior_trajectory_topic, self.prior_subtraj_callback, 10
        )
        qos_profile = QoSProfile(depth=1)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.task_id_sub = self.create_subscription(
            String, "/mtc_task_id", self.task_id_callback, qos_profile
        )
        self.ready_publisher = self.create_publisher(String, self.ready_topic, 10)
        self.get_logger().info(
            f"Dual topic: {self.trajectory_topic}; prior topic: {self.prior_trajectory_topic}; "
            f"follower={self.left_mios_ip}:{self.left_mios_traj_port}"
        )

    def task_id_callback(self, message: String) -> None:
        self.task_id = message.data
        self.get_logger().info(f"[TASK ID] Updated to: {self.task_id}")

    def write_trajectory_udp(
        self, path: str, host: str, port: int, traj_pos: np.ndarray,
        traj_vel: np.ndarray, traj_time: np.ndarray, stage_id: int,
        traj_id: int, task_id: str | None = None,
    ) -> str:
        os.makedirs(path, exist_ok=True)
        task_name = self.task_id if task_id is None else task_id
        filename = f"real_world_task_{task_name}_stage_{stage_id}_traj_{traj_id}.txt"
        file_path = os.path.join(path, filename)
        trajectory_data = {
            "time": traj_time.tolist(),
            "positions": traj_pos.tolist(),
            "velocities": traj_vel.tolist(),
        }
        if host == "local":
            with open(file_path, "w", encoding="utf-8") as output:
                for timestamp, positions, velocities in zip(traj_time, traj_pos, traj_vel):
                    output.write(
                        f"{timestamp} {' '.join(map(str, positions))} "
                        f"{' '.join(map(str, velocities))}\n"
                    )
            return filename
        with socket.create_connection((host, port), timeout=10.0) as client:
            client.sendall(b"write_moveit")
            time.sleep(self.socket_delay_s)
            client.sendall(filename.encode("utf-8"))
            time.sleep(self.socket_delay_s)
            client.sendall(pickle.dumps(trajectory_data))
        self.get_logger().info(f"Sent {filename} to {host}:{port}")
        return filename

    def write_tcp_trajectory_udp(self, path: str, host: str, port: int, stage_id: int, traj_id: int) -> None:
        if host == "local":
            self.get_logger().info("Local host specified, skipping TCP trajectory send.")
            return
        filename = f"clip{self.task_id}_stage_{stage_id}_tcp_trajectory_{traj_id}.txt"
        file_path = os.path.join(path, filename)
        if not os.path.exists(file_path):
            self.get_logger().warning(f"TCP trajectory file not found: {file_path}")
            return
        times: list[float] = []
        transforms: list[list[list[float]]] = []
        with open(file_path, "r", encoding="utf-8") as source:
            for line in source:
                parts = line.strip().split()
                if len(parts) != 17:
                    continue
                times.append(float(parts[0]))
                transforms.append(np.asarray(list(map(float, parts[1:]))).reshape((4, 4), order="F").tolist())
        with socket.create_connection((host, port), timeout=10.0) as client:
            client.sendall(b"write_tcp")
            time.sleep(self.socket_delay_s)
            client.sendall(filename.encode("utf-8"))
            time.sleep(self.socket_delay_s)
            client.sendall(pickle.dumps({"time": times, "transforms": transforms}))

    @staticmethod
    def _arrays(message: SubTrajectory, indices: list[int]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        times: list[float] = []
        positions: list[list[float]] = []
        velocities: list[list[float]] = []
        for point in message.trajectory.joint_trajectory.points:
            times.append(point.time_from_start.sec + point.time_from_start.nanosec / 1e9)
            positions.append([point.positions[i] for i in indices])
            velocities.append([point.velocities[i] if i < len(point.velocities) else 0.0 for i in indices])
        return np.asarray(positions), np.asarray(velocities), np.asarray(times)

    def subtraj_callback(self, message: SubTrajectory) -> None:
        """Original dual-arm forwarding behavior."""
        names = message.trajectory.joint_trajectory.joint_names
        if not names or not message.trajectory.joint_trajectory.points:
            self.get_logger().warning(f"Ignoring empty dual-arm trajectory {message.info.id}")
            return
        left_indices = [i for i, name in enumerate(names) if "left" in name and "finger" not in name]
        right_indices = [i for i, name in enumerate(names) if "right" in name and "finger" not in name]
        if left_indices:
            positions, velocities, times = self._arrays(message, left_indices)
            self.write_trajectory_udp(
                self.follower_output_dir, self.left_mios_ip, self.left_mios_traj_port,
                positions, velocities, times, message.info.stage_id, message.info.id,
            )
            self.write_tcp_trajectory_udp(
                self.follower_output_dir, self.left_mios_ip, self.left_mios_tcp_traj_port,
                message.info.stage_id, message.info.id,
            )
        if right_indices:
            positions, velocities, times = self._arrays(message, right_indices)
            self.write_trajectory_udp(
                self.leader_output_dir, self.right_mios_ip, self.right_mios_traj_port,
                positions, velocities, times, message.info.stage_id, message.info.id,
            )
            self.write_tcp_trajectory_udp(
                self.leader_output_dir, self.right_mios_ip, self.right_mios_tcp_traj_port,
                message.info.stage_id, message.info.id,
            )

    def _publish_ready(self, data: dict[str, Any]) -> None:
        ready = String()
        ready.data = json.dumps(data, sort_keys=True)
        self.ready_publisher.publish(ready)

    def prior_subtraj_callback(self, message: SubTrajectory) -> None:
        """Prior-specific follower-only forwarding and ready confirmation."""
        names = message.trajectory.joint_trajectory.joint_names
        task_id = message.info.planner_id or self.task_id or "prior_transition"
        trajectory_id = int(message.info.id)
        filename = f"real_world_task_{task_id}_stage_{message.info.stage_id}_traj_{trajectory_id}.txt"
        try:
            if not names or not message.trajectory.joint_trajectory.points:
                raise ValueError("empty follower trajectory")
            if any("right" in name or "finger" in name for name in names):
                raise ValueError(f"prior trajectory contains non-follower joints: {names}")
            if any("left_panda_joint" not in name for name in names) or len(names) != 7:
                raise ValueError(f"expected seven left follower joints, got {names}")
            positions, velocities, times = self._arrays(message, list(range(len(names))))
            host = self.left_mios_ip if self.send_to_robot else "local"
            stored_filename = self.write_trajectory_udp(
                self.follower_output_dir, host, self.left_mios_traj_port,
                positions, velocities, times, message.info.stage_id, trajectory_id, task_id=task_id,
            )
            self._publish_ready({
                "task_id": task_id, "ok": True, "stage": "trajectory_ready",
                "stage_id": int(message.info.stage_id), "trajectory_id": trajectory_id,
                "filename": stored_filename, "sent_to_robot": self.send_to_robot,
                "waypoint_count": len(times),
            })
        except Exception as error:
            self.get_logger().error(f"Could not forward prior trajectory: {error}")
            self._publish_ready({
                "task_id": task_id, "ok": False, "stage": "trajectory_ready",
                "stage_id": int(message.info.stage_id), "trajectory_id": trajectory_id,
                "filename": filename, "sent_to_robot": False, "error": str(error),
            })


def main(args: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Dual-arm and prior SubTrajectory subscriber")
    parser.parse_known_args(args)
    rclpy.init(args=args)
    node = PriorSubtrajectorySubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
