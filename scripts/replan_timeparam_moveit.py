#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import numpy as np
import pandas as pd
from replan_timeparam_utils import *
from replan_display import plot_3d_trajectories
from scipy.spatial.transform import Rotation, Slerp 
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Line3DCollection

def compute_rotation_distance(R_a, R_b, ignore_axis=None):
    """
    Geodesic angle between Ra and Rb (radians).
    If ignore_axis is given ('x'|'y'|'z' or a 3-vector), we remove the spin
    about that axis (axis expressed in Ra's local frame) before measuring.
    """
    R_rel = R_a.T @ R_b

    if ignore_axis is None:
        return Rotation.from_matrix(R_rel).magnitude()

    # relative rotation vector ω in A-frame
    w = Rotation.from_matrix(R_rel).as_rotvec()

    if isinstance(ignore_axis, str):
        axis_map = {'x': np.array([1.0, 0.0, 0.0]),
                    'y': np.array([0.0, 1.0, 0.0]),
                    'z': np.array([0.0, 0.0, 1.0])}
        v = axis_map[ignore_axis.lower()]
    else:
        v = np.asarray(ignore_axis, dtype=float)  # already in Ra frame

    n = np.linalg.norm(v)
    if n < 1e-12:
        return np.linalg.norm(w)
    v /= n

    # remove component about v (spin around the chosen Ra-axis)
    w_perp = w - v * (v @ w)
    return np.linalg.norm(w_perp)

def compute_pose_distance(p_a, R_a, p_b, R_b, rotation_weight):
    dp = np.linalg.norm(p_a - p_b)
    dth = compute_rotation_distance(R_a, R_b)
    return dp, dth, dp*dp + (rotation_weight * dth)**2

def desired_pose(T_ref, T_offset=np.eye(4)):
    """
    Create a desired pose based on a reference pose with position and orientation offsets.
    Offsets are in T_ref's frame.
    """
    T_desired = np.copy(T_ref)
    T_desired = T_desired @ T_offset  # Apply the offset in the reference frame
    return T_desired

def compute_and_plot_distance_one_by_one(master_plan, slave_plan, plan_time, master_pos_all, slave_pos_all, desired_pose_distance=0.12, use_interpolation=True):
    pts_all = np.vstack([master_pos_all, slave_pos_all])
    xyz_min, xyz_max = pts_all.min(axis=0), pts_all.max(axis=0)
    center = (xyz_min + xyz_max) / 2.0
    max_range = (xyz_max - xyz_min).max()
    r = max_range / 2.0
    pad = 0.02 * max_range

    # ---- set up figure once ----
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(master_pos_all[:,0], master_pos_all[:,1], master_pos_all[:,2], 'o-', label='Master')
    resampled_line, = ax.plot([], [], [], '^-', label='Slave (resampled)')
    arrow_len = 0.05 * max_range

    # fixed bounds BEFORE adding any collections
    ax.set_xlim(center[0]-r-pad, center[0]+r+pad)
    ax.set_ylim(center[1]-r-pad, center[1]+r+pad)
    ax.set_zlim(center[2]-r-pad, center[2]+r+pad)
    ax.set_box_aspect((1,1,1))
    ax.set_xlabel('X [m]'); ax.set_ylabel('Y [m]'); ax.set_zlabel('Z [m]')
    ax.set_title('Live pairing: Master ↔ Slave (resampled)')
    ax.legend()
    plt.tight_layout()

    # ---- incremental pairing ----
    T_slave_resampled = []
    segments = []          # list of (2,3) arrays
    resampled_pts = []     # Nx3, for the live slave line
    j_start = 0
    lc = None              # create collection lazily

    # === ARROWS: tiny x-axis segments for leader & follower ===
    leader_x_segments = []      # list of (2,3)
    follower_x_segments = []    # list of (2,3)
    lc_leader_x = None
    lc_follower_x = None

    for i in range(master_plan.getWaypointCount()):
        leader_T = master_plan.getWaypoint(i)     # 4x4
        leader_p = leader_T[:3, 3]

        # search best j from j_start (your logic)
        best_position_err = float('inf')
        best_j = j_start
        position_error_tolerance = 0.001
        orientation_error_tolerance = 1e-4
        best_alignment_candidates = []
        for j in range(j_start, slave_plan.getWaypointCount()):
            follower_T = slave_plan.getWaypoint(j)
            dp = np.linalg.norm(follower_T[:3, 3] - leader_p)
            position_err = abs(dp - desired_pose_distance)
            # print(f"i={i}, j={j}, position_err={position_err:.6f}")
            if position_err < best_position_err:
                best_position_err, best_position_j = position_err, j
            if position_err < position_error_tolerance:
               best_alignment_candidates.append((j, follower_T))
        
        if not best_alignment_candidates:
            # if no candidates satisfy the tolerance, use the best found
            print(f"Warning: No position alignment candidates found for i={i}")
            if use_interpolation:
                print(f"Using interpolation for alignment.")
                T_offset = np.eye(4)
                T_offset[:3, 3] = [-desired_pose_distance, 0, 0]  # offset in x direction of leader's frame
                T_offset[:3, :3] = Rotation.from_euler('xyz', [0, 0, 0]).as_matrix()  # no rotation
                desired_follower_T = desired_pose(leader_T, T_offset=T_offset)

                aligned_follower_T, best_j = slave_plan.getStateAtPositionDistance(desired_follower_T[:3, 3], minimum_index=j_start)
                aligned_follower_p = aligned_follower_T[:3, 3]
            else:
                print(f"Using best j={best_position_j} with position error {best_position_err:.6f}")
                # print(f"Minimum position difference found at j={best_position_j} with value {best_position_err:.6f}")
                aligned_follower_T = slave_plan.getWaypoint(best_position_j)
                aligned_follower_p = aligned_follower_T[:3, 3]
                best_j = best_position_j
        else:
            best_orientation_err = float('inf')
            best_orientation_candidates = []
            # compare angular distances and choose the best one
            for j, follower_T in best_alignment_candidates:
                dtheta = compute_rotation_distance(leader_T[:3, :3], follower_T[:3, :3], ignore_axis='y')  # ignore rotation around z-axis
                # print(f"Comparing j={j}, angular distance={dtheta:.6f}")
                if dtheta - best_orientation_err < 0:
                    best_orientation_err = dtheta # update best
                    best_orientation_j = j
                    best_orientation_candidates = [(j, follower_T)] # reset candidates
                elif dtheta - best_orientation_err < orientation_error_tolerance:
                    # if the difference is very small, consider it a candidate
                    best_orientation_candidates.append((j, follower_T))

            if not best_orientation_candidates:
                print(f"Warning: No orientation alignment candidates found for i={i}")
                # if no candidates, use the best found
                aligned_follower_T = slave_plan.getWaypoint(best_orientation_j)
                aligned_follower_p = aligned_follower_T[:3, 3]
                best_j = best_orientation_j
                print(f"Using best j={best_j} with orientation error {best_orientation_err:.6f}")
            else:
                # if multiple candidates, choose the one with the largest index
                best_j, aligned_follower_T = max(best_orientation_candidates, key=lambda x: x[0])
                aligned_follower_p = aligned_follower_T[:3, 3]
                print(f"Best alignment for i={i} found at j={best_j} with position error {best_position_err:.6f} and orientation error {best_orientation_err:.6f}")
        # update slave path
        T_slave_resampled.append(aligned_follower_T)
        resampled_pts.append(aligned_follower_p)
        pts_arr = np.asarray(resampled_pts)
        resampled_line.set_data(pts_arr[:,0], pts_arr[:,1])
        resampled_line.set_3d_properties(pts_arr[:,2])

        # add or update the connection collection
        seg = np.stack([leader_p, aligned_follower_p], axis=0)  # (2,3)
        segments.append(seg)
        if lc is None:
            lc = Line3DCollection(segments, linewidths=0.8, alpha=0.7, color='green')
            ax.add_collection3d(lc)           # now it's non-empty → no autoscale error
        else:
            lc.set_segments(segments)

        # === ARROWS: append tiny x-axis segments and update collections ===
        # leader +X arrow
        leader_x = leader_T[:3, 0]  # column 0 is x-axis
        leader_x_seg = np.stack([leader_p, leader_p + arrow_len * leader_x / (np.linalg.norm(leader_x) + 1e-12)], axis=0)
        leader_x_segments.append(leader_x_seg)
        if lc_leader_x is None:
            lc_leader_x = Line3DCollection(leader_x_segments, linewidths=1.2, alpha=0.9, color='blue')
            ax.add_collection3d(lc_leader_x)
        else:
            lc_leader_x.set_segments(leader_x_segments)

        # follower +X arrow
        follower_x = aligned_follower_T[:3, 0]
        follower_x_seg = np.stack([aligned_follower_p, aligned_follower_p + arrow_len * follower_x / (np.linalg.norm(follower_x) + 1e-12)], axis=0)
        follower_x_segments.append(follower_x_seg)
        if lc_follower_x is None:
            lc_follower_x = Line3DCollection(follower_x_segments, linewidths=1.2, alpha=0.9, color='orange')
            ax.add_collection3d(lc_follower_x)
        else:
            lc_follower_x.set_segments(follower_x_segments)

        # plt.pause(0.001)

        # advance starting j; use +1 for strict monotonicity if you prefer
        j_start = best_j

    T_slave_resampled = np.array(T_slave_resampled)
    t_s_resampled = plan_time

    plt.show()


class RobotCartesianTrajectory:
    def __init__(self,time_stamps, waypoints):
        self.waypoints = waypoints # array of shape (N, 4, 4)
        self.durations = time_stamps

        self.duration_from_previous = np.zeros_like(self.durations)
        self.duration_from_previous[1:] = np.diff(self.durations)

        self.distance_from_previous = np.zeros_like(self.duration_from_previous)
        for i in range(1, len(self.waypoints)):
            position_dist, orientation_dist, pose_dist = compute_pose_distance(
                self.waypoints[i, :3, 3], self.waypoints[i, :3, :3],
                self.waypoints[i-1, :3, 3], self.waypoints[i-1, :3, :3],
                rotation_weight=0.0
            )

            if pose_dist > 1e-6:
                self.distance_from_previous[i] = pose_dist
            else:
                self.distance_from_previous[i] = 0.0
        self.arc_distances = np.cumsum(self.distance_from_previous)
    
    def __getitem__(self, idx):
        return self.waypoints[idx], self.duration_from_previous[idx], self.distance_from_previous[idx]
    
    def interpolate(self, from_pose, to_pose, blend):
        from_position, to_position = from_pose[:3, 3], to_pose[:3, 3]
        from_rotation, to_rotation = from_pose[:3, :3], to_pose[:3, :3]

        # Simple linear interpolation for position
        interpolated_position = (1 - blend) * from_position + blend * to_position
        # Slerp for rotation
        slerp = Slerp([0.0, 1.0], Rotation.from_matrix([from_rotation, to_rotation]))
        interpolated_rotation = slerp([blend]).as_matrix()[0]

        # Construct the interpolated pose
        interpolated_pose = np.eye(4)
        interpolated_pose[:3, 3] = interpolated_position
        interpolated_pose[:3, :3] = interpolated_rotation

        return interpolated_pose
    
    def getWaypoint(self, idx):
        if idx < 0 or idx >= len(self.waypoints):
            raise IndexError("Waypoint index out of range.")
        return self.waypoints[idx]

    def getWaypointCount(self):
        return len(self.waypoints)

    def getWaypointPositions(self):
        return self.waypoints[:, :3, 3]

    def getWaypointDurationFromStart(self, idx):
        if idx >= len(self.duration_from_previous):
            idx = len(self.duration_from_previous) - 1
        return self.durations[idx] - self.durations[0]
    
    def getWaypointDistanceFromStart(self, idx):
        if idx >= len(self.distance_from_previous):
            idx = len(self.distance_from_previous) - 1
        return self.arc_distances[idx] - self.arc_distances[0]
    
    def findWayPointIndicesForDurationAfterStart(self, duration):
        """
        Find indices and blend for a given duration after start.
        Returns (before, after, blend)
        """
        if duration < 0.0:
            return 0, 0, 0.0

        index = 0
        num_points = len(self.waypoints)
        running_duration = 0.0
        while index < num_points:
            running_duration += self.duration_from_previous[index]
            if running_duration >= duration:
                break
            index += 1

        before = max(index - 1, 0)
        after = min(index, num_points - 1)

        before_time = running_duration - self.duration_from_previous[index] if index < num_points else running_duration
        blend = 1.0 if after == before else (duration - before_time) / self.duration_from_previous[index] if index < num_points and self.duration_from_previous[index] > 0 else 0.0
        return before, after, blend

    def findWayPointIndicesForArcDistanceAfterStart(self, distance):
        """
        Find indices and blend for a given arc distance after start.
        Returns (before, after, blend)
        """
        num_points = len(self.waypoints)
        if distance < 0.0:
            return 0, 0, 0.0

        total_distance = self.getWaypointDistanceFromStart(num_points - 1)
        if distance >= total_distance:
            print(f"[WARNING] Requested distance {distance} exceeds total trajectory distance {total_distance}. Clamping to end.")
            return num_points - 1, num_points - 1, 1.0 

        index = 0
        running_arcdist = 0.0
        while index < num_points:
            running_arcdist += self.distance_from_previous[index]
            if running_arcdist >= distance:
                break
            index += 1

        before = max(index - 1, 0)
        after = min(index, num_points - 1)

        before_distance = running_arcdist - self.distance_from_previous[index] if index < num_points else running_arcdist
        blend = 1.0 if after == before else (distance - before_distance) / self.distance_from_previous[index] if index < num_points and self.distance_from_previous[index] > 0 else 0.0
        return before, after, blend
    
    def findWayPointIndicesForPositionDistance(self, desired_position, minimum_index=0):
        """
        Project desired_position (3,) onto the TCP position polyline.
        Returns (before, after, blend) where the closest point is
        P[before] + blend * (P[after] - P[before]).
        """
        T = self.waypoints
        N = T.shape[0]
        if N == 0:
            raise ValueError("Trajectory has no waypoints.")
        if N == 1:
            return 0, 0, 0.0

        P = T[:, :3, 3]                   # (N,3)
        p_star = np.asarray(desired_position, dtype=float)
        eps = 1e-12

        best_i, best_u = 0, 0.0
        best_d2 = float("inf")

        for i in range(minimum_index, N - 1):
            a = P[i]
            b = P[i + 1]
            v = b - a
            vv = float(np.dot(v, v))
            if vv <= eps:                 # zero-length segment: treat as point
                u = 0.0
                q = a
            else:
                u_raw = float(np.dot(p_star - a, v) / vv)
                u = float(np.clip(u_raw, 0.0, 1.0))
                q = a + u * v

            d2 = float(np.sum((p_star - q) ** 2))
            if d2 < best_d2:
                best_d2, best_i, best_u = d2, i, u

        return best_i, best_i + 1, best_u
    
    def getStateAtDurationFromStart(self, requested_duration):
        if self.getWaypointCount() == 0:
            raise ValueError("Trajectory has no waypoints.")
        before, after, blend = self.findWayPointIndicesForDurationAfterStart(requested_duration)
        output_state = self.interpolate(
            self.waypoints[before],
            self.waypoints[after],
            blend
        )
        return output_state, after
    
    def getStateAtArcDistanceFromStart(self, requested_distance):
        if self.getWaypointCount() == 0:
            raise ValueError("Trajectory has no waypoints.")
        before, after, blend = self.findWayPointIndicesForArcDistanceAfterStart(requested_distance)
        output_state = self.interpolate(
            self.waypoints[before],
            self.waypoints[after],
            blend
        )
        return output_state, after
    
    def getStateAtPositionDistance(self, desired_position, minimum_index, return_info=False):
        """
        Pose at the closest point (by position) on the path to desired_position.
        Uses linear position + SLERP orientation between the bracketing waypoints.
        If return_info=True, also returns a dict with segment, blend, closest point, and distance.
        """
        if self.getWaypointCount() == 0:
            raise ValueError("Trajectory has no waypoints.")

        before, after, blend = self.findWayPointIndicesForPositionDistance(desired_position, minimum_index)

        output_state = self.interpolate(
            self.waypoints[before],
            self.waypoints[after],
            blend)

        # if not return_info:
        #     return output_state

        # # build a small info payload
        # p0, p1 = T0[:3, 3], T1[:3, 3]
        # closest_point = (1.0 - blend) * p0 + blend * p1
        # distance = float(np.linalg.norm(closest_point - np.asarray(desired_position, float)))

        # info = dict(segment=before, after=after, blend=blend,
        #             closest_point=closest_point, distance=distance)
        return output_state, after


# =========================
# Pipeline
# =========================
def main():
    dir = '/home/tp2/ws_humble/MTC_connect_visualization/'
    prefix = "no_matching" # no_matching or success
    master_traj_file = dir + 'leader_' + prefix + '_tcp_003.txt'
    slave_traj_file  = dir + 'follower_' + prefix + '_tcp_003.txt'
    master_path_file = dir + 'leader_tcp_path_' + prefix + '_003.txt'

    print("[*] Loading trajectories ...")
    t_m, T_master = load_traj_txt(master_traj_file) # poses are in robot base frame
    t_s, T_slave = load_traj_txt(slave_traj_file)
    T_slave_offset = np.zeros_like(T_slave)
    T_slave_offset[:, 1, 3] -= 0.562  # translate slave poses to master frame
    T_slave = T_slave + T_slave_offset  # 对 slave 进行 y 轴补偿

    path_master = load_path_txt(master_path_file)
    path_master_offset = np.zeros_like(path_master)
    path_master_offset[:, 1] -= 0.281 
    path_master_offset[:, 2] -= 1.025
    path_master += path_master_offset  # 对 master 路径进行 y 轴补偿

    path_master_poses = np.zeros((len(path_master), 4, 4))
    for i, pose in enumerate(path_master):
        path_master_poses[i, :3, 3] = pose[:3]
        path_master_poses[i, :3, :3] = Rotation.from_quat(pose[3:]).as_matrix()
        path_master_poses[i, 3, 3] = 1.0  # homogeneous coordinate

    master_plan_traj = RobotCartesianTrajectory(t_m, T_master) # 
    slave_plan_traj  = RobotCartesianTrajectory(t_s, T_slave)
    master_ref_traj = RobotCartesianTrajectory(t_s, path_master_poses)

    # 1) Pad short trajectories with last sample for visualization
    t_m_a, T_master_a, t_s_a, T_slave_a = pad_trajectories(t_m, T_master, t_s, T_slave)
    print(f"[ALIGN] master: {len(T_master_a)} samples, slave: {len(T_slave_a)} samples")
    pts = np.vstack([T_master_a[:, :3, 3], T_slave_a[:, :3, 3]])
    xyz_min, xyz_max = pts.min(axis=0), pts.max(axis=0)

    plot_3d_trajectories(T_master_a[:, :3, 3], T_slave_a[:, :3, 3], xyz_min, xyz_max, path_master_poses[:, :3, 3])

    # 2) Check distance between master path and slave trajectory
    print("[*] Checking distance between master path and slave trajectory ...")

    # convert 7 dim (x, y, z, qx, qy, qz, qw) master path to 4x4 poses
    compute_and_plot_distance_one_by_one(
        master_ref_traj, slave_plan_traj,
        slave_plan_traj.durations,
        master_ref_traj.getWaypointPositions(), 
        slave_plan_traj.getWaypointPositions(),
        desired_pose_distance=0.12,
        use_interpolation=True
    )

    # 3) Check distance between master trajectory and slave trajectory
    compute_and_plot_distance_one_by_one(
        master_plan_traj, slave_plan_traj,
        master_plan_traj.durations,
        master_plan_traj.getWaypointPositions(), 
        slave_plan_traj.getWaypointPositions(),
        desired_pose_distance=0.12
    )

    # 4) Check distance between master trajectory and slave dense trajectory
    t_s_dense, T_slave_dense = densify_trajectory(t_s, T_slave)
    slave_dense_traj  = RobotCartesianTrajectory(t_s_dense, T_slave_dense)
    compute_and_plot_distance_one_by_one(
        master_plan_traj, slave_dense_traj,
        master_plan_traj.durations,
        master_plan_traj.getWaypointPositions(), 
        slave_dense_traj.getWaypointPositions(),
        desired_pose_distance=0.12
    )

if __name__ == "__main__":
    main()
