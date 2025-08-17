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

def compute_and_plot_distance_one_by_one(A_plan, B_plan, plan_time, A_pos_all, B_pos_all, desired_pose_distance=0.12, use_interpolation=True, remove_y_rotation=False):
    # desired_pose_distance = A_pos - B_pos
    pts_all = np.vstack([A_pos_all, B_pos_all])
    xyz_min, xyz_max = pts_all.min(axis=0), pts_all.max(axis=0)
    center = (xyz_min + xyz_max) / 2.0
    max_range = (xyz_max - xyz_min).max()
    r = max_range / 2.0
    pad = 0.02 * max_range

    # ---- set up figure once ----
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(A_pos_all[:,0], A_pos_all[:,1], A_pos_all[:,2], 'o-', label='A')
    resampled_line, = ax.plot([], [], [], '^-', label='B (resampled)')
    arrow_len = 0.05 * max_range

    # fixed bounds BEFORE adding any collections
    ax.set_xlim(center[0]-r-pad, center[0]+r+pad)
    ax.set_ylim(center[1]-r-pad, center[1]+r+pad)
    ax.set_zlim(center[2]-r-pad, center[2]+r+pad)
    ax.set_box_aspect((1,1,1))
    ax.set_xlabel('X [m]'); ax.set_ylabel('Y [m]'); ax.set_zlabel('Z [m]')
    ax.set_title('Live pairing: A ↔ B (resampled)')
    ax.legend()
    plt.tight_layout()

    # ---- incremental pairing ----
    T_B_resampled = []
    segments = []          # list of (2,3) arrays
    resampled_pts = []     # Nx3, for the live B line
    j_start = 0
    lc = None              # create collection lazily

    # === ARROWS: tiny x-axis segments for leader & follower ===
    A_x_segments = []      # list of (2,3)
    B_x_segments = []    # list of (2,3)
    lc_A_x = None
    lc_B_x = None

    for i in range(A_plan.getWaypointCount()):
        A_T = A_plan.getWaypoint(i)     # 4x4
        A_p = A_T[:3, 3]

        # search best j from j_start (your logic)
        best_position_err = float('inf')
        best_j = j_start
        position_error_tolerance = 0.001
        orientation_error_tolerance = 1e-4
        best_alignment_candidates = []
        for j in range(j_start, B_plan.getWaypointCount()):
            B_T = B_plan.getWaypoint(j)
            B_p = B_T[:3, 3]

            # desired_B_T has a offset in x direction of A's frame with no rotation about y-axis
            T_offset = np.eye(4)
            T_offset[:3, 3] = [desired_pose_distance, 0, 0]  # offset in x direction of A's frame
            T_offset[:3, :3] = Rotation.from_euler('xyz', [0, 0, 0]).as_matrix()  # no rotation7
            if remove_y_rotation:
                reference_x = A_T[:3, :3] @ np.array([1.0, 0.0, 0.0])  # +X in A's frame
                # remove rotation about y-axis
                A_R_without_y = remove_rotation_about_local_axis(
                                R_end_effector= A_T[:3, :3],
                                local_axis='y',
                                reference_world_vector=reference_x,
                                reference_local_axis='x',
                                use_unsigned=False)
                A_T_without_y_rotation = A_T.copy()
                A_T_without_y_rotation[:3, :3] = A_R_without_y
                desired_B_T = A_T_without_y_rotation @ T_offset
            else:
                desired_B_T = A_T @ T_offset
            desired_B_p = desired_B_T[:3, 3]

            dp = np.linalg.norm(B_p - desired_B_p)
            position_err = abs(dp)
            print(f"i={i}, j={j}, position_err={position_err:.6f}")
            if position_err < best_position_err:
                best_position_err, best_position_j = position_err, j
            if position_err < position_error_tolerance:
               best_alignment_candidates.append((j, B_T))
        
        if not best_alignment_candidates:
            # if no candidates satisfy the tolerance, use the best found
            print(f"Warning: No position alignment candidates found for i={i}")
            if use_interpolation:
                print(f"Using interpolation for alignment.")

                aligned_B_T, best_j, blend, _ = B_plan.getStateAtPositionDistance(desired_B_p, minimum_index=j_start)
                aligned_B_p = aligned_B_T[:3, 3]
                print(f"Interpolated between (best j={best_j} and {best_j + 1}) using blend {blend} with position error {best_position_err:.6f}")
            else:
                print(f"Using best j={best_position_j} with position error {best_position_err:.6f}")
                # print(f"Minimum position difference found at j={best_position_j} with value {best_position_err:.6f}")
                aligned_B_T = B_plan.getWaypoint(best_position_j)
                aligned_B_p = aligned_B_T[:3, 3]
                best_j = best_position_j
        else:
            best_orientation_err = float('inf')
            best_orientation_candidates = []
            # compare angular distances and choose the best one
            for j, B_T in best_alignment_candidates:
                dtheta = compute_rotation_distance(A_T[:3, :3], B_T[:3, :3], ignore_axis='y')  # ignore rotation around z-axis
                # print(f"Comparing j={j}, angular distance={dtheta:.6f}")
                if dtheta - best_orientation_err < 0:
                    best_orientation_err = dtheta # update best
                    best_orientation_j = j
                    best_orientation_candidates = [(j, B_T)] # reset candidates
                elif dtheta - best_orientation_err < orientation_error_tolerance:
                    # if the difference is very small, consider it a candidate
                    best_orientation_candidates.append((j, B_T))

            if not best_orientation_candidates:
                print(f"Warning: No orientation alignment candidates found for i={i}")
                # if no candidates, use the best found
                aligned_B_T = B_plan.getWaypoint(best_orientation_j)
                aligned_B_p = aligned_B_T[:3, 3]
                best_j = best_orientation_j
                print(f"Using best j={best_j} with orientation error {best_orientation_err:.6f}")
            else:
                # if multiple candidates, choose the one with the largest index
                best_j, aligned_B_T = max(best_orientation_candidates, key=lambda x: x[0])
                aligned_B_p = aligned_B_T[:3, 3]
                print(f"Best alignment for i={i} found at j={best_j} with position error {best_position_err:.6f} and orientation error {best_orientation_err:.6f}")
        # update B path
        T_B_resampled.append(aligned_B_T)
        resampled_pts.append(aligned_B_p)
        pts_arr = np.asarray(resampled_pts)
        resampled_line.set_data(pts_arr[:,0], pts_arr[:,1])
        resampled_line.set_3d_properties(pts_arr[:,2])

        # add or update the connection collection
        seg = np.stack([A_p, aligned_B_p], axis=0)  # (2,3)
        segments.append(seg)
        if lc is None:
            lc = Line3DCollection(segments, linewidths=0.8, alpha=0.7, color='green')
            ax.add_collection3d(lc)           # now it's non-empty → no autoscale error
        else:
            lc.set_segments(segments)

        # === ARROWS: append tiny x-axis segments and update collections ===
        # A +X arrow
        A_x = A_T[:3, 0]  # column 0 is x-axis
        A_x_seg = np.stack([A_p, A_p + arrow_len * A_x / (np.linalg.norm(A_x) + 1e-12)], axis=0)
        A_x_segments.append(A_x_seg)
        if lc_A_x is None:
            lc_A_x = Line3DCollection(A_x_segments, linewidths=1.2, alpha=0.9, color='blue')
            ax.add_collection3d(lc_A_x)
        else:
            lc_A_x.set_segments(A_x_segments)

        # B +X arrow
        B_x = aligned_B_T[:3, 0]
        B_x_seg = np.stack([aligned_B_p, aligned_B_p + arrow_len * B_x / (np.linalg.norm(B_x) + 1e-12)], axis=0)
        B_x_segments.append(B_x_seg)
        if lc_B_x is None:
            lc_B_x = Line3DCollection(B_x_segments, linewidths=1.2, alpha=0.9, color='orange')
            ax.add_collection3d(lc_B_x)
        else:
            lc_B_x.set_segments(B_x_segments)

        # plt.pause(0.001)

        # advance starting j; use +1 for strict monotonicity if you prefer
        j_start = best_j

    T_B_resampled = np.array(T_B_resampled)
    t_s_resampled = plan_time

    plt.show()
    return T_B_resampled, t_s_resampled


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
        eps = 1e-12

        best_i, best_u = None, None
        best_d2 = float("inf")

        for j in range(minimum_index, self.waypoints.shape[0] - 1):
            segment_start_position = self.waypoints[j, :3, 3]
            segment_end_position   = self.waypoints[j + 1, :3, 3]
            segment_vector = segment_end_position - segment_start_position
            segment_length_squared = float(np.dot(segment_vector, segment_vector))
            if segment_length_squared <= eps:                 # zero-length segment: treat as point
                projection_blend = 0.0
                projected_point_on_segment = segment_start_position
            else:
                projection_blend = float(np.dot(desired_position - segment_start_position, segment_vector) / segment_length_squared)
                if projection_blend < 0.0 or projection_blend > 1.0:
                    continue  # outside segment, skip
                projected_point_on_segment = segment_start_position + projection_blend * segment_vector
    
            d2 = float(np.sum((desired_position - projected_point_on_segment) ** 2))
            print(f"interpolation at j={j}, d2={d2:.6f}, blend={projection_blend:.6f}")
            if d2 < best_d2:
                best_d2, best_i, best_u = d2, j, projection_blend
        if best_i is None or best_u is None:
            raise ValueError("No valid segment found for the desired position.")
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
    
    def getStateAtPositionDistance(self, desired_position, minimum_index):
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

        return output_state, before, blend, after

# =========================
# Pipeline
# =========================
def main():
    dir = '/home/tp2/ws_humble/MTC_connect_visualization/'
    prefix = "success" # no_matching or success
    master_traj_file = dir + 'leader_' + prefix + '_tcp_002.txt'
    slave_traj_file  = dir + 'follower_' + prefix + '_tcp_002.txt'
    master_path_file = dir + 'leader_tcp_path_' + prefix + '_002.txt'

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

    # # convert 7 dim (x, y, z, qx, qy, qz, qw) master path to 4x4 poses
    slave_resampled_traj, slave_resampled_time = compute_and_plot_distance_one_by_one(
        master_ref_traj, slave_plan_traj,
        slave_plan_traj.durations,
        master_ref_traj.getWaypointPositions(), 
        slave_plan_traj.getWaypointPositions(),
        desired_pose_distance=-0.12,
        use_interpolation=False,
        remove_y_rotation=False  # remove rotation about y-axis
    )

    # # test remove rotation about y-axis
    # T_no_y_rotation = np.copy(T_slave)

    # for i, waypoint in enumerate(T_no_y_rotation):
    #     # master_ref = master_ref_traj.getWaypoint(i)
    #     # reference_x = master_ref[:3, :3] @ np.array([1.0, 0.0, 0.0])  # +X in master frame
    #     reference_x = waypoint[:3, :3] @ np.array([1.0, 0.0, 0.0])  # +X in A's frame

    #     R = waypoint[:3, :3]
    #     R_without_y = remove_rotation_about_local_axis(
    #         R_end_effector=R,
    #         local_axis='y',
    #         reference_world_vector=reference_x,
    #         reference_local_axis='x',
    #         use_unsigned=False
    #     )
    #     waypoint[:3, :3] = R_without_y
    # slave_no_y_rotation_traj = RobotCartesianTrajectory(t_s, T_no_y_rotation)

    # slave_resampled_traj, slave_resampled_time = compute_and_plot_distance_one_by_one(
    #     master_ref_traj, slave_no_y_rotation_traj,
    #     slave_no_y_rotation_traj.durations,
    #     master_ref_traj.getWaypointPositions(), 
    #     slave_no_y_rotation_traj.getWaypointPositions(),
    #     desired_pose_distance=-0.12,
    #     use_interpolation=True
    # )

    # 3) Check interpolation for master itself
    master_resampled_traj, master_resampled_time = compute_and_plot_distance_one_by_one(
        master_ref_traj, master_plan_traj, 
        master_ref_traj.durations,
        master_ref_traj.getWaypointPositions(), 
        master_plan_traj.getWaypointPositions(),
        desired_pose_distance=0.0,
        use_interpolation=True,
        remove_y_rotation=False
    )

    # 4) Check distance between master path and slave trajectory
    master_resampled_traj, master_resampled_time = compute_and_plot_distance_one_by_one(
        slave_plan_traj, master_plan_traj, 
        slave_plan_traj.durations,
        slave_plan_traj.getWaypointPositions(), 
        master_plan_traj.getWaypointPositions(),
        desired_pose_distance=0.12,
        use_interpolation=True,
        remove_y_rotation=True
    )

    # 3) Check distance between master trajectory and slave trajectory
    # compute_and_plot_distance_one_by_one(
    #     master_plan_traj, slave_plan_traj,
    #     master_plan_traj.durations,
    #     master_plan_traj.getWaypointPositions(), 
    #     slave_plan_traj.getWaypointPositions(),
    #     desired_pose_distance=-0.12
    # )

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
